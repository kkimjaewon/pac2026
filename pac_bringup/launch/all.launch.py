"""One Gazebo + actual A/B/C, using a single validated scenario snapshot."""
import math
import time
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_config(config, world_path):
    if config['version'] != 1 or config['robot'] not in ('franka_fr3', 'ur5e', 'ha006b'):
        raise ValueError('This team-world integration supports franka_fr3, ur5e or ha006b')
    if config['map_frame'] != 'map' or config['camera_frame'] != 'depth_camera_link':
        raise ValueError('Frame contract must match team camera/world')
    for key in ('planned_position', 'target_offset'):
        if len(config[key]) != 3 or not all(math.isfinite(v) for v in config[key]):
            raise ValueError('finite xyz required: ' + key)
    if (config['command_ttl_ns'] != 5000000 or config['ai_ttl_ns'] <= 0 or
        not 0 <= config['confidence_min'] <= 1 or not math.isfinite(config['confidence_min']) or
        not 0 < config['accept_radius_m'] <= .05 or
        not 0 < config['v_max_m_s'] <= .1 or not 0 < config['a_max_m_s2'] <= .2):
        raise ValueError('Invalid shared TTL/confidence/motion limits')
    # Keep the validated existing team world; do not create another geometry copy.
    if config['world_file'] != 'franka_world.sdf':
        raise ValueError('team_workcell uses the canonical franka_world.sdf')
    world = ET.parse(world_path).getroot().find('world')
    if world.findtext("model[@name='rgbd_camera']/link/sensor/gz_frame_id") != config['camera_frame']:
        raise ValueError('camera_frame must match the SDF sensor frame')
    plate = world.find("model[@name='steel_plate']")
    pose = [float(v) for v in plate.findtext('pose').split()]
    size = [float(v) for v in plate.findtext('link/collision/geometry/box/size').split()]
    surface = [pose[0], pose[1], pose[2] + size[2]/2]
    if any(abs(a-b) > 1e-6 for a,b in zip(surface, config['planned_position'])):
        raise ValueError('planned_position must match the plate surface in the SDF; update world/scene together')
    if any(abs(v) > 1e-9 for v in pose[3:]):
        raise ValueError('This scenario requires a horizontal plate')


def start(context):
    config = yaml.safe_load(Path(LaunchConfiguration('scenario_config').perform(context)).read_text())
    # Robot swap changes only C's robot profile; A/B settings come from the same snapshot.
    robot = LaunchConfiguration('robot').perform(context)
    if robot:
        config['robot'] = robot
    validate_config(config, Path(share('pac_worlds'))/'worlds/franka_world.sdf')
    session = int(LaunchConfiguration('session_id').perform(context))
    run_id = LaunchConfiguration('run_id').perform(context)
    if session <= 0 or not run_id or Path(run_id).name != run_id or run_id in ('.', '..'):
        raise ValueError('Positive session and a single-component run_id are required')
    log_root = Path(LaunchConfiguration('log_root').perform(context)).expanduser()
    folder = log_root / run_id
    if folder.exists():
        raise ValueError('Use a new run_id; an existing run must not be overwritten')
    folder.mkdir(parents=True)
    (folder/'scenario_config.yaml').write_text(yaml.safe_dump(config))
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('pac_bringup'))/'launch/sim.launch.py')),
        launch_arguments={
            'robot': config['robot'], 'gui': LaunchConfiguration('gui'), 'session_id': str(session),
            'run_id': run_id, 'log_directory': str(log_root),
            'executor_cpu': LaunchConfiguration('executor_cpu'),
            'executor_fifo_priority': LaunchConfiguration('executor_fifo_priority'),
            'ik_budget_clock': LaunchConfiguration('ik_budget_clock'),
        }.items())
    a = Node(package='ai_inference', executable='ai_inference_node', name='ai_inference_node', output='screen',
        parameters=[str(Path(share('ai_inference'))/'config/sensor_config.yaml'), {
            'map_frame': config['map_frame'], 'camera_frame': config['camera_frame'],
            'frame_timeout_s': config['ai_ttl_ns']/1e9,
            'confidence_min': float(config['confidence_min']), 'command_max_age_ns': config['ai_ttl_ns'],
            'detector_mode': config['detector_mode'], 'primary_impl': config['primary_impl'],
            'color_v_min': config['color_v_min'], 'log_dir': str(folder/'A'),
        }])
    b = Node(package='rt_control', executable='rt_control_node', name='rt_control_node', output='screen',
        parameters=[str(Path(share('rt_control'))/'config/fr3_integration.yaml'), {
            'session_id': session, 'run_id': run_id, 'log_root': str(log_root),
            'planned_position': config['planned_position'], 'target_offset': config['target_offset'],
            'ttl_ns': config['ai_ttl_ns'], 'confidence_min': float(config['confidence_min']),
            'accept_radius': float(config['accept_radius_m']), 'hal_ttl_ns': config['command_ttl_ns'],
            'v_max': float(config['v_max_m_s']), 'a_max': float(config['a_max_m_s2']),
        }])
    # B already waits for a fresh C READY/ACTIVE; no fixed startup sleep is needed.
    return [sim, a, b]


def generate_launch_description():
    defaults = {'scenario_config': str(Path(share('pac_bringup'))/'config/scenario_config.yaml'),
        'robot': '', 'gui': 'false', 'session_id': '42', 'run_id': f'all_{time.time_ns()}',
        'log_root': str(Path.home()/'pac_logs'), 'executor_cpu': '-1',
        'executor_fifo_priority': '0', 'ik_budget_clock': 'iterations'}
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()],
        OpaqueFunction(function=start)])
