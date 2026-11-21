"""Team workcell with position-mode Gazebo hardware and the C HAL."""
import hashlib
import json
import math
import subprocess
import tempfile
import yaml
import os
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from pac_hal.contract import load_config
from pac_hal.workcell import validate_scene_world, camera_from_world
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


# Simulated model -> (C adapter, default profile, model package). This is only
# simulation wiring; C accepts a new arm through its validated profile.
SIM_ROBOTS = {
    'franka_fr3': ('franka_fr3', 'robot_fr3.yaml', 'franka_description'),
    'ur5e': ('ur5e', 'ur5e.yaml', 'ur_description'),
    # HD Hyundai Robotics official hdr_description (BSD-3); C uses generic_gazebo.
    'ha006b': ('generic_gazebo', 'ha006b_team.yaml', 'hdr_description'),
}


def start(context):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in SIM_ROBOTS:
        raise ValueError('robot must be one of ' + ', '.join(SIM_ROBOTS))
    adapter, default_profile, model_package = SIM_ROBOTS[robot]
    share = get_package_share_directory
    if robot == 'franka_fr3':
        file = Path(share('franka_gazebo_bringup'))/'urdf/franka_arm.gazebo.xacro'
        mapping = {'robot_type': 'fr3', 'hand': 'false', 'gazebo_effort': 'false'}
    elif robot == 'ha006b':
        file = Path(share('hdr_description'))/'urdf/hdr.urdf.xacro'
        mapping = {'robot_model': 'ha006b', 'name': 'ha006b', 'use_sim': 'true',
                   'hdr_ros2_control': str(Path(share('pac_hal'))/'config/gazebo_controllers.yaml')}
    else:
        file = Path(share('ur_simulation_gz'))/'urdf/ur_gz.urdf.xacro'
        mapping = {'ur_type': 'ur5e', 'name': 'ur',
                   'simulation_controllers': str(
                       Path(share('pac_hal'))/'config/gazebo_controllers.yaml')}
    description = xacro.process_file(str(file), mappings=mapping).toxml()
    if robot == 'franka_fr3':
        model = ET.fromstring(description)
        plugins = model.findall("gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']/parameters")
        if len(plugins) != 1:
            raise ValueError('FR3 must have one gz_ros2_control parameter file')
        # Shared manager defaults, no example motion controller. The C controller's
        # robot limits/session/watchdog are generated from its validated profile.
        plugins[0].text = str(Path(share('pac_hal'))/'config/fr3_controllers.yaml')
        description = ET.tostring(model, encoding='unicode')
    if robot == 'ha006b':
        model = ET.fromstring(description)
        # The SDF owns the floor. The 'world' root stays: Gazebo welds the arm there.
        for tag, name in [('joint', 'ground_plane_joint'), ('link', 'ground_plane')]:
            element = model.find(f"{tag}[@name='{name}']")
            if element is not None:
                model.remove(element)
        description = ET.tostring(model, encoding='unicode')
    if robot == 'ur5e':
        model = ET.fromstring(description)
        # The SDF owns the floor; do not carry UR example scenery on the robot pedestal.
        for tag, name in [('joint', 'ground_plane_joint'), ('link', 'ground_plane')]:
            element = model.find(f"{tag}[@name='{name}']")
            if element is not None:
                model.remove(element)
        # Give the robot anchor its own name; 'world' is not the shifted robot base.
        for element in model.iter():
            for attr in ('name', 'link', 'reference'):
                if element.get(attr) == 'world':
                    element.set(attr, 'ur5e_mount')
        # Nonsingular demonstration posture; only simulated hardware initial states.
        initial = [0., -1.2, 1.2, -1.4, -1.57, .1]
        for joint, value in zip(model.findall('ros2_control/joint'), initial):
            param = joint.find("state_interface[@name='position']/param[@name='initial_value']")
            if param is None:
                raise ValueError('UR initial position parameter missing')
            param.text = str(value)
        description = ET.tostring(model, encoding='unicode')
    initial_text = LaunchConfiguration('initial_joints').perform(context)
    if initial_text:
        initial = json.loads(initial_text)
        if len(initial) != (7 if robot == 'franka_fr3' else 6) or not all(type(x) in (int, float) and math.isfinite(x) for x in initial):
            raise ValueError('finite configured initial joint count required')
        model = ET.fromstring(description)
        for joint, value in zip(model.findall('ros2_control/joint'), initial):
            param = joint.find("state_interface[@name='position']/param[@name='initial_value']")
            if param is None:
                raise ValueError('initial position missing')
            param.text = str(value)
        description = ET.tostring(model, encoding='unicode')
    world_text = LaunchConfiguration('world_file').perform(context)
    world = Path(world_text) if world_text else Path(share('pac_worlds'))/'worlds/franka_world.sdf'
    profile_path = LaunchConfiguration('config_file').perform(context)
    if profile_path and load_config(profile_path)['robot']['adapter'] != adapter:
        raise ValueError('launch robot and profile adapter disagree')
    scene_file = LaunchConfiguration('collision_scene_file').perform(context)
    if scene_file:
        validate_scene_world(world, scene_file)
        config_path = LaunchConfiguration('config_file').perform(context) or str(Path(share('pac_hal'))/'config'/default_profile)
        config = load_config(config_path)
        config['runtime']['active_session_id'] = int(LaunchConfiguration('session_id').perform(context))
        model = ET.fromstring(description)
        values = [float(j.find("state_interface[@name='position']/param[@name='initial_value']").text)
                  for j in model.findall('ros2_control/joint')]
        with tempfile.TemporaryDirectory(prefix='fr3_preflight_') as directory:
            folder = Path(directory)
            (folder/'robot.urdf').write_text(description)
            (folder/'config.yaml').write_text(yaml.safe_dump(config))
            check = subprocess.run([str(Path(get_package_prefix('pac_hal'))/'lib/pac_hal/hal_scene_check'),
                                    str(folder/'config.yaml'), scene_file, str(folder/'robot.urdf'), json.dumps(values)],
                                   capture_output=True, text=True, timeout=30)
            if check.returncode:
                raise ValueError('robot initial scene check failed: ' + check.stdout + check.stderr)
    world_model = ET.parse(world).getroot().find('world')
    world_name = world_model.get('name')
    # Only resolve the selected robot's assets: missing UR must not stop FR3.
    paths = [str(Path(share(model_package)).parent)]
    old = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = ':'.join(paths+[old])
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    # Launch directly so signals reach Gazebo instead of an intermediate shell.
    gz = ExecuteProcess(cmd=['gz', 'sim'] + ([] if gui else ['-s']) + ['-r', str(world)],
                        output='screen', on_exit=Shutdown(reason='Gazebo exited'),
                        additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join([
                            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                            os.environ.get('LD_LIBRARY_PATH', '')])})
    publisher = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     parameters=[{'robot_description': description, 'use_sim_time': True}])
    # Spawn pose and C map->base transform come from the same immutable profile.
    profile = load_config(profile_path or str(Path(share('pac_hal'))/'config'/default_profile))
    install = profile['frames']['reference_to_base']
    q, t = install['rotation'], install['translation']
    roll = math.atan2(2*(q['w']*q['x']+q['y']*q['z']), 1-2*(q['x']**2+q['y']**2))
    pitch = math.asin(max(-1., min(1., 2*(q['w']*q['y']-q['z']*q['x']))))
    yaw = math.atan2(2*(q['w']*q['z']+q['x']*q['y']), 1-2*(q['y']**2+q['z']**2))
    # Gazebo welds a model to the world only through a root link named 'world'
    # (FR3: world -> fr3_link0 fixed). The UR anchor is renamed for TF/C, so spawn
    # Gazebo from a copy with the original name; otherwise the UR5e is a free body
    # on its pedestal and tips over when the arm reaches out (measured roll 0.82 rad
    # at TCP x=0.395 while joint FK still reported the commanded pose).
    source = ['-topic', '/robot_description']
    if robot == 'ur5e':
        gazebo_model = ET.fromstring(description)
        for element in gazebo_model.iter():
            for attr in ('name', 'link', 'reference'):
                if element.get(attr) == 'ur5e_mount':
                    element.set(attr, 'world')
        gazebo_text = ET.tostring(gazebo_model, encoding='unicode')
        gazebo_file = Path(tempfile.gettempdir())/'pac_hal_spawn'/f"ur5e_{hashlib.sha256(gazebo_text.encode()).hexdigest()[:12]}.urdf"
        gazebo_file.parent.mkdir(exist_ok=True)
        if not gazebo_file.exists():
            gazebo_file.write_text(gazebo_text)
        source = ['-file', str(gazebo_file)]
    spawn = Node(package='ros_gz_sim', executable='create',
                 arguments=[*source, '-name', robot,
                            '-x', str(t['x']), '-y', str(t['y']), '-z', str(t['z']),
                            '-R', str(roll), '-P', str(pitch), '-Y', str(yaw)], output='screen')
    links = {link.get('name') for link in ET.fromstring(description).findall('link')}
    children = {joint.find('child').get('link') for joint in ET.fromstring(description).findall('joint')}
    roots = links - children
    if len(roots) != 1:
        raise ValueError('single robot TF root required')
    # These bundled models have an identity fixed root->base transform.
    # Keep ROS TF, Gazebo placement and C's installation transform consistent.
    anchor = roots.pop()
    placement_tf = Node(package='tf2_ros', executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', anchor,
                   '--x', str(t['x']), '--y', str(t['y']), '--z', str(t['z']),
                   '--qx', str(q['x']), '--qy', str(q['y']), '--qz', str(q['z']), '--qw', str(q['w'])])
    broadcaster = Node(package='controller_manager', executable='spawner',
                       arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'])
    camera = camera_from_world(world)
    base = camera['topic']
    topic_mode = LaunchConfiguration('camera_ros_topics').perform(context)
    if topic_mode not in ('native', 'ai_contract'):
        raise ValueError('camera_ros_topics must be native or ai_contract')
    remappings = [] if topic_mode == 'native' else [
        (base+'/image', '/depth_camera/image_raw'),
        (base+'/depth_image', '/depth_camera/depth/image_raw'),
        (base+'/camera_info', '/depth_camera/camera_info')]
    camera_args = [base+'/image@sensor_msgs/msg/Image[gz.msgs.Image',
                   base+'/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                   base+'/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']
    camera_bridges = []
    camera_config = LaunchConfiguration('camera_bridge_config').perform(context)
    if camera_config:
        entries = yaml.safe_load(Path(camera_config).read_text())
        expected = {(base+'/image', '/depth_camera/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                    (base+'/depth_image', '/depth_camera/depth/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                    (base+'/camera_info', '/depth_camera/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')}
        actual = {(e['gz_topic_name'], e['ros_topic_name'], e['ros_type_name'], e['gz_type_name']) for e in entries}
        if topic_mode != 'ai_contract' or len(entries) != 3 or actual != expected or any(e['direction'] != 'GZ_TO_ROS' for e in entries):
            raise ValueError('camera bridge config must match the SDF and AI topic contract')
        camera_args, remappings = [], []
        # parameter_bridge's CLI path does not load config_file entries; keep the camera YAML on its own node.
        camera_bridges = [Node(package='ros_gz_bridge', executable='parameter_bridge',
            name='camera_bridge', parameters=[{'config_file': camera_config}])]
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
        f'/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', *camera_args,
    ], remappings=remappings)
    camera_nodes = []
    if LaunchConfiguration('camera_tf').perform(context).lower() == 'true':
        xyz, quat = camera['translation'], camera['quaternion_xyzw']
        camera_nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
            arguments=['--frame-id','map','--child-frame-id',camera['frame'],
                       '--x',str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),
                       '--qx',str(quat[0]),'--qy',str(quat[1]),'--qz',str(quat[2]),'--qw',str(quat[3])]))
    hal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(share('pac_hal'))/'launch/hal_system.launch.py')),
        launch_arguments={**{key: LaunchConfiguration(key) for key in
                          ('session_id', 'run_id', 'log_directory', 'config_file', 'collision_scene_file', 'executor_cpu', 'executor_fifo_priority', 'ik_budget_clock', 'task_config_file', 'backend_period_ns')},
                          'robot': adapter}.items())

    warmup = None
    if scene_file:
        plate = world_model.find("model[@name='steel_plate']")
        if plate is None:
            raise ValueError('guarded workcell steel_plate model missing')
        warmup = Node(package='pac_hal', executable='prepare_workcell_service', output='screen',
                      arguments=['--service', f'/world/{world_name}/set_pose', '--entity', 'steel_plate',
                                 '--pose-rpy=' + plate.findtext('pose', '0 0 0 0 0 0'),
                                 '--joint-names', json.dumps([j['name'] for j in config['joints']])])

    def after_spawn(event, _context):
        return [broadcaster] if event.returncode == 0 else [
            Shutdown(reason='robot spawn failed')]

    def after_broadcaster(event, _context):
        return [warmup if warmup is not None else hal] if event.returncode == 0 else [
            Shutdown(reason='joint broadcaster startup failed')]

    warmup_handler = [RegisterEventHandler(OnProcessExit(target_action=warmup,
        on_exit=lambda event, _context: [hal] if event.returncode == 0 else [
            Shutdown(reason='workcell service warmup failed')]))] if warmup is not None else []
    return [gz, publisher, placement_tf, spawn, bridge, *camera_bridges, *camera_nodes, *warmup_handler,
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
            RegisterEventHandler(OnProcessExit(
                target_action=broadcaster, on_exit=after_broadcaster))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='franka_fr3'),
        DeclareLaunchArgument('session_id', default_value='42'),
        DeclareLaunchArgument('run_id', default_value=f'hal_sim_{time.time_ns()}'),
        DeclareLaunchArgument('log_directory', default_value=''),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world_file', default_value=''),
        DeclareLaunchArgument('camera_ros_topics', default_value='native'),
        DeclareLaunchArgument('camera_tf', default_value='false'),
        DeclareLaunchArgument('camera_bridge_config', default_value=''),
        DeclareLaunchArgument('initial_joints', default_value=''),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('task_config_file', default_value=''),
        DeclareLaunchArgument('backend_period_ns', default_value=''),
        DeclareLaunchArgument('collision_scene_file', default_value=''),
        DeclareLaunchArgument('ik_budget_clock', default_value='iterations'),
        DeclareLaunchArgument('executor_cpu', default_value='-1'),
        DeclareLaunchArgument('executor_fifo_priority', default_value='0'),
        OpaqueFunction(function=start),
    ])
