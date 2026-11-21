"""Same map task and C++ contract on FR3/UR5e; robot-specific placement and scene."""
from pathlib import Path
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

INITIAL = {'franka_fr3': '[-0.1915941617730299, -0.3611638851715237, 0.12164291046410534, -1.4187463010169854, 0.04915708020230413, 1.060542641166919, -0.05370596034279201]', 'ur5e': '[3.5323157813303343, -1.1736688280581857, -1.826774093867833, -1.7119460582538688, 1.570796326590547, -1.1800731988493458]'}

def start(context):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in INITIAL:
        raise ValueError('supported robot required')
    share = Path(get_package_share_directory('pac_hal'))
    is_ur = robot == 'ur5e'
    prefix = 'ur5e' if is_ur else 'fr3'
    arguments = {k: LaunchConfiguration(k) for k in
        ('robot','session_id','run_id','gui','log_directory','ik_budget_clock','executor_cpu','executor_fifo_priority','backend_period_ns')}
    arguments.update({
        'task_config_file': LaunchConfiguration('task_config_file'),
        'world_file': str(share/'config'/f'{prefix}_workcell_demo.sdf'),
        'collision_scene_file': str(share/'config'/f'{prefix}_workcell_scene.yaml'),
        'config_file': str(share/'config'/('ur5e_workcell.yaml' if is_ur else 'robot_fr3.yaml')),
        # Simulation initialization only; never send these as a real robot jump.
        'initial_joints': INITIAL[robot],
    })
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/hal_simulation.launch.py')),
        launch_arguments=arguments.items())]

def generate_launch_description():
    defaults = {'robot':'franka_fr3','session_id':'42','run_id':f'common_{time.time_ns()}',
        'gui':'true','log_directory':'','ik_budget_clock':'iterations','executor_cpu':'-1','executor_fifo_priority':'0','backend_period_ns':'','task_config_file':str(Path(get_package_share_directory('pac_hal'))/'config/task_common_tcp.yaml')}
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],OpaqueFunction(function=start)])
