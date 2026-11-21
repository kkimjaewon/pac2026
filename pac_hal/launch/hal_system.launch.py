import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.actions import EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pac_hal.contract import load_config, validate_config
import yaml


def start(context):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('franka_fr3', 'ur5e', 'generic_gazebo'):
        raise ValueError('robot must be franka_fr3, ur5e or explicit generic_gazebo')
    path = LaunchConfiguration('config_file').perform(context)
    if not path and robot == 'generic_gazebo':
        raise ValueError('generic_gazebo requires an explicit validated config_file and externally spawned URDF')
    if not path:
        profile = 'robot_fr3.yaml' if robot == 'franka_fr3' else robot+'.yaml'
        path = os.path.join(get_package_share_directory('pac_hal'), 'config', profile)
    config = load_config(path)
    backend = LaunchConfiguration('backend').perform(context)
    if backend:
        config['robot']['execution_backend'] = backend
    config['runtime']['active_session_id'] = int(
        LaunchConfiguration('session_id').perform(context))
    for argument, key in (('run_id', 'run_id'), ('log_directory', 'directory')):
        value = LaunchConfiguration(argument).perform(context)
        if value:
            config['logging'][key] = value
    task_file = LaunchConfiguration('task_config_file').perform(context)
    if task_file:
        config['execution']['task'] = yaml.safe_load(Path(task_file).read_text())
    backend_period = LaunchConfiguration('backend_period_ns').perform(context)
    if backend_period:
        config['execution']['backend_period_ns'] = int(backend_period)
    # 틀린 좌표/QoS/TTL 설정으로 controller를 전환하기 전에 먼저 전체 스키마를 검증한다.
    validate_config(config)
    gazebo = config['robot']['execution_backend'] == 'gazebo'
    if gazebo and not LaunchConfiguration('collision_scene_file').perform(context):
        raise ValueError('executable HAL requires an explicit collision scene; use team/common_workcell launch')
    if gazebo and config['runtime']['active_session_id'] <= 0:
        raise ValueError('Gazebo execution requires session_id > 0')
    # 변경된 파일을 실행 중 다시 읽지 않는다. node와 controller 준비는 같은 snapshot을 공유한다.
    snapshot = Path(tempfile.mkdtemp(prefix='pac_hal_config_'))/'profile.yaml'
    snapshot.write_text(yaml.safe_dump(config, sort_keys=False))
    # Both robots share command validation, state/error semantics and the C++ gateway.
    if config['robot']['adapter'] != robot:
        raise ValueError('launch robot and profile adapter disagree')
    executable = 'hal_node' if gazebo else 'hal_config_node'
    node = Node(package='pac_hal', executable=executable,
                name='hal_command_gateway', output='screen', parameters=[{
                    'config_file': str(snapshot),
                    'expected_model_file': str(snapshot.with_suffix('.urdf')),
                    'session_id': config['runtime']['active_session_id'],
                    'run_id': config['logging']['run_id'],
                    'log_directory': config['logging']['directory'],
                    'collision_scene_file': LaunchConfiguration('collision_scene_file').perform(context),
                    'ik_budget_clock': LaunchConfiguration('ik_budget_clock').perform(context),
                    'executor_cpu': int(LaunchConfiguration('executor_cpu').perform(context)),
                    'executor_fifo_priority': int(LaunchConfiguration('executor_fifo_priority').perform(context)),
                }])
    if not gazebo:
        return [node]
    # URDF가 Gazebo인지 검사하고 position 수신 controller를 준비한 후에만 명령 gateway를 연다.
    prepare = Node(package='pac_hal', executable='prepare_hal_controller', output='screen',
                   arguments=['--config-file', str(snapshot),
                              '--description-output', str(snapshot.with_suffix('.urdf'))])

    def after_prepare(event, _context):
        if event.returncode != 0:
            return [LogInfo(msg='HAL receiver setup failed; gateway not started'),
                    EmitEvent(event=Shutdown(reason='HAL setup failure'))]
        return [node]

    return [RegisterEventHandler(OnProcessExit(
        target_action=prepare, on_exit=after_prepare)), prepare]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='franka_fr3'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('backend', default_value=''),
        DeclareLaunchArgument('task_config_file', default_value=''),
        DeclareLaunchArgument('backend_period_ns', default_value=''),
        DeclareLaunchArgument('session_id', default_value='0'),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('log_directory', default_value=''),
        DeclareLaunchArgument('collision_scene_file', default_value=''),
        DeclareLaunchArgument('ik_budget_clock', default_value='iterations'),
        DeclareLaunchArgument('executor_cpu', default_value='-1'),
        DeclareLaunchArgument('executor_fifo_priority', default_value='0'),
        OpaqueFunction(function=start),
    ])
