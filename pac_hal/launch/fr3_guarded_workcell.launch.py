"""Proposed FR3 cell with a checked elevated standby pose."""
from pathlib import Path
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('pac_hal'))
    defaults = {'session_id': '42', 'run_id': f'fr3_guarded_{time.time_ns()}',
                'log_directory': '', 'gui': 'true', 'executor_cpu': '-1',
                'executor_fifo_priority': '0', 'ik_budget_clock': 'iterations'}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/hal_simulation.launch.py')),
            launch_arguments={
                **{k: LaunchConfiguration(k) for k in defaults},
                'robot': 'franka_fr3', 'world_file': str(share/'config/fr3_workcell_demo.sdf'),
                'collision_scene_file': str(share/'config/fr3_workcell_scene.yaml'),
                # 시뮬 초기화 값이다. 실물에 이 관절값을 즉시 명령하는 용도가 아니다.
                'initial_joints': '[-0.4,-0.3571429835,0,-1.4191722009,0,1.0620303777,0.3853970320]',
            }.items()),
    ])
