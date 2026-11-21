"""Single team simulator, C controller, camera bridge and SDF-derived TF."""
import time
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    defaults = {'robot': 'franka_fr3', 'gui': 'false', 'session_id': '42', 'run_id': f'sim_{time.time_ns()}',
        'log_directory': '', 'executor_cpu': '-1', 'executor_fifo_priority': '0',
        'ik_budget_clock': 'iterations'}
    target = Path(get_package_share_directory('pac_hal'))/'launch/team_workcell.launch.py'
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(target)),
            launch_arguments={**{k: LaunchConfiguration(k) for k in defaults},
                'camera_bridge_config': str(Path(get_package_share_directory('pac_bringup'))/'config/depth_camera_bridge.yaml')}.items()),
    ])
