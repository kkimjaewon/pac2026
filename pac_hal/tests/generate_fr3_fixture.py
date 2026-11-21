"""Generate an official FR3 model for native converter tests at CMake configure time."""
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
import xacro

source = (Path(get_package_share_directory('franka_gazebo_bringup')) /
          'urdf/franka_arm.gazebo.xacro')
model = xacro.process_file(str(source), mappings={
    'robot_type': 'fr3', 'hand': 'false', 'gazebo_effort': 'false'})
Path(sys.argv[1]).write_text(model.toxml())
