from pathlib import Path
import sys
from ament_index_python.packages import get_package_share_directory
import xacro
source = Path(get_package_share_directory('ur_simulation_gz'))/'urdf/ur_gz.urdf.xacro'
xml = xacro.process_file(str(source), mappings={'ur_type':'ur5e','name':'ur',
    'simulation_controllers':str(Path(get_package_share_directory('pac_hal'))/'config/gazebo_controllers.yaml')})
Path(sys.argv[1]).write_text(xml.toxml())
