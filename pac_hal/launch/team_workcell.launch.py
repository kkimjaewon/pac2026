"""BC-07/08: one Gazebo, upstream team world, guarded FR3 or UR5e, camera contract/TF.

Parking joints initialize only simulated hardware; this never commands a real
robot to jump. B must still plan the approach from the fresh measured TCP.
The team world (camera, plate, table) is the same for both robots; UR5e only
adds the 0.3m pedestal of its validated mount (ur5e_workcell.yaml).
"""
from pathlib import Path
import json,tempfile,time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pac_hal.workcell import world_with_pedestal

# Both parkings put the TCP at z=0.83 with the tool pointing down (xyzw 1,0,0,0).
# FR3: TCP (0.10,0,0.83). UR5e cannot reach x<=0.12 tool-down on its pedestal,
# so it parks at (0.25,0,0.83): min Jacobian singular value 0.142, outside the
# plate motion envelope (x>=0.30).
ROBOTS = {
    'franka_fr3': {'parking': [0.5217159956713354,-0.887787653263612,-0.08295272477974769,
                               -1.614392028894036,-0.09646110181032354,0.730606472727573,0.3973491336150268],
                   'config': 'robot_fr3.yaml', 'scene': 'fr3_team_scene.yaml', 'pedestal': False},
    'ur5e': {'parking': [3.703971,-0.917388,-1.887315,-1.907686,1.570796,-1.008418],
             'config': 'ur5e_workcell.yaml', 'scene': 'ur5e_team_scene.yaml', 'pedestal': True},
    # HA006B on the floor at x=-0.4 parks at the same TCP as FR3 (0.10,0,0.83) tool-down.
    'ha006b': {'parking': [0.0,2.246203,-0.676256,0.0,-1.569954,3.141593],
               'config': 'ha006b_team.yaml', 'scene': 'ha006b_team_scene.yaml', 'pedestal': False},
}

def team_world(robot, hal):
    world=Path(get_package_share_directory('pac_worlds'))/'worlds/franka_world.sdf'
    if not ROBOTS[robot]['pedestal']:
        return world
    return world_with_pedestal(world, hal/'config/ur5e_workcell_demo.sdf',
                               Path(tempfile.gettempdir())/'pac_hal_team_world')

def start(context):
    hal=Path(get_package_share_directory('pac_hal'))
    robot=LaunchConfiguration('robot').perform(context)
    if robot not in ROBOTS:
        raise ValueError('robot must be one of ' + ', '.join(ROBOTS))
    spec=ROBOTS[robot]
    arguments={k:LaunchConfiguration(k) for k in
               ('gui','session_id','run_id','log_directory','executor_cpu','executor_fifo_priority','ik_budget_clock','camera_bridge_config')}
    arguments.update({'robot':robot,'world_file':str(team_world(robot,hal)),
                      'initial_joints':json.dumps(spec['parking']),'config_file':str(hal/'config'/spec['config']),
                      'collision_scene_file':str(hal/'config'/spec['scene']),
                      'camera_ros_topics':'ai_contract','camera_tf':'true'})
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(hal/'launch/hal_simulation.launch.py')),
                                     launch_arguments=arguments.items())]

def generate_launch_description():
    defaults={'robot':'franka_fr3','gui':'false','session_id':'42','run_id':f'team_{time.time_ns()}',
              'log_directory':'','executor_cpu':'-1','executor_fifo_priority':'0','ik_budget_clock':'iterations','camera_bridge_config':''}
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
                              OpaqueFunction(function=start)])
