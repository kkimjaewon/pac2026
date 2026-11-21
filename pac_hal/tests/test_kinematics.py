"""Use installed official Gazebo models to test FK/IK and frame conventions."""
import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from pac_hal.contract import load_config
from pac_hal.kinematics import Kinematics
import PyKDL as kdl
import pytest
import xacro


@pytest.fixture(params=['franka_fr3', 'ur5e'])
def robot(request):
    name = request.param
    if name == 'ur5e' and os.environ.get('PAC_HAL_TEST_UR', '1') == '0':
        pytest.skip('UR packages unavailable or optional UR tests disabled')
    config = load_config(Path(__file__).parents[1]/'config'/f'{name}.yaml')
    if name == 'franka_fr3':
        file = (Path(get_package_share_directory('franka_gazebo_bringup')) /
                'urdf/franka_arm.gazebo.xacro')
        mappings = {'robot_type': 'fr3', 'hand': 'false', 'gazebo_effort': 'true'}
        q = [0., -math.pi/4, 0., -3*math.pi/4, 0., math.pi/2, math.pi/4]
    else:
        file = Path(get_package_share_directory('ur_simulation_gz'))/'urdf/ur_gz.urdf.xacro'
        mappings = {'name': 'ur', 'ur_type': 'ur5e'}
        q = [0., -1.2, 1.2, -1.4, -1.57, 0.1]
    xml = xacro.process_file(str(file), mappings=mappings).toxml()
    return config, xml, q


def test_small_pose_ik_and_joint_order(robot):
    config, xml, q = robot
    model = Kinematics(xml, config)
    assert model.names == [j['name'] for j in config['joints']]
    pose = model.fk(q)
    target = kdl.Frame(pose.M, pose.p+kdl.Vector(1e-5, 0., 0.))
    # Relax test timing to avoid scheduling-load failures; production deadline separately tested.
    config['timing']['ik_budget_ns'] = 100_000_000
    solved, elapsed = model.solve(target, q)
    error = kdl.diff(model.fk(solved), target)
    assert error.vel.Norm() < 1e-6 and error.rot.Norm() < 1e-6
    assert elapsed > 0


def test_installation_and_tcp_transform_roundtrip(robot):
    config, xml, q = robot
    original_model = Kinematics(xml, config)
    # Recover base->flange before composing the NEW installation/tool transforms.
    # UR has a nonidentity flange->tool0 alignment even without a physical tool.
    original = original_model.base.Inverse()*original_model.fk(q)*original_model.tool.Inverse()
    config['frames']['reference_to_base']['translation']['x'] = 0.2
    config['frames']['reference_to_base']['rotation'].update(
        z=math.sin(math.pi/4), w=math.cos(math.pi/4))
    config['frames']['flange_to_tcp']['translation']['z'] = 0.05
    config['timing']['ik_budget_ns'] = 100_000_000
    model = Kinematics(xml, config)
    expected = model.base*original*model.tool
    actual = model.fk(q)
    assert kdl.diff(expected, actual).vel.Norm() < 1e-10
    solved, _ = model.solve(actual, q)
    assert kdl.diff(model.fk(solved), actual).rot.Norm() < 1e-8


def test_twist_matches_fk_finite_difference(robot):
    config, xml, q = robot
    config['frames']['flange_to_tcp']['translation']['z'] = 0.05
    model = Kinematics(xml, config)
    dq = [0.01*(i+1) for i in range(len(q))]
    linear, _ = model.twist(q, dq)
    epsilon = 1e-6
    measured = (model.fk([v+d*epsilon for v, d in zip(q, dq)]).p-model.fk(q).p)/epsilon
    assert (linear-measured).Norm() < 1e-6


def test_deadline_and_unreachable_pose_are_rejected(robot):
    config, xml, q = robot
    model = Kinematics(xml, config)
    pose = model.fk(q)
    config['timing']['ik_budget_ns'] = 1
    with pytest.raises(ValueError, match='deadline'):
        model.solve(pose, q)
    config['timing']['ik_budget_ns'] = 100_000_000
    with pytest.raises(ValueError):
        model.solve(kdl.Frame(pose.M, kdl.Vector(100., 100., 100.)), q)


def test_physical_urdf_is_not_accepted_by_gazebo_backend(robot):
    config, xml, _ = robot
    with pytest.raises(ValueError, match='simulation URDF'):
        Kinematics(xml.replace('gz_ros2_control-system', 'physical-hardware'), config)


def test_operational_joint_limits_must_fit_model(robot):
    config, xml, _ = robot
    config['joints'][0]['max_velocity_rad_s'] = 100.
    with pytest.raises(ValueError, match='model joint limits'):
        Kinematics(xml, config)
