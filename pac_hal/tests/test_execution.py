"""Profiles cannot claim an unimplemented mode/backend or weaken timing checks."""
from copy import deepcopy
from pathlib import Path
import pytest
import yaml
from pac_hal.contract import load_config,validate_config,ContractError
BASE=Path(__file__).resolve().parents[1]/'config'

@pytest.mark.parametrize('name',['robot_fr3','ur5e','ur5e_workcell'])
def test_explicit_execution_profiles(name):
    c=load_config(BASE/(name+'.yaml'))
    assert c['execution']['capabilities']['control_modes']==['position']

@pytest.mark.parametrize('key,value', [('state_feedback',False),('independent_watchdog',False),('control_modes',['force']),('stop_strategy','hold')])
def test_false_backend_claim_rejected(key,value):
    c=deepcopy(load_config(BASE/'robot_fr3.yaml'))
    c['execution']['capabilities'][key]=value
    with pytest.raises(ContractError):validate_config(c)

@pytest.mark.parametrize('period',[0,500000,1500000,5000000,8000000])
def test_rate_cannot_exceed_command_lifetime(period):
    c=deepcopy(load_config(BASE/'robot_fr3.yaml'));c['execution']['backend_period_ns']=period
    with pytest.raises(ContractError):validate_config(c)

def test_slow_rate_requires_explicit_task_permission():
    c=deepcopy(load_config(BASE/'robot_fr3.yaml'));c['execution']['backend_period_ns']=2000000
    with pytest.raises(ContractError):validate_config(c)
    c['execution']['task']['maximum_period_ns']=4000000
    validate_config(c)

@pytest.mark.parametrize('dof',[0,5,8,100])
def test_unsupported_arm_capacity_rejected(dof):
    c=deepcopy(load_config(BASE/'robot_fr3.yaml'));c['robot'].update(model='third_test',adapter='generic_gazebo',joint_count=dof)
    with pytest.raises(ContractError):validate_config(c)

def test_generic_requires_explicit_contract_and_still_rejects_real_backend():
    c=deepcopy(load_config(BASE/'robot_fr3.yaml'));c['robot'].update(model='third_test',adapter='generic_gazebo')
    validate_config(c)  # metadata extension only; actual URDF/scene/feedback preflight still required
    c['robot']['execution_backend']='real'
    with pytest.raises(ContractError):validate_config(c)
    c['robot']['execution_backend']='gazebo';del c['execution']
    with pytest.raises(ContractError):validate_config(c)

@pytest.mark.parametrize('pose',[[.35,0,.81,0,0,0,0],[float('nan'),0,.81,1,0,0,0],[.35,0,.81]])
def test_bad_task_pose_rejected(pose):
    c=deepcopy(load_config(BASE/'robot_fr3.yaml'));c['execution']['task']['sample_tcp_poses']=[pose]
    with pytest.raises(ContractError):validate_config(c)


def test_process_corridor_profile_is_refused_in_this_build():
    c=deepcopy(load_config(BASE/'robot_fr3.yaml'))
    c['execution']['task']['tcp_corridor']={'start_m':[.33,0,.812],'end_m':[.37,0,.812],'radius_m':.01,
        'orientation_xyzw':[1,0,0,0],'max_orientation_error_rad':.01}
    with pytest.raises(ContractError, match='tcp_corridor'):validate_config(c)
