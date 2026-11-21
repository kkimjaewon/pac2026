from pathlib import Path

from pac_hal.contract import CommandValidator, ContractError, load_config, validate_config
import pytest

ROOT = Path(__file__).parents[1]


def profile(name='franka_fr3'):
    c = load_config(ROOT/'config'/f'{name}.yaml')
    c['runtime']['active_session_id'] = 42
    return c


def command(seq=1):
    return {
        'session_id': 42, 'command_seq': seq, 'created_mono_ns': 1_000_000_000,
        'expires_mono_ns': 1_005_000_000, 'mode': 0, 'frame_id': 'map',
        'position': [0.5, 0.0, 0.3], 'orientation': [0., 0., 0., 1.],
    }


@pytest.mark.parametrize('name,dof', [('franka_fr3', 7), ('ur5e', 6)])
def test_both_profiles(name, dof):
    c = profile(name)
    assert len(c['joints']) == dof
    assert c['robot']['execution_backend'] == 'gazebo'


@pytest.mark.parametrize('section,key,value', [
    ('timing', 'command_ttl_ns', 200_000_000),
    ('timing', 'state_stale_ns', 4_000_000),
    ('timing', 'ik_budget_ns', 1_000_000),
    ('runtime', 'state_heartbeat_hz', float('nan')),
    ('runtime', 'active_session_id', True),
    ('robot', 'execution_backend', 'physical_robot'),
    ('frames', 'reference', 'base_link'),
    ('logging', 'run_id', '../escape'),
])
def test_config_rejects_contract_breaking_settings(section, key, value):
    c = profile()
    c[section][key] = value
    with pytest.raises(ContractError):
        validate_config(c)


def test_duplicate_axis_and_invalid_transform():
    c = profile()
    c['joints'][1]['device_index'] = 0
    with pytest.raises(ContractError):
        validate_config(c)
    c = profile()
    c['frames']['flange_to_tcp']['rotation']['w'] = 0.0
    with pytest.raises(ContractError):
        validate_config(c)


def test_feedback_period_controls_stale_thresholds():
    c = profile()
    c['timing'].update(state_period_ns=10_000_000, state_stale_ns=30_000_000,
                       state_lost_ns=100_000_000, receiver_watchdog_ns=100_000_000)
    validate_config(c)


def test_qos_and_unknown_keys_fail_fast():
    c = profile()
    c['qos']['command']['reliability'] = 'reliable'
    with pytest.raises(ContractError):
        validate_config(c)
    c = profile()
    c['workspace']['max_tpc_speed'] = 10
    with pytest.raises(ContractError):
        validate_config(c)


@pytest.mark.parametrize('key,value,code', [
    ('session_id', 99, 1), ('frame_id', 'fr3_link0', 2), ('mode', 1, 2),
    ('created_mono_ns', 1_002_000_000, 1),
    ('expires_mono_ns', 1_020_000_000, 1),
    ('position', [float('nan'), 0., 0.3], 1),
    ('position', [2., 0., 0.3], 4),
    ('orientation', [0., 0., 0., 0.], 1),
])
def test_command_rejection(key, value, code):
    c = command()
    c[key] = value
    assert CommandValidator(profile()).check(c, 1_001_000_000)[0] == code


def test_expiry_boundary_and_replay():
    v = CommandValidator(profile())
    assert v.check(command(), 1_005_000_000)[0] == 3
    assert v.check(command(), 1_001_000_000)[0] == 1
    assert v.check(command(0), 1_001_000_000)[0] == 1
    assert v.check(command(2), 1_001_000_000)[0] == 0
    assert v.check(command(2), 1_001_000_000)[0] == 1


def test_rejected_own_session_command_consumes_identity():
    v = CommandValidator(profile())
    bad = command(100)
    bad['position'][0] = 10.
    assert v.check(bad, 1_001_000_000)[0] == 4
    assert v.check(command(1), 1_001_000_000)[0] == 1
    assert v.check(command(100), 1_001_000_000)[0] == 1
    other = command(1000000)
    other['session_id'] = 99
    assert v.check(other, 1_001_000_000)[0] == 1
    assert v.check(command(101), 1_001_000_000)[0] == 0


@pytest.mark.parametrize('position', [[], [0., 0.], [0., 0., 0., 0.], None])
def test_interface_validator_rejects_malformed_position(position):
    msg = command()
    msg['position'] = position
    assert CommandValidator(profile()).check(msg, 1_001_000_000)[0] == 1


def test_zero_session_never_implicitly_arms():
    c = profile()
    c['runtime']['active_session_id'] = 0
    msg = command()
    msg['session_id'] = 0
    assert CommandValidator(c).check(msg, 1_001_000_000)[0] == 1
