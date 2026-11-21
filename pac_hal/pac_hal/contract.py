"""Pure validation functions. No ROS imports, no device I/O, no IK guarantees."""
import math
from pathlib import Path
import re

import yaml


class ContractError(ValueError):
    pass


def number(value, label, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError(f'{label}: finite number required')
    if not math.isfinite(value) or (positive and value <= 0):
        raise ContractError(f'{label}: invalid numeric value')
    return value


def integer(value, label, minimum=1, maximum=2**63-1):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ContractError(f'{label}: integer in [{minimum}, {maximum}] required')
    return value


def fields(value, expected, label):
    if not isinstance(value, dict) or set(value) != set(expected.split()):
        raise ContractError(f'{label}: expected keys {expected}')


def text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ContractError(f'{label}: nonempty text required')


def quaternion(values, label):
    if len(values) != 4:
        raise ContractError(f'{label}: xyzw required')
    for v in values:
        number(v, label)
    if abs(sum(v*v for v in values) - 1.0) > 1e-6:
        raise ContractError(f'{label}: quaternion must be normalized')


def transform(value, label):
    fields(value, 'translation rotation', label)
    fields(value['translation'], 'x y z', label + '.translation')
    fields(value['rotation'], 'x y z w', label + '.rotation')
    for v in value['translation'].values():
        number(v, label)
    quaternion([value['rotation'][k] for k in ('x', 'y', 'z', 'w')], label)


def validate_config(c):
    fields(c, 'contract_version robot frames joints workspace timing ik transport '
           'topics qos runtime logging' + (' execution' if 'execution' in c else ''), 'config')
    if c['contract_version'] != '0.3':
        raise ContractError('contract_version must be 0.3')
    r = c['robot']
    fields(r, 'id model adapter execution_backend robot_description_parameter' + (' joint_count' if 'joint_count' in r else ''), 'robot')
    for k, v in r.items():
        if k != 'joint_count':text(v, 'robot.' + k)
    if r['execution_backend'] not in ('interface_only', 'gazebo'):
        raise ContractError('only interface_only/gazebo backends are implemented')
    expected = {'fr3': ('franka_fr3', 7), 'ur5e': ('ur5e', 6)}
    generic = r['adapter'] == 'generic_gazebo' and 'joint_count' in r and 'execution' in c
    if not generic and (r['model'] not in expected or r['adapter'] != expected[r['model']][0]):
        raise ContractError('model and adapter must match fr3/franka_fr3 or ur5e/ur5e')
    count = r['joint_count'] if generic else expected[r['model']][1]
    integer(count, 'joint_count', 6, 7)
    if 'joint_count' in r and r['joint_count'] != count:raise ContractError('declared joint_count mismatch')
    f = c['frames']
    fields(f, 'reference base flange tcp reference_to_base flange_to_tcp', 'frames')
    if f['reference'] != 'map':
        raise ContractError('v0.3 reference frame is map')
    for key in ('base', 'flange', 'tcp'):
        text(f[key], 'frames.' + key)
    if len({f[k] for k in ('reference', 'base', 'flange', 'tcp')}) != 4:
        raise ContractError('frame names must be distinct')
    transform(f['reference_to_base'], 'reference_to_base')
    transform(f['flange_to_tcp'], 'flange_to_tcp')
    joints = c['joints']
    if not isinstance(joints, list) or len(joints) != count:
        raise ContractError('joint count does not match robot model')
    for j in joints:
        fields(j, 'name device_index direction zero_offset_rad lower_rad upper_rad '
               'max_velocity_rad_s max_acceleration_rad_s2', 'joint')
        text(j['name'], 'joint.name')
        integer(j['device_index'], 'device_index', 0, len(joints)-1)
        if type(j['direction']) not in (float, int) or j['direction'] not in (-1.0, 1.0):
            raise ContractError('joint direction must be +1 or -1')
        for k in ('zero_offset_rad', 'lower_rad', 'upper_rad'):
            number(j[k], k)
        if j['lower_rad'] >= j['upper_rad']:
            raise ContractError('joint lower must be less than upper')
        for k in ('max_velocity_rad_s', 'max_acceleration_rad_s2'):
            number(j[k], k, positive=True)
    if len({j['name'] for j in joints}) != len(joints):
        raise ContractError('joint names must be unique')
    if sorted(j['device_index'] for j in joints) != list(range(len(joints))):
        raise ContractError('device_index must be a permutation')
    w = c['workspace']
    fields(w, 'min_m max_m max_tcp_speed_m_s max_tcp_acceleration_m_s2', 'workspace')
    for k in ('min_m', 'max_m'):
        if not isinstance(w[k], list) or len(w[k]) != 3:
            raise ContractError('workspace bounds must contain xyz')
        for v in w[k]:
            number(v, k)
    if any(a >= b for a, b in zip(w['min_m'], w['max_m'])):
        raise ContractError('workspace min must be less than max')
    for k in ('max_tcp_speed_m_s', 'max_tcp_acceleration_m_s2'):
        number(w[k], k, positive=True)
    t = c['timing']
    fields(t, 'command_period_ns command_ttl_ns state_period_ns state_stale_ns '
           'state_lost_ns receiver_watchdog_ns ik_budget_ns', 'timing')
    for k, v in t.items():
        integer(v, k)
    if t['command_period_ns'] != 1_000_000 or t['command_ttl_ns'] != 5_000_000:
        raise ContractError('v0.3 fixes command period 1ms and TTL 5ms')
    if t['state_stale_ns'] != max(5_000_000, 3*t['state_period_ns']):
        raise ContractError('state_stale_ns must be max(5ms, 3*state_period_ns)')
    if t['state_lost_ns'] != max(20_000_000, 10*t['state_period_ns']):
        raise ContractError('state_lost_ns must be max(20ms, 10*state_period_ns)')
    if t['ik_budget_ns'] >= t['command_period_ns']:
        raise ContractError('IK budget must leave room within command period')
    if t['receiver_watchdog_ns'] < t['state_stale_ns']:
        raise ContractError('receiver watchdog must not be less than state stale time')
    ik = c['ik']
    fields(ik, 'solver max_iterations position_tolerance_m orientation_tolerance_rad seed', 'ik')
    if ik['solver'] != 'kdl' or ik['seed'] != 'last_solution':
        raise ContractError('initial solver/seed contract is kdl/last_solution')
    integer(ik['max_iterations'], 'max_iterations', 1, 10000)
    for k in ('position_tolerance_m', 'orientation_tolerance_rad'):
        number(ik[k], k, positive=True)
    tr = c['transport']
    fields(tr, 'controller_manager controller_name joint_command_topic joint_state_topic '
           'receiver_stop_strategy receiver_status_topic', 'transport')
    for k, v in tr.items():
        text(v, 'transport.' + k)
    for k in ('controller_manager', 'joint_command_topic', 'joint_state_topic',
              'receiver_status_topic'):
        if not tr[k].startswith('/'):
            raise ContractError(f'transport.{k} must be an absolute ROS name')
    if tr['receiver_stop_strategy'] != 'decelerate_then_hold':
        raise ContractError('stop strategy must be decelerate_then_hold')
    if tr['joint_command_topic'] == tr['joint_state_topic']:
        raise ContractError('command and state topics must differ')
    expected_topics = {'command': '/hal/final_command', 'state': '/hal/state',
                       'error': '/hal/error'}
    if c['topics'] != expected_topics:
        raise ContractError('v0.3 fixes public topic names')
    expected_qos = {
        k: {'reliability': 'reliable' if k == 'error' else 'best_effort',
            'durability': 'volatile', 'depth': 32 if k == 'error' else 1}
        for k in ('command', 'state', 'error')
    }
    if c['qos'] != expected_qos:
        raise ContractError('QoS must match v0.3 public contract')
    fields(c['runtime'], 'active_session_id state_heartbeat_hz', 'runtime')
    integer(c['runtime']['active_session_id'], 'active_session_id', 0, 2**64-1)
    hz = number(c['runtime']['state_heartbeat_hz'], 'state_heartbeat_hz', positive=True)
    if hz > 100:
        raise ContractError('interface-only heartbeat must be <=100Hz')
    log = c['logging']
    fields(log, 'run_id directory enabled', 'logging')
    if not isinstance(log['run_id'], str) or not re.fullmatch(r'[A-Za-z0-9_.-]+', log['run_id']):
        raise ContractError('run_id must be a simple path component')
    if log['run_id'] in ('.', '..'):
        raise ContractError('run_id must not traverse directories')
    text(log['directory'], 'logging.directory')
    if type(log['enabled']) is not bool:
        raise ContractError('logging.enabled must be bool')
    if 'execution' in c:
        e = c['execution']
        fields(e, 'backend_period_ns capabilities task', 'execution')
        integer(e['backend_period_ns'], 'backend_period_ns', t['command_period_ns'], t['command_ttl_ns']-1)
        if e['backend_period_ns'] % t['command_period_ns']:
            raise ContractError('backend period must be an integer multiple of input period')
        cap = e['capabilities']
        fields(cap, 'control_modes state_feedback independent_watchdog stop_strategy', 'capabilities')
        if cap['control_modes'] != ['position'] or cap['state_feedback'] is not True or cap['independent_watchdog'] is not True or cap['stop_strategy'] != 'decelerate_then_hold':
            raise ContractError('CAPABILITY_MISMATCH: implemented position/feedback/watchdog/stop required')
        task = e['task']
        if isinstance(task, dict) and 'tcp_corridor' in task:
            # Process-corridor profiles are not part of this build; refuse before start.
            raise ContractError('CAPABILITY_MISMATCH: tcp_corridor task profile unsupported')
        fields(task, 'control_mode min_dof preferred_period_ns maximum_period_ns sample_tcp_poses', 'task')
        if task['control_mode'] != 'position':raise ContractError('CAPABILITY_MISMATCH: unsupported task mode')
        integer(task['min_dof'], 'min_dof', 6, count)
        integer(task['preferred_period_ns'], 'preferred_period_ns')
        integer(task['maximum_period_ns'], 'maximum_period_ns', task['preferred_period_ns'], t['command_ttl_ns']-1)
        if e['backend_period_ns'] > task['maximum_period_ns']:
            raise ContractError('CAPABILITY_MISMATCH: transport too slow for task')
        samples = task['sample_tcp_poses']
        if not isinstance(samples,list) or len(samples)>16:raise ContractError('at most 16 representative poses')
        for sample in samples:
            if not isinstance(sample,list) or len(sample)!=7:raise ContractError('task pose must be xyz+xyzw')
            for v in sample[:3]:number(v,'task xyz')
            quaternion(sample[3:],'task quaternion')
    return c


def load_config(path):
    with Path(path).open() as stream:
        return validate_config(yaml.safe_load(stream))


class CommandValidator:
    """Only format/time/session/workspace checks; no IK or device acceptance."""

    def __init__(self, config):
        self.config = config
        self.last_seq = None

    def check(self, command, now_ns):
        # Error values match HalError.msg. No implicit session switching.
        if command['session_id'] != self.config['runtime']['active_session_id']:
            return 1, 'session mismatch; start C and B with the same nonzero session_id'
        if command['session_id'] == 0:
            return 1, 'session_id=0 is not armed for a B RUN session'
        sequence = command['command_seq']
        if type(sequence) is not int or not 0 < sequence < 2**64:
            return 1, 'command_seq must be a positive uint64'
        if self.last_seq is not None and sequence <= self.last_seq:
            return 1, 'duplicate or out-of-order command_seq (v0.3 has no separate replay code)'
        # Match the executable gateway: consume rejected own-session attempts too.
        self.last_seq = sequence
        if command['mode'] != 0 or command['frame_id'] != self.config['frames']['reference']:
            return 2, 'unsupported mode or frame'
        created, expires = command['created_mono_ns'], command['expires_mono_ns']
        if created < 0 or created > now_ns:
            return 1, 'invalid/future created_mono_ns; B and C must share CLOCK_MONOTONIC'
        if expires <= created or expires-created != self.config['timing']['command_ttl_ns']:
            return 1, 'expires_mono_ns must equal created_mono_ns + 5ms'
        if now_ns >= expires:
            return 3, 'expired command'
        try:
            if not isinstance(command['position'], (list, tuple)) or len(command['position']) != 3:
                raise ContractError('target_pose.position must have three coordinates')
            for v in command['position']:
                number(v, 'target_pose.position')
            quaternion(command['orientation'], 'target_pose.orientation')
        except ContractError as e:
            return 1, str(e)
        w = self.config['workspace']
        if any(not lo <= v <= hi for lo, v, hi in
               zip(w['min_m'], command['position'], w['max_m'])):
            return 4, 'TCP target outside configured workspace box'
        return 0, 'format/time/session/workspace validated; backend checks still required'
