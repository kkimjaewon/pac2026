"""Imported pac_b_mock B stand-in, adapted to the existing FR3 C contract.

This is a development publisher, not B's real-time controller. Public message fields
stay unchanged. Session registration belongs to C launch, never to the first command.
"""
import collections
import copy
import math
import threading
import time

from intelion_interfaces.msg import HalCommand, HalError, HalState
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

QOS_CMD = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
QOS_STATE = QOS_CMD
QOS_ERROR = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
EXPIRE_NS = 5_000_000
SOURCE_PERIOD_NS = 100_000_000
REPLAY_AGO_S = 10.0
MOTIONS = ('hold', 'sine', 'line')
FAULTS = ('none', 'expired', 'replay', 'nan', 'invalid_number',
          'unreachable', 'new_session', 'stop')
HAL_STATE_NAMES = {0: 'INITIALIZING', 1: 'CONNECTING', 2: 'READY',
                   3: 'ACTIVE', 4: 'FAULT', 5: 'RECONNECTING'}
CMD_RESULT_NAMES = {0: 'ACCEPTED', 1: 'REJECTED', 2: 'SENT'}


def planned_position(motion, t, anchor, amplitude=.002, period=6., absolute=False):
    """Generate smooth 2mm y motion relative to a fresh measured TCP.

    The archive's fixed positions remain opt-in for rejection/compatibility tests.
    Smooth starts/turns prevent arbitrary jumps from violating C joint acceleration.
    """
    if absolute:
        if motion == 'sine':
            return (.50, .05*math.sin(2*math.pi*t/4.), .20)
        if motion == 'line':
            distance = (.02*t) % .8
            return (.40, -.20 + (distance if distance <= .4 else .8-distance), .20)
        return (.50, .00, .20)
    if motion == 'hold':
        return tuple(anchor)
    if motion == 'sine':
        offset = amplitude*(1-math.cos(2*math.pi*t/period))/2
    else:
        phase = (t/period) % 1.
        progress = 2*phase if phase <= .5 else 2*(1-phase)
        # Quintic ease has zero velocity and acceleration at the line endpoints.
        offset = amplitude*(10*progress**3-15*progress**4+6*progress**5)
    return (anchor[0], anchor[1]+offset, anchor[2])


def expired_times(now):
    """Keep the required 5ms TTL shape, but make both timestamps old.

    expires < created is E_FORMAT, so the archive's original expired case did not
    actually isolate E_EXPIRED. This case does, without relaxing C validation.
    """
    return now-10_000_000, now-5_000_000


class FakeB(Node):
    def __init__(self):
        super().__init__('fake_b_node')
        fixed = ParameterDescriptor(read_only=True)
        for name, value in [('rate_hz', 1000.), ('session_id', 42),
                            ('reference_mode', 'measured'), ('amplitude_m', .002),
                            ('motion_period_s', 6.), ('duration_s', 0.)]:
            self.declare_parameter(name, value, fixed)
        self.declare_parameter('motion', 'hold')
        self.declare_parameter('fault_mode', 'none')
        self.rate_hz = self.get_parameter('rate_hz').value
        self.session_id = self.get_parameter('session_id').value
        self.reference_mode = self.get_parameter('reference_mode').value
        self.amplitude = self.get_parameter('amplitude_m').value
        self.period = self.get_parameter('motion_period_s').value
        self.duration = self.get_parameter('duration_s').value
        self.motion = self.get_parameter('motion').value
        self.fault = self.get_parameter('fault_mode').value
        if (not math.isfinite(self.rate_hz) or not 0 < self.rate_hz <= 1000 or
                self.session_id <= 0 or self.session_id >= 2**63):
            raise ValueError('rate_hz must be (0,1000], session_id in [1,2^63)')
        if self.reference_mode not in ('measured', 'absolute'):
            raise ValueError('reference_mode must be measured or absolute')
        if (not math.isfinite(self.amplitude) or not 0 < self.amplitude <= .005 or
                not math.isfinite(self.period) or self.period < 4 or
                not math.isfinite(self.duration) or self.duration < 0):
            raise ValueError('amplitude_m in (0,.005], period >=4s, duration >=0s')
        if self.motion not in MOTIONS or self.fault not in FAULTS:
            raise ValueError('unknown motion or fault_mode')
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.state_count, self.last_state = 0, None
        self.error_counts, self.last_error = collections.Counter(), None
        self.anchor, self.started_ns = None, None
        self.t0_ns, self.seq = time.monotonic_ns(), 0
        self.history = collections.deque(maxlen=max(1, int(self.rate_hz*REPLAY_AGO_S)))
        self.pub = self.create_publisher(HalCommand, '/hal/final_command', QOS_CMD)
        self.create_subscription(HalState, '/hal/state', self.on_state, QOS_STATE)
        self.create_subscription(HalError, '/hal/error', self.on_error, QOS_ERROR)
        self.add_on_set_parameters_callback(self.on_params)
        self.create_timer(1., self.report_hal)
        self.context.on_shutdown(self.stop_event.set)
        self.thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'B mock session={self.session_id} motion={self.motion} '
            f'reference={self.reference_mode}; waiting for fresh matching HAL state')

    def on_params(self, params):
        for p in params:
            if p.name == 'motion' and p.value not in MOTIONS:
                return SetParametersResult(successful=False, reason=f'motion: {MOTIONS}')
            if p.name == 'fault_mode' and p.value not in FAULTS:
                return SetParametersResult(successful=False, reason=f'fault_mode: {FAULTS}')
        with self.lock:
            for p in params:
                if p.name == 'motion' and p.value != self.motion:
                    self.motion = p.value
                    self.anchor = None  # New motion starts smoothly from current measured pose.
                elif p.name == 'fault_mode':
                    self.fault = p.value
                    self.get_logger().warn(f'fault_mode -> {p.value}')
        return SetParametersResult(successful=True)

    def on_state(self, msg):
        with self.lock:
            self.state_count += 1
            self.last_state = msg

    def on_error(self, msg):
        with self.lock:
            self.error_counts[(msg.code, msg.active)] += 1
            self.last_error = msg

    def tick(self):
        now = time.monotonic_ns()
        change_session = False
        injected_session = None
        with self.lock:
            state, fault = self.last_state, self.fault
            if fault == 'stop':
                return False
            if (state is None or state.stale or not state.pose_valid or
                    state.hal_state not in (HalState.HAL_READY, HalState.HAL_ACTIVE) or
                    not 0 <= now-state.received_mono_ns < 30_000_000):
                return False  # Stale/missing robot measurements cannot authorize movement.
            if state.active_session_id != self.session_id:
                return False
            if self.pub.get_subscription_count() == 0:
                return False
            if self.anchor is None:
                self.anchor = copy.deepcopy(state.tcp_pose)
                self.started_ns = now
                # Restarting mock in the same C session must not restart seq at 1.
                self.seq = max(self.seq, state.last_command_seq)
                self.get_logger().info(f'anchored at measured TCP, next seq={self.seq+1}')
            if self.duration and (now-self.started_ns)*1e-9 >= self.duration:
                self.stop_event.set()
                self.get_logger().info('duration complete; publishing stopped for C watchdog test')
                return False
            if fault == 'new_session':
                injected_session = now
                change_session = True
                fault = 'none'
                self.get_logger().warn(
                    f'fault: unregistered session={injected_session}; C keeps its active session')
            self.seq += 1
            seq = self.seq
            source = (now-self.t0_ns)//SOURCE_PERIOD_NS+1
            pose = copy.deepcopy(self.anchor)
            anchor = (pose.position.x, pose.position.y, pose.position.z)
            xyz = planned_position(self.motion, (now-self.started_ns)*1e-9, anchor,
                                   self.amplitude, self.period, self.reference_mode == 'absolute')
            pose.position.x, pose.position.y, pose.position.z = xyz
            if self.reference_mode == 'absolute':
                pose.orientation.x, pose.orientation.y = 1., 0.
                pose.orientation.z, pose.orientation.w = 0., 0.
            self.history.append((seq, copy.deepcopy(pose), source))
            if fault == 'replay':
                seq, pose, source = self.history[0]
                pose = copy.deepcopy(pose)
            elif fault in ('nan', 'invalid_number'):
                pose.position.x = pose.position.y = pose.position.z = math.nan
            elif fault == 'unreachable':
                pose.position.x, pose.position.y, pose.position.z = 2., 0., .2
            msg = HalCommand()
            # Copy actual Gazebo stamp; MONOTONIC alone governs creation/expiration.
            msg.header.stamp, msg.header.frame_id = state.header.stamp, 'map'
            # One unregistered-session probe, then resume the configured session/sequence.
            msg.session_id = injected_session if injected_session is not None else self.session_id
            msg.command_seq = 1 if injected_session is not None else seq
            msg.created_mono_ns, msg.expires_mono_ns = (
                expired_times(now) if fault == 'expired' else (now, now+EXPIRE_NS))
            msg.mode, msg.target_pose = HalCommand.MODE_POSITION, pose
            msg.source_result_id = source
        if change_session:
            # Parameter callbacks take the same lock, so never call this while locked.
            self.set_parameters([Parameter('fault_mode', Parameter.Type.STRING, 'none')])
        self.pub.publish(msg)
        return True

    def publish_loop(self):
        period_ns = int(1e9/self.rate_hz)
        next_ns = time.monotonic_ns()
        start, count, gap, last_pub = next_ns, 0, 0, None
        while not self.stop_event.is_set():
            next_ns += period_ns
            delay = next_ns-time.monotonic_ns()
            if delay > 0:
                if self.stop_event.wait(delay*1e-9):
                    break
            elif -delay > 10*period_ns:
                next_ns = time.monotonic_ns()  # Never burst old commands to catch up.
            try:
                published = self.tick()
            except Exception as exc:
                if not self.stop_event.is_set() and rclpy.ok(context=self.context):
                    self.get_logger().error(f'publisher stopped: {exc}')
                self.stop_event.set()
                break
            now = time.monotonic_ns()
            if published:
                if last_pub is not None:
                    gap = max(gap, now-last_pub)
                last_pub, count = now, count+1
            else:
                last_pub = None
            if now-start >= 1_000_000_000:
                self.get_logger().info(
                    f'[publish] {count*1e9/(now-start):.1f} Hz max_gap={gap/1000:.0f}us '
                    f'seq={self.seq} motion={self.motion} fault={self.fault}')
                start, count, gap = now, 0, 0

    def report_hal(self):
        # An accidental second B publisher must not race the same HAL sequence stream.
        if self.count_publishers('/hal/final_command') > 1 and not self.stop_event.is_set():
            self.get_logger().error('another B publisher exists; stopping this mock')
            self.stop_event.set()
        with self.lock:
            count, state = self.state_count, self.last_state
            errors, error = dict(self.error_counts), self.last_error
            self.state_count = 0
            self.error_counts.clear()
        if state is None:
            line = '[C] no /hal/state yet'
        else:
            line = (f'[C] {count} states/s {HAL_STATE_NAMES.get(state.hal_state)} '
                    f'stale={state.stale} session={state.active_session_id} '
                    f'last_seq={state.last_command_seq} '
                    f'result={CMD_RESULT_NAMES.get(state.last_command_result)} '
                    f'errors={list(state.active_errors)}')
            if state.active_session_id != self.session_id:
                line += f' | session mismatch: start B/C with same id (B={self.session_id})'
        if errors:
            line += f' | events={errors} last={error.detail}'
        self.get_logger().info(line)

    def destroy_node(self):
        self.stop_event.set()
        self.thread.join(timeout=1.)
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = FakeB()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
