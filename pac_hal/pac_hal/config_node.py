"""Interface-only gateway. Never creates robot commands or claims READY."""
import csv
from pathlib import Path
import time

from intelion_interfaces.msg import HalCommand, HalError, HalState
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .contract import CommandValidator, load_config, validate_config


class HalConfigNode(Node):
    def __init__(self):
        super().__init__('hal_command_gateway')
        ro = ParameterDescriptor(read_only=True)
        self.declare_parameter('config_file', '', ro)
        self.declare_parameter('session_id', 0, ro)
        self.declare_parameter('run_id', '', ro)
        self.declare_parameter('log_directory', '', ro)
        c = load_config(self.get_parameter('config_file').value)
        c['runtime']['active_session_id'] = self.get_parameter('session_id').value
        for parameter, key in (('run_id', 'run_id'), ('log_directory', 'directory')):
            value = self.get_parameter(parameter).value
            if value:
                c['logging'][key] = value
        self.config = validate_config(c)
        self.validator = CommandValidator(c)
        self.active_errors = set()
        self.last_command_seq = 0
        self.last_command_result = HalState.CMD_REJECTED
        latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=DurabilityPolicy.VOLATILE)
        events = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.VOLATILE)
        self.state_pub = self.create_publisher(HalState, c['topics']['state'], latest)
        self.error_pub = self.create_publisher(HalError, c['topics']['error'], events)
        self.sub = self.create_subscription(
            HalCommand, c['topics']['command'], self.on_command, latest)
        self.timer = self.create_timer(1.0/c['runtime']['state_heartbeat_hz'], self.on_state)
        self.log_file = None
        self.writer = None
        if c['logging']['enabled']:
            folder = Path(c['logging']['directory']).expanduser()/c['logging']['run_id']
            folder.mkdir(parents=True, exist_ok=True)
            # Avoid silently mixing two robots/runs; opening existing logs fails clearly.
            self.log_file = (folder/'hal.csv').open('x', newline='', buffering=1)
            self.writer = csv.writer(self.log_file)
            self.writer.writerow(['session_id', 'command_seq', 't1_mono_ns', 't2_mono_ns',
                                  't3_mono_ns', 't4_mono_ns', 't5_mono_ns', 't6_mono_ns',
                                  'result', 'error_code'])
        self.get_logger().info(
            f"HAL backend={c['robot']['execution_backend']}: {c['robot']['model']}, "
            f"session={c['runtime']['active_session_id']}; "
            'timing values are configurations, not benchmarks.')

    def error_transition(self, code, active, command, detail):
        previous = code in self.active_errors
        if previous == active:
            return
        if active:
            self.active_errors.add(code)
        else:
            self.active_errors.discard(code)
        msg = HalError()
        msg.header = command.header
        msg.code, msg.active = code, active
        msg.event_mono_ns = time.monotonic_ns()
        msg.session_id, msg.command_seq = command.session_id, command.command_seq
        msg.source, msg.detail = 'gateway', detail
        self.error_pub.publish(msg)

    def on_command(self, msg):
        received = time.monotonic_ns()
        p, q = msg.target_pose.position, msg.target_pose.orientation
        command = {
            'session_id': msg.session_id, 'command_seq': msg.command_seq,
            'created_mono_ns': msg.created_mono_ns, 'expires_mono_ns': msg.expires_mono_ns,
            'mode': msg.mode, 'frame_id': msg.header.frame_id,
            'position': [p.x, p.y, p.z], 'orientation': [q.x, q.y, q.z, q.w],
        }
        code, detail = self.validator.check(command, received)
        validated = time.monotonic_ns()
        if code == 0 and validated >= msg.expires_mono_ns:
            code, detail = HalError.E_EXPIRED, 'expired after validation; never sent'
        if code == 0:
            code = HalError.E_UNSUPPORTED
            detail = 'interface_only: IK/robot transport not implemented'
        if msg.session_id == self.config['runtime']['active_session_id']:
            self.last_command_seq = max(self.last_command_seq, msg.command_seq)
        self.last_command_result = HalState.CMD_REJECTED
        # Reconcile active errors: event per transition, not one event per 1ms command.
        for old in tuple(self.active_errors):
            if old != code:
                self.error_transition(old, False, msg, 'condition no longer present')
        self.error_transition(code, True, msg, detail)
        if self.writer:
            # t3..t6 did NOT happen. Leave empty rather than inventing timing/ACK data.
            self.writer.writerow([msg.session_id, msg.command_seq, received, validated,
                                  '', '', '', '', 'rejected', code])

    def on_state(self):
        msg = HalState()
        msg.header.frame_id = self.config['frames']['reference']
        msg.hal_state = HalState.HAL_INITIALIZING
        # No robot samples: stamp, state_seq and received_mono_ns remain zero.
        msg.stale = True
        msg.pose_valid = False
        msg.twist_valid = False
        msg.active_session_id = self.config['runtime']['active_session_id']
        msg.last_command_seq = self.last_command_seq
        msg.last_command_result = self.last_command_result
        msg.active_errors = sorted(self.active_errors)
        self.state_pub.publish(msg)

    def destroy_node(self):
        if self.log_file:
            self.log_file.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HalConfigNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
