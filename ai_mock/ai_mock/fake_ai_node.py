import math
import time

import rclpy
from intelion_interfaces.msg import ControlCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)
BAD_TARGET = [9.9, 9.9, 9.9, 0.0]


def make_target(mode, t, base, amp, period):
    """fixed: 고정 위치 / sine: y가 좌우로 흔들리는 철판(시나리오 3 흉내)"""
    x, y, z = base
    if mode == 'sine':
        y = y + amp * math.sin(2.0 * math.pi * t / period)
    return [x, y, z, 0.0]


class FakeAi(Node):
    def __init__(self):
        super().__init__('fake_ai_node')
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('mode', 'fixed')            # fixed | sine
        self.declare_parameter('command_type', 'position')  # position | path_adjust
        self.declare_parameter('fault_mode', 'none')       # none | delay | stop | bad_value | not_detected | sensor_invalid
        self.declare_parameter('fault_delay_ms', 200)
        self.declare_parameter('base_target', [0.5, 0.0, 0.71])
        self.declare_parameter('amp', 0.1)
        self.declare_parameter('period', 8.0)
        self._pub = self.create_publisher(ControlCommand, '/ai/control_command', QOS)
        self.boot_id = time.time_ns() & 0xFFFFFFFF
        self.seq = 0
        self.t0 = time.monotonic()
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
        self.get_logger().info(f'가짜 A 시작 boot_id={self.boot_id}  (파라미터로 모드를 바꿀 수 있음)')

    def tick(self):
        get = self.get_parameter
        capture = time.monotonic_ns()
        fault = str(get('fault_mode').value)
        if fault == 'stop':
            return
        if fault == 'delay':
            time.sleep(float(get('fault_delay_ms').value) / 1000.0)
        ctype = ControlCommand.CMD_POSITION
        if str(get('command_type').value) == 'path_adjust':
            ctype = getattr(ControlCommand, 'CMD_PATH_ADJUST', 2)
        status, conf = ControlCommand.ST_OK, 1.0
        target = make_target(str(get('mode').value), time.monotonic() - self.t0,
                             [float(v) for v in get('base_target').value],
                             float(get('amp').value), float(get('period').value))
        if fault == 'bad_value':
            target = list(BAD_TARGET)
        elif fault == 'not_detected':
            status, conf, target = ControlCommand.ST_NOT_DETECTED, 0.0, [0.0] * 4
        elif fault == 'sensor_invalid':
            status, conf, target = ControlCommand.ST_SENSOR_INVALID, 0.0, [0.0] * 4
        m = ControlCommand()
        m.seq = self.seq
        m.writer_boot_id = self.boot_id
        m.layout_version = 1
        m.command_type = int(ctype)
        m.status = int(status)
        m.target = [float(v) for v in target]
        m.confidence = float(conf)
        m.age_ns = time.monotonic_ns() - capture
        self._pub.publish(m)
        self.seq += 1


def main():
    rclpy.init()
    node = FakeAi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
