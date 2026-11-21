"""A→B 가짜 AI 입력. B 판단을 우회해 HAL을 직접 움직이지 않는다."""
import math
import secrets
import time

from intelion_interfaces.msg import ControlCommand
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class FakeAiNode(Node):
    """실제 AI 대신 정상/실패 입력을 일정 주기로 만드는 개발 전용 publisher."""

    def __init__(self):
        super().__init__('fake_ai_node')
        fixed = ParameterDescriptor(read_only=True)
        self.declare_parameter('mode', 'normal', fixed)
        self.declare_parameter('rate_hz', 15., fixed)
        self.declare_parameter('drop_after_s', 2., fixed)
        self.mode = self.get_parameter('mode').value
        rate = self.get_parameter('rate_hz').value
        self.drop_after = self.get_parameter('drop_after_s').value
        if self.mode not in ('normal', 'not_detected', 'low_confidence', 'stale', 'invalid_number', 'drop'):
            raise ValueError('unknown fake AI mode')
        if not math.isfinite(rate) or not 0 < rate <= 100:
            raise ValueError('rate_hz must be in (0, 100]')
        if not math.isfinite(self.drop_after) or self.drop_after < 0:
            raise ValueError('drop_after_s must be finite and nonnegative')
        # 재시작마다 boot id를 바꿔 B가 seq=0을 이전 실행의 역순 메시지로 오해하지 않게 한다.
        self.boot = secrets.randbits(32) or 1
        self.seq = 0
        self.started = time.monotonic()
        self.pub = self.create_publisher(ControlCommand, '/ai/control_command',
                                         QoSProfile(depth=1,
                                                    reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(1./rate, self.publish)
        self.get_logger().info(f'fake AI mode={self.mode}; A→B only, does not command HAL')

    def publish(self):
        elapsed = time.monotonic()-self.started
        if self.mode == 'drop' and elapsed >= self.drop_after:
            return  # publisher는 살아 있지만 입력이 멈춘 상황: B의 receive timeout 시험용.
        capture_ns = time.monotonic_ns()
        msg = ControlCommand()
        msg.seq, msg.writer_boot_id, msg.layout_version = self.seq, self.boot, 1
        self.seq += 1
        msg.command_type, msg.status = ControlCommand.CMD_POSITION, ControlCommand.ST_OK
        # target은 4원소 계약의 synthetic 값일 뿐이다. B의 좌표/경로 처리 후 HalCommand가 된다.
        # v0.3 문서의 position 계약은 [x,y,z,0], layout_version은 문서 버전과 달리 현재 1이다.
        msg.target = [.3069, 0., .5903 + .001*math.sin(elapsed), 0.]
        msg.confidence = .95
        msg.age_ns = time.monotonic_ns()-capture_ns  # A 내부 경과시간: 다른 PC 절대시각을 보내지 않는다.
        if self.mode == 'not_detected':
            msg.status, msg.confidence = ControlCommand.ST_NOT_DETECTED, 0.
        elif self.mode == 'low_confidence':
            msg.confidence = .1
        elif self.mode == 'stale':
            msg.age_ns = 250_000_000  # 200ms AI TTL보다 오래된 결과: B가 폐기하는지 확인한다.
        elif self.mode == 'invalid_number':
            msg.target[0] = float('nan')  # 형식 오류를 의도적으로 만들며 정상 입력으로 사용하지 않는다.
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FakeAiNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
