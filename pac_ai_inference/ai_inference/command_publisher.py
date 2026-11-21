import time
import math

from intelion_interfaces.msg import ControlCommand
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

CONTROL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class CommandPublisher:
    """seq, writer_boot_id, layout_version, age_ns를 책임지고 /ai/control_command로 발행한다."""

    def __init__(self, node, topic='/ai/control_command', qos=CONTROL_QOS,
                 confidence_min=0.0, max_age_ns=None):
        if not math.isfinite(confidence_min) or not 0 <= confidence_min <= 1 or (max_age_ns is not None and max_age_ns <= 0):
            raise ValueError('Invalid confidence/age limits')
        self.confidence_min, self.max_age_ns = confidence_min, max_age_ns
        self._pub = node.create_publisher(ControlCommand, topic, qos)
        self.boot_id = time.time_ns() & 0xFFFFFFFF
        self.seq = 1

    def publish(self, command_type, status, target, confidence, capture_mono_ns):
        """실패(status != OK)여도 반드시 호출한다. 이 경우 confidence는 0으로 강제한다."""
        msg = ControlCommand()
        msg.seq = self.seq
        msg.writer_boot_id = self.boot_id
        msg.layout_version = 1
        msg.command_type = int(command_type)
        msg.status = int(status)
        msg.target = [float(v) for v in target]
        msg.confidence = float(confidence) if status == ControlCommand.ST_OK else 0.0
        msg.age_ns = time.monotonic_ns() - int(capture_mono_ns)
        if status == ControlCommand.ST_OK:
            # Reject invalid geometry before it can become a usable correction.
            # Deliberately finite out-of-range fault injection still reaches B.
            if not all(math.isfinite(v) for v in msg.target):
                msg.status = ControlCommand.ST_SENSOR_INVALID
                msg.confidence = 0.0
                msg.target = [0.0] * 4
            if not math.isfinite(confidence) or not self.confidence_min <= confidence <= 1:
                msg.status = ControlCommand.ST_NOT_DETECTED
                msg.confidence = 0.0
            if msg.age_ns < 0 or (self.max_age_ns is not None and msg.age_ns > self.max_age_ns):
                msg.status = ControlCommand.ST_SENSOR_INVALID
                msg.confidence = 0.0
        self._pub.publish(msg)
        self.seq += 1
        return msg
