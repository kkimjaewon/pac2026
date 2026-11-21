import threading
import time
from dataclasses import dataclass, replace

import message_filters
import numpy as np
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

WIDTH, HEIGHT = 640, 480


@dataclass
class CameraInfoLite:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass
class SensorFrame:
    seq: int
    capture_mono_ns: int  # A가 동기화한 RGB/깊이 쌍을 받은 시각. 물리 촬영/전송 지연은 포함하지 않음.
    sim_time_ns: int      # Gazebo header.stamp. 지연 계산에는 쓰지 않음
    rgb: object
    depth: object
    camera_info: object
    depth_valid_ratio: float
    valid: bool
    error_reason: str


class SensorAdapter:
    # 시뮬 시각이 이만큼(1초) 넘게 뒤로 가면 Gazebo를 다시 켠 것으로 보고 기준을 새로 잡는다.
    SIM_RESET_NS = 10**9

    def __init__(self, node, rgb_topic, depth_topic, info_topic, on_frame=None):
        self._lock = threading.Lock()
        self._latest = None
        self._info = None
        self._seq = 0
        self._last_sim_ns = None
        self.sim_resets = 0
        self._on_frame = on_frame
        # 큰 영상을 UDP로 받을 때 best effort는 조각 하나만 빠져도 프레임이 사라진다.
        # 기본은 reliable, 필요하면 -p sensor_reliable:=false 로 되돌린다.
        if not node.has_parameter('sensor_reliable'):
            node.declare_parameter('sensor_reliable', True)
        reliable = bool(node.get_parameter('sensor_reliable').value)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        node.create_subscription(CameraInfo, info_topic, self._on_info, qos)
        rgb_sub = message_filters.Subscriber(node, Image, rgb_topic, qos_profile=qos)
        depth_sub = message_filters.Subscriber(node, Image, depth_topic, qos_profile=qos)
        self._sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.02)
        self._sync.registerCallback(self._on_pair)

    def _on_info(self, msg):
        k = msg.k
        self._info = CameraInfoLite(k[0], k[4], k[2], k[5], msg.width, msg.height)

    def _decode(self, rgb_msg, depth_msg):
        if self._info is None:
            return None, None, 0.0, 'no_camera_info'
        # 해상도는 고정하지 않는다: RGB, 깊이, camera_info가 서로 같은 크기이기만 하면 된다.
        w, h = rgb_msg.width, rgb_msg.height
        if w <= 0 or h <= 0 or depth_msg.width != w or depth_msg.height != h:
            return None, None, 0.0, 'size_mismatch'
        if self._info.width != w or self._info.height != h:
            return None, None, 0.0, 'size_mismatch'
        if rgb_msg.encoding != 'rgb8' or depth_msg.encoding != '32FC1':
            return None, None, 0.0, 'encoding_mismatch'
        if rgb_msg.step != w * 3 or depth_msg.step != w * 4:
            return None, None, 0.0, 'size_mismatch'
        try:
            rgb = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(h, w, 3)
            depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(h, w)
        except ValueError:
            return None, None, 0.0, 'decode_error'
        ratio = float(np.count_nonzero(np.isfinite(depth) & (depth > 0))) / depth.size
        if ratio == 0.0:
            return rgb, depth, ratio, 'all_nan'
        return rgb, depth, ratio, ''

    def _on_pair(self, rgb_msg, depth_msg):
        capture = time.monotonic_ns()
        stamp = depth_msg.header.stamp
        sim_ns = stamp.sec * 10**9 + stamp.nanosec
        rgb, depth, ratio, reason = self._decode(rgb_msg, depth_msg)
        # Gazebo를 다시 켜면 시뮬 시각이 0부터 다시 시작한다. 기준을 새로 잡지 않으면
        # A를 다시 켤 때까지 모든 프레임이 stale로 버려진다.
        if self._last_sim_ns is not None and sim_ns < self._last_sim_ns - self.SIM_RESET_NS:
            self._last_sim_ns = None
            self.sim_resets += 1
        if not reason and self._last_sim_ns is not None and sim_ns <= self._last_sim_ns:
            reason = 'stale'
        if self._last_sim_ns is None or sim_ns > self._last_sim_ns:
            self._last_sim_ns = sim_ns
        frame = SensorFrame(self._seq, capture, sim_ns, rgb, depth, self._info, ratio, reason == '', reason)
        self._seq += 1
        with self._lock:
            self._latest = frame
        if self._on_frame is not None:
            self._on_frame(frame)

    def get_latest(self, max_age_ms=None):
        """최신 프레임 1장. 너무 오래됐으면 valid=False, error_reason='stale_frame'로 돌려준다."""
        with self._lock:
            frame = self._latest
        if frame is None:
            return None
        if max_age_ms is not None and (time.monotonic_ns() - frame.capture_mono_ns) / 1e6 > max_age_ms:
            return replace(frame, valid=False, error_reason='stale_frame')
        return frame
