"""SensorAdapter: Gazebo를 다시 켜서 시뮬 시각이 되돌아가도 프레임을 받는지."""
import threading
from types import SimpleNamespace

import numpy as np

from ai_inference.sensor_adapter import CameraInfoLite, SensorAdapter

W, H = 4, 4


def make_adapter():
    a = object.__new__(SensorAdapter)
    a._lock = threading.Lock()
    a._latest = None
    a._info = CameraInfoLite(100., 100., 2., 2., W, H)
    a._seq = 0
    a._last_sim_ns = None
    a.sim_resets = 0
    a._on_frame = None
    return a


def pair(sim_s):
    sec = int(sim_s)
    nanosec = int(round((sim_s - sec) * 1e9))
    hdr = SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec))
    rgb = SimpleNamespace(width=W, height=H, encoding='rgb8', step=W * 3, header=hdr,
                          data=np.zeros((H, W, 3), np.uint8).tobytes())
    depth = SimpleNamespace(width=W, height=H, encoding='32FC1', step=W * 4, header=hdr,
                            data=np.ones((H, W), np.float32).tobytes())
    return rgb, depth


def feed(a, sim_s):
    a._on_pair(*pair(sim_s))
    return a._latest


def test_increasing_frames_are_valid():
    a = make_adapter()
    assert feed(a, 10.0).valid
    assert feed(a, 10.1).valid
    assert a.sim_resets == 0


def test_duplicate_and_slightly_older_frames_stay_stale():
    a = make_adapter()
    feed(a, 10.0)
    for t in (10.0, 9.5):
        f = feed(a, t)
        assert not f.valid and f.error_reason == 'stale'
    assert a.sim_resets == 0


def test_sim_time_reset_restarts_baseline():
    a = make_adapter()
    feed(a, 100.0)
    f = feed(a, 0.5)
    assert f.valid and a.sim_resets == 1
    assert feed(a, 0.6).valid
    assert feed(a, 0.55).error_reason == 'stale'
