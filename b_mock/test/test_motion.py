"""Check smooth startup/turns and the exact fault TTL without needing a ROS graph."""
import math

from b_mock.fake_b_node import EXPIRE_NS, expired_times, planned_position
import pytest

ANCHOR = (.31, -.01, .59)


@pytest.mark.parametrize('t', [0., 3., 6., 20.])
def test_hold_uses_measured_anchor(t):
    assert planned_position('hold', t, ANCHOR) == ANCHOR


@pytest.mark.parametrize('motion', ['sine', 'line'])
def test_smooth_motion_starts_and_returns_to_anchor(motion):
    assert planned_position(motion, 0., ANCHOR) == ANCHOR
    assert planned_position(motion, 6., ANCHOR) == ANCHOR
    peak = planned_position(motion, 3., ANCHOR)
    assert peak == pytest.approx((ANCHOR[0], ANCHOR[1]+.002, ANCHOR[2]))


@pytest.mark.parametrize('endpoint', [0., 3., 6.])
def test_line_reverses_without_velocity_or_acceleration_jump(endpoint):
    h = 1e-4

    def f(t):
        return planned_position('line', t, ANCHOR)[1]
    velocity = (f(endpoint+h)-f(endpoint-h))/(2*h)
    acceleration = (f(endpoint+h)-2*f(endpoint)+f(endpoint-h))/h**2
    assert abs(velocity) < 1e-8
    assert abs(acceleration) < 1e-5


def test_expired_fault_keeps_five_ms_ttl():
    now = 1_000_000_000
    created, expires = expired_times(now)
    assert 0 <= created < expires < now
    assert expires-created == EXPIRE_NS


def test_archive_absolute_positions_remain_opt_in():
    assert planned_position('hold', 0., ANCHOR, absolute=True) == (.5, 0., .2)
    assert planned_position('sine', 1., ANCHOR, absolute=True) == pytest.approx((.5, .05, .2))
    assert all(math.isfinite(v) for v in planned_position('line', 20., ANCHOR, absolute=True))
