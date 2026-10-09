import math
import pytest
from bicycle_control.velocity_profiler import VelocityProfiler


def test_velocity_profiler_straight():
    """Near zero curvature should yield maximum configured speed."""
    profiler = VelocityProfiler(default_speed=4.0, max_speed=8.0, max_lat_accel=5.0)
    v_target = profiler.compute_target_speed(kappa=0.0)
    assert v_target == 8.0


def test_velocity_profiler_sharp_turn():
    """Sharp turn should limit speed to sqrt(a_lat / kappa)."""
    profiler = VelocityProfiler(default_speed=4.0, max_speed=8.0, max_lat_accel=5.0)
    kappa = 0.5  # radius = 2.0 m
    v_target = profiler.compute_target_speed(kappa=kappa)
    expected = math.sqrt(5.0 / 0.5)
    assert pytest.approx(v_target, rel=1e-3) == expected
