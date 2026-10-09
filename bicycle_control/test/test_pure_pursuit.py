import math
import pytest
from bicycle_control.pure_pursuit import PurePursuitController


def test_pure_pursuit_straight():
    """Target waypoint directly ahead on x-axis -> zero steering."""
    pp = PurePursuitController(wheelbase=1.25, kv=0.2, l_min=0.8, l_max=2.5)
    delta = pp.compute_steering(x=0.0, y=0.0, yaw=0.0, target_pt=(1.5, 0.0), lookahead=1.5)
    assert pytest.approx(delta, abs=1e-4) == 0.0


def test_pure_pursuit_turn_left():
    """Target waypoint to the left (y > 0) -> positive steering (left)."""
    pp = PurePursuitController(wheelbase=1.25, kv=0.2, l_min=0.8, l_max=2.5)
    lookahead = math.sqrt(2.0)
    delta = pp.compute_steering(x=0.0, y=0.0, yaw=0.0, target_pt=(1.0, 1.0), lookahead=lookahead)
    assert delta > 0.0
