import pytest
from bicycle_control.mpc import KinematicBicycleMPC


def test_mpc_straight_tracking():
    """Vehicle tracking a straight reference path."""
    mpc = KinematicBicycleMPC(wheelbase=1.25, dt=0.1, horizon=5)
    # Straight path reference along x
    ref_traj = [[(k + 1) * 0.4, 0.0, 0.0, 4.0] for k in range(5)]
    x0 = [0.0, 0.0, 0.0, 4.0]
    delta, throttle = mpc.solve(x0, ref_traj, current_steer=0.0)
    assert pytest.approx(delta, abs=0.05) == 0.0
