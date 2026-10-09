from bicycle_control.lateral_pid import LateralPIDController


def test_lateral_pid_steer_right():
    """Vehicle left of path (cte > 0) -> negative steering (steer right)."""
    controller = LateralPIDController(kp=1.0, ki=0.0, kd=0.0, k_yaw=0.0, dt=0.1)
    delta = controller.compute_steering(cte=0.5, heading_err=0.0)
    assert delta < 0.0


def test_lateral_pid_steer_left():
    """Vehicle right of path (cte < 0) -> positive steering (steer left)."""
    controller = LateralPIDController(kp=1.0, ki=0.0, kd=0.0, k_yaw=0.0, dt=0.1)
    delta = controller.compute_steering(cte=-0.5, heading_err=0.0)
    assert delta > 0.0
