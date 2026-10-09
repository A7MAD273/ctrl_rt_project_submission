from bicycle_control.longitudinal_pid import PIDLongitudinalController


def test_longitudinal_pid_positive_error():
    """Vehicle slower than target speed -> positive throttle."""
    controller = PIDLongitudinalController(kp=1.0, ki=0.1, kd=0.01, dt=0.1)
    u_throttle = controller.compute(target_vel=5.0, current_vel=3.0)
    assert u_throttle > 0.0
    assert u_throttle <= 1.0


def test_longitudinal_pid_negative_error():
    """Vehicle faster than target speed -> negative throttle / braking."""
    controller = PIDLongitudinalController(kp=1.0, ki=0.1, kd=0.01, dt=0.1)
    u_throttle = controller.compute(target_vel=3.0, current_vel=5.0)
    assert u_throttle < 0.0
    assert u_throttle >= -1.0


def test_longitudinal_pid_anti_windup():
    """Sustained error should not cause unbounded integral accumulation."""
    controller = PIDLongitudinalController(kp=1.0, ki=1.0, kd=0.0, dt=0.1, integral_limit=2.0)
    for _ in range(100):
        controller.compute(target_vel=10.0, current_vel=0.0)
    assert controller.integral <= 2.0
    assert controller.integral >= -2.0
