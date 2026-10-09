import math
import numpy as np
import rclpy
from bicycle_sim.bicycle_model import Car


def test_bicycle_kinematics_straight():
    """Verify straight line motion with zero steering angle."""
    rclpy.init()
    try:
        car = Car(xInitial=[0.0, 0.0, 0.0, 5.0], dt=0.1, wheelbase_length=1.25)
        car.u[0] = 0.0
        car.u[1] = 0.0
        car.update_x_dot()
        assert math.isclose(car.x_dot[0], 5.0, rel_tol=1e-3)
        assert math.isclose(car.x_dot[1], 0.0, abs_tol=1e-4)
        assert math.isclose(car.x_dot[2], 0.0, abs_tol=1e-4)
    finally:
        car.destroy_node()
        rclpy.shutdown()


def test_bicycle_kinematics_turning():
    """Verify yaw rate when steering angle is non-zero."""
    rclpy.init()
    try:
        car = Car(xInitial=[0.0, 0.0, 0.0, 5.0], dt=0.1, wheelbase_length=1.25)
        delta = math.radians(10.0)
        car.u[0] = 0.0
        car.u[1] = delta
        car.update_x_dot()
        expected_psi_dot = 5.0 / 1.25 * math.tan(delta)
        assert math.isclose(car.x_dot[2], expected_psi_dot, rel_tol=1e-3)
    finally:
        car.destroy_node()
        rclpy.shutdown()


def test_bicycle_acceleration_and_drag():
    """Verify longitudinal acceleration with drag and rolling resistance."""
    rclpy.init()
    try:
        car = Car(xInitial=[0.0, 0.0, 0.0, 4.0], dt=0.1, wheelbase_length=1.25)
        car.u[0] = 0.5  # 50% throttle
        car.u[1] = 0.0
        car.update_x_dot()
        # v_dot = k_a * u_th - (c_drag * v^2 + c_roll * v)
        #       = 4.0 * 0.5 - (0.005 * 16 + 0.05 * 4) = 2.0 - (0.08 + 0.2) = 1.72
        expected_v_dot = 4.0 * 0.5 - (0.005 * 16.0 + 0.05 * 4.0)
        assert math.isclose(car.x_dot[3], expected_v_dot, rel_tol=1e-3)
    finally:
        car.destroy_node()
        rclpy.shutdown()


def test_bicycle_euler_integration():
    """Verify Forward Euler state update, heading wrapping, and speed clamping."""
    rclpy.init()
    try:
        car = Car(xInitial=[0.0, 0.0, 3.10, 0.5], dt=0.1, wheelbase_length=1.25)
        car.x_dot = np.array([1.0, 0.0, 0.5, -10.0], dtype=np.float64)
        car.update_x()

        # x position advanced by 1.0 * 0.1 = 0.1
        assert math.isclose(car.x[0], 0.1, abs_tol=1e-4)

        # Heading 3.10 + 0.05 = 3.15 wraps around pi to negative angle
        assert -math.pi <= car.x[2] < math.pi

        # Deceleration clamped to 0.0 m/s (no reversing)
        assert car.x[3] == 0.0
    finally:
        car.destroy_node()
        rclpy.shutdown()
