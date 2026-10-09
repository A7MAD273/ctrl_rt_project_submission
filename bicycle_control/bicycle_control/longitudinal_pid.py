"""
Low-Level Powertrain Cruise Controller (Longitudinal PID).
Regulates vehicle speed via normalized throttle/braking effort.
"""

import numpy as np  # noqa: F401


class PIDLongitudinalController:
    """Low-Level Powertrain Cruise Controller / Electronic Speed Control (ESC).

    Translates high-level velocity requests into normalized throttle/brake effort.
    Because physical vehicles experience friction and speed-squared aerodynamic drag,
    a closed-loop speed regulator is required to maintain target velocity.
    """

    def __init__(self, kp=1.0, ki=0.2, kd=0.05, dt=0.1,
                 max_throttle=1.0, max_brake=1.0, integral_limit=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target_vel, current_vel):
        """Computes normalized throttle/braking effort in [-1.0, 1.0]."""
        # TODO: Milestone 4.1 — Longitudinal PID Speed Control & Anti-Windup
        # This is the speed regulator. Because the car has drag, simply setting
        # a target speed isn't enough — it needs closed-loop control.
        # Implement a PID controller on the velocity error with anti-windup on the integrator.
        
        error = target_vel - current_vel
        self.integral += error * self.dt
        self.integral = float(np.clip(self.integral, -self.integral_limit, self.integral_limit))

        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * derivative
        output = p_term + i_term + d_term
        return float(np.clip(output, -self.max_brake, self.max_throttle))

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral = 0.0
        self.prev_error = 0.0
