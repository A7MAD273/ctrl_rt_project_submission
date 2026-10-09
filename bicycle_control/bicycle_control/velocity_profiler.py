"""
Target Velocity Profiler based on track curvature.
Calculates maximum safe cornering speeds subject to lateral acceleration limits.
"""

import math  # noqa: F401


class VelocityProfiler:
    """Generates target speed profiles based on track curvature or precomputed data."""

    def __init__(self, default_speed=4.0, max_speed=8.0, max_lat_accel=5.0):
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.max_lat_accel = max_lat_accel

    def compute_target_speed(self, kappa, fallback_speed=None):
        """Calculates curvature-limited velocity: v_max = sqrt(a_lat_max / |kappa|)."""
        # TODO: Milestone 5.1 — Curvature-Limited Velocity Profiler
        # This controls how fast the car drives based on the road shape.
        # It slows the car down in sharp turns to prevent slipping.
        # Implement the formula to calculate safe speed from curvature, and clamp it.
        if abs(kappa) < 1e-6:
            return self.max_speed
        return min(self.max_speed, math.sqrt(self.max_lat_accel / abs(kappa)))
