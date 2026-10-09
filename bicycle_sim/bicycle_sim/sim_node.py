"""
Simulation executable entry point for the kinematic bicycle plant.
Loads initial vehicle pose from the racetrack environment and spins the simulation node.
"""

import sys
import numpy as np
import rclpy
from bicycle_sim.bicycle_model import Car

try:
    from track_environment.track import Track, DEFAULT_TRACK_FILE
except ImportError:
    Track = None
    DEFAULT_TRACK_FILE = 'centerline_0.csv'


def main(args=None):
    rclpy.init(args=args)

    # Resolve track start pose
    track_file = DEFAULT_TRACK_FILE
    for i, arg in enumerate(sys.argv):
        if arg in ('--track', '-t') and i + 1 < len(sys.argv):
            track_file = sys.argv[i + 1]

    x0, y0, psi0 = 0.0, 0.0, 0.0
    if Track is not None:
        try:
            track = Track(track_file=track_file)
            x0, y0, psi0 = track.start_pose
        except Exception as exc:
            raise RuntimeError(
                f"Failed to load track '{track_file}' for simulator start pose."
            ) from exc

    x_initial = np.array([x0, y0, psi0, 0.0], dtype=np.float64)

    car = Car(xInitial=x_initial, dt=0.1, wheelbase_length=1.25)
    car.get_logger().info(
        f"Spawned car at start pose: x={x0:.3f} m, y={y0:.3f} m, "
        f"yaw={psi0:.3f} rad ({np.degrees(psi0):.1f}°), v=0.0 m/s"
    )

    try:
        rclpy.spin(car)
    except KeyboardInterrupt:
        pass
    finally:
        car.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
