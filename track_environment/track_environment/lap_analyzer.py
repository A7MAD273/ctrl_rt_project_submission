"""
Lap Analyzer Node:
Performance evaluation, real-time telemetry, lap timing, and RViz HUD visualization.
Decoupled observer monitoring /path and /state to compute cross-track error, heading error,
lap times, and dynamic metrics.
"""

import json  # noqa: F401
import math
import numpy as np  # noqa: F401
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Float32  # noqa: F401
from geometry_msgs.msg import Point  # noqa: F401
from visualization_msgs.msg import Marker, MarkerArray


class LapAnalyzer(Node):
    def __init__(self):
        super().__init__('lap_analyzer')
        self.get_logger().info('Initializing Lap Analyzer Node...')

        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.state_sub = self.create_subscription(Odometry, '/state', self.state_callback, 10)

        # Publishers
        self.metrics_pub = self.create_publisher(String, '/lap/metrics', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/lap/visualization', 10)

        # Standardized Plottable Telemetry Publishers (for rqt_plot & PlotJuggler)
        self.cte_pub = self.create_publisher(Float32, '/telemetry/cte', 10)
        self.speed_pub = self.create_publisher(Float32, '/telemetry/speed', 10)
        self.heading_err_pub = self.create_publisher(
            Float32, '/telemetry/heading_err_deg', 10
        )
        self.lap_time_pub = self.create_publisher(Float32, '/telemetry/lap_time', 10)

        # Path storage
        self.path_points = []  # [(x, y, psi)]
        self.path_cum_dist = []
        self.track_length = 0.0
        self.path_received = False

        # State & Timing
        self.start_sim_time = None
        self.last_state_time = None
        self.lap_start_time = None

        # Lap Tracking
        self.lap_count = 0
        self.last_s = 0.0
        self.total_distance = 0.0
        self.last_xy = None

        # Lap Times
        self.current_lap_time = 0.0
        self.last_lap_time = None
        self.best_lap_time = None
        self.lap_times = []

        # Error & Speed Statistics (Per Lap)
        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []

        # Global Statistics
        self.global_ctes = []
        self.global_max_speed = 0.0

        # Current live metrics
        self.current_cte = 0.0
        self.current_heading_err = 0.0
        self.current_speed = 0.0
        self.proj_xy = (0.0, 0.0)

        # Publish periodic summary and HUD at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_telemetry)

    def path_callback(self, msg: Path):
        """Processes received path and precomputes cumulative distance."""
        if self.path_received and len(msg.poses) == len(self.path_points):
            return  # Path already loaded and unchanged

        pts = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            # Extract yaw from quaternion
            qz = p.pose.orientation.z
            qw = p.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            pts.append((x, y, yaw))

        if len(pts) < 2:
            return

        self.path_points = pts
        # Compute cumulative distance
        cum = [0.0]
        for i in range(1, len(pts)):
            d = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            cum.append(cum[-1] + d)

        self.path_cum_dist = cum
        self.track_length = cum[-1]
        self.path_received = True
        self.get_logger().info(
            f"Lap Analyzer: Loaded path with {len(pts)} waypoints, "
            f"perimeter: {self.track_length:.2f} m"
        )

    def state_callback(self, msg: Odometry):
        """Processes vehicle odometry and updates progress, lap timing, and errors."""
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_sim_time is None:
            self.start_sim_time = now_sec
            self.lap_start_time = now_sec

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x

        self.current_speed = v
        self.global_max_speed = max(self.global_max_speed, v)

        # Distance traveled
        if self.last_xy is not None:
            step_d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.total_distance += step_d
        self.last_xy = (x, y)

        if not self.path_received or len(self.path_points) < 2:
            return

        # Find nearest point & projection
        proj_x, proj_y, s, cte, heading_err = self.project_to_path(x, y, yaw)
        self.proj_xy = (proj_x, proj_y)
        self.current_cte = cte
        self.current_heading_err = heading_err

        # Accumulate metrics
        abs_cte = abs(cte)
        self.lap_ctes.append(abs_cte)
        self.lap_heading_errors.append(abs(heading_err))
        self.lap_speeds.append(v)
        self.global_ctes.append(abs_cte)

        self.current_lap_time = now_sec - self.lap_start_time

        # Lap Crossing Detection (s wrapped around track_length while moving forward).
        # e.g., last_s near end (> 70% length) and current s near start (< 30% length).
        if self.track_length > 5.0 and v > 0.1:
            if self.last_s > 0.75 * self.track_length and s < 0.25 * self.track_length:
                # Sub-tick lap time interpolation
                ds_total = (self.track_length - self.last_s) + s
                dt_step = max(now_sec - (self.last_state_time or now_sec), 1e-4)
                frac = (self.track_length - self.last_s) / max(ds_total, 1e-4)
                t_crossing = (self.last_state_time or now_sec) + frac * dt_step

                lap_duration = t_crossing - self.lap_start_time
                self.record_lap_completion(lap_duration, now_sec)
                self.lap_start_time = t_crossing

        self.last_s = s
        self.last_state_time = now_sec

    def project_to_path(self, x, y, yaw):
        """Finds closest segment and projects (x, y) to compute exact orthogonal CTE."""
        pts = self.path_points
        n = len(pts)

        # 1. Find nearest waypoint
        min_dist_sq = float('inf')
        nearest_idx = 0
        for i in range(n):
            dx = pts[i][0] - x
            dy = pts[i][1] - y
            d_sq = dx * dx + dy * dy
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                nearest_idx = i

        # 2. Check candidate segments: (prev, nearest) and (nearest, next)
        best_dist = float('inf')
        best_proj = (pts[nearest_idx][0], pts[nearest_idx][1])
        best_s = self.path_cum_dist[nearest_idx]
        best_seg_yaw = pts[nearest_idx][2]
        best_signed_cte = 0.0

        candidate_segments = [
            ((nearest_idx - 1) % n, nearest_idx),
            (nearest_idx, (nearest_idx + 1) % n)
        ]
        for prev_i, next_i in candidate_segments:
            x1, y1, yaw1 = pts[prev_i]
            x2, y2, _ = pts[next_i]
            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-6:
                continue

            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / seg_len_sq))
            px = x1 + t * dx
            py = y1 + t * dy
            dist = math.hypot(x - px, y - py)

            if dist < best_dist:
                best_dist = dist
                best_proj = (px, py)
                best_s = self.path_cum_dist[prev_i] + t * math.sqrt(seg_len_sq)
                best_seg_yaw = math.atan2(dy, dx)

                # Signed cross track error: positive if car is to the left of path
                cross = dx * (y - y1) - dy * (x - x1)
                best_signed_cte = math.copysign(dist, cross)

        # Heading error in [-pi, pi]
        heading_err = math.atan2(math.sin(yaw - best_seg_yaw), math.cos(yaw - best_seg_yaw))

        return best_proj[0], best_proj[1], best_s, best_signed_cte, heading_err

    def record_lap_completion(self, lap_duration, now_sec):
        """Records finished lap and prints summary."""
        self.lap_count += 1
        self.last_lap_time = lap_duration
        self.lap_times.append(lap_duration)

        if self.best_lap_time is None or lap_duration < self.best_lap_time:
            self.best_lap_time = lap_duration

        # ======================================================================
        # TODO: Lap Performance Analysis & Metrics Aggregation
        #
        # 1. Summary Statistics:
        #    Compute key performance indicators from self.lap_ctes and self.lap_speeds:
        #      - mean_cte: Mean absolute Cross-Track Error (m)
        #      - max_cte: Maximum Cross-Track Error (m)
        #      - rms_cte: Root-Mean-Square Cross-Track Error: sqrt(mean(cte^2)) (m)
        #      - mean_speed: Average speed across the lap (m/s)
        #      - max_speed: Peak instantaneous speed (m/s)
        #
        # 2. Console Summary:
        #    Log a clean, structured terminal banner reporting lap time, best lap,
        #    CTE metrics (Mean, RMS, Max), speed metrics, and total distance.
        #
        # 3. Buffer Reset:
        #    Clear per-lap history buffers (self.lap_ctes, self.lap_heading_errors,
        #    self.lap_speeds) so the next lap starts fresh.
        # ======================================================================
        mean_cte = float(np.mean(self.lap_ctes))
        max_cte = float(np.max(self.lap_ctes))
        rms_cte = float(np.sqrt(np.mean(np.square(self.lap_ctes))))
        mean_speed = float(np.mean(self.lap_speeds))
        max_speed = float(np.max(self.lap_speeds))

        self.get_logger().info(
            f'LAP {self.lap_count}: time {lap_duration:.2f} s | best {self.best_lap_time:.2f} s | '
            f'CTE mean {mean_cte:.3f} max {max_cte:.3f} rms {rms_cte:.3f} m | '
            f'speed mean {mean_speed:.2f} max {max_speed:.2f} m/s | '
            f'distance {self.total_distance:.1f} m'
        )

        all_cte = np.array(self.global_ctes)
        self.get_logger().info(
            f'TABLE ROW | {self.best_lap_time:.2f} | {self.global_max_speed:.2f} | '
            f'{np.mean(all_cte):.3f} | {np.max(all_cte):.3f} | '
            f'{np.sqrt(np.mean(all_cte ** 2)):.3f} | {self.lap_count} laps |'
        )

        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []

    def publish_telemetry(self):
        """Periodically publishes numerical telemetry and RViz visual markers at 10 Hz."""
        # ======================================================================
        # TODO: Telemetry Publishing for Graphing (PlotJuggler / rqt_plot) & Logging
        #
        # 1. Real-Time Numerical Signals (for rqt_plot / PlotJuggler):
        #    Publish individual Float32 messages so students can graph signals live:
        #      - self.cte_pub -> self.current_cte
        #      - self.speed_pub -> self.current_speed
        #      - self.heading_err_pub -> math.degrees(self.current_heading_err)
        #      - self.lap_time_pub -> self.current_lap_time
        #
        # 2. JSON Telemetry Message:
        #    Assemble a telemetry dictionary (lap, current_lap_time, last_lap_time,
        #    best_lap_time, speed, current_cte, rms_cte, heading_err_deg) and publish
        #    it as a serialized JSON String to self.metrics_pub.
        #
        # 3. Visual Telemetry (RViz):
        #    Pass the telemetry dict to self.publish_rviz_markers(telemetry).
        # ======================================================================
        self.cte_pub.publish(Float32(data=float(self.current_cte)))
        self.speed_pub.publish(Float32(data=float(self.current_speed)))
        self.heading_err_pub.publish(Float32(data=float(math.degrees(self.current_heading_err))))
        self.lap_time_pub.publish(Float32(data=float(self.current_lap_time)))

        rms_cte = float(np.sqrt(np.mean(np.square(self.lap_ctes)))) if self.lap_ctes else 0.0
        telemetry = {
            'lap': self.lap_count + 1,
            'current_lap_time': round(self.current_lap_time, 2),
            'last_lap_time': self.last_lap_time,
            'best_lap_time': self.best_lap_time,
            'speed': round(self.current_speed, 2),
            'current_cte': round(self.current_cte, 3),
            'rms_cte': round(rms_cte, 3),
            'heading_err_deg': round(math.degrees(self.current_heading_err), 2),
        }
        self.metrics_pub.publish(String(data=json.dumps(telemetry)))

        # Baseline start-gate visualization hook
        self.publish_rviz_markers(telemetry)

    def publish_rviz_markers(self, telemetry=None):
        """Renders start gate, error whisker, and on-screen HUD text in RViz."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # ----------------------------------------------------------------------
        # Marker 1: Start/Finish Gate Line (Provided for Track Origin Reference)
        # ----------------------------------------------------------------------
        if self.path_points:
            p0 = self.path_points[0]
            gate = Marker()
            gate.header.frame_id = 'map'
            gate.header.stamp = now
            gate.ns = 'start_gate'
            gate.id = 0
            gate.type = Marker.CYLINDER
            gate.action = Marker.ADD
            gate.pose.position.x = p0[0]
            gate.pose.position.y = p0[1]
            gate.pose.position.z = 0.5
            gate.pose.orientation.w = 1.0
            gate.scale.x = 0.1
            gate.scale.y = 1.2
            gate.scale.z = 1.0
            gate.color.r = 0.1
            gate.color.g = 0.9
            gate.color.b = 0.2
            gate.color.a = 0.7
            ma.markers.append(gate)

        # ======================================================================
        # TODO: Custom Real-Time RViz Visualizations & Telemetry HUD
        #
        # Develop live visual feedback to analyze controller tracking performance:
        #
        # Minimum Requirements:
        # 1. Cross-Track Error Whisker (Marker.LINE_STRIP):
        #    - Connect the vehicle rear axle (self.last_xy) to the projected point
        #      on the path (self.proj_xy).
        #    - Style with dynamic color (e.g., green when small, red when drifting)
        #      so tracking deviation is immediately visible.
        #
        # 2. 3D Telemetry HUD Scoreboard (Marker.TEXT_VIEW_FACING):
        #    - Position floating text above the track start or trailing the vehicle.
        #    - Display live lap number, lap time, speed, CTE, and best lap time.
        #
        # Creative / Bonus Ideas (Optional):
        #  - Controller Lookahead Preview: Render a Marker.SPHERE at target waypoint.
        #  - Vehicle Breadcrumbs / Trajectory History: Render a Marker.POINTS trail
        #    color-coded by speed or CTE magnitude.
        #  - Lateral Acceleration Gauge: Render a vertical bar showing cornering load.
        # ======================================================================
        if self.last_xy is not None:
            whisker = Marker()
            whisker.header.frame_id = 'map'
            whisker.header.stamp = now
            whisker.ns = 'cte_whisker'
            whisker.id = 1
            whisker.type = Marker.LINE_STRIP
            whisker.action = Marker.ADD
            whisker.pose.orientation.w = 1.0
            whisker.scale.x = 0.1
            off_track = abs(self.current_cte) > 0.2
            whisker.color.r = 1.0 if off_track else 0.0
            whisker.color.g = 0.0 if off_track else 1.0
            whisker.color.a = 1.0
            whisker.points = [
                Point(x=float(self.last_xy[0]), y=float(self.last_xy[1]), z=0.2),
                Point(x=float(self.proj_xy[0]), y=float(self.proj_xy[1]), z=0.2),
            ]
            ma.markers.append(whisker)

            hud = Marker()
            hud.header.frame_id = 'map'
            hud.header.stamp = now
            hud.ns = 'telemetry_hud'
            hud.id = 2
            hud.type = Marker.TEXT_VIEW_FACING
            hud.action = Marker.ADD
            hud.pose.position.x = float(self.last_xy[0])
            hud.pose.position.y = float(self.last_xy[1])
            hud.pose.position.z = 3.0
            hud.pose.orientation.w = 1.0
            hud.scale.z = 0.8
            hud.color.r = 1.0
            hud.color.g = 1.0
            hud.color.b = 1.0
            hud.color.a = 1.0
            best = f'{self.best_lap_time:.2f}' if self.best_lap_time else '--'
            hud.text = (
                f'Lap {self.lap_count + 1}  {self.current_lap_time:.1f} s\n'
                f'Speed {self.current_speed:.1f} m/s  CTE {self.current_cte:+.2f} m\n'
                f'Best {best} s'
            )
            ma.markers.append(hud)

        self.viz_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    analyzer = LapAnalyzer()
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
