"""
Integrated Two-Tier Autonomous Vehicle Controller Node.
Coordinates target speed profiling, longitudinal ESC regulation, and lateral path tracking.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32

from bicycle_control.longitudinal_pid import PIDLongitudinalController
from bicycle_control.velocity_profiler import VelocityProfiler
from bicycle_control.lateral_pid import LateralPIDController
from bicycle_control.pure_pursuit import PurePursuitController
from bicycle_control.mpc import KinematicBicycleMPC


class ControllerNode(Node):
    """ROS 2 Node coordinating Two-Tier Autonomous Vehicle Control."""

    def __init__(self):
        super().__init__('controller')
        self.get_logger().info('Initializing Two-Tier Autonomous Vehicle Controller...')

        # Parameters
        # Available control modes: 'lateral_pid', 'pure_pursuit', 'mpc'
        self.declare_parameter('control_mode', 'pure_pursuit')
        self.declare_parameter('target_speed', 4.0)             # m/s base speed
        self.declare_parameter('velocity_mode', 'curvature')    # 'curvature', 'constant'
        self.declare_parameter('wheelbase', 1.25)

        self.control_mode = str(self.get_parameter('control_mode').value).lower()
        self.target_speed = float(self.get_parameter('target_speed').value)
        self.velocity_mode = str(self.get_parameter('velocity_mode').value)
        self.wheelbase = float(self.get_parameter('wheelbase').value)

        # Controllers
        self.pid_longitudinal = PIDLongitudinalController(kp=1.0, ki=0.2, kd=0.05, dt=0.1)
        self.profiler = VelocityProfiler(default_speed=self.target_speed, max_speed=7.5)
        self.lateral_pid = LateralPIDController(kp=0.8, ki=0.02, kd=0.15, k_yaw=0.5, dt=0.1)
        self.pure_pursuit = PurePursuitController(
            wheelbase=self.wheelbase, kv=0.25, l_min=0.8, l_max=2.5
        )
        self.mpc = KinematicBicycleMPC(wheelbase=self.wheelbase, dt=0.1, horizon=10)

        # Publishers (10 Hz rate per assignment specification)
        self.throttle_pub = self.create_publisher(Float32, '/throttle', 10)
        self.steer_pub = self.create_publisher(Float32, '/steer', 10)

        # Subscribers
        self.state_sub = self.create_subscription(Odometry, '/state', self.state_callback, 10)
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)

        # State storage
        self.current_state = None  # (x, y, yaw, v)
        self.path_points = []     # [(x, y, yaw)]
        self.current_steer = 0.0

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Vehicle Controller Active in mode: {self.control_mode.upper()}')

    def state_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x
        self.current_state = (x, y, yaw, v)

    def path_callback(self, msg: Path):
        pts = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            qz = p.pose.orientation.z
            qw = p.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            pts.append((x, y, yaw))
        self.path_points = pts

    def compute_track_errors(self, x, y, yaw):
        """Computes orthogonal CTE, heading error, and local curvature."""
        pts = self.path_points
        n = len(pts)
        if n < 2:
            return 0.0, 0.0, 0.0

        min_d_sq = float('inf')
        nearest_idx = 0
        for i in range(n):
            dx = pts[i][0] - x
            dy = pts[i][1] - y
            d_sq = dx * dx + dy * dy
            if d_sq < min_d_sq:
                min_d_sq = d_sq
                nearest_idx = i

        prev_idx = (nearest_idx - 1) % n
        next_idx = (nearest_idx + 1) % n
        p1 = pts[nearest_idx]
        p2 = pts[next_idx]

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        seg_len_sq = dx * dx + dy * dy

        if seg_len_sq > 1e-6:
            t = max(0.0, min(1.0, ((x - p1[0]) * dx + (y - p1[1]) * dy) / seg_len_sq))
            px = p1[0] + t * dx
            py = p1[1] + t * dy
            dist = math.hypot(x - px, y - py)
            cross = dx * (y - p1[1]) - dy * (x - p1[0])
            cte = math.copysign(dist, cross)
            path_yaw = math.atan2(dy, dx)
        else:
            cte = 0.0
            path_yaw = p1[2]

        heading_err = math.atan2(math.sin(yaw - path_yaw), math.cos(yaw - path_yaw))

        dyaw = pts[next_idx][2] - pts[prev_idx][2]
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        pt_diff = math.hypot(
            pts[next_idx][0] - pts[prev_idx][0],
            pts[next_idx][1] - pts[prev_idx][1]
        )
        ds = max(pt_diff, 1e-3)
        kappa = dyaw / ds

        return cte, heading_err, kappa

    def control_loop(self):
        """Executes selected controller at 10 Hz."""
        if self.current_state is None or len(self.path_points) < 2:
            return

        x, y, yaw, v = self.current_state

        if self.control_mode == 'lateral_pid':
            # Mode A: Lateral PID Benchmark
            cte, heading_err, kappa = self.compute_track_errors(x, y, yaw)
            steer_rad = self.lateral_pid.compute_steering(cte, heading_err)

            target_v = self.target_speed
            if self.velocity_mode == 'curvature':
                target_v = self.profiler.compute_target_speed(
                    kappa, fallback_speed=self.target_speed
                )

            throttle_cmd = self.pid_longitudinal.compute(target_v, v)

        elif self.control_mode == 'mpc':
            # Mode C: Kinematic Bicycle MPC Benchmark
            ref_traj = self.build_mpc_reference(x, y, v, horizon=10)
            steer_rad, throttle_cmd = self.mpc.solve(
                [x, y, yaw, v], ref_traj, current_steer=self.current_steer
            )

        else:
            # Mode B: Geometric Pure Pursuit Benchmark (Default)
            lookahead = self.pure_pursuit.compute_lookahead(v)
            tgt_idx, tgt_pt = self.pure_pursuit.find_target_waypoint(
                x, y, self.path_points, lookahead
            )
            steer_rad = self.pure_pursuit.compute_steering(x, y, yaw, tgt_pt, lookahead)

            target_v = self.target_speed
            if self.velocity_mode == 'curvature':
                prev_pt = self.path_points[(tgt_idx - 1) % len(self.path_points)]
                next_pt = self.path_points[(tgt_idx + 1) % len(self.path_points)]
                dyaw = next_pt[2] - prev_pt[2]
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
                pt_diff = math.hypot(next_pt[0] - prev_pt[0], next_pt[1] - prev_pt[1])
                ds = max(pt_diff, 1e-3)
                kappa = dyaw / ds
                target_v = self.profiler.compute_target_speed(
                    kappa, fallback_speed=self.target_speed
                )

            throttle_cmd = self.pid_longitudinal.compute(target_v, v)

        self.current_steer = float(steer_rad)

        # Publish commands: steering in radians, powertrain throttle in [-1.0, 1.0]
        s_msg = Float32()
        s_msg.data = float(steer_rad)
        self.steer_pub.publish(s_msg)

        t_msg = Float32()
        t_msg.data = float(throttle_cmd)
        self.throttle_pub.publish(t_msg)

    def get_waypoint_at_distance(self, start_idx, distance_ahead):
        """Walks forward along path by distance_ahead and interpolates reference pose."""
        pts = self.path_points
        n = len(pts)
        if n < 2:
            return pts[0] if pts else (0.0, 0.0, 0.0)

        cur_idx = start_idx
        d_acc = 0.0
        while d_acc < distance_ahead:
            next_idx = (cur_idx + 1) % n
            seg = math.hypot(
                pts[next_idx][0] - pts[cur_idx][0],
                pts[next_idx][1] - pts[cur_idx][1]
            )
            if d_acc + seg >= distance_ahead:
                frac = (distance_ahead - d_acc) / max(seg, 1e-4)
                x = pts[cur_idx][0] + frac * (pts[next_idx][0] - pts[cur_idx][0])
                y = pts[cur_idx][1] + frac * (pts[next_idx][1] - pts[cur_idx][1])
                psi = pts[cur_idx][2]
                return (x, y, psi)
            d_acc += seg
            cur_idx = next_idx
            if cur_idx == start_idx:
                break
        return pts[cur_idx]

    def build_mpc_reference(self, x, y, v=4.0, horizon=10):
        """Generates distance-based future reference poses along path for MPC."""
        pts = self.path_points
        n = len(pts)
        if n < 2:
            return []

        min_d = float('inf')
        nearest_idx = 0
        for i in range(n):
            d = (pts[i][0] - x) ** 2 + (pts[i][1] - y) ** 2
            if d < min_d:
                min_d = d
                nearest_idx = i

        speed = max(float(v), 1.0)
        dt = 0.1
        ref = []
        for k in range(horizon):
            dist_ahead = (k + 1) * speed * dt
            px, py, pyaw = self.get_waypoint_at_distance(nearest_idx, dist_ahead)
            ref.append([px, py, pyaw, self.target_speed])
        return ref


def main(args=None):
    rclpy.init(args=args)
    controller = ControllerNode()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
