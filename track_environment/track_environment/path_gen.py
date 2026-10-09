"""
Path generator node: loads real racetrack waypoints from a CSV file
and publishes nav_msgs/Path to /path and track boundary markers to /track_bounds.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from track_environment.track import Track, DEFAULT_TRACK_FILE


class PathGenerator(Node):
    def __init__(self):
        super().__init__('PathGenerator')
        self.get_logger().info('Initializing Path Generator Node...')

        # Declare ROS parameters
        self.declare_parameter('track_file', DEFAULT_TRACK_FILE)
        self.declare_parameter('trajectory_type', 'centerline')
        self.declare_parameter('close_loop', True)

        track_file = str(self.get_parameter('track_file').value)
        trajectory_type = str(self.get_parameter('trajectory_type').value)
        close_loop = bool(self.get_parameter('close_loop').value)

        # Load racetrack waypoints
        try:
            self.track = Track(
                track_file=track_file,
                trajectory_type=trajectory_type,
                close_loop=close_loop
            )
            self.get_logger().info(
                f"Loaded track '{track_file}' [{trajectory_type}]: "
                f"{len(self.track.waypoints)} waypoints, "
                f"total length: {self.track.total_length:.2f} m"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load track '{track_file}': {e}")
            raise

        # Publishers
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.bounds_pub = self.create_publisher(MarkerArray, '/track_bounds', 10)

        # Pre-build path and boundary messages
        self.path_msg = self.build_path_msg()
        self.bounds_msg = self.build_bounds_msg()

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def build_path_msg(self):
        """Converts track waypoints into nav_msgs/Path with valid yaw quaternions."""
        path = Path()
        path.header.frame_id = 'map'
        now = self.get_clock().now().to_msg()
        path.header.stamp = now

        for wp in self.track.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = now
            pose.pose.position.x = wp['x']
            pose.pose.position.y = wp['y']
            pose.pose.position.z = 0.0

            # Compute orientation quaternion from yaw (psi)
            half_yaw = wp['psi'] / 2.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)

            path.poses.append(pose)

        return path

    def build_bounds_msg(self):
        """Converts CSV track boundary markers to MarkerArray."""
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        for m_data in self.track.trackbounds_markers:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = 'track_bounds'
            marker.id = int(m_data.get('id', 0))
            marker.type = int(m_data.get('type', Marker.SPHERE))
            marker.action = Marker.ADD

            pos = m_data.get('pose', {}).get('position', {})
            marker.pose.position.x = float(pos.get('x', 0.0))
            marker.pose.position.y = float(pos.get('y', 0.0))
            marker.pose.position.z = float(pos.get('z', 0.0))
            marker.pose.orientation.w = 1.0

            scale = m_data.get('scale', {})
            marker.scale.x = float(scale.get('x', 0.08))
            marker.scale.y = float(scale.get('y', 0.08))
            marker.scale.z = float(scale.get('z', 0.08))

            color = m_data.get('color', {})
            marker.color.r = float(color.get('r', 0.8))
            marker.color.g = float(color.get('g', 0.2))
            marker.color.b = float(color.get('b', 0.2))
            marker.color.a = float(color.get('a', 0.8))

            marker_array.markers.append(marker)

        return marker_array

    def timer_callback(self):
        """Publishes path and track bounds with updated timestamp."""
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        self.path_pub.publish(self.path_msg)

        if self.bounds_msg.markers:
            for marker in self.bounds_msg.markers:
                marker.header.stamp = now
            self.bounds_pub.publish(self.bounds_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
