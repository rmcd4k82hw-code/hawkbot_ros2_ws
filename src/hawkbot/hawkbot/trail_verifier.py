import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import rclpy.duration
import math
import numpy as np
import json
import os

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quaternion(yaw):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

class TrailVerifier(Node):
    def __init__(self):
        super().__init__('trail_verifier')

        self.declare_parameter('robot_ns', 'robot1')
        self.declare_parameter('route_file', '/home/hawkbot/ROS2_WS/src/hawkbot/map/recorded_route.json')
        self.declare_parameter('max_allowed_distance_shrink', 0.15) # Max 15% distance closing allowed

        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.route_file = self.get_parameter('route_file').get_parameter_value().string_value
        self.max_shrink = self.get_parameter('max_allowed_distance_shrink').get_parameter_value().double_value

        self.frame_prefix = f"{self.robot_ns}/" if self.robot_ns else ""

        # Disabled by default! Only activates after option 2 (trajectory save)
        self.is_active = False
        self.route_waypoints = [] # Full waypoints with wall_dists signatures
        self.map_data = None

        # TF Buffer (capped at 10s memory cache)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Services to enable/disable trail verifier
        self.srv_enable = self.create_service(Trigger, 'trail_verifier/enable', self.handle_enable)
        self.srv_disable = self.create_service(Trigger, 'trail_verifier/disable', self.handle_disable)

        # Publishers for pose correction
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, f'/{self.robot_ns}/initialpose' if self.robot_ns else 'initialpose', 10)
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, f'/{self.robot_ns}/set_pose' if self.robot_ns else 'set_pose', 10)

        # Static Map Subscriber
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/static_map', self.map_callback, map_qos)

        # Subscriber to live Lidar scan
        self.scan_sub = self.create_subscription(LaserScan, f'/{self.robot_ns}/scan', self.scan_callback, qos_profile_sensor_data)

        self.last_correction_time = self.get_clock().now()

        self.get_logger().info(f"Trail Verifier (Strict Wall Distance Guard <= {int(self.max_shrink*100)}%) initialized for '{self.robot_ns}'.")

    def map_callback(self, msg):
        self.map_data = msg

    def handle_enable(self, request, response):
        self.load_route_file()
        self.is_active = True
        response.success = True
        response.message = f"[{self.robot_ns}] Trail Verifier & Wall Distance Guard ACTIVÉ ({len(self.route_waypoints)} waypoints)."
        self.get_logger().info(response.message)
        return response

    def handle_disable(self, request, response):
        self.is_active = False
        self.route_waypoints.clear()
        response.success = True
        response.message = f"[{self.robot_ns}] Trail Verifier DÉSACTIVÉ."
        self.get_logger().info(response.message)
        return response

    def load_route_file(self):
        if os.path.exists(self.route_file):
            try:
                with open(self.route_file, 'r') as f:
                    self.route_waypoints = json.load(f)
                self.get_logger().info(f"Loaded {len(self.route_waypoints)} waypoints with wall distance signatures from '{self.route_file}'.")
            except Exception as e:
                self.get_logger().error(f"Error reading route file: {e}")

    def measure_live_wall_dists(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        dists = []
        for target_deg in [0, 45, 90, 135, 180, 225, 270, 315]:
            target_rad = math.radians(target_deg)
            mask = (angles >= target_rad - 0.13) & (angles <= target_rad + 0.13) & (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
            r_sector = ranges[mask]
            if len(r_sector) > 0:
                dists.append(float(np.min(r_sector)))
            else:
                dists.append(2.0)
        return dists

    def scan_callback(self, msg):
        if not self.is_active or not self.route_waypoints:
            return

        # Run at 5 Hz (200 ms) for continuous, real-time protection
        time_since_last = (self.get_clock().now() - self.last_correction_time).nanoseconds / 1e9
        if time_since_last < 0.2:
            return

        try:
            now = rclpy.time.Time()
            robot_trans = self.tf_buffer.lookup_transform(
                'map',
                f'{self.frame_prefix}base_footprint',
                now,
                timeout=rclpy.duration.Duration(seconds=0.03)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        rx = robot_trans.transform.translation.x
        ry = robot_trans.transform.translation.y
        ryaw = quaternion_to_yaw(robot_trans.transform.rotation)

        # 1. Find closest recorded waypoint on the blue trail
        closest_wp = None
        min_dist = float('inf')
        for wp in self.route_waypoints:
            d = math.hypot(wp['x'] - rx, wp['y'] - ry)
            if d < min_dist:
                min_dist = d
                closest_wp = wp

        if closest_wp is None or 'wall_dists' not in closest_wp:
            return

        ref_dists = closest_wp['wall_dists']
        live_dists = self.measure_live_wall_dists(msg)

        # 2. Check if live wall distances shrink/close in by > 15% relative to tracing reference
        shrink_violations = []
        for k in range(8):
            ref_d = ref_dists[k]
            live_d = live_dists[k]
            if ref_d > 0.3 and live_d > 0.3:
                ratio = live_d / ref_d
                # Violation: wall closed in by > 15% (ratio < 0.85) or expanded by > 15% (ratio > 1.15)
                if ratio < (1.0 - self.max_shrink) or ratio > (1.0 + self.max_shrink):
                    shrink_violations.append((k, ref_d, live_d, ratio))

        # 3. If wall distances shrink or close in by > 15%, lock & push back pose!
        if len(shrink_violations) >= 2:
            self.get_logger().warn(
                f"[{self.robot_ns}] WALL DISTANCE DRIFT DETECTED! {len(shrink_violations)} sectors violated 15% distance limit. Recalibrating pose..."
            )

            # Calculate translation correction (dx, dy) to restore original recorded wall distances
            dx_total = 0.0
            dy_total = 0.0
            angles_deg = [0, 45, 90, 135, 180, 225, 270, 315]

            for (k, ref_d, live_d, ratio) in shrink_violations:
                ang_rad = ryaw + math.radians(angles_deg[k])
                # Difference between reference and live wall distance
                diff = ref_d - live_d
                dx_total += diff * math.cos(ang_rad)
                dy_total += diff * math.sin(ang_rad)

            dx_corr = (dx_total / len(shrink_violations)) * 0.4
            dy_corr = (dy_total / len(shrink_violations)) * 0.4

            # Publish initialpose and set_pose correction to lock pose within 10-15% of recorded tracing distance
            corr_pose = PoseWithCovarianceStamped()
            corr_pose.header.stamp = self.get_clock().now().to_msg()
            corr_pose.header.frame_id = 'map'
            corr_pose.pose.pose.position.x = rx + dx_corr
            corr_pose.pose.pose.position.y = ry + dy_corr
            corr_pose.pose.pose.position.z = 0.0
            corr_pose.pose.pose.orientation = yaw_to_quaternion(ryaw)

            corr_pose.pose.covariance[0] = 1e-3
            corr_pose.pose.covariance[7] = 1e-3
            corr_pose.pose.covariance[35] = 1e-3

            self.initial_pose_pub.publish(corr_pose)
            self.set_pose_pub.publish(corr_pose)
            self.last_correction_time = self.get_clock().now()

def main(args=None):
    rclpy.init(args=args)
    node = TrailVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
