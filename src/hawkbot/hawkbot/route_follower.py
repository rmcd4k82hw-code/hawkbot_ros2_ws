import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import rclpy.duration
import math
import json
import os

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class RouteFollower(Node):
    def __init__(self):
        super().__init__('route_follower')

        # Parameters
        self.declare_parameter('robot_ns', 'robot1')
        self.declare_parameter('lookahead_distance', 0.15)
        self.declare_parameter('max_linear_speed', 0.09)
        self.declare_parameter('max_angular_speed', 0.75)
        default_route_file = os.path.expanduser('~/ROS2_WS/src/hawkbot/map/recorded_route.json')
        self.declare_parameter('route_file', default_route_file)

        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.route_file = self.get_parameter('route_file').get_parameter_value().string_value

        self.frame_prefix = f"{self.robot_ns}/" if self.robot_ns else ""

        self.is_following = False
        self.localization_ok = True
        self.route_waypoints = []
        self.current_waypoint_idx = 0
        self.first_run = True

        # Low-Pass Filter for TF pose smoothing
        self.rx_filt = None
        self.ry_filt = None

        # TF Buffer (capped at 10s memory cache)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Services
        self.srv_start = self.create_service(Trigger, 'route_follower/start', self.handle_start)
        self.srv_stop = self.create_service(Trigger, 'route_follower/stop', self.handle_stop)

        # Subscribers
        self.loc_sub = self.create_subscription(
            Bool,
            'localization_ok',
            self.loc_callback,
            10
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, 'target_waypoint_marker', 10)

        # Control Loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f"Route Follower node initialized for namespace '{self.robot_ns}'")

    def loc_callback(self, msg):
        self.localization_ok = msg.data

    def handle_start(self, request, response):
        if not os.path.exists(self.route_file):
            response.success = False
            response.message = f"Impossible de trouver le fichier de parcours '{self.route_file}'. Enregistrez un tracé d'abord !"
            return response

        try:
            with open(self.route_file, 'r') as f:
                self.route_waypoints = json.load(f)
        except Exception as e:
            response.success = False
            response.message = f"Erreur de lecture du fichier JSON: {e}"
            return response

        if not self.route_waypoints:
            response.success = False
            response.message = "Le fichier de parcours est vide !"
            return response

        self.is_following = True
        self.current_waypoint_idx = 0
        self.first_run = True
        self.rx_filt = None
        self.ry_filt = None
        response.success = True
        response.message = f"Suivi du parcours bleu démarré pour {self.robot_ns} ({len(self.route_waypoints)} waypoints)."
        self.get_logger().info(response.message)
        return response

    def clear_target_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.robot_ns}_target"
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)

    def handle_stop(self, request, response):
        self.is_following = False
        self.stop_robot()
        self.clear_target_marker()
        response.success = True
        response.message = f"Suivi de parcours arrêté pour {self.robot_ns}."
        self.get_logger().info(response.message)
        return response

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def control_loop(self):
        if not self.is_following:
            return

        # Safety pause if localization is lost
        if not self.localization_ok:
            self.stop_robot()
            self.get_logger().warn(f"PAUSE SÉCURITÉ [{self.robot_ns}]: Localisation perdue. En attente...", throttle_duration_sec=2.0)
            return

        if not self.route_waypoints:
            self.stop_robot()
            return

        # Lookup robot TF in map frame
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                f'{self.frame_prefix}base_footprint',
                now,
                timeout=rclpy.duration.Duration(seconds=0.08)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed in RouteFollower: {e}", throttle_duration_sec=2.0)
            return

        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
        ryaw = quaternion_to_yaw(trans.transform.rotation)

        # Apply Low-Pass Filter (alpha=0.25) to absorb Cartographer optimization jumps
        if self.rx_filt is None or self.ry_filt is None:
            self.rx_filt = rx
            self.ry_filt = ry
        else:
            self.rx_filt = 0.25 * rx + 0.75 * self.rx_filt
            self.ry_filt = 0.25 * ry + 0.75 * self.ry_filt

        rx_use = self.rx_filt
        ry_use = self.ry_filt

        n = len(self.route_waypoints)

        # 1. Find nearest waypoint index on the route
        min_dist = float('inf')
        nearest_idx = self.current_waypoint_idx

        # If first iteration after start, search ALL waypoints to lock onto nearest loop point
        if self.first_run:
            for idx, wp in enumerate(self.route_waypoints):
                d = math.hypot(wp['x'] - rx_use, wp['y'] - ry_use)
                if d < min_dist:
                    min_dist = d
                    nearest_idx = idx
            self.first_run = False
            self.get_logger().info(f"[{self.robot_ns}] Initialized route target to nearest waypoint #{nearest_idx} (dist: {min_dist:.2f}m)")
        else:
            # Search in a forward window of 30 waypoints to prevent backward jumping
            search_window = min(30, n)
            for offset in range(search_window):
                idx = (self.current_waypoint_idx + offset) % n
                wp = self.route_waypoints[idx]
                d = math.hypot(wp['x'] - rx_use, wp['y'] - ry_use)
                if d < min_dist:
                    min_dist = d
                    nearest_idx = idx

        self.current_waypoint_idx = nearest_idx

        # 2. Find target waypoint at short lookahead distance Ld (8 cm) for high precision tracking
        target_idx = nearest_idx
        accumulated_dist = 0.0
        # Target a very short lookahead so the robot strictly hits every waypoint within 1-2 cm
        while accumulated_dist < self.lookahead_distance:
            next_idx = (target_idx + 1) % n
            wp_a = self.route_waypoints[target_idx]
            wp_b = self.route_waypoints[next_idx]
            accumulated_dist += math.hypot(wp_b['x'] - wp_a['x'], wp_b['y'] - wp_a['y'])
            target_idx = next_idx

        target_wp = self.route_waypoints[target_idx]

        # 3. Transform target waypoint to robot local frame
        dx = target_wp['x'] - rx_use
        dy = target_wp['y'] - ry_use
        cos_yaw = math.cos(-ryaw)
        sin_yaw = math.sin(-ryaw)

        x_local = dx * cos_yaw - dy * sin_yaw
        y_local = dx * sin_yaw + dy * cos_yaw
        Ld = math.hypot(x_local, y_local)

        if Ld < 0.01:
            Ld = 0.01

        # 4. Pure Pursuit Curvature & Control (Strict Forward Motion - No Backtracking)
        gamma = (2.0 * y_local) / (Ld * Ld)
        angle_err = math.atan2(y_local, x_local)

        twist = Twist()
        
        # Pivot smoothly on the spot if heading error is > 23 degrees
        if abs(angle_err) > 0.4:
            twist.linear.x = 0.0
            twist.angular.z = self.max_angular_speed * 0.7 if angle_err > 0 else -self.max_angular_speed * 0.7
        else:
            speed_factor = math.cos(angle_err)
            twist.linear.x = self.max_linear_speed * max(0.3, speed_factor)
            twist.angular.z = twist.linear.x * gamma

            # Clamp velocities strictly to positive forward speeds
            twist.linear.x = min(max(twist.linear.x, 0.0), self.max_linear_speed)
            twist.angular.z = min(max(twist.angular.z, -self.max_angular_speed), self.max_angular_speed)

        self.cmd_pub.publish(twist)

        # 5. Publish target waypoint marker to RViz
        self.publish_target_marker(target_wp['x'], target_wp['y'])

    def publish_target_marker(self, tx, ty):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.robot_ns}_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 0.0
        marker.color.g = 1.0  # Bright Lime Green Target Sphere
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.pose.position.x = float(tx)
        marker.pose.position.y = float(ty)
        marker.pose.position.z = 0.1
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = RouteFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
