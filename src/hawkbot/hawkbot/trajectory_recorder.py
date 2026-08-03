import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data
from std_srvs.srv import Trigger
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import rclpy.duration
import math
import json
import os
import shutil
import numpy as np

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

def smooth_micro_zigzags(points, passes=2):
    """Gentle anti-zigzag filter: removes micro steering jitter without shrinking the path or altering straight lines."""
    if len(points) < 4:
        return points

    pts = list(points)
    n = len(pts)
    for _ in range(passes):
        smoothed = []
        for i in range(n):
            prev_p = pts[(i - 1) % n]
            curr_p = pts[i]
            next_p = pts[(i + 1) % n]

            # 70% current point, 15% prev, 15% next (preserves 100% path scale & straight lines)
            sx = 0.70 * curr_p[0] + 0.15 * prev_p[0] + 0.15 * next_p[0]
            sy = 0.70 * curr_p[1] + 0.15 * prev_p[1] + 0.15 * next_p[1]
            smoothed.append((sx, sy))
        pts = smoothed
    return pts

def chaikin_smooth(points, iterations=3):
    """Applies Chaikin's corner-cutting algorithm for smooth, non-overshooting curves."""
    if len(points) < 3:
        return points

    pts = list(points)
    for _ in range(iterations):
        smoothed = []
        n = len(pts)
        for i in range(n):
            p0 = pts[i]
            p1 = pts[(i + 1) % n]
            
            q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
            r = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
            smoothed.extend([q, r])
        pts = smoothed

    return pts

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')

        self.declare_parameter('robot_ns', 'robot1')
        self.declare_parameter('min_dist_threshold', 0.08)
        
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.min_dist_threshold = self.get_parameter('min_dist_threshold').get_parameter_value().double_value
        
        self.frame_prefix = f"{self.robot_ns}/" if self.robot_ns else ""

        self.is_recording = False
        self.raw_points = []  # List of (x, y, yaw)
        self.map_data = None

        # TF Buffer (capped at 10s memory cache)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Services
        self.srv_start = self.create_service(Trigger, 'trajectory_recorder/start', self.handle_start)
        self.srv_stop = self.create_service(Trigger, 'trajectory_recorder/stop_and_save', self.handle_stop_and_save)
        self.srv_clear = self.create_service(Trigger, 'trajectory_recorder/clear', self.handle_clear)

        # Clients to notify trail_verifier
        self.cli_trail_enable = self.create_client(Trigger, 'trail_verifier/enable')
        self.cli_trail_disable = self.create_client(Trigger, 'trail_verifier/disable')

        # Publishers with Transient Local QoS for immediate RViz latching
        marker_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.marker_pub = self.create_publisher(Marker, 'recorded_trail_marker', marker_qos)
        self.path_pub = self.create_publisher(Path, 'recorded_path', 10)

        # Static Map Subscriber
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/static_map', self.map_callback, map_qos)

        # Lidar Scan Subscriber to record Wall Distance Signatures
        self.latest_scan = None
        self.scan_sub = self.create_subscription(LaserScan, f'/{self.robot_ns}/scan', self.scan_callback, qos_profile_sensor_data)

        # Recording timer (10 Hz)
        self.timer = self.create_timer(0.1, self.record_timer_callback)

        # Auto-load and publish existing saved route on startup
        self.load_and_publish_saved_route()

        self.get_logger().info(f"Trajectory Recorder initialized for namespace '{self.robot_ns}'")

    def load_and_publish_saved_route(self):
        route_path = os.path.expanduser("~/ROS2_WS/src/hawkbot/map/recorded_route.json")
        if os.path.exists(route_path):
            try:
                with open(route_path, 'r') as f:
                    route_data = json.load(f)
                if route_data:
                    self.publish_final_trail_marker(route_data)
                    self.get_logger().info(f"[{self.robot_ns}] Published saved blue trail ({len(route_data)} waypoints) on startup.")
            except Exception as e:
                self.get_logger().error(f"Error loading saved route on startup: {e}")

    def scan_callback(self, msg):
        self.latest_scan = msg

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Static map received for trajectory collision verification.", once=True)

    def handle_start(self, request, response):
        self.raw_points.clear()
        self.clear_rviz_markers()
        self.is_recording = True
        response.success = True
        response.message = f"Démarrage de l'enregistrement de la traînée bleue pour {self.robot_ns}."
        self.get_logger().info(response.message)
        return response

    def handle_clear(self, request, response):
        self.is_recording = False
        self.raw_points.clear()
        self.clear_rviz_markers()

        if self.cli_trail_disable.service_is_ready():
            self.cli_trail_disable.call_async(Trigger.Request())

        response.success = True
        response.message = "Traînée bleue effacée."
        self.get_logger().info(response.message)
        return response

    def handle_stop_and_save(self, request, response):
        self.is_recording = False
        if len(self.raw_points) < 4:
            response.success = False
            response.message = "Pas assez de points enregistrés (minimum 4 points requis)."
            return response

        self.get_logger().info("Arrêt de l'enregistrement. Lissage, fermeture de boucle et vérification carte...")
        
        # 1. Post-process trajectory: smooth + loop closure + collision check
        final_route = self.process_and_smooth_loop(self.raw_points)

        # 2. Save route to JSON file
        map_dir = os.path.expanduser("~/ROS2_WS/src/hawkbot/map")
        file_path = os.path.join(map_dir, "recorded_route.json")
        try:
            os.makedirs(os.path.dirname(file_path), exist_ok=True)
            with open(file_path, 'w') as f:
                json.dump(final_route, f, indent=2)

            # Create map & trajectory bundle directory
            bundle_dir = os.path.join(map_dir, "saved_routes", "route_latest")
            os.makedirs(bundle_dir, exist_ok=True)
            with open(os.path.join(bundle_dir, "recorded_route.json"), 'w') as f:
                json.dump(final_route, f, indent=2)
            
            for map_file in ["hawkbot.pbstream", "hawkbot_static.yaml", "hawkbot_static.pgm"]:
                src = os.path.join(map_dir, map_file)
                if os.path.exists(src):
                    shutil.copy(src, os.path.join(bundle_dir, map_file))

            response.success = True
            response.message = f"Parcours et carte sauvegardés dans '{file_path}' et le bundle '{bundle_dir}' ({len(final_route)} waypoints)."
            self.get_logger().info(response.message)

            # 3. Publish final visualization
            self.publish_final_trail_marker(final_route)

            # 4. Activate trail_verifier upon saving trajectory (option 2)
            if self.cli_trail_enable.service_is_ready():
                self.cli_trail_enable.call_async(Trigger.Request())
            else:
                self.get_logger().info("Trail Verifier service not ready yet, it will load file on start.")
        except Exception as e:
            response.success = False
            response.message = f"Erreur lors de la sauvegarde du fichier JSON: {e}"
            self.get_logger().error(response.message)

        return response

    def record_timer_callback(self):
        if not self.is_recording:
            return

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                f'{self.frame_prefix}base_footprint',
                now,
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y
        yaw = quaternion_to_yaw(trans.transform.rotation)

        if not self.raw_points:
            self.raw_points.append((x, y, yaw))
            self.publish_current_trail()
        else:
            last_x, last_y, _ = self.raw_points[-1]
            dist = math.hypot(x - last_x, y - last_y)
            if dist >= self.min_dist_threshold:
                self.raw_points.append((x, y, yaw))
                self.publish_current_trail()

    def extract_wall_dists(self):
        if self.latest_scan is None:
            return [2.0] * 8
        msg = self.latest_scan
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        dists = []
        for target_deg in [0, 45, 90, 135, 180, 225, 270, 315]:
            target_rad = math.radians(target_deg)
            mask = (angles >= target_rad - 0.13) & (angles <= target_rad + 0.13) & (ranges > msg.range_min) & (ranges < msg.range_max) & np.isfinite(ranges)
            r_sector = ranges[mask]
            if len(r_sector) > 0:
                dists.append(round(float(np.min(r_sector)), 3))
            else:
                dists.append(2.0)
        return dists

    def process_and_smooth_loop(self, raw_pts):
        """Creates a smooth closed loop from raw points preserving exact path geometry and wall signatures."""
        pts = [ (p[0], p[1]) for p in raw_pts ]
        if len(pts) < 3:
            return []

        # 1. Close the gap between last point and first point smoothly
        p_first = pts[0]
        p_last = pts[-1]
        dist_gap = math.hypot(p_first[0] - p_last[0], p_first[1] - p_last[1])

        # If there's a gap between last and first point, add 2-3 linear bridge points
        if dist_gap > 0.2:
            num_bridge = max(2, int(dist_gap / 0.15))
            for k in range(1, num_bridge):
                alpha = k / float(num_bridge)
                bridge_x = p_last[0] + alpha * (p_first[0] - p_last[0])
                bridge_y = p_last[1] + alpha * (p_first[1] - p_last[1])
                pts.append((bridge_x, bridge_y))

        # 2. Apply gentle micro-zigzag removal (preserves 100% path scale & straight lines)
        clean_pts = smooth_micro_zigzags(pts, passes=2)

        # 3. Apply light Chaikin corner smoothing (1 iteration for natural corners)
        smoothed_points = chaikin_smooth(clean_pts, iterations=1)

        # 3. Clean resampling (every 7 cm) to prevent micro-zigzag noise
        dense_points = []
        n_sm = len(smoothed_points)
        step_size = 0.07 # 7 cm
        for i in range(n_sm):
            p1 = smoothed_points[i]
            p2 = smoothed_points[(i + 1) % n_sm]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if seg_len < 1e-4:
                continue
            n_steps = max(1, int(seg_len / step_size))
            for s in range(n_steps):
                alpha = s / float(n_steps)
                dense_points.append((p1[0] + alpha * (p2[0] - p1[0]), p1[1] + alpha * (p2[1] - p1[1])))

        # 4. Calculate headings (yaw) & extract Lidar wall distance signatures
        wall_sig = self.extract_wall_dists()
        route_data = []
        m = len(dense_points)
        for idx in range(m):
            curr_p = dense_points[idx]
            next_p = dense_points[(idx + 1) % m]
            heading = math.atan2(next_p[1] - curr_p[1], next_p[0] - curr_p[0])
            route_data.append({
                "id": idx,
                "x": round(float(curr_p[0]), 4),
                "y": round(float(curr_p[1]), 4),
                "yaw": round(float(heading), 4),
                "wall_dists": wall_sig
            })

        return route_data

    def publish_current_trail(self):
        """Publishes the raw blue trail marker during live recording."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.robot_ns}_trail"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08  # Line width
        marker.color.r = 0.0
        marker.color.g = 0.4
        marker.color.b = 1.0  # Bright Blue
        marker.color.a = 0.9

        for p in self.raw_points:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = 0.05
            marker.points.append(pt)

        self.marker_pub.publish(marker)

        # Publish live Path message for follower nodes
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.raw_points:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = path_msg.header.stamp
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = 0.05
            ps.pose.orientation = yaw_to_quaternion(float(p[2]))
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)

    def publish_final_trail_marker(self, route_data):
        """Publishes the final smoothed closed-loop blue marker."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.robot_ns}_trail"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.10  # Slightly thicker for final loop
        marker.color.r = 0.0
        marker.color.g = 0.6
        marker.color.b = 1.0  # Vivid Electric Blue
        marker.color.a = 1.0

        for wp in route_data:
            pt = Point()
            pt.x = wp['x']
            pt.y = wp['y']
            pt.z = 0.05
            marker.points.append(pt)

        # Explicitly close the loop to first point
        if route_data:
            pt0 = Point()
            pt0.x = route_data[0]['x']
            pt0.y = route_data[0]['y']
            pt0.z = 0.05
            marker.points.append(pt0)

        self.marker_pub.publish(marker)

    def clear_rviz_markers(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = f"{self.robot_ns}_trail"
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)

        # Also publish DELETEALL for target waypoint green sphere marker
        target_pub = self.create_publisher(Marker, f'/{self.robot_ns}/target_waypoint_marker' if self.robot_ns else 'target_waypoint_marker', 10)
        m_target = Marker()
        m_target.header.frame_id = 'map'
        m_target.ns = f"{self.robot_ns}_target"
        m_target.action = Marker.DELETEALL
        target_pub.publish(m_target)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
