import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import rclpy.duration
import math

class ScanMatchMonitor(Node):
    def __init__(self):
        super().__init__('scan_match_monitor')
        
        # Declare parameters
        self.declare_parameter('robot_ns', 'robot1')
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        
        # Prefixes
        self.frame_prefix = f"{self.robot_ns}/" if self.robot_ns else ""
        
        # Threshholds
        self.declare_parameter('min_match_ratio', 0.80)  # Match ratio below this = lost
        self.declare_parameter('ok_match_ratio', 0.90)   # Match ratio above this = recovered
        self.min_match_ratio = self.get_parameter('min_match_ratio').get_parameter_value().double_value
        self.ok_match_ratio = self.get_parameter('ok_match_ratio').get_parameter_value().double_value
        
        self.map_data = None
        self.is_localized = True
        
        # TF Buffer and Listener (capped at 10s memory cache)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publisher for localization status (True = OK, False = LOST/PAUSE)
        self.status_pub = self.create_publisher(Bool, 'localization_ok', 10)
        
        # Publisher for dynamic recentering
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        
        # Publisher for score percentage
        self.score_pub = self.create_publisher(Float32, 'scan_match_score', 10)
        
        # Last known good pose
        self.last_known_good_pose = None
        
        # Subscribers
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/static_map',
            self.map_callback,
            map_qos
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            f'/{self.robot_ns}/scan',
            self.scan_callback,
            10
        )
        
        self.get_logger().info(f"Scan Match Monitor started for namespace: '{self.robot_ns}'")
        self.get_logger().info(f"Thresholds: lost < {self.min_match_ratio*100}%, recovered > {self.ok_match_ratio*100}%")

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Occupancy map received for scan-matching comparison.", once=True)

    def is_near_wall(self, x, y, window_size=3):
        """Check if map pixel at (x,y) or its 3x3 neighbors (7.5cm range) is occupied."""
        if self.map_data is None:
            return True
            
        info = self.map_data.info
        res = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        width = info.width
        height = info.height
        
        col = int((x - origin_x) / res)
        row = int((y - origin_y) / res)
        
        half_w = window_size // 2
        for r in range(row - half_w, row + half_w + 1):
            for c in range(col - half_w, col + half_w + 1):
                if c >= 0 and c < width and r >= 0 and r < height:
                    idx = r * width + c
                    val = self.map_data.data[idx]
                    # val > 65 means occupied cell (wall)
                    if val > 65:
                        return True
        return False

    def scan_callback(self, msg):
        if self.map_data is None:
            # Publish True to not block robots if map is not loaded yet
            status_msg = Bool()
            status_msg.data = True
            self.status_pub.publish(status_msg)
            # Throttle the warning so it doesn't spam
            self.get_logger().warn("Waiting for /static_map...", throttle_duration_sec=2.0)
            return

        try:
            # Look up the transform from 'map' to the scan sensor frame
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                f'{self.frame_prefix}base_scan',
                now,
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            # Also lookup base_footprint for dynamic recentering
            trans_footprint = self.tf_buffer.lookup_transform(
                'map',
                f'{self.frame_prefix}base_footprint',
                now,
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"TF lookup failed in scan_match_monitor: {e}", throttle_duration_sec=3.0)
            return

        # Extract translation and rotation from TF
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        
        # Calculate yaw from quaternion
        q = trans.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        valid_points = 0
        matched_points = 0
        
        # Project scan points into map frame and verify superposition
        for i, r in enumerate(msg.ranges):
            # Filter out invalid lidar readings
            if math.isnan(r) or math.isinf(r) or r < msg.range_min or r > msg.range_max:
                continue
                
            valid_points += 1
            angle = msg.angle_min + i * msg.angle_increment
            
            # Point coordinate in base_scan frame
            x_l = r * math.cos(angle)
            y_l = r * math.sin(angle)
            
            # Rotate and translate to map frame
            x_m = tx + x_l * cos_yaw - y_l * sin_yaw
            y_m = ty + x_l * sin_yaw + y_l * cos_yaw
            
            # Check proximity to walls (using a 3x3 window ~ 7.5cm)
            if self.is_near_wall(x_m, y_m, window_size=3):
                matched_points += 1

        if valid_points == 0:
            return

        ratio = matched_points / valid_points

        # Publish score percentage to dedicated ROS topic
        score_msg = Float32()
        score_msg.data = float(ratio * 100.0)
        self.score_pub.publish(score_msg)

        # Hysteresis state logic to prevent rapid switching
        if self.is_localized:
            if ratio < self.min_match_ratio:
                self.is_localized = False
                self.get_logger().warn(
                    f"Localization score dropped for {self.robot_ns}! Match ratio: {ratio*100:.1f}% (limit: {self.min_match_ratio*100:.1f}%). Pausing route follower for safety.",
                    throttle_duration_sec=2.0
                )
                # Note: We do NOT publish to /initialpose automatically here because forcing trajectory restarts
                # mid-navigation snaps the robot's pose unexpectedly. /initialpose is reserved for manual RViz estimation.
        else:
            if ratio > self.ok_match_ratio:
                self.is_localized = True
                self.get_logger().info(
                    f"Localization RECOVERED for {self.robot_ns}! Match ratio: {ratio*100:.1f}%. RESUMING."
                )
        
        # Update last known good pose whenever score is healthy (above 70%)
        if self.is_localized or ratio > 0.70:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose.position.x = trans_footprint.transform.translation.x
            pose_msg.pose.pose.position.y = trans_footprint.transform.translation.y
            pose_msg.pose.pose.position.z = trans_footprint.transform.translation.z
            pose_msg.pose.pose.orientation = trans_footprint.transform.rotation
            pose_msg.pose.covariance[0] = 0.05
            pose_msg.pose.covariance[7] = 0.05
            pose_msg.pose.covariance[35] = 0.05
            self.last_known_good_pose = pose_msg

        # Publish status
        status_msg = Bool()
        status_msg.data = self.is_localized
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanMatchMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
