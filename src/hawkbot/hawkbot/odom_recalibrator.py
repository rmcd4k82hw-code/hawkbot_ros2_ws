import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import rclpy.duration
import math

def get_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def euler_to_quaternion(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

class OdomRecalibrator(Node):
    def __init__(self):
        super().__init__('odom_recalibrator')

        self.declare_parameter('robot_ns', 'robot1')
        self.declare_parameter('decay_rate', 0.08) # 8% smooth decay per 20ms frame
        
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.decay_rate = self.get_parameter('decay_rate').get_parameter_value().double_value
        
        self.frame_prefix = f"{self.robot_ns}/" if self.robot_ns else ""
        self.odom_frame = f"{self.frame_prefix}odom_combined"
        self.base_frame = f"{self.frame_prefix}base_footprint"

        # Publisher to EKF set_pose topic (relative topic within robot namespace)
        self.set_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'set_pose', 10)

        # TF Buffer (capped at 10s memory cache)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # High frequency 50 Hz timer (every 20 ms) for smooth real-time linear & angular recalibration
        self.timer = self.create_timer(0.02, self.smooth_recalibrate_callback)

        self.get_logger().info(f"High-frequency 50Hz Real-Time Linear & Angular Odom Recalibrator started for '{self.robot_ns}'.")

    def smooth_recalibrate_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.01)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        cx = trans.transform.translation.x
        cy = trans.transform.translation.y
        dist = math.hypot(cx, cy)

        cyaw = get_yaw(trans.transform.rotation)

        should_update = False
        target_x = cx
        target_y = cy
        target_yaw = cyaw

        # Smooth linear recalibration
        if dist > 0.005:
            target_x = cx * (1.0 - self.decay_rate)
            target_y = cy * (1.0 - self.decay_rate)
            should_update = True

        # Smooth angular recalibration (~0.3 degrees threshold)
        if abs(cyaw) > 0.005:
            target_yaw = cyaw * (1.0 - self.decay_rate)
            should_update = True

        if should_update:
            reset_msg = PoseWithCovarianceStamped()
            reset_msg.header.stamp = self.get_clock().now().to_msg()
            reset_msg.header.frame_id = self.odom_frame
            reset_msg.pose.pose.position.x = target_x
            reset_msg.pose.pose.position.y = target_y
            reset_msg.pose.pose.position.z = 0.0
            reset_msg.pose.pose.orientation = euler_to_quaternion(target_yaw)

            # Low variance for smooth tracking
            reset_msg.pose.covariance[0] = 1e-3
            reset_msg.pose.covariance[7] = 1e-3
            reset_msg.pose.covariance[35] = 1e-3
            
            # Publish real-time smooth linear & angular update
            self.set_pose_pub.publish(reset_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomRecalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
