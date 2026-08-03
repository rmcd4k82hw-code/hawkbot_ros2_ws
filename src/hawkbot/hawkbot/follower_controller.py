import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')
        
        # Safety radius (10 cm)
        self.R = 0.10
        
        # Controller gains
        self.kp_linear = 0.6
        self.kp_angular = 2.5
        self.max_linear_speed = 0.11  # m/s
        self.max_angular_speed = 0.75  # rad/s
        
        # Localization status states
        self.robot1_ok = True
        self.robot2_ok = True
        
        # Publisher to Robot 2 velocity
        self.cmd_pub = self.create_publisher(Twist, '/robot2/cmd_vel', 10)
        
        # Subscribers to Scan-to-Map Match Monitors
        self.r1_status_sub = self.create_subscription(
            Bool,
            '/robot1/localization_ok',
            self.r1_status_callback,
            10
        )
        self.r2_status_sub = self.create_subscription(
            Bool,
            '/robot2/localization_ok',
            self.r2_status_callback,
            10
        )
        
        # Low-pass filter variables for relative pose smoothing (limits TF/Cartographer jumps)
        self.x_filt = None
        self.y_filt = None
        self.alpha = 0.12  # Smoothing factor (running at 20Hz, creates a ~0.4s time constant)
        
        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Main control loop timer running at 20 Hz
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("Follower Controller Node started using TF localization with LPF.")
        self.get_logger().info("Targeting a 10cm periphery around /robot1/base_footprint.")
        self.get_logger().info("Automatic Pause-on-Lost feature enabled.")

    def r1_status_callback(self, msg):
        self.robot1_ok = msg.data

    def r2_status_callback(self, msg):
        self.robot2_ok = msg.data

    def control_loop(self):
        # Safety Pause: if either robot is not localized, stop and wait
        if not self.robot1_ok or not self.robot2_ok:
            twist = Twist()
            self.cmd_pub.publish(twist)
            self.get_logger().warn(
                f"PAUSE ACTIVE: robot1_ok={self.robot1_ok}, robot2_ok={self.robot2_ok}. Waiting for localization alignment...",
                throttle_duration_sec=2.0
            )
            return

        try:
            # Look up the transform from Robot 2's base frame to Robot 1's base frame.
            # This handles the spatial relationship in the shared global map frame.
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'robot2/base_footprint',
                'robot1/base_footprint',
                now,
                timeout=rclpy.duration.Duration(seconds=0.08)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not look up TF between robots: {e}", throttle_duration_sec=2.0)
            return

        # Relative position of Robot 1 in Robot 2's frame
        x_raw = trans.transform.translation.x
        y_raw = trans.transform.translation.y
        d_raw = math.sqrt(x_raw**2 + y_raw**2)
        
        # Apply Low-Pass Filter to smooth out high-frequency EKF/Cartographer TF jitters
        if self.x_filt is None or self.y_filt is None:
            self.x_filt = x_raw
            self.y_filt = y_raw
        else:
            self.x_filt = self.alpha * x_raw + (1.0 - self.alpha) * self.x_filt
            self.y_filt = self.alpha * y_raw + (1.0 - self.alpha) * self.y_filt
            
        x_rel = self.x_filt
        y_rel = self.y_filt
        d = math.sqrt(x_rel**2 + y_rel**2)
        
        twist = Twist()
        
        # Avoid division by zero
        if d < 0.01:
            self.cmd_pub.publish(twist)
            return
            
        # Target point on the boundary of Robot 1 closest to Robot 2
        err_dist = d - self.R
        
        # Heading to Robot 1
        err_theta = math.atan2(y_rel, x_rel)
        
        # Normalize angle error to [-pi, pi]
        err_theta = math.atan2(math.sin(err_theta), math.cos(err_theta))
        
        # Log status (displays filtered vs raw distance to monitor noise rejection)
        self.get_logger().info(
            f"TF Track: dist_filt={d:.3f}m (raw={d_raw:.3f}m), error={err_dist:.3f}m, angle={err_theta*180/math.pi:.1f}deg", 
            throttle_duration_sec=1.0
        )
        
        # Control logic
        if abs(err_dist) < 0.05: # Stop if within 5cm of target periphery
            twist.linear.x = 0.0
            # Adjust yaw if angle error is significant
            if abs(err_theta) > 0.1:
                twist.angular.z = self.kp_angular * err_theta
            else:
                twist.angular.z = 0.0
        else:
            # If angle error is large, pivot on spot first
            if abs(err_theta) > 0.4:
                twist.linear.x = 0.0
                twist.angular.z = self.kp_angular * err_theta
            else:
                # Proportional linear and angular control
                twist.linear.x = self.kp_linear * err_dist
                twist.angular.z = self.kp_angular * err_theta
                
                # Saturate velocities to safety limits
                twist.linear.x = min(twist.linear.x, self.max_linear_speed)
                twist.linear.x = max(twist.linear.x, -self.max_linear_speed)
                
            twist.angular.z = min(twist.angular.z, self.max_angular_speed)
            twist.angular.z = max(twist.angular.z, -self.max_angular_speed)
            
        # Publish command to Robot 2
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on shutdown
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
