#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from std_msgs.msg import Bool
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

def quaternion_to_euler(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def euler_to_quaternion(yaw):
    from geometry_msgs.msg import Quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.w = cy
    q.z = sy
    return q

class TFStabilizer(Node):
    def __init__(self):
        super().__init__('tf_stabilizer')
        
        self.declare_parameter('robot_ns', '')
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.frame_prefix = f"{self.robot_ns}/" if self.robot_ns else ""
        
        self.odom_carto_frame = f"{self.frame_prefix}odom_carto"
        self.odom_combined_frame = f"{self.frame_prefix}odom_combined"
        
        # TF Buffer & Listener to get map -> odom_carto
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # TF Broadcaster to publish odom_carto -> odom_combined
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscribers
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # State
        self.last_cmd_time = None
        self.is_stopped = False
        
        # Frozen Target Pose (map -> odom_combined)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.has_target = False
        
        self.create_timer(0.02, self.timer_callback) # 50 Hz
        self.get_logger().info(f"TF Stabilizer started for {self.robot_ns}")

    def cmd_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        if abs(msg.linear.x) < 0.01 and abs(msg.angular.z) < 0.01:
            self.is_stopped = True
        else:
            self.is_stopped = False

    def timer_callback(self):
        now_time = self.get_clock().now()
        
        # Check timeout (0.5s without cmd_vel = stopped)
        if self.last_cmd_time is None or (now_time - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            self.is_stopped = True
            
        try:
            # Look up the jittery Cartographer pose
            t = self.tf_buffer.lookup_transform(
                'map',
                self.odom_carto_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.01)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            # Bootstrap: if Cartographer hasn't published yet, publish Identity 
            # so Cartographer can find odom_carto in the tree and start publishing!
            offset_msg = TransformStamped()
            future_time = now_time + rclpy.duration.Duration(seconds=0.1)
            offset_msg.header.stamp = future_time.to_msg()
            offset_msg.header.frame_id = self.odom_carto_frame
            offset_msg.child_frame_id = self.odom_combined_frame
            offset_msg.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(offset_msg)
            return
            
        c_x = t.transform.translation.x
        c_y = t.transform.translation.y
        c_theta = quaternion_to_euler(t.transform.rotation)
        
        if not self.is_stopped:
            # Moving: we want odom_carto -> odom_combined to be Identity
            # so target just follows Cartographer
            self.target_x = c_x
            self.target_y = c_y
            self.target_theta = c_theta
            self.has_target = True
            
            o_x = 0.0
            o_y = 0.0
            o_theta = 0.0
        else:
            if not self.has_target:
                # First time stopping, freeze the target here
                self.target_x = c_x
                self.target_y = c_y
                self.target_theta = c_theta
                self.has_target = True
                
            # Stopped: calculate the offset to absorb Cartographer's jitter
            dx = self.target_x - c_x
            dy = self.target_y - c_y
            
            o_theta = self.target_theta - c_theta
            o_x = dx * math.cos(c_theta) + dy * math.sin(c_theta)
            o_y = -dx * math.sin(c_theta) + dy * math.cos(c_theta)
            
        # Publish the offset transform
        offset_msg = TransformStamped()
        # Publish slightly into the future to satisfy tf2 interpolators for incoming sensor data
        future_time = now_time + rclpy.duration.Duration(seconds=0.1)
        offset_msg.header.stamp = future_time.to_msg()
        offset_msg.header.frame_id = self.odom_carto_frame
        offset_msg.child_frame_id = self.odom_combined_frame
        
        offset_msg.transform.translation.x = o_x
        offset_msg.transform.translation.y = o_y
        offset_msg.transform.translation.z = 0.0
        offset_msg.transform.rotation = euler_to_quaternion(o_theta)
        
        self.tf_broadcaster.sendTransform(offset_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFStabilizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
