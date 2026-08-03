#!/usr/bin/env python3
"""
Kinematic TF Filter & Auto-Heal (kinematic_tf_filter.py)

Intercepts Cartographer's TF updates (/tf_carto) and filters them before
publishing to the real /tf topic.

Features:
1. TF Smoothing: If Cartographer hallucinates a jump (e.g., speed > 0.2 m/s 
   or rotation > 0.5 rad/s), the jump is rejected. The filter freezes the 
   map -> odom_combined transform.
2. Auto-Healing: When a jump is detected, the filter automatically publishes 
   the last known good position to /initialpose, forcing Cartographer to 
   snap back and correct its internal state.
3. Visual Motion Lock: If the camera optical flow says the robot is stationary
   but Cartographer says it moved, the transform is locked.

Subscribes:
  /tf_carto (tf2_msgs/TFMessage) - Intercepted Cartographer TFs
  /<robot_ns>/visual_motion_ok (std_msgs/Bool) - Camera optical flow status
  TF: <robot_ns>/odom_combined -> <robot_ns>/base_footprint (from EKF)

Publishes:
  /tf (tf2_msgs/TFMessage) - Clean, bounded TFs
  /<robot_ns>/initialpose (geometry_msgs/PoseWithCovarianceStamped) - Auto-heal
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
import tf2_ros
import math
import time

class KinematicTfFilter(Node):
    def __init__(self):
        super().__init__('kinematic_tf_filter')

        self.declare_parameter('robot_ns', 'robot1')
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.frame_prefix = f"{self.robot_ns}/" if self.robot_ns else ""

        # Kinematic limits (Robot max physical: ~0.15m/s and ~0.33rad/s)
        # We add a safety margin to allow legitimate corrections
        self.declare_parameter('max_linear_velocity', 0.25)
        self.declare_parameter('max_angular_velocity', 0.60)
        
        self.max_v = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.max_w = self.get_parameter('max_angular_velocity').get_parameter_value().double_value

        self.target_frame = f"{self.frame_prefix}odom_combined"
        self.base_frame = f"{self.frame_prefix}base_footprint"

        # State
        self.last_accepted_tf = None
        self.last_accepted_time = None
        self.visual_motion_ok = True
        self.healing_in_progress = False
        self.healing_start_time = 0.0

        # ROS Interfaces
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, f'/{self.robot_ns}/initialpose', 10)
        
        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 100)
        self.visual_sub = self.create_subscription(Bool, f'/{self.robot_ns}/visual_motion_ok', self.visual_callback, 10)

        # TF Buffer for EKF queries
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Kinematic TF Filter started for '{self.robot_ns}'. Limits: v={self.max_v}m/s, w={self.max_w}rad/s")

    def visual_callback(self, msg):
        self.visual_motion_ok = msg.data

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def quaternion_from_yaw(self, yaw):
        return [0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)]

    def trigger_auto_heal(self, tf_msg):
        """Forces Cartographer back to the last known good pose."""
        now = time.time()
        # Prevent spamming the heal command
        if now - self.healing_start_time < 3.0:
            return

        self.healing_in_progress = True
        self.healing_start_time = now
        self.get_logger().error(">>> CARTOGRAPHER JUMP DETECTED! Triggering Auto-Heal... <<<")

        try:
            # We need the full map -> base_footprint pose to initialize Cartographer
            # We use our frozen map -> odom_combined AND the current EKF odom_combined -> base_footprint
            odom_to_base = self.tf_buffer.lookup_transform(
                self.target_frame, self.base_frame, rclpy.time.Time()
            )

            # Reconstruct map -> base_footprint
            # Map -> Odom
            mx = self.last_accepted_tf.transform.translation.x
            my = self.last_accepted_tf.transform.translation.y
            myaw = self.get_yaw(self.last_accepted_tf.transform.rotation)

            # Odom -> Base
            ox = odom_to_base.transform.translation.x
            oy = odom_to_base.transform.translation.y
            oyaw = self.get_yaw(odom_to_base.transform.rotation)

            # Compose (2D)
            base_x = mx + (ox * math.cos(myaw) - oy * math.sin(myaw))
            base_y = my + (ox * math.sin(myaw) + oy * math.cos(myaw))
            base_yaw = myaw + oyaw

            q = self.quaternion_from_yaw(base_yaw)

            # Publish to /initialpose
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.pose.position.x = base_x
            pose_msg.pose.pose.position.y = base_y
            pose_msg.pose.pose.orientation.x = q[0]
            pose_msg.pose.pose.orientation.y = q[1]
            pose_msg.pose.pose.orientation.z = q[2]
            pose_msg.pose.pose.orientation.w = q[3]

            self.initialpose_pub.publish(pose_msg)
            self.get_logger().info(f"Auto-Heal sent: x={base_x:.2f}, y={base_y:.2f}, yaw={math.degrees(base_yaw):.1f}°")

        except Exception as e:
            self.get_logger().warn(f"Auto-Heal failed to lookup EKF transform: {e}")

    def tf_callback(self, msg):
        out_msg = TFMessage()

        for transform in msg.transforms:
            # We only filter the map -> odom_combined transform from Cartographer
            if transform.header.frame_id == 'map' and transform.child_frame_id == self.target_frame:
                
                # Copy the transform to avoid mutating the incoming message reference
                import copy
                publish_tf = copy.deepcopy(transform)
                
                now_sec = publish_tf.header.stamp.sec + publish_tf.header.stamp.nanosec * 1e-9
                
                # First transform initialization
                if self.last_accepted_tf is None:
                    self.last_accepted_tf = publish_tf
                    self.last_accepted_time = now_sec
                    out_msg.transforms.append(publish_tf)
                    continue

                dt = now_sec - self.last_accepted_time
                if dt <= 0:
                    out_msg.transforms.append(publish_tf)
                    continue

                # Compute change (map -> odom is the drift correction, it should move slowly)
                dx = publish_tf.transform.translation.x - self.last_accepted_tf.transform.translation.x
                dy = publish_tf.transform.translation.y - self.last_accepted_tf.transform.translation.y
                
                yaw_now = self.get_yaw(publish_tf.transform.rotation)
                yaw_prev = self.get_yaw(self.last_accepted_tf.transform.rotation)
                dyaw = yaw_now - yaw_prev
                # Normalize dyaw to [-pi, pi]
                dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))

                v = math.sqrt(dx**2 + dy**2) / dt
                w = abs(dyaw) / dt

                is_jump = v > self.max_v or w > self.max_w
                is_visual_drift = not self.visual_motion_ok and (math.sqrt(dx**2 + dy**2) > 0.05 or abs(dyaw) > 0.08)

                if is_jump or is_visual_drift:
                    # REJECT
                    reason = "KINEMATIC JUMP" if is_jump else "VISUAL DRIFT"
                    self.get_logger().warn(f"[{reason}] Rejected Cartographer TF! v={v:.2f}m/s, w={w:.2f}rad/s", throttle_duration_sec=1.0)
                    
                    # Update timestamp of the frozen transform to keep it alive for tf2
                    self.last_accepted_tf.header.stamp = publish_tf.header.stamp
                    out_msg.transforms.append(self.last_accepted_tf)

                    if is_jump:
                        self.trigger_auto_heal(publish_tf)
                else:
                    # ACCEPT
                    self.last_accepted_tf = publish_tf
                    self.last_accepted_time = now_sec
                    out_msg.transforms.append(publish_tf)
                    self.healing_in_progress = False

        if out_msg.transforms:
            self.tf_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicTfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
