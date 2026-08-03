import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import math

def quaternion_to_euler(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def quaternion_to_roll_pitch(q):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    return roll, pitch

def euler_to_quaternion(roll, pitch, yaw):
    from geometry_msgs.msg import Quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q

class OdomCorrector(Node):
    def __init__(self):
        super().__init__('odom_corrector')
        
        self.declare_parameter('robot_ns', '')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('odom_yaw_scale', 0.898734)
        self.declare_parameter('imu_yaw_scale', 1.1888)
        
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.odom_yaw_scale = self.get_parameter('odom_yaw_scale').get_parameter_value().double_value
        self.imu_yaw_scale = self.get_parameter('imu_yaw_scale').get_parameter_value().double_value
        
        # Create namespace prefix
        self.frame_prefix = f"{self.robot_ns}/" if self.robot_ns else ""

        self.get_logger().info(f"Odom Corrector initialized for namespace: '{self.robot_ns}'")
        self.get_logger().info(f"Publish TF: {self.publish_tf}")

        # Publishers
        self.pub = self.create_publisher(Odometry, 'odom_corrected', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_corrected', 10)

        # Subscribers
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        
        # Speed tracking for gyro calibration
        self.last_linear_speed = 0.0
        self.last_angular_speed = 0.0
        self.last_odom_time = self.get_clock().now()

        # Gyro bias calibration (computed on RAW gyro, before scaling)
        self.gyro_bias = 0.0
        self.bias_samples = 0
        
        # Unwrapping state for scaling
        self.last_raw_odom_yaw = None
        self.unwrapped_odom_yaw = 0.0
        
        # Integrate bias-corrected+scaled gyro for orientation output
        self.integrated_imu_yaw = 0.0
        self.last_imu_time_stamp = None

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def handle_reset_origin(self, request, response):
        self.initial_odom_x = None
        self.initial_odom_y = None
        response.success = True
        response.message = f"Origine odom_combined réinitialisée aux pieds du robot {self.robot_ns}."
        self.get_logger().info(response.message)
        return response

    def odom_callback(self, msg):
        self.get_logger().info("Received odom message, correcting and publishing...", once=True)
        
        # Track speed and update time
        self.last_odom_time = self.get_clock().now()
        self.last_linear_speed = abs(msg.twist.twist.linear.x)
        self.last_angular_speed = abs(msg.twist.twist.angular.z)

        corrected_msg = Odometry()
        corrected_msg.header = msg.header
        
        # 1. Correct the frame IDs to namespaced versions
        corrected_msg.header.frame_id = f"{self.frame_prefix}odom_combined"
        corrected_msg.child_frame_id = f"{self.frame_prefix}base_footprint"
        
        # 2. Copy the pose and twist data
        corrected_msg.pose.pose = msg.pose.pose
        corrected_msg.twist.twist = msg.twist.twist
        
        raw_yaw = quaternion_to_euler(msg.pose.pose.orientation)
        
        if self.last_raw_odom_yaw is None:
            self.last_raw_odom_yaw = raw_yaw
            self.unwrapped_odom_yaw = raw_yaw
            
        delta_yaw = raw_yaw - self.last_raw_odom_yaw
        if delta_yaw > math.pi:
            delta_yaw -= 2.0 * math.pi
        elif delta_yaw < -math.pi:
            delta_yaw += 2.0 * math.pi
            
        self.last_raw_odom_yaw = raw_yaw
        self.unwrapped_odom_yaw += delta_yaw
        
        scaled_yaw = self.unwrapped_odom_yaw * self.odom_yaw_scale
        scaled_yaw = math.atan2(math.sin(scaled_yaw), math.cos(scaled_yaw))
        
        corrected_msg.pose.pose.orientation = euler_to_quaternion(0, 0, scaled_yaw)
        corrected_msg.twist.twist.angular.z *= self.odom_yaw_scale
        
        # 3. Apply a healthy, symmetric positive-definite covariance matrix
        pose_cov = np.zeros(36)
        pose_cov[0] = 0.05   # X variance
        pose_cov[7] = 0.05   # Y variance
        pose_cov[14] = 1000.0 # Z variance (planar)
        pose_cov[21] = 1000.0 # Roll variance (planar)
        pose_cov[28] = 1000.0 # Pitch variance (planar)
        pose_cov[35] = 0.1    # Yaw variance
        
        twist_cov = np.zeros(36)
        twist_cov[0] = 0.02   # vx variance
        twist_cov[7] = 1000.0 # vy variance
        twist_cov[14] = 1000.0 # vz variance
        twist_cov[21] = 1000.0 # vroll variance
        twist_cov[28] = 1000.0 # vpitch variance
        twist_cov[35] = 0.05   # vyaw variance
        
        corrected_msg.pose.covariance = list(pose_cov)
        corrected_msg.twist.covariance = list(twist_cov)
        
        # Publish corrected message
        self.pub.publish(corrected_msg)
        
        # Publish TF transform if EKF is disabled/missing
        if self.publish_tf:
            t = TransformStamped()
            t.header = corrected_msg.header
            t.child_frame_id = corrected_msg.child_frame_id
            t.transform.translation.x = corrected_msg.pose.pose.position.x
            t.transform.translation.y = corrected_msg.pose.pose.position.y
            t.transform.translation.z = corrected_msg.pose.pose.position.z
            t.transform.rotation = corrected_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def imu_callback(self, msg):
        corrected_imu = Imu()
        corrected_imu.header = msg.header
        corrected_imu.header.frame_id = f"{self.frame_prefix}gyro_link"
        corrected_imu.orientation_covariance = msg.orientation_covariance
        
        # --- Stationary detection ---
        time_since_last_odom = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
        is_moving = (time_since_last_odom < 0.5) and ((self.last_linear_speed > 0.005) or (self.last_angular_speed > 0.005))
        
        # Work on RAW gyro for bias estimation (before any scaling)
        raw_gyro_z = msg.angular_velocity.z
        
        # Override: if raw gyro clearly shows rotation, mark as moving
        if abs(raw_gyro_z) > 0.04:
            is_moving = True
        
        if not is_moving:
            # Update running average of gyro bias on RAW values
            if self.bias_samples < 1500:
                self.bias_samples += 1
                self.gyro_bias += (raw_gyro_z - self.gyro_bias) / self.bias_samples
                if self.bias_samples % 250 == 0:
                    percent = int(self.bias_samples / 15)
                    self.get_logger().info(f"Gyro calibration: {percent}% ({self.bias_samples}/1500), bias={self.gyro_bias:.6f} rad/s")
                if self.bias_samples == 1500:
                    self.get_logger().info(f"Gyro calibration complete. Bias: {self.gyro_bias:.6f} rad/s")
            else:
                # Slowly adapt to temperature drift
                self.gyro_bias = 0.999 * self.gyro_bias + 0.001 * raw_gyro_z
            
            # Stationary: force corrected rate to exactly 0.0
            corrected_gyro_z = 0.0
        else:
            # Moving: subtract RAW bias, THEN apply scale
            corrected_gyro_z = (raw_gyro_z - self.gyro_bias) * self.imu_yaw_scale
            
        corrected_imu.angular_velocity.x = msg.angular_velocity.x
        corrected_imu.angular_velocity.y = msg.angular_velocity.y
        corrected_imu.angular_velocity.z = corrected_gyro_z
        
        # Integrate bias-corrected+scaled gyro for orientation
        now_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        roll, pitch = quaternion_to_roll_pitch(msg.orientation)
        
        if self.last_imu_time_stamp is not None:
            dt = now_time - self.last_imu_time_stamp
            if dt > 0.0 and dt < 0.5:
                self.integrated_imu_yaw += corrected_gyro_z * dt
            
            display_yaw = math.atan2(math.sin(self.integrated_imu_yaw), math.cos(self.integrated_imu_yaw))
            corrected_imu.orientation = euler_to_quaternion(roll, pitch, display_yaw)
        else:
            # Initialize integrated yaw from the raw IMU orientation to match starting heading
            init_yaw = quaternion_to_euler(msg.orientation)
            self.integrated_imu_yaw = init_yaw
            corrected_imu.orientation = euler_to_quaternion(roll, pitch, init_yaw)
        self.last_imu_time_stamp = now_time
            
        corrected_imu.angular_velocity_covariance = msg.angular_velocity_covariance
        corrected_imu.linear_acceleration = msg.linear_acceleration
        corrected_imu.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.imu_pub.publish(corrected_imu)

def main(args=None):
    rclpy.init(args=args)
    node = OdomCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
