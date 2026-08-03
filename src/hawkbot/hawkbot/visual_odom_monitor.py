#!/usr/bin/env python3
"""
Visual Odometry Monitor (visual_odom_monitor.py)

Uses camera optical flow to detect contradictions between physical robot motion
and TF coordinate changes. If coordinates drift while the camera confirms the
robot is stationary, this node flags the localization as unreliable.

Principle:
  - Optical flow between consecutive camera frames measures PHYSICAL motion
  - TF lookup (map -> base_footprint) measures ESTIMATED coordinate changes
  - If TF says the robot moved significantly but the camera sees no motion
    → the coordinates are drifting (EKF/Cartographer error)
  - After several consecutive contradictions, publish visual_motion_ok = False

Publishes:
  /<robot_ns>/visual_motion_ok (std_msgs/Bool)
    True  = coordinates are consistent with visual motion
    False = drift detected, coordinates unreliable

Subscribes:
  /<robot_ns>/image_raw/compressed (sensor_msgs/CompressedImage)
  TF: map -> <robot_ns>/base_footprint
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import rclpy.duration


class VisualOdomMonitor(Node):
    def __init__(self):
        super().__init__('visual_odom_monitor')

        # --- Parameters ---
        self.declare_parameter('robot_ns', 'robot1')
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.frame_prefix = f"{self.robot_ns}/" if self.robot_ns else ""

        # Optical flow: below this median pixel displacement = visually stationary
        self.declare_parameter('flow_stationary_threshold', 1.5)
        self.flow_threshold = self.get_parameter('flow_stationary_threshold').get_parameter_value().double_value

        # TF: above this accumulated displacement (meters) between frames = coordinate movement
        self.declare_parameter('tf_drift_threshold', 0.06)
        self.tf_drift_threshold = self.get_parameter('tf_drift_threshold').get_parameter_value().double_value

        # Number of consecutive contradictory frames before declaring drift
        self.declare_parameter('contradiction_limit', 15)
        self.contradiction_limit = self.get_parameter('contradiction_limit').get_parameter_value().integer_value

        # --- Internal state ---
        self.bridge = CvBridge()
        self.prev_gray = None
        self.contradiction_count = 0
        self.is_consistent = True

        # TF position tracking (capped at 10s memory cache)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_tf_x = None
        self.last_tf_y = None
        self.tf_displacement_accum = 0.0

        # Lucas-Kanade optical flow parameters
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )
        # Good features to track (Shi-Tomasi corners)
        self.feature_params = dict(
            maxCorners=80,
            qualityLevel=0.3,
            minDistance=7,
            blockSize=7
        )

        # --- ROS interfaces ---
        self.image_sub = self.create_subscription(
            CompressedImage,
            f'/{self.robot_ns}/image_raw/compressed',
            self.image_callback,
            1  # QoS depth 1: always process the latest frame only
        )

        self.status_pub = self.create_publisher(
            Bool, f'/{self.robot_ns}/visual_motion_ok', 10
        )

        # Timer at 10Hz to accumulate TF displacement between camera frames
        self.create_timer(0.1, self.tf_accumulate_timer)

        self.get_logger().info(
            f"Visual Odom Monitor started for '{self.robot_ns}'. "
            f"Thresholds: flow<{self.flow_threshold}px=stationary, "
            f"tf>{self.tf_drift_threshold*100:.0f}cm=moved, "
            f"contradiction limit={self.contradiction_limit} frames"
        )

    # ------------------------------------------------------------------ #
    #  TF displacement accumulator (runs at 10Hz between camera frames)  #
    # ------------------------------------------------------------------ #
    def tf_accumulate_timer(self):
        """Sample TF at 10Hz and accumulate displacement since last camera frame."""
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                f'{self.frame_prefix}base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        if self.last_tf_x is not None:
            dx = x - self.last_tf_x
            dy = y - self.last_tf_y
            self.tf_displacement_accum += math.sqrt(dx * dx + dy * dy)

        self.last_tf_x = x
        self.last_tf_y = y

    # ------------------------------------------------------------------ #
    #  Camera frame processing (optical flow)                            #
    # ------------------------------------------------------------------ #
    def image_callback(self, msg):
        # Decode compressed JPEG to OpenCV BGR
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}", throttle_duration_sec=5.0)
            return

        # Downscale and convert to grayscale for performance
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (320, 240))
        
        # --- CROP ROI: BOTTOM 35% ---
        # Only look at the immediate floor to avoid distant moving objects, windows, or arena borders
        h, w = gray.shape
        roi_top = int(h * 0.65)
        roi = gray[roi_top:h, :]
        
        # Light Gaussian blur to reduce JPEG compression artefacts
        roi = cv2.GaussianBlur(roi, (5, 5), 0)

        if self.prev_gray is None:
            self.prev_gray = roi
            return

        # --- Sparse optical flow (Lucas-Kanade) ---
        # Detect corner features in previous frame (bottom half only)
        prev_pts = cv2.goodFeaturesToTrack(self.prev_gray, **self.feature_params)

        if prev_pts is None or len(prev_pts) < 5:
            # Not enough features (e.g. blank wall in front of camera)
            self.prev_gray = roi
            return

        # Track those features in the current frame
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, roi, prev_pts, None, **self.lk_params
        )

        if next_pts is None:
            self.prev_gray = roi
            return

        # Keep only successfully tracked points
        good_mask = status.ravel() == 1
        if np.sum(good_mask) < 3:
            self.prev_gray = roi
            return

        good_prev = prev_pts[good_mask].reshape(-1, 2)
        good_next = next_pts[good_mask].reshape(-1, 2)

        # Compute per-point displacement magnitude (pixels)
        displacements = good_next - good_prev
        flow_magnitudes = np.linalg.norm(displacements, axis=1)

        # Use MEDIAN to be robust to outliers (someone walking past, partial occlusion)
        flow_magnitude = float(np.median(flow_magnitudes))

        # --- Read and reset accumulated TF displacement ---
        tf_displacement = self.tf_displacement_accum
        self.tf_displacement_accum = 0.0

        # --- Contradiction detection ---
        visually_stationary = flow_magnitude < self.flow_threshold
        tf_moved = tf_displacement > self.tf_drift_threshold

        if tf_moved and visually_stationary:
            # Coordinates say the robot moved, but the camera sees nothing → DRIFT
            self.contradiction_count += 1
        else:
            # Consistent: decay the contradiction counter
            self.contradiction_count = max(0, self.contradiction_count - 1)

        # --- State machine with hysteresis ---
        if self.is_consistent and self.contradiction_count >= self.contradiction_limit:
            self.is_consistent = False
            self.get_logger().warn(
                f"⚠ VISUAL DRIFT DETECTED [{self.robot_ns}]: "
                f"Camera flow={flow_magnitude:.1f}px (stationary), "
                f"but TF displaced {tf_displacement * 100:.1f}cm. "
                f"Coordinates unreliable!"
            )
        elif not self.is_consistent and self.contradiction_count <= 3:
            self.is_consistent = True
            self.get_logger().info(
                f"✓ Visual consistency RESTORED [{self.robot_ns}]. "
                f"Camera and coordinates agree again."
            )

        # Periodic info log
        self.get_logger().info(
            f"[{self.robot_ns}] flow={flow_magnitude:.1f}px, "
            f"tf_disp={tf_displacement * 100:.1f}cm, "
            f"ok={self.is_consistent}, "
            f"contradict={self.contradiction_count}/{self.contradiction_limit}",
            throttle_duration_sec=5.0
        )

        # --- Publish status ---
        status_msg = Bool()
        status_msg.data = self.is_consistent
        self.status_pub.publish(status_msg)

        # Update previous frame for next iteration
        self.prev_gray = roi


def main(args=None):
    rclpy.init(args=args)
    node = VisualOdomMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
