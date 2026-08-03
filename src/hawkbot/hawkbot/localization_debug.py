#!/usr/bin/env python3
"""
Localization Debug Node (localization_debug.py)

Publishes annotated camera feed with optical flow overlay and TF drift diagnostics.
Allows monitoring in RViz via Image display panels.

Published topics:
  /<robot_ns>/camera_debug (sensor_msgs/Image) - annotated camera with optical flow vectors
  /<robot_ns>/localization_diag (std_msgs/String) - text diagnostics

Subscribed topics:
  /<robot_ns>/image_raw/compressed (sensor_msgs/CompressedImage) - raw camera feed
  TF: map -> <robot_ns>/base_footprint - position tracking
  TF: <robot_ns>/odom_combined -> <robot_ns>/base_footprint - EKF tracking
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LocalizationDebug(Node):
    def __init__(self):
        super().__init__('localization_debug')

        self.declare_parameter('robot_ns', 'robot1')
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self.frame_prefix = f"{self.robot_ns}/" if self.robot_ns else ""

        self.bridge = CvBridge()
        self.prev_gray = None

        # --- TF tracking ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Cartographer TF: map -> odom_combined
        self.carto_x = 0.0
        self.carto_y = 0.0
        self.carto_yaw = 0.0
        self.carto_drift = 0.0  # accumulated drift per second

        # EKF TF: odom_combined -> base_footprint
        self.ekf_x = 0.0
        self.ekf_y = 0.0
        self.ekf_yaw = 0.0
        self.ekf_drift = 0.0

        # Previous values for drift computation
        self.prev_carto_x = None
        self.prev_carto_y = None
        self.prev_carto_yaw = None
        self.prev_ekf_x = None
        self.prev_ekf_y = None
        self.prev_ekf_yaw = None
        self.prev_tf_time = None

        # Optical flow stats
        self.flow_magnitude = 0.0

        # Lucas-Kanade parameters
        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )
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
            1
        )
        self.debug_image_pub = self.create_publisher(
            Image, f'/{self.robot_ns}/camera_debug', 1
        )
        self.diag_pub = self.create_publisher(
            String, f'/{self.robot_ns}/localization_diag', 10
        )

        # TF monitoring timer (10Hz)
        self.create_timer(0.1, self.tf_monitor)

        # Diagnostic log timer (2Hz)
        self.create_timer(0.5, self.publish_diagnostics)

        self.get_logger().info(f"Localization Debug started for '{self.robot_ns}'")
        self.get_logger().info(f"View camera in RViz: add Image display → topic: /{self.robot_ns}/camera_debug")

    # ─────────────────────────────────────────────
    #  TF monitoring: identify which transform drifts
    # ─────────────────────────────────────────────
    def tf_monitor(self):
        now = time.time()

        # 1. Cartographer TF: map -> odom_combined
        try:
            t = self.tf_buffer.lookup_transform(
                'map', f'{self.frame_prefix}odom_combined',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02)
            )
            self.carto_x = t.transform.translation.x
            self.carto_y = t.transform.translation.y
            self.carto_yaw = quat_to_yaw(t.transform.rotation)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        # 2. EKF TF: odom_combined -> base_footprint
        try:
            t = self.tf_buffer.lookup_transform(
                f'{self.frame_prefix}odom_combined',
                f'{self.frame_prefix}base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02)
            )
            self.ekf_x = t.transform.translation.x
            self.ekf_y = t.transform.translation.y
            self.ekf_yaw = quat_to_yaw(t.transform.rotation)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        # Compute drift rates
        if self.prev_tf_time is not None:
            dt = now - self.prev_tf_time
            if dt > 0.05:
                # Cartographer drift
                if self.prev_carto_x is not None:
                    cdx = self.carto_x - self.prev_carto_x
                    cdy = self.carto_y - self.prev_carto_y
                    self.carto_drift = math.sqrt(cdx**2 + cdy**2) / dt

                # EKF drift
                if self.prev_ekf_x is not None:
                    edx = self.ekf_x - self.prev_ekf_x
                    edy = self.ekf_y - self.prev_ekf_y
                    self.ekf_drift = math.sqrt(edx**2 + edy**2) / dt

        self.prev_carto_x = self.carto_x
        self.prev_carto_y = self.carto_y
        self.prev_carto_yaw = self.carto_yaw
        self.prev_ekf_x = self.ekf_x
        self.prev_ekf_y = self.ekf_y
        self.prev_ekf_yaw = self.ekf_yaw
        self.prev_tf_time = now

    # ─────────────────────────────────────────────
    #  Publish text diagnostics
    # ─────────────────────────────────────────────
    def publish_diagnostics(self):
        # Identify source
        if self.carto_drift > 0.01 and self.ekf_drift < 0.005:
            source = ">>> CARTOGRAPHER is drifting <<<"
        elif self.ekf_drift > 0.01 and self.carto_drift < 0.005:
            source = ">>> EKF is drifting <<<"
        elif self.carto_drift > 0.01 and self.ekf_drift > 0.01:
            source = ">>> BOTH drifting <<<"
        else:
            source = "Stable"

        diag = (
            f"[{self.robot_ns}] DRIFT SOURCE: {source}\n"
            f"  Cartographer (map->odom): pos=({self.carto_x:.3f}, {self.carto_y:.3f}), "
            f"yaw={math.degrees(self.carto_yaw):.1f}°, drift={self.carto_drift*100:.1f}cm/s\n"
            f"  EKF (odom->base):         pos=({self.ekf_x:.3f}, {self.ekf_y:.3f}), "
            f"yaw={math.degrees(self.ekf_yaw):.1f}°, drift={self.ekf_drift*100:.1f}cm/s\n"
            f"  Camera optical flow: {self.flow_magnitude:.1f}px"
        )

        msg = String()
        msg.data = diag
        self.diag_pub.publish(msg)

        self.get_logger().info(diag, throttle_duration_sec=2.0)

    # ─────────────────────────────────────────────
    #  Camera processing + annotated debug output
    # ─────────────────────────────────────────────
    def image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Image error: {e}", throttle_duration_sec=5.0)
            return

        # Resize to standard size
        frame = cv2.resize(frame, (640, 480))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # --- CROP ROI: BOTTOM 35% ---
        h, w = gray.shape
        roi_top = int(h * 0.65)
        roi = gray[roi_top:h, :]
        
        display = frame.copy()

        # --- Optical flow ---
        if self.prev_gray is not None:
            prev_pts = cv2.goodFeaturesToTrack(self.prev_gray, **self.feature_params)

            if prev_pts is not None and len(prev_pts) >= 5:
                next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                    self.prev_gray, roi, prev_pts, None, **self.lk_params
                )

                if next_pts is not None:
                    good_mask = status.ravel() == 1
                    if np.sum(good_mask) >= 3:
                        good_prev = prev_pts[good_mask].reshape(-1, 2)
                        good_next = next_pts[good_mask].reshape(-1, 2)

                        # Compute flow
                        displacements = good_next - good_prev
                        magnitudes = np.linalg.norm(displacements, axis=1)
                        self.flow_magnitude = float(np.median(magnitudes))

                        # Draw flow vectors on display image
                        for p0, p1 in zip(good_prev, good_next):
                            x0, y0 = int(p0[0]), int(p0[1] + roi_top) # Add roi_top to draw on full image
                            x1, y1 = int(p1[0]), int(p1[1] + roi_top)
                            # Scale vectors for visibility (x5)
                            dx = (x1 - x0) * 5
                            dy = (y1 - y0) * 5
                            cv2.arrowedLine(display, (x0, y0), (x0 + dx, y0 + dy),
                                            (0, 255, 0), 1, tipLength=0.3)
                            cv2.circle(display, (x0, y0), 2, (0, 0, 255), -1)
                        
                        # Draw ROI boundary (Thick Green line)
                        cv2.line(display, (0, roi_top), (w, roi_top), (0, 255, 0), 3)

        self.prev_gray = roi

        # --- Draw HUD overlay ---
        # Background bar
        cv2.rectangle(display, (0, 0), (640, 110), (0, 0, 0), -1)

        # Motion status
        if self.flow_magnitude < 1.5:
            motion_text = "STATIONARY"
            motion_color = (0, 255, 0)
        else:
            motion_text = f"MOVING ({self.flow_magnitude:.1f}px)"
            motion_color = (0, 200, 255)

        cv2.putText(display, f"Camera: {motion_text}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, motion_color, 2)

        # TF drift info
        cv2.putText(display, f"Cartographer drift: {self.carto_drift*100:.1f} cm/s  yaw={math.degrees(self.carto_yaw):.1f} deg",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)
        cv2.putText(display, f"EKF drift:          {self.ekf_drift*100:.1f} cm/s  yaw={math.degrees(self.ekf_yaw):.1f} deg",
                    (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 200), 1)

        # Drift source indicator
        if self.carto_drift > 0.01 and self.ekf_drift < 0.005:
            src = "DRIFT: CARTOGRAPHER"
            src_color = (0, 0, 255)
        elif self.ekf_drift > 0.01 and self.carto_drift < 0.005:
            src = "DRIFT: EKF"
            src_color = (0, 0, 255)
        elif self.carto_drift > 0.01 and self.ekf_drift > 0.01:
            src = "DRIFT: BOTH"
            src_color = (0, 0, 255)
        else:
            src = "OK - STABLE"
            src_color = (0, 255, 0)

        cv2.putText(display, src, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, src_color, 2)

        # Publish annotated image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(display, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f"Publish error: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
