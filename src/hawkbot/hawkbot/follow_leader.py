"""
Follow-the-Leader Node
======================
Robot2 follows Robot1's blue trail in real-time.

Rules:
  - Robot1's trail recording is activated automatically when this mode starts.
  - Robot2 follows the blue trail waypoints in the same direction as Robot1.
  - Robot2 only moves when Robot1 is moving (Robot1 paused => Robot2 paused).
  - Robot2 must stay at least 35 cm center-to-center from Robot1 (leaves 15-20 cm
    physical bumper clearance even when Robot1 rotates on the spot).
  - Uses Pure Pursuit path tracking with strict sequential waypoint progression.

Motion detection: subscribes to leader's cmd_vel topic (the actual velocity
command sent to the robot hardware). This is immune to Cartographer TF noise.

Live Trail Subscription: subscribes directly to leader's /recorded_path topic
so robot2 follows the LIVE blue trail being drawn in real-time.

Strict Sequential Waypoint Progress: Robot2 ONLY evaluates the next 5 waypoints in line,
making it impossible to jump ahead or revisit old waypoints regardless of map geometry.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import rclpy.duration
import math
import json
import os
import time


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FollowLeader(Node):
    def __init__(self):
        super().__init__('follow_leader')

        # ── Parameters ──
        self.declare_parameter('leader_ns', 'robot1')
        self.declare_parameter('follower_ns', 'robot2')
        self.declare_parameter('lookahead_distance', 0.15)
        self.declare_parameter('max_linear_speed', 0.09)
        self.declare_parameter('max_angular_speed', 0.75)
        self.declare_parameter('min_separation', 0.35)       # 35 cm center-to-center (~15-20 cm bumper clearance)
        self.declare_parameter('consume_radius', 0.12)       # waypoint consumed when robot2 passes within 12 cm
        self.declare_parameter('route_file', '/home/hawkbot/ROS2_WS/src/hawkbot/map/recorded_route.json')

        self.leader_ns = self.get_parameter('leader_ns').get_parameter_value().string_value
        self.follower_ns = self.get_parameter('follower_ns').get_parameter_value().string_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.min_separation = self.get_parameter('min_separation').get_parameter_value().double_value
        self.consume_radius = self.get_parameter('consume_radius').get_parameter_value().double_value
        self.route_file = self.get_parameter('route_file').get_parameter_value().string_value

        self.leader_prefix = f"{self.leader_ns}/"
        self.follower_prefix = f"{self.follower_ns}/"

        # ── State ──
        self.is_active = False
        self.localization_ok = True
        self.route_waypoints = []
        self.current_waypoint_idx = 0
        self.first_run = True

        # Waypoint consumption: tracks the highest consumed index
        self.consumed_up_to = -1

        # Leader motion detection via cmd_vel subscription
        self.leader_is_moving = False
        self.leader_last_cmd_time = 0.0
        self.leader_cmd_timeout = 0.5

        # Low-Pass Filter for follower TF pose smoothing (for Pure Pursuit only)
        self.rx_filt = None
        self.ry_filt = None

        # ── TF ──
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Services (start/stop this mode) ──
        self.srv_start = self.create_service(Trigger, 'follow_leader/start', self.handle_start)
        self.srv_stop = self.create_service(Trigger, 'follow_leader/stop', self.handle_stop)

        # ── Service clients to start/stop robot1 trail recording ──
        self.cli_trail_start = self.create_client(
            Trigger, f'/{self.leader_ns}/trajectory_recorder/start')
        self.cli_trail_stop = self.create_client(
            Trigger, f'/{self.leader_ns}/trajectory_recorder/stop_and_save')

        # ── Subscribers ──
        self.loc_sub = self.create_subscription(
            Bool,
            f'/{self.follower_ns}/localization_ok',
            self.loc_callback,
            10
        )
        # Leader cmd_vel: the DEFINITIVE source of truth for leader motion
        self.leader_cmd_sub = self.create_subscription(
            Twist,
            f'/{self.leader_ns}/cmd_vel',
            self.leader_cmd_callback,
            10
        )

        # Leader live path subscription: receives live blue trail from leader in real-time
        self.live_path_sub = self.create_subscription(
            Path,
            f'/{self.leader_ns}/recorded_path',
            self.live_path_callback,
            10
        )

        # ── Publishers ──
        self.cmd_pub = self.create_publisher(
            Twist, f'/{self.follower_ns}/cmd_vel', 10)
        self.marker_pub = self.create_publisher(
            Marker, f'/{self.follower_ns}/follow_leader_marker', 10)
        self.status_pub = self.create_publisher(
            Bool, 'follow_leader/active', 10)

        # ── Control Loop (20 Hz) ──
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Follow-the-Leader node initialized: "
            f"leader={self.leader_ns}, follower={self.follower_ns}, "
            f"min_separation={self.min_separation}m, consume_radius={self.consume_radius}m")

    # ─────────────── Callbacks ───────────────

    def loc_callback(self, msg):
        self.localization_ok = msg.data

    def leader_cmd_callback(self, msg):
        """Track leader's cmd_vel to determine if it is actively moving."""
        vel_linear = abs(msg.linear.x) + abs(msg.linear.y)
        vel_angular = abs(msg.angular.z)

        if vel_linear > 0.005 or vel_angular > 0.01:
            self.leader_last_cmd_time = time.monotonic()
            if not self.leader_is_moving:
                self.leader_is_moving = True
                self.get_logger().info(
                    f"[{self.leader_ns}] Leader is MOVING (cmd_vel detected)",
                    throttle_duration_sec=2.0)

    def live_path_callback(self, msg):
        """Receive live path recorded by leader's trajectory_recorder in real-time."""
        if not self.is_active:
            return

        if not msg.poses:
            return

        live_waypoints = []
        for idx, pose_stamped in enumerate(msg.poses):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            yaw = quaternion_to_yaw(pose_stamped.pose.orientation)
            live_waypoints.append({
                "id": idx,
                "x": px,
                "y": py,
                "yaw": yaw
            })

        self.route_waypoints = live_waypoints
        if self.first_run and len(live_waypoints) > 0:
            self.get_logger().info(
                f"[{self.follower_ns}] Live trail stream connected from {self.leader_ns} ({len(live_waypoints)} waypoints)")

    # ─────────────── Service Handlers ───────────────

    def handle_start(self, request, response):
        """Activate follow-the-leader mode."""
        self.is_active = True
        self.route_waypoints = []
        self.current_waypoint_idx = 0
        self.first_run = True
        self.consumed_up_to = -1
        self.rx_filt = None
        self.ry_filt = None
        self.leader_is_moving = False
        self.leader_last_cmd_time = 0.0

        if self.cli_trail_start.wait_for_service(timeout_sec=3.0):
            future = self.cli_trail_start.call_async(Trigger.Request())
            future.add_done_callback(self._trail_start_done)
        else:
            self.get_logger().warn("Trail recorder service not available, starting without it.")

        response.success = True
        response.message = (
            f"Mode Follow-the-Leader activé : {self.follower_ns} suit la traînée bleue de {self.leader_ns} EN DIRECT.")
        self.get_logger().info(response.message)
        return response

    def handle_stop(self, request, response):
        """Deactivate follow-the-leader mode."""
        self.is_active = False
        self.stop_follower()
        self.clear_markers()

        if self.cli_trail_stop.wait_for_service(timeout_sec=3.0):
            future = self.cli_trail_stop.call_async(Trigger.Request())
            future.add_done_callback(self._trail_stop_done)

        response.success = True
        response.message = f"Mode Follow-the-Leader désactivé pour {self.follower_ns}."
        self.get_logger().info(response.message)
        return response

    def _trail_start_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Live trail recording started for {self.leader_ns}")
            else:
                self.get_logger().warn(f"Trail recording start failed: {res.message}")
        except Exception as e:
            self.get_logger().error(f"Trail start service error: {e}")

    def _trail_stop_done(self, future):
        try:
            res = future.result()
            if res.success:
                self.get_logger().info(f"Trail saved for {self.leader_ns}: {res.message}")
        except Exception as e:
            self.get_logger().error(f"Trail stop service error: {e}")

    # ─────────────── Motion Control ───────────────

    def stop_follower(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def clear_markers(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.follower_ns}_follow_target"
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)

    def _get_robot_pose(self, frame_prefix):
        """Get robot position (x, y, yaw) in map frame."""
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'map',
                f'{frame_prefix}base_footprint',
                now,
                timeout=rclpy.duration.Duration(seconds=0.08)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            yaw = quaternion_to_yaw(trans.transform.rotation)
            return x, y, yaw
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _is_leader_moving(self):
        """Check if the leader is currently moving based on cmd_vel messages."""
        if self.leader_last_cmd_time == 0.0:
            self.leader_is_moving = False
            return False

        elapsed = time.monotonic() - self.leader_last_cmd_time
        if elapsed > self.leader_cmd_timeout:
            if self.leader_is_moving:
                self.leader_is_moving = False
                self.get_logger().info(
                    f"[{self.leader_ns}] Leader is STOPPED (no cmd_vel for {elapsed:.1f}s)",
                    throttle_duration_sec=2.0)
            return False

        return True

    def _consume_waypoints(self, fx_raw, fy_raw):
        """Mark waypoints as consumed sequentially when robot2 passes close enough.
        
        Uses STRICT forward search window (checking only the next 3 waypoints).
        """
        n = len(self.route_waypoints)
        if n == 0:
            return

        search_start = max(0, self.consumed_up_to + 1)
        search_end = min(search_start + 4, n)

        new_consumed = self.consumed_up_to
        for idx in range(search_start, search_end):
            wp = self.route_waypoints[idx]
            d = math.hypot(wp['x'] - fx_raw, wp['y'] - fy_raw)
            if d < self.consume_radius:
                new_consumed = max(new_consumed, idx)

        if new_consumed > self.consumed_up_to:
            self.consumed_up_to = new_consumed

    def control_loop(self):
        """Main 20 Hz control loop."""
        status_msg = Bool()
        status_msg.data = self.is_active
        self.status_pub.publish(status_msg)

        if not self.is_active:
            return

        # Safety: localization check
        if not self.localization_ok:
            self.stop_follower()
            self.get_logger().warn(
                f"PAUSE SÉCURITÉ [{self.follower_ns}]: Localisation perdue.",
                throttle_duration_sec=2.0)
            return

        # No live route waypoints received yet — wait for leader to move and record
        if not self.route_waypoints:
            self.stop_follower()
            return

        # ── 1. Check if leader is moving (via cmd_vel) ──
        if not self._is_leader_moving():
            self.stop_follower()
            return

        # ── 2. Get leader pose (RAW) ──
        leader_pose = self._get_robot_pose(self.leader_prefix)
        if leader_pose is None:
            self.get_logger().warn(
                f"Cannot get {self.leader_ns} TF pose", throttle_duration_sec=2.0)
            return
        lx, ly, _ = leader_pose

        # ── 3. Get follower pose (RAW) ──
        follower_pose = self._get_robot_pose(self.follower_prefix)
        if follower_pose is None:
            self.get_logger().warn(
                f"Cannot get {self.follower_ns} TF pose", throttle_duration_sec=2.0)
            return
        fx_raw, fy_raw, fyaw = follower_pose

        # ── 4. Check minimum separation (35cm center-to-center = 15-20cm physical bumper gap) ──
        dist_to_leader = math.hypot(lx - fx_raw, ly - fy_raw)
        if dist_to_leader < self.min_separation:
            self.stop_follower()
            self.get_logger().info(
                f"[{self.follower_ns}] Safety distance reached ({dist_to_leader:.2f}m < {self.min_separation}m), pausing.",
                throttle_duration_sec=2.0)
            return

        # ── 5. Apply low-pass filter to follower pose (for Pure Pursuit steering) ──
        if self.rx_filt is None or self.ry_filt is None:
            self.rx_filt = fx_raw
            self.ry_filt = fy_raw
        else:
            self.rx_filt = 0.25 * fx_raw + 0.75 * self.rx_filt
            self.ry_filt = 0.25 * fy_raw + 0.75 * self.ry_filt

        fx_use = self.rx_filt
        fy_use = self.ry_filt

        # ── 6. Consume passed waypoints sequentially ──
        self._consume_waypoints(fx_raw, fy_raw)

        # ── 7. Strict Sequential Nearest Search ──
        n = len(self.route_waypoints)
        valid_start = self.consumed_up_to + 1
        if valid_start >= n:
            self.stop_follower()
            return

        # ONLY search the next 5 waypoints in strict sequence! Never look further ahead!
        search_end = min(valid_start + 5, n)
        min_dist = float('inf')
        nearest_idx = valid_start

        for idx in range(valid_start, search_end):
            wp = self.route_waypoints[idx]
            d = math.hypot(wp['x'] - fx_use, wp['y'] - fy_use)
            if d < min_dist:
                min_dist = d
                nearest_idx = idx

        self.current_waypoint_idx = nearest_idx

        # ── 8. Find leader's nearest waypoint index (to cap follower) ──
        leader_nearest_idx = nearest_idx
        leader_min_dist = float('inf')
        for idx in range(valid_start, n):
            wp = self.route_waypoints[idx]
            d = math.hypot(wp['x'] - lx, wp['y'] - ly)
            if d < leader_min_dist:
                leader_min_dist = d
                leader_nearest_idx = idx

        # ── 9. Find target waypoint at lookahead distance ──
        target_idx = nearest_idx
        accumulated_dist = 0.0
        while accumulated_dist < self.lookahead_distance:
            next_idx = target_idx + 1
            if next_idx >= n:
                break
            if next_idx >= leader_nearest_idx and leader_nearest_idx > nearest_idx:
                break
            wp_a = self.route_waypoints[target_idx]
            wp_b = self.route_waypoints[next_idx]
            accumulated_dist += math.hypot(wp_b['x'] - wp_a['x'], wp_b['y'] - wp_a['y'])
            target_idx = next_idx

        target_wp = self.route_waypoints[target_idx]

        # ── 10. Transform target to follower local frame ──
        dx = target_wp['x'] - fx_use
        dy = target_wp['y'] - fy_use
        cos_yaw = math.cos(-fyaw)
        sin_yaw = math.sin(-fyaw)

        x_local = dx * cos_yaw - dy * sin_yaw
        y_local = dx * sin_yaw + dy * cos_yaw
        Ld = math.hypot(x_local, y_local)

        if Ld < 0.01:
            Ld = 0.01

        # ── 11. Pure Pursuit curvature & velocity ──
        gamma = (2.0 * y_local) / (Ld * Ld)
        angle_err = math.atan2(y_local, x_local)

        twist = Twist()

        if abs(angle_err) > 0.4:
            twist.linear.x = 0.0
            twist.angular.z = (self.max_angular_speed * 0.7
                               if angle_err > 0
                               else -self.max_angular_speed * 0.7)
        else:
            speed_factor = math.cos(angle_err)
            twist.linear.x = self.max_linear_speed * max(0.3, speed_factor)
            twist.angular.z = twist.linear.x * gamma

            twist.linear.x = min(max(twist.linear.x, 0.0), self.max_linear_speed)
            twist.angular.z = min(max(twist.angular.z, -self.max_angular_speed), self.max_angular_speed)

        self.cmd_pub.publish(twist)

        # ── 12. Publish target marker (orange sphere) ──
        self.publish_target_marker(target_wp['x'], target_wp['y'])

    def publish_target_marker(self, tx, ty):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.follower_ns}_follow_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 0.5   # Orange target sphere
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = float(tx)
        marker.pose.position.y = float(ty)
        marker.pose.position.z = 0.1
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FollowLeader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_follower()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
