#!/usr/bin/env python3
"""Last safety gate between Nav2 and the Ranger base.

The gate starts disarmed and continually publishes a zero command until all
localization and chassis checks are healthy and an operator explicitly arms it.
"""

import math
import re
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from ranger_msgs.msg import SystemState
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


@dataclass
class UltrasonicLatch:
    """Fail-safe hysteresis for one intermittent ultrasonic sensor."""

    blocked: bool = False
    last_blocked_sec: float = -math.inf
    clear_count: int = 0
    clear_kind: str = ""

    def update(
            self, value, now_sec, stop_distance, clear_distance,
            hold_sec, finite_clear_samples, no_echo_clear_samples):
        finite_clear_samples = max(1, int(finite_clear_samples))
        no_echo_clear_samples = max(
            finite_clear_samples, int(no_echo_clear_samples))

        if not math.isfinite(value):
            if not (math.isinf(value) and value > 0.0):
                self.blocked = True
                self.last_blocked_sec = now_sec
                self.clear_count = 0
                self.clear_kind = ""
                return
            clear_kind = "no_echo"
            required_samples = no_echo_clear_samples
        elif value <= stop_distance:
            self.blocked = True
            self.last_blocked_sec = now_sec
            self.clear_count = 0
            self.clear_kind = ""
            return
        elif value >= clear_distance:
            clear_kind = "finite"
            required_samples = finite_clear_samples
        else:
            # Stay in the current hysteresis state between stop and clear.
            self.clear_count = 0
            self.clear_kind = ""
            return

        if not self.blocked:
            return
        if now_sec - self.last_blocked_sec < hold_sec:
            self.clear_count = 0
            self.clear_kind = ""
            return
        if self.clear_kind != clear_kind:
            self.clear_count = 0
            self.clear_kind = clear_kind
        self.clear_count += 1
        if self.clear_count >= required_samples:
            self.blocked = False
            self.clear_count = 0
            self.clear_kind = ""


class NavMotionSafetyGate(Node):
    def __init__(self):
        super().__init__("nav_motion_safety_gate")
        self.input_topic = self.declare_parameter(
            "input_topic", "/cmd_vel_nav_smoothed").value
        self.output_topic = self.declare_parameter("output_topic", "/cmd_vel").value
        self.arm_service = self.declare_parameter(
            "arm_service", "/set_nav_motion_enabled").value
        self.max_fitness = float(self.declare_parameter("max_fitness", 0.10).value)
        self.localization_timeout = float(
            self.declare_parameter("localization_timeout", 0.50).value)
        self.odom_timeout = float(
            self.declare_parameter("odom_timeout", 0.30).value)
        self.chassis_timeout = float(
            self.declare_parameter("chassis_timeout", 0.50).value)
        self.command_timeout = float(
            self.declare_parameter("command_timeout", 0.30).value)
        self.obstacle_status_timeout = float(
            self.declare_parameter("obstacle_status_timeout", 0.50).value)
        self.max_obstacle_age = float(
            self.declare_parameter("max_obstacle_age", 1.00).value)
        self.max_linear = float(self.declare_parameter("max_linear", 0.08).value)
        self.max_angular = float(self.declare_parameter("max_angular", 0.18).value)
        self.minimum_linear_for_turn = float(self.declare_parameter(
            "minimum_linear_for_turn", 0.012).value)
        self.max_motion_curvature = float(self.declare_parameter(
            "max_motion_curvature", 2.25).value)
        self.curvature_slack = float(self.declare_parameter(
            "curvature_slack", 0.015).value)
        self.allow_small_in_place_rotation = bool(self.declare_parameter(
            "allow_small_in_place_rotation", True).value)
        self.max_in_place_rotation = float(self.declare_parameter(
            "max_in_place_rotation", 3.25).value)
        self.max_in_place_angular = float(self.declare_parameter(
            "max_in_place_angular", 0.12).value)
        self.small_spin_reset_distance = float(self.declare_parameter(
            "small_spin_reset_distance", 0.10).value)
        self.reverse_straight_only = bool(self.declare_parameter(
            "reverse_straight_only", True).value)
        self.max_reverse_angular = float(self.declare_parameter(
            "max_reverse_angular", 0.015).value)
        self.plan_topic = str(self.declare_parameter(
            "plan_topic", "/direct_reverse_plan").value)
        self.plan_timeout = float(self.declare_parameter(
            "plan_timeout", 3.0).value)
        self.straight_path_min_length = float(self.declare_parameter(
            "straight_path_min_length", 0.05).value)
        self.straight_path_max_lateral_error = float(self.declare_parameter(
            "straight_path_max_lateral_error", 0.10).value)
        self.straight_path_max_heading_span = float(self.declare_parameter(
            "straight_path_max_heading_span", 0.12).value)
        self.straight_path_max_length_ratio = float(self.declare_parameter(
            "straight_path_max_length_ratio", 1.03).value)
        self.straight_path_confirmations_required = max(
            1, int(self.declare_parameter(
                "straight_path_confirmations_required", 2).value))
        self.ultrasonic_enabled = bool(self.declare_parameter(
            "ultrasonic_enabled", True).value)
        self.ultrasonic_motion_direction = str(self.declare_parameter(
            "ultrasonic_motion_direction", "reverse").value).lower()
        self.ultrasonic_timeout = float(self.declare_parameter(
            "ultrasonic_timeout", 0.75).value)
        self.ultrasonic_stop_distance = float(self.declare_parameter(
            "ultrasonic_stop_distance", 0.22).value)
        self.ultrasonic_clear_distance = float(self.declare_parameter(
            "ultrasonic_clear_distance", 0.35).value)
        self.ultrasonic_turn_allow_distance = float(
            self.declare_parameter(
                "ultrasonic_turn_allow_distance", 0.25).value)
        self.ultrasonic_self_echo_enabled = bool(self.declare_parameter(
            "ultrasonic_self_echo_enabled", True).value)
        self.ultrasonic_self_echo_min_distance = float(
            self.declare_parameter(
                "ultrasonic_self_echo_min_distance", 0.16).value)
        self.ultrasonic_self_echo_max_distance = float(
            self.declare_parameter(
                "ultrasonic_self_echo_max_distance", 0.22).value)
        self.ultrasonic_sensor_to_rear_edge = float(self.declare_parameter(
            "ultrasonic_sensor_to_rear_edge", 0.20).value)
        self.ultrasonic_required_tail_clearance = float(
            self.declare_parameter(
                "ultrasonic_required_tail_clearance", 0.02).value)
        self.ultrasonic_braking_margin = float(self.declare_parameter(
            "ultrasonic_braking_margin", 0.0).value)
        self.ultrasonic_noise_margin = float(self.declare_parameter(
            "ultrasonic_noise_margin", 0.0).value)
        self.ultrasonic_block_hold_sec = float(self.declare_parameter(
            "ultrasonic_block_hold_sec", 1.50).value)
        self.ultrasonic_clear_samples_required = max(
            1, int(self.declare_parameter(
                "ultrasonic_clear_samples_required", 3).value))
        self.ultrasonic_no_echo_clear_samples_required = max(
            self.ultrasonic_clear_samples_required,
            int(self.declare_parameter(
                "ultrasonic_no_echo_clear_samples_required", 8).value))
        self.ultrasonic_turn_guard_enabled = bool(self.declare_parameter(
            "ultrasonic_turn_guard_enabled", True).value)
        self.ultrasonic_reverse_speed_taper_enabled = bool(
            self.declare_parameter(
                "ultrasonic_reverse_speed_taper_enabled", True).value)
        self.operator_heartbeat_required = bool(self.declare_parameter(
            "operator_heartbeat_required", True).value)
        self.operator_heartbeat_timeout = float(self.declare_parameter(
            "operator_heartbeat_timeout", 2.0).value)
        self.goal_lease_required = bool(self.declare_parameter(
            "goal_lease_required", True).value)
        self.goal_lease_timeout = float(self.declare_parameter(
            "goal_lease_timeout", 0.75).value)

        required_stop_distance = (
            self.ultrasonic_sensor_to_rear_edge
            + self.ultrasonic_required_tail_clearance
            + self.ultrasonic_braking_margin
            + self.ultrasonic_noise_margin
        )
        if self.ultrasonic_stop_distance + 1.0e-6 < required_stop_distance:
            raise ValueError(
                "ultrasonic_stop_distance is smaller than the measured "
                "rear-setback + tail-clearance + braking/noise margins: "
                f"{self.ultrasonic_stop_distance:.3f} < "
                f"{required_stop_distance:.3f} m")
        if self.ultrasonic_clear_distance <= self.ultrasonic_stop_distance:
            raise ValueError(
                "ultrasonic_clear_distance must exceed "
                "ultrasonic_stop_distance")
        if self.ultrasonic_turn_allow_distance <= self.ultrasonic_stop_distance:
            raise ValueError(
                "ultrasonic_turn_allow_distance must exceed the hard-stop "
                "distance")
        if self.ultrasonic_self_echo_enabled and not (
                0.0 <= self.ultrasonic_self_echo_min_distance
                < self.ultrasonic_self_echo_max_distance
                <= self.ultrasonic_stop_distance):
            raise ValueError(
                "ultrasonic self-echo band must be ordered and remain inside "
                "the blocked stop range")

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.ready_pub = self.create_publisher(Bool, "/nav_motion_ready", 1)
        self.armed_pub = self.create_publisher(Bool, "/nav_motion_armed", 1)
        self.status_pub = self.create_publisher(String, "/nav_motion_status", 1)
        self.ultrasonic_status_pub = self.create_publisher(
            String, "/rear_ultrasonic_safety_status", 1)
        self.ultrasonic_reverse_allowed_pub = self.create_publisher(
            Bool, "/rear_ultrasonic_reverse_allowed", 1)
        self.ultrasonic_left_turn_allowed_pub = self.create_publisher(
            Bool, "/rear_ultrasonic_left_turn_allowed", 1)
        self.ultrasonic_right_turn_allowed_pub = self.create_publisher(
            Bool, "/rear_ultrasonic_right_turn_allowed", 1)
        self.operator_link_status_pub = self.create_publisher(
            String, "/hn_nav_operator_link_status", 1)
        self.path_policy_pub = self.create_publisher(
            String, "/nav_reverse_path_policy", 1)
        self.create_subscription(Twist, self.input_topic, self.on_command, 10)
        self.create_subscription(Path, self.plan_topic, self.on_plan, 10)
        self.create_subscription(Odometry, "/relocalization_odom", self.on_localization, sensor_qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, sensor_qos)
        self.create_subscription(SystemState, "/system_state", self.on_system_state, 10)
        self.create_subscription(
            String, "/nav_obstacle_cloud_status", self.on_obstacle_status, 10)
        self.create_subscription(
            Bool, "/ultrasonic/healthy", self.on_ultrasonic_health, 10)
        self.create_subscription(
            Bool,
            "/hn_nav_operator_heartbeat",
            self.on_operator_heartbeat,
            sensor_qos,
        )
        self.create_subscription(
            Bool,
            "/aligned_goal_active",
            self.on_goal_active,
            reliable_qos,
        )
        self.create_subscription(
            Range,
            "/ultrasonic/sensor_1/range",
            lambda msg: self.on_ultrasonic_range(0, msg),
            sensor_qos,
        )
        self.create_subscription(
            Range,
            "/ultrasonic/sensor_2/range",
            lambda msg: self.on_ultrasonic_range(1, msg),
            sensor_qos,
        )
        self.create_service(SetBool, self.arm_service, self.on_set_arm)

        self.last_command = None
        self.last_command_time = None
        self.last_plan_time = None
        self.plan_is_straight = False
        self.straight_path_confirmations = 0
        self.plan_metrics = "waiting for path"
        self.last_path_policy = None
        self.last_localization_time = None
        self.last_fitness = math.inf
        self.last_odom_time = None
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.spin_active = False
        self.spin_last_yaw = None
        self.spin_origin_x = None
        self.spin_origin_y = None
        self.spin_accumulated = 0.0
        self.spin_limit_reached = False
        self.last_system_time = None
        self.last_obstacle_status_time = None
        self.last_obstacle_age = math.inf
        self.last_ultrasonic_health_time = None
        self.ultrasonic_healthy = False
        self.last_operator_heartbeat_time = None
        self.operator_present = False
        self.last_goal_lease_time = None
        self.goal_active = False
        self.last_ultrasonic_times = [None, None]
        self.ultrasonic_ranges = [math.nan, math.nan]
        self.ultrasonic_min_ranges = [math.nan, math.nan]
        self.ultrasonic_max_ranges = [math.nan, math.nan]
        self.ultrasonic_latches = [UltrasonicLatch(), UltrasonicLatch()]
        self.ultrasonic_turn_latches = [
            UltrasonicLatch(), UltrasonicLatch()]
        self.system_error = None
        self.vehicle_state = None
        self.control_mode = None
        self.armed = False
        self.last_status = "starting"
        # AgileX requires motion frames at >= 50 Hz.  This includes zero-speed
        # hold frames while disarmed; a 10 Hz final gate lets the chassis drop
        # out of commanded mode and makes ROS commands appear to be ignored.
        self.create_timer(0.02, self.tick)
        self.create_timer(1.0, self.publish_status_heartbeat)
        self.get_logger().info(
            "motion gate starts DISARMED: "
            f"fitness<={self.max_fitness:.3f}, "
            f"localization<={self.localization_timeout:.2f}s, "
            f"cmd<={self.command_timeout:.2f}s, "
            f"obstacle_age<={self.max_obstacle_age:.2f}s, "
            f"rear_sonar={self.ultrasonic_motion_direction} "
            f"stop<={self.ultrasonic_stop_distance:.2f}m "
            f"clear>={self.ultrasonic_clear_distance:.2f}m, "
            f"turn_allow>{self.ultrasonic_turn_allow_distance:.2f}m, "
            f"self_echo=[{self.ultrasonic_self_echo_min_distance:.2f},"
            f"{self.ultrasonic_self_echo_max_distance:.2f}]m "
            f"sensor_to_tail={self.ultrasonic_sensor_to_rear_edge:.2f}m "
            f"tail_clearance>={self.ultrasonic_required_tail_clearance:.2f}m "
            f"hold={self.ultrasonic_block_hold_sec:.2f}s "
            f"clear_samples={self.ultrasonic_clear_samples_required}/"
            f"{self.ultrasonic_no_echo_clear_samples_required}, "
            f"turn_guard={self.ultrasonic_turn_guard_enabled}, "
            f"reverse_speed_taper="
            f"{self.ultrasonic_reverse_speed_taper_enabled}, "
            f"operator_heartbeat<={self.operator_heartbeat_timeout:.2f}s, "
            f"goal_lease<={self.goal_lease_timeout:.2f}s, "
            f"limits=({self.max_linear:.2f} m/s, {self.max_angular:.2f} rad/s), "
            f"small_spin<={math.degrees(self.max_in_place_rotation):.1f}deg "
            f"at<={self.max_in_place_angular:.2f}rad/s "
            f"reset_after={self.small_spin_reset_distance:.2f}m, "
            f"curvature<={self.max_motion_curvature:.2f} 1/m, "
            f"reverse_straight_only={self.reverse_straight_only} "
            f"reverse_angular<={self.max_reverse_angular:.3f}rad/s "
            f"plan={self.plan_topic} timeout={self.plan_timeout:.1f}s "
            f"straight_limits=(lateral<={self.straight_path_max_lateral_error:.2f}m, "
            f"heading_span<={self.straight_path_max_heading_span:.2f}rad, "
            f"length_ratio<={self.straight_path_max_length_ratio:.2f}, "
            f"confirmations={self.straight_path_confirmations_required})")

    def on_command(self, msg):
        self.last_command = msg
        self.last_command_time = self.get_clock().now()

    @staticmethod
    def wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def classify_plan(self, msg):
        points = []
        for pose in msg.poses:
            point = (float(pose.pose.position.x), float(pose.pose.position.y))
            if not points or math.hypot(
                    point[0] - points[-1][0],
                    point[1] - points[-1][1]) >= 0.01:
                points.append(point)
        if len(points) < 2:
            return False, "points<2"

        segment_lengths = []
        headings = []
        for first, second in zip(points, points[1:]):
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            length = math.hypot(dx, dy)
            if length < 1.0e-6:
                continue
            segment_lengths.append(length)
            headings.append(math.atan2(dy, dx))
        path_length = sum(segment_lengths)
        if path_length < self.straight_path_min_length or not headings:
            return False, (
                f"length={path_length:.3f}m<"
                f"{self.straight_path_min_length:.3f}m")

        start = points[0]
        end = points[-1]
        chord_x = end[0] - start[0]
        chord_y = end[1] - start[1]
        chord_length = math.hypot(chord_x, chord_y)
        if chord_length < 1.0e-6:
            return False, "zero chord"
        max_lateral = max(
            abs(chord_x * (start[1] - point[1])
                - (start[0] - point[0]) * chord_y) / chord_length
            for point in points
        )
        base_heading = headings[0]
        relative_headings = [
            self.wrap(heading - base_heading) for heading in headings]
        heading_span = max(relative_headings) - min(relative_headings)
        length_ratio = path_length / chord_length
        straight = (
            max_lateral <= self.straight_path_max_lateral_error
            and heading_span <= self.straight_path_max_heading_span
            and length_ratio <= self.straight_path_max_length_ratio
        )
        metrics = (
            f"points={len(points)} length={path_length:.3f}m "
            f"lateral={max_lateral:.3f}m heading_span={heading_span:.3f}rad "
            f"length_ratio={length_ratio:.3f}")
        return straight, metrics

    def publish_path_policy(self):
        confirmed = (
            self.plan_is_straight
            and self.straight_path_confirmations
            >= self.straight_path_confirmations_required
        )
        age = self.elapsed(self.last_plan_time)
        state = "STRAIGHT_REVERSE_ALLOWED" if confirmed else "FORWARD_ONLY"
        text = (
            f"{state} confirmations={self.straight_path_confirmations}/"
            f"{self.straight_path_confirmations_required} age={age:.3f}s "
            f"{self.plan_metrics}")
        self.path_policy_pub.publish(String(data=text))
        if text.split(" age=", 1)[0] != self.last_path_policy:
            self.get_logger().info(text)
            self.last_path_policy = text.split(" age=", 1)[0]

    def on_plan(self, msg):
        straight, metrics = self.classify_plan(msg)
        self.last_plan_time = self.get_clock().now()
        self.plan_is_straight = straight
        self.plan_metrics = metrics
        if straight:
            self.straight_path_confirmations = min(
                self.straight_path_confirmations + 1,
                self.straight_path_confirmations_required)
        else:
            self.straight_path_confirmations = 0
        self.publish_path_policy()

    def on_localization(self, msg):
        self.last_localization_time = self.get_clock().now()
        self.last_fitness = float(msg.pose.covariance[0])

    def reset_small_spin_budget(self):
        self.spin_active = False
        self.spin_last_yaw = self.odom_yaw
        self.spin_origin_x = self.odom_x
        self.spin_origin_y = self.odom_y
        self.spin_accumulated = 0.0
        self.spin_limit_reached = False

    def on_odom(self, msg):
        self.last_odom_time = self.get_clock().now()
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z),
        )
        self.odom_x = float(position.x)
        self.odom_y = float(position.y)
        self.odom_yaw = yaw
        if self.spin_active and self.spin_last_yaw is not None:
            self.spin_accumulated += abs(
                self.wrap(yaw - self.spin_last_yaw))
            if self.spin_accumulated >= self.max_in_place_rotation:
                self.spin_limit_reached = True
        self.spin_last_yaw = yaw
        if (
                self.spin_active
                and self.spin_origin_x is not None
                and math.hypot(
                    self.odom_x - self.spin_origin_x,
                    self.odom_y - self.spin_origin_y,
                ) >= self.small_spin_reset_distance):
            self.get_logger().info(
                "small-spin budget reset after "
                f"{self.small_spin_reset_distance:.2f}m translation")
            self.reset_small_spin_budget()

    def on_system_state(self, msg):
        self.last_system_time = self.get_clock().now()
        self.system_error = int(msg.error_code)
        self.vehicle_state = int(msg.vehicle_state)
        self.control_mode = int(msg.control_mode)

    def on_obstacle_status(self, msg):
        match = re.search(r"age=([0-9.]+)s", msg.data)
        self.last_obstacle_status_time = self.get_clock().now()
        self.last_obstacle_age = float(match.group(1)) if match else math.inf

    def on_ultrasonic_health(self, msg):
        self.last_ultrasonic_health_time = self.get_clock().now()
        self.ultrasonic_healthy = bool(msg.data)

    def on_operator_heartbeat(self, msg):
        self.last_operator_heartbeat_time = self.get_clock().now()
        self.operator_present = bool(msg.data)

    def on_goal_active(self, msg):
        self.last_goal_lease_time = self.get_clock().now()
        self.goal_active = bool(msg.data)

    def on_ultrasonic_range(self, index, msg):
        value = float(msg.range)
        now = self.get_clock().now()
        self.last_ultrasonic_times[index] = now
        self.ultrasonic_ranges[index] = value
        self.ultrasonic_min_ranges[index] = float(msg.min_range)
        self.ultrasonic_max_ranges[index] = float(msg.max_range)
        self.ultrasonic_latches[index].update(
            value,
            now.nanoseconds / 1e9,
            self.ultrasonic_stop_distance,
            self.ultrasonic_clear_distance,
            self.ultrasonic_block_hold_sec,
            self.ultrasonic_clear_samples_required,
            self.ultrasonic_no_echo_clear_samples_required,
        )
        self.ultrasonic_turn_latches[index].update(
            value,
            now.nanoseconds / 1e9,
            self.ultrasonic_turn_allow_distance,
            self.ultrasonic_turn_allow_distance,
            self.ultrasonic_block_hold_sec,
            self.ultrasonic_clear_samples_required,
            self.ultrasonic_no_echo_clear_samples_required,
        )

    def is_ultrasonic_self_echo(self, value):
        return (
            self.ultrasonic_self_echo_enabled
            and math.isfinite(value)
            and self.ultrasonic_self_echo_min_distance
            <= value <= self.ultrasonic_self_echo_max_distance
        )

    def classify_ultrasonic(self, index):
        value = self.ultrasonic_ranges[index]
        latch = self.ultrasonic_latches[index]
        if math.isnan(value) or (math.isinf(value) and value < 0.0):
            return "INVALID"
        if math.isinf(value) and value > 0.0:
            return "NO_ECHO_CLEAR" if not latch.blocked else "NO_ECHO_HELD"
        if self.is_ultrasonic_self_echo(value):
            return "SELF_ECHO"
        if value < self.ultrasonic_self_echo_min_distance:
            return "OBSTACLE_NEAR"
        if value <= self.ultrasonic_stop_distance:
            return "OBSTACLE"
        if value < self.ultrasonic_clear_distance:
            return "HYSTERESIS_BLOCKED" if latch.blocked else "HYSTERESIS_CLEAR"
        return "CLEAR" if not latch.blocked else "CLEAR_PENDING"

    def classify_turn_ultrasonic(self, index):
        value = self.ultrasonic_ranges[index]
        latch = self.ultrasonic_turn_latches[index]
        if math.isnan(value) or (math.isinf(value) and value < 0.0):
            return "INVALID"
        if math.isinf(value) and value > 0.0:
            return "NO_ECHO_CLEAR" if not latch.blocked else "NO_ECHO_HELD"
        if self.is_ultrasonic_self_echo(value):
            return "SELF_ECHO"
        if value <= self.ultrasonic_turn_allow_distance:
            return "TURN_TOO_CLOSE"
        return "TURN_CLEAR" if not latch.blocked else "TURN_CLEAR_PENDING"

    def rear_reverse_allowed(self):
        return self.ultrasonic_sensor_allowed(
            0, self.ultrasonic_latches) and self.ultrasonic_sensor_allowed(
                1, self.ultrasonic_latches)

    def ultrasonic_sensor_allowed(self, index, latches):
        if not self.ultrasonic_enabled or not self.ultrasonic_healthy:
            return False
        if self.elapsed(self.last_ultrasonic_health_time) > self.ultrasonic_timeout:
            return False
        stamp = self.last_ultrasonic_times[index]
        return (
            self.elapsed(stamp) <= self.ultrasonic_timeout
            and not latches[index].blocked
        )

    def rear_left_turn_allowed(self):
        # Turning in a narrow aisle is permitted only when both rear sectors
        # are clear; either rear corner may sweep toward nearby structure.
        return self.rear_turn_allowed()

    def rear_right_turn_allowed(self):
        return self.rear_turn_allowed()

    def rear_turn_allowed(self):
        return self.ultrasonic_sensor_allowed(
            0, self.ultrasonic_turn_latches) and \
            self.ultrasonic_sensor_allowed(
                1, self.ultrasonic_turn_latches)

    def ultrasonic_blocking_sides(self, latches):
        if not self.ultrasonic_enabled:
            return ("LEFT", "RIGHT")
        if (
                not self.ultrasonic_healthy
                or self.elapsed(self.last_ultrasonic_health_time)
                > self.ultrasonic_timeout):
            return ("LEFT", "RIGHT")
        blocked = []
        for index, name in enumerate(("LEFT", "RIGHT")):
            if (
                    self.elapsed(self.last_ultrasonic_times[index])
                    > self.ultrasonic_timeout
                    or latches[index].blocked):
                blocked.append(name)
        return tuple(blocked)

    def rear_reverse_speed_scale(self):
        """Linearly taper reverse speed between hard-stop and clear ranges."""
        if not self.rear_reverse_allowed():
            return 0.0
        if not self.ultrasonic_reverse_speed_taper_enabled:
            return 1.0
        span = self.ultrasonic_clear_distance - self.ultrasonic_stop_distance
        if span <= 1.0e-6:
            return 0.0
        scale = 1.0
        for value in self.ultrasonic_ranges:
            if math.isinf(value) and value > 0.0:
                continue
            if not math.isfinite(value):
                return 0.0
            scale = min(
                scale,
                max(0.0, min(
                    1.0,
                    (value - self.ultrasonic_stop_distance) / span,
                )),
            )
        return scale

    @staticmethod
    def guarded_ultrasonic_indices(
            linear_x, angular_z, motion_direction, turn_guard_enabled):
        indices = set()
        if motion_direction == "reverse" and linear_x < -0.001:
            indices.update((0, 1))
        elif motion_direction == "forward" and linear_x > 0.001:
            indices.update((0, 1))
        elif motion_direction not in ("reverse", "forward"):
            if abs(linear_x) > 0.001:
                indices.update((0, 1))

        if turn_guard_enabled:
            # Narrow-space policy: every left/right turn checks both rear
            # ultrasonic sectors. A single blocked corner stops either turn.
            if abs(angular_z) > 0.001:
                indices.update((0, 1))
        return tuple(sorted(indices))

    def guarded_ultrasonic_sensors(self):
        if not self.ultrasonic_enabled or self.last_command is None:
            return ()
        linear_x = float(self.last_command.linear.x)
        angular_z = float(self.last_command.angular.z)
        return self.guarded_ultrasonic_indices(
            linear_x,
            angular_z,
            self.ultrasonic_motion_direction,
            self.ultrasonic_turn_guard_enabled,
        )

    def reverse_path_reason(self):
        if not self.reverse_straight_only or self.last_command is None:
            return None
        if float(self.last_command.linear.x) >= -0.001:
            return None
        if abs(float(self.last_command.angular.z)) > self.max_reverse_angular:
            return (
                "reverse prohibited: angular command "
                f"{float(self.last_command.angular.z):+.3f}rad/s exceeds "
                f"{self.max_reverse_angular:.3f}rad/s"
            )
        plan_age = self.elapsed(self.last_plan_time)
        if plan_age > self.plan_timeout:
            return "reverse prohibited: stale or missing plan"
        if not self.plan_is_straight:
            return f"reverse prohibited: curved path {self.plan_metrics}"
        if (
            self.straight_path_confirmations
            < self.straight_path_confirmations_required
        ):
            return (
                "reverse prohibited: straight path awaiting confirmation "
                f"{self.straight_path_confirmations}/"
                f"{self.straight_path_confirmations_required}")
        return None

    def on_set_arm(self, request, response):
        requested = bool(request.data)
        if requested != self.armed:
            self.armed = requested
            self.reset_small_spin_budget()
            self.get_logger().warn(
                f"motion gate {'ARMED' if self.armed else 'DISARMED'}")
        response.success = True
        response.message = "armed" if self.armed else "disarmed"
        self.armed_pub.publish(Bool(data=self.armed))
        return response

    def elapsed(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def reason(self):
        if not self.armed:
            return "disarmed"
        if self.operator_heartbeat_required:
            if (
                self.elapsed(self.last_operator_heartbeat_time)
                > self.operator_heartbeat_timeout
            ):
                return "stale HN operator heartbeat"
            if not self.operator_present:
                return "HN operator/RViz unavailable"
        if self.goal_lease_required:
            if self.elapsed(self.last_goal_lease_time) > self.goal_lease_timeout:
                return "stale navigation goal lease"
            if not self.goal_active:
                return "no active navigation goal"
        if self.elapsed(self.last_system_time) > self.chassis_timeout:
            return "stale chassis state"
        if self.system_error != 0:
            return f"chassis error_code={self.system_error}"
        if self.vehicle_state != int(SystemState.VEHICLE_STATE_NORMAL):
            return f"chassis vehicle_state={self.vehicle_state} is not NORMAL"
        if self.control_mode != int(SystemState.CONTROL_MODE_CAN):
            return f"chassis control_mode={self.control_mode} is not CAN"
        if self.elapsed(self.last_localization_time) > self.localization_timeout:
            return "stale localization"
        if self.elapsed(self.last_odom_time) > self.odom_timeout:
            return "stale odometry"
        if self.elapsed(self.last_obstacle_status_time) > self.obstacle_status_timeout:
            return "stale obstacle status"
        if self.last_obstacle_age > self.max_obstacle_age:
            return f"stale obstacle cloud age={self.last_obstacle_age:.3f}s"
        if not math.isfinite(self.last_fitness) or self.last_fitness > self.max_fitness:
            return f"bad NDT fitness={self.last_fitness:.3f}"
        if self.elapsed(self.last_command_time) > self.command_timeout:
            return "stale navigation command"
        if self.small_spin_requested():
            if not self.allow_small_in_place_rotation:
                return "in-place rotation prohibited"
            if self.spin_limit_reached:
                return (
                    "small in-place rotation limit reached "
                    f"{math.degrees(self.spin_accumulated):.1f}deg/"
                    f"{math.degrees(self.max_in_place_rotation):.1f}deg"
                )
        reverse_reason = self.reverse_path_reason()
        if reverse_reason is not None:
            return reverse_reason
        guarded_sensors = self.guarded_ultrasonic_sensors()
        if guarded_sensors:
            turning = (
                self.last_command is not None
                and abs(float(self.last_command.angular.z)) > 0.001
            )
            active_latches = (
                self.ultrasonic_turn_latches
                if turning else self.ultrasonic_latches
            )
            classifier = (
                self.classify_turn_ultrasonic
                if turning else self.classify_ultrasonic
            )
            if (
                self.elapsed(self.last_ultrasonic_health_time)
                > self.ultrasonic_timeout
            ):
                return "stale rear ultrasonic health: both sides blocked"
            if not self.ultrasonic_healthy:
                return "rear ultrasonic unhealthy: both sides blocked"
            failed_indices = [
                index for index in guarded_sensors
                if (
                    self.elapsed(self.last_ultrasonic_times[index])
                    > self.ultrasonic_timeout
                    or active_latches[index].blocked
                )
            ]
            if len(failed_indices) >= 2:
                return (
                    "rear ultrasonic blocked both sides "
                    f"left={self.format_distance(self.ultrasonic_ranges[0])}m "
                    f"right={self.format_distance(self.ultrasonic_ranges[1])}m "
                    "classifications=["
                    f"{classifier(0)},{classifier(1)}]"
                )
            for index in guarded_sensors:
                side = "left" if index == 0 else "right"
                stamp = self.last_ultrasonic_times[index]
                if self.elapsed(stamp) > self.ultrasonic_timeout:
                    return (
                        f"stale rear ultrasonic {side} "
                        f"sensor_{index + 1}")
                latch = active_latches[index]
                if latch.blocked:
                    value = self.ultrasonic_ranges[index]
                    tail_clearance = (
                        value - self.ultrasonic_sensor_to_rear_edge
                        if math.isfinite(value) else value
                    )
                    condition = (
                        "self-echo" if self.is_ultrasonic_self_echo(value)
                        else (
                            (
                                "turn-clear-pending"
                                if turning and math.isfinite(value)
                                and value > self.ultrasonic_turn_allow_distance
                                else "turn-distance"
                            ) if turning
                            else "blocked"
                        )
                    )
                    return (
                        f"rear ultrasonic {condition} {side} "
                        f"sensor_{index + 1} "
                        f"raw={value:.3f}m "
                        f"tail_clearance={tail_clearance:.3f}m "
                        + (
                            f"turn_requires>{self.ultrasonic_turn_allow_distance:.3f}m "
                            if turning else ""
                        )
                        + f"clear_count={latch.clear_count}"
                    )
        return "ready"

    def small_spin_requested(self):
        if self.last_command is None:
            return False
        return (
            abs(float(self.last_command.linear.x))
            < self.minimum_linear_for_turn
            and abs(float(self.last_command.angular.z)) > 0.001
        )

    def publish_status(self, reason):
        ready = reason == "ready"
        self.ready_pub.publish(Bool(data=ready))
        if reason != self.last_status:
            self.status_pub.publish(String(data=reason))
            self.get_logger().warn(f"motion blocked: {reason}") if not ready else \
                self.get_logger().info("motion gate ready")
            self.last_status = reason

    def publish_status_heartbeat(self):
        reason = self.reason()
        self.armed_pub.publish(Bool(data=self.armed))
        self.ready_pub.publish(Bool(data=reason == "ready"))
        self.status_pub.publish(String(data=reason))
        self.publish_ultrasonic_status()
        self.publish_operator_link_status()
        self.publish_path_policy()

    def publish_operator_link_status(self):
        age = self.elapsed(self.last_operator_heartbeat_time)
        if age > self.operator_heartbeat_timeout:
            state = "STALE"
        elif not self.operator_present:
            state = "UNAVAILABLE"
        else:
            state = "READY"
        self.operator_link_status_pub.publish(String(data=(
            f"state={state} present={str(self.operator_present).lower()} "
            f"age_s={age:.3f} "
            f"timeout_s={self.operator_heartbeat_timeout:.3f}"
        )))

    @staticmethod
    def format_distance(value):
        if math.isinf(value):
            return "+inf" if value > 0.0 else "-inf"
        if math.isnan(value):
            return "nan"
        return f"{value:.3f}"

    def publish_ultrasonic_status(self):
        ages = [
            self.elapsed(stamp) for stamp in self.last_ultrasonic_times
        ]
        fresh = [
            age <= self.ultrasonic_timeout for age in ages
        ]
        tail_clearances = [
            (
                value - self.ultrasonic_sensor_to_rear_edge
                if math.isfinite(value) else value
            )
            for value in self.ultrasonic_ranges
        ]
        blocked = [
            latch.blocked for latch in self.ultrasonic_latches
        ]
        turn_blocked = [
            latch.blocked for latch in self.ultrasonic_turn_latches
        ]
        classifications = [
            self.classify_ultrasonic(index) for index in range(2)
        ]
        turn_classifications = [
            self.classify_turn_ultrasonic(index) for index in range(2)
        ]
        reverse_allowed = self.rear_reverse_allowed()
        left_turn_allowed = self.rear_left_turn_allowed()
        right_turn_allowed = self.rear_right_turn_allowed()
        reverse_speed_scale = self.rear_reverse_speed_scale()
        reverse_blocking_sides = self.ultrasonic_blocking_sides(
            self.ultrasonic_latches)
        turn_blocking_sides = self.ultrasonic_blocking_sides(
            self.ultrasonic_turn_latches)
        blocking_sides = tuple(sorted(set(
            reverse_blocking_sides + turn_blocking_sides)))
        state = (
            "CLEAR" if reverse_allowed
            else ("TURN_ONLY" if left_turn_allowed else "BLOCKED")
        )
        self.ultrasonic_reverse_allowed_pub.publish(
            Bool(data=reverse_allowed))
        self.ultrasonic_left_turn_allowed_pub.publish(
            Bool(data=left_turn_allowed))
        self.ultrasonic_right_turn_allowed_pub.publish(
            Bool(data=right_turn_allowed))
        self.ultrasonic_status_pub.publish(String(data=(
            f"state={state} healthy={str(self.ultrasonic_healthy).lower()} "
            f"reverse_allowed={str(reverse_allowed).lower()} "
            f"left_turn_allowed={str(left_turn_allowed).lower()} "
            f"right_turn_allowed={str(right_turn_allowed).lower()} "
            f"reverse_speed_scale={reverse_speed_scale:.3f} "
            "blocking_sides=["
            f"{','.join(blocking_sides) if blocking_sides else 'NONE'}] "
            "reverse_blocking_sides=["
            f"{','.join(reverse_blocking_sides) if reverse_blocking_sides else 'NONE'}] "
            "turn_blocking_sides=["
            f"{','.join(turn_blocking_sides) if turn_blocking_sides else 'NONE'}] "
            f"blocked=[{str(blocked[0]).lower()},"
            f"{str(blocked[1]).lower()}] "
            f"turn_blocked=[{str(turn_blocked[0]).lower()},"
            f"{str(turn_blocked[1]).lower()}] "
            f"classification=[{classifications[0]},{classifications[1]}] "
            "turn_classification=["
            f"{turn_classifications[0]},{turn_classifications[1]}] "
            "raw_m=["
            f"{self.format_distance(self.ultrasonic_ranges[0])},"
            f"{self.format_distance(self.ultrasonic_ranges[1])}] "
            "tail_clearance_m=["
            f"{self.format_distance(tail_clearances[0])},"
            f"{self.format_distance(tail_clearances[1])}] "
            f"age_s=[{ages[0]:.3f},{ages[1]:.3f}] "
            f"fresh=[{str(fresh[0]).lower()},{str(fresh[1]).lower()}] "
            "clear_count=["
            f"{self.ultrasonic_latches[0].clear_count},"
            f"{self.ultrasonic_latches[1].clear_count}] "
            f"stop_m={self.ultrasonic_stop_distance:.3f} "
            f"clear_m={self.ultrasonic_clear_distance:.3f} "
            f"turn_allow_m={self.ultrasonic_turn_allow_distance:.3f} "
            "detect_m=["
            f"{self.format_distance(self.ultrasonic_min_ranges[0])},"
            f"{self.format_distance(self.ultrasonic_max_ranges[0])}] "
            "self_echo_m=["
            f"{self.ultrasonic_self_echo_min_distance:.3f},"
            f"{self.ultrasonic_self_echo_max_distance:.3f}] "
            f"sensor_to_tail_m={self.ultrasonic_sensor_to_rear_edge:.3f} "
            "turn_guard=[left:both,right:both,reverse:both]"
        )))

    def tick(self):
        reason = self.reason()
        self.publish_status(reason)
        if reason != "ready" or self.last_command is None:
            self.pub.publish(Twist())
            return
        out = Twist()
        out.linear.x = max(-self.max_linear, min(self.max_linear, self.last_command.linear.x))
        out.angular.z = max(-self.max_angular, min(self.max_angular, self.last_command.angular.z))
        if out.linear.x < -0.001:
            out.linear.x *= self.rear_reverse_speed_scale()
        if abs(out.linear.x) < self.minimum_linear_for_turn:
            out.linear.x = 0.0
            if abs(out.angular.z) <= 0.001:
                out.angular.z = 0.0
            else:
                if not self.spin_active:
                    self.spin_active = True
                    self.spin_last_yaw = self.odom_yaw
                    self.spin_origin_x = self.odom_x
                    self.spin_origin_y = self.odom_y
                    self.spin_accumulated = 0.0
                    self.spin_limit_reached = False
                    self.get_logger().info(
                        "bounded small in-place rotation started")
                out.angular.z = max(
                    -self.max_in_place_angular,
                    min(self.max_in_place_angular, out.angular.z),
                )
        else:
            angular_limit = min(
                self.max_angular,
                abs(out.linear.x) * self.max_motion_curvature
                + self.curvature_slack,
            )
            out.angular.z = max(
                -angular_limit, min(angular_limit, out.angular.z))
        self.pub.publish(out)


def main():
    rclpy.init()
    node = NavMotionSafetyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble may raise RCLError (a RuntimeError subclass) when SIGTERM
        # invalidates the context while the executor is rebuilding its wait set.
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
