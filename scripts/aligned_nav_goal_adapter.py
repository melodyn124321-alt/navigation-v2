#!/usr/bin/env python3
"""Convert an RViz goal into an obstacle-checked aligned approach."""

import copy
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import (
    BackUp,
    DriveOnHeading,
    NavigateThroughPoses,
    NavigateToPose,
    Spin,
)
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def choose_direct_route_mode(
        forward_turn, reverse_turn, alignment_limit,
        automatic_reverse_enabled):
    """Choose an automatic direct-route direction without changing hardware.

    Disabling automatic reverse does not remove the /backup behavior or the
    reverse safety gate.  It only makes an RViz navigation goal align to the
    forward bearing and then use the forward-only Nav2 planner/controller.
    """
    if not automatic_reverse_enabled:
        if abs(forward_turn) <= alignment_limit + 1.0e-6:
            return "forward", forward_turn
        return None, None
    if (
            abs(forward_turn) <= abs(reverse_turn)
            and abs(forward_turn) <= alignment_limit):
        return "forward", forward_turn
    if abs(reverse_turn) <= alignment_limit:
        return "straight_reverse", reverse_turn
    return None, None


def calculate_spin_time_allowance(
        angle, minimum_timeout, effective_min_angular_speed,
        timeout_factor, timeout_margin):
    """Return a conservative allowance for a collision-checked spin step."""
    nominal_duration = (
        abs(angle) / max(0.01, effective_min_angular_speed)
    )
    return max(
        minimum_timeout,
        nominal_duration * max(1.0, timeout_factor)
        + max(0.0, timeout_margin),
    )


def should_run_turn_clearance(
        initial_turn, minimum_turn, left_allowed, right_allowed, enabled):
    """Return true when forward clearance is needed before a real turn."""
    return (
        enabled
        and abs(initial_turn) >= minimum_turn
        and not (left_allowed is True and right_allowed is True)
    )


def should_retry_terminal_residual(
        position_error, position_tolerance, maximum_correction_distance,
        attempt, retry_count):
    """Allow only bounded, measured corrections after a completed drive.

    DriveOnHeading reports completion from chassis odometry.  A skid-steer
    chassis can still have a small map-frame NDT residual after that action
    succeeds.  Keep the strict arrival tolerance, but permit a finite number
    of collision-checked corrections while that measured residual remains
    inside the configured correction envelope.
    """
    return (
        math.isfinite(position_error)
        and position_error > position_tolerance
        and position_error <= maximum_correction_distance
        and attempt <= retry_count
    )


def terminal_ndt_stop_reason(
        distance, best_distance, arrival_stop_distance,
        overshoot_activation_distance, overshoot_growth):
    """Decide whether a fixed-odometry terminal drive must stop early.

    DriveOnHeading measures completion in chassis odometry, while arrival is
    defined in the map frame by NDT.  Stop immediately on measured arrival.
    Also stop when a near-target distance starts increasing materially: this
    means the chassis crossed the target even if wheel odometry still reports
    an unfinished distance.
    """
    if not math.isfinite(distance):
        return None
    if distance <= arrival_stop_distance:
        return "arrival"
    if (
            math.isfinite(best_distance)
            and best_distance <= overshoot_activation_distance
            and distance >= best_distance + overshoot_growth):
        return "overshoot"
    return None


def bounded_alignment_step(alignment, maximum_step):
    """Return one signed alignment step without precomputing later steps."""
    limit = max(1.0e-6, abs(maximum_step))
    return max(-limit, min(limit, alignment))


def choose_final_position_correction(
        distance, alignment, position_tolerance, forward_alignment_limit,
        micro_reverse_enabled, micro_reverse_min_distance,
        micro_reverse_max_distance,
        micro_reverse_lateral_margin,
        micro_reverse_alignment_limit):
    """Choose a strict, bounded correction for post-spin position drift.

    A skid-steer final yaw correction can move ``base_link`` a few centimetres
    past the target.  Requiring a forward-only correction in that state makes
    the chassis turn almost 180 degrees for a very short translation.  Prefer
    a straight micro reverse only when its lateral projection is already
    inside the strict position circle; otherwise retain a segmented,
    collision-checked forward correction.
    """
    if abs(alignment) <= forward_alignment_limit:
        return "forward", distance
    backward_projection = -distance * math.cos(alignment)
    lateral_error = abs(distance * math.sin(alignment))
    predicted_limit = max(
        0.0, position_tolerance - max(0.0, micro_reverse_lateral_margin))
    reverse_command = max(
        micro_reverse_min_distance, backward_projection)
    predicted_error = math.hypot(
        lateral_error, reverse_command - backward_projection)
    if (
            micro_reverse_enabled
            and backward_projection > 0.0
            and reverse_command <= micro_reverse_max_distance
            and predicted_error <= predicted_limit):
        return "micro_reverse", reverse_command
    reverse_alignment = normalize_angle(alignment - math.pi)
    if (
            micro_reverse_enabled
            and distance >= micro_reverse_min_distance
            and distance <= micro_reverse_max_distance
            and abs(reverse_alignment) <= micro_reverse_alignment_limit):
        return "aligned_micro_reverse", distance
    return "segmented_forward", distance


def correction_execution_alignment_limit(mode, selection_limit):
    """Keep a forward correction valid after measured bearing updates.

    The configured selection limit distinguishes an initially aligned
    correction from a segmented one.  It must not remain an abort threshold
    after the first bounded turn: skid-steer translation can move a 50-degree
    residual past the 60-degree selection boundary even while the correction
    is converging.  Non-reverse corrections may therefore continue through
    measured bounded steps; corridor, distance-growth, collision and recheck
    limits still provide the safety bounds.
    """
    if mode in ("forward", "segmented_forward"):
        return math.pi
    return selection_limit


def final_approach_is_useful(
        direct_distance, direct_turn, staging_distance, staging_turn,
        approach_distance, extra_distance=0.15, extra_turn=0.52):
    """Reject a final-yaw staging pose that makes a short route worse."""
    return (
        math.isfinite(staging_distance)
        and math.isfinite(approach_distance)
        and staging_distance + approach_distance
        <= direct_distance + max(extra_distance, 0.10 * direct_distance)
        and abs(staging_turn) <= abs(direct_turn) + extra_turn
    )


def classify_motion_block(state):
    """Map a gate reason to a bounded active-goal watchdog category."""
    if state.startswith("reverse prohibited"):
        return "reverse policy"
    if state.startswith("LiDAR collision monitor blocked command"):
        return "LiDAR obstacle"
    if state.startswith((
            "sustained bad NDT fitness", "invalid NDT fitness",
            "stale localization")):
        return "localization"
    if state.startswith((
            "rear ultrasonic", "stale rear ultrasonic")):
        return "rear ultrasonic obstacle"
    if state.startswith((
            "small in-place rotation limit reached",
            "in-place rotation prohibited")):
        return "rotation safety limit"
    if state.startswith((
            "stale HN operator heartbeat", "HN operator/RViz unavailable",
            "stale chassis state", "chassis state warming",
            "chassis error_code=", "chassis vehicle_state=",
            "chassis control_mode=", "stale odometry",
            "stale obstacle status", "stale obstacle cloud age=")):
        return "safety prerequisite"
    if state == "stale navigation command":
        return "stale command"
    return None


class AlignedNavGoalAdapter(Node):
    def __init__(self):
        super().__init__("aligned_nav_goal_adapter")
        self.approach_distance = float(
            self.declare_parameter("approach_distance", 0.70).value)
        self.min_approach_distance = float(
            self.declare_parameter("min_approach_distance", 0.45).value)
        self.approach_step = float(
            self.declare_parameter("approach_step", 0.05).value)
        # A locally clear staging point behind the target is not necessarily
        # globally reachable from the robot. Let the global planner reach the
        # real target and use measured final-pose correction by default.
        self.use_final_approach_route = bool(
            self.declare_parameter(
                "use_final_approach_route", False).value)
        self.maximum_cost = int(
            self.declare_parameter("maximum_cost", 90).value)
        self.map_wait_timeout = float(
            self.declare_parameter("map_wait_timeout", 5.0).value)
        self.outer_action_name = str(
            self.declare_parameter(
                "outer_action_name", "/aligned_navigate_to_pose").value)
        self.direct_reverse_plan_topic = str(
            self.declare_parameter(
                "direct_reverse_plan_topic",
                "/direct_reverse_plan").value)
        self.feedback_period = float(
            self.declare_parameter("feedback_period", 0.20).value)
        self.progress_timeout = float(
            self.declare_parameter("progress_timeout", 45.0).value)
        self.progress_min_displacement = float(
            self.declare_parameter("progress_min_displacement", 0.03).value)
        self.small_spin_min_angle = float(
            self.declare_parameter("small_spin_min_angle", 0.08).value)
        self.small_spin_max_angle = float(
            self.declare_parameter("small_spin_max_angle", 0.52).value)
        self.direct_alignment_max_angle = float(
            self.declare_parameter(
                "direct_alignment_max_angle", math.pi).value)
        self.automatic_reverse_enabled = bool(
            self.declare_parameter(
                "automatic_reverse_enabled", False).value)
        self.final_alignment_max_angle = float(
            self.declare_parameter(
                "final_alignment_max_angle", math.pi).value)
        self.small_spin_timeout = float(
            self.declare_parameter("small_spin_timeout", 12.0).value)
        self.small_spin_effective_min_angular_speed = float(
            self.declare_parameter(
                "small_spin_effective_min_angular_speed", 0.020).value)
        self.small_spin_timeout_factor = float(
            self.declare_parameter(
                "small_spin_timeout_factor", 1.50).value)
        self.small_spin_timeout_margin = float(
            self.declare_parameter(
                "small_spin_timeout_margin", 5.0).value)
        self.use_map_yaw_spin = bool(
            self.declare_parameter("use_map_yaw_spin", True).value)
        self.map_spin_command_topic = str(
            self.declare_parameter(
                "map_spin_command_topic", "/cmd_vel_nav").value)
        self.map_spin_angular_speed = float(
            self.declare_parameter(
                "map_spin_angular_speed", 0.18).value)
        self.final_map_spin_angular_speed = float(
            self.declare_parameter(
                "final_map_spin_angular_speed", 0.06).value)
        self.map_spin_yaw_tolerance = float(
            self.declare_parameter(
                "map_spin_yaw_tolerance", 0.08).value)
        self.final_map_spin_yaw_tolerance = float(
            self.declare_parameter(
                "final_map_spin_yaw_tolerance", 0.035).value)
        self.map_spin_max_position_drift = float(
            self.declare_parameter(
                "map_spin_max_position_drift", 0.15).value)
        self.straight_reverse_speed = float(
            self.declare_parameter(
                "straight_reverse_speed", 0.04).value)
        self.reverse_time_allowance_factor = float(
            self.declare_parameter(
                "reverse_time_allowance_factor", 3.0).value)
        self.terminal_handoff_distance = float(
            self.declare_parameter(
                "terminal_handoff_distance", 0.35).value)
        self.use_terminal_handoff = bool(
            self.declare_parameter(
                "use_terminal_handoff", False).value)
        self.terminal_position_tolerance = float(
            self.declare_parameter(
                "terminal_position_tolerance", 0.05).value)
        self.final_alignment_position_tolerance = float(
            self.declare_parameter(
                "final_alignment_position_tolerance", 0.05).value)
        self.final_pose_confirmations = int(
            self.declare_parameter("final_pose_confirmations", 3).value)
        self.final_pose_confirmation_period = float(
            self.declare_parameter(
                "final_pose_confirmation_period", 0.15).value)
        self.final_position_correction_cycles = int(
            self.declare_parameter(
                "final_position_correction_cycles", 3).value)
        self.final_position_correction_max_distance = float(
            self.declare_parameter(
                "final_position_correction_max_distance", 0.20).value)
        self.final_position_correction_alignment_limit = float(
            self.declare_parameter(
                "final_position_correction_alignment_limit", 1.047198).value)
        self.final_position_micro_reverse_enabled = bool(
            self.declare_parameter(
                "final_position_micro_reverse_enabled", True).value)
        self.final_position_micro_reverse_max_distance = float(
            self.declare_parameter(
                "final_position_micro_reverse_max_distance", 0.12).value)
        self.final_position_micro_reverse_min_distance = float(
            self.declare_parameter(
                "final_position_micro_reverse_min_distance", 0.05).value)
        self.final_position_micro_reverse_lateral_margin = float(
            self.declare_parameter(
                "final_position_micro_reverse_lateral_margin", 0.004).value)
        self.final_position_micro_reverse_alignment_limit = float(
            self.declare_parameter(
                "final_position_micro_reverse_alignment_limit", 0.12).value)
        self.terminal_forward_speed = float(
            self.declare_parameter(
                "terminal_forward_speed", 0.04).value)
        self.terminal_time_allowance_factor = float(
            self.declare_parameter(
                "terminal_time_allowance_factor", 3.0).value)
        self.terminal_drive_retry_count = int(
            self.declare_parameter(
                "terminal_drive_retry_count", 2).value)
        self.terminal_drive_retry_delay = float(
            self.declare_parameter(
                "terminal_drive_retry_delay", 1.5).value)
        self.terminal_ndt_overshoot_activation_distance = float(
            self.declare_parameter(
                "terminal_ndt_overshoot_activation_distance", 0.15).value)
        self.terminal_ndt_overshoot_growth = float(
            self.declare_parameter(
                "terminal_ndt_overshoot_growth", 0.04).value)
        self.terminal_ndt_brake_distance = max(
            self.terminal_position_tolerance,
            float(self.declare_parameter(
                "terminal_ndt_brake_distance", 0.08).value),
        )
        self.terminal_alignment_max_angle = float(
            self.declare_parameter(
                "terminal_alignment_max_angle", 0.52).value)
        self.terminal_bearing_yaw_tolerance = float(
            self.declare_parameter(
                "terminal_bearing_yaw_tolerance", 0.035).value)
        self.terminal_alignment_recheck_count = int(
            self.declare_parameter(
                "terminal_alignment_recheck_count", 12).value)
        self.terminal_alignment_max_distance_growth = float(
            self.declare_parameter(
                "terminal_alignment_max_distance_growth", 0.35).value)
        self.policy_block_timeout = float(
            self.declare_parameter("policy_block_timeout", 6.0).value)
        self.stale_command_abort_timeout = float(
            self.declare_parameter(
                "stale_command_abort_timeout", 15.0).value)
        self.collision_block_abort_timeout = float(
            self.declare_parameter(
                "collision_block_abort_timeout", 3.0).value)
        self.localization_block_abort_timeout = float(
            self.declare_parameter(
                "localization_block_abort_timeout", 3.0).value)
        self.degraded_localization_block_abort_timeout = float(
            self.declare_parameter(
                "degraded_localization_block_abort_timeout", 6.0).value)
        self.turn_clearance_forward_enabled = bool(
            self.declare_parameter(
                "turn_clearance_forward_enabled", True).value)
        self.turn_clearance_forward_distance = float(
            self.declare_parameter(
                "turn_clearance_forward_distance", 0.30).value)
        self.turn_clearance_permission_wait = float(
            self.declare_parameter(
                "turn_clearance_permission_wait", 2.50).value)
        self.global_costmap = None
        self.active = False
        self.inner_handle = None
        self.spin_handle = None
        self.backup_handle = None
        self.drive_handle = None
        self.direct_reverse_plan = None
        self.direct_reverse_distance = 0.0
        self.last_backup_status_time = None
        self.last_feedback_time = None
        self.last_progress_status_time = None
        self.last_motion_status = None
        self.last_distance = math.inf
        self.best_distance = math.inf
        self.last_recoveries = 0
        self.progress_anchor_x = None
        self.progress_anchor_y = None
        self.progress_anchor_time = None
        self.progress_abort_requested = False
        self.abort_reason = None
        self.motion_block_start_time = None
        self.motion_block_kind = None
        self.motion_armed = None
        self.motion_ready = False
        self.left_turn_allowed = None
        self.right_turn_allowed = None
        self.goal_lock = threading.Lock()
        self.goal_reserved = False
        self.motion_authorized = False
        self.goal_sequence = 0
        self.current_goal_id = None
        self.active_target = None
        self.terminal_handoff_enabled = False
        self.terminal_handoff_requested = False
        self.final_alignment_completed = False
        self.last_drive_status_time = None
        self.terminal_drive_target = None
        self.terminal_drive_best_distance = math.inf
        self.terminal_drive_cancel_reason = None

        callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        status_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.on_costmap,
            latched_qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            String,
            "/nav_motion_status",
            self.on_motion_status,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            Bool,
            "/nav_motion_armed",
            self.on_motion_armed,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            Bool,
            "/rear_ultrasonic_left_turn_allowed",
            self.on_left_turn_allowed,
            10,
            callback_group=callback_group,
        )
        self.create_subscription(
            Bool,
            "/rear_ultrasonic_right_turn_allowed",
            self.on_right_turn_allowed,
            10,
            callback_group=callback_group,
        )
        self.status_pub = self.create_publisher(
            String, "/aligned_goal_status", status_qos)
        self.approach_pub = self.create_publisher(
            PoseStamped, "/aligned_goal_approach_pose", latched_qos)
        self.target_pub = self.create_publisher(
            PoseStamped, "/aligned_goal_target_pose", latched_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/nav_goal_markers", latched_qos)
        # Heartbeat-style lease consumed by the final motion gate.  A fresh
        # false lease stops residual controller/smoother commands after a goal
        # finishes or is rejected; a missing lease also fails closed.
        self.goal_active_pub = self.create_publisher(
            Bool, "/aligned_goal_active", latched_qos)
        self.inner_client = ActionClient(
            self,
            NavigateThroughPoses,
            "/navigate_through_poses",
            callback_group=callback_group,
        )
        self.spin_client = ActionClient(
            self,
            Spin,
            "/spin",
            callback_group=callback_group,
        )
        self.backup_client = ActionClient(
            self,
            BackUp,
            "/backup",
            callback_group=callback_group,
        )
        self.drive_client = ActionClient(
            self,
            DriveOnHeading,
            "/drive_on_heading",
            callback_group=callback_group,
        )
        self.plan_pub = self.create_publisher(Path, "/plan", 10)
        self.direct_reverse_plan_pub = self.create_publisher(
            Path, self.direct_reverse_plan_topic, 10)
        self.map_spin_command_pub = self.create_publisher(
            Twist, self.map_spin_command_topic, 10)
        self.outer_server = ActionServer(
            self,
            NavigateToPose,
            self.outer_action_name,
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=callback_group,
        )
        self.create_timer(
            0.50, self.check_persistent_motion_block,
            callback_group=callback_group)
        self.create_timer(
            0.50, self.publish_direct_reverse_plan,
            callback_group=callback_group)
        self.create_timer(
            0.20, self.publish_goal_active_lease,
            callback_group=callback_group)
        self.publish_goal_active_lease()
        self.publish_status(
            f"READY approach={self.approach_distance:.2f}m "
            f"outer_action={self.outer_action_name} "
            f"reverse_plan={self.direct_reverse_plan_topic} "
            f"minimum={self.min_approach_distance:.2f}m "
            f"final_approach_route={self.use_final_approach_route} "
            f"progress={self.progress_min_displacement:.2f}m/"
            f"{self.progress_timeout:.0f}s "
            f"small_spin_step<="
            f"{math.degrees(self.small_spin_max_angle):.0f}deg "
            f"spin_timeout=dynamic(min={self.small_spin_timeout:.0f}s,"
            f"effective_rate={self.small_spin_effective_min_angular_speed:.3f}rad/s) "
            f"spin_feedback={'map_yaw' if self.use_map_yaw_spin else 'odom_action'} "
            f"spin_command={self.map_spin_angular_speed:.2f}rad/s "
            f"final_spin_command="
            f"{self.final_map_spin_angular_speed:.2f}rad/s "
            f"initial_alignment<="
            f"{math.degrees(self.direct_alignment_max_angle):.0f}deg "
            f"automatic_reverse={self.automatic_reverse_enabled} "
            f"strategy="
            f"{'shortest_direction' if self.automatic_reverse_enabled else 'spin_then_forward'} "
            f"final_alignment<="
            f"{math.degrees(self.final_alignment_max_angle):.0f}deg "
            f"policy_abort={self.policy_block_timeout:.0f}s "
            f"stale_abort={self.stale_command_abort_timeout:.0f}s "
            f"localization_abort="
            f"{self.localization_block_abort_timeout:.0f}s "
            f"straight_reverse_speed={self.straight_reverse_speed:.2f}m/s "
            f"reverse_allowance_factor="
            f"{self.reverse_time_allowance_factor:.1f} "
            f"terminal_handoff_enabled={self.use_terminal_handoff} "
            f"terminal_handoff={self.terminal_handoff_distance:.2f}m "
            f"terminal_speed={self.terminal_forward_speed:.2f}m/s "
            f"terminal_allowance_factor="
            f"{self.terminal_time_allowance_factor:.1f} "
            f"terminal_tolerance={self.terminal_position_tolerance:.2f}m "
            f"terminal_ndt_stop=brake<="
            f"{self.terminal_ndt_brake_distance:.2f}m/strict<="
            f"{self.terminal_position_tolerance:.2f}m/overshoot="
            f"{self.terminal_ndt_overshoot_activation_distance:.2f}m+"
            f"{self.terminal_ndt_overshoot_growth:.2f}m "
            f"terminal_bearing_tolerance="
            f"{math.degrees(self.terminal_bearing_yaw_tolerance):.1f}deg "
            f"final_alignment_position_tolerance="
            f"{self.final_alignment_position_tolerance:.2f}m "
            f"final_pose_confirmations={self.final_pose_confirmations} "
            f"final_position_corrections="
            f"{self.final_position_correction_cycles}x/"
            f"{self.final_position_correction_max_distance:.2f}m "
            f"final_correction_alignment<="
            f"{math.degrees(self.final_position_correction_alignment_limit):.0f}deg "
            f"final_micro_reverse="
            f"{self.final_position_micro_reverse_enabled}/"
            f"{self.final_position_micro_reverse_min_distance:.2f}-"
            f"{self.final_position_micro_reverse_max_distance:.2f}m/"
            f"{math.degrees(self.final_position_micro_reverse_alignment_limit):.1f}deg "
            f"turn_clearance_forward="
            f"{self.turn_clearance_forward_enabled} "
            f"turn_clearance_distance="
            f"{self.turn_clearance_forward_distance:.2f}m")

    def on_costmap(self, msg):
        self.global_costmap = msg

    def publish_status(self, text):
        if self.current_goal_id is not None and "goal_id=" not in text:
            text = f"{text} goal_id={self.current_goal_id}"
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_goal_active_lease(self):
        # The gate needs the outer goal lease before it can pass the first
        # child-action command. Tying this lease to motion_authorized creates
        # a circular wait: the adapter waits for command feedback while the
        # gate waits for the adapter lease. The gate still fails closed on
        # disarm, stale localization, stale odometry, obstacles, and heartbeat.
        with self.goal_lock:
            active = self.active
        self.goal_active_pub.publish(Bool(data=active))

    def set_motion_authorized(self, authorized):
        with self.goal_lock:
            changed = self.motion_authorized != bool(authorized)
            self.motion_authorized = bool(authorized)
        self.publish_goal_active_lease()
        if changed:
            state = "ACTIVE" if authorized else "BLOCKED"
            self.publish_status(
                f"MOTION_LEASE_{state} "
                "current child action command authorization")

    def clear_recovered_terminal_reverse_abort(self):
        """Clear only a recovered terminal micro-reverse policy abort.

        The safety gate can cancel a centimetre-scale BackUp action at the
        exact straight-path length boundary after the chassis has already
        entered the strict NDT position circle.  That child cancellation must
        not poison the following measured final-yaw step.  Obstacle,
        localization, E-stop, heartbeat, and generic progress aborts remain
        latched and are never cleared here.
        """
        reason = self.abort_reason or ""
        if not (
                self.progress_abort_requested
                and reason.startswith("persistent reverse policy:")):
            return False
        self.progress_abort_requested = False
        self.abort_reason = None
        self.motion_block_start_time = None
        self.motion_block_kind = None
        self.publish_status(
            "TERMINAL_REVERSE_POLICY_RECOVERED strict NDT position is "
            "inside tolerance; cleared the completed micro-reverse child "
            "policy abort before final heading")
        return True

    def on_motion_status(self, msg):
        state = msg.data.strip()
        # The idle "no active navigation goal" state is precisely the state
        # from which a new outer goal may start. Record readiness even while no
        # goal is active; goal-specific watchdog/log handling remains below.
        self.motion_ready = state in ("ready", "no active navigation goal")
        if not self.active:
            self.last_motion_status = state
            return
        now = self.get_clock().now().nanoseconds / 1e9
        block_kind = classify_motion_block(state)
        if block_kind != self.motion_block_kind:
            self.motion_block_kind = block_kind
            self.motion_block_start_time = now if block_kind else None
        if state == self.last_motion_status:
            return
        self.last_motion_status = state
        if state == "ready":
            self.publish_status("RUNNING motion permitted")
        elif state == "disarmed":
            self.publish_status(
                "BLOCKED motion gate is DISARMED; arm before sending a goal")
        else:
            self.publish_status(f"BLOCKED {state}")

    def on_motion_armed(self, msg):
        was_armed = self.motion_armed
        self.motion_armed = bool(msg.data)
        if (
                was_armed is True
                and self.motion_armed is False
                and self.active
                and not self.progress_abort_requested):
            self.progress_abort_requested = True
            self.abort_reason = "motion gate was disarmed during active goal"
            self.publish_status(f"ABORTING {self.abort_reason}")
            for handle in (
                    self.inner_handle, self.spin_handle,
                    self.backup_handle, self.drive_handle):
                if handle is not None:
                    handle.cancel_goal_async()

    def on_left_turn_allowed(self, msg):
        self.left_turn_allowed = bool(msg.data)

    def on_right_turn_allowed(self, msg):
        self.right_turn_allowed = bool(msg.data)

    def check_persistent_motion_block(self):
        handle = (
            self.inner_handle or self.backup_handle or self.drive_handle)
        manual_map_spin = (
            self.use_map_yaw_spin
            and self.motion_authorized
            and handle is None
        )
        if (
                not self.active
                or (handle is None and not manual_map_spin)
                or self.motion_block_kind is None
                or self.motion_block_start_time is None
                or self.progress_abort_requested):
            return
        now = self.get_clock().now().nanoseconds / 1e9
        blocked_for = now - self.motion_block_start_time
        if self.motion_block_kind == "reverse policy":
            timeout = self.policy_block_timeout
        elif self.motion_block_kind in (
                "LiDAR obstacle", "rear ultrasonic obstacle"):
            timeout = self.collision_block_abort_timeout
        elif self.motion_block_kind == "localization":
            timeout = (
                self.degraded_localization_block_abort_timeout
                if "mode=degraded_recovery" in (self.last_motion_status or "")
                else self.localization_block_abort_timeout
            )
        elif self.motion_block_kind in (
                "rotation safety limit", "safety prerequisite"):
            timeout = self.policy_block_timeout
        else:
            timeout = self.stale_command_abort_timeout
        if blocked_for < timeout:
            return
        self.progress_abort_requested = True
        self.abort_reason = (
            f"persistent {self.motion_block_kind}: "
            f"{self.last_motion_status} for {blocked_for:.1f}s"
        )
        self.publish_status(f"ABORTING {self.abort_reason}")
        if handle is not None:
            handle.cancel_goal_async()

    def make_straight_path(self, start, target):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        distance = math.hypot(
            target.pose.position.x - start[0],
            target.pose.position.y - start[1],
        )
        samples = max(3, int(math.ceil(distance / 0.05)) + 1)
        for index in range(samples):
            ratio = index / (samples - 1)
            pose = PoseStamped()
            pose.header = copy.deepcopy(path.header)
            pose.pose.position.x = (
                start[0]
                + ratio * (target.pose.position.x - start[0])
            )
            pose.pose.position.y = (
                start[1]
                + ratio * (target.pose.position.y - start[1])
            )
            self.set_pose_yaw(pose, start[2])
            path.poses.append(pose)
        return path

    def publish_direct_reverse_plan(self):
        if self.direct_reverse_plan is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.direct_reverse_plan.header.stamp = stamp
        for pose in self.direct_reverse_plan.poses:
            pose.header.stamp = stamp
        self.direct_reverse_plan_pub.publish(self.direct_reverse_plan)

    def clear_direct_reverse_plan(self):
        empty = Path()
        empty.header.frame_id = "map"
        empty.header.stamp = self.get_clock().now().to_msg()
        self.direct_reverse_plan_pub.publish(empty)

    def relay_backup_feedback(self, outer_handle, feedback_msg):
        self.set_motion_authorized(True)
        now = self.get_clock().now().nanoseconds / 1e9
        traveled = float(feedback_msg.feedback.distance_traveled)
        remaining = max(0.0, self.direct_reverse_distance - traveled)
        feedback = NavigateToPose.Feedback()
        base_pose = self.current_base_pose()
        if base_pose is not None:
            feedback.current_pose.header.frame_id = "map"
            feedback.current_pose.header.stamp = (
                self.get_clock().now().to_msg())
            feedback.current_pose.pose.position.x = base_pose[0]
            feedback.current_pose.pose.position.y = base_pose[1]
            self.set_pose_yaw(feedback.current_pose, base_pose[2])
        feedback.distance_remaining = remaining
        outer_handle.publish_feedback(feedback)
        if (
                self.last_backup_status_time is None
                or now - self.last_backup_status_time >= 1.0):
            self.publish_status(
                "STRAIGHT_REVERSE_RUNNING "
                f"traveled={traveled:.3f}m "
                f"remaining={remaining:.3f}m "
                f"speed={self.straight_reverse_speed:.3f}m/s")
            self.last_backup_status_time = now

    def relay_drive_feedback(self, outer_handle, feedback_msg):
        self.set_motion_authorized(True)
        now = self.get_clock().now().nanoseconds / 1e9
        base_pose = self.current_base_pose()
        target_distance = math.inf
        feedback = NavigateToPose.Feedback()
        if base_pose is not None:
            feedback.current_pose.header.frame_id = "map"
            feedback.current_pose.header.stamp = (
                self.get_clock().now().to_msg())
            feedback.current_pose.pose.position.x = base_pose[0]
            feedback.current_pose.pose.position.y = base_pose[1]
            self.set_pose_yaw(feedback.current_pose, base_pose[2])
            if self.terminal_drive_target is not None:
                target_distance = math.hypot(
                    self.terminal_drive_target.pose.position.x - base_pose[0],
                    self.terminal_drive_target.pose.position.y - base_pose[1],
                )
        feedback.distance_remaining = target_distance
        outer_handle.publish_feedback(feedback)
        if math.isfinite(target_distance):
            self.last_distance = target_distance
            self.best_distance = min(self.best_distance, target_distance)
            self.terminal_drive_best_distance = min(
                self.terminal_drive_best_distance, target_distance)
            stop_reason = terminal_ndt_stop_reason(
                target_distance,
                self.terminal_drive_best_distance,
                self.terminal_ndt_brake_distance,
                self.terminal_ndt_overshoot_activation_distance,
                self.terminal_ndt_overshoot_growth,
            )
            if (
                    stop_reason is not None
                    and self.terminal_drive_cancel_reason is None):
                self.terminal_drive_cancel_reason = stop_reason
                if stop_reason == "arrival":
                    self.publish_status(
                        "TERMINAL_NDT_ARRIVAL_STOP measured map-frame "
                        f"distance={target_distance:.3f}m entered braking "
                        f"envelope={self.terminal_ndt_brake_distance:.3f}m "
                        f"before strict tolerance="
                        f"{self.terminal_position_tolerance:.3f}m; "
                        "canceling fixed-odometry drive and braking")
                else:
                    self.publish_status(
                        "TERMINAL_NDT_OVERSHOOT_STOP measured map-frame "
                        "distance increased after the near-target minimum "
                        f"best={self.terminal_drive_best_distance:.3f}m "
                        f"current={target_distance:.3f}m growth_limit="
                        f"{self.terminal_ndt_overshoot_growth:.3f}m; "
                        "canceling fixed-odometry drive and remeasuring")
                if self.drive_handle is not None:
                    self.drive_handle.cancel_goal_async()
        if (
                self.last_drive_status_time is None
                or now - self.last_drive_status_time >= 1.0):
            traveled = float(feedback_msg.feedback.distance_traveled)
            self.publish_status(
                "TERMINAL_FORWARD_RUNNING "
                f"traveled={traveled:.3f}m "
                f"target_distance={target_distance:.3f}m "
                f"speed={self.terminal_forward_speed:.3f}m/s")
            self.last_drive_status_time = now

    async def wait_for_turn_clearance(self):
        deadline = time.monotonic() + max(
            0.0, self.turn_clearance_permission_wait)
        while time.monotonic() < deadline:
            if (
                    self.left_turn_allowed is True
                    and self.right_turn_allowed is True):
                return True
            time.sleep(0.10)
        return (
            self.left_turn_allowed is True
            and self.right_turn_allowed is True
        )

    async def run_forward_for_turn_clearance(self, outer_handle, target):
        """Move straight through a checked corridor before retrying a turn."""
        self.publish_status(
            "TURN_CLEARANCE_WAIT turn is temporarily restricted; waiting "
            "for both rear ultrasonic sectors to clear")
        if await self.wait_for_turn_clearance():
            self.publish_status(
                "TURN_CLEARANCE_READY both rear sectors cleared without "
                "extra motion")
            return True

        base_pose = self.current_base_pose()
        if base_pose is None:
            self.publish_status(
                "TURN_CLEARANCE_ABORT cannot read map->base_link pose")
            return False
        target_distance = math.hypot(
            target.pose.position.x - base_pose[0],
            target.pose.position.y - base_pose[1],
        )
        distance = min(
            self.turn_clearance_forward_distance,
            max(0.0, target_distance - self.terminal_position_tolerance),
        )
        if distance < max(0.12, self.terminal_position_tolerance):
            self.publish_status(
                "TURN_CLEARANCE_ABORT target is too close for a safe "
                f"straight lead-in distance={distance:.3f}m")
            return False

        clearance_target = PoseStamped()
        clearance_target.header.frame_id = "map"
        clearance_target.header.stamp = self.get_clock().now().to_msg()
        clearance_target.pose.position.x = (
            base_pose[0] + distance * math.cos(base_pose[2]))
        clearance_target.pose.position.y = (
            base_pose[1] + distance * math.sin(base_pose[2]))
        self.set_pose_yaw(clearance_target, base_pose[2])
        if not self.corridor_is_clear(
                base_pose[0], base_pose[1],
                clearance_target.pose.position.x,
                clearance_target.pose.position.y):
            self.publish_status(
                "TURN_CLEARANCE_ABORT forward lead-in corridor is occupied; "
                "refusing blind motion")
            return False

        self.publish_status(
            "TURN_CLEARANCE_FORWARD rear turn is restricted; executing a "
            f"collision-checked straight lead-in distance={distance:.3f}m")
        if not await self.run_terminal_forward(
                outer_handle, clearance_target):
            self.publish_status(
                "TURN_CLEARANCE_ABORT straight lead-in did not complete")
            return False
        if not await self.wait_for_turn_clearance():
            self.publish_status(
                "TURN_CLEARANCE_ABORT both rear sectors remain restricted "
                "after the straight lead-in")
            return False
        self.publish_status(
            "TURN_CLEARANCE_READY straight lead-in complete; both rear "
            "sectors permit turning")
        return True

    async def run_terminal_forward(
            self, outer_handle, target, attempt=1,
            alignment_limit=None, phase="terminal",
            alignment_recheck=0, alignment_origin_distance=None):
        base_pose = self.current_base_pose()
        if base_pose is None:
            self.publish_status(
                "TERMINAL_ABORT cannot read map->base_link pose")
            return False
        remaining = math.hypot(
            target.pose.position.x - base_pose[0],
            target.pose.position.y - base_pose[1],
        )
        if alignment_origin_distance is None:
            alignment_origin_distance = remaining
        elif remaining > (
                alignment_origin_distance
                + self.terminal_alignment_max_distance_growth):
            self.publish_status(
                "TERMINAL_ABORT repeated bearing alignment moved away from "
                "the target beyond the bounded growth limit "
                f"distance={remaining:.3f}m "
                f"origin={alignment_origin_distance:.3f}m "
                f"growth_limit="
                f"{self.terminal_alignment_max_distance_growth:.3f}m "
                f"phase={phase}")
            return False
        if remaining <= self.terminal_position_tolerance:
            self.publish_status(
                "TERMINAL_POSITION_REACHED "
                f"distance={remaining:.3f}m without extra motion")
            return True
        if not self.corridor_is_clear(
                base_pose[0], base_pose[1],
                target.pose.position.x, target.pose.position.y):
            self.publish_status(
                "TERMINAL_ABORT final straight corridor is occupied")
            return False

        bearing = math.atan2(
            target.pose.position.y - base_pose[1],
            target.pose.position.x - base_pose[0],
        )
        alignment = math.atan2(
            math.sin(bearing - base_pose[2]),
            math.cos(bearing - base_pose[2]),
        )
        if alignment_limit is None:
            alignment_limit = self.terminal_alignment_max_angle
        if abs(alignment) > alignment_limit:
            self.publish_status(
                "TERMINAL_ABORT final corridor requires excessive alignment "
                f"angle={math.degrees(alignment):+.1f}deg "
                f"limit={math.degrees(alignment_limit):.1f}deg "
                f"phase={phase}")
            return False
        # The terminal line must be aimed more accurately than the general
        # final-yaw tolerance.  At the 0.45-0.70 m staging distance, the old
        # 0.08 rad (4.6 deg) limit alone can create 4-6 cm of lateral miss and
        # trigger an oscillating position/yaw correction cycle.
        alignment_tolerance = max(
            0.01, self.terminal_bearing_yaw_tolerance)
        if abs(alignment) > alignment_tolerance:
            if alignment_recheck >= self.terminal_alignment_recheck_count:
                self.publish_status(
                    "TERMINAL_ABORT target bearing did not converge after "
                    "bounded NDT rechecks "
                    f"checks={alignment_recheck}/"
                    f"{self.terminal_alignment_recheck_count} "
                    f"angle={math.degrees(alignment):+.1f}deg "
                    f"phase={phase}")
                return False
            alignment_step = bounded_alignment_step(
                alignment, self.small_spin_max_angle)
            self.publish_status(
                "TERMINAL_BEARING_STEP remeasuring target bearing after "
                "each bounded turn "
                f"check={alignment_recheck + 1}/"
                f"{self.terminal_alignment_recheck_count} "
                f"residual={math.degrees(alignment):+.1f}deg "
                f"step={math.degrees(alignment_step):+.1f}deg "
                f"phase={phase}")
            if not await self.run_segmented_spin(
                    outer_handle, alignment_step, phase,
                    yaw_tolerance=alignment_tolerance):
                self.publish_status(
                    "TERMINAL_ABORT bounded terminal alignment is blocked")
                return False

            refreshed_pose = self.current_base_pose()
            if refreshed_pose is None:
                self.publish_status(
                    "TERMINAL_ABORT pose unavailable after bearing step")
                return False
            refreshed_remaining = math.hypot(
                target.pose.position.x - refreshed_pose[0],
                target.pose.position.y - refreshed_pose[1],
            )
            if refreshed_remaining <= self.terminal_position_tolerance:
                self.publish_status(
                    "TERMINAL_POSITION_REACHED "
                    f"distance={refreshed_remaining:.3f}m during alignment")
                return True
            refreshed_bearing = math.atan2(
                target.pose.position.y - refreshed_pose[1],
                target.pose.position.x - refreshed_pose[0],
            )
            refreshed_alignment = normalize_angle(
                refreshed_bearing - refreshed_pose[2])
            if abs(refreshed_alignment) > alignment_tolerance:
                self.publish_status(
                    "TERMINAL_BEARING_RECHECK chassis translated during "
                    "turn; recomputing from measured NDT pose "
                    f"distance={refreshed_remaining:.3f}m "
                    f"new_residual="
                    f"{math.degrees(refreshed_alignment):+.1f}deg "
                    f"phase={phase}")
                return await self.run_terminal_forward(
                    outer_handle, target, attempt=attempt,
                    alignment_limit=alignment_limit, phase=phase,
                    alignment_recheck=alignment_recheck + 1,
                    alignment_origin_distance=alignment_origin_distance)

        base_pose = self.current_base_pose()
        if base_pose is None:
            self.publish_status(
                "TERMINAL_ABORT pose unavailable after alignment")
            return False
        remaining = math.hypot(
            target.pose.position.x - base_pose[0],
            target.pose.position.y - base_pose[1],
        )
        if remaining <= self.terminal_position_tolerance:
            self.publish_status(
                "TERMINAL_POSITION_REACHED "
                f"distance={remaining:.3f}m after alignment")
            return True
        if not self.corridor_is_clear(
                base_pose[0], base_pose[1],
                target.pose.position.x, target.pose.position.y):
            self.publish_status(
                "TERMINAL_ABORT corridor changed after alignment")
            return False
        if not self.drive_client.wait_for_server(timeout_sec=8.0):
            self.publish_status(
                "TERMINAL_ABORT /drive_on_heading action unavailable")
            return False

        terminal_path = self.make_straight_path(base_pose, target)
        self.plan_pub.publish(terminal_path)
        time.sleep(0.10)
        self.plan_pub.publish(terminal_path)
        self.publish_status(
            "TERMINAL_FORWARD "
            f"distance={remaining:.3f}m "
            f"speed={self.terminal_forward_speed:.3f}m/s "
            "planner=replanning-disabled behavior=/drive_on_heading")
        drive_goal = DriveOnHeading.Goal()
        drive_goal.target.x = remaining
        drive_goal.speed = self.terminal_forward_speed
        allowance = min(
            90.0,
            max(
                8.0,
                remaining
                / max(0.01, self.terminal_forward_speed)
                * self.terminal_time_allowance_factor,
            ),
        )
        drive_goal.time_allowance.sec = int(allowance)
        drive_goal.time_allowance.nanosec = int(
            (allowance - int(allowance)) * 1.0e9)
        self.last_drive_status_time = None
        self.terminal_drive_target = copy.deepcopy(target)
        self.terminal_drive_best_distance = remaining
        self.terminal_drive_cancel_reason = None
        self.drive_handle = await self.drive_client.send_goal_async(
            drive_goal,
            feedback_callback=lambda msg: self.relay_drive_feedback(
                outer_handle, msg),
        )
        if not self.drive_handle.accepted:
            self.drive_handle = None
            self.terminal_drive_target = None
            self.publish_status(
                "TERMINAL_ABORT /drive_on_heading rejected")
            return False
        self.set_motion_authorized(True)
        if self.terminal_drive_cancel_reason is not None:
            self.drive_handle.cancel_goal_async()
        drive_result = await self.drive_handle.get_result_async()
        self.set_motion_authorized(False)
        drive_cancel_reason = self.terminal_drive_cancel_reason
        self.drive_handle = None
        self.terminal_drive_target = None
        self.terminal_drive_cancel_reason = None
        if outer_handle.is_cancel_requested:
            return False
        if drive_result.status != GoalStatus.STATUS_SUCCEEDED:
            stopped_pose = self.current_base_pose()
            stopped_error = math.inf
            if stopped_pose is not None:
                stopped_error = math.hypot(
                    target.pose.position.x - stopped_pose[0],
                    target.pose.position.y - stopped_pose[1],
                )
            if stopped_error <= self.terminal_position_tolerance:
                self.publish_status(
                    "TERMINAL_POSITION_REACHED "
                    f"distance={stopped_error:.3f}m "
                    f"after /drive_on_heading status={drive_result.status} "
                    f"ndt_stop={drive_cancel_reason or 'none'}")
                return True
            if (
                    self.progress_abort_requested
                    and drive_cancel_reason is None):
                self.publish_status(
                    "TERMINAL_ABORT active safety watchdog canceled the "
                    f"drive: {self.abort_reason or 'unknown reason'}")
                return False
            if (
                    drive_cancel_reason in ("arrival", "overshoot")
                    and stopped_error
                    <= self.final_position_correction_max_distance):
                bearing = math.atan2(
                    target.pose.position.y - stopped_pose[1],
                    target.pose.position.x - stopped_pose[0],
                )
                alignment = normalize_angle(bearing - stopped_pose[2])
                correction_mode, correction_distance = (
                    choose_final_position_correction(
                        stopped_error,
                        alignment,
                        self.final_alignment_position_tolerance,
                        self.final_position_correction_alignment_limit,
                        self.final_position_micro_reverse_enabled,
                        self.final_position_micro_reverse_min_distance,
                        self.final_position_micro_reverse_max_distance,
                        self.final_position_micro_reverse_lateral_margin,
                        self.final_position_micro_reverse_alignment_limit,
                    )
                )
                self.publish_status(
                    "TERMINAL_NDT_SETTLE_CORRECTION braking completed "
                    "outside strict tolerance; selecting correction from "
                    f"the settled NDT pose mode={correction_mode} "
                    f"distance={stopped_error:.3f}m "
                    f"bearing_error={math.degrees(alignment):+.1f}deg "
                    f"ndt_stop={drive_cancel_reason}")
                if correction_mode in (
                        "micro_reverse", "aligned_micro_reverse"):
                    return await self.run_final_position_micro_reverse(
                        outer_handle,
                        target,
                        correction_distance,
                        stopped_error,
                        align_reverse=(
                            correction_mode == "aligned_micro_reverse"),
                    )
                correction_alignment_limit = (
                    correction_execution_alignment_limit(
                        correction_mode,
                        self.final_position_correction_alignment_limit,
                    )
                )
                return await self.run_terminal_forward(
                    outer_handle,
                    target,
                    attempt + 1,
                    alignment_limit=correction_alignment_limit,
                    phase=phase,
                    alignment_origin_distance=stopped_error,
                )
            if attempt <= self.terminal_drive_retry_count:
                if stopped_pose is None or not self.corridor_is_clear(
                        stopped_pose[0], stopped_pose[1],
                        target.pose.position.x, target.pose.position.y):
                    self.publish_status(
                        "TERMINAL_ABORT retry refused because the live pose "
                        "or straight corridor is unavailable "
                        f"status={drive_result.status} "
                        f"distance={stopped_error:.3f}m")
                    return False
                self.publish_status(
                    "TERMINAL_RETRY transient /drive_on_heading failure; "
                    "the live pose and corridor remain safe "
                    f"attempt={attempt}/{self.terminal_drive_retry_count} "
                    f"status={drive_result.status} "
                    f"distance={stopped_error:.3f}m "
                    f"ndt_stop={drive_cancel_reason or 'none'}")
                time.sleep(max(0.1, self.terminal_drive_retry_delay))
                return await self.run_terminal_forward(
                    outer_handle, target, attempt + 1,
                    alignment_limit=alignment_limit, phase=phase,
                    alignment_origin_distance=stopped_error)
            self.publish_status(
                "TERMINAL_ABORT /drive_on_heading "
                f"status={drive_result.status} "
                f"distance={stopped_error:.3f}m")
            return False
        final_pose = self.current_base_pose()
        if final_pose is None:
            self.publish_status(
                "TERMINAL_ABORT final pose unavailable")
            return False
        final_error = math.hypot(
            target.pose.position.x - final_pose[0],
            target.pose.position.y - final_pose[1],
        )
        if final_error > self.terminal_position_tolerance:
            if should_retry_terminal_residual(
                    final_error,
                    self.terminal_position_tolerance,
                    self.final_position_correction_max_distance,
                    attempt,
                    self.terminal_drive_retry_count):
                if not self.corridor_is_clear(
                        final_pose[0], final_pose[1],
                        target.pose.position.x, target.pose.position.y):
                    self.publish_status(
                        "TERMINAL_ABORT measured residual corridor is "
                        "occupied after /drive_on_heading success "
                        f"distance={final_error:.3f}m")
                    return False
                self.publish_status(
                    "TERMINAL_NDT_CORRECTION /drive_on_heading completed "
                    "but strict NDT position is not yet reached; "
                    f"correction={attempt}/"
                    f"{self.terminal_drive_retry_count} "
                    f"distance={final_error:.3f}m/"
                    f"{self.terminal_position_tolerance:.3f}m "
                    "rechecking geometry, costmap corridor and collision "
                    "monitor before a bounded residual correction")
                time.sleep(max(0.1, self.terminal_drive_retry_delay))
                bearing = math.atan2(
                    target.pose.position.y - final_pose[1],
                    target.pose.position.x - final_pose[0],
                )
                alignment = normalize_angle(bearing - final_pose[2])
                correction_mode, correction_distance = (
                    choose_final_position_correction(
                        final_error,
                        alignment,
                        self.final_alignment_position_tolerance,
                        self.final_position_correction_alignment_limit,
                        self.final_position_micro_reverse_enabled,
                        self.final_position_micro_reverse_min_distance,
                        self.final_position_micro_reverse_max_distance,
                        self.final_position_micro_reverse_lateral_margin,
                        self.final_position_micro_reverse_alignment_limit,
                    )
                )
                if correction_mode in (
                        "micro_reverse", "aligned_micro_reverse"):
                    self.publish_status(
                        "TERMINAL_RESIDUAL_MICRO_REVERSE avoiding a large "
                        "post-overshoot turn; executing a straight checked "
                        f"correction distance={correction_distance:.3f}m "
                        f"bearing_error={math.degrees(alignment):+.1f}deg")
                    return await self.run_final_position_micro_reverse(
                        outer_handle, target, correction_distance,
                        final_error,
                        align_reverse=(
                            correction_mode == "aligned_micro_reverse"),
                    )
                return await self.run_terminal_forward(
                    outer_handle, target, attempt + 1,
                    alignment_limit=correction_execution_alignment_limit(
                        correction_mode,
                        self.final_position_correction_alignment_limit,
                    ),
                    phase=phase,
                    alignment_origin_distance=final_error)
            self.publish_status(
                "TERMINAL_ABORT forward segment ended outside position "
                f"tolerance distance={final_error:.3f}m "
                f"limit={self.terminal_position_tolerance:.3f}m "
                f"correction_limit="
                f"{self.final_position_correction_max_distance:.3f}m "
                f"attempt={attempt}")
            return False
        self.publish_status(
            "TERMINAL_POSITION_REACHED "
            f"distance={final_error:.3f}m")
        return True

    async def run_final_alignment(self, outer_handle, target, desired_yaw):
        if self.final_alignment_completed:
            self.publish_status("FINAL_HEADING_ALREADY_REACHED")
            return True
        base_pose = self.current_base_pose()
        if base_pose is None:
            self.publish_status(
                "FINAL_HEADING_ABORT pose unavailable; cannot verify heading")
            return False
        initial_turn = math.atan2(
            math.sin(desired_yaw - base_pose[2]),
            math.cos(desired_yaw - base_pose[2]),
        )
        if abs(initial_turn) > self.final_alignment_max_angle:
            self.publish_status(
                "FINAL_HEADING_ABORT requested rotation is larger than "
                f"the bounded limit angle={math.degrees(initial_turn):+.1f}deg "
                f"limit={math.degrees(self.final_alignment_max_angle):.1f}deg")
            return False
        planned_steps = max(
            1,
            int(math.ceil(
                abs(initial_turn) / self.small_spin_max_angle)),
        )
        self.publish_status(
            "POSITION_REACHED target position accepted once; "
            "starting measured segmented final alignment "
            f"angle={math.degrees(initial_turn):+.1f}deg "
            f"steps={planned_steps} "
            f"step_limit={math.degrees(self.small_spin_max_angle):.1f}deg")

        # Re-read map->base_link before every bounded step.  This prevents
        # accumulated wheel-slip or NDT yaw error from being mistaken for a
        # correctly completed final orientation.
        for step_index in range(planned_steps + 2):
            if outer_handle.is_cancel_requested:
                return False
            base_pose = self.current_base_pose()
            if base_pose is None:
                self.publish_status(
                    "FINAL_HEADING_ABORT pose unavailable between steps")
                return False
            residual = math.atan2(
                math.sin(desired_yaw - base_pose[2]),
                math.cos(desired_yaw - base_pose[2]),
            )
            position_error = math.hypot(
                target.pose.position.x - base_pose[0],
                target.pose.position.y - base_pose[1],
            )
            if abs(residual) < self.small_spin_min_angle:
                if position_error > self.final_alignment_position_tolerance:
                    self.publish_status(
                        "FINAL_HEADING_ABORT position drifted during "
                        f"alignment distance={position_error:.3f}m "
                        f"limit="
                        f"{self.final_alignment_position_tolerance:.3f}m")
                    return False
                if not self.confirm_final_pose(target, desired_yaw):
                    # A skid-steer chassis can spring back a few degrees only
                    # after the spin command has stopped.  The confirmation
                    # samples are the source of truth: remeasure on the next
                    # bounded iteration and use the already-limited remaining
                    # correction steps instead of aborting immediately.
                    self.publish_status(
                        "FINAL_POSE_RECHECK stable confirmation detected "
                        "post-spin yaw/position rebound; remeasuring before "
                        "the next bounded correction")
                    continue
                self.final_alignment_completed = True
                self.publish_status(
                    "FINAL_HEADING_REACHED "
                    f"yaw_error={math.degrees(residual):+.1f}deg "
                    f"position_error={position_error:.3f}m "
                    f"position_limit="
                    f"{self.final_alignment_position_tolerance:.3f}m "
                    f"steps={step_index}")
                return True
            step_angle = math.copysign(
                min(abs(residual), self.small_spin_max_angle),
                residual,
            )
            self.publish_status(
                "FINAL_HEADING_STEP "
                f"step={step_index + 1}/{planned_steps + 2} "
                f"command={math.degrees(step_angle):+.1f}deg "
                f"remaining={math.degrees(residual):+.1f}deg "
                f"position_error={position_error:.3f}m")
            if not await self.run_segmented_spin(
                    outer_handle, step_angle, "final"):
                return False
            time.sleep(0.10)

        # The last bounded command can enter tolerance on the final allowed
        # iteration. Re-check the measured pose before reporting exhaustion;
        # otherwise a valid final yaw is incorrectly turned into ABORTED.
        base_pose = self.current_base_pose()
        residual = math.inf
        position_error = math.inf
        if base_pose is not None:
            residual = math.atan2(
                math.sin(desired_yaw - base_pose[2]),
                math.cos(desired_yaw - base_pose[2]),
            )
            position_error = math.hypot(
                target.pose.position.x - base_pose[0],
                target.pose.position.y - base_pose[1],
            )
            position_limit = self.final_alignment_position_tolerance
            if (
                    abs(residual) < self.small_spin_min_angle
                    and position_error <= position_limit):
                if not self.confirm_final_pose(target, desired_yaw):
                    return False
                self.final_alignment_completed = True
                self.publish_status(
                    "FINAL_HEADING_REACHED post_step_check=true "
                    f"yaw_error={math.degrees(residual):+.1f}deg "
                    f"position_error={position_error:.3f}m "
                    f"position_limit={position_limit:.3f}m "
                    f"steps={planned_steps + 2}")
                return True
        self.publish_status(
            "FINAL_HEADING_ABORT did not converge within bounded steps "
            f"yaw_error={math.degrees(residual):+.1f}deg "
            f"position_error={position_error:.3f}m "
            f"position_limit={self.final_alignment_position_tolerance:.3f}m")
        return False

    def confirm_final_pose(self, target, desired_yaw):
        """Require stable measured arrival before returning action success."""
        confirmations = max(1, self.final_pose_confirmations)
        last_position_error = math.inf
        last_yaw_error = math.inf
        for sample in range(confirmations):
            base_pose = self.current_base_pose()
            if base_pose is None:
                self.publish_status(
                    "FINAL_POSE_UNCONFIRMED map->base_link pose unavailable "
                    f"sample={sample + 1}/{confirmations}")
                return False
            last_position_error = math.hypot(
                target.pose.position.x - base_pose[0],
                target.pose.position.y - base_pose[1],
            )
            last_yaw_error = normalize_angle(desired_yaw - base_pose[2])
            if (
                    last_position_error
                    > self.final_alignment_position_tolerance
                    or abs(last_yaw_error) >= self.small_spin_min_angle):
                self.publish_status(
                    "FINAL_POSE_UNCONFIRMED measured pose is outside strict "
                    f"arrival tolerance sample={sample + 1}/{confirmations} "
                    f"position_error={last_position_error:.3f}m/"
                    f"{self.final_alignment_position_tolerance:.3f}m "
                    f"yaw_error={math.degrees(last_yaw_error):+.1f}deg/"
                    f"{math.degrees(self.small_spin_min_angle):.1f}deg")
                return False
            if sample + 1 < confirmations:
                time.sleep(max(0.0, self.final_pose_confirmation_period))
        self.publish_status(
            "FINAL_POSE_CONFIRMED measured NDT arrival is stable "
            f"samples={confirmations}/{confirmations} "
            f"position_error={last_position_error:.3f}m "
            f"yaw_error={math.degrees(last_yaw_error):+.1f}deg")
        return True

    async def run_final_position_micro_reverse(
            self, outer_handle, target, reverse_distance, initial_error,
            align_reverse=False):
        """Apply one straight, policy-checked reverse projection.

        This is not planner-selected reverse travel.  It is a bounded terminal
        correction used only after the requested position was already reached
        once and a final in-place yaw adjustment introduced a small drift.
        """
        if not self.backup_client.wait_for_server(timeout_sec=8.0):
            self.publish_status(
                "FINAL_MICRO_REVERSE_ABORT /backup action unavailable")
            return False
        start_pose = self.current_base_pose()
        if start_pose is None:
            self.publish_status(
                "FINAL_MICRO_REVERSE_ABORT pose unavailable")
            return False

        if align_reverse:
            for alignment_check in range(4):
                bearing = math.atan2(
                    target.pose.position.y - start_pose[1],
                    target.pose.position.x - start_pose[0],
                )
                reverse_alignment = normalize_angle(
                    bearing - normalize_angle(start_pose[2] + math.pi))
                if abs(reverse_alignment) <= self.terminal_bearing_yaw_tolerance:
                    break
                if alignment_check >= 3:
                    self.publish_status(
                        "FINAL_MICRO_REVERSE_ABORT reverse bearing did not "
                        "converge after three bounded measured turns")
                    return False
                self.publish_status(
                    "FINAL_MICRO_REVERSE_ALIGN measured straight-reverse "
                    f"bearing check={alignment_check + 1}/3 "
                    f"angle={math.degrees(reverse_alignment):+.1f}deg")
                if not await self.run_segmented_spin(
                        outer_handle, reverse_alignment,
                        "final_position_reverse_alignment",
                        yaw_tolerance=self.terminal_bearing_yaw_tolerance):
                    self.publish_status(
                        "FINAL_MICRO_REVERSE_ABORT reverse bearing alignment "
                        "was blocked")
                    return False
                start_pose = self.current_base_pose()
                if start_pose is None:
                    self.publish_status(
                        "FINAL_MICRO_REVERSE_ABORT pose unavailable after "
                        "reverse bearing alignment")
                    return False
            reverse_distance = math.hypot(
                target.pose.position.x - start_pose[0],
                target.pose.position.y - start_pose[1],
            )
            if not (
                    self.final_position_micro_reverse_min_distance
                    <= reverse_distance
                    <= self.final_position_micro_reverse_max_distance):
                self.publish_status(
                    "FINAL_MICRO_REVERSE_ABORT measured distance changed "
                    "outside bounded range after alignment "
                    f"distance={reverse_distance:.3f}m")
                return False

        endpoint = PoseStamped()
        endpoint.header.frame_id = "map"
        endpoint.header.stamp = self.get_clock().now().to_msg()
        endpoint.pose.position.x = (
            start_pose[0] - reverse_distance * math.cos(start_pose[2]))
        endpoint.pose.position.y = (
            start_pose[1] - reverse_distance * math.sin(start_pose[2]))
        self.set_pose_yaw(endpoint, start_pose[2])
        if not self.corridor_is_clear(
                start_pose[0], start_pose[1],
                endpoint.pose.position.x, endpoint.pose.position.y):
            self.publish_status(
                "FINAL_MICRO_REVERSE_ABORT projected reverse corridor is "
                "occupied in the global costmap")
            return False

        self.direct_reverse_distance = reverse_distance
        self.direct_reverse_plan = self.make_straight_path(
            start_pose, endpoint)
        self.publish_direct_reverse_plan()
        time.sleep(0.12)
        self.publish_direct_reverse_plan()
        self.publish_status(
            "FINAL_MICRO_REVERSE straight-only terminal correction "
            f"distance={reverse_distance:.3f}m "
            f"speed={self.straight_reverse_speed:.3f}m/s "
            "checks=global_costmap+LiDAR_collision+rear_ultrasonic")

        backup_goal = BackUp.Goal()
        backup_goal.target.x = reverse_distance
        backup_goal.speed = min(
            self.straight_reverse_speed, self.terminal_forward_speed)
        allowance = min(
            45.0,
            max(
                8.0,
                reverse_distance / max(0.01, backup_goal.speed)
                * self.reverse_time_allowance_factor,
            ),
        )
        backup_goal.time_allowance.sec = int(allowance)
        backup_goal.time_allowance.nanosec = int(
            (allowance - int(allowance)) * 1.0e9)
        self.last_backup_status_time = None
        self.backup_handle = await self.backup_client.send_goal_async(
            backup_goal,
            feedback_callback=lambda msg: self.relay_backup_feedback(
                outer_handle, msg),
        )
        if not self.backup_handle.accepted:
            self.backup_handle = None
            self.direct_reverse_plan = None
            self.clear_direct_reverse_plan()
            self.publish_status(
                "FINAL_MICRO_REVERSE_ABORT /backup request rejected")
            return False

        # Authorize immediately. Waiting for the first feedback before
        # granting the lease can deadlock with a fail-closed motion gate.
        self.set_motion_authorized(True)
        backup_result = await self.backup_handle.get_result_async()
        self.set_motion_authorized(False)
        self.backup_handle = None
        self.direct_reverse_plan = None
        self.clear_direct_reverse_plan()
        self.direct_reverse_distance = 0.0
        if outer_handle.is_cancel_requested:
            return False

        stopped_pose = self.current_base_pose()
        if stopped_pose is None:
            self.publish_status(
                "FINAL_MICRO_REVERSE_ABORT final pose unavailable")
            return False
        final_error = math.hypot(
            target.pose.position.x - stopped_pose[0],
            target.pose.position.y - stopped_pose[1],
        )
        if final_error <= self.final_alignment_position_tolerance:
            recovered_policy_abort = (
                self.clear_recovered_terminal_reverse_abort())
            if recovered_policy_abort:
                # Give the zero command and empty reverse plan one gate timer
                # cycle to replace the former reverse-only policy state before
                # starting an angular command.
                time.sleep(0.30)
            self.publish_status(
                "FINAL_MICRO_REVERSE_REACHED strict NDT position recovered "
                f"distance={final_error:.3f}m "
                f"status={backup_result.status}")
            return True
        if backup_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.publish_status(
                "FINAL_MICRO_REVERSE_ABORT action failed outside strict "
                f"tolerance status={backup_result.status} "
                f"distance={final_error:.3f}m")
            return False
        if (
                final_error >= initial_error - 0.005
                or final_error > self.final_position_correction_max_distance):
            self.publish_status(
                "FINAL_MICRO_REVERSE_ABORT no measured NDT improvement "
                f"before={initial_error:.3f}m after={final_error:.3f}m")
            return False

        # Odometry can under-run a centimetre-scale command.  If the straight
        # projection improved the NDT error but did not quite enter the strict
        # circle, finish via the general segmented forward correction.
        self.publish_status(
            "FINAL_MICRO_REVERSE_RECHECK improved but still outside strict "
            f"tolerance before={initial_error:.3f}m "
            f"after={final_error:.3f}m; using bounded forward completion")
        return await self.run_terminal_forward(
            outer_handle, target,
            alignment_limit=math.pi,
            phase="final_position_correction_after_reverse")

    async def run_final_position_correction(
            self, outer_handle, target, cycle, position_error):
        base_pose = self.current_base_pose()
        if base_pose is None:
            self.publish_status(
                "FINAL_POSITION_CORRECTION_ABORT pose unavailable")
            return False
        bearing = math.atan2(
            target.pose.position.y - base_pose[1],
            target.pose.position.x - base_pose[0],
        )
        alignment = normalize_angle(bearing - base_pose[2])
        mode, correction_distance = choose_final_position_correction(
            position_error,
            alignment,
            self.final_alignment_position_tolerance,
            self.final_position_correction_alignment_limit,
            self.final_position_micro_reverse_enabled,
            self.final_position_micro_reverse_min_distance,
            self.final_position_micro_reverse_max_distance,
            self.final_position_micro_reverse_lateral_margin,
            self.final_position_micro_reverse_alignment_limit,
        )
        self.publish_status(
            "FINAL_POSITION_CORRECTION_MODE "
            f"mode={mode} distance={position_error:.3f}m "
            f"bearing_error={math.degrees(alignment):+.1f}deg "
            f"command_distance={correction_distance:.3f}m")
        if mode in ("micro_reverse", "aligned_micro_reverse"):
            return await self.run_final_position_micro_reverse(
                outer_handle, target, correction_distance, position_error,
                align_reverse=(mode == "aligned_micro_reverse"))
        alignment_limit = correction_execution_alignment_limit(
            mode, self.final_position_correction_alignment_limit)
        if mode == "segmented_forward":
            self.publish_status(
                "FINAL_POSITION_CORRECTION_SEGMENTED target cannot be "
                "reached by a strict straight micro reverse; applying "
                "measured bounded turns before the forward correction")
        return await self.run_terminal_forward(
            outer_handle, target, attempt=cycle + 1,
            alignment_limit=alignment_limit,
            phase="final_position_correction")

    async def converge_final_pose(self, outer_handle, target, desired_yaw):
        """Alternate strict heading checks and collision-checked corrections."""
        corrections = max(0, self.final_position_correction_cycles)
        for cycle in range(corrections + 1):
            self.final_alignment_completed = False
            # Nav2's child action can report success a few centimetres outside
            # our stricter measured route tolerance.  Do not rotate first:
            # turning a skid-steer chassis changes the measured NDT position
            # and turned the small residual into an expensive correction
            # loop.  Reach the 0.05 m route circle before final yaw; only the
            # post-yaw confirmation may use the wider anti-skid tolerance.
            entry_pose = self.current_base_pose()
            if entry_pose is None:
                self.publish_status(
                    "FINAL_POSITION_PREALIGN_ABORT pose unavailable")
                return False
            entry_error = math.hypot(
                target.pose.position.x - entry_pose[0],
                target.pose.position.y - entry_pose[1],
            )
            if entry_error > self.terminal_position_tolerance:
                if cycle >= corrections:
                    self.publish_status(
                        "FINAL_POSITION_PREALIGN_ABORT exhausted bounded "
                        f"corrections distance={entry_error:.3f}m "
                        f"strict_limit={self.terminal_position_tolerance:.3f}m")
                    return False
                if entry_error > self.final_position_correction_max_distance:
                    self.publish_status(
                        "FINAL_POSITION_PREALIGN_ABORT residual exceeds "
                        f"bounded correction distance={entry_error:.3f}m "
                        f"limit={self.final_position_correction_max_distance:.3f}m")
                    return False
                self.publish_status(
                    "FINAL_POSITION_PREALIGN_CORRECTION child route ended "
                    f"at distance={entry_error:.3f}m outside strict "
                    f"route_limit={self.terminal_position_tolerance:.3f}m; "
                    "correcting position before final heading")
                if not await self.run_final_position_correction(
                        outer_handle, target, cycle, entry_error):
                    self.publish_status(
                        "FINAL_POSITION_PREALIGN_ABORT collision-checked "
                        f"correction failed cycle={cycle + 1}/{corrections}")
                    return False
                continue
            if await self.run_final_alignment(
                    outer_handle, target, desired_yaw):
                return True
            if outer_handle.is_cancel_requested:
                return False
            base_pose = self.current_base_pose()
            if base_pose is None:
                self.publish_status(
                    "FINAL_POSITION_CORRECTION_ABORT pose unavailable")
                return False
            position_error = math.hypot(
                target.pose.position.x - base_pose[0],
                target.pose.position.y - base_pose[1],
            )
            if position_error <= self.final_alignment_position_tolerance:
                # Position is already strict; the failed alignment was caused
                # by a blocked/timeout/canceled rotation and must not be hidden.
                return False
            if cycle >= corrections:
                self.publish_status(
                    "FINAL_POSITION_CORRECTION_ABORT exhausted bounded "
                    f"corrections distance={position_error:.3f}m "
                    f"cycles={corrections}")
                return False
            if position_error > self.final_position_correction_max_distance:
                self.publish_status(
                    "FINAL_POSITION_CORRECTION_ABORT drift exceeds bounded "
                    f"correction distance={position_error:.3f}m "
                    f"limit={self.final_position_correction_max_distance:.3f}m")
                return False
            self.publish_status(
                "FINAL_POSITION_CORRECTION strict NDT arrival was lost during "
                f"heading alignment; cycle={cycle + 1}/{corrections} "
                f"distance={position_error:.3f}m; checking costmap corridor "
                "and applying low-speed collision-monitored correction")
            if not await self.run_final_position_correction(
                    outer_handle, target, cycle, position_error):
                self.publish_status(
                    "FINAL_POSITION_CORRECTION_ABORT collision-checked "
                    f"correction failed cycle={cycle + 1}/{corrections}")
                return False
        return False

    async def finish_if_position_reached_after_child_abort(
            self, outer_handle, target, desired_yaw, child_status):
        """Close a child-action abort as success only after measured arrival.

        Nav2 can abort its planner/controller action while the chassis is
        already inside the terminal tolerance (for example when the local
        controller stops publishing at the goal).  The outer action must use
        the measured map->base_link pose as the source of truth, then perform
        the existing bounded final-yaw alignment before reporting success.
        """
        base_pose = self.current_base_pose()
        if base_pose is None:
            return False
        position_error = math.hypot(
            target.pose.position.x - base_pose[0],
            target.pose.position.y - base_pose[1],
        )
        tolerance = self.terminal_position_tolerance
        if position_error > tolerance:
            return False

        self.publish_status(
            "TERMINAL_POSITION_REACHED_AFTER_CHILD_ABORT "
            f"child_status={child_status} distance={position_error:.3f}m "
            f"tolerance={tolerance:.3f}m; verifying final heading")
        if not await self.converge_final_pose(
                outer_handle, target, desired_yaw):
            if outer_handle.is_cancel_requested:
                self.publish_status(
                    "CANCELED during final bounded alignment after child abort")
                outer_handle.canceled()
            else:
                self.publish_status(
                    "ABORTED position reached after child abort but final "
                    "heading alignment is collision-blocked or unverifiable")
                outer_handle.abort()
            return True

        self.publish_status(
            "SUCCEEDED child action aborted after physical arrival "
            f"distance={position_error:.3f}m yaw_request={desired_yaw:.3f}")
        self.publish_target_markers(target, reached=True)
        outer_handle.succeed()
        return True

    def publish_target_markers(self, target, reached=False):
        now = self.get_clock().now().to_msg()

        center = Marker()
        center.header.frame_id = "map"
        center.header.stamp = now
        center.ns = "nav_goal_immediate"
        center.id = 0
        center.type = Marker.CYLINDER
        center.action = Marker.ADD
        center.pose = copy.deepcopy(target.pose)
        center.pose.position.z = 0.14 if reached else 0.08
        center.scale.x = 0.36 if reached else 0.16
        center.scale.y = 0.36 if reached else 0.16
        center.scale.z = 0.24 if reached else 0.08
        center.color.r = 0.10 if reached else 0.0
        center.color.g = 1.0 if reached else 0.9
        center.color.b = 0.15 if reached else 1.0
        center.color.a = 0.95
        center.frame_locked = True

        arrow = copy.deepcopy(center)
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.pose = copy.deepcopy(target.pose)
        arrow.pose.position.z = 0.14
        arrow.scale.x = 0.55
        arrow.scale.y = 0.12
        arrow.scale.z = 0.12
        arrow.color.r = 1.0
        arrow.color.g = 0.45
        arrow.color.b = 0.0

        ring = copy.deepcopy(center)
        ring.id = 2
        ring.type = Marker.LINE_STRIP
        ring.pose.orientation.x = 0.0
        ring.pose.orientation.y = 0.0
        ring.pose.orientation.z = 0.0
        ring.pose.orientation.w = 1.0
        ring.scale.x = 0.035
        ring.color.r = 0.10 if reached else 0.0
        ring.color.g = 1.0
        ring.color.b = 0.15 if reached else 0.25
        ring.color.a = 1.0
        ring.points = []
        for index in range(49):
            angle = 2.0 * math.pi * index / 48.0
            ring.points.append(Point(
                x=0.08 * math.cos(angle),
                y=0.08 * math.sin(angle),
                z=0.02,
            ))

        label = copy.deepcopy(center)
        label.id = 3
        label.type = Marker.TEXT_VIEW_FACING
        target_yaw = yaw_from_quaternion(target.pose.orientation)
        label.pose.position.x = (
            target.pose.position.x + math.cos(target_yaw) * 1.10)
        label.pose.position.y = (
            target.pose.position.y + math.sin(target_yaw) * 1.10)
        label.pose.position.z = 1.90
        label.pose.orientation.x = 0.0
        label.pose.orientation.y = 0.0
        label.pose.orientation.z = 0.0
        label.pose.orientation.w = 1.0
        label.scale.x = 0.0
        label.scale.y = 0.0
        label.scale.z = 0.14
        label.color.r = 0.10 if reached else 0.0
        label.color.g = 1.0 if reached else 0.9
        label.color.b = 0.15 if reached else 1.0
        label.color.a = 1.0
        if reached:
            label.text = "GOAL REACHED\nNDT ARRIVAL CONFIRMED"
        else:
            label.text = (
                f"NAV GOAL {self.current_goal_id}"
                if self.current_goal_id is not None
                else "NAV GOAL"
            )

        leader = copy.deepcopy(center)
        leader.id = 4
        leader.type = Marker.LINE_LIST
        leader.pose.position.x = 0.0
        leader.pose.position.y = 0.0
        leader.pose.position.z = 0.0
        leader.pose.orientation.x = 0.0
        leader.pose.orientation.y = 0.0
        leader.pose.orientation.z = 0.0
        leader.pose.orientation.w = 1.0
        leader.scale.x = 0.025
        leader.scale.y = 0.0
        leader.scale.z = 0.0
        leader.color = copy.deepcopy(label.color)
        leader.color.a = 0.75
        leader.points = [
            Point(
                x=target.pose.position.x,
                y=target.pose.position.y,
                z=0.18,
            ),
            Point(
                x=label.pose.position.x,
                y=label.pose.position.y,
                z=1.82,
            ),
        ]

        self.marker_pub.publish(MarkerArray(
            markers=[center, arrow, ring, label, leader]))

    def on_goal(self, request):
        if self.motion_armed is not True:
            self.publish_status(
                "REJECTED motion gate is DISARMED; run "
                "seeed_arm_nav_motion.sh and wait for [MOTION_ARMED] "
                "before sending a goal")
            self.get_logger().warn(
                "rejecting navigation goal while motion gate is disarmed")
            return GoalResponse.REJECT
        if not self.motion_ready:
            self.publish_status(
                "REJECTED motion prerequisites are not ready; inspect "
                "/nav_motion_status before sending another goal")
            self.get_logger().warn(
                "rejecting navigation goal while motion prerequisites "
                "are blocked")
            return GoalResponse.REJECT
        if request.pose.header.frame_id not in ("", "map"):
            self.get_logger().error(
                f"goal frame must be map, got {request.pose.header.frame_id!r}")
            return GoalResponse.REJECT
        with self.goal_lock:
            if self.active or self.goal_reserved:
                self.get_logger().warn(
                    "rejecting a second goal while one is active or reserved")
                self.publish_status(
                    "DUPLICATE_REJECTED another goal is already active; "
                    "the existing goal will not be submitted again")
                return GoalResponse.REJECT
            self.goal_sequence += 1
            self.current_goal_id = f"G{self.goal_sequence:04d}"
            self.goal_reserved = True
            self.motion_authorized = False
        self.publish_goal_active_lease()
        pose = request.pose.pose
        self.publish_status(
            "RECEIVED exactly once "
            f"target=({pose.position.x:.3f},{pose.position.y:.3f}) "
            f"yaw={math.degrees(yaw_from_quaternion(pose.orientation)):+.1f}deg")
        return GoalResponse.ACCEPT

    def on_cancel(self, _goal_handle):
        self.set_motion_authorized(False)
        if self.inner_handle is not None:
            self.inner_handle.cancel_goal_async()
        if self.spin_handle is not None:
            self.spin_handle.cancel_goal_async()
        if self.backup_handle is not None:
            self.backup_handle.cancel_goal_async()
        if self.drive_handle is not None:
            self.drive_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    @staticmethod
    def set_pose_yaw(pose_stamped, yaw):
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)

    async def run_segmented_spin(
            self, outer_handle, angle, phase, yaw_tolerance=None):
        final_phase = phase.startswith("final")
        effective_tolerance = (
            (
                self.final_map_spin_yaw_tolerance
                if final_phase else self.small_spin_min_angle
            )
            if yaw_tolerance is None else max(0.005, yaw_tolerance)
        )
        if abs(angle) < effective_tolerance:
            return True
        if self.use_map_yaw_spin:
            return await self.run_map_yaw_segmented_spin(
                outer_handle, angle, phase,
                yaw_tolerance=effective_tolerance)
        if not self.spin_client.wait_for_server(timeout_sec=5.0):
            self.publish_status(
                f"SPIN_BLOCKED bounded /spin action unavailable phase={phase}")
            return False
        spin_steps = max(
            1,
            int(math.ceil(abs(angle) / self.small_spin_max_angle)),
        )
        spin_step_angle = angle / spin_steps
        self.publish_status(
            f"SMALL_SPIN phase={phase} "
            f"total={math.degrees(angle):+.1f}deg "
            f"steps={spin_steps} "
            f"step_limit={math.degrees(self.small_spin_max_angle):.1f}deg")
        for spin_index in range(spin_steps):
            if outer_handle.is_cancel_requested:
                return False
            self.publish_status(
                f"SMALL_SPIN phase={phase} "
                f"step={spin_index + 1}/{spin_steps} "
                f"angle={math.degrees(spin_step_angle):+.1f}deg "
                f"total={math.degrees(angle):+.1f}deg")
            spin_goal = Spin.Goal()
            spin_goal.target_yaw = float(spin_step_angle)
            spin_allowance = calculate_spin_time_allowance(
                spin_step_angle,
                self.small_spin_timeout,
                self.small_spin_effective_min_angular_speed,
                self.small_spin_timeout_factor,
                self.small_spin_timeout_margin,
            )
            self.publish_status(
                f"SMALL_SPIN_ALLOWANCE phase={phase} "
                f"step={spin_index + 1}/{spin_steps} "
                f"angle={math.degrees(spin_step_angle):+.1f}deg "
                f"allowance={spin_allowance:.1f}s")
            spin_goal.time_allowance.sec = int(spin_allowance)
            spin_goal.time_allowance.nanosec = int(
                (spin_allowance - int(spin_allowance)) * 1.0e9)
            spin_started = time.monotonic()
            self.spin_handle = await self.spin_client.send_goal_async(spin_goal)
            if not self.spin_handle.accepted:
                self.publish_status(
                    "SPIN_BLOCKED bounded small-spin request rejected "
                    f"phase={phase} step={spin_index + 1}/{spin_steps}")
                self.spin_handle = None
                return False
            self.set_motion_authorized(True)
            spin_result = await self.spin_handle.get_result_async()
            self.set_motion_authorized(False)
            self.spin_handle = None
            if outer_handle.is_cancel_requested:
                return False
            if spin_result.status != GoalStatus.STATUS_SUCCEEDED:
                spin_elapsed = time.monotonic() - spin_started
                failure_kind = (
                    "SPIN_TIMEOUT"
                    if spin_elapsed >= max(0.0, spin_allowance - 0.75)
                    else "SPIN_ABORTED"
                )
                self.publish_status(
                    f"{failure_kind} bounded small-spin failed "
                    f"phase={phase} step={spin_index + 1}/{spin_steps} "
                    f"status={spin_result.status} "
                    f"elapsed={spin_elapsed:.1f}s "
                    f"allowance={spin_allowance:.1f}s")
                return False
        return True

    def publish_map_spin_stop(self):
        zero = Twist()
        for _ in range(3):
            self.map_spin_command_pub.publish(zero)
            time.sleep(0.04)

    async def run_map_yaw_segmented_spin(
            self, outer_handle, angle, phase, yaw_tolerance=None):
        spin_speed = (
            self.final_map_spin_angular_speed
            if phase.startswith("final") else self.map_spin_angular_speed
        )
        effective_tolerance = (
            self.map_spin_yaw_tolerance
            if yaw_tolerance is None else max(0.005, yaw_tolerance)
        )
        spin_steps = max(
            1,
            int(math.ceil(abs(angle) / self.small_spin_max_angle)),
        )
        nominal_step = angle / spin_steps
        self.publish_status(
            f"MAP_YAW_SPIN phase={phase} "
            f"total={math.degrees(angle):+.1f}deg "
            f"steps={spin_steps} "
            f"step_limit={math.degrees(self.small_spin_max_angle):.1f}deg "
            f"command={spin_speed:.2f}rad/s")

        for spin_index in range(spin_steps):
            if outer_handle.is_cancel_requested:
                self.publish_map_spin_stop()
                return False
            start_pose = self.current_base_pose()
            if start_pose is None:
                self.publish_status(
                    "MAP_YAW_SPIN_ABORT pose unavailable before step "
                    f"phase={phase} step={spin_index + 1}/{spin_steps}")
                self.publish_map_spin_stop()
                return False
            target_yaw = normalize_angle(start_pose[2] + nominal_step)
            allowance = calculate_spin_time_allowance(
                nominal_step,
                self.small_spin_timeout,
                self.small_spin_effective_min_angular_speed,
                self.small_spin_timeout_factor,
                self.small_spin_timeout_margin,
            )
            started = time.monotonic()
            next_status = started
            self.publish_status(
                f"MAP_YAW_SPIN_STEP phase={phase} "
                f"step={spin_index + 1}/{spin_steps} "
                f"angle={math.degrees(nominal_step):+.1f}deg "
                f"target_yaw={math.degrees(target_yaw):+.1f}deg "
                f"allowance={allowance:.1f}s")
            self.set_motion_authorized(True)
            time.sleep(0.12)
            while time.monotonic() - started < allowance:
                if (
                        outer_handle.is_cancel_requested
                        or self.progress_abort_requested
                        or self.motion_armed is not True):
                    self.publish_map_spin_stop()
                    self.set_motion_authorized(False)
                    return False
                pose = self.current_base_pose()
                if pose is None:
                    time.sleep(0.10)
                    continue
                residual = normalize_angle(target_yaw - pose[2])
                drift = math.hypot(
                    pose[0] - start_pose[0], pose[1] - start_pose[1])
                if drift > self.map_spin_max_position_drift:
                    self.publish_status(
                        "MAP_YAW_SPIN_ABORT position drift exceeded limit "
                        f"phase={phase} step={spin_index + 1}/{spin_steps} "
                        f"drift={drift:.3f}m "
                        f"limit={self.map_spin_max_position_drift:.3f}m")
                    self.publish_map_spin_stop()
                    self.set_motion_authorized(False)
                    return False
                if abs(residual) <= effective_tolerance:
                    self.publish_map_spin_stop()
                    self.set_motion_authorized(False)
                    self.publish_status(
                        "MAP_YAW_SPIN_REACHED "
                        f"phase={phase} step={spin_index + 1}/{spin_steps} "
                        f"yaw_error={math.degrees(residual):+.1f}deg "
                        f"elapsed={time.monotonic() - started:.1f}s")
                    break
                command = Twist()
                command.angular.z = math.copysign(
                    spin_speed, residual)
                self.map_spin_command_pub.publish(command)
                now = time.monotonic()
                if now >= next_status:
                    self.publish_status(
                        "MAP_YAW_SPIN_RUNNING "
                        f"phase={phase} step={spin_index + 1}/{spin_steps} "
                        f"remaining={math.degrees(residual):+.1f}deg "
                        f"drift={drift:.3f}m "
                        f"elapsed={now - started:.1f}/{allowance:.1f}s")
                    next_status = now + 1.0
                time.sleep(0.10)
            else:
                self.publish_map_spin_stop()
                self.set_motion_authorized(False)
                self.publish_status(
                    "MAP_YAW_SPIN_TIMEOUT no measured yaw convergence "
                    f"phase={phase} step={spin_index + 1}/{spin_steps} "
                    f"allowance={allowance:.1f}s")
                return False
        return True

    def current_base_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                Time(),
                timeout=Duration(seconds=0.50),
            )
        except TransformException as error:
            self.get_logger().warn(
                f"cannot inspect direct-route heading: {error}")
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            yaw_from_quaternion(rotation),
        )

    def grid_cost(self, x, y):
        grid = self.global_costmap
        if grid is None:
            return None
        info = grid.info
        cx = int(math.floor((x - info.origin.position.x) / info.resolution))
        cy = int(math.floor((y - info.origin.position.y) / info.resolution))
        if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
            return None
        return int(grid.data[cy * info.width + cx])

    def corridor_is_clear(self, start_x, start_y, goal_x, goal_y):
        grid = self.global_costmap
        if grid is None:
            return False
        distance = math.hypot(goal_x - start_x, goal_y - start_y)
        samples = max(2, int(math.ceil(distance / grid.info.resolution)) + 1)
        for index in range(samples):
            ratio = index / (samples - 1)
            x = start_x + ratio * (goal_x - start_x)
            y = start_y + ratio * (goal_y - start_y)
            cost = self.grid_cost(x, y)
            if cost is None or cost < 0 or cost > self.maximum_cost:
                return False
        return True

    def select_approach_pose(self, target):
        yaw = yaw_from_quaternion(target.pose.orientation)
        distance = self.approach_distance
        while distance + 1.0e-6 >= self.min_approach_distance:
            approach = copy.deepcopy(target)
            approach.header.stamp = self.get_clock().now().to_msg()
            approach.pose.position.x -= distance * math.cos(yaw)
            approach.pose.position.y -= distance * math.sin(yaw)
            if self.corridor_is_clear(
                    approach.pose.position.x, approach.pose.position.y,
                    target.pose.position.x, target.pose.position.y):
                return approach, distance
            distance -= self.approach_step
        return None, None

    def relay_feedback(self, outer_handle, feedback_msg):
        self.set_motion_authorized(True)
        now = self.get_clock().now().nanoseconds / 1e9
        if (self.last_feedback_time is not None and
                now - self.last_feedback_time < self.feedback_period):
            return
        self.last_feedback_time = now
        source = feedback_msg.feedback
        feedback = NavigateToPose.Feedback()
        feedback.current_pose = source.current_pose
        feedback.navigation_time = source.navigation_time
        feedback.estimated_time_remaining = source.estimated_time_remaining
        feedback.number_of_recoveries = source.number_of_recoveries
        position = source.current_pose.pose.position
        if self.active_target is not None:
            target_distance = math.hypot(
                float(position.x) - self.active_target.pose.position.x,
                float(position.y) - self.active_target.pose.position.y,
            )
        else:
            target_distance = float(source.distance_remaining)
        feedback.distance_remaining = target_distance
        outer_handle.publish_feedback(feedback)
        self.last_distance = target_distance
        self.best_distance = min(self.best_distance, self.last_distance)
        if (
                self.terminal_handoff_enabled
                and not self.terminal_handoff_requested
                and not self.progress_abort_requested
                and self.motion_armed is True
                and target_distance <= self.terminal_handoff_distance
                and self.corridor_is_clear(
                    float(position.x), float(position.y),
                    self.active_target.pose.position.x,
                    self.active_target.pose.position.y)):
            self.terminal_handoff_requested = True
            self.publish_status(
                "TERMINAL_HANDOFF stopping 2D A* replanning before the "
                "short measured terminal segment "
                f"target_distance={target_distance:.3f}m "
                f"path_remaining={source.distance_remaining:.3f}m")
            if self.inner_handle is not None:
                self.inner_handle.cancel_goal_async()
        if self.progress_anchor_time is None:
            self.progress_anchor_x = float(position.x)
            self.progress_anchor_y = float(position.y)
            self.progress_anchor_time = now
        displacement = math.hypot(
            float(position.x) - self.progress_anchor_x,
            float(position.y) - self.progress_anchor_y,
        )
        watchdog_age = now - self.progress_anchor_time
        if displacement >= self.progress_min_displacement:
            self.progress_anchor_x = float(position.x)
            self.progress_anchor_y = float(position.y)
            self.progress_anchor_time = now
            displacement = 0.0
            watchdog_age = 0.0
        elif (
                watchdog_age >= self.progress_timeout
                and not self.progress_abort_requested):
            self.progress_abort_requested = True
            self.abort_reason = (
                "no effective chassis displacement: "
                f"motion={self.last_motion_status or 'unknown'} "
                f"moved={displacement:.3f}m "
                f"limit={self.progress_min_displacement:.3f}m/"
                f"{self.progress_timeout:.0f}s "
                f"distance={self.last_distance:.3f}m"
            )
            self.publish_status(
                f"ABORTING {self.abort_reason}")
            if self.inner_handle is not None:
                self.inner_handle.cancel_goal_async()
        recoveries = int(source.number_of_recoveries)
        if recoveries > self.last_recoveries:
            self.publish_status(
                f"RECOVERY count={recoveries} "
                f"distance={self.last_distance:.3f}m")
        self.last_recoveries = recoveries
        if (self.last_progress_status_time is None or
                now - self.last_progress_status_time >= 1.0):
            elapsed = (
                float(source.navigation_time.sec)
                + float(source.navigation_time.nanosec) / 1.0e9
            )
            self.publish_status(
                f"RUNNING target_distance={target_distance:.3f}m "
                f"path_remaining={source.distance_remaining:.3f}m "
                f"recoveries={source.number_of_recoveries} "
                f"elapsed={elapsed:.1f}s "
                f"moved={displacement:.3f}m/"
                f"{self.progress_min_displacement:.3f}m "
                f"watchdog={watchdog_age:.1f}/"
                f"{self.progress_timeout:.0f}s")
            self.last_progress_status_time = now

    async def execute(self, outer_handle):
        with self.goal_lock:
            self.active = True
            self.goal_reserved = False
            self.motion_authorized = False
        self.publish_goal_active_lease()
        self.inner_handle = None
        self.spin_handle = None
        self.backup_handle = None
        self.drive_handle = None
        self.direct_reverse_plan = None
        self.clear_direct_reverse_plan()
        self.direct_reverse_distance = 0.0
        self.last_backup_status_time = None
        self.last_drive_status_time = None
        self.terminal_drive_target = None
        self.terminal_drive_best_distance = math.inf
        self.terminal_drive_cancel_reason = None
        self.last_feedback_time = None
        self.last_progress_status_time = None
        self.last_motion_status = None
        self.last_distance = math.inf
        self.best_distance = math.inf
        self.last_recoveries = 0
        self.progress_anchor_x = None
        self.progress_anchor_y = None
        self.progress_anchor_time = None
        self.progress_abort_requested = False
        self.abort_reason = None
        self.motion_block_start_time = None
        self.motion_block_kind = None
        self.active_target = None
        self.terminal_handoff_enabled = False
        self.terminal_handoff_requested = False
        self.final_alignment_completed = False
        result = NavigateToPose.Result()
        try:
            deadline = self.get_clock().now().nanoseconds / 1e9 + self.map_wait_timeout
            while self.global_costmap is None:
                if self.get_clock().now().nanoseconds / 1e9 >= deadline:
                    self.publish_status("REJECTED global costmap unavailable")
                    outer_handle.abort()
                    return result
                # Action callbacks run in rclpy's multithreaded executor, not
                # an asyncio event loop. Another executor thread can still
                # receive the costmap while this callback briefly waits.
                time.sleep(0.10)

            target = copy.deepcopy(outer_handle.request.pose)
            target.header.frame_id = "map"
            target.header.stamp = self.get_clock().now().to_msg()
            self.active_target = copy.deepcopy(target)
            self.target_pub.publish(target)
            self.publish_target_markers(target)
            desired_yaw = yaw_from_quaternion(target.pose.orientation)
            navigation_target = copy.deepcopy(target)
            base_pose = self.current_base_pose()
            if base_pose is not None:
                entry_position_error = math.hypot(
                    target.pose.position.x - base_pose[0],
                    target.pose.position.y - base_pose[1],
                )
                # A newly received goal is position-complete only inside the
                # strict route-arrival tolerance.  The wider final-alignment
                # tolerance exists solely to absorb measured skid after the
                # route has already reached its target; using it here caused
                # nearby 0.30 m goals to rotate and report success without
                # ever translating to the requested position.
                if entry_position_error <= self.terminal_position_tolerance:
                    entry_yaw_error = normalize_angle(
                        desired_yaw - base_pose[2])
                    self.publish_status(
                        "GOAL_ALREADY_WITHIN_POSITION_TOLERANCE "
                        f"distance={entry_position_error:.3f}m "
                        f"limit="
                        f"{self.terminal_position_tolerance:.3f}m "
                        f"yaw_error="
                        f"{math.degrees(entry_yaw_error):+.1f}deg; "
                        "skipping route replanning")
                    if await self.converge_final_pose(
                            outer_handle, target, desired_yaw):
                        self.publish_status(
                            "SUCCEEDED goal was already within final pose "
                            f"tolerance yaw={desired_yaw:.3f}")
                        self.publish_target_markers(target, reached=True)
                        outer_handle.succeed()
                    elif outer_handle.is_cancel_requested:
                        self.publish_status(
                            "CANCELED while verifying an already-reached goal")
                        outer_handle.canceled()
                    else:
                        self.publish_status(
                            "ABORTED position was already reached but final "
                            "heading could not be verified safely")
                        outer_handle.abort()
                    return result
            direct_route = False
            direct_route_mode = "forward"
            forward_direct_corridor = False
            final_heading_approach_route = False
            final_alignment_after_route = False
            forward_pre_alignment = False
            initial_turn = 0.0
            final_turn = 0.0
            approach = None
            distance = None
            if base_pose is not None and self.corridor_is_clear(
                    base_pose[0], base_pose[1],
                    target.pose.position.x, target.pose.position.y):
                direct_route = True
                bearing = math.atan2(
                    target.pose.position.y - base_pose[1],
                    target.pose.position.x - base_pose[0],
                )
                forward_initial_turn = math.atan2(
                    math.sin(bearing - base_pose[2]),
                    math.cos(bearing - base_pose[2]),
                )
                reverse_heading = math.atan2(
                    math.sin(bearing + math.pi),
                    math.cos(bearing + math.pi),
                )
                reverse_initial_turn = math.atan2(
                    math.sin(reverse_heading - base_pose[2]),
                    math.cos(reverse_heading - base_pose[2]),
                )
                direct_route_mode, initial_turn = choose_direct_route_mode(
                    forward_initial_turn,
                    reverse_initial_turn,
                    self.direct_alignment_max_angle,
                    self.automatic_reverse_enabled,
                )
                if direct_route_mode is None:
                    self.publish_status(
                        "REJECTED direct travel heading unavailable: "
                        f"forward={math.degrees(forward_initial_turn):+.1f}deg "
                        f"reverse={math.degrees(reverse_initial_turn):+.1f}deg "
                        f"initial_limit="
                        f"{math.degrees(self.direct_alignment_max_angle):.1f}deg")
                    outer_handle.abort()
                    return result
                travel_heading = (
                    bearing
                    if direct_route_mode == "forward"
                    else reverse_heading
                )
                final_turn = math.atan2(
                    math.sin(desired_yaw - travel_heading),
                    math.cos(desired_yaw - travel_heading),
                )
                if abs(final_turn) > self.final_alignment_max_angle + 1.0e-6:
                    self.publish_status(
                        "FINAL_HEADING_LIMIT requested final rotation exceeds "
                        "the configured total segmented-rotation limit "
                        f"angle={math.degrees(final_turn):+.1f}deg "
                        f"limit="
                        f"{math.degrees(self.final_alignment_max_angle):.1f}deg")
                elif abs(final_turn) > self.small_spin_max_angle:
                    self.publish_status(
                        "FINAL_HEADING_SEGMENTED requested final rotation "
                        "will be split into measured small steps "
                        f"angle={math.degrees(final_turn):+.1f}deg "
                        f"step_limit="
                        f"{math.degrees(self.small_spin_max_angle):.1f}deg")
                direct_distance = math.hypot(
                    target.pose.position.x - base_pose[0],
                    target.pose.position.y - base_pose[1],
                )
                distance = min(
                    self.approach_distance,
                    max(0.10, direct_distance * 0.50),
                )
                approach = copy.deepcopy(target)
                approach.header.stamp = self.get_clock().now().to_msg()
                approach.pose.position.x -= distance * math.cos(bearing)
                approach.pose.position.y -= distance * math.sin(bearing)
                self.set_pose_yaw(approach, travel_heading)
                self.set_pose_yaw(navigation_target, travel_heading)
                self.publish_status(
                    "DIRECT_GEOMETRY "
                    f"mode={direct_route_mode} "
                    f"initial={math.degrees(initial_turn):+.1f}deg "
                    f"final={math.degrees(final_turn):+.1f}deg; "
                    "position and final heading will be executed separately")
                if direct_route_mode == "forward":
                    final_approach = None
                    final_approach_distance = None
                    # A final turn that fits in one bounded step is cheaper
                    # and safer to close at the actual target.  Constructing
                    # a 0.45-0.70 m staging pose for a 0.35 m goal placed that
                    # staging pose behind the chassis and caused an unrelated
                    # 144 degree pre-spin.
                    if (
                        self.use_final_approach_route
                        and abs(final_turn) > self.terminal_alignment_max_angle
                    ):
                        final_approach, final_approach_distance = (
                            self.select_approach_pose(target))
                    elif (
                        not self.use_final_approach_route
                        and abs(final_turn) > self.terminal_alignment_max_angle
                    ):
                        self.publish_status(
                            "FINAL_APPROACH_DISABLED globally reachable real "
                            "target is used; requested final heading will be "
                            "completed by measured segmented alignment")
                    elif abs(final_turn) >= self.small_spin_min_angle:
                        self.publish_status(
                            "FINAL_APPROACH_SKIPPED requested final heading "
                            "fits in one bounded measured step; using the "
                            "short direct position route")
                    if final_approach is not None:
                        staging_distance = math.hypot(
                            final_approach.pose.position.x - base_pose[0],
                            final_approach.pose.position.y - base_pose[1],
                        )
                        staging_bearing = math.atan2(
                            final_approach.pose.position.y - base_pose[1],
                            final_approach.pose.position.x - base_pose[0],
                        )
                        staging_turn = normalize_angle(
                            staging_bearing - base_pose[2])
                        if not final_approach_is_useful(
                                direct_distance, forward_initial_turn,
                                staging_distance, staging_turn,
                                final_approach_distance):
                            self.publish_status(
                                "FINAL_APPROACH_SKIPPED candidate staging "
                                "pose would lengthen the direct route or "
                                "require an unrelated large pre-spin "
                                f"direct={direct_distance:.2f}m "
                                f"staged_total="
                                f"{staging_distance + final_approach_distance:.2f}m "
                                f"direct_turn="
                                f"{math.degrees(forward_initial_turn):+.1f}deg "
                                f"staging_turn="
                                f"{math.degrees(staging_turn):+.1f}deg")
                            final_approach = None
                            final_approach_distance = None
                    if final_approach is not None:
                        # Reach the requested yaw geometrically instead of
                        # arriving from an unrelated bearing and then asking
                        # the skid-steer chassis to pivot at the exact target.
                        # Its measured pivot has a translation component, so
                        # a 30-45 degree terminal spin can otherwise move the
                        # base_link point 0.08-0.13 m outside strict arrival.
                        approach = final_approach
                        navigation_target = copy.deepcopy(target)
                        distance = final_approach_distance
                        approach_bearing = math.atan2(
                            approach.pose.position.y - base_pose[1],
                            approach.pose.position.x - base_pose[0],
                        )
                        initial_turn = math.atan2(
                            math.sin(approach_bearing - base_pose[2]),
                            math.cos(approach_bearing - base_pose[2]),
                        )
                        final_heading_approach_route = True
                        forward_direct_corridor = True
                        forward_pre_alignment = True
                        direct_route = False
                        final_alignment_after_route = True
                        self.publish_status(
                            "FINAL_APPROACH_ROUTE target heading differs "
                            "from the direct travel bearing; using a "
                            "costmap-checked forward approach segment "
                            f"distance={distance:.2f}m "
                            f"initial={math.degrees(initial_turn):+.1f}deg "
                            f"avoided_terminal_spin="
                            f"{math.degrees(final_turn):+.1f}deg")
                    else:
                    # Align to the forward travel bearing before asking Nav2
                    # to move. This prevents the controller from trying to
                    # solve a goal behind the chassis by selecting /backup or
                    # by drawing a large forward loop. Every spin command is
                    # still bounded by small_spin_max_angle.
                        forward_direct_corridor = True
                        forward_pre_alignment = True
                        direct_route = False
                        approach = copy.deepcopy(target)
                        navigation_target = copy.deepcopy(target)
                        self.set_pose_yaw(navigation_target, travel_heading)
                        distance = 0.0
                        final_alignment_after_route = True
            else:
                # A blocked direct corridor must not impose an unrelated
                # 0.45-0.70 m staging-space requirement behind the target.
                # Give the requested reachable pose directly to the
                # forward-only planner and let it produce the obstacle detour.
                final_approach = None
                final_approach_distance = None
                if self.use_final_approach_route:
                    final_approach, final_approach_distance = (
                        self.select_approach_pose(target))
                if final_approach is not None:
                    approach = final_approach
                    navigation_target = copy.deepcopy(target)
                    distance = final_approach_distance
                    final_heading_approach_route = True
                    final_alignment_after_route = True
                    self.publish_status(
                        "FINAL_APPROACH_ROUTE direct corridor is occupied; "
                        "the forward-only planner will detour to a "
                        "costmap-checked final approach segment "
                        f"distance={distance:.2f}m")
                else:
                    approach = copy.deepcopy(target)
                    distance = 0.0

            if forward_pre_alignment and should_run_turn_clearance(
                    initial_turn,
                    self.small_spin_min_angle,
                    self.left_turn_allowed,
                    self.right_turn_allowed,
                    self.turn_clearance_forward_enabled):
                if not await self.run_forward_for_turn_clearance(
                        outer_handle, target):
                    if outer_handle.is_cancel_requested:
                        self.publish_status(
                            "CANCELED during straight turn-clearance lead-in")
                        outer_handle.canceled()
                    else:
                        self.publish_status(
                            "ABORTED turning remained unsafe after the "
                            "collision-checked straight lead-in")
                        outer_handle.abort()
                    return result
                refreshed_pose = self.current_base_pose()
                if refreshed_pose is None:
                    self.publish_status(
                        "ABORTED pose unavailable after straight turn-"
                        "clearance lead-in")
                    outer_handle.abort()
                    return result
                refreshed_bearing = math.atan2(
                    target.pose.position.y - refreshed_pose[1],
                    target.pose.position.x - refreshed_pose[0],
                )
                initial_turn = math.atan2(
                    math.sin(refreshed_bearing - refreshed_pose[2]),
                    math.cos(refreshed_bearing - refreshed_pose[2]),
                )
                self.publish_status(
                    "TURN_CLEARANCE_REPLAN recomputed forward alignment "
                    f"angle={math.degrees(initial_turn):+.1f}deg after "
                    "straight lead-in")

            if forward_pre_alignment and not await self.run_segmented_spin(
                    outer_handle, initial_turn, "forward_pre"):
                if outer_handle.is_cancel_requested:
                    self.publish_status(
                        "CANCELED during forward pre-alignment")
                    outer_handle.canceled()
                else:
                    self.publish_status(
                        "ABORTED forward pre-alignment failed or was safety-"
                        "blocked; automatic reverse remains disabled")
                    outer_handle.abort()
                return result
            if forward_pre_alignment:
                self.publish_status(
                    "FORWARD_AFTER_SPIN automatic reverse is disabled; "
                    f"aligned={math.degrees(initial_turn):+.1f}deg and "
                    "starting the forward-only route")

            if direct_route and not await self.run_segmented_spin(
                    outer_handle, initial_turn, "pre"):
                if outer_handle.is_cancel_requested:
                    self.publish_status(
                        "CANCELED during direct-route pre-alignment")
                    outer_handle.canceled()
                    return result
                # Straight reverse requires alignment. If the behavior server
                # rejects that in-place rotation because its swept footprint is
                # occupied, retain safety and fall back to the ordinary
                # forward-only planner instead of aborting the whole goal.
                self.publish_status(
                    "REVERSE_FALLBACK pre-alignment was collision-blocked; "
                    "using the forward-only obstacle-avoidance planner")
                forward_direct_corridor = True
                direct_route = False
                direct_route_mode = "forward"
                approach = copy.deepcopy(target)
                navigation_target = copy.deepcopy(target)
                distance = 0.0
                initial_turn = 0.0
                final_turn = 0.0
            if direct_route:
                self.publish_status(
                    f"DIRECT mode={direct_route_mode} small-spin complete; "
                    "starting short route")
            elif forward_direct_corridor:
                if final_heading_approach_route:
                    self.publish_status(
                        "FORWARD_ROUTE final-heading approach is clear; "
                        "a collision-checked forward-only 2D A* path enters "
                        "the target along its requested arrow")
                else:
                    self.publish_status(
                        "FORWARD_ROUTE direct corridor is clear; initial "
                        "heading is aligned by bounded spin, then a "
                        "collision-checked forward-only 2D A* path reaches "
                        "the target position; final yaw is separate")
            else:
                self.publish_status(
                    "DETOUR direct corridor is occupied; using the "
                    "forward-only obstacle-avoidance planner")

            if not (
                    direct_route
                    and direct_route_mode == "straight_reverse"):
                # Keep Nav2's static, obstacle and inflation layers in control
                # through position arrival.  The legacy terminal handoff is a
                # compatibility option only; it stops planner replanning and
                # must never be enabled by default.
                self.terminal_handoff_enabled = self.use_terminal_handoff
                final_alignment_after_route = True

            if direct_route and direct_route_mode == "straight_reverse":
                if not self.backup_client.wait_for_server(timeout_sec=10.0):
                    self.publish_status(
                        "REJECTED /backup action unavailable")
                    outer_handle.abort()
                    return result
                start_pose = self.current_base_pose()
                if start_pose is None:
                    self.publish_status(
                        "REJECTED cannot refresh base pose before "
                        "straight reverse")
                    outer_handle.abort()
                    return result
                reverse_distance = math.hypot(
                    target.pose.position.x - start_pose[0],
                    target.pose.position.y - start_pose[1],
                )
                self.direct_reverse_distance = reverse_distance
                self.direct_reverse_plan = self.make_straight_path(
                    start_pose, target)
                # Two immediate publications satisfy the independent
                # consecutive-plan requirement before the first negative
                # velocity arrives. The timer keeps the plan fresh afterward.
                self.publish_direct_reverse_plan()
                time.sleep(0.10)
                self.publish_direct_reverse_plan()
                self.publish_status(
                    "STRAIGHT_REVERSE_CHECKED "
                    f"distance={reverse_distance:.3f}m "
                    f"points={len(self.direct_reverse_plan.poses)} "
                    "planner=none behavior=/backup")
                backup_goal = BackUp.Goal()
                backup_goal.target.x = reverse_distance
                backup_goal.speed = self.straight_reverse_speed
                allowance = min(
                    300.0,
                    max(
                        15.0,
                        reverse_distance
                        / max(0.01, self.straight_reverse_speed)
                        * self.reverse_time_allowance_factor,
                    ),
                )
                backup_goal.time_allowance.sec = int(allowance)
                backup_goal.time_allowance.nanosec = int(
                    (allowance - int(allowance)) * 1.0e9)
                self.backup_handle = await self.backup_client.send_goal_async(
                    backup_goal,
                    feedback_callback=lambda msg: self.relay_backup_feedback(
                        outer_handle, msg),
                )
                if not self.backup_handle.accepted:
                    self.publish_status(
                        "ABORTED checked straight reverse was rejected")
                    outer_handle.abort()
                    return result
                self.set_motion_authorized(False)
                backup_result = await self.backup_handle.get_result_async()
                self.set_motion_authorized(False)
                self.backup_handle = None
                self.direct_reverse_plan = None
                self.clear_direct_reverse_plan()
                if outer_handle.is_cancel_requested:
                    self.publish_status(
                        "CANCELED checked straight reverse")
                    outer_handle.canceled()
                elif (
                        backup_result.status == GoalStatus.STATUS_CANCELED
                        and self.progress_abort_requested):
                    self.publish_status(
                        f"ABORTED {self.abort_reason or 'motion watchdog'}")
                    outer_handle.abort()
                else:
                    stopped_pose = self.current_base_pose()
                    stopped_error = math.inf
                    if stopped_pose is not None:
                        stopped_error = math.hypot(
                            target.pose.position.x - stopped_pose[0],
                            target.pose.position.y - stopped_pose[1],
                        )
                    position_reached = (
                        stopped_error <= self.terminal_position_tolerance)
                    if (
                            backup_result.status
                            != GoalStatus.STATUS_SUCCEEDED
                            and not position_reached):
                        self.publish_status(
                            "ABORTED checked straight reverse "
                            f"status={backup_result.status} "
                            f"distance={stopped_error:.3f}m")
                        outer_handle.abort()
                        return result
                    if (
                            backup_result.status
                            == GoalStatus.STATUS_SUCCEEDED
                            and stopped_error
                            > self.terminal_position_tolerance):
                        self.publish_status(
                            "ABORTED /backup reported success outside "
                            f"position tolerance distance={stopped_error:.3f}m")
                        outer_handle.abort()
                        return result
                    if backup_result.status != GoalStatus.STATUS_SUCCEEDED:
                        self.publish_status(
                            "TERMINAL_POSITION_REACHED "
                            f"distance={stopped_error:.3f}m after "
                            f"/backup status={backup_result.status}")
                    if not await self.converge_final_pose(
                            outer_handle, target, desired_yaw):
                        if outer_handle.is_cancel_requested:
                            self.publish_status(
                                "CANCELED during final bounded alignment")
                            outer_handle.canceled()
                        else:
                            self.publish_status(
                                "ABORTED position reached but final heading "
                                "alignment is collision-blocked or unverifiable")
                            outer_handle.abort()
                    else:
                        self.publish_status(
                            "SUCCEEDED checked straight reverse arrival "
                            f"yaw={desired_yaw:.3f}")
                        self.publish_target_markers(target, reached=True)
                        outer_handle.succeed()
                return result

            if not self.inner_client.wait_for_server(timeout_sec=10.0):
                self.publish_status("REJECTED /navigate_through_poses unavailable")
                outer_handle.abort()
                return result

            self.approach_pub.publish(approach)
            if direct_route:
                self.publish_status(
                    f"ACTIVE approach={distance:.2f}m "
                    f"pre=({approach.pose.position.x:.3f},"
                    f"{approach.pose.position.y:.3f}) "
                    f"goal=({target.pose.position.x:.3f},"
                    f"{target.pose.position.y:.3f}) yaw={desired_yaw:.3f}")
            elif forward_direct_corridor:
                if final_heading_approach_route:
                    self.publish_status(
                        "ACTIVE forward final-heading approach "
                        f"pre=({approach.pose.position.x:.3f},"
                        f"{approach.pose.position.y:.3f}) "
                        f"goal=({target.pose.position.x:.3f},"
                        f"{target.pose.position.y:.3f}) "
                        f"yaw={desired_yaw:.3f}")
                else:
                    self.publish_status(
                        "ACTIVE forward direct route to "
                        f"goal=({target.pose.position.x:.3f},"
                        f"{target.pose.position.y:.3f}) yaw={desired_yaw:.3f}")
            else:
                self.publish_status(
                    "ACTIVE forward detour directly to "
                    f"goal=({target.pose.position.x:.3f},"
                    f"{target.pose.position.y:.3f}) yaw={desired_yaw:.3f}")

            inner_goal = NavigateThroughPoses.Goal()
            if final_heading_approach_route:
                # Stop at the checked staging pose first. Humble Navfn may
                # introduce a cusp when an approach and target are submitted
                # in one ComputePathThroughPoses request. A separate measured
                # straight handoff keeps the entire last segment forward.
                inner_goal.poses = [approach]
            elif direct_route:
                inner_goal.poses = [approach, navigation_target]
            else:
                inner_goal.poses = [navigation_target]
            inner_goal.behavior_tree = ""
            self.publish_status(
                "PLANNER forward-only GridBased 2D A* tree selected")
            send_future = self.inner_client.send_goal_async(
                inner_goal,
                feedback_callback=lambda msg: self.relay_feedback(
                    outer_handle, msg),
            )
            self.inner_handle = await send_future
            if not self.inner_handle.accepted:
                self.publish_status("ABORTED aligned NavigateThroughPoses rejected")
                outer_handle.abort()
                return result
            self.set_motion_authorized(False)

            wrapped = await self.inner_handle.get_result_async()
            self.set_motion_authorized(False)
            if outer_handle.is_cancel_requested:
                self.publish_status("CANCELED aligned navigation")
                outer_handle.canceled()
            elif (
                    wrapped.status == GoalStatus.STATUS_CANCELED
                    and self.progress_abort_requested):
                self.publish_status(
                    f"ABORTED {self.abort_reason or 'motion watchdog'} "
                    f"distance={self.last_distance:.3f}m "
                    f"best={self.best_distance:.3f}m")
                outer_handle.abort()
            elif (
                    wrapped.status == GoalStatus.STATUS_CANCELED
                    and self.terminal_handoff_requested):
                self.inner_handle = None
                if not await self.run_terminal_forward(
                        outer_handle, target):
                    if outer_handle.is_cancel_requested:
                        self.publish_status(
                            "CANCELED during terminal forward segment")
                        outer_handle.canceled()
                    else:
                        self.publish_status(
                            "ABORTED safe terminal forward segment failed; "
                            "the planner loop was not followed")
                        outer_handle.abort()
                elif not await self.converge_final_pose(
                        outer_handle, target, desired_yaw):
                    if outer_handle.is_cancel_requested:
                        self.publish_status(
                            "CANCELED during bounded final alignment")
                        outer_handle.canceled()
                    else:
                        self.publish_status(
                            "ABORTED target position reached but bounded "
                            "final alignment did not satisfy measured "
                            "yaw/position tolerances")
                        outer_handle.abort()
                else:
                    self.publish_status(
                        "SUCCEEDED terminal straight arrival "
                        f"yaw_request={desired_yaw:.3f}")
                    self.publish_target_markers(target, reached=True)
                    outer_handle.succeed()
            elif wrapped.status == GoalStatus.STATUS_CANCELED:
                self.publish_status("CANCELED aligned navigation")
                outer_handle.canceled()
            elif wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                if final_heading_approach_route:
                    self.publish_status(
                        "FINAL_APPROACH_HANDOFF Nav2 reached the checked "
                        "staging pose; aligning in bounded steps and driving "
                        "the collision-monitored straight segment to target")
                    if not await self.run_terminal_forward(
                            outer_handle, target,
                            alignment_limit=self.final_alignment_max_angle,
                            phase="final_approach"):
                        if outer_handle.is_cancel_requested:
                            self.publish_status(
                                "CANCELED during final approach segment")
                            outer_handle.canceled()
                        else:
                            self.publish_status(
                                "ABORTED costmap-checked final approach "
                                "segment failed")
                            outer_handle.abort()
                    elif not await self.converge_final_pose(
                            outer_handle, target, desired_yaw):
                        if outer_handle.is_cancel_requested:
                            self.publish_status(
                                "CANCELED during final approach verification")
                            outer_handle.canceled()
                        else:
                            self.publish_status(
                                "ABORTED final approach reached position but "
                                "strict NDT pose verification failed")
                            outer_handle.abort()
                    else:
                        self.publish_status(
                            "SUCCEEDED forward final-heading approach "
                            f"yaw={desired_yaw:.3f}")
                        self.publish_target_markers(target, reached=True)
                        outer_handle.succeed()
                elif (
                        (direct_route or final_alignment_after_route)
                        and not await self.converge_final_pose(
                            outer_handle, target, desired_yaw)):
                    if outer_handle.is_cancel_requested:
                        self.publish_status(
                            "CANCELED during final bounded alignment")
                        outer_handle.canceled()
                    else:
                        outer_handle.abort()
                else:
                    self.publish_status(
                        "SUCCEEDED aligned arrival "
                        f"yaw={desired_yaw:.3f}")
                    self.publish_target_markers(target, reached=True)
                    outer_handle.succeed()
            else:
                if await self.finish_if_position_reached_after_child_abort(
                        outer_handle, target, desired_yaw, wrapped.status):
                    return result
                self.publish_status(
                    f"ABORTED inner action status={wrapped.status} "
                    f"distance={self.last_distance:.3f}m "
                    f"best={self.best_distance:.3f}m "
                    f"recoveries={self.last_recoveries}")
                outer_handle.abort()
            return result
        except Exception as error:
            self.publish_status(f"ABORTED adapter exception: {error}")
            outer_handle.abort()
            return result
        finally:
            self.inner_handle = None
            self.spin_handle = None
            self.backup_handle = None
            self.drive_handle = None
            self.terminal_drive_target = None
            self.terminal_drive_best_distance = math.inf
            self.terminal_drive_cancel_reason = None
            self.direct_reverse_plan = None
            self.clear_direct_reverse_plan()
            self.active_target = None
            self.terminal_handoff_enabled = False
            self.terminal_handoff_requested = False
            with self.goal_lock:
                self.active = False
                self.goal_reserved = False
                self.motion_authorized = False
                self.current_goal_id = None
            self.publish_goal_active_lease()

    def destroy_node(self):
        self.outer_server.destroy()
        self.inner_client.destroy()
        self.spin_client.destroy()
        self.backup_client.destroy()
        self.drive_client.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    node = AlignedNavGoalAdapter()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
