#!/usr/bin/env python3
"""Bridge RViz's plain /goal_pose topic to the local NavigateToPose action.

The stock Nav2 RViz action plugin has repeatedly segfaulted on the HN host.
Keeping the action client on Seeed also avoids cross-host action discovery
being part of the operator click path.
"""

import json
import math
from pathlib import Path
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RvizGoalPoseBridge(Node):
    def __init__(self):
        super().__init__("rviz_goal_pose_bridge")
        self.goal_topic = str(
            self.declare_parameter("goal_topic", "/goal_pose_relay").value)
        self.legacy_goal_topic = str(
            self.declare_parameter("legacy_goal_topic", "").value)
        self.action_name = str(
            self.declare_parameter(
                "action_name", "/aligned_navigate_to_pose").value)
        self.required_frame = str(
            self.declare_parameter("required_frame", "map").value)
        self.duplicate_window = float(
            self.declare_parameter("duplicate_window", 3.0).value)
        self.goal_inbox_max_age = float(
            self.declare_parameter("goal_inbox_max_age", 10.0).value)
        self.queued_handoff_delay = float(
            self.declare_parameter("queued_handoff_delay", 1.0).value)
        self.queued_retry_limit = int(
            self.declare_parameter("queued_retry_limit", 5).value)
        self.goal_inbox_path = Path(str(self.declare_parameter(
            "goal_inbox_path",
            "/home/seeed/ros2/logs/hn_goal_pose_inbox.json").value))

        goal_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(
            String, "/rviz_goal_pose_bridge_status", status_qos)
        self.create_subscription(
            PoseStamped, self.goal_topic, self.on_goal_pose, goal_qos)
        if self.legacy_goal_topic and self.legacy_goal_topic != self.goal_topic:
            self.create_subscription(
                PoseStamped, self.legacy_goal_topic, self.on_goal_pose,
                goal_qos)
        self.action_client = ActionClient(
            self, NavigateToPose, self.action_name)

        self.pending = False
        self.active_goal_handle = None
        self.sequence = 0
        self.last_signature = None
        self.last_received = -math.inf
        # A relayed RViz click can arrive through DDS and the SSH fallback at
        # different times.  The old time-only filter expired while a long goal
        # was still active, so the delayed copy canceled and restarted that
        # same goal.  Track active, queued and recently-finished signatures
        # independently so only a genuinely different click can replace it.
        self.current_signature = None
        self.queued_signature = None
        self.last_finished_signature = None
        self.last_finished_at = -math.inf
        self.last_event = "STARTING"
        self.queued_goal = None
        self.queued_dispatch_at = math.inf
        self.queued_retry_count = 0
        self.current_goal_message = None
        self.current_retry_count = 0
        self.latest_source_created_at_ns = 0
        self.create_timer(0.10, self.poll_goal_inbox)
        self.create_timer(0.10, self.dispatch_queued_goal)
        self.create_timer(0.50, self.publish_heartbeat)
        self.get_logger().info(
            f"RViz goal bridge: {self.goal_topic} (legacy "
            f"{self.legacy_goal_topic}) -> {self.action_name}; "
            "duplicates are rejected before active-goal replacement")

    def poll_goal_inbox(self):
        try:
            payload = json.loads(
                self.goal_inbox_path.read_text(encoding="utf-8"))
            self.goal_inbox_path.unlink(missing_ok=True)
        except FileNotFoundError:
            return
        except (OSError, ValueError, TypeError) as error:
            self.goal_inbox_path.unlink(missing_ok=True)
            self.publish_event(f"REJECTED invalid SSH goal inbox: {error}")
            return

        try:
            created_at_ns = int(payload["created_at_ns"])
            age_sec = (time.time_ns() - created_at_ns) / 1e9
            if age_sec > self.goal_inbox_max_age or age_sec < -5.0:
                self.publish_event(
                    "REJECTED stale SSH goal inbox "
                    f"age={age_sec:.3f}s max={self.goal_inbox_max_age:.3f}s")
                return
            message = PoseStamped()
            message.header.frame_id = str(payload["frame"])
            message.header.stamp = self.get_clock().now().to_msg()
            message.pose.position.x = float(payload["x"])
            message.pose.position.y = float(payload["y"])
            message.pose.position.z = float(payload["z"])
            message.pose.orientation.x = float(payload["qx"])
            message.pose.orientation.y = float(payload["qy"])
            message.pose.orientation.z = float(payload["qz"])
            message.pose.orientation.w = float(payload["qw"])
        except (KeyError, TypeError, ValueError) as error:
            self.publish_event(f"REJECTED invalid SSH goal fields: {error}")
            return
        self.get_logger().info("SSH_INBOX goal received")
        self.on_goal_pose(message, source_created_at_ns=created_at_ns)

    def queue_goal(self, message, signature, retry_count, reason):
        self.queued_goal = message
        self.queued_signature = signature
        self.queued_retry_count = retry_count
        self.queued_dispatch_at = (
            time.monotonic() + max(self.queued_handoff_delay, 0.2))
        self.publish_event(
            f"QUEUED {reason}; clean_handoff_in="
            f"{max(self.queued_handoff_delay, 0.2):.1f}s "
            f"retry={retry_count}/{self.queued_retry_limit}")

    def dispatch_queued_goal(self):
        if self.queued_goal is None:
            return
        if self.pending or self.active_goal_handle is not None:
            return
        if time.monotonic() < self.queued_dispatch_at:
            return
        if not self.action_client.server_is_ready():
            self.queued_dispatch_at = time.monotonic() + 0.5
            return
        message = self.queued_goal
        retry_count = self.queued_retry_count
        self.queued_goal = None
        self.queued_signature = None
        self.queued_dispatch_at = math.inf
        self.submit_goal(message, retry_count=retry_count)

    @staticmethod
    def quaternion_norm(orientation):
        return math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )

    @staticmethod
    def signature(message):
        pose = message.pose
        return tuple(round(value, 4) for value in (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ))

    def publish_event(self, text):
        self.last_event = text
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def publish_heartbeat(self):
        ready = self.action_client.server_is_ready()
        active = (
            self.pending or self.active_goal_handle is not None
            or self.queued_goal is not None)
        self.status_pub.publish(String(data=(
            f"READY action_server={str(ready).lower()} "
            f"active={str(active).lower()} "
            f"last_event={self.last_event}"
        )))

    def on_goal_pose(self, message, source_created_at_ns=None):
        now = time.monotonic()
        if source_created_at_ns is None:
            stamp = message.header.stamp
            source_created_at_ns = int(stamp.sec) * 1_000_000_000 + int(
                stamp.nanosec)
            if source_created_at_ns <= 0:
                source_created_at_ns = time.time_ns()
        age_sec = (time.time_ns() - source_created_at_ns) / 1e9
        if age_sec > self.goal_inbox_max_age or age_sec < -5.0:
            self.publish_event(
                "REJECTED stale/future goal sample "
                f"age={age_sec:.3f}s max={self.goal_inbox_max_age:.3f}s")
            return
        if (
            self.latest_source_created_at_ns > 0
            and source_created_at_ns < self.latest_source_created_at_ns
        ):
            self.publish_event(
                "REJECTED out-of-order goal sample "
                f"lag={(self.latest_source_created_at_ns - source_created_at_ns) / 1e9:.3f}s")
            return
        self.latest_source_created_at_ns = max(
            self.latest_source_created_at_ns, source_created_at_ns)
        frame = message.header.frame_id.strip()
        if frame != self.required_frame:
            self.publish_event(
                f"REJECTED frame={frame or 'empty'} "
                f"required={self.required_frame}")
            return
        norm = self.quaternion_norm(message.pose.orientation)
        if not math.isfinite(norm) or norm < 0.5:
            self.publish_event("REJECTED invalid goal orientation")
            return

        signature = self.signature(message)
        if (
            (self.pending or self.active_goal_handle is not None)
            and signature == self.current_signature
        ):
            self.publish_event(
                "DUPLICATE_REJECTED active goal is already executing")
            return
        if self.queued_goal is not None and signature == self.queued_signature:
            self.publish_event(
                "DUPLICATE_REJECTED replacement goal is already queued")
            return
        if (
            signature == self.last_finished_signature
            and now - self.last_finished_at < self.duplicate_window
        ):
            self.publish_event(
                "DUPLICATE_REJECTED recently finished goal cooldown")
            return
        if (
            signature == self.last_signature
            and now - self.last_received < self.duplicate_window
        ):
            self.publish_event(
                "DUPLICATE_REJECTED repeated /goal_pose sample")
            return
        if self.pending or self.active_goal_handle is not None:
            self.queue_goal(
                message, signature, 0,
                "replacing the active goal after cancellation")
            if self.active_goal_handle is not None:
                self.active_goal_handle.cancel_goal_async()
            return
        if not self.action_client.server_is_ready():
            self.publish_event(
                f"REJECTED action server unavailable: {self.action_name}")
            return

        self.submit_goal(message, retry_count=0)

    def submit_goal(self, message, retry_count=0):
        now = time.monotonic()
        signature = self.signature(message)
        if not self.action_client.server_is_ready():
            self.queue_goal(
                message, signature, retry_count,
                "waiting for action server")
            return
        self.sequence += 1
        goal_id = f"T{self.sequence:04d}"
        self.last_signature = signature
        self.last_received = now
        self.current_signature = signature
        self.current_goal_message = message
        self.current_retry_count = retry_count
        self.pending = True

        goal = NavigateToPose.Goal()
        goal.pose = message
        goal.pose.header.frame_id = self.required_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.behavior_tree = ""

        self.publish_event(
            "RECEIVED "
            f"goal_id={goal_id} "
            f"target=({goal.pose.pose.position.x:.3f},"
            f"{goal.pose.pose.position.y:.3f})")
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result, identifier=goal_id:
            self.on_goal_response(result, identifier))

    def on_goal_response(self, future, goal_id):
        self.pending = False
        try:
            goal_handle = future.result()
        except Exception as error:  # rclpy transport exceptions vary by build
            failed_message = self.current_goal_message
            failed_signature = self.current_signature
            retry_count = self.current_retry_count + 1
            self.current_signature = None
            self.current_goal_message = None
            self.publish_event(
                f"REJECTED goal_id={goal_id} send_error={error}")
            if failed_message is not None and retry_count <= self.queued_retry_limit:
                self.queue_goal(
                    failed_message, failed_signature, retry_count,
                    "retrying transient action send failure")
            return
        if not goal_handle.accepted:
            failed_message = self.current_goal_message
            failed_signature = self.current_signature
            retry_count = self.current_retry_count + 1
            self.current_signature = None
            self.current_goal_message = None
            self.publish_event(
                f"REJECTED goal_id={goal_id} action_server_rejected")
            if failed_message is not None and retry_count <= self.queued_retry_limit:
                self.queue_goal(
                    failed_message, failed_signature, retry_count,
                    "retrying clean action handoff")
            return

        self.active_goal_handle = goal_handle
        self.publish_event(f"ACCEPTED goal_id={goal_id}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, identifier=goal_id:
            self.on_goal_result(result, identifier))

    def on_goal_result(self, future, goal_id):
        self.active_goal_handle = None
        self.current_goal_message = None
        self.current_retry_count = 0
        finished_signature = self.current_signature
        self.current_signature = None
        if finished_signature is not None:
            self.last_finished_signature = finished_signature
            self.last_finished_at = time.monotonic()
        try:
            wrapped_result = future.result()
            status = int(wrapped_result.status)
        except Exception as error:  # rclpy transport exceptions vary by build
            self.publish_event(
                f"ABORTED goal_id={goal_id} result_error={error}")
        else:
            labels = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
            }
            label = labels.get(status, f"FINISHED_STATUS_{status}")
            self.publish_event(f"{label} goal_id={goal_id}")

        if self.queued_goal is not None:
            self.queued_dispatch_at = (
                time.monotonic() + max(self.queued_handoff_delay, 0.2))


def main():
    rclpy.init()
    node = RvizGoalPoseBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
