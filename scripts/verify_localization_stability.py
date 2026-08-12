#!/usr/bin/env python3
"""Verify NDT health, persistent RViz markers, and the live chassis TF."""

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray


class LocalizationVerifier(Node):
    def __init__(self, max_fitness):
        super().__init__("localization_stability_verifier")
        self.max_fitness = max_fitness
        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        marker_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.fitness_samples = []
        self.pose_samples = []
        self.healthy_streak = 0
        self.maximum_healthy_streak = 0
        self.bad_samples = 0
        self.last_odom_monotonic = None
        self.marker_status = ""
        self.marker_samples = 0
        self.marker_lifetime_sec = 0.0
        self.tf_successes = 0
        self.odom_subscription = self.create_subscription(
            Odometry, "/relocalization_odom", self.on_odom, sensor_qos)
        self.marker_subscription = self.create_subscription(
            MarkerArray, "/relocalization_markers", self.on_markers, marker_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_odom(self, msg):
        fitness = float(msg.pose.covariance[0])
        pose = msg.pose.pose
        yaw = math.atan2(
            2.0 * (
                pose.orientation.w * pose.orientation.z
                + pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (
                pose.orientation.y * pose.orientation.y
                + pose.orientation.z * pose.orientation.z),
        )
        self.fitness_samples.append(fitness)
        self.fitness_samples = self.fitness_samples[-20:]
        self.pose_samples.append((
            float(pose.position.x), float(pose.position.y), yaw))
        self.pose_samples = self.pose_samples[-20:]
        self.last_odom_monotonic = time.monotonic()
        if math.isfinite(fitness) and fitness <= self.max_fitness:
            self.healthy_streak += 1
            self.maximum_healthy_streak = max(
                self.maximum_healthy_streak, self.healthy_streak)
        else:
            self.bad_samples += 1
            self.healthy_streak = 0

    def on_markers(self, msg):
        if not msg.markers:
            return
        for marker in msg.markers:
            if marker.text.startswith("LOCALIZED"):
                self.marker_status = marker.text
                self.marker_samples += 1
                self.marker_lifetime_sec = (
                    float(marker.lifetime.sec)
                    + float(marker.lifetime.nanosec) / 1e9
                )

    def check_tf(self):
        try:
            self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            self.tf_successes += 1
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument("--recovery-timeout", type=float, default=12.0)
    parser.add_argument("--max-fitness", type=float, default=0.10)
    parser.add_argument("--min-samples", type=int, default=2)
    parser.add_argument("--required-consecutive", type=int, default=5)
    parser.add_argument("--max-sample-age", type=float, default=2.5)
    parser.add_argument("--max-position-span", type=float, default=0.08)
    parser.add_argument("--max-yaw-span", type=float, default=0.08)
    args = parser.parse_args()

    rclpy.init()
    node = LocalizationVerifier(args.max_fitness)
    started = time.monotonic()
    minimum_deadline = started + max(0.1, args.duration)
    deadline = minimum_deadline + max(0.0, args.recovery_timeout)
    required_consecutive = max(args.min_samples, args.required_consecutive)
    passed = False

    def pose_stability():
        recent = node.pose_samples[-required_consecutive:]
        if len(recent) < required_consecutive:
            return math.inf, math.inf, False
        xs = [sample[0] for sample in recent]
        ys = [sample[1] for sample in recent]
        reference_yaw = recent[0][2]
        relative_yaws = [
            math.atan2(
                math.sin(sample[2] - reference_yaw),
                math.cos(sample[2] - reference_yaw))
            for sample in recent
        ]
        position_span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        yaw_span = max(relative_yaws) - min(relative_yaws)
        stable = (
            position_span <= args.max_position_span
            and yaw_span <= args.max_yaw_span
        )
        return position_span, yaw_span, stable

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        node.check_tf()
        now = time.monotonic()
        sample_age = (
            math.inf
            if node.last_odom_monotonic is None
            else now - node.last_odom_monotonic
        )
        marker_ok = (
            node.marker_samples >= 3
            and node.marker_status.startswith("LOCALIZED")
            and (
                node.marker_lifetime_sec == 0.0
                or node.marker_lifetime_sec >= 1.0
            )
        )
        tf_ok = node.tf_successes >= 10
        _, _, pose_stable = pose_stability()
        if (
            now >= minimum_deadline
            and node.healthy_streak >= required_consecutive
            and sample_age <= args.max_sample_age
            and marker_ok
            and tf_ok
            and pose_stable
        ):
            passed = True
            break

    recent = node.fitness_samples[-5:]
    finished = time.monotonic()
    sample_age = (
        math.inf
        if node.last_odom_monotonic is None
        else finished - node.last_odom_monotonic
    )
    marker_ok = (
        node.marker_samples >= 3
        and node.marker_status.startswith("LOCALIZED")
        and (
            node.marker_lifetime_sec == 0.0
            or node.marker_lifetime_sec >= 1.0
        )
    )
    tf_ok = node.tf_successes >= 10
    fitness_ok = (
        node.healthy_streak >= required_consecutive
        and sample_age <= args.max_sample_age
    )
    position_span, yaw_span, pose_stable = pose_stability()
    passed = passed or (fitness_ok and marker_ok and tf_ok and pose_stable)
    print(
        "LOCALIZATION_STABILITY "
        f"{'PASS' if passed else 'FAIL'} "
        f"recent_fitness={recent} marker='{node.marker_status}' "
        f"healthy_streak={node.healthy_streak}/{required_consecutive} "
        f"max_healthy_streak={node.maximum_healthy_streak} "
        f"bad_samples={node.bad_samples} "
        f"sample_age={sample_age:.2f}s "
        f"marker_samples={node.marker_samples} "
        f"marker_lifetime={node.marker_lifetime_sec:.2f}s "
        f"tf_successes={node.tf_successes} "
        f"position_span={position_span:.3f}m/"
        f"{args.max_position_span:.3f}m "
        f"yaw_span={yaw_span:.3f}rad/{args.max_yaw_span:.3f}rad "
        f"elapsed={finished - started:.2f}s"
    )
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
