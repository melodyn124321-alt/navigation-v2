#!/usr/bin/env python3
"""Verify that a stationary Livox IMU matches its calibrated mounting pose."""

import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class ImuCollector(Node):
    def __init__(self, topic):
        super().__init__("verify_live_livox_mount")
        qos = QoSProfile(depth=500)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.samples = []
        self.create_subscription(Imu, topic, self.on_imu, qos)

    def on_imu(self, message):
        self.samples.append((
            float(message.linear_acceleration.x),
            float(message.linear_acceleration.y),
            float(message.linear_acceleration.z),
            float(message.angular_velocity.x),
            float(message.angular_velocity.y),
            float(message.angular_velocity.z),
        ))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/livox/imu")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--min-samples", type=int, default=500)
    parser.add_argument("--max-abs-roll", type=float, default=3.0)
    parser.add_argument("--max-abs-pitch", type=float, default=3.0)
    parser.add_argument("--expected-roll", type=float)
    parser.add_argument("--expected-pitch", type=float)
    parser.add_argument("--max-angle-error", type=float, default=0.75)
    parser.add_argument("--max-gyro-std", type=float, default=0.03)
    args = parser.parse_args()

    rclpy.init()
    node = ImuCollector(args.topic)
    started = time.monotonic()
    first_sample_at = None
    try:
        while rclpy.ok() and time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.10)
            if node.samples and first_sample_at is None:
                first_sample_at = time.monotonic()
            if (
                first_sample_at is not None
                and time.monotonic() - first_sample_at >= args.duration
            ):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(node.samples) < args.min_samples:
        print(
            f"RESULT: FAIL received only {len(node.samples)} IMU samples "
            f"from {args.topic}; need at least {args.min_samples}",
            file=sys.stderr,
        )
        return 2

    columns = list(zip(*node.samples))
    means = [statistics.fmean(column) for column in columns]
    stddev = [statistics.pstdev(column) for column in columns]
    ax, ay, az = means[:3]
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
    print(
        f"stationary_imu: samples={len(node.samples)} "
        f"accel_mean=({ax:.5f},{ay:.5f},{az:.5f}) "
        f"accel_std=({stddev[0]:.5f},{stddev[1]:.5f},{stddev[2]:.5f}) "
        f"norm={accel_norm:.5f}"
    )
    print(
        f"gyro_std=({stddev[3]:.5f},{stddev[4]:.5f},{stddev[5]:.5f})rad/s "
        f"gravity_tilt: roll={roll:+.3f}deg pitch={pitch:+.3f}deg"
    )

    moving = max(stddev[3:]) > args.max_gyro_std
    calibrated_mode = (
        args.expected_roll is not None
        and args.expected_pitch is not None
    )
    if calibrated_mode:
        roll_error = abs(roll - args.expected_roll)
        pitch_error = abs(pitch - args.expected_pitch)
        tilted = (
            roll_error > args.max_angle_error
            or pitch_error > args.max_angle_error
        )
        print(
            "mount_calibration: "
            f"expected_roll={args.expected_roll:+.3f}deg "
            f"expected_pitch={args.expected_pitch:+.3f}deg "
            f"error=({roll_error:.3f},{pitch_error:.3f})deg "
            f"limit={args.max_angle_error:.3f}deg"
        )
    else:
        tilted = (
            abs(roll) > args.max_abs_roll
            or abs(pitch) > args.max_abs_pitch
        )
    if moving:
        print(
            "RESULT: FAIL chassis or LiDAR moved during the mount check; "
            "keep it stationary and retry",
            file=sys.stderr,
        )
        return 3
    if tilted:
        if calibrated_mode:
            print(
                "RESULT: FAIL LiDAR mounting pose differs from calibration: "
                f"roll_error={roll_error:.3f}deg, "
                f"pitch_error={pitch_error:.3f}deg",
                file=sys.stderr,
            )
        else:
            print(
                "RESULT: FAIL LiDAR is not level: "
                f"|roll|={abs(roll):.3f}/{args.max_abs_roll:.3f}deg, "
                f"|pitch|={abs(pitch):.3f}/{args.max_abs_pitch:.3f}deg",
                file=sys.stderr,
            )
        return 4
    if calibrated_mode:
        print("RESULT: PASS live LiDAR calibrated mounting pose")
    else:
        print("RESULT: PASS live LiDAR mounting angle")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
