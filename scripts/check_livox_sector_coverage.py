#!/usr/bin/env python3
"""Measure live Livox elevation coverage in vehicle front/rear sectors."""

import argparse
import math

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


class SectorCheck(Node):
    def __init__(self, args):
        super().__init__("live_livox_sector_check")
        self.target_frames = args.frames
        self.sector_half_angle = args.sector_half_angle
        self.stride = max(1, args.point_stride)
        self.rotation = rpy_matrix(
            math.radians(args.roll),
            math.radians(args.pitch),
            math.radians(args.yaw),
        )
        self.frames = 0
        self.front = []
        self.rear = []
        self.done = False
        self.create_subscription(
            CustomMsg, "/livox/lidar", self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        if self.done:
            return
        self.frames += 1
        for point in msg.points[::self.stride]:
            raw = (float(point.x), float(point.y), float(point.z))
            x, y, z = (
                sum(self.rotation[row][column] * raw[column]
                    for column in range(3))
                for row in range(3)
            )
            horizontal = math.hypot(x, y)
            if horizontal < 0.10:
                continue
            elevation = math.degrees(math.atan2(z, horizontal))
            azimuth = math.degrees(math.atan2(y, x))
            if abs(azimuth) <= self.sector_half_angle:
                self.front.append(elevation)
            if abs(abs(azimuth) - 180.0) <= self.sector_half_angle:
                self.rear.append(elevation)
        self.done = self.frames >= self.target_frames


def report(name, values):
    values.sort()
    if not values:
        print(f"{name}: NO_POINTS")
        return

    def percentile(ratio):
        return values[round((len(values) - 1) * ratio)]

    print(
        f"{name}: points={len(values)} min={values[0]:+.3f}deg "
        f"p01={percentile(0.01):+.3f}deg "
        f"p50={percentile(0.50):+.3f}deg max={values[-1]:+.3f}deg")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--point-stride", type=int, default=4)
    parser.add_argument("--sector-half-angle", type=float, default=20.0)
    parser.add_argument("--roll", type=float, required=True)
    parser.add_argument("--pitch", type=float, required=True)
    parser.add_argument("--yaw", type=float, default=90.0)
    args = parser.parse_args()

    rclpy.init()
    node = SectorCheck(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        print(
            f"frames={node.frames} transform_rpy_deg="
            f"({args.roll:+.3f},{args.pitch:+.3f},{args.yaw:+.3f})")
        report("vehicle_front_sector", node.front)
        report("vehicle_rear_sector", node.rear)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
