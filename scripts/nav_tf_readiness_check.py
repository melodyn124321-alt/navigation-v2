#!/usr/bin/env python3
"""Wait for stable navigation TF in the same ROS participant as Nav2."""

import argparse
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class NavTfReadinessCheck(Node):
    def __init__(self, args):
        super().__init__("nav_tf_readiness_check")
        self.args = args
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def available(self):
        try:
            self.buffer.lookup_transform(
                self.args.map_frame,
                self.args.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.15),
            )
            if not self.args.skip_odom:
                self.buffer.lookup_transform(
                    self.args.odom_frame,
                    self.args.base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.15),
                )
            return True
        except TransformException as exc:
            odom_part = (
                "" if self.args.skip_odom
                else f" and {self.args.odom_frame}->{self.args.base_frame}"
            )
            self.get_logger().info(
                f"waiting for {self.args.map_frame}->{self.args.base_frame}"
                f"{odom_part}: {exc}",
                throttle_duration_sec=2.0,
            )
            return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--skip-odom", action="store_true")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()

    rclpy.init()
    node = NavTfReadinessCheck(args)
    deadline = time.monotonic() + args.timeout
    consecutive = 0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.available():
                consecutive += 1
                if consecutive >= args.samples:
                    odom_part = (
                        "" if args.skip_odom
                        else f" and {args.odom_frame}->{args.base_frame}"
                    )
                    node.get_logger().info(
                        f"TF READY: {args.samples} consecutive samples for "
                        f"{args.map_frame}->{args.base_frame}{odom_part}")
                    return 0
            else:
                consecutive = 0
            time.sleep(0.05)
        node.get_logger().error(
            f"TF NOT READY after {args.timeout:.1f}s; Nav2 must not start.")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
