#!/usr/bin/env python3
"""Stop only FAST-LIO when odometry diverges or scan matching collapses."""

import collections
import math
import os
import signal
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class FastlioWatchdog(Node):
    def __init__(self):
        super().__init__("fastlio_live_watchdog")
        self.declare_parameter(
            "fastlio_log", "/home/seeed/ros2/logs/live_fastlio.log")
        self.declare_parameter(
            "trip_file", "/home/seeed/ros2/logs/fastlio_watchdog.trip")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("startup_grace_sec", 12.0)
        self.declare_parameter("max_position_jump_m", 0.75)
        self.declare_parameter("max_speed_mps", 3.0)
        self.declare_parameter("max_abs_position_m", 100.0)
        self.declare_parameter("no_effective_limit", 40)
        self.declare_parameter("no_effective_window_sec", 10.0)
        self.declare_parameter("dry_run", False)

        self.fastlio_log = self.get_parameter(
            "fastlio_log").get_parameter_value().string_value
        self.trip_file = self.get_parameter(
            "trip_file").get_parameter_value().string_value
        self.startup_grace = self.get_parameter(
            "startup_grace_sec").value
        self.max_jump = self.get_parameter("max_position_jump_m").value
        self.max_speed = self.get_parameter("max_speed_mps").value
        self.max_abs_position = self.get_parameter(
            "max_abs_position_m").value
        self.no_effective_limit = self.get_parameter(
            "no_effective_limit").value
        self.no_effective_window = self.get_parameter(
            "no_effective_window_sec").value
        self.dry_run = self.get_parameter("dry_run").value

        self.started = time.monotonic()
        self.previous = None
        self.no_effective = collections.deque()
        self.triggered = False
        self.log_handle = None
        self.log_identity = None

        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value,
            self.on_odometry, qos)
        self.trip_publisher = self.create_publisher(
            String, "/fastlio_watchdog/trip", 1)
        self.create_timer(0.2, self.poll_log)
        self.create_timer(1.0, self.report_health)
        self.get_logger().info(
            f"armed: jump={self.max_jump:.2f}m "
            f"speed={self.max_speed:.2f}m/s "
            f"abs={self.max_abs_position:.1f}m "
            f"NoEffective={self.no_effective_limit}/"
            f"{self.no_effective_window:.1f}s "
            f"grace={self.startup_grace:.1f}s dry_run={self.dry_run}")

    def in_grace(self):
        return time.monotonic() - self.started < self.startup_grace

    def on_odometry(self, message):
        if self.triggered:
            return
        p = message.pose.pose.position
        xyz = (float(p.x), float(p.y), float(p.z))
        stamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9)
        if not all(math.isfinite(value) for value in xyz):
            self.trip("non-finite odometry position")
            return
        magnitude = math.sqrt(sum(value * value for value in xyz))
        if magnitude > self.max_abs_position:
            self.trip(
                f"absolute odometry position {magnitude:.3f}m exceeds "
                f"{self.max_abs_position:.3f}m")
            return
        if self.previous is not None:
            old_stamp, old_xyz = self.previous
            dt = stamp - old_stamp
            distance = math.sqrt(sum(
                (value - old) ** 2 for value, old in zip(xyz, old_xyz)))
            if not self.in_grace() and 0.0 < dt <= 1.0:
                if distance > self.max_jump:
                    self.trip(
                        f"odometry jump {distance:.3f}m in {dt:.3f}s "
                        f"exceeds {self.max_jump:.3f}m")
                    return
                speed = distance / dt
                if speed > self.max_speed:
                    self.trip(
                        f"odometry speed {speed:.3f}m/s exceeds "
                        f"{self.max_speed:.3f}m/s")
                    return
        self.previous = (stamp, xyz)

    def open_log_if_needed(self):
        try:
            stat = os.stat(self.fastlio_log)
        except FileNotFoundError:
            return
        identity = (stat.st_dev, stat.st_ino)
        if self.log_handle is None or identity != self.log_identity:
            if self.log_handle is not None:
                self.log_handle.close()
            self.log_handle = open(
                self.fastlio_log, "r", encoding="utf-8", errors="replace")
            self.log_identity = identity

    def poll_log(self):
        if self.triggered:
            return
        self.open_log_if_needed()
        if self.log_handle is None:
            return
        now = time.monotonic()
        for line in self.log_handle.readlines():
            if "Integer indices would overflow" in line:
                self.trip("PCL voxel index overflow in FAST-LIO log")
                return
            if "No Effective Points!" in line:
                self.no_effective.append(now)
        cutoff = now - self.no_effective_window
        while self.no_effective and self.no_effective[0] < cutoff:
            self.no_effective.popleft()
        if (
            not self.in_grace()
            and len(self.no_effective) >= self.no_effective_limit
        ):
            self.trip(
                f"No Effective Points count {len(self.no_effective)} in "
                f"{self.no_effective_window:.1f}s exceeds "
                f"{self.no_effective_limit}")

    @staticmethod
    def fastlio_pids():
        pids = []
        for name in os.listdir("/proc"):
            if not name.isdigit():
                continue
            try:
                with open(
                    f"/proc/{name}/comm", "r", encoding="ascii"
                ) as handle:
                    if handle.read().strip() == "fastlio_mapping":
                        pids.append(int(name))
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
        return pids

    def trip(self, reason):
        if self.triggered:
            return
        self.triggered = True
        timestamp = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        message = f"{timestamp} FASTLIO_AUTO_STOP: {reason}"
        self.get_logger().fatal(message)
        os.makedirs(os.path.dirname(self.trip_file), exist_ok=True)
        with open(self.trip_file, "w", encoding="utf-8") as handle:
            handle.write(message + "\n")
        self.trip_publisher.publish(String(data=message))
        if self.dry_run:
            self.get_logger().warn("dry_run=true: FAST-LIO was not signalled")
            return
        pids = self.fastlio_pids()
        for pid in pids:
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        self.get_logger().fatal(
            f"sent SIGINT to FAST-LIO pids={pids}; "
            "Livox and rosbag were untouched")

    def report_health(self):
        if not self.triggered:
            self.get_logger().debug(
                "healthy: odometry="
                f"{'seen' if self.previous else 'waiting'} "
                f"no_effective_window={len(self.no_effective)}")


def main():
    rclpy.init()
    node = FastlioWatchdog()
    try:
        rclpy.spin(node)
    finally:
        if node.log_handle is not None:
            node.log_handle.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
