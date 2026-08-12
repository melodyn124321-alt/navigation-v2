#!/usr/bin/env python3
"""Require a stable, stopped and goal-free state after a navigation stop."""

import argparse
import re
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ranger_msgs.msg import SystemState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class RecoveryCheck(Node):
    def __init__(self, max_fitness, required_samples):
        super().__init__("nav_estop_recovery_check")
        self.max_fitness = max_fitness
        self.required_samples = required_samples
        self.values = {}
        self.times = {}
        self.consecutive = 0
        self.last_counted_sonar_time = None
        self.success = False
        self.last_report = 0.0

        sensor_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, "/rear_ultrasonic_safety_status",
            lambda msg: self.set_value("sonar", msg.data), reliable_qos)
        self.create_subscription(
            String, "/nav_obstacle_alarm_status",
            lambda msg: self.set_value("alarm", msg.data), reliable_qos)
        self.create_subscription(
            String, "/rviz_goal_pose_bridge_status",
            lambda msg: self.set_value("bridge", msg.data), reliable_qos)
        self.create_subscription(
            Bool, "/nav_motion_armed",
            lambda msg: self.set_value("armed", bool(msg.data)), reliable_qos)
        self.create_subscription(
            String, "/hn_nav_operator_link_status",
            lambda msg: self.set_value("operator", msg.data), reliable_qos)
        self.create_subscription(
            Twist, "/cmd_vel", self.on_velocity, reliable_qos)
        self.create_subscription(
            Odometry, "/relocalization_odom",
            self.on_localization, sensor_qos)
        self.create_subscription(
            SystemState, "/system_state", self.on_system, sensor_qos)
        self.create_timer(0.25, self.evaluate)

    def set_value(self, name, value):
        self.values[name] = value
        self.times[name] = time.monotonic()

    def on_velocity(self, message):
        stopped = (
            abs(float(message.linear.x)) <= 0.001
            and abs(float(message.angular.z)) <= 0.001
        )
        self.set_value("stopped", stopped)

    def on_localization(self, message):
        self.set_value(
            "fitness", float(message.pose.covariance[0]))

    def on_system(self, message):
        self.set_value("ranger_error", int(message.error_code))
        self.set_value("ranger_vehicle", int(message.vehicle_state))
        self.set_value("ranger_control_mode", int(message.control_mode))

    def fresh(self, name, now, limit=2.0):
        return (
            name in self.times
            and now - self.times[name] <= limit
        )

    def state_ready(self, now):
        required = (
            "sonar", "alarm", "bridge", "armed",
            "operator", "stopped", "fitness", "ranger_error",
            "ranger_vehicle", "ranger_control_mode",
        )
        if not all(self.fresh(name, now) for name in required):
            return False
        sonar = str(self.values["sonar"])
        alarm = str(self.values["alarm"])
        bridge = str(self.values["bridge"])
        fitness = float(self.values["fitness"])
        return (
            "state=CLEAR" in sonar
            and "healthy=true" in sonar
            and "blocked=[false,false]" in sonar
            and "fresh=[true,true]" in sonar
            and alarm.startswith(("IDLE", "CLEAR"))
            and "READY action_server=true" in bridge
            and "active=false" in bridge
            and self.values["armed"] is False
            and "state=READY" in str(self.values["operator"])
            and self.values["stopped"] is True
            and 0.0 <= fitness <= self.max_fitness
            and self.values["ranger_error"] == 0
            and self.values["ranger_vehicle"]
            == int(SystemState.VEHICLE_STATE_NORMAL)
            and self.values["ranger_control_mode"]
            == int(SystemState.CONTROL_MODE_CAN)
        )

    def evaluate(self):
        now = time.monotonic()
        if self.state_ready(now):
            sonar_time = self.times["sonar"]
            if sonar_time != self.last_counted_sonar_time:
                self.consecutive += 1
                self.last_counted_sonar_time = sonar_time
        else:
            self.consecutive = 0
            self.last_counted_sonar_time = None
        if self.consecutive >= self.required_samples:
            self.success = True
            return
        if now - self.last_report >= 2.0:
            sonar = str(self.values.get("sonar", "missing"))
            sonar_state = re.search(r"state=\w+", sonar)
            print(
                "WAITING_ESTOP_RECOVERY "
                f"samples={self.consecutive}/{self.required_samples} "
                f"sonar={sonar_state.group(0) if sonar_state else 'missing'} "
                f"alarm={self.values.get('alarm', 'missing')} "
                f"bridge={self.values.get('bridge', 'missing')} "
                f"armed={self.values.get('armed', 'missing')} "
                f"operator={self.values.get('operator', 'missing')} "
                f"stopped={self.values.get('stopped', 'missing')} "
                f"fitness={self.values.get('fitness', 'missing')} "
                f"ranger_error={self.values.get('ranger_error', 'missing')} "
                f"ranger_vehicle={self.values.get('ranger_vehicle', 'missing')}"
                f" ranger_control_mode="
                f"{self.values.get('ranger_control_mode', 'missing')}",
                flush=True,
            )
            self.last_report = now


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--max-fitness", type=float, default=0.10)
    parser.add_argument("--required-samples", type=int, default=4)
    args = parser.parse_args()

    rclpy.init()
    node = RecoveryCheck(
        max(0.001, args.max_fitness),
        max(3, args.required_samples),
    )
    deadline = time.monotonic() + max(5.0, args.timeout)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
            if node.success:
                print(
                    "ESTOP_RECOVERY_STABLE PASS "
                    f"samples={node.consecutive} "
                    f"fitness={float(node.values['fitness']):.6f} "
                    "sonar=CLEAR alarm=IDLE goal=NONE "
                    "cmd_vel=ZERO armed=FALSE ranger_error=0 "
                    "ranger_vehicle=NORMAL ranger_control_mode=CAN",
                    flush=True,
                )
                return 0
        print(
            "ESTOP_RECOVERY_STABLE FAIL safety conditions did not remain "
            "healthy and stopped for the required window",
            file=sys.stderr,
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
