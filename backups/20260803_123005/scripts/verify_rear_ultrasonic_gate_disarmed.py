#!/usr/bin/env python3
"""Exercise a remapped safety gate without touching the physical /cmd_vel."""

import importlib.util
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ranger_msgs.msg import SystemState
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


MODULE_PATH = Path(__file__).with_name("nav_motion_safety_gate.py")
SPEC = importlib.util.spec_from_file_location("nav_motion_safety_gate", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TestDriver(Node):
    def __init__(self):
        super().__init__("rear_ultrasonic_gate_test_driver")
        self.command = Twist()
        self.ranges = [math.inf, math.inf]
        self.operator_present = True
        self.goal_active = True
        self.outputs = []
        self.status = ""
        self.ultrasonic_status = ""
        self.reverse_allowed = None
        self.left_turn_allowed = None
        self.right_turn_allowed = None

        self.command_pub = self.create_publisher(
            Twist, "/test_ultrasonic/cmd_in", 10)
        self.localization_pub = self.create_publisher(
            Odometry, "/test_ultrasonic/localization", 10)
        self.odom_pub = self.create_publisher(
            Odometry, "/test_ultrasonic/odom", 10)
        self.system_pub = self.create_publisher(
            SystemState, "/test_ultrasonic/system_state", 10)
        self.obstacle_pub = self.create_publisher(
            String, "/test_ultrasonic/obstacle_status", 10)
        self.health_pub = self.create_publisher(
            Bool, "/test_ultrasonic/health", 10)
        self.operator_pub = self.create_publisher(
            Bool, "/test_ultrasonic/operator", 10)
        self.goal_active_pub = self.create_publisher(
            Bool, "/test_ultrasonic/goal_active", 10)
        self.range_pubs = (
            self.create_publisher(
                Range, "/test_ultrasonic/range_1", 10),
            self.create_publisher(
                Range, "/test_ultrasonic/range_2", 10),
        )
        self.create_subscription(
            Twist, "/test_ultrasonic/cmd_out", self.on_output, 10)
        self.create_subscription(
            String, "/test_ultrasonic/status", self.on_status, 10)
        self.create_subscription(
            String, "/test_ultrasonic/safety_status",
            self.on_ultrasonic_status, 10)
        self.create_subscription(
            Bool, "/test_ultrasonic/reverse_allowed",
            self.on_reverse_allowed, 10)
        self.create_subscription(
            Bool, "/test_ultrasonic/left_turn_allowed",
            self.on_left_turn_allowed, 10)
        self.create_subscription(
            Bool, "/test_ultrasonic/right_turn_allowed",
            self.on_right_turn_allowed, 10)
        self.arm_client = self.create_client(
            SetBool, "/test_ultrasonic/set_enabled")
        self.create_timer(0.05, self.publish_fast_inputs)
        self.create_timer(0.25, self.publish_sonar)

    def publish_fast_inputs(self):
        stamp = self.get_clock().now().to_msg()
        self.command_pub.publish(self.command)

        localization = Odometry()
        localization.header.stamp = stamp
        localization.pose.covariance[0] = 0.01
        self.localization_pub.publish(localization)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        system = SystemState()
        system.header.stamp = stamp
        system.control_mode = SystemState.CONTROL_MODE_CAN
        system.error_code = 0
        self.system_pub.publish(system)
        self.obstacle_pub.publish(
            String(data="ok received=1 published=1 age=0.010s"))
        self.health_pub.publish(Bool(data=True))
        self.operator_pub.publish(Bool(data=self.operator_present))
        self.goal_active_pub.publish(Bool(data=self.goal_active))

    def publish_sonar(self):
        for index, publisher in enumerate(self.range_pubs):
            message = Range()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = f"test_rear_{index + 1}"
            message.radiation_type = Range.ULTRASOUND
            message.field_of_view = 0.30
            message.min_range = 0.08
            message.max_range = 2.0
            message.range = self.ranges[index]
            publisher.publish(message)

    def on_output(self, message):
        self.outputs.append(
            (time.monotonic(), float(message.linear.x),
             float(message.angular.z)))
        self.outputs = self.outputs[-200:]

    def on_status(self, message):
        self.status = message.data

    def on_ultrasonic_status(self, message):
        self.ultrasonic_status = message.data

    def on_reverse_allowed(self, message):
        self.reverse_allowed = bool(message.data)

    def on_left_turn_allowed(self, message):
        self.left_turn_allowed = bool(message.data)

    def on_right_turn_allowed(self, message):
        self.right_turn_allowed = bool(message.data)

    def recent_linear_peak(self, since):
        values = [
            abs(linear) for stamp, linear, _angular in self.outputs
            if stamp >= since
        ]
        return max(values, default=0.0)

    def recent_nonzero(self, since):
        return any(
            stamp >= since and (abs(linear) > 0.001 or abs(angular) > 0.001)
            for stamp, linear, angular in self.outputs
        )

    def recent_all_zero(self, since):
        selected = [
            (linear, angular) for stamp, linear, angular in self.outputs
            if stamp >= since
        ]
        return bool(selected) and all(
            abs(linear) <= 0.001 and abs(angular) <= 0.001
            for linear, angular in selected
        )


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}")


def main():
    args = [
        "--ros-args",
        "-p", "input_topic:=/test_ultrasonic/cmd_in",
        "-p", "output_topic:=/test_ultrasonic/cmd_out",
        "-p", "arm_service:=/test_ultrasonic/set_enabled",
        "-p", "reverse_straight_only:=false",
        "-p", "localization_timeout:=0.50",
        "-p", "odom_timeout:=0.50",
        "-p", "chassis_timeout:=0.50",
        "-p", "command_timeout:=0.50",
        "-p", "obstacle_status_timeout:=0.50",
        "-p", "ultrasonic_timeout:=0.75",
        "-p", "ultrasonic_stop_distance:=0.22",
        "-p", "ultrasonic_clear_distance:=0.35",
        "-p", "ultrasonic_self_echo_enabled:=true",
        "-p", "ultrasonic_self_echo_min_distance:=0.16",
        "-p", "ultrasonic_self_echo_max_distance:=0.22",
        "-p", "ultrasonic_sensor_to_rear_edge:=0.20",
        "-p", "ultrasonic_required_tail_clearance:=0.02",
        "-p", "ultrasonic_braking_margin:=0.00",
        "-p", "ultrasonic_noise_margin:=0.00",
        "-p", "ultrasonic_block_hold_sec:=1.50",
        "-p", "ultrasonic_clear_samples_required:=3",
        "-p", "ultrasonic_no_echo_clear_samples_required:=8",
        "-p", "ultrasonic_turn_guard_enabled:=true",
        "-p", "ultrasonic_reverse_speed_taper_enabled:=true",
        "-p", "goal_lease_required:=true",
        "-p", "goal_lease_timeout:=0.50",
        "-r", "/relocalization_odom:=/test_ultrasonic/localization",
        "-r", "/odom:=/test_ultrasonic/odom",
        "-r", "/system_state:=/test_ultrasonic/system_state",
        "-r", "/nav_obstacle_cloud_status:=/test_ultrasonic/obstacle_status",
        "-r", "/ultrasonic/healthy:=/test_ultrasonic/health",
        "-r", "/hn_nav_operator_heartbeat:=/test_ultrasonic/operator",
        "-r", "/aligned_goal_active:=/test_ultrasonic/goal_active",
        "-r", "/ultrasonic/sensor_1/range:=/test_ultrasonic/range_1",
        "-r", "/ultrasonic/sensor_2/range:=/test_ultrasonic/range_2",
        "-r", "/nav_motion_status:=/test_ultrasonic/status",
        "-r", "/nav_motion_ready:=/test_ultrasonic/ready",
        "-r", "/nav_motion_armed:=/test_ultrasonic/armed",
        "-r", "/nav_reverse_path_policy:=/test_ultrasonic/path_policy",
        "-r", "/rear_ultrasonic_safety_status:=/test_ultrasonic/safety_status",
        "-r", "/rear_ultrasonic_reverse_allowed:=/test_ultrasonic/reverse_allowed",
        "-r", "/rear_ultrasonic_left_turn_allowed:=/test_ultrasonic/left_turn_allowed",
        "-r", "/rear_ultrasonic_right_turn_allowed:=/test_ultrasonic/right_turn_allowed",
        "-r", "/hn_nav_operator_link_status:=/test_ultrasonic/operator_status",
    ]
    rclpy.init(args=args)
    gate = MODULE.NavMotionSafetyGate()
    driver = TestDriver()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(gate)
    executor.add_node(driver)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(driver.arm_client.service_is_ready, 5.0, "test arm service")
        time.sleep(1.0)
        request = SetBool.Request()
        request.data = True
        future = driver.arm_client.call_async(request)
        wait_until(future.done, 5.0, "test gate arm response")
        if not future.result().success:
            raise RuntimeError("test gate refused arm request")

        driver.command.linear.x = -0.04
        clear_start = time.monotonic()
        wait_until(
            lambda: driver.recent_nonzero(clear_start),
            2.0, "clear reverse command")
        wait_until(
            lambda: driver.reverse_allowed is True
            and driver.left_turn_allowed is True
            and driver.right_turn_allowed is True
            and "detect_m=[0.080,2.000]" in driver.ultrasonic_status
            and "self_echo_m=[0.160,0.220]" in driver.ultrasonic_status,
            2.0, "clear maneuver permissions and detection-range status")

        driver.goal_active = False
        wait_until(
            lambda: "no active navigation goal" in driver.status
            and driver.recent_all_zero(time.monotonic() - 0.20),
            2.0, "residual command blocked without active goal")
        driver.goal_active = True
        wait_until(
            lambda: driver.recent_nonzero(time.monotonic() - 0.30),
            2.0, "active goal lease recovery")

        driver.ranges[1] = 0.19
        wait_until(
            lambda: "rear ultrasonic self-echo sensor_2" in driver.status
            and driver.reverse_allowed is False
            and driver.left_turn_allowed is True
            and driver.right_turn_allowed is False
            and "classification=[NO_ECHO_CLEAR,SELF_ECHO]"
            in driver.ultrasonic_status
            and driver.recent_all_zero(time.monotonic() - 0.20),
            2.0, "sensor_2 self-echo fail-safe stop")
        driver.ranges[1] = math.inf
        wait_until(
            lambda: driver.reverse_allowed is True
            and driver.recent_nonzero(time.monotonic() - 0.30),
            6.0, "self-echo latch release after eight no-echo samples")

        # Once clear, 0.22-0.35 m remains permitted but linearly tapers
        # straight-reverse speed. At 0.285 m the expected factor is 0.5.
        driver.ranges[1] = 0.285
        wait_until(
            lambda: driver.reverse_allowed is True
            and "reverse_speed_scale=0.500" in driver.ultrasonic_status,
            2.0, "compact-range reverse speed scale")
        taper_start = time.monotonic()
        wait_until(
            lambda: driver.recent_linear_peak(taper_start) >= 0.010,
            2.0, "compact-range tapered reverse output")
        if driver.recent_linear_peak(taper_start) > 0.030:
            raise RuntimeError("compact-range reverse was not speed tapered")

        driver.ranges[1] = 0.15
        block_start = time.monotonic()
        wait_until(
            lambda: "rear ultrasonic blocked sensor_2" in driver.status
            and driver.recent_all_zero(time.monotonic() - 0.20),
            2.0, "sensor_2 reverse stop")
        wait_until(
            lambda: "state=PARTIAL_OR_BLOCKED" in driver.ultrasonic_status
            and "blocked=[false,true]" in driver.ultrasonic_status,
            2.0, "independent sensor_2 blocked status")

        driver.ranges[1] = math.inf
        held_start = time.monotonic()
        time.sleep(2.0)
        if driver.recent_nonzero(held_start + 0.20):
            raise RuntimeError(
                "one or a few no-echo samples incorrectly released the latch")
        wait_until(
            lambda: driver.recent_nonzero(time.monotonic() - 0.30),
            3.0, "eight-sample no-echo release")

        driver.operator_present = False
        wait_until(
            lambda: "HN operator/RViz unavailable" in driver.status
            and driver.recent_all_zero(time.monotonic() - 0.20),
            2.0, "operator heartbeat stop")
        driver.operator_present = True
        wait_until(
            lambda: driver.recent_nonzero(time.monotonic() - 0.30),
            2.0, "operator heartbeat recovery")

        driver.command.linear.x = 0.0
        driver.command.angular.z = 0.08
        driver.ranges[0] = 0.15
        wait_until(
            lambda: "rear ultrasonic blocked sensor_1" in driver.status
            and driver.left_turn_allowed is False
            and driver.right_turn_allowed is True
            and driver.recent_all_zero(time.monotonic() - 0.20),
            2.0, "left rear turn-sweep stop")

        # The opposite turn only sweeps the clear right rear corner and must
        # remain available in a narrow aisle.
        driver.command.angular.z = -0.08
        wait_until(
            lambda: driver.recent_nonzero(time.monotonic() - 0.30),
            2.0, "right turn remains available with left rear blocked")

        print(
            "REAR_ULTRASONIC_INTEGRATION_PASS "
            "maneuver_permissions=PASS self_echo=PASS hard_stop=PASS "
            "reverse_taper=PASS latch=PASS "
            "no_echo_release=PASS operator_heartbeat=PASS turn_guard=PASS "
            "goal_lease=PASS "
            "physical_cmd_vel=UNTOUCHED"
        )
        return 0
    finally:
        request = SetBool.Request()
        request.data = False
        if driver.arm_client.service_is_ready():
            driver.arm_client.call_async(request)
            time.sleep(0.20)
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        gate.destroy_node()
        driver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
