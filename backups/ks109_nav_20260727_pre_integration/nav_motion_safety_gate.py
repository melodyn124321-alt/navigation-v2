#!/usr/bin/env python3
"""Last safety gate between Nav2 and the Ranger base.

The gate starts disarmed and continually publishes a zero command until all
localization and chassis checks are healthy and an operator explicitly arms it.
"""

import math
import re

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ranger_msgs.msg import SystemState
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


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

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.ready_pub = self.create_publisher(Bool, "/nav_motion_ready", 1)
        self.status_pub = self.create_publisher(String, "/nav_motion_status", 1)
        self.create_subscription(Twist, self.input_topic, self.on_command, 10)
        self.create_subscription(Odometry, "/relocalization_odom", self.on_localization, sensor_qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, sensor_qos)
        self.create_subscription(SystemState, "/system_state", self.on_system_state, 10)
        self.create_subscription(
            String, "/nav_obstacle_cloud_status", self.on_obstacle_status, 10)
        self.create_service(SetBool, self.arm_service, self.on_set_arm)

        self.last_command = None
        self.last_command_time = None
        self.last_localization_time = None
        self.last_fitness = math.inf
        self.last_odom_time = None
        self.last_system_time = None
        self.last_obstacle_status_time = None
        self.last_obstacle_age = math.inf
        self.system_error = None
        self.control_mode = None
        self.armed = False
        self.last_status = "starting"
        # Ten output updates per second are sufficient for the 0.08 m/s limit
        # and leave CPU time for Fast-LIO, NDT, costmaps, and the controller.
        self.create_timer(0.10, self.tick)
        self.create_timer(1.0, self.publish_status_heartbeat)
        self.get_logger().info(
            "motion gate starts DISARMED: "
            f"fitness<={self.max_fitness:.3f}, "
            f"localization<={self.localization_timeout:.2f}s, "
            f"cmd<={self.command_timeout:.2f}s, "
            f"obstacle_age<={self.max_obstacle_age:.2f}s, "
            f"limits=({self.max_linear:.2f} m/s, {self.max_angular:.2f} rad/s), "
            f"no-spin linear>={self.minimum_linear_for_turn:.3f} m/s, "
            f"curvature<={self.max_motion_curvature:.2f} 1/m")

    def on_command(self, msg):
        self.last_command = msg
        self.last_command_time = self.get_clock().now()

    def on_localization(self, msg):
        self.last_localization_time = self.get_clock().now()
        self.last_fitness = float(msg.pose.covariance[0])

    def on_odom(self, _msg):
        self.last_odom_time = self.get_clock().now()

    def on_system_state(self, msg):
        self.last_system_time = self.get_clock().now()
        self.system_error = int(msg.error_code)
        self.control_mode = int(msg.control_mode)

    def on_obstacle_status(self, msg):
        match = re.search(r"age=([0-9.]+)s", msg.data)
        self.last_obstacle_status_time = self.get_clock().now()
        self.last_obstacle_age = float(match.group(1)) if match else math.inf

    def on_set_arm(self, request, response):
        requested = bool(request.data)
        if requested != self.armed:
            self.armed = requested
            self.get_logger().warn(
                f"motion gate {'ARMED' if self.armed else 'DISARMED'}")
        response.success = True
        response.message = "armed" if self.armed else "disarmed"
        return response

    def elapsed(self, stamp):
        if stamp is None:
            return math.inf
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def reason(self):
        if not self.armed:
            return "disarmed"
        if self.elapsed(self.last_system_time) > self.chassis_timeout:
            return "stale chassis state"
        if self.system_error != 0:
            return f"chassis error_code={self.system_error}"
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
        return "ready"

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
        self.ready_pub.publish(Bool(data=reason == "ready"))
        self.status_pub.publish(String(data=reason))

    def tick(self):
        reason = self.reason()
        self.publish_status(reason)
        if reason != "ready" or self.last_command is None:
            self.pub.publish(Twist())
            return
        out = Twist()
        out.linear.x = max(-self.max_linear, min(self.max_linear, self.last_command.linear.x))
        out.angular.z = max(-self.max_angular, min(self.max_angular, self.last_command.angular.z))
        if abs(out.linear.x) < self.minimum_linear_for_turn:
            out.angular.z = 0.0
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
