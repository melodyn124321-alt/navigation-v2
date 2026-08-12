#!/usr/bin/env python3
"""Check all warm-path navigation prerequisites with one DDS participant."""

import argparse
import math
import re
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, OccupancyGrid
from ranger_msgs.msg import SystemState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Range
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool


LIFECYCLE_NODES = (
    "map_server",
    "planner_server",
    "controller_server",
    "smoother_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
    "collision_monitor",
)


class ArmReadinessCheck(Node):
    def __init__(self, max_fitness, recovery_max_fitness, sonar_stop_distance):
        super().__init__("nav_arm_readiness_check")
        self.max_fitness = max_fitness
        self.recovery_max_fitness = recovery_max_fitness
        self.sonar_stop_distance = sonar_stop_distance
        self.healthy = None
        self.left_range = None
        self.right_range = None
        self.reverse_allowed = None
        self.left_turn_allowed = None
        self.right_turn_allowed = None
        self.sonar_status = None
        self.obstacle_status = None
        self.goal_bridge_status = None
        self.operator_link_status = None
        self.fitness = None
        self.system_error = None
        self.vehicle_state = None
        self.control_mode = None
        self.ultrasonic_receive_times = {
            "healthy": None,
            "left_range": None,
            "right_range": None,
            "reverse_allowed": None,
            "left_turn_allowed": None,
            "right_turn_allowed": None,
            "status": None,
        }

        sensor_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Bool, "/ultrasonic/healthy", self.on_healthy, sensor_qos)
        self.create_subscription(
            Range, "/ultrasonic/sensor_1/range", self.on_left, sensor_qos)
        self.create_subscription(
            Range, "/ultrasonic/sensor_2/range", self.on_right, sensor_qos)
        self.create_subscription(
            Bool, "/rear_ultrasonic_reverse_allowed",
            self.on_reverse_allowed, sensor_qos)
        self.create_subscription(
            Bool, "/rear_ultrasonic_left_turn_allowed",
            self.on_left_turn_allowed, sensor_qos)
        self.create_subscription(
            Bool, "/rear_ultrasonic_right_turn_allowed",
            self.on_right_turn_allowed, sensor_qos)
        self.create_subscription(
            String, "/rear_ultrasonic_safety_status",
            self.on_sonar_status, sensor_qos)
        self.create_subscription(
            String, "/nav_obstacle_cloud_status",
            self.on_obstacle_status, sensor_qos)
        self.create_subscription(
            String, "/rviz_goal_pose_bridge_status",
            self.on_goal_bridge_status, sensor_qos)
        self.create_subscription(
            String, "/hn_nav_operator_link_status",
            self.on_operator_link_status, sensor_qos)
        self.create_subscription(
            Odometry, "/relocalization_odom",
            self.on_localization, sensor_qos)
        self.create_subscription(
            SystemState, "/system_state", self.on_system, sensor_qos)

        # Retain typed subscriptions so graph validation covers the exact
        # runtime types expected by the safety chain.
        self.endpoint_subscriptions = (
            self.create_subscription(
                PointCloud2, "/nav_obstacle_cloud", lambda _msg: None,
                sensor_qos),
            self.create_subscription(
                OccupancyGrid, "/local_costmap/costmap", lambda _msg: None, 1),
            self.create_subscription(
                OccupancyGrid, "/global_costmap/costmap", lambda _msg: None, 1),
            self.create_subscription(
                String, "/nav_obstacle_alarm_status", lambda _msg: None, 1),
        )
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in LIFECYCLE_NODES
        }
        self.arm_client = self.create_client(
            SetBool, "/set_nav_motion_enabled")

    def on_healthy(self, msg):
        self.healthy = bool(msg.data)
        self.ultrasonic_receive_times["healthy"] = time.monotonic()

    def on_left(self, msg):
        self.left_range = float(msg.range)
        self.ultrasonic_receive_times["left_range"] = time.monotonic()

    def on_right(self, msg):
        self.right_range = float(msg.range)
        self.ultrasonic_receive_times["right_range"] = time.monotonic()

    def on_reverse_allowed(self, msg):
        self.reverse_allowed = bool(msg.data)
        self.ultrasonic_receive_times["reverse_allowed"] = time.monotonic()

    def on_left_turn_allowed(self, msg):
        self.left_turn_allowed = bool(msg.data)
        self.ultrasonic_receive_times["left_turn_allowed"] = time.monotonic()

    def on_right_turn_allowed(self, msg):
        self.right_turn_allowed = bool(msg.data)
        self.ultrasonic_receive_times["right_turn_allowed"] = time.monotonic()

    def on_sonar_status(self, msg):
        self.sonar_status = msg.data
        self.ultrasonic_receive_times["status"] = time.monotonic()

    def on_obstacle_status(self, msg):
        self.obstacle_status = msg.data

    def on_goal_bridge_status(self, msg):
        self.goal_bridge_status = msg.data

    def on_operator_link_status(self, msg):
        self.operator_link_status = msg.data

    def on_localization(self, msg):
        self.fitness = float(msg.pose.covariance[0])

    def on_system(self, msg):
        self.system_error = int(msg.error_code)
        self.vehicle_state = int(msg.vehicle_state)
        self.control_mode = int(msg.control_mode)

    def graph_ready(self):
        node_names = {
            f"{namespace.rstrip('/')}/{name}".replace("//", "/")
            for name, namespace in self.get_node_names_and_namespaces()
        }
        if "/nav_motion_safety_gate" not in node_names:
            return False
        if "/nav_obstacle_block_alarm" not in node_names:
            return False
        if "/rviz_goal_pose_bridge" not in node_names:
            return False
        required_publishers = (
            "/nav_obstacle_cloud",
            "/local_costmap/costmap",
            "/global_costmap/costmap",
            "/nav_obstacle_alarm_status",
            "/cmd_vel_nav_collision_safe",
            "/nav_reverse_path_policy",
            "/nav_speed_profile_status",
            "/rear_ultrasonic_safety_status",
            "/rear_ultrasonic_reverse_allowed",
            "/rear_ultrasonic_left_turn_allowed",
            "/rear_ultrasonic_right_turn_allowed",
            "/rviz_goal_pose_bridge_status",
            "/hn_nav_operator_link_status",
        )
        return all(self.count_publishers(topic) >= 1
                   for topic in required_publishers)

    def sensor_ready(self):
        if self.healthy is not True:
            return False
        # The permission topics must be live, but their values are not warm
        # readiness conditions. The motion gate evaluates them for every
        # reverse or turning command, so a nearby rear obstacle may limit a
        # maneuver without disabling collision-checked forward navigation.
        if any(value is None for value in (
                self.reverse_allowed,
                self.left_turn_allowed,
                self.right_turn_allowed)):
            return False
        now = time.monotonic()
        if any(
                stamp is None or now - stamp > 1.0
                for stamp in self.ultrasonic_receive_times.values()):
            return False
        if not self.sonar_status or "healthy=true" not in self.sonar_status:
            return False
        if "fresh=[true,true]" not in self.sonar_status:
            return False
        if self.left_range is None or self.right_range is None:
            return False
        for value in (self.left_range, self.right_range):
            if not (
                (math.isinf(value) and value > 0.0)
                or (
                    math.isfinite(value)
                    and value >= 0.0
                )
            ):
                return False
        recovery_max_fitness = getattr(
            self, "recovery_max_fitness", self.max_fitness)
        if self.fitness is None or self.fitness > recovery_max_fitness:
            return False
        if self.system_error != 0:
            return False
        if self.vehicle_state != int(SystemState.VEHICLE_STATE_NORMAL):
            return False
        if self.control_mode != int(SystemState.CONTROL_MODE_CAN):
            return False
        if not self.obstacle_status:
            return False
        if not (
            self.goal_bridge_status
            and "READY action_server=true" in self.goal_bridge_status
            and "active=false" in self.goal_bridge_status
        ):
            return False
        if not (
            self.operator_link_status
            and "state=READY" in self.operator_link_status
        ):
            return False
        match = re.search(r"age=([0-9.]+)s", self.obstacle_status)
        return (
            self.obstacle_status.startswith("ok received=")
            and match is not None
            and float(match.group(1)) <= 1.0
        )

    def wait_ready(self, timeout):
        started = time.monotonic()
        deadline = time.monotonic() + timeout
        next_report = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            services_ready = all(
                client.service_is_ready()
                for client in self.lifecycle_clients.values())
            if (
                services_ready
                and self.arm_client.service_is_ready()
                and self.graph_ready()
                and self.sensor_ready()
            ):
                break
            now = time.monotonic()
            if now >= next_report:
                print(
                    "WAITING warm readiness: "
                    f"services={services_ready} graph={self.graph_ready()} "
                    f"ultrasonic={self.healthy} fitness={self.fitness} "
                    f"reverse_allowed={self.reverse_allowed} "
                    f"left_turn_allowed={self.left_turn_allowed} "
                    f"right_turn_allowed={self.right_turn_allowed} "
                    f"sonar_status={self.sonar_status} "
                    f"ranger_error={self.system_error} "
                    f"ranger_vehicle={self.vehicle_state} "
                    f"ranger_control_mode={self.control_mode} "
                    f"goal_bridge={self.goal_bridge_status} "
                    f"operator_link={self.operator_link_status}",
                    flush=True,
                )
                next_report = now + 2.0
        else:
            missing = []
            if self.count_publishers("/system_state") == 0:
                missing.append("/system_state publisher discovery")
            if not all(
                    client.service_is_ready()
                    for client in self.lifecycle_clients.values()):
                missing.append("Nav2 lifecycle services")
            if not self.arm_client.service_is_ready():
                missing.append("/set_nav_motion_enabled service")
            if not self.graph_ready():
                missing.append("navigation graph endpoints")
            if not self.sensor_ready():
                missing.append("live safety samples")
            detail = ", ".join(missing) if missing else "unknown prerequisite"
            raise TimeoutError(
                "warm navigation prerequisites did not converge: " + detail
                + ". This may be delayed Fast DDS discovery; CAN is declared "
                "dead only by the startup CAN_RX_LIVE/CAN_RX_DEAD preflight")

        futures = {
            name: client.call_async(GetState.Request())
            for name, client in self.lifecycle_clients.items()
        }
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(future.done() for future in futures.values()):
                break
        else:
            pending = [name for name, future in futures.items()
                       if not future.done()]
            raise TimeoutError(
                "lifecycle state responses timed out: " + ", ".join(pending))

        inactive = []
        for name, future in futures.items():
            response = future.result()
            state_id = int(response.current_state.id)
            label = response.current_state.label
            print(f"/{name}: {label} [{state_id}]", flush=True)
            if state_id != State.PRIMARY_STATE_ACTIVE:
                inactive.append(name)
        if inactive:
            raise RuntimeError(
                "lifecycle nodes not active: " + ", ".join(inactive))

        print(
            "WARM_READINESS PASS "
            f"fitness={self.fitness:.6f} forward_navigation=true "
            f"localization_mode="
            f"{'FULL' if self.fitness <= self.max_fitness else 'BOUNDED_RECOVERY'} "
            f"ranger_error={self.system_error} "
            f"ranger_vehicle={self.vehicle_state} "
            f"ranger_control_mode=CAN({self.control_mode}) "
            f"sonar=({self.left_range:.3f},{self.right_range:.3f}) "
            f"maneuvers=(reverse={self.reverse_allowed},"
            f"left={self.left_turn_allowed},right={self.right_turn_allowed}) "
            f"obstacle='{self.obstacle_status}' "
            f"goal_bridge='{self.goal_bridge_status}'",
            flush=True,
        )
        if not all((
                self.reverse_allowed,
                self.left_turn_allowed,
                self.right_turn_allowed)):
            print(
                "MANEUVER_LIMIT "
                f"reverse={self.reverse_allowed} "
                f"left={self.left_turn_allowed} "
                f"right={self.right_turn_allowed}; "
                "the motion gate will block only the affected commands",
                flush=True,
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--max-fitness", type=float, default=0.10)
    parser.add_argument("--recovery-max-fitness", type=float, default=0.32)
    parser.add_argument("--sonar-stop-distance", type=float, default=0.22)
    args = parser.parse_args()

    rclpy.init()
    node = ArmReadinessCheck(
        args.max_fitness, max(args.max_fitness, args.recovery_max_fitness),
        max(0.01, args.sonar_stop_distance))
    try:
        node.wait_ready(max(2.0, args.timeout))
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
