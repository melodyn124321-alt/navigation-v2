#!/usr/bin/env python3

import ctypes
import math
import os
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32


I2C_M_RD = 0x0001
I2C_RDWR = 0x0707


class I2cMessage(ctypes.Structure):
    _fields_ = [
        ("addr", ctypes.c_uint16),
        ("flags", ctypes.c_uint16),
        ("len", ctypes.c_uint16),
        ("buf", ctypes.POINTER(ctypes.c_uint8)),
    ]


class I2cRdwrData(ctypes.Structure):
    _fields_ = [
        ("msgs", ctypes.POINTER(I2cMessage)),
        ("nmsgs", ctypes.c_uint32),
    ]


@dataclass
class SensorState:
    name: str
    address: int
    frame_id: str
    publisher: object
    valid_count: int = 0
    no_echo_count: int = 0
    invalid_count: int = 0
    error_count: int = 0
    near_floor_candidate_m: float = math.nan
    near_floor_count: int = 0
    near_floor_latched: bool = False
    near_floor_reject_count: int = 0
    near_floor_last_seen_sec: float = -math.inf


class Ks109Bus:
    def __init__(self, bus: int):
        self.path = f"/dev/i2c-{bus}"
        self.fd = os.open(self.path, os.O_RDWR | os.O_CLOEXEC)
        self.libc = ctypes.CDLL(None, use_errno=True)

    def close(self):
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def _transfer(self, messages):
        message_array = (I2cMessage * len(messages))(*messages)
        data = I2cRdwrData(
            msgs=ctypes.cast(message_array, ctypes.POINTER(I2cMessage)),
            nmsgs=len(messages),
        )
        result = self.libc.ioctl(self.fd, I2C_RDWR, ctypes.byref(data))
        if result < 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number))

    def write(self, address: int, payload: bytes):
        buffer = (ctypes.c_uint8 * len(payload))(*payload)
        message = I2cMessage(
            addr=address,
            flags=0,
            len=len(payload),
            buf=ctypes.cast(buffer, ctypes.POINTER(ctypes.c_uint8)),
        )
        self._transfer([message])

    def read_register(self, address: int, register: int, length: int) -> bytes:
        register_buffer = (ctypes.c_uint8 * 1)(register)
        read_buffer = (ctypes.c_uint8 * length)()
        messages = [
            I2cMessage(
                addr=address,
                flags=0,
                len=1,
                buf=ctypes.cast(
                    register_buffer, ctypes.POINTER(ctypes.c_uint8)
                ),
            ),
            I2cMessage(
                addr=address,
                flags=I2C_M_RD,
                len=length,
                buf=ctypes.cast(read_buffer, ctypes.POINTER(ctypes.c_uint8)),
            ),
        ]
        self._transfer(messages)
        return bytes(read_buffer)

    def read_byte(self, address: int, register: int) -> int:
        return self.read_register(address, register, 1)[0]

    def measure_mm(self, address: int, command: int, delay_s: float) -> int:
        self.write(address, bytes((0x02, command)))
        time.sleep(delay_s)
        result = self.read_register(address, 0x02, 2)
        return (result[0] << 8) | result[1]


class Ks109DualRangeNode(Node):
    def __init__(self):
        super().__init__("ks109_dual_range")

        self.declare_parameter("bus", 7)
        self.declare_parameter("sensor_1_address", 0x74)
        self.declare_parameter("sensor_2_address", 0x75)
        self.declare_parameter("sensor_1_frame", "rear_ultrasonic_left")
        self.declare_parameter("sensor_2_frame", "rear_ultrasonic_right")
        self.declare_parameter("command", 0xB4)
        self.declare_parameter("conversion_delay_s", 0.095)
        self.declare_parameter("minimum_range_m", 0.08)
        self.declare_parameter("maximum_range_m", 11.28)
        self.declare_parameter("field_of_view_rad", 0.30)
        self.declare_parameter("near_floor_reject_enabled", True)
        self.declare_parameter("near_floor_reject_max_m", 0.10)
        self.declare_parameter("near_floor_reject_samples", 12)
        self.declare_parameter("near_floor_reject_tolerance_m", 0.004)
        self.declare_parameter("near_floor_reject_learning_window_s", 8.0)
        self.declare_parameter("near_floor_reject_sensor_indices", [1])

        bus_number = int(self.get_parameter("bus").value)
        self.command = int(self.get_parameter("command").value)
        self.conversion_delay_s = float(
            self.get_parameter("conversion_delay_s").value
        )
        self.minimum_range_m = float(
            self.get_parameter("minimum_range_m").value
        )
        self.maximum_range_m = float(
            self.get_parameter("maximum_range_m").value
        )
        self.field_of_view_rad = float(
            self.get_parameter("field_of_view_rad").value
        )
        self.near_floor_reject_enabled = bool(
            self.get_parameter("near_floor_reject_enabled").value
        )
        self.near_floor_reject_max_m = float(
            self.get_parameter("near_floor_reject_max_m").value
        )
        self.near_floor_reject_samples = max(
            2, int(self.get_parameter("near_floor_reject_samples").value)
        )
        self.near_floor_reject_tolerance_m = float(
            self.get_parameter("near_floor_reject_tolerance_m").value
        )
        self.near_floor_reject_learning_window_s = max(
            1.0,
            float(self.get_parameter(
                "near_floor_reject_learning_window_s").value),
        )
        self.near_floor_reject_sensor_indices = {
            int(value) for value in self.get_parameter(
                "near_floor_reject_sensor_indices").value
        }
        self.near_floor_learning_deadline_sec = (
            time.monotonic() + self.near_floor_reject_learning_window_s
        )

        self.bus = Ks109Bus(bus_number)
        self.minimum_publisher = self.create_publisher(
            Float32, "/ultrasonic/min_range", 10
        )
        self.health_publisher = self.create_publisher(
            Bool, "/ultrasonic/healthy", 10
        )

        self.sensors = []
        for index in (1, 2):
            address = int(
                self.get_parameter(f"sensor_{index}_address").value
            )
            frame_id = str(
                self.get_parameter(f"sensor_{index}_frame").value
            )
            publisher = self.create_publisher(
                Range, f"/ultrasonic/sensor_{index}/range", 10
            )
            self.sensors.append(
                SensorState(
                    name=f"sensor_{index}",
                    address=address,
                    frame_id=frame_id,
                    publisher=publisher,
                )
            )

        self._validate_devices()
        cycle_time = max(
            0.01,
            (2.0 * self.conversion_delay_s) + 0.005,
        )
        self.timer = self.create_timer(cycle_time, self._sample_cycle)
        self.last_status_time = self.get_clock().now()
        self.get_logger().info(
            "KS109 dual ranging started: "
            f"bus={bus_number}, addresses="
            f"{','.join(hex(sensor.address) for sensor in self.sensors)}, "
            f"sequential_rate~{1.0 / cycle_time:.2f} Hz/sensor, "
            f"near_floor_learning={self.near_floor_reject_learning_window_s:.1f}s "
            f"sensors={sorted(self.near_floor_reject_sensor_indices)}"
        )

    def _validate_devices(self):
        failures = []
        for sensor in self.sensors:
            try:
                model = self.bus.read_byte(sensor.address, 0x09)
            except OSError as error:
                failures.append(
                    f"{sensor.name}@{hex(sensor.address)}: {error}"
                )
                continue
            if model != 0x6D:
                failures.append(
                    f"{sensor.name}@{hex(sensor.address)}: "
                    f"model=0x{model:02x}, expected=0x6d"
                )
            else:
                self.get_logger().info(
                    f"{sensor.name}@{hex(sensor.address)} model=0x{model:02x}"
                )
                try:
                    millimetres = self.bus.measure_mm(
                        sensor.address,
                        self.command,
                        self.conversion_delay_s,
                    )
                except OSError as error:
                    failures.append(
                        f"{sensor.name}@{hex(sensor.address)} "
                        f"startup ranging failed: {error}"
                    )
                    continue
                if millimetres == 0:
                    failures.append(
                        f"{sensor.name}@{hex(sensor.address)} "
                        "startup ranging returned zero; check VCC/GND "
                        "and power-cycle the sensor"
                    )
        if failures:
            self.bus.close()
            raise RuntimeError(
                "KS109 validation failed: " + "; ".join(failures)
            )

    def _make_range(self, sensor: SensorState, distance_m: float) -> Range:
        message = Range()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = sensor.frame_id
        message.radiation_type = Range.ULTRASOUND
        message.field_of_view = self.field_of_view_rad
        message.min_range = self.minimum_range_m
        message.max_range = self.maximum_range_m
        message.range = distance_m
        return message

    def _reject_persistent_near_floor(
        self, sensor: SensorState, distance_m: float
    ) -> float:
        if not self.near_floor_reject_enabled or not math.isfinite(distance_m):
            return distance_m

        sensor_index = self.sensors.index(sensor) + 1
        if sensor_index not in self.near_floor_reject_sensor_indices:
            return distance_m

        if sensor.near_floor_latched:
            # A learned installation echo is permanent for this process. The
            # echo is intermittent, so ordinary clear returns must not erase
            # the learned floor and cause the next 89 mm sample to hard-stop.
            if distance_m <= self.near_floor_reject_max_m:
                sensor.near_floor_reject_count += 1
                sensor.near_floor_last_seen_sec = time.monotonic()
                return math.inf
            return distance_m

        if time.monotonic() > self.near_floor_learning_deadline_sec:
            return distance_m

        if distance_m > self.near_floor_reject_max_m:
            # Keep the candidate through intermittent clear returns during the
            # bounded startup learning window; only inconsistent near-floor
            # returns replace it.
            return distance_m

        if (
            math.isfinite(sensor.near_floor_candidate_m)
            and abs(distance_m - sensor.near_floor_candidate_m)
            <= self.near_floor_reject_tolerance_m
        ):
            sensor.near_floor_count += 1
        else:
            sensor.near_floor_candidate_m = distance_m
            sensor.near_floor_count = 1

        if sensor.near_floor_count < self.near_floor_reject_samples:
            return distance_m

        sensor.near_floor_latched = True
        sensor.near_floor_reject_count += 1
        sensor.near_floor_last_seen_sec = time.monotonic()
        self.get_logger().warn(
            f"{sensor.name} learned persistent installation echo "
            f"at {sensor.near_floor_candidate_m:.3f}m; reserved "
            f"<={self.near_floor_reject_max_m:.3f}m band will publish +inf"
        )
        return math.inf

    def _sample_cycle(self):
        finite_ranges = []
        healthy = True

        for sensor in self.sensors:
            try:
                millimetres = self.bus.measure_mm(
                    sensor.address,
                    self.command,
                    self.conversion_delay_s,
                )
                distance_m = millimetres / 1000.0
                if millimetres == 0:
                    sensor.invalid_count += 1
                    healthy = False
                    distance_m = math.nan
                elif millimetres > int(self.maximum_range_m * 1000):
                    sensor.no_echo_count += 1
                    distance_m = math.inf
                elif millimetres < int(self.minimum_range_m * 1000):
                    sensor.no_echo_count += 1
                    distance_m = -math.inf
                else:
                    sensor.valid_count += 1
                    distance_m = self._reject_persistent_near_floor(
                        sensor, distance_m
                    )
                    if math.isfinite(distance_m):
                        finite_ranges.append(distance_m)
            except OSError as error:
                sensor.error_count += 1
                healthy = False
                distance_m = math.nan
                if sensor.error_count <= 3:
                    self.get_logger().error(
                        f"{sensor.name}@{hex(sensor.address)} I2C error: "
                        f"{error}"
                    )
            sensor.publisher.publish(self._make_range(sensor, distance_m))

        minimum_message = Float32()
        minimum_message.data = (
            min(finite_ranges) if finite_ranges else math.inf
        )
        self.minimum_publisher.publish(minimum_message)

        health_message = Bool()
        health_message.data = healthy
        self.health_publisher.publish(health_message)

        now = self.get_clock().now()
        if (now - self.last_status_time).nanoseconds >= 5_000_000_000:
            status = " ".join(
                f"{sensor.name}[ok={sensor.valid_count},"
                f"no_echo={sensor.no_echo_count},"
                f"invalid={sensor.invalid_count},err={sensor.error_count},"
                f"near_floor_rejected={sensor.near_floor_reject_count}]"
                for sensor in self.sensors
            )
            self.get_logger().info(status)
            self.last_status_time = now

    def destroy_node(self):
        self.bus.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = Ks109DualRangeNode()
        rclpy.spin(node)
    except (OSError, RuntimeError) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f"KS109 startup failed: {error}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
