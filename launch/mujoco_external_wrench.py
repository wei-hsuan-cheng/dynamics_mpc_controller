#!/usr/bin/env python3

import math
import time as wall_time
from typing import Sequence

import numpy as np

import rclpy
from mujoco_ros2_control.msg import MujocoExternalWrench
from rclpy.node import Node

DEFAULT_TOPIC = "/mujoco_ros2_control/external_wrench"
DEFAULT_BODY_NAME = "ur_arm_tool0"
DEFAULT_WRENCH_FRAME = "global" # "global" | "local"

DEFAULT_FORCE_CENTER = np.array([0.0, 0.0, 0.0])
DEFAULT_FORCE_AMPLITUDE = np.array([0.0, 0.0, 20.0])
DEFAULT_FORCE_PHASE = np.array([0.0, 0.0, 0.0])

DEFAULT_TORQUE_CENTER = np.array([0.0, 0.0, 0.0])
DEFAULT_TORQUE_AMPLITUDE = np.array([1.0, 0.0, 0.0])
DEFAULT_TORQUE_PHASE = np.array([0.0, 0.0, 0.0])

DEFAULT_WRENCH_FREQUENCY = 0.25 # [Hz]

def _as_array(value, fallback: Sequence[float]) -> np.ndarray:
    if value is None:
        return np.asarray(fallback, dtype=float).copy()
    return np.asarray(value, dtype=float).reshape(-1)


def _resize(values: Sequence[float], size: int, fill: float) -> np.ndarray:
    values = np.asarray(values, dtype=float).reshape(-1)
    if values.size >= size:
        return values[:size].copy()
    return np.concatenate((values, np.full(size - values.size, fill, dtype=float)))


def _sample_wave(center: np.ndarray, amplitude: np.ndarray, phase: np.ndarray, frequency: float, t: float):
    omega = 2.0 * math.pi * frequency
    return np.array([
        center[i] + amplitude[i] * math.sin(omega * t + phase[i])
        for i in range(len(center))
    ])


class MujocoExternalWrenchPublisher(Node):
    def __init__(self):
        super().__init__("mujoco_external_wrench_publisher")

        self.declare_parameter("topic", DEFAULT_TOPIC)
        self.declare_parameter("body_name", DEFAULT_BODY_NAME)
        self.declare_parameter("wrench_frame", DEFAULT_WRENCH_FRAME)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("sine_frequency", DEFAULT_WRENCH_FREQUENCY)
        self.declare_parameter("time_offset", 0.0)
        self.declare_parameter("duration", 0.0)
        self.declare_parameter("start_delay", 0.0)
        self.declare_parameter("zero_on_stop", True)
        self.declare_parameter("stamp_with_node_time", False)
        self.declare_parameter("force_center", DEFAULT_FORCE_CENTER.tolist())
        self.declare_parameter("force_amplitude", DEFAULT_FORCE_AMPLITUDE.tolist())
        self.declare_parameter("force_phase", DEFAULT_FORCE_PHASE.tolist())
        self.declare_parameter("torque_center", DEFAULT_TORQUE_CENTER.tolist())
        self.declare_parameter("torque_amplitude", DEFAULT_TORQUE_AMPLITUDE.tolist())
        self.declare_parameter("torque_phase", DEFAULT_TORQUE_PHASE.tolist())

        self.topic = str(self.get_parameter("topic").value)
        self.body_name = str(self.get_parameter("body_name").value)
        self.wrench_frame = str(self.get_parameter("wrench_frame").value)
        self.publish_rate = max(1e-6, float(self.get_parameter("publish_rate").value))
        self.sine_frequency = float(self.get_parameter("sine_frequency").value)
        self.time_offset = float(self.get_parameter("time_offset").value)
        self.duration = max(0.0, float(self.get_parameter("duration").value))
        self.start_delay = max(0.0, float(self.get_parameter("start_delay").value))
        self.zero_on_stop = bool(self.get_parameter("zero_on_stop").value)
        self.stamp_with_node_time = bool(self.get_parameter("stamp_with_node_time").value)
        self.force_center = _resize(
            _as_array(self.get_parameter("force_center").value, DEFAULT_FORCE_CENTER),
            3,
            0.0,
        )
        self.force_amplitude = _resize(
            _as_array(self.get_parameter("force_amplitude").value, DEFAULT_FORCE_AMPLITUDE),
            3,
            0.0,
        )
        self.force_phase = _resize(
            _as_array(self.get_parameter("force_phase").value, DEFAULT_FORCE_PHASE),
            3,
            0.0,
        )
        self.torque_center = _resize(
            _as_array(self.get_parameter("torque_center").value, DEFAULT_TORQUE_CENTER),
            3,
            0.0,
        )
        self.torque_amplitude = _resize(
            _as_array(self.get_parameter("torque_amplitude").value, DEFAULT_TORQUE_AMPLITUDE),
            3,
            0.0,
        )
        self.torque_phase = _resize(
            _as_array(self.get_parameter("torque_phase").value, DEFAULT_TORQUE_PHASE),
            3,
            0.0,
        )

        self.start_wall_time = wall_time.monotonic()
        self.zero_sent = False

        self.publisher = self.create_publisher(MujocoExternalWrench, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)

        duration_text = "continuous" if self.duration == 0.0 else f"{self.duration:.3f} s"
        self.get_logger().info(
            f"Publishing MuJoCo external wrench command to {self.topic} at "
            f"{self.publish_rate:.3f} Hz | body={self.body_name}, "
            f"wrench_frame={self.wrench_frame}, "
            f"sine_frequency={self.sine_frequency:.3f} Hz, "
            f"force_center={self.force_center.tolist()}, "
            f"force_amplitude={self.force_amplitude.tolist()}, "
            f"force_phase={self.force_phase.tolist()}, "
            f"torque_center={self.torque_center.tolist()}, "
            f"torque_amplitude={self.torque_amplitude.tolist()}, "
            f"torque_phase={self.torque_phase.tolist()}, "
            f"duration={duration_text}, start_delay={self.start_delay:.3f} s, "
            f"stamp_with_node_time={self.stamp_with_node_time}"
        )

    def elapsed_wall_time(self) -> float:
        return wall_time.monotonic() - self.start_wall_time

    def make_message(self, force: np.ndarray, torque: np.ndarray) -> MujocoExternalWrench:
        msg = MujocoExternalWrench()
        if self.stamp_with_node_time:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.wrench_frame
        msg.body_name = self.body_name
        msg.wrench_frame = self.wrench_frame
        msg.wrench.force.x = float(force[0])
        msg.wrench.force.y = float(force[1])
        msg.wrench.force.z = float(force[2])
        msg.wrench.torque.x = float(torque[0])
        msg.wrench.torque.y = float(torque[1])
        msg.wrench.torque.z = float(torque[2])
        return msg

    def publish_zero_once(self):
        if self.zero_sent:
            return
        self.publisher.publish(self.make_message(np.zeros(3), np.zeros(3)))
        self.zero_sent = True
        self.get_logger().info("Published zero MuJoCo external wrench command.")

    def publish(self):
        elapsed = self.elapsed_wall_time()
        if elapsed < self.start_delay:
            return

        active_elapsed = elapsed - self.start_delay
        if self.duration > 0.0 and active_elapsed > self.duration:
            if self.zero_on_stop:
                self.publish_zero_once()
            return

        t = active_elapsed + self.time_offset
        force = _sample_wave(
            self.force_center,
            self.force_amplitude,
            self.force_phase,
            self.sine_frequency,
            t,
        )
        torque = _sample_wave(
            self.torque_center,
            self.torque_amplitude,
            self.torque_phase,
            self.sine_frequency,
            t,
        )
        self.publisher.publish(self.make_message(force, torque))


def main(args=None):
    rclpy.init(args=args)
    node = MujocoExternalWrenchPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.zero_on_stop:
            node.publish_zero_once()
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
