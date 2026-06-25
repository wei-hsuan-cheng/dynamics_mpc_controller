#!/usr/bin/env python3

import math
import time as wall_time
from typing import List

import rclpy
from rclpy.node import Node

from ocs2_msgs.msg import DynamicsMpcTargets
from ocs2_msgs.msg import MpcObservation, MpcState
from std_msgs.msg import Float64MultiArray

DEFAULT_TARGET_TOPIC = "/mpc_targets"
DEFAULT_OBSERVATION_TOPIC = "/mpc_observation"
DEFAULT_COMMAND_TYPE = "joint_position"  # "joint_position" | "joint_velocity" | "joint"

DEFAULT_JOINT_NAMES = [
    "ur_arm_shoulder_pan_joint",
    "ur_arm_shoulder_lift_joint",
    "ur_arm_elbow_joint",
    "ur_arm_wrist_1_joint",
    "ur_arm_wrist_2_joint",
    "ur_arm_wrist_3_joint",
]

DEFAULT_POSITION_CENTER = [0.0, -1.5708, 1.5708, 3.1416, -1.5708, 0.0]
DEFAULT_POSITION_AMPLITUDE = [0.4, 0.4, 0.4, 0.4, 0.4, 0.4]
# DEFAULT_POSITION_CENTER = [1.0, -0.0, 1.0, 3.1416, -1.5708, 0.0]
# DEFAULT_POSITION_AMPLITUDE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_POSITION_PHASE = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5]

DEFAULT_VELOCITY_CENTER = [0.5, 0.2, 0.0, 0.0, 0.0, 0.0]
DEFAULT_VELOCITY_AMPLITUDE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
DEFAULT_VELOCITY_PHASE = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5]

DEFAULT_POSITION_WEIGHTS = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]
DEFAULT_VELOCITY_WEIGHTS = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]


def _as_list(value, fallback: List):
    if value is None:
        return list(fallback)
    return list(value)


def _resize(values: List[float], size: int, fill: float) -> List[float]:
    values = list(values)
    if len(values) >= size:
        return values[:size]
    return values + [fill] * (size - len(values))


class JointTrackingTargetPublisher(Node):
    def __init__(self):
        super().__init__("joint_tracking_target_publisher")

        self.declare_parameter("topic", DEFAULT_TARGET_TOPIC)
        self.declare_parameter("observation_topic", DEFAULT_OBSERVATION_TOPIC)
        self.declare_parameter("command_type", DEFAULT_COMMAND_TYPE)
        self.declare_parameter("wait_for_observation", True)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("trajectory_duration", 2.0)
        self.declare_parameter("trajectory_dt", 0.02)
        self.declare_parameter("sine_frequency", 0.5)
        self.declare_parameter("time_offset", 0.0)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("position_center", DEFAULT_POSITION_CENTER)
        self.declare_parameter("position_amplitude", DEFAULT_POSITION_AMPLITUDE)
        self.declare_parameter("position_phase", DEFAULT_POSITION_PHASE)
        self.declare_parameter("velocity_center", DEFAULT_VELOCITY_CENTER)
        self.declare_parameter("velocity_amplitude", DEFAULT_VELOCITY_AMPLITUDE)
        self.declare_parameter("velocity_phase", DEFAULT_VELOCITY_PHASE)
        self.declare_parameter("position_weights", DEFAULT_POSITION_WEIGHTS)
        self.declare_parameter("velocity_weights", DEFAULT_VELOCITY_WEIGHTS)

        self.topic = self.get_parameter("topic").value
        self.observation_topic = self.get_parameter("observation_topic").value
        self.command_type = str(self.get_parameter("command_type").value)
        if self.command_type not in ("joint_position", "joint_velocity", "joint"):
            raise RuntimeError(
                "command_type must be 'joint_position', 'joint_velocity', or 'joint'"
            )
        self.wait_for_observation = bool(self.get_parameter("wait_for_observation").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.trajectory_duration = float(self.get_parameter("trajectory_duration").value)
        self.trajectory_dt = float(self.get_parameter("trajectory_dt").value)
        self.sine_frequency = float(self.get_parameter("sine_frequency").value)
        self.time_offset = float(self.get_parameter("time_offset").value)
        self.joint_names = _as_list(self.get_parameter("joint_names").value, DEFAULT_JOINT_NAMES)

        joint_count = len(self.joint_names)
        self.position_center = _resize(
            _as_list(self.get_parameter("position_center").value, DEFAULT_POSITION_CENTER),
            joint_count,
            0.0,
        )
        self.position_amplitude = _resize(
            _as_list(self.get_parameter("position_amplitude").value, DEFAULT_POSITION_AMPLITUDE),
            joint_count,
            0.0,
        )
        self.position_phase = _resize(
            _as_list(self.get_parameter("position_phase").value, DEFAULT_POSITION_PHASE),
            joint_count,
            0.0,
        )
        self.velocity_center = _resize(
            _as_list(self.get_parameter("velocity_center").value, DEFAULT_VELOCITY_CENTER),
            joint_count,
            0.0,
        )
        self.velocity_amplitude = _resize(
            _as_list(self.get_parameter("velocity_amplitude").value, DEFAULT_VELOCITY_AMPLITUDE),
            joint_count,
            0.0,
        )
        self.velocity_phase = _resize(
            _as_list(self.get_parameter("velocity_phase").value, DEFAULT_VELOCITY_PHASE),
            joint_count,
            0.0,
        )
        self.position_weights = _resize(
            _as_list(self.get_parameter("position_weights").value, DEFAULT_POSITION_WEIGHTS),
            joint_count,
            20.0,
        )
        self.velocity_weights = _resize(
            _as_list(self.get_parameter("velocity_weights").value, DEFAULT_VELOCITY_WEIGHTS),
            joint_count,
            2.0,
        )

        self.trajectory_duration = max(1e-6, self.trajectory_duration)
        self.trajectory_dt = max(1e-6, self.trajectory_dt)
        self.trajectory_samples = int(math.floor(self.trajectory_duration / self.trajectory_dt)) + 1
        self.publish_rate = max(1e-6, self.publish_rate)
        self.start_time = self.get_clock().now()
        self.latest_observation_time = None
        self.last_wait_log_time = 0.0

        self.publisher = self.create_publisher(DynamicsMpcTargets, self.topic, 1)
        self.observation_subscription = self.create_subscription(
            MpcObservation,
            self.observation_topic,
            self.observation_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)

        self.get_logger().info(
            f"Publishing {self.command_type} DynamicsMpcTargets to {self.topic} with {joint_count} joints, "
            f"{self.trajectory_samples} ZOH trajectory samples over {self.trajectory_duration:.3f} s "
            f"with dt {self.trajectory_dt:.3f} s, using OCS2 time from "
            f"{self.observation_topic}"
        )

    def elapsed_time(self) -> float:
        now = self.get_clock().now()
        return (now - self.start_time).nanoseconds * 1e-9

    def observation_callback(self, msg: MpcObservation):
        self.latest_observation_time = float(msg.time)

    def current_target_time(self):
        if self.latest_observation_time is not None:
            return self.latest_observation_time + self.time_offset
        if not self.wait_for_observation:
            return self.elapsed_time() + self.time_offset

        now = wall_time.monotonic()
        if now - self.last_wait_log_time > 2.0:
            self.get_logger().warn(
                f"Waiting for OCS2 observation on {self.observation_topic} before publishing targets."
            )
            self.last_wait_log_time = now
        return None

    def joint_target(self, t: float):
        omega = 2.0 * math.pi * self.sine_frequency
        position = [
            self.position_center[i] +
            self.position_amplitude[i] * math.sin(omega * t + self.position_phase[i])
            for i in range(len(self.joint_names))
        ]
        velocity = [
            self.velocity_center[i] +
            self.velocity_amplitude[i] * math.sin(omega * t + self.velocity_phase[i])
            for i in range(len(self.joint_names))
        ]
        return position, velocity

    def fill_trajectory(self, msg: DynamicsMpcTargets, t0: float):
        zoh_position, zoh_velocity = self.joint_target(t0)

        for sample_index in range(self.trajectory_samples):
            t = t0 + sample_index * self.trajectory_dt
            state = MpcState()
            if self.command_type == "joint_position":
                state.value = [float(q) for q in zoh_position]
            elif self.command_type == "joint_velocity":
                state.value = [float(v) for v in zoh_velocity]
            else:
                state.value = (
                    [float(q) for q in zoh_position] +
                    [float(v) for v in zoh_velocity]
                )

            msg.time_trajectory.append(float(t))
            msg.state_trajectory.append(state)

    def publish(self):
        t0 = self.current_target_time()
        if t0 is None:
            return

        msg = DynamicsMpcTargets()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = self.command_type
        msg.joint_names = list(self.joint_names)
        if self.command_type in ("joint_position", "joint"):
            msg.joint_position_tracking_weights = Float64MultiArray()
            msg.joint_position_tracking_weights.data = [float(w) for w in self.position_weights]
        if self.command_type in ("joint_velocity", "joint"):
            msg.joint_velocity_tracking_weights = Float64MultiArray()
            msg.joint_velocity_tracking_weights.data = [float(w) for w in self.velocity_weights]
        self.fill_trajectory(msg, t0)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointTrackingTargetPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
