#!/usr/bin/env python3

import math
import time as wall_time
from typing import List

import rclpy
from rclpy.node import Node

from ocs2_msgs.msg import MpcTargets
from ocs2_msgs.msg import MpcObservation, MpcState, MpcTargetTrajectories
from std_msgs.msg import Float64MultiArray

DEFAULT_TARGET_TOPIC = "/mpc_targets"
DEFAULT_OBSERVATION_TOPIC = "/forward_dynamics_mpc_observation" # "/inverse_dynamics_mpc_observation" | "/forward_dynamics_mpc_observation"

DEFAULT_JOINT_NAMES = [
    "ur_arm_shoulder_pan_joint",
    "ur_arm_shoulder_lift_joint",
    "ur_arm_elbow_joint",
    "ur_arm_wrist_1_joint",
    "ur_arm_wrist_2_joint",
    "ur_arm_wrist_3_joint",
]

# DEFAULT_CENTER = [0.0, -1.5708, 1.5708, 3.1416, -1.5708, 0.0]
# DEFAULT_AMPLITUDE = [0.1, 0.20, 0.20, 0.25, 0.20, 0.25]
DEFAULT_CENTER = [1.0, -1.0, 1.0, 3.1416, -1.5708, 0.0]
DEFAULT_AMPLITUDE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# DEFAULT_CENTER = [0.0, -1.5708, 1.5708, 3.1416, -1.5708, 0.0]
# DEFAULT_AMPLITUDE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

DEFAULT_PHASE = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5]
DEFAULT_WEIGHTS = [20.0, 20.0, 20.0, 20.0, 20.0, 20.0]


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
        self.declare_parameter("wait_for_observation", True)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("trajectory_duration", 2.0)
        self.declare_parameter("trajectory_dt", 0.02)
        self.declare_parameter("sine_frequency", 0.5)
        self.declare_parameter("time_offset", 0.0)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("center", DEFAULT_CENTER)
        self.declare_parameter("amplitude", DEFAULT_AMPLITUDE)
        self.declare_parameter("phase", DEFAULT_PHASE)
        self.declare_parameter("weights", DEFAULT_WEIGHTS)

        self.topic = self.get_parameter("topic").value
        self.observation_topic = self.get_parameter("observation_topic").value
        self.wait_for_observation = bool(self.get_parameter("wait_for_observation").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.trajectory_duration = float(self.get_parameter("trajectory_duration").value)
        self.trajectory_dt = float(self.get_parameter("trajectory_dt").value)
        self.sine_frequency = float(self.get_parameter("sine_frequency").value)
        self.time_offset = float(self.get_parameter("time_offset").value)
        self.joint_names = _as_list(self.get_parameter("joint_names").value, DEFAULT_JOINT_NAMES)

        joint_count = len(self.joint_names)
        self.center = _resize(_as_list(self.get_parameter("center").value, DEFAULT_CENTER), joint_count, 0.0)
        self.amplitude = _resize(_as_list(self.get_parameter("amplitude").value, DEFAULT_AMPLITUDE), joint_count, 0.0)
        self.phase = _resize(_as_list(self.get_parameter("phase").value, DEFAULT_PHASE), joint_count, 0.0)
        self.weights = _resize(_as_list(self.get_parameter("weights").value, DEFAULT_WEIGHTS), joint_count, 1.0)

        self.trajectory_duration = max(1e-6, self.trajectory_duration)
        self.trajectory_dt = max(1e-6, self.trajectory_dt)
        self.trajectory_samples = int(math.floor(self.trajectory_duration / self.trajectory_dt)) + 1
        self.publish_rate = max(1e-6, self.publish_rate)
        self.start_time = self.get_clock().now()
        self.latest_observation_time = None
        self.last_wait_log_time = 0.0

        self.publisher = self.create_publisher(MpcTargets, self.topic, 1)
        self.observation_subscription = self.create_subscription(
            MpcObservation,
            self.observation_topic,
            self.observation_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)

        self.get_logger().info(
            f"Publishing joint MpcTargets to {self.topic} with {joint_count} joints, "
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

    def joint_target(self, t: float) -> List[float]:
        omega = 2.0 * math.pi * self.sine_frequency
        return [
            self.center[i] + self.amplitude[i] * math.sin(omega * t + self.phase[i])
            for i in range(len(self.joint_names))
        ]

    def build_trajectory(self, t0: float) -> MpcTargetTrajectories:
        trajectory = MpcTargetTrajectories()
        zoh_target = self.joint_target(t0)

        for sample_index in range(self.trajectory_samples):
            t = t0 + sample_index * self.trajectory_dt
            state = MpcState()
            state.value = [float(q) for q in zoh_target]

            trajectory.time_trajectory.append(float(t))
            trajectory.state_trajectory.append(state)
        return trajectory

    def publish(self):
        t0 = self.current_target_time()
        if t0 is None:
            return

        msg = MpcTargets()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = "joint"
        msg.joint_names = list(self.joint_names)
        msg.joint_tracking_weights = Float64MultiArray()
        msg.joint_tracking_weights.data = [float(w) for w in self.weights]
        msg.target_trajectories = [self.build_trajectory(t0)]

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
