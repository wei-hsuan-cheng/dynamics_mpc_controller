import math
import numpy as np
from typing import Sequence

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from ocs2_msgs.msg import DynamicsMpcTargets
from ocs2_msgs.msg import MpcInput, MpcState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

DEFAULT_TARGET_TOPIC = "/mpc_targets"
DEFAULT_COMMAND_TYPE = "ee_motion_pose"  # "ee_motion_pose" | "ee_motion_twist" | "ee_motion"
DEFAULT_TWIST_FRAME = "ee"  # "base" | "ee"

DEFAULT_TRANSLATION_CENTER = np.array([0.573, 0.11, 0.51])
DEFAULT_TRANSLATION_AMPLITUDE = np.array([0.0, 0.0, 0.0])
# DEFAULT_TRANSLATION_CENTER = np.array([0.573, 0.11, 0.1])
# DEFAULT_TRANSLATION_AMPLITUDE = np.array([0.0, 0.0, 0.1])
# DEFAULT_TRANSLATION_CENTER = np.array([0.45, 0.0, 0.2])
# DEFAULT_TRANSLATION_AMPLITUDE = np.array([0.0, 0.0, 0.0])
DEFAULT_TRANSLATION_PHASE = np.array([0.0, 0.0, 0.0])

DEFAULT_ORIENTATION_RPY_CENTER = np.array([-np.pi / 2.0, np.pi / 4.0, -np.pi / 2.0])
# DEFAULT_ORIENTATION_RPY_CENTER = np.array([-np.pi / 1.0, 0.0, -np.pi / 2.0])
DEFAULT_ORIENTATION_RPY_AMPLITUDE = np.array([0.0, 0.0, 0.0])
DEFAULT_ORIENTATION_RPY_PHASE = np.array([0.0, 0.0, 0.0])

DEFAULT_TWIST_LINEAR_CENTER = np.array([0.0, 0.0, 0.0])
DEFAULT_TWIST_LINEAR_AMPLITUDE = np.array([0.0, 0.0, 0.0])
DEFAULT_TWIST_LINEAR_PHASE = np.array([0.0, 0.0, 0.0])

DEFAULT_TWIST_ANGULAR_CENTER = np.array([0.0, 0.0, 0.0])
DEFAULT_TWIST_ANGULAR_AMPLITUDE = np.array([0.0, 0.0, 0.0])
DEFAULT_TWIST_ANGULAR_PHASE = np.array([0.0, 0.0, 0.0])

DEFAULT_POSE_WEIGHTS = np.array([2.0, 2.0, 2.0, 0.5, 0.5, 0.5]) * 1000.0
DEFAULT_TWIST_WEIGHTS = np.array([2.0, 2.0, 2.0, 0.5, 0.5, 0.5]) * 1000.0

DEFAULT_EE_WRENCH_FRAME = "ee"  # empty uses controller YAML default; otherwise "world" | "base" | "ee"
DEFAULT_EE_WRENCH_CENTER = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
DEFAULT_EE_WRENCH_AMPLITUDE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
DEFAULT_EE_WRENCH_PHASE = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


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
    return [
        center[i] + amplitude[i] * math.sin(omega * t + phase[i])
        for i in range(len(center))
    ]


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


class EeMotionTrackingTargetPublisher(Node):
    def __init__(self):
        super().__init__("ee_motion_tracking_target_publisher")

        self.declare_parameter("topic", DEFAULT_TARGET_TOPIC)
        self.declare_parameter("command_type", DEFAULT_COMMAND_TYPE)
        self.declare_parameter("twist_frame", DEFAULT_TWIST_FRAME)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("trajectory_duration", 2.0)
        self.declare_parameter("trajectory_dt", 0.02)
        self.declare_parameter("sine_frequency", 0.05)
        self.declare_parameter("time_offset", 0.0)
        self.declare_parameter("tf_parent_frame", "world")
        self.declare_parameter("tf_child_frame", "ee_motion_command")

        self.declare_parameter("translation_center", DEFAULT_TRANSLATION_CENTER.tolist())
        self.declare_parameter("translation_amplitude", DEFAULT_TRANSLATION_AMPLITUDE.tolist())
        self.declare_parameter("translation_phase", DEFAULT_TRANSLATION_PHASE.tolist())
        self.declare_parameter("orientation_rpy_center", DEFAULT_ORIENTATION_RPY_CENTER.tolist())
        self.declare_parameter("orientation_rpy_amplitude", DEFAULT_ORIENTATION_RPY_AMPLITUDE.tolist())
        self.declare_parameter("orientation_rpy_phase", DEFAULT_ORIENTATION_RPY_PHASE.tolist())

        self.declare_parameter("twist_linear_center", DEFAULT_TWIST_LINEAR_CENTER.tolist())
        self.declare_parameter("twist_linear_amplitude", DEFAULT_TWIST_LINEAR_AMPLITUDE.tolist())
        self.declare_parameter("twist_linear_phase", DEFAULT_TWIST_LINEAR_PHASE.tolist())
        self.declare_parameter("twist_angular_center", DEFAULT_TWIST_ANGULAR_CENTER.tolist())
        self.declare_parameter("twist_angular_amplitude", DEFAULT_TWIST_ANGULAR_AMPLITUDE.tolist())
        self.declare_parameter("twist_angular_phase", DEFAULT_TWIST_ANGULAR_PHASE.tolist())
        self.declare_parameter("pose_weights", DEFAULT_POSE_WEIGHTS.tolist())
        self.declare_parameter("twist_weights", DEFAULT_TWIST_WEIGHTS.tolist())
        self.declare_parameter("ee_wrench_frame", DEFAULT_EE_WRENCH_FRAME)
        self.declare_parameter("ee_wrench_center", DEFAULT_EE_WRENCH_CENTER.tolist())
        self.declare_parameter("ee_wrench_amplitude", DEFAULT_EE_WRENCH_AMPLITUDE.tolist())
        self.declare_parameter("ee_wrench_phase", DEFAULT_EE_WRENCH_PHASE.tolist())

        self.topic = self.get_parameter("topic").value
        self.command_type = str(self.get_parameter("command_type").value)
        if self.command_type not in ("ee_motion_pose", "ee_motion_twist", "ee_motion"):
            raise RuntimeError(
                "command_type must be 'ee_motion_pose', 'ee_motion_twist', or 'ee_motion'"
            )
        self.twist_frame = str(self.get_parameter("twist_frame").value).lower()
        if self.twist_frame not in ("", "base", "ee"):
            raise RuntimeError("twist_frame must be empty, 'base', or 'ee'")
        self.publish_rate = max(1e-6, float(self.get_parameter("publish_rate").value))
        self.trajectory_duration = max(1e-6, float(self.get_parameter("trajectory_duration").value))
        self.trajectory_dt = max(1e-6, float(self.get_parameter("trajectory_dt").value))
        self.sine_frequency = float(self.get_parameter("sine_frequency").value)
        self.time_offset = float(self.get_parameter("time_offset").value)
        self.tf_parent_frame = self.get_parameter("tf_parent_frame").value
        self.tf_child_frame = self.get_parameter("tf_child_frame").value

        self.translation_center = _resize(
            _as_array(self.get_parameter("translation_center").value, DEFAULT_TRANSLATION_CENTER), 3, 0.0)
        self.translation_amplitude = _resize(
            _as_array(self.get_parameter("translation_amplitude").value, DEFAULT_TRANSLATION_AMPLITUDE), 3, 0.0)
        self.translation_phase = _resize(
            _as_array(self.get_parameter("translation_phase").value, DEFAULT_TRANSLATION_PHASE), 3, 0.0)
        self.orientation_rpy_center = _resize(
            _as_array(self.get_parameter("orientation_rpy_center").value, DEFAULT_ORIENTATION_RPY_CENTER), 3, 0.0)
        self.orientation_rpy_amplitude = _resize(
            _as_array(self.get_parameter("orientation_rpy_amplitude").value, DEFAULT_ORIENTATION_RPY_AMPLITUDE), 3, 0.0)
        self.orientation_rpy_phase = _resize(
            _as_array(self.get_parameter("orientation_rpy_phase").value, DEFAULT_ORIENTATION_RPY_PHASE), 3, 0.0)

        self.twist_linear_center = _resize(
            _as_array(self.get_parameter("twist_linear_center").value, DEFAULT_TWIST_LINEAR_CENTER), 3, 0.0)
        self.twist_linear_amplitude = _resize(
            _as_array(self.get_parameter("twist_linear_amplitude").value, DEFAULT_TWIST_LINEAR_AMPLITUDE), 3, 0.0)
        self.twist_linear_phase = _resize(
            _as_array(self.get_parameter("twist_linear_phase").value, DEFAULT_TWIST_LINEAR_PHASE), 3, 0.0)
        self.twist_angular_center = _resize(
            _as_array(self.get_parameter("twist_angular_center").value, DEFAULT_TWIST_ANGULAR_CENTER), 3, 0.0)
        self.twist_angular_amplitude = _resize(
            _as_array(self.get_parameter("twist_angular_amplitude").value, DEFAULT_TWIST_ANGULAR_AMPLITUDE), 3, 0.0)
        self.twist_angular_phase = _resize(
            _as_array(self.get_parameter("twist_angular_phase").value, DEFAULT_TWIST_ANGULAR_PHASE), 3, 0.0)
        self.pose_weights = _resize(
            _as_array(self.get_parameter("pose_weights").value, DEFAULT_POSE_WEIGHTS), 6, 20.0)
        self.twist_weights = _resize(
            _as_array(self.get_parameter("twist_weights").value, DEFAULT_TWIST_WEIGHTS), 6, 2.0)
        self.ee_wrench_frame = str(self.get_parameter("ee_wrench_frame").value).lower()
        if self.ee_wrench_frame not in ("", "base", "global", "world", "ee", "end_effector", "end_effector_frame"):
            raise RuntimeError(
                "ee_wrench_frame must be empty, 'world'/'base', or 'ee'"
            )
        self.ee_wrench_center = _resize(
            _as_array(self.get_parameter("ee_wrench_center").value, DEFAULT_EE_WRENCH_CENTER), 6, 0.0)
        self.ee_wrench_amplitude = _resize(
            _as_array(self.get_parameter("ee_wrench_amplitude").value, DEFAULT_EE_WRENCH_AMPLITUDE), 6, 0.0)
        self.ee_wrench_phase = _resize(
            _as_array(self.get_parameter("ee_wrench_phase").value, DEFAULT_EE_WRENCH_PHASE), 6, 0.0)

        self.trajectory_samples = int(math.floor(self.trajectory_duration / self.trajectory_dt)) + 1

        self.publisher = self.create_publisher(DynamicsMpcTargets, self.topic, 1)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish)

        self.get_logger().info(
            f"Publishing {self.command_type} DynamicsMpcTargets to {self.topic}, "
            f"twist_frame={self.twist_frame}, "
            f"ee_wrench_frame={self.ee_wrench_frame}, "
            f"{self.trajectory_samples} ZOH trajectory samples over {self.trajectory_duration:.3f} s "
            f"with dt {self.trajectory_dt:.3f} s, using ROS node time"
        )

    def pose_target(self, t: float):
        translation = _sample_wave(
            self.translation_center,
            self.translation_amplitude,
            self.translation_phase,
            self.sine_frequency,
            t,
        )
        rpy = _sample_wave(
            self.orientation_rpy_center,
            self.orientation_rpy_amplitude,
            self.orientation_rpy_phase,
            self.sine_frequency,
            t,
        )
        return translation + _quaternion_from_rpy(rpy[0], rpy[1], rpy[2])

    def publish_command_tf(self, pose):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.tf_parent_frame
        transform.child_frame_id = self.tf_child_frame
        transform.transform.translation.x = float(pose[0])
        transform.transform.translation.y = float(pose[1])
        transform.transform.translation.z = float(pose[2])
        transform.transform.rotation.x = float(pose[3])
        transform.transform.rotation.y = float(pose[4])
        transform.transform.rotation.z = float(pose[5])
        transform.transform.rotation.w = float(pose[6])
        self.tf_broadcaster.sendTransform(transform)

    def twist_target(self, t: float):
        linear_twist = _sample_wave(
            self.twist_linear_center,
            self.twist_linear_amplitude,
            self.twist_linear_phase,
            self.sine_frequency,
            t,
        )
        angular_twist = _sample_wave(
            self.twist_angular_center,
            self.twist_angular_amplitude,
            self.twist_angular_phase,
            self.sine_frequency,
            t,
        )
        return linear_twist + angular_twist

    def ee_wrench_target(self, t: float):
        return _sample_wave(
            self.ee_wrench_center,
            self.ee_wrench_amplitude,
            self.ee_wrench_phase,
            self.sine_frequency,
            t,
        )

    def fill_trajectory(self, msg: DynamicsMpcTargets, t0: float):
        zoh_pose = self.pose_target(t0)
        zoh_twist = self.twist_target(t0)
        zoh_wrench = self.ee_wrench_target(t0)

        for sample_index in range(self.trajectory_samples):
            t = t0 + sample_index * self.trajectory_dt
            state = MpcState()
            if self.command_type == "ee_motion_pose":
                state.value = [float(x) for x in zoh_pose]
            elif self.command_type == "ee_motion_twist":
                state.value = [float(x) for x in zoh_twist]
            else:
                state.value = [float(x) for x in zoh_pose + zoh_twist]

            msg.time_trajectory.append(float(t))
            msg.state_trajectory.append(state)

            wrench = MpcInput()
            wrench.value = [float(x) for x in zoh_wrench]
            msg.ee_wrench_trajectory.append(wrench)

    def publish(self):
        t0 = self.get_clock().now().nanoseconds / 1e9 + self.time_offset

        msg = DynamicsMpcTargets()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_type = self.command_type
        msg.ee_motion_twist_frame = self.twist_frame
        msg.ee_wrench_frame = self.ee_wrench_frame
        
        if self.command_type in ("ee_motion_pose", "ee_motion"):
            msg.ee_motion_pose_tracking_weights = Float64MultiArray()
            msg.ee_motion_pose_tracking_weights.data = [float(w) for w in self.pose_weights]
        if self.command_type in ("ee_motion_twist", "ee_motion"):
            msg.ee_motion_twist_tracking_weights = Float64MultiArray()
            msg.ee_motion_twist_tracking_weights.data = [float(w) for w in self.twist_weights]
        self.fill_trajectory(msg, t0)

        self.publisher.publish(msg)
        if self.command_type in ("ee_motion_pose", "ee_motion"):
            self.publish_command_tf(self.pose_target(t0))


def main(args=None):
    rclpy.init(args=args)
    node = EeMotionTrackingTargetPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
