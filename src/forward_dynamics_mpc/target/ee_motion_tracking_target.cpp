#include "dynamics_mpc_controller/forward_dynamics_mpc/target/ee_motion_tracking_target.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>

#include "dynamics_mpc_controller/common/target/target_encoding.hpp"
#include "dynamics_mpc_controller/forward_dynamics_mpc/forward_dynamics_mpc_interface.hpp"

namespace dynamics_mpc_controller
{
namespace target
{
namespace
{

ocs2::vector_t vectorFromMessage(const ocs2_msgs::msg::MpcInput& msg)
{
  ocs2::vector_t out(static_cast<Eigen::Index>(msg.value.size()));
  for (std::size_t i = 0; i < msg.value.size(); ++i) {
    out(static_cast<Eigen::Index>(i)) = msg.value[i];
  }
  return out;
}

ocs2::vector_t vectorFromMessage(const ocs2_msgs::msg::MpcState& msg)
{
  ocs2::vector_t out(static_cast<Eigen::Index>(msg.value.size()));
  for (std::size_t i = 0; i < msg.value.size(); ++i) {
    out(static_cast<Eigen::Index>(i)) = msg.value[i];
  }
  return out;
}

ocs2::vector_t weightVectorFromMessage(
  const std_msgs::msg::Float64MultiArray& msg,
  const std::string& name)
{
  ocs2::vector_t weights = ocs2::vector_t::Zero(6);
  if (msg.data.size() == 1) {
    weights.setConstant(msg.data.front());
  } else if (msg.data.size() == 6) {
    for (std::size_t i = 0; i < 6; ++i) {
      weights(static_cast<Eigen::Index>(i)) = msg.data[i];
    }
  } else {
    throw std::runtime_error(name + " size must be 1 or 6");
  }
  if ((weights.array() < 0.0).any()) {
    throw std::runtime_error(name + " entries must be non-negative");
  }
  return weights;
}

template<typename Trajectory>
void validateTrajectorySize(
  const Trajectory& trajectory,
  std::size_t expectedSize,
  const std::string& name)
{
  if (!trajectory.empty() && trajectory.size() != expectedSize) {
    throw std::runtime_error(
      name + " size must match time/state trajectory size when provided");
  }
}

enum class EeMotionCommandMode
{
  Pose,
  Twist,
  PoseTwist,
};

bool supportsCommandType(const std::string& commandType)
{
  return commandType == "ee_motion_pose" ||
         commandType == "ee_motion_twist" ||
         commandType == "ee_motion";
}

EeMotionCommandMode commandModeFromMessage(const std::string& commandType)
{
  if (commandType == "ee_motion_pose") {
    return EeMotionCommandMode::Pose;
  }
  if (commandType == "ee_motion_twist") {
    return EeMotionCommandMode::Twist;
  }
  if (commandType == "ee_motion") {
    return EeMotionCommandMode::PoseTwist;
  }
  throw std::invalid_argument(
    "unsupported dynamics MPC command_type: '" +
    commandType + "'");
}

double twistFrameFromMessage(const std::string& frame, const std::string& defaultFrame)
{
  const std::string& selected_frame = frame.empty() ? defaultFrame : frame;
  if (selected_frame == "base" || selected_frame == "global" || selected_frame == "world") {
    return target_encoding::kEeMotionTwistFrameBase;
  }
  if (selected_frame == "ee" || selected_frame == "end_effector" ||
      selected_frame == "end_effector_frame") {
    return target_encoding::kEeMotionTwistFrameEe;
  }
  throw std::invalid_argument(
    "ee_motion_twist_frame must be empty, 'base', or 'ee', got '" + selected_frame + "'");
}

ocs2::vector_t makeEeMotionTargetState(
  const ocs2::vector_t& rawState,
  EeMotionCommandMode mode,
  double twistFrame,
  const ocs2::vector_t* poseWeights,
  const ocs2::vector_t* twistWeights)
{
  if (mode == EeMotionCommandMode::Pose) {
    if (rawState.size() != target_encoding::kEeMotionPoseTargetDim) {
      throw std::runtime_error(
        "ee_motion_pose target state dimension must be 7: [px, py, pz, qx, qy, qz, qw]");
    }
    if (poseWeights != nullptr) {
      ocs2::vector_t target =
        ocs2::vector_t::Zero(target_encoding::kEeMotionWeightedTargetDim);
      target.head(target_encoding::kEeMotionPoseTargetDim) = rawState;
      target.segment(target_encoding::kEeMotionTargetDim, 6) = *poseWeights;
      return target;
    }
    return rawState;
  }
  if (mode == EeMotionCommandMode::Twist) {
    if (rawState.size() != 6) {
      throw std::runtime_error(
        "ee_motion_twist target state dimension must be 6: [vx, vy, vz, wx, wy, wz]");
    }
    if (twistWeights != nullptr) {
      ocs2::vector_t target =
        ocs2::vector_t::Zero(target_encoding::kEeMotionWeightedTargetWithTwistFrameDim);
      target(target_encoding::kEeMotionWeightedTargetDim) = twistFrame;
      target.segment(target_encoding::kEeMotionPoseTargetDim, 6) = rawState;
      target.segment(target_encoding::kEeMotionTargetDim + 6, 6) = *twistWeights;
      return target;
    }
    ocs2::vector_t target =
      ocs2::vector_t::Zero(target_encoding::kEeMotionTwistOnlyTargetDim);
    target(0) = twistFrame;
    target.segment(1, 6) = rawState;
    return target;
  }
  if (rawState.size() != target_encoding::kEeMotionTargetDim) {
    throw std::runtime_error(
      "ee_motion target state dimension must be 13: [px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz]");
  }
  if (poseWeights != nullptr || twistWeights != nullptr) {
    ocs2::vector_t target =
      ocs2::vector_t::Zero(target_encoding::kEeMotionWeightedTargetWithTwistFrameDim);
    target(target_encoding::kEeMotionWeightedTargetDim) = twistFrame;
    target.head(target_encoding::kEeMotionTargetDim) = rawState;
    if (poseWeights != nullptr) {
      target.segment(target_encoding::kEeMotionTargetDim, 6) = *poseWeights;
    }
    if (twistWeights != nullptr) {
      target.segment(target_encoding::kEeMotionTargetDim + 6, 6) = *twistWeights;
    }
    return target;
  }
  ocs2::vector_t target =
    ocs2::vector_t::Zero(target_encoding::kEeMotionTargetWithTwistFrameDim);
  target.head(target_encoding::kEeMotionTargetDim) = rawState;
  target(target_encoding::kEeMotionTargetDim) = twistFrame;
  return target;
}

std::vector<std::size_t> makeReorderIndices(
  const std::vector<std::string>& incomingNames,
  const std::vector<std::string>& modelNames)
{
  const std::size_t n = modelNames.size();
  if (incomingNames.size() != n) {
    throw std::runtime_error(
      "joint_names size must match the MPC joint dimension when joint torque references are provided");
  }

  std::vector<std::size_t> reorder_indices(n);
  std::unordered_map<std::string, std::size_t> incoming;
  for (std::size_t i = 0; i < incomingNames.size(); ++i) {
    if (!incoming.emplace(incomingNames[i], i).second) {
      throw std::runtime_error("duplicate joint target name: " + incomingNames[i]);
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    const auto it = incoming.find(modelNames[i]);
    if (it == incoming.end()) {
      throw std::runtime_error("missing joint target for " + modelNames[i]);
    }
    reorder_indices[i] = it->second;
  }
  return reorder_indices;
}

}  // namespace

ForwardEeMotionTrackingTarget::ForwardEeMotionTrackingTarget(
  const ForwardDynamicsMpcInterface& interface)
: interface_(interface)
{
}

bool ForwardEeMotionTrackingTarget::supports(const TargetMsg& msg) const noexcept
{
  return supportsCommandType(msg.command_type);
}

ForwardEeMotionTrackingTarget::TargetTrajectories ForwardEeMotionTrackingTarget::fromMessage(
  const TargetMsg& msg) const
{
  if (!supports(msg)) {
    throw std::invalid_argument(
      "unsupported dynamics MPC command_type: '" +
      msg.command_type + "'");
  }

  const auto& model = interface_.getForwardDynamicsMpcModel();
  const EeMotionCommandMode command_mode = commandModeFromMessage(msg.command_type);
  const double twist_frame = twistFrameFromMessage(
    msg.ee_motion_twist_frame, interface_.defaultEeMotionTwistFrame());
  const std::size_t n = model.jointDim();
  if (msg.time_trajectory.empty() || msg.state_trajectory.empty()) {
    throw std::runtime_error("dynamics MPC target trajectory is empty");
  }
  if (msg.time_trajectory.size() != msg.state_trajectory.size()) {
    throw std::runtime_error("dynamics MPC target time and state trajectory sizes do not match");
  }
  validateTrajectorySize(
    msg.joint_acceleration_trajectory, msg.time_trajectory.size(),
    "joint_acceleration_trajectory");
  validateTrajectorySize(
    msg.joint_torque_trajectory, msg.time_trajectory.size(),
    "joint_torque_trajectory");
  validateTrajectorySize(
    msg.ee_wrench_trajectory, msg.time_trajectory.size(),
    "ee_wrench_trajectory");

  const bool has_pose_weights = !msg.ee_motion_pose_tracking_weights.data.empty();
  const bool has_twist_weights = !msg.ee_motion_twist_tracking_weights.data.empty();
  if (command_mode == EeMotionCommandMode::Pose && has_twist_weights) {
    throw std::runtime_error(
      "ee_motion_twist_tracking_weights cannot be used with ee_motion_pose targets");
  }
  if (command_mode == EeMotionCommandMode::Twist && has_pose_weights) {
    throw std::runtime_error(
      "ee_motion_pose_tracking_weights cannot be used with ee_motion_twist targets");
  }
  if (command_mode == EeMotionCommandMode::PoseTwist &&
      has_pose_weights != has_twist_weights) {
    throw std::runtime_error(
      "ee_motion_pose_tracking_weights and ee_motion_twist_tracking_weights must be provided together");
  }
  const ocs2::vector_t pose_weights = has_pose_weights ?
    weightVectorFromMessage(
      msg.ee_motion_pose_tracking_weights, "ee_motion_pose_tracking_weights") :
    ocs2::vector_t();
  const ocs2::vector_t twist_weights = has_twist_weights ?
    weightVectorFromMessage(
      msg.ee_motion_twist_tracking_weights, "ee_motion_twist_tracking_weights") :
    ocs2::vector_t();

  std::vector<std::size_t> reorder_indices;
  if (!msg.joint_torque_trajectory.empty()) {
    reorder_indices = makeReorderIndices(msg.joint_names, model.dofNames());
  }

  ocs2::vector_array_t state_trajectory;
  ocs2::vector_array_t input_trajectory;
  state_trajectory.reserve(msg.state_trajectory.size());
  input_trajectory.reserve(msg.state_trajectory.size());

  for (std::size_t sample = 0; sample < msg.state_trajectory.size(); ++sample) {
    state_trajectory.push_back(
      makeEeMotionTargetState(
        vectorFromMessage(msg.state_trajectory[sample]),
        command_mode,
        twist_frame,
        (has_pose_weights &&
         command_mode != EeMotionCommandMode::Twist) ? &pose_weights : nullptr,
        (has_twist_weights &&
         command_mode != EeMotionCommandMode::Pose) ? &twist_weights : nullptr));

    ocs2::vector_t input =
      ocs2::vector_t::Zero(static_cast<Eigen::Index>(model.inputDim()));
    if (!msg.joint_torque_trajectory.empty()) {
      const ocs2::vector_t raw_torque =
        vectorFromMessage(msg.joint_torque_trajectory[sample]);
      if (raw_torque.size() != static_cast<Eigen::Index>(n)) {
        throw std::runtime_error("joint_torque_trajectory sample dimension must match joint dimension");
      }
      for (std::size_t i = 0; i < n; ++i) {
        input(static_cast<Eigen::Index>(model.tauOffset() + i)) =
          raw_torque(static_cast<Eigen::Index>(reorder_indices[i]));
      }
    }
    input_trajectory.push_back(std::move(input));
  }

  return TargetTrajectories(msg.time_trajectory, state_trajectory, input_trajectory);
}

}  // namespace target
}  // namespace dynamics_mpc_controller
