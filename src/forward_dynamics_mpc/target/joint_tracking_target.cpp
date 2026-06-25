#include "dynamics_mpc_controller/forward_dynamics_mpc/target/joint_tracking_target.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <std_msgs/msg/float64_multi_array.hpp>

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
  const std::vector<std::size_t>& reorderIndices,
  const std::string& name)
{
  const std::size_t n = reorderIndices.size();
  ocs2::vector_t weights = ocs2::vector_t::Zero(static_cast<Eigen::Index>(n));
  if (msg.data.size() == 1) {
    weights.setConstant(msg.data.front());
  } else if (msg.data.size() == n) {
    for (std::size_t i = 0; i < n; ++i) {
      weights(static_cast<Eigen::Index>(i)) =
        msg.data[reorderIndices[i]];
    }
  } else {
    throw std::runtime_error(name + " size must be 1 or match joint_names size");
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

ocs2::vector_t makeTargetState(
  const ocs2::vector_t& state,
  const ocs2::vector_t* positionWeights,
  const ocs2::vector_t* velocityWeights)
{
  if (positionWeights == nullptr || velocityWeights == nullptr) {
    return state;
  }

  const Eigen::Index state_dim = state.size();
  const Eigen::Index n = state_dim / 2;
  ocs2::vector_t weighted_state = ocs2::vector_t::Zero(2 * state_dim);
  weighted_state.head(state_dim) = state;
  weighted_state.segment(state_dim, n) = *positionWeights;
  weighted_state.segment(state_dim + n, n) = *velocityWeights;
  return weighted_state;
}

}  // namespace

ForwardJointTrackingTarget::ForwardJointTrackingTarget(const ForwardDynamicsMpcInterface& interface)
: interface_(interface)
{
}

bool ForwardJointTrackingTarget::supports(const TargetMsg& msg) const noexcept
{
  return msg.command_type == "joint";
}

ForwardJointTrackingTarget::TargetTrajectories ForwardJointTrackingTarget::fromObservation(
  const ocs2::SystemObservation& observation) const
{
  const auto& model = interface_.getForwardDynamicsMpcModel();
  ocs2::vector_t desired_state = observation.state;
  desired_state.tail(static_cast<Eigen::Index>(model.jointDim())).setZero();
  const ocs2::vector_t desired_input =
    interface_.computeNonlinearEffects(
      model.getQ(desired_state),
      model.getV(desired_state));

  return TargetTrajectories(
    {observation.time},
    {std::move(desired_state)},
    {desired_input});
}

ForwardJointTrackingTarget::TargetTrajectories ForwardJointTrackingTarget::fromMessage(
  const TargetMsg& msg) const
{
  if (!supports(msg)) {
    throw std::invalid_argument(
      "joint tracking target requires command_type='joint', got '" + msg.command_type + "'");
  }
  const auto& model = interface_.getForwardDynamicsMpcModel();
  const std::size_t n = model.jointDim();
  if (msg.time_trajectory.empty() || msg.state_trajectory.empty()) {
    throw std::runtime_error("joint target trajectory is empty");
  }
  if (msg.time_trajectory.size() != msg.state_trajectory.size()) {
    throw std::runtime_error("joint target time and state trajectory sizes do not match");
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

  if (msg.joint_names.size() != n) {
    throw std::runtime_error("joint_names size must match the MPC joint dimension");
  }

  std::vector<std::size_t> reorder_indices(n);
  std::unordered_map<std::string, std::size_t> incoming;
  for (std::size_t i = 0; i < msg.joint_names.size(); ++i) {
    if (!incoming.emplace(msg.joint_names[i], i).second) {
      throw std::runtime_error("duplicate joint target name: " + msg.joint_names[i]);
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    const auto it = incoming.find(model.dofNames()[i]);
    if (it == incoming.end()) {
      throw std::runtime_error("missing joint target for " + model.dofNames()[i]);
    }
    reorder_indices[i] = it->second;
  }

  const bool has_position_weights = !msg.joint_position_tracking_weights.data.empty();
  const bool has_velocity_weights = !msg.joint_velocity_tracking_weights.data.empty();
  if (has_position_weights != has_velocity_weights) {
    throw std::runtime_error(
      "joint_position_tracking_weights and joint_velocity_tracking_weights must be provided together");
  }
  const ocs2::vector_t position_weights = has_position_weights ?
    weightVectorFromMessage(
      msg.joint_position_tracking_weights, reorder_indices, "joint_position_tracking_weights") :
    ocs2::vector_t();
  const ocs2::vector_t velocity_weights = has_velocity_weights ?
    weightVectorFromMessage(
      msg.joint_velocity_tracking_weights, reorder_indices, "joint_velocity_tracking_weights") :
    ocs2::vector_t();

  ocs2::vector_array_t state_trajectory;
  ocs2::vector_array_t input_trajectory;
  state_trajectory.reserve(msg.state_trajectory.size());
  input_trajectory.reserve(msg.state_trajectory.size());

  for (std::size_t sample = 0; sample < msg.state_trajectory.size(); ++sample) {
    const ocs2::vector_t raw_state = vectorFromMessage(msg.state_trajectory[sample]);
    ocs2::vector_t state =
      ocs2::vector_t::Zero(static_cast<Eigen::Index>(model.stateDim()));
    if (raw_state.size() == static_cast<Eigen::Index>(n) ||
        raw_state.size() == static_cast<Eigen::Index>(2 * n)) {
      for (std::size_t i = 0; i < n; ++i) {
        state(static_cast<Eigen::Index>(i)) =
          raw_state(static_cast<Eigen::Index>(reorder_indices[i]));
        if (raw_state.size() == static_cast<Eigen::Index>(2 * n)) {
          state(static_cast<Eigen::Index>(n + i)) =
            raw_state(static_cast<Eigen::Index>(n + reorder_indices[i]));
        }
      }
    } else {
      throw std::runtime_error("joint target state dimension must be n or 2n");
    }

    ocs2::vector_t input = interface_.computeNonlinearEffects(
      model.getQ(state),
      model.getV(state));

    if (!msg.joint_torque_trajectory.empty()) {
      const ocs2::vector_t raw_torque =
        vectorFromMessage(msg.joint_torque_trajectory[sample]);
      if (raw_torque.size() != static_cast<Eigen::Index>(n)) {
        throw std::runtime_error("joint_torque_trajectory sample dimension must match joint dimension");
      }
      for (std::size_t i = 0; i < n; ++i) {
        input(static_cast<Eigen::Index>(i)) =
          raw_torque(static_cast<Eigen::Index>(reorder_indices[i]));
      }
    }

    state_trajectory.push_back(makeTargetState(
      state,
      has_position_weights ? &position_weights : nullptr,
      has_velocity_weights ? &velocity_weights : nullptr));
    input_trajectory.push_back(std::move(input));
  }

  return TargetTrajectories(msg.time_trajectory, state_trajectory, input_trajectory);
}

}  // namespace target
}  // namespace dynamics_mpc_controller
