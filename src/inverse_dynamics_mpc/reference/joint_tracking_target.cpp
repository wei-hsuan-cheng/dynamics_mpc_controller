#include "dynamics_mpc_controller/inverse_dynamics_mpc/reference/joint_tracking_target.hpp"

#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include "dynamics_mpc_controller/inverse_dynamics_mpc/inverse_dynamics_mpc_interface.hpp"

namespace dynamics_mpc_controller
{
namespace reference
{

JointTrackingTarget::JointTrackingTarget(const InverseDynamicsMpcInterface& interface)
: interface_(interface)
{
}

bool JointTrackingTarget::supports(const TargetMsg& msg) const noexcept
{
  return msg.command_type == "joint";
}

JointTrackingTarget::TargetTrajectories JointTrackingTarget::fromObservation(
  const ocs2::SystemObservation& observation) const
{
  const auto& model = interface_.getInverseDynamicsMpcModel();
  // Preserve measured velocity for nonlinear feedforward before defining a zero-velocity hold target.
  const ocs2::vector_t q = model.getQ(observation.state);
  const ocs2::vector_t v = model.getV(observation.state);
  ocs2::vector_t desired_state = observation.state;
  desired_state.tail(static_cast<Eigen::Index>(model.jointDim())).setZero();

  ocs2::vector_t desired_input =
    ocs2::vector_t::Zero(static_cast<Eigen::Index>(model.inputDim()));
  desired_input.segment(
    static_cast<Eigen::Index>(model.tauOffset()),
    static_cast<Eigen::Index>(model.jointDim())) =
    interface_.computeNonlinearEffects(q, v);

  return TargetTrajectories(
    {observation.time},
    {std::move(desired_state)},
    {std::move(desired_input)});
}

JointTrackingTarget::TargetTrajectories JointTrackingTarget::fromMessage(
  const TargetMsg& msg) const
{
  if (!supports(msg)) {
    throw std::invalid_argument(
      "joint tracking target requires command_type='joint', got '" + msg.command_type + "'");
  }
  if (msg.target_trajectories.empty()) {
    throw std::runtime_error("joint target message has no trajectories");
  }

  const auto& model = interface_.getInverseDynamicsMpcModel();
  const std::size_t n = model.jointDim();
  ocs2::TargetTrajectories raw =
    ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(msg.target_trajectories.front());

  if (raw.timeTrajectory.empty() || raw.stateTrajectory.empty()) {
    throw std::runtime_error("joint target trajectory is empty");
  }
  if (raw.timeTrajectory.size() != raw.stateTrajectory.size()) {
    throw std::runtime_error("joint target time and state trajectory sizes do not match");
  }
  if (!raw.inputTrajectory.empty() && raw.inputTrajectory.size() != raw.stateTrajectory.size()) {
    throw std::runtime_error("joint target input and state trajectory sizes must match when inputs are provided");
  }

  std::vector<std::size_t> reorder_indices(n);
  for (std::size_t i = 0; i < n; ++i) {
    reorder_indices[i] = i;
  }
  if (msg.joint_names.size() == n) {
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
  } else if (!msg.joint_names.empty()) {
    throw std::runtime_error("joint_names size must be zero or match the MPC joint dimension");
  }

  ocs2::vector_array_t state_trajectory;
  ocs2::vector_array_t input_trajectory;
  state_trajectory.reserve(raw.stateTrajectory.size());
  input_trajectory.reserve(raw.stateTrajectory.size());

  for (std::size_t sample = 0; sample < raw.stateTrajectory.size(); ++sample) {
    const auto& raw_state = raw.stateTrajectory[sample];
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

    ocs2::vector_t input =
      ocs2::vector_t::Zero(static_cast<Eigen::Index>(model.inputDim()));
    input.segment(
      static_cast<Eigen::Index>(model.tauOffset()),
      static_cast<Eigen::Index>(n)) =
      interface_.computeNonlinearEffects(model.getQ(state), model.getV(state));

    if (model.hasEeWrenchInput() && !model.trackZeroWrench() && !raw.inputTrajectory.empty()) {
      const auto& raw_input = raw.inputTrajectory[sample];
      const Eigen::Index raw_input_size = raw_input.size();
      if (raw_input_size == 6) {
        input.segment(static_cast<Eigen::Index>(model.wrenchOffset()), 6) = raw_input;
      } else if (raw_input_size == static_cast<Eigen::Index>(model.inputDim())) {
        input.segment(static_cast<Eigen::Index>(model.wrenchOffset()), 6) =
          raw_input.segment(static_cast<Eigen::Index>(model.wrenchOffset()), 6);
      } else {
        throw std::runtime_error(
          "joint target input dimension must be 6 or full MPC input dimension when trackZeroWrench=false");
      }
    }

    state_trajectory.push_back(std::move(state));
    input_trajectory.push_back(std::move(input));
  }

  return TargetTrajectories(raw.timeTrajectory, state_trajectory, input_trajectory);
}

}  // namespace reference
}  // namespace dynamics_mpc_controller
