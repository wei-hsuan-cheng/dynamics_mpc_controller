#include "dynamics_mpc_controller/common/cost/joint_tracking_cost.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace dynamics_mpc_controller
{
namespace cost
{
namespace
{

void validateWeights(const ocs2::vector_t& weights, std::size_t jointDim, const char* name)
{
  if (weights.size() != static_cast<Eigen::Index>(jointDim)) {
    throw std::runtime_error(std::string("[JointTrackingCost] ") + name + " size mismatch.");
  }
  if ((weights.array() < 0.0).any()) {
    throw std::runtime_error(std::string("[JointTrackingCost] ") + name + " entries must be non-negative.");
  }
}

}  // namespace

JointTrackingCost::JointTrackingCost(
  ocs2::vector_t defaultPositionWeights,
  ocs2::vector_t defaultVelocityWeights,
  std::size_t jointDim)
: default_position_weights_(std::move(defaultPositionWeights)),
  default_velocity_weights_(std::move(defaultVelocityWeights)),
  joint_dim_(jointDim)
{
  validateWeights(default_position_weights_, joint_dim_, "defaultPositionWeights");
  validateWeights(default_velocity_weights_, joint_dim_, "defaultVelocityWeights");
}

JointTrackingCost::Target JointTrackingCost::getTarget(
  ocs2::scalar_t time,
  const ocs2::TargetTrajectories& targetTrajectories) const
{
  const Eigen::Index n = static_cast<Eigen::Index>(joint_dim_);
  Target target{
    ocs2::vector_t::Zero(2 * n),
    ocs2::vector_t::Zero(2 * n)};

  target.weights.head(n) = default_position_weights_;
  target.weights.tail(n) = default_velocity_weights_;

  const ocs2::vector_t raw_state = targetTrajectories.getDesiredState(time);
  if (raw_state.size() == 4 * n) {
    target.state = raw_state.head(2 * n);
    target.weights.head(n) = raw_state.segment(2 * n, n);
    target.weights.tail(n) = raw_state.segment(3 * n, n);
  } else if (raw_state.size() == 3 * n) {
    target.state.tail(n) = raw_state.head(n);
    target.weights.head(n).setZero();
  } else if (raw_state.size() == 2 * n) {
    target.state = raw_state;
  } else if (raw_state.size() == n) {
    target.state.head(n) = raw_state;
    target.weights.tail(n).setZero();
  } else {
    target.weights.setZero();
  }

  if ((target.weights.array() < 0.0).any()) {
    throw std::runtime_error("[JointTrackingCost] target weights must be non-negative.");
  }
  return target;
}

ocs2::scalar_t JointTrackingCost::getValue(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::vector_t&,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::PreComputation&) const
{
  const auto target = getTarget(time, targetTrajectories);
  const ocs2::vector_t state_deviation = state - target.state;
  return 0.5 * state_deviation.dot(target.weights.cwiseProduct(state_deviation));
}

ocs2::ScalarFunctionQuadraticApproximation JointTrackingCost::getQuadraticApproximation(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::vector_t& input,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::PreComputation&) const
{
  const auto target = getTarget(time, targetTrajectories);
  const ocs2::vector_t state_deviation = state - target.state;

  ocs2::ScalarFunctionQuadraticApproximation cost;
  cost.setZero(state.size(), input.size());
  cost.dfdxx = target.weights.asDiagonal();
  cost.dfdx = target.weights.cwiseProduct(state_deviation);
  cost.f = 0.5 * state_deviation.dot(cost.dfdx);
  return cost;
}

}  // namespace cost
}  // namespace dynamics_mpc_controller
