#include "dynamics_mpc_controller/inverse_dynamics_mpc/cost/inverse_dynamics_state_input_cost.hpp"

#include <utility>

namespace dynamics_mpc_controller
{
namespace
{

struct WeightedTarget
{
  ocs2::vector_t state;
  ocs2::vector_t weights;
};

WeightedTarget getWeightedTarget(
  ocs2::scalar_t time,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::vector_t& defaultPositionWeights,
  const ocs2::vector_t& defaultVelocityWeights,
  std::size_t jointDim)
{
  const Eigen::Index n = static_cast<Eigen::Index>(jointDim);
  WeightedTarget target{
    ocs2::vector_t::Zero(2 * n),
    ocs2::vector_t::Zero(2 * n)};

  target.weights.head(n) = defaultPositionWeights;
  target.weights.tail(n) = defaultVelocityWeights;

  const ocs2::vector_t raw_state = targetTrajectories.getDesiredState(time);
  if (raw_state.size() == 4 * n) {
    target.state = raw_state.head(2 * n);
    target.weights.head(n) = raw_state.segment(2 * n, n);
    target.weights.tail(n) = raw_state.segment(3 * n, n);
  } else if (raw_state.size() == 2 * n) {
    target.state = raw_state;
  } else if (raw_state.size() == n) {
    target.state.head(n) = raw_state;
  }
  return target;
}

ocs2::vector_t getDesiredInput(
  ocs2::scalar_t time,
  const ocs2::TargetTrajectories& targetTrajectories,
  std::size_t inputDim)
{
  ocs2::vector_t desired_input = ocs2::vector_t::Zero(static_cast<Eigen::Index>(inputDim));
  const ocs2::vector_t raw_input = targetTrajectories.getDesiredInput(time);
  if (raw_input.size() == desired_input.size()) {
    desired_input = raw_input;
  }
  return desired_input;
}

}  // namespace

InverseDynamicsStateInputCost::InverseDynamicsStateInputCost(
  ocs2::matrix_t R,
  ocs2::vector_t defaultPositionWeights,
  ocs2::vector_t defaultVelocityWeights,
  std::size_t jointDim,
  std::size_t inputDim)
: R_(std::move(R)),
  default_position_weights_(std::move(defaultPositionWeights)),
  default_velocity_weights_(std::move(defaultVelocityWeights)),
  joint_dim_(jointDim),
  input_dim_(inputDim)
{
}

ocs2::scalar_t InverseDynamicsStateInputCost::getValue(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::vector_t& input,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::PreComputation&) const
{
  const auto target = getWeightedTarget(
    time, targetTrajectories, default_position_weights_, default_velocity_weights_, joint_dim_);
  const ocs2::vector_t desired_input =
    getDesiredInput(time, targetTrajectories, input_dim_);

  const ocs2::vector_t state_deviation = state - target.state;
  const ocs2::vector_t input_deviation = input - desired_input;
  return 0.5 * state_deviation.dot(target.weights.cwiseProduct(state_deviation)) +
         0.5 * input_deviation.dot(R_ * input_deviation);
}

ocs2::ScalarFunctionQuadraticApproximation InverseDynamicsStateInputCost::getQuadraticApproximation(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::vector_t& input,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::PreComputation&) const
{
  const auto target = getWeightedTarget(
    time, targetTrajectories, default_position_weights_, default_velocity_weights_, joint_dim_);
  const ocs2::vector_t desired_input =
    getDesiredInput(time, targetTrajectories, input_dim_);

  const ocs2::vector_t state_deviation = state - target.state;
  const ocs2::vector_t input_deviation = input - desired_input;

  ocs2::ScalarFunctionQuadraticApproximation cost;
  cost.setZero(state.size(), input.size());
  cost.dfdxx = target.weights.asDiagonal();
  cost.dfduu = R_;
  cost.dfdx = target.weights.cwiseProduct(state_deviation);
  cost.dfdu = R_ * input_deviation;
  cost.f = 0.5 * state_deviation.dot(cost.dfdx) +
           0.5 * input_deviation.dot(cost.dfdu);

  return cost;
}

}  // namespace dynamics_mpc_controller
