#include "dynamics_mpc_controller/inverse_dynamics_mpc/cost/inverse_dynamics_state_input_cost.hpp"

#include <utility>

namespace dynamics_mpc_controller
{

InverseDynamicsStateInputCost::InverseDynamicsStateInputCost(
  ocs2::matrix_t Q,
  ocs2::matrix_t R,
  std::size_t jointDim,
  std::size_t inputDim)
: ocs2::QuadraticStateInputCost(std::move(Q), std::move(R)),
  joint_dim_(jointDim),
  input_dim_(inputDim)
{
}

std::pair<ocs2::vector_t, ocs2::vector_t> InverseDynamicsStateInputCost::getStateInputDeviation(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::vector_t& input,
  const ocs2::TargetTrajectories& targetTrajectories) const
{
  ocs2::vector_t desired_state = ocs2::vector_t::Zero(2 * joint_dim_);
  const ocs2::vector_t raw_state = targetTrajectories.getDesiredState(time);
  if (raw_state.size() == desired_state.size()) {
    desired_state = raw_state;
  } else if (raw_state.size() == static_cast<Eigen::Index>(joint_dim_)) {
    desired_state.head(static_cast<Eigen::Index>(joint_dim_)) = raw_state;
  }

  ocs2::vector_t desired_input = ocs2::vector_t::Zero(static_cast<Eigen::Index>(input_dim_));
  const ocs2::vector_t raw_input = targetTrajectories.getDesiredInput(time);
  if (raw_input.size() == desired_input.size()) {
    desired_input = raw_input;
  }

  return {state - desired_state, input - desired_input};
}

}  // namespace dynamics_mpc_controller
