#include "dynamics_mpc_controller/inverse_dynamics_mpc/constraint/inverse_dynamics_ee_wrench_tracking_constraint.hpp"

#include <stdexcept>

namespace dynamics_mpc_controller
{

InverseDynamicsEeWrenchTrackingConstraint::InverseDynamicsEeWrenchTrackingConstraint(
  std::size_t stateDim,
  std::size_t inputDim,
  std::size_t wrenchOffset,
  const ocs2::ReferenceManagerInterface& referenceManager)
: ocs2::StateInputConstraint(ocs2::ConstraintOrder::Linear),
  state_dim_(stateDim),
  input_dim_(inputDim),
  wrench_offset_(wrenchOffset),
  reference_manager_ptr_(&referenceManager)
{
  if (wrench_offset_ + 6 > input_dim_) {
    throw std::runtime_error(
      "[InverseDynamicsEeWrenchTrackingConstraint] wrench input segment exceeds input dimension.");
  }
}

std::size_t InverseDynamicsEeWrenchTrackingConstraint::getNumConstraints(ocs2::scalar_t) const
{
  return 6;
}

ocs2::vector_t InverseDynamicsEeWrenchTrackingConstraint::getValue(
  ocs2::scalar_t time,
  const ocs2::vector_t&,
  const ocs2::vector_t& input,
  const ocs2::PreComputation&) const
{
  if (input.size() != static_cast<Eigen::Index>(input_dim_)) {
    throw std::runtime_error(
      "[InverseDynamicsEeWrenchTrackingConstraint] input dimension does not match the MPC model.");
  }

  return input.segment(static_cast<Eigen::Index>(wrench_offset_), 6) - desiredWrench(time);
}

ocs2::VectorFunctionLinearApproximation
InverseDynamicsEeWrenchTrackingConstraint::getLinearApproximation(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::vector_t& input,
  const ocs2::PreComputation& preComputation) const
{
  ocs2::VectorFunctionLinearApproximation approximation;
  approximation.f = getValue(time, state, input, preComputation);
  approximation.dfdx = ocs2::matrix_t::Zero(6, static_cast<Eigen::Index>(state_dim_));
  approximation.dfdu = ocs2::matrix_t::Zero(6, static_cast<Eigen::Index>(input_dim_));
  approximation.dfdu.block(0, static_cast<Eigen::Index>(wrench_offset_), 6, 6).setIdentity();
  return approximation;
}

ocs2::vector_t InverseDynamicsEeWrenchTrackingConstraint::desiredWrench(ocs2::scalar_t time) const
{
  const auto& target_trajectories = reference_manager_ptr_->getTargetTrajectories();
  if (target_trajectories.empty() || target_trajectories.inputTrajectory.empty()) {
    return ocs2::vector_t::Zero(6);
  }

  const ocs2::vector_t desired_input = target_trajectories.getDesiredInput(time);
  if (desired_input.size() == 6) {
    return desired_input;
  }
  if (desired_input.size() == static_cast<Eigen::Index>(input_dim_)) {
    return desired_input.segment(static_cast<Eigen::Index>(wrench_offset_), 6);
  }

  throw std::runtime_error(
    "[InverseDynamicsEeWrenchTrackingConstraint] target input dimension must be 6 or full MPC input dimension.");
}

}  // namespace dynamics_mpc_controller
