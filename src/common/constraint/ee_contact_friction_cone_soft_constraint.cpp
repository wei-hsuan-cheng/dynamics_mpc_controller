#include "dynamics_mpc_controller/common/constraint/ee_contact_friction_cone_soft_constraint.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>

namespace dynamics_mpc_controller
{
namespace constraint
{

namespace
{

ocs2::vector_t normalizedContactNormal(const ocs2::vector_t& contactNormal)
{
  if (contactNormal.size() != 3) {
    throw std::runtime_error(
      "[EeContactFrictionConeConstraint] contact normal must contain exactly 3 values.");
  }

  const ocs2::scalar_t norm = contactNormal.norm();
  if (norm <= 1.0e-12) {
    throw std::runtime_error(
      "[EeContactFrictionConeConstraint] contact normal must be nonzero.");
  }
  return contactNormal / norm;
}

void validateDimensions(
  std::size_t inputDim,
  std::size_t wrenchOffset)
{
  if (wrenchOffset + 6 > inputDim) {
    throw std::runtime_error(
      "[EeContactFrictionConeConstraint] wrench input segment exceeds input dimension.");
  }
}

}  // namespace

EeContactFrictionConeConstraint::EeContactFrictionConeConstraint(
  std::size_t stateDim,
  std::size_t inputDim,
  std::size_t wrenchOffset,
  const ocs2::vector_t& contactNormal,
  ocs2::scalar_t frictionCoefficient,
  ocs2::scalar_t coneRegularization,
  ocs2::scalar_t normalForceLowerBound)
: ocs2::StateInputConstraint(ocs2::ConstraintOrder::Linear),
  state_dim_(stateDim),
  input_dim_(inputDim),
  wrench_offset_(wrenchOffset),
  contact_normal_(normalizedContactNormal(contactNormal)),
  friction_coefficient_(frictionCoefficient),
  cone_regularization_(coneRegularization),
  normal_force_lower_bound_(normalForceLowerBound)
{
  validateDimensions(input_dim_, wrench_offset_);
  if (friction_coefficient_ <= 0.0) {
    throw std::runtime_error(
      "[EeContactFrictionConeConstraint] friction coefficient must be positive.");
  }
  if (cone_regularization_ <= 0.0) {
    throw std::runtime_error(
      "[EeContactFrictionConeConstraint] cone regularization must be positive.");
  }
}

std::size_t EeContactFrictionConeConstraint::getNumConstraints(ocs2::scalar_t) const
{
  return 2;
}

ocs2::vector_t EeContactFrictionConeConstraint::getValue(
  ocs2::scalar_t,
  const ocs2::vector_t&,
  const ocs2::vector_t& input,
  const ocs2::PreComputation&) const
{
  if (input.size() != static_cast<Eigen::Index>(input_dim_)) {
    throw std::runtime_error(
      "[EeContactFrictionConeConstraint] input dimension does not match the MPC model.");
  }

  const ocs2::vector_t force =
    input.segment(static_cast<Eigen::Index>(wrench_offset_), 3);
  const ocs2::scalar_t normal_force = contact_normal_.dot(force);
  const ocs2::vector_t tangent_force = force - contact_normal_ * normal_force;
  const ocs2::scalar_t tangent_norm =
    std::sqrt(tangent_force.squaredNorm() + cone_regularization_);

  ocs2::vector_t value(2);
  value(0) = normal_force - normal_force_lower_bound_;
  value(1) = friction_coefficient_ * normal_force - tangent_norm;
  return value;
}

ocs2::VectorFunctionLinearApproximation
EeContactFrictionConeConstraint::getLinearApproximation(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::vector_t& input,
  const ocs2::PreComputation& preComputation) const
{
  ocs2::VectorFunctionLinearApproximation approximation;
  approximation.f = getValue(time, state, input, preComputation);
  approximation.dfdx =
    ocs2::matrix_t::Zero(2, static_cast<Eigen::Index>(state_dim_));
  approximation.dfdu =
    ocs2::matrix_t::Zero(2, static_cast<Eigen::Index>(input_dim_));

  const ocs2::vector_t force =
    input.segment(static_cast<Eigen::Index>(wrench_offset_), 3);
  const ocs2::scalar_t normal_force = contact_normal_.dot(force);
  const ocs2::vector_t tangent_force = force - contact_normal_ * normal_force;
  const ocs2::scalar_t tangent_norm =
    std::sqrt(tangent_force.squaredNorm() + cone_regularization_);

  approximation.dfdu.block(0, static_cast<Eigen::Index>(wrench_offset_), 1, 3) =
    contact_normal_.transpose();
  approximation.dfdu.block(1, static_cast<Eigen::Index>(wrench_offset_), 1, 3) =
    friction_coefficient_ * contact_normal_.transpose() -
    tangent_force.transpose() / tangent_norm;
  return approximation;
}

std::unique_ptr<ocs2::StateInputCost> createEeContactFrictionConeSoftConstraint(
  std::size_t stateDim,
  std::size_t inputDim,
  std::size_t wrenchOffset,
  const ocs2::vector_t& contactNormal,
  ocs2::scalar_t frictionCoefficient,
  ocs2::scalar_t coneRegularization,
  ocs2::scalar_t normalForceLowerBound,
  ocs2::scalar_t penaltyMu,
  ocs2::scalar_t penaltyDelta)
{
  return std::make_unique<ocs2::StateInputSoftConstraint>(
    std::make_unique<EeContactFrictionConeConstraint>(
      stateDim,
      inputDim,
      wrenchOffset,
      contactNormal,
      frictionCoefficient,
      coneRegularization,
      normalForceLowerBound),
    std::make_unique<ocs2::RelaxedBarrierPenalty>(
      ocs2::RelaxedBarrierPenalty::Config{penaltyMu, penaltyDelta}));
}

}  // namespace constraint
}  // namespace dynamics_mpc_controller
