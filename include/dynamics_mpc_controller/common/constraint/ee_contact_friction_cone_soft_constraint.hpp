#ifndef DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__EE_CONTACT_FRICTION_CONE_SOFT_CONSTRAINT_HPP_
#define DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__EE_CONTACT_FRICTION_CONE_SOFT_CONSTRAINT_HPP_

#include <cstddef>
#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/StateInputCost.h>

namespace dynamics_mpc_controller
{
namespace constraint
{

class EeContactFrictionConeConstraint final : public ocs2::StateInputConstraint
{
public:
  EeContactFrictionConeConstraint(
    std::size_t stateDim,
    std::size_t inputDim,
    std::size_t wrenchOffset,
    const ocs2::vector_t& contactNormal,
    ocs2::scalar_t frictionCoefficient,
    ocs2::scalar_t coneRegularization,
    ocs2::scalar_t normalForceLowerBound);

  EeContactFrictionConeConstraint* clone() const override
  {
    return new EeContactFrictionConeConstraint(*this);
  }

  std::size_t getNumConstraints(ocs2::scalar_t time) const override;

  ocs2::vector_t getValue(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    const ocs2::vector_t& input,
    const ocs2::PreComputation& preComputation) const override;

  ocs2::VectorFunctionLinearApproximation getLinearApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    const ocs2::vector_t& input,
    const ocs2::PreComputation& preComputation) const override;

private:
  std::size_t state_dim_;
  std::size_t input_dim_;
  std::size_t wrench_offset_;
  ocs2::vector_t contact_normal_;
  ocs2::scalar_t friction_coefficient_;
  ocs2::scalar_t cone_regularization_;
  ocs2::scalar_t normal_force_lower_bound_;
};

std::unique_ptr<ocs2::StateInputCost> createEeContactFrictionConeSoftConstraint(
  std::size_t stateDim,
  std::size_t inputDim,
  std::size_t wrenchOffset,
  const ocs2::vector_t& contactNormal,
  ocs2::scalar_t frictionCoefficient,
  ocs2::scalar_t coneRegularization,
  ocs2::scalar_t normalForceLowerBound,
  ocs2::scalar_t penaltyMu,
  ocs2::scalar_t penaltyDelta);

}  // namespace constraint
}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__EE_CONTACT_FRICTION_CONE_SOFT_CONSTRAINT_HPP_
