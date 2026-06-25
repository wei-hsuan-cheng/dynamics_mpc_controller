#ifndef DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__CONSTRAINT__INVERSE_DYNAMICS_EE_WRENCH_TRACKING_CONSTRAINT_HPP_
#define DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__CONSTRAINT__INVERSE_DYNAMICS_EE_WRENCH_TRACKING_CONSTRAINT_HPP_

#include <cstddef>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

namespace dynamics_mpc_controller
{

class InverseDynamicsEeWrenchTrackingConstraint final : public ocs2::StateInputConstraint
{
public:
  InverseDynamicsEeWrenchTrackingConstraint(
    std::size_t stateDim,
    std::size_t inputDim,
    std::size_t wrenchOffset,
    const ocs2::ReferenceManagerInterface& referenceManager,
    bool trackZeroWrench);

  InverseDynamicsEeWrenchTrackingConstraint* clone() const override
  {
    return new InverseDynamicsEeWrenchTrackingConstraint(*this);
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
  ocs2::vector_t desiredWrench(ocs2::scalar_t time) const;

  std::size_t state_dim_;
  std::size_t input_dim_;
  std::size_t wrench_offset_;
  const ocs2::ReferenceManagerInterface* reference_manager_ptr_;
  bool track_zero_wrench_;
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__CONSTRAINT__INVERSE_DYNAMICS_EE_WRENCH_TRACKING_CONSTRAINT_HPP_
