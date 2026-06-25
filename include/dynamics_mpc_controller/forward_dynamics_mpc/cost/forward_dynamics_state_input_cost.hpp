#ifndef DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__COST__FORWARD_DYNAMICS_STATE_INPUT_COST_HPP_
#define DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__COST__FORWARD_DYNAMICS_STATE_INPUT_COST_HPP_

#include <cstddef>

#include <ocs2_core/cost/StateInputCost.h>

namespace dynamics_mpc_controller
{

class ForwardDynamicsStateInputCost final : public ocs2::StateInputCost
{
public:
  ForwardDynamicsStateInputCost(
    ocs2::matrix_t R,
    ocs2::vector_t defaultPositionWeights,
    ocs2::vector_t defaultVelocityWeights,
    std::size_t jointDim,
    std::size_t inputDim);

  ~ForwardDynamicsStateInputCost() override = default;
  ForwardDynamicsStateInputCost* clone() const override { return new ForwardDynamicsStateInputCost(*this); }

private:
  ForwardDynamicsStateInputCost(const ForwardDynamicsStateInputCost& rhs) = default;

  ocs2::scalar_t getValue(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    const ocs2::vector_t& input,
    const ocs2::TargetTrajectories& targetTrajectories,
    const ocs2::PreComputation& preComp) const override;

  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    const ocs2::vector_t& input,
    const ocs2::TargetTrajectories& targetTrajectories,
    const ocs2::PreComputation& preComp) const override;

  ocs2::matrix_t R_;
  ocs2::vector_t default_position_weights_;
  ocs2::vector_t default_velocity_weights_;
  std::size_t joint_dim_{0};
  std::size_t input_dim_{0};
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__COST__FORWARD_DYNAMICS_STATE_INPUT_COST_HPP_
