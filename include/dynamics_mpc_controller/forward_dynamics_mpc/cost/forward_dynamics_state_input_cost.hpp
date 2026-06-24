#ifndef DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__COST__FORWARD_DYNAMICS_STATE_INPUT_COST_HPP_
#define DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__COST__FORWARD_DYNAMICS_STATE_INPUT_COST_HPP_

#include <cstddef>
#include <utility>

#include <ocs2_core/cost/QuadraticStateInputCost.h>

namespace dynamics_mpc_controller
{

class ForwardDynamicsStateInputCost final : public ocs2::QuadraticStateInputCost
{
public:
  ForwardDynamicsStateInputCost(
    ocs2::matrix_t Q,
    ocs2::matrix_t R,
    std::size_t jointDim,
    std::size_t inputDim);

  ~ForwardDynamicsStateInputCost() override = default;
  ForwardDynamicsStateInputCost* clone() const override { return new ForwardDynamicsStateInputCost(*this); }

private:
  ForwardDynamicsStateInputCost(const ForwardDynamicsStateInputCost& rhs) = default;

  std::pair<ocs2::vector_t, ocs2::vector_t> getStateInputDeviation(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    const ocs2::vector_t& input,
    const ocs2::TargetTrajectories& targetTrajectories) const override;

  std::size_t joint_dim_{0};
  std::size_t input_dim_{0};
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__COST__FORWARD_DYNAMICS_STATE_INPUT_COST_HPP_
