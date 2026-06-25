#ifndef DYNAMICS_MPC_CONTROLLER__COMMON__COST__INPUT_TRACKING_COST_HPP_
#define DYNAMICS_MPC_CONTROLLER__COMMON__COST__INPUT_TRACKING_COST_HPP_

#include <cstddef>

#include <ocs2_core/cost/StateInputCost.h>

namespace dynamics_mpc_controller
{
namespace cost
{

class InputTrackingCost final : public ocs2::StateInputCost
{
public:
  InputTrackingCost(ocs2::matrix_t R, std::size_t inputDim);

  ~InputTrackingCost() override = default;
  InputTrackingCost* clone() const override { return new InputTrackingCost(*this); }

private:
  InputTrackingCost(const InputTrackingCost& rhs) = default;

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

  ocs2::vector_t getDesiredInput(
    ocs2::scalar_t time,
    const ocs2::TargetTrajectories& targetTrajectories) const;

  ocs2::matrix_t R_;
  std::size_t input_dim_{0};
};

}  // namespace cost
}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__COMMON__COST__INPUT_TRACKING_COST_HPP_
