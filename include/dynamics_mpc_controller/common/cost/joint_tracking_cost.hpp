#ifndef DYNAMICS_MPC_CONTROLLER__COMMON__COST__JOINT_TRACKING_COST_HPP_
#define DYNAMICS_MPC_CONTROLLER__COMMON__COST__JOINT_TRACKING_COST_HPP_

#include <cstddef>

#include <ocs2_core/cost/StateInputCost.h>

namespace dynamics_mpc_controller
{
namespace cost
{

class JointTrackingCost final : public ocs2::StateInputCost
{
public:
  JointTrackingCost(
    ocs2::vector_t defaultPositionWeights,
    ocs2::vector_t defaultVelocityWeights,
    std::size_t jointDim);

  ~JointTrackingCost() override = default;
  JointTrackingCost* clone() const override { return new JointTrackingCost(*this); }

private:
  struct Target
  {
    ocs2::vector_t state;
    ocs2::vector_t weights;
  };

  JointTrackingCost(const JointTrackingCost& rhs) = default;

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

  Target getTarget(
    ocs2::scalar_t time,
    const ocs2::TargetTrajectories& targetTrajectories) const;

  ocs2::vector_t default_position_weights_;
  ocs2::vector_t default_velocity_weights_;
  std::size_t joint_dim_{0};
};

}  // namespace cost
}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__COMMON__COST__JOINT_TRACKING_COST_HPP_
