#ifndef DYNAMICS_MPC_CONTROLLER__COMMON__COST__EE_MOTION_TRACKING_COST_HPP_
#define DYNAMICS_MPC_CONTROLLER__COMMON__COST__EE_MOTION_TRACKING_COST_HPP_

#include <cstddef>

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/multibody/fwd.hpp>

namespace dynamics_mpc_controller
{
namespace cost
{

class EeMotionTrackingCost final : public ocs2::StateInputCost
{
public:
  EeMotionTrackingCost(
    ocs2::PinocchioInterface pinocchioInterface,
    pinocchio::FrameIndex endEffectorFrameId,
    ocs2::vector_t defaultPoseWeights,
    ocs2::vector_t defaultTwistWeights,
    std::size_t jointDim);

  ~EeMotionTrackingCost() override = default;
  EeMotionTrackingCost* clone() const override { return new EeMotionTrackingCost(*this); }

private:
  struct Target
  {
    ocs2::vector_t residual;
    ocs2::vector_t weights;
  };

  EeMotionTrackingCost(const EeMotionTrackingCost& rhs) = default;

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

  Target getTargetResidual(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    const ocs2::TargetTrajectories& targetTrajectories) const;

  mutable ocs2::PinocchioInterface pinocchio_interface_;
  pinocchio::FrameIndex end_effector_frame_id_;
  ocs2::vector_t default_pose_weights_;
  ocs2::vector_t default_twist_weights_;
  std::size_t joint_dim_{0};
};

}  // namespace cost
}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__COMMON__COST__EE_MOTION_TRACKING_COST_HPP_
