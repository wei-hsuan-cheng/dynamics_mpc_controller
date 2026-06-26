#ifndef DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__TARGET__EE_MOTION_TRACKING_TARGET_HPP_
#define DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__TARGET__EE_MOTION_TRACKING_TARGET_HPP_

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_msgs/msg/dynamics_mpc_targets.hpp>

namespace dynamics_mpc_controller
{

class InverseDynamicsMpcInterface;

namespace target
{

class EeMotionTrackingTarget
{
public:
  using TargetMsg = ocs2_msgs::msg::DynamicsMpcTargets;
  using TargetTrajectories = ocs2::TargetTrajectories;

  explicit EeMotionTrackingTarget(const InverseDynamicsMpcInterface& interface);

  bool supports(const TargetMsg& msg) const noexcept;
  TargetTrajectories fromMessage(const TargetMsg& msg) const;

private:
  const InverseDynamicsMpcInterface& interface_;
};

}  // namespace target
}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__TARGET__EE_MOTION_TRACKING_TARGET_HPP_
