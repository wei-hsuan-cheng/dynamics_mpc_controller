#ifndef DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__TARGET__EE_MOTION_TRACKING_TARGET_HPP_
#define DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__TARGET__EE_MOTION_TRACKING_TARGET_HPP_

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_msgs/msg/dynamics_mpc_targets.hpp>

namespace dynamics_mpc_controller
{

class ForwardDynamicsMpcInterface;

namespace target
{

class ForwardEeMotionTrackingTarget
{
public:
  using TargetMsg = ocs2_msgs::msg::DynamicsMpcTargets;
  using TargetTrajectories = ocs2::TargetTrajectories;

  explicit ForwardEeMotionTrackingTarget(const ForwardDynamicsMpcInterface& interface);

  bool supports(const TargetMsg& msg) const noexcept;
  TargetTrajectories fromMessage(const TargetMsg& msg) const;

private:
  const ForwardDynamicsMpcInterface& interface_;
};

}  // namespace target
}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__TARGET__EE_MOTION_TRACKING_TARGET_HPP_
