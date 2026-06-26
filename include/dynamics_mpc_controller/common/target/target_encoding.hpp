#ifndef DYNAMICS_MPC_CONTROLLER__COMMON__TARGET__TARGET_ENCODING_HPP_
#define DYNAMICS_MPC_CONTROLLER__COMMON__TARGET__TARGET_ENCODING_HPP_

#include <Eigen/Core>

namespace dynamics_mpc_controller
{
namespace target_encoding
{

inline constexpr Eigen::Index kEeMotionPoseTargetDim = 7;
inline constexpr Eigen::Index kEeMotionTwistOnlyTargetDim = 8;
inline constexpr Eigen::Index kEeMotionTargetDim = 13;
inline constexpr Eigen::Index kEeMotionTargetWithTwistFrameDim = 14;
inline constexpr Eigen::Index kEeMotionWeightedTargetDim = 25;
inline constexpr Eigen::Index kEeMotionWeightedTargetWithTwistFrameDim = 26;

inline constexpr double kEeMotionTwistFrameBase = 0.0;
inline constexpr double kEeMotionTwistFrameEe = 1.0;

inline bool isEeMotionTargetStateSize(Eigen::Index size)
{
  return size == kEeMotionPoseTargetDim ||
         size == kEeMotionTwistOnlyTargetDim ||
         size == kEeMotionTargetDim ||
         size == kEeMotionTargetWithTwistFrameDim ||
         size == kEeMotionWeightedTargetDim ||
         size == kEeMotionWeightedTargetWithTwistFrameDim;
}

}  // namespace target_encoding
}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__COMMON__TARGET__TARGET_ENCODING_HPP_
