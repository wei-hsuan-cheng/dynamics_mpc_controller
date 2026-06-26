#include "dynamics_mpc_controller/common/cost/ee_motion_tracking_cost.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "dynamics_mpc_controller/common/target/target_encoding.hpp"

namespace dynamics_mpc_controller
{
namespace cost
{
namespace
{

constexpr Eigen::Index kResidualDim = 12;
constexpr Eigen::Index kVector6Dim = 6;
constexpr Eigen::Index kEeMotionPoseWeightsOffset = target_encoding::kEeMotionTargetDim;
constexpr Eigen::Index kEeMotionTwistWeightsOffset = kEeMotionPoseWeightsOffset + kVector6Dim;
constexpr Eigen::Index kEeMotionFullTwistFrameOffset = target_encoding::kEeMotionTargetDim;
constexpr Eigen::Index kEeMotionWeightedTwistFrameOffset =
  target_encoding::kEeMotionWeightedTargetDim;

void validateWeights(const ocs2::vector_t& weights, const char* name)
{
  if (weights.size() != kVector6Dim) {
    throw std::runtime_error(std::string("[EeMotionTrackingCost] ") + name + " size must be 6.");
  }
  if ((weights.array() < 0.0).any()) {
    throw std::runtime_error(std::string("[EeMotionTrackingCost] ") + name + " entries must be non-negative.");
  }
}

Eigen::Quaternion<ocs2::scalar_t> quaternionFromTarget(const ocs2::vector_t& target, Eigen::Index offset)
{
  Eigen::Quaternion<ocs2::scalar_t> orientation(
    target(offset + 3),
    target(offset + 0),
    target(offset + 1),
    target(offset + 2));
  if (orientation.squaredNorm() < 1.0e-12) {
    orientation.setIdentity();
  } else {
    orientation.normalize();
  }
  return orientation;
}

bool targetUsesEeFrameTwist(const ocs2::vector_t& target)
{
  if (target.size() == target_encoding::kEeMotionTwistOnlyTargetDim) {
    return std::abs(target(0) - target_encoding::kEeMotionTwistFrameEe) < 0.5;
  }
  if (target.size() == target_encoding::kEeMotionTargetWithTwistFrameDim) {
    return std::abs(target(kEeMotionFullTwistFrameOffset) -
      target_encoding::kEeMotionTwistFrameEe) < 0.5;
  }
  if (target.size() == target_encoding::kEeMotionWeightedTargetWithTwistFrameDim) {
    return std::abs(target(kEeMotionWeightedTwistFrameOffset) -
      target_encoding::kEeMotionTwistFrameEe) < 0.5;
  }
  return false;
}

}  // namespace

EeMotionTrackingCost::EeMotionTrackingCost(
  ocs2::PinocchioInterface pinocchioInterface,
  pinocchio::FrameIndex endEffectorFrameId,
  ocs2::vector_t defaultPoseWeights,
  ocs2::vector_t defaultTwistWeights,
  std::size_t jointDim,
  ocs2::scalar_t weightScale)
: pinocchio_interface_(std::move(pinocchioInterface)),
  end_effector_frame_id_(endEffectorFrameId),
  default_pose_weights_(std::move(defaultPoseWeights)),
  default_twist_weights_(std::move(defaultTwistWeights)),
  joint_dim_(jointDim),
  weight_scale_(weightScale)
{
  validateWeights(default_pose_weights_, "defaultPoseWeights");
  validateWeights(default_twist_weights_, "defaultTwistWeights");
  if (!std::isfinite(weight_scale_) || weight_scale_ < 0.0) {
    throw std::runtime_error("[EeMotionTrackingCost] weightScale must be finite and non-negative.");
  }
}

EeMotionTrackingCost::Target EeMotionTrackingCost::getTargetResidual(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::TargetTrajectories& targetTrajectories) const
{
  Target target{
    ocs2::vector_t::Zero(kResidualDim),
    ocs2::vector_t::Zero(kResidualDim)};

  const ocs2::vector_t raw_target = targetTrajectories.getDesiredState(time);
  if (!target_encoding::isEeMotionTargetStateSize(raw_target.size())) {
    return target;
  }

  if (state.size() < static_cast<Eigen::Index>(2 * joint_dim_)) {
    throw std::runtime_error("[EeMotionTrackingCost] state dimension is smaller than 2 * jointDim.");
  }

  const Eigen::Index n = static_cast<Eigen::Index>(joint_dim_);
  const ocs2::vector_t q = state.head(n);
  const ocs2::vector_t v = state.segment(n, n);

  const auto& model = pinocchio_interface_.getModel();
  auto& data = pinocchio_interface_.getData();
  pinocchio::forwardKinematics(model, data, q, v);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  const auto& placement = data.oMf[end_effector_frame_id_];
  Eigen::Quaternion<ocs2::scalar_t> orientation(placement.rotation());
  if (orientation.squaredNorm() < 1.0e-12) {
    orientation.setIdentity();
  } else {
    orientation.normalize();
  }

  Eigen::Matrix<ocs2::scalar_t, 6, Eigen::Dynamic> frame_jacobian(6, model.nv);
  frame_jacobian.setZero();
  const auto twist_reference_frame = targetUsesEeFrameTwist(raw_target) ?
    pinocchio::ReferenceFrame::LOCAL :
    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  pinocchio::getFrameJacobian(
    model,
    data,
    end_effector_frame_id_,
    twist_reference_frame,
    frame_jacobian);
  const ocs2::vector_t twist = frame_jacobian.leftCols(n) * v;

  const bool has_message_weights =
    raw_target.size() == target_encoding::kEeMotionWeightedTargetDim ||
    raw_target.size() == target_encoding::kEeMotionWeightedTargetWithTwistFrameDim;
  const bool track_pose = raw_target.size() == target_encoding::kEeMotionPoseTargetDim ||
                          raw_target.size() == target_encoding::kEeMotionTargetDim ||
                          raw_target.size() == target_encoding::kEeMotionTargetWithTwistFrameDim ||
                          (has_message_weights &&
                           raw_target.segment(kEeMotionPoseWeightsOffset, kVector6Dim).cwiseAbs().maxCoeff() > 0.0);
  const bool track_twist = raw_target.size() == target_encoding::kEeMotionTwistOnlyTargetDim ||
                           raw_target.size() == target_encoding::kEeMotionTargetDim ||
                           raw_target.size() == target_encoding::kEeMotionTargetWithTwistFrameDim ||
                           (has_message_weights &&
                            raw_target.segment(kEeMotionTwistWeightsOffset, kVector6Dim).cwiseAbs().maxCoeff() > 0.0);

  if (track_pose) {
    const Eigen::Matrix<ocs2::scalar_t, 3, 1> desired_translation = raw_target.head<3>();
    const Eigen::Quaternion<ocs2::scalar_t> desired_orientation =
      quaternionFromTarget(raw_target, 3);
    target.residual.head<3>() = placement.translation() - desired_translation;
    target.residual.segment<3>(3) =
      ocs2::quaternionDistance<ocs2::scalar_t>(orientation, desired_orientation);
    target.weights.head(kVector6Dim) = has_message_weights ?
      raw_target.segment(kEeMotionPoseWeightsOffset, kVector6Dim).eval() :
      default_pose_weights_;
  }

  if (track_twist) {
    const ocs2::vector_t desired_twist =
      raw_target.size() == target_encoding::kEeMotionTwistOnlyTargetDim ?
      raw_target.segment(1, kVector6Dim).eval() :
      raw_target.segment(target_encoding::kEeMotionPoseTargetDim, kVector6Dim).eval();
    target.residual.tail(kVector6Dim) = twist - desired_twist;
    target.weights.tail(kVector6Dim) = has_message_weights ?
      raw_target.segment(kEeMotionTwistWeightsOffset, kVector6Dim).eval() :
      default_twist_weights_;
  }

  if ((target.weights.array() < 0.0).any()) {
    throw std::runtime_error("[EeMotionTrackingCost] target weights must be non-negative.");
  }
  target.weights *= weight_scale_;

  return target;
}

ocs2::scalar_t EeMotionTrackingCost::getValue(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::vector_t&,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::PreComputation&) const
{
  return getStateValue(time, state, targetTrajectories);
}

ocs2::ScalarFunctionQuadraticApproximation EeMotionTrackingCost::getQuadraticApproximation(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::vector_t& input,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::PreComputation&) const
{
  const auto state_cost = getStateQuadraticApproximation(time, state, targetTrajectories);

  ocs2::ScalarFunctionQuadraticApproximation cost;
  cost.setZero(state.size(), input.size());
  cost.f = state_cost.f;
  cost.dfdx = state_cost.dfdx;
  cost.dfdxx = state_cost.dfdxx;
  return cost;
}

ocs2::scalar_t EeMotionTrackingCost::getStateValue(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::TargetTrajectories& targetTrajectories) const
{
  const auto target = getTargetResidual(time, state, targetTrajectories);
  return 0.5 * target.residual.dot(target.weights.cwiseProduct(target.residual));
}

ocs2::ScalarFunctionQuadraticApproximation EeMotionTrackingCost::getStateQuadraticApproximation(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::TargetTrajectories& targetTrajectories) const
{
  const auto target = getTargetResidual(time, state, targetTrajectories);

  ocs2::ScalarFunctionQuadraticApproximation cost;
  cost.setZero(state.size());
  if (target.weights.cwiseAbs().maxCoeff() < 1.0e-12) {
    return cost;
  }

  ocs2::matrix_t residual_jacobian =
    ocs2::matrix_t::Zero(target.residual.size(), state.size());
  for (Eigen::Index i = 0; i < state.size(); ++i) {
    const ocs2::scalar_t step =
      1.0e-6 * std::max<ocs2::scalar_t>(1.0, std::abs(state(i)));
    ocs2::vector_t state_plus = state;
    ocs2::vector_t state_minus = state;
    state_plus(i) += step;
    state_minus(i) -= step;
    const auto plus = getTargetResidual(time, state_plus, targetTrajectories);
    const auto minus = getTargetResidual(time, state_minus, targetTrajectories);
    residual_jacobian.col(i) = (plus.residual - minus.residual) / (2.0 * step);
  }

  const ocs2::matrix_t weight = target.weights.asDiagonal();
  cost.f = 0.5 * target.residual.dot(target.weights.cwiseProduct(target.residual));
  cost.dfdx = residual_jacobian.transpose() * weight * target.residual;
  cost.dfdxx = residual_jacobian.transpose() * weight * residual_jacobian;
  return cost;
}

EeMotionTrackingTerminalCost::EeMotionTrackingTerminalCost(
  ocs2::PinocchioInterface pinocchioInterface,
  pinocchio::FrameIndex endEffectorFrameId,
  ocs2::vector_t defaultPoseWeights,
  ocs2::vector_t defaultTwistWeights,
  std::size_t jointDim,
  ocs2::scalar_t weightScale)
: tracking_cost_(
    std::move(pinocchioInterface),
    endEffectorFrameId,
    std::move(defaultPoseWeights),
    std::move(defaultTwistWeights),
    jointDim,
    weightScale)
{
}

ocs2::scalar_t EeMotionTrackingTerminalCost::getValue(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::PreComputation&) const
{
  return tracking_cost_.getStateValue(time, state, targetTrajectories);
}

ocs2::ScalarFunctionQuadraticApproximation EeMotionTrackingTerminalCost::getQuadraticApproximation(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::PreComputation&) const
{
  return tracking_cost_.getStateQuadraticApproximation(time, state, targetTrajectories);
}

}  // namespace cost
}  // namespace dynamics_mpc_controller
