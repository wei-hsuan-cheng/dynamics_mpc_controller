#include "dynamics_mpc_controller/estimation/momentum_observer_wrench_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>

namespace dynamics_mpc_controller::estimation
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kNumericalEpsilon = 1.0e-12;

}  // namespace

MomentumObserverWrenchEstimator::MomentumObserverWrenchEstimator(
  const pinocchio::Model& model,
  pinocchio::FrameIndex endEffectorFrameId,
  MomentumObserverWrenchEstimatorSettings settings)
: model_(model),
  data_(model_),
  end_effector_frame_id_(endEffectorFrameId),
  settings_(std::move(settings)),
  filtered_velocity_(Eigen::VectorXd::Zero(model_.nv)),
  generalized_momentum_(Eigen::VectorXd::Zero(model_.nv)),
  beta_(Eigen::VectorXd::Zero(model_.nv)),
  integral_state_(Eigen::VectorXd::Zero(model_.nv)),
  residual_(Eigen::VectorXd::Zero(model_.nv)),
  previous_integrand_(Eigen::VectorXd::Zero(model_.nv)),
  inverse_torque_variance_(Eigen::VectorXd::Zero(model_.nv)),
  predicted_residual_(Eigen::VectorXd::Zero(model_.nv)),
  ee_jacobian_(6, model_.nv),
  weighted_jacobian_(6, model_.nv)
{
  validateSettings();
  if (end_effector_frame_id_ >= model_.frames.size()) {
    throw std::invalid_argument("Momentum observer end-effector frame ID is invalid.");
  }

  inverse_torque_variance_ = settings_.joint_torque_std_dev.array().square().inverse();
  ee_jacobian_.setZero();
  weighted_jacobian_.setZero();
  estimate_.external_joint_torque = Eigen::VectorXd::Zero(model_.nv);
}

void MomentumObserverWrenchEstimator::validateSettings() const
{
  if (settings_.observer_gains.size() != model_.nv ||
      settings_.torque_bias.size() != model_.nv ||
      settings_.joint_torque_std_dev.size() != model_.nv) {
    throw std::invalid_argument(
      "Momentum observer gains, torque bias, and torque standard deviation must match model.nv.");
  }
  if (!settings_.observer_gains.allFinite() ||
      (settings_.observer_gains.array() <= 0.0).any() ||
      !settings_.torque_bias.allFinite() ||
      !settings_.joint_torque_std_dev.allFinite() ||
      (settings_.joint_torque_std_dev.array() <= 0.0).any() ||
      !std::isfinite(settings_.velocity_filter_cutoff_hz) ||
      settings_.velocity_filter_cutoff_hz <= 0.0 ||
      !std::isfinite(settings_.damping_min) || settings_.damping_min <= 0.0 ||
      !std::isfinite(settings_.damping_high) ||
      settings_.damping_high < settings_.damping_min ||
      !std::isfinite(settings_.sigma_min_threshold) ||
      settings_.sigma_min_threshold <= 0.0 ||
      !std::isfinite(settings_.relative_projection_error_threshold) ||
      settings_.relative_projection_error_threshold < 0.0) {
    throw std::invalid_argument("Momentum observer settings are invalid.");
  }
}

void MomentumObserverWrenchEstimator::validateSample(
  const Eigen::VectorXd& q,
  const Eigen::VectorXd& v,
  const Eigen::VectorXd& measuredTorque) const
{
  if (q.size() != model_.nq || v.size() != model_.nv || measuredTorque.size() != model_.nv) {
    throw std::invalid_argument("Momentum observer sample dimensions do not match the model.");
  }
  if (!q.allFinite() || !v.allFinite() || !measuredTorque.allFinite()) {
    throw std::invalid_argument("Momentum observer sample contains nonfinite values.");
  }
}

void MomentumObserverWrenchEstimator::updateModelTerms(const Eigen::VectorXd& q)
{
  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  generalized_momentum_.noalias() = data_.M * filtered_velocity_;

  const Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(model_, data_, q);
  pinocchio::computeCoriolisMatrix(model_, data_, q, filtered_velocity_);
  beta_.noalias() = gravity - data_.C.transpose() * filtered_velocity_;

  ee_jacobian_.setZero();
  pinocchio::computeFrameJacobian(
    model_, data_, q, end_effector_frame_id_, pinocchio::LOCAL, ee_jacobian_);
}

void MomentumObserverWrenchEstimator::reset(
  const Eigen::VectorXd& q,
  const Eigen::VectorXd& v,
  const Eigen::VectorXd& measuredTorque)
{
  validateSample(q, v, measuredTorque);
  filtered_velocity_ = v;
  updateModelTerms(q);

  integral_state_ = generalized_momentum_;
  residual_.setZero();
  previous_integrand_ = measuredTorque - settings_.torque_bias - beta_;
  estimate_.external_joint_torque.setZero();
  estimate_.wrench.setZero();
  estimate_.observer_residual_norm = 0.0;
  estimate_.valid = false;
  initialized_ = true;
}

const MomentumObserverWrenchEstimate& MomentumObserverWrenchEstimator::update(
  const Eigen::VectorXd& q,
  const Eigen::VectorXd& v,
  const Eigen::VectorXd& measuredTorque,
  double dt)
{
  validateSample(q, v, measuredTorque);
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("Momentum observer update period must be finite and positive.");
  }
  if (!initialized_) {
    reset(q, v, measuredTorque);
    return estimate_;
  }

  const double velocity_alpha =
    1.0 - std::exp(-kTwoPi * settings_.velocity_filter_cutoff_hz * dt);
  filtered_velocity_ += velocity_alpha * (v - filtered_velocity_);
  updateModelTerms(q);

  const Eigen::VectorXd integrand =
    measuredTorque - settings_.torque_bias - beta_ + residual_;
  integral_state_ += 0.5 * dt * (integrand + previous_integrand_);
  previous_integrand_ = integrand;
  residual_ = settings_.observer_gains.asDiagonal() * (generalized_momentum_ - integral_state_);

  updateWrenchEstimate();
  return estimate_;
}

void MomentumObserverWrenchEstimator::updateWrenchEstimate()
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(ee_jacobian_);
  const Eigen::VectorXd singular_values = svd.singularValues();
  const double sigma_max = singular_values.size() > 0 ? singular_values.maxCoeff() : 0.0;
  const double sigma_min = singular_values.size() > 0 ? singular_values.minCoeff() : 0.0;
  const double condition_number = sigma_max / std::max(sigma_min, kNumericalEpsilon);
  const double damping = sigma_min < settings_.sigma_min_threshold ?
    settings_.damping_high : settings_.damping_min;

  weighted_jacobian_ = ee_jacobian_ * inverse_torque_variance_.asDiagonal();
  Eigen::Matrix<double, 6, 6> lhs = weighted_jacobian_ * ee_jacobian_.transpose();
  lhs.diagonal().array() += damping * damping;
  const Eigen::Matrix<double, 6, 1> rhs =
    ee_jacobian_ * (inverse_torque_variance_.array() * residual_.array()).matrix();

  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(lhs);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Failed to factor the momentum-observer wrench system.");
  }
  estimate_.wrench = solver.solve(rhs);
  if (solver.info() != Eigen::Success || !estimate_.wrench.allFinite()) {
    throw std::runtime_error("Failed to calculate a finite momentum-observer wrench.");
  }

  predicted_residual_.noalias() = ee_jacobian_.transpose() * estimate_.wrench;
  const double residual_norm = residual_.norm();
  const double projection_error = (residual_ - predicted_residual_).norm();
  const double relative_projection_error =
    projection_error / std::max(residual_norm, 1.0e-9);

  estimate_.external_joint_torque = residual_;
  estimate_.observer_residual_norm = residual_norm;
  estimate_.jacobian_sigma_min = sigma_min;
  estimate_.jacobian_condition_number = condition_number;
  estimate_.projection_error = projection_error;
  estimate_.relative_projection_error = relative_projection_error;
  estimate_.valid = residual_.allFinite() && sigma_min > kNumericalEpsilon &&
    std::isfinite(condition_number) &&
    relative_projection_error <= settings_.relative_projection_error_threshold;
}

}  // namespace dynamics_mpc_controller::estimation
