#ifndef DYNAMICS_MPC_CONTROLLER__ESTIMATION__MOMENTUM_OBSERVER_WRENCH_ESTIMATOR_HPP_
#define DYNAMICS_MPC_CONTROLLER__ESTIMATION__MOMENTUM_OBSERVER_WRENCH_ESTIMATOR_HPP_

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace dynamics_mpc_controller::estimation
{

struct MomentumObserverWrenchEstimatorSettings
{
  Eigen::VectorXd observer_gains;
  double velocity_filter_cutoff_hz{20.0};
  Eigen::VectorXd torque_bias;
  Eigen::VectorXd joint_torque_std_dev;
  double damping_min{0.01};
  double damping_high{0.5};
  double sigma_min_threshold{0.03};
  double relative_projection_error_threshold{0.3};
};

struct MomentumObserverWrenchEstimate
{
  Eigen::VectorXd external_joint_torque;
  Eigen::Matrix<double, 6, 1> wrench{Eigen::Matrix<double, 6, 1>::Zero()};
  double observer_residual_norm{0.0};
  double jacobian_sigma_min{0.0};
  double jacobian_condition_number{0.0};
  double projection_error{0.0};
  double relative_projection_error{0.0};
  bool valid{false};
};

class MomentumObserverWrenchEstimator
{
public:
  MomentumObserverWrenchEstimator(
    const pinocchio::Model& model,
    pinocchio::FrameIndex endEffectorFrameId,
    MomentumObserverWrenchEstimatorSettings settings);

  void reset(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const Eigen::VectorXd& measuredTorque);

  const MomentumObserverWrenchEstimate& update(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const Eigen::VectorXd& measuredTorque,
    double dt);

  const MomentumObserverWrenchEstimate& estimate() const { return estimate_; }

private:
  void validateSettings() const;
  void validateSample(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& v,
    const Eigen::VectorXd& measuredTorque) const;
  void updateModelTerms(const Eigen::VectorXd& q);
  void updateWrenchEstimate();

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::FrameIndex end_effector_frame_id_;
  MomentumObserverWrenchEstimatorSettings settings_;

  bool initialized_{false};
  Eigen::VectorXd filtered_velocity_;
  Eigen::VectorXd generalized_momentum_;
  Eigen::VectorXd beta_;
  Eigen::VectorXd integral_state_;
  Eigen::VectorXd residual_;
  Eigen::VectorXd previous_integrand_;
  Eigen::VectorXd inverse_torque_variance_;
  Eigen::VectorXd predicted_residual_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> ee_jacobian_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> weighted_jacobian_;
  MomentumObserverWrenchEstimate estimate_;
};

}  // namespace dynamics_mpc_controller::estimation

#endif  // DYNAMICS_MPC_CONTROLLER__ESTIMATION__MOMENTUM_OBSERVER_WRENCH_ESTIMATOR_HPP_
