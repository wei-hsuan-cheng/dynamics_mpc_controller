#ifndef DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_INTERFACE_HPP_
#define DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_INTERFACE_HPP_

#include <memory>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_sqp/SqpSettings.h>

#include "dynamics_mpc_controller/common/constraint/dynamics_self_collision_constraint.hpp"
#include "dynamics_mpc_controller/inverse_dynamics_mpc/inverse_dynamics_mpc_model.hpp"
#include "dynamics_mpc_controller/inverse_dynamics_mpc_controller_parameters.hpp"

namespace dynamics_mpc_controller
{

struct InverseDynamicsEvaluation
{
  ocs2::vector_t rnea_torque;
  ocs2::matrix_t ee_jacobian;
};

class InverseDynamicsMpcInterface final : public ocs2::RobotInterface
{
public:
  using Params = inverse_dynamics_mpc_controller::Params;

  explicit InverseDynamicsMpcInterface(const Params& parameters);

  ocs2::ddp::Settings& ddpSettings() { return ddp_settings_; }
  ocs2::mpc::Settings& mpcSettings() { return mpc_settings_; }
  ocs2::sqp::Settings& sqpSettings() { return sqp_settings_; }

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }
  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override;
  const ocs2::Initializer& getInitializer() const override { return *initializer_ptr_; }

  const ocs2::RolloutBase& getRollout() const { return *rollout_ptr_; }
  const ocs2::PinocchioInterface& getPinocchioInterface() const { return *pinocchio_interface_ptr_; }
  const InverseDynamicsMpcModel& getInverseDynamicsMpcModel() const { return inverse_dynamics_model_; }
  const std::string& defaultEeMotionTwistFrame() const { return default_ee_motion_twist_frame_; }
  bool inputLimitsActive() const { return input_limits_active_; }
  const ocs2::vector_t& inputLowerBounds() const { return input_lower_bounds_; }
  const ocs2::vector_t& inputUpperBounds() const { return input_upper_bounds_; }
  bool stateLimitsActive() const { return state_limits_active_; }
  const ocs2::vector_t& stateLowerBounds() const { return state_lower_bounds_; }
  const ocs2::vector_t& stateUpperBounds() const { return state_upper_bounds_; }
  bool selfCollisionHardStopActive() const { return self_collision_hard_stop_active_; }
  double selfCollisionHardStopDistance() const { return self_collision_hard_stop_distance_; }
  double computeMinimumSelfCollisionDistance(const ocs2::vector_t& q) const;

  InverseDynamicsEvaluation evaluateInverseDynamics(
    const ocs2::vector_t& q,
    const ocs2::vector_t& v,
    const ocs2::vector_t& a) const;

  ocs2::vector_t computeInverseDynamicsTorque(
    const ocs2::vector_t& q,
    const ocs2::vector_t& v,
    const ocs2::vector_t& a) const;

  ocs2::vector_t computeInverseDynamicsTorqueWithEeWrench(
    const ocs2::vector_t& q,
    const ocs2::vector_t& v,
    const ocs2::vector_t& a,
    const ocs2::vector_t& eeWrenchEnvOnRobot) const;

  ocs2::vector_t computeNonlinearEffects(
    const ocs2::vector_t& q,
    const ocs2::vector_t& v) const;

  ocs2::vector_t computeGravityCompensation(const ocs2::vector_t& q) const;

private:
  void loadSolverSettings(const Params& parameters);
  void setupPinocchio(const Params& parameters);
  void setupOptimalControlProblem(const Params& parameters);

  ocs2::ddp::Settings ddp_settings_;
  ocs2::mpc::Settings mpc_settings_;
  ocs2::sqp::Settings sqp_settings_;

  ocs2::OptimalControlProblem problem_;
  std::shared_ptr<ocs2::ReferenceManager> reference_manager_ptr_;
  std::unique_ptr<ocs2::RolloutBase> rollout_ptr_;
  std::unique_ptr<ocs2::Initializer> initializer_ptr_;
  std::unique_ptr<ocs2::PinocchioInterface> pinocchio_interface_ptr_;

  InverseDynamicsMpcModel inverse_dynamics_model_;
  std::string default_ee_motion_twist_frame_{"ee"};
  bool input_limits_active_{false};
  bool state_limits_active_{false};
  bool self_collision_hard_stop_active_{false};
  double self_collision_hard_stop_distance_{0.0};
  ocs2::vector_t input_lower_bounds_;
  ocs2::vector_t input_upper_bounds_;
  ocs2::vector_t state_lower_bounds_;
  ocs2::vector_t state_upper_bounds_;
  std::unique_ptr<constraint::DynamicsSelfCollisionDistanceEvaluator>
    self_collision_distance_evaluator_;
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_INTERFACE_HPP_
