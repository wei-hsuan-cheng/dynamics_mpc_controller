#ifndef DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__FORWARD_DYNAMICS_MPC_INTERFACE_HPP_
#define DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__FORWARD_DYNAMICS_MPC_INTERFACE_HPP_

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

#include "dynamics_mpc_controller/forward_dynamics_mpc/forward_dynamics_mpc_model.hpp"
#include "dynamics_mpc_controller/forward_dynamics_mpc_controller_parameters.hpp"

namespace dynamics_mpc_controller
{

class ForwardDynamicsMpcInterface final : public ocs2::RobotInterface
{
public:
  using Params = forward_dynamics_mpc_controller::Params;

  explicit ForwardDynamicsMpcInterface(const Params& parameters);

  ocs2::ddp::Settings& ddpSettings() { return ddp_settings_; }
  ocs2::mpc::Settings& mpcSettings() { return mpc_settings_; }
  ocs2::sqp::Settings& sqpSettings() { return sqp_settings_; }

  const ocs2::OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }
  std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override;
  const ocs2::Initializer& getInitializer() const override { return *initializer_ptr_; }

  const ocs2::RolloutBase& getRollout() const { return *rollout_ptr_; }
  const ocs2::PinocchioInterface& getPinocchioInterface() const { return *pinocchio_interface_ptr_; }
  const ForwardDynamicsMpcModel& getForwardDynamicsMpcModel() const { return forward_dynamics_model_; }

  bool inputLimitsActive() const { return input_limits_active_; }
  const ocs2::vector_t& inputLowerBounds() const { return input_lower_bounds_; }
  const ocs2::vector_t& inputUpperBounds() const { return input_upper_bounds_; }

  bool stateLimitsActive() const { return state_limits_active_; }
  const ocs2::vector_t& stateLowerBounds() const { return state_lower_bounds_; }
  const ocs2::vector_t& stateUpperBounds() const { return state_upper_bounds_; }

  ocs2::vector_t computeForwardDynamics(
    const ocs2::vector_t& q,
    const ocs2::vector_t& v,
    const ocs2::vector_t& tau) const;

  ocs2::vector_t computeRneaTorque(
    const ocs2::vector_t& q,
    const ocs2::vector_t& v,
    const ocs2::vector_t& a) const;

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

  ForwardDynamicsMpcModel forward_dynamics_model_;
  bool input_limits_active_{false};
  bool state_limits_active_{false};
  ocs2::vector_t input_lower_bounds_;
  ocs2::vector_t input_upper_bounds_;
  ocs2::vector_t state_lower_bounds_;
  ocs2::vector_t state_upper_bounds_;
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__FORWARD_DYNAMICS_MPC_INTERFACE_HPP_
