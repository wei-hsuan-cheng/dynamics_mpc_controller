#ifndef DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_CONTROLLER_HPP_
#define DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_CONTROLLER_HPP_

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <ocs2_msgs/msg/mpc_targets.hpp>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_sqp/SqpMpc.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "dynamics_mpc_controller/inverse_dynamics_mpc/inverse_dynamics_mpc_interface.hpp"
#include "dynamics_mpc_controller/inverse_dynamics_mpc_controller_parameters.hpp"
#include "dynamics_mpc_controller/mpc_data.hpp"

namespace dynamics_mpc_controller
{

namespace estimation
{
class MomentumObserverWrenchEstimator;
}

namespace diagnostics
{
class MpcPolicyPublisher;
}

namespace reference
{
class JointTrackingTarget;
}

class InverseDynamicsMpcController : public controller_interface::ChainableControllerInterface
{
public:
  using Params = inverse_dynamics_mpc_controller::Params;
  using ParamListener = inverse_dynamics_mpc_controller::ParamListener;
  using TargetMsg = ocs2_msgs::msg::MpcTargets;

  InverseDynamicsMpcController() = default;
  ~InverseDynamicsMpcController() override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

private:
  using vector_t = ocs2::vector_t;
  using SystemObservation = ocs2::SystemObservation;

  bool configure_mpc_data();
  bool configure_mpc_ocs2();
  controller_interface::CallbackReturn configure_hardware_interface();
  SystemObservation build_observation(double time_sec) const;
  void check_initializer(const SystemObservation& observation);
  void apply_hold_command();
  void apply_torque_command(const vector_t& policy_input);
  void write_torque_command(const vector_t& torque);
  bool policy_input_is_acceptable(
    const SystemObservation& observation,
    const vector_t& policy_input,
    double& rnea_residual,
    double& input_bound_violation) const;
  bool policy_performance_is_acceptable(const ocs2::PerformanceIndex& performance) const;
  void update_wrench_estimate(double period_sec);
  void publish_wrench_estimate(const rclcpp::Time& stamp);
  void mpc_loop();

  vector_t current_position_vector() const;
  vector_t current_velocity_vector() const;
  vector_t current_effort_vector() const;
  std::pair<std::string, std::string> resolve_name(const std::string& name, bool has_prefix) const;

  std::shared_ptr<ParamListener> param_listener_;
  Params parameters_;

  MPCData mpc_data_;

  std::unique_ptr<InverseDynamicsMpcInterface> interface_;
  std::shared_ptr<ocs2::ReferenceManagerInterface> reference_manager_;
  std::unique_ptr<ocs2::MPC_BASE> mpc_solver_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt_interface_;
  std::unique_ptr<estimation::MomentumObserverWrenchEstimator> wrench_estimator_;
  std::unique_ptr<reference::JointTrackingTarget> joint_tracking_target_;
  std::shared_ptr<diagnostics::MpcPolicyPublisher> mpc_policy_observer_;

  double virtual_time_{0.0};
  std::int64_t status_log_period_ms_{2000};
  std::string solver_type_{"ddp"};
  vector_t last_input_;
  vector_t last_tau_command_;
  vector_t estimated_ee_wrench_;
  double wrench_publish_elapsed_{0.0};
  double observer_residual_norm_{0.0};
  double jacobian_sigma_min_{0.0};
  double jacobian_condition_number_{0.0};
  double relative_projection_error_{0.0};
  bool wrench_estimate_valid_{false};

  std::thread mpc_thread_;
  std::atomic_bool execute_mpc_{false};
  std::atomic_bool policy_performance_acceptable_{false};
  std::atomic_bool reset_mpc_warm_start_requested_{false};

  realtime_tools::RealtimeBuffer<std::shared_ptr<TargetMsg>> received_target_msg_;
  rclcpp::Subscription<TargetMsg>::SharedPtr target_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_estimate_publisher_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcFlattenedController>::SharedPtr mpc_policy_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>
    realtime_wrench_estimate_publisher_;

  std::map<std::string, std::map<std::string, HWInterfaces>> robot_hardware_interfaces_;
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_CONTROLLER_HPP_
