#include "dynamics_mpc_controller/inverse_dynamics_mpc/inverse_dynamics_mpc_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <pluginlib/class_list_macros.hpp>

#include "dynamics_mpc_controller/common/controller_utils.hpp"
#include "dynamics_mpc_controller/diagnostics/mpc_policy_publisher.hpp"
#include "dynamics_mpc_controller/estimation/momentum_observer_wrench_estimator.hpp"
#include "dynamics_mpc_controller/inverse_dynamics_mpc/target/ee_motion_tracking_target.hpp"
#include "dynamics_mpc_controller/inverse_dynamics_mpc/target/joint_tracking_target.hpp"

namespace dynamics_mpc_controller
{
namespace
{

Eigen::VectorXd toEigenVector(const std::vector<double>& values)
{
  return Eigen::Map<const Eigen::VectorXd>(
    values.data(), static_cast<Eigen::Index>(values.size()));
}

Eigen::VectorXd toEigenVectorExact(
  const std::vector<double>& values,
  std::size_t expected_size,
  const std::string& parameter_name)
{
  if (values.size() != expected_size) {
    throw std::runtime_error(
      parameter_name + " must contain " + std::to_string(expected_size) +
      " values, got " + std::to_string(values.size()) + ".");
  }
  return toEigenVector(values);
}

}  // namespace

InverseDynamicsMpcController::~InverseDynamicsMpcController() = default;

controller_interface::CallbackReturn InverseDynamicsMpcController::on_init()
{
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    parameters_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[InverseDynamicsMpcController] init failed: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "[InverseDynamicsMpcController] init success");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InverseDynamicsMpcController::on_configure(
  const rclcpp_lifecycle::State&)
{
  if (param_listener_->is_old(parameters_)) {
    parameters_ = param_listener_->get_params();
  }

  if (!configure_mpc_data() || !configure_mpc_ocs2()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  const auto& model = interface_->getInverseDynamicsMpcModel();
  target_subscription_ = get_node()->create_subscription<TargetMsg>(
    mpc_data_.target_trajectories_topic_,
    rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<TargetMsg> msg) {
      received_target_msg_.writeFromNonRT(msg);
      latest_target_receive_time_sec_.writeFromNonRT(get_node()->get_clock()->now().seconds());
      target_received_ = true;
    });
  mpc_observation_publisher_ = get_node()->create_publisher<ocs2_msgs::msg::MpcObservation>(
    mpc_data_.mpc_observation_topic_,
    rclcpp::QoS(1));
  realtime_mpc_observation_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<ocs2_msgs::msg::MpcObservation>>(
      mpc_observation_publisher_);
  wrench_estimate_publisher_.reset();
  realtime_wrench_estimate_publisher_.reset();
  if (parameters_.wrenchEstimation.publish) {
    wrench_estimate_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      parameters_.topics.estimated_ee_wrench_topic,
      rclcpp::SensorDataQoS());
    realtime_wrench_estimate_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
        wrench_estimate_publisher_);
  }
  mpc_policy_publisher_.reset();
  if (parameters_.policyPublishing.publish) {
    mpc_policy_publisher_ =
      get_node()->create_publisher<ocs2_msgs::msg::MpcFlattenedController>(
        parameters_.topics.mpc_policy_topic,
        rclcpp::QoS(1));
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "[InverseDynamicsMpcController] configured | stateDim=%zu inputDim=%zu joints=%zu eeFrame=%s",
    model.stateDim(),
    model.inputDim(),
    model.jointDim(),
    model.endEffectorFrame().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

bool InverseDynamicsMpcController::configure_mpc_data()
{
  mpc_data_.lib_folder_ = parameters_.paths.libFolder;
  mpc_data_.urdf_file_ = parameters_.paths.urdfFile;
  mpc_data_.target_trajectories_topic_ = parameters_.topics.target_trajectories_topic;
  mpc_data_.mpc_observation_topic_ = parameters_.topics.mpc_observation_topic;
  mpc_data_.command_smoothing_alpha_ = parameters_.numeric.commandSmoothingAlpha;
  mpc_data_.gravity_compensation_only_ = parameters_.numeric.gravityCompensationOnly;
  mpc_data_.hold_velocity_damping_ = parameters_.numeric.holdVelocityDamping;
  status_log_period_ms_ = std::max<std::int64_t>(
    1,
    static_cast<std::int64_t>(std::llround(1000.0 * parameters_.numeric.statusLogPeriod)));
  solver_type_ = parameters_.ocs2.mpc.solverType;

  if (mpc_data_.lib_folder_.empty() || mpc_data_.urdf_file_.empty()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[InverseDynamicsMpcController] paths.libFolder or paths.urdfFile is empty.");
    return false;
  }

  std::error_code error;
  const auto lib_folder = std::filesystem::absolute(mpc_data_.lib_folder_, error);
  if (error) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[InverseDynamicsMpcController] failed to resolve CppAD library folder '%s': %s",
      mpc_data_.lib_folder_.c_str(),
      error.message().c_str());
    return false;
  }
  std::filesystem::create_directories(lib_folder, error);
  if (error) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[InverseDynamicsMpcController] failed to create CppAD library folder '%s': %s",
      lib_folder.c_str(),
      error.message().c_str());
    return false;
  }
  mpc_data_.lib_folder_ = lib_folder.lexically_normal().string();
  parameters_.paths.libFolder = mpc_data_.lib_folder_;
  RCLCPP_INFO(
    get_node()->get_logger(),
    "[InverseDynamicsMpcController] CppAD library folder: %s | recompileLibraries=%s",
    mpc_data_.lib_folder_.c_str(),
    parameters_.ocs2.task.model_settings.recompileLibraries ? "true" : "false");
  return true;
}

bool InverseDynamicsMpcController::configure_mpc_ocs2()
{
  try {
    interface_ = std::make_unique<InverseDynamicsMpcInterface>(parameters_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[InverseDynamicsMpcController] failed to create OCS2 interface: %s",
      e.what());
    return false;
  }

  const auto& model = interface_->getInverseDynamicsMpcModel();
  joint_tracking_target_ = std::make_unique<target::JointTrackingTarget>(*interface_);
  ee_motion_tracking_target_ = std::make_unique<target::EeMotionTrackingTarget>(*interface_);
  last_input_ = vector_t::Zero(static_cast<Eigen::Index>(model.inputDim()));
  last_tau_command_ = vector_t::Zero(static_cast<Eigen::Index>(model.jointDim()));
  low_level_pd_kp_ = vector_t::Zero(static_cast<Eigen::Index>(model.jointDim()));
  low_level_pd_kd_ = vector_t::Zero(static_cast<Eigen::Index>(model.jointDim()));
  estimated_ee_wrench_ = vector_t::Zero(6);

  low_level_pd_feedback_active_ = parameters_.lowLevelPDFeedback.activate;
  if (low_level_pd_feedback_active_) {
    try {
      low_level_pd_kp_ = toEigenVectorExact(
        parameters_.lowLevelPDFeedback.jointKp,
        model.jointDim(),
        "lowLevelPDFeedback.jointKp");
      low_level_pd_kd_ = toEigenVectorExact(
        parameters_.lowLevelPDFeedback.jointKd,
        model.jointDim(),
        "lowLevelPDFeedback.jointKd");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "[InverseDynamicsMpcController] failed to configure low-level PD feedback: %s",
        e.what());
      return false;
    }
    RCLCPP_INFO(
      get_node()->get_logger(),
      "[InverseDynamicsMpcController] low-level PD feedback is active; OCS2 feedback policy should remain disabled for inverse-dynamics MPC.");
  }

  wrench_estimator_.reset();
  if (parameters_.wrenchEstimation.activate) {
    try {
      estimation::MomentumObserverWrenchEstimatorSettings settings;
      settings.observer_gains = toEigenVector(parameters_.wrenchEstimation.observerGains);
      settings.velocity_filter_cutoff_hz = parameters_.wrenchEstimation.velocityFilterCutoffHz;
      settings.torque_bias = toEigenVector(parameters_.wrenchEstimation.torqueBias);
      settings.joint_torque_std_dev =
        toEigenVector(parameters_.wrenchEstimation.jointTorqueStdDev);
      settings.damping_min = parameters_.wrenchEstimation.dampingMin;
      settings.damping_high = parameters_.wrenchEstimation.dampingHigh;
      settings.sigma_min_threshold = parameters_.wrenchEstimation.sigmaMinThreshold;
      settings.relative_projection_error_threshold =
        parameters_.wrenchEstimation.relativeProjectionErrorThreshold;
      wrench_estimator_ = std::make_unique<estimation::MomentumObserverWrenchEstimator>(
        interface_->getPinocchioInterface().getModel(),
        model.endEffectorFrameId(),
        std::move(settings));
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "[InverseDynamicsMpcController] failed to configure wrench estimator: %s",
        e.what());
      return false;
    }
  }
  return true;
}

controller_interface::CallbackReturn InverseDynamicsMpcController::configure_hardware_interface()
{
  const auto logger = get_node()->get_logger();
  const std::string prefix = parameters_.robot.prefix;
  robot_hardware_interfaces_.clear();

  for (const auto& [body_joint_name, body_joint_params] : parameters_.robot.body_joint_names_map) {
    const auto [body_name, joint_name] = controller_utils::resolveJointName(body_joint_name, false);
    HWInterfaces joint_hw;

    for (const auto& interface_name : body_joint_params.state_interfaces) {
      const auto state_handle = std::find_if(
        state_interfaces_.begin(),
        state_interfaces_.end(),
        [&prefix, &body_name, &joint_name, &interface_name](const auto& interface) {
          return interface.get_prefix_name() == (prefix + "_" + body_name + "_" + joint_name) &&
                 interface.get_interface_name() == interface_name;
        });
      if (state_handle == state_interfaces_.end()) {
        RCLCPP_ERROR(
          logger,
          "[InverseDynamicsMpcController] missing state interface %s for %s_%s",
          interface_name.c_str(),
          body_name.c_str(),
          joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      joint_hw.state.emplace(interface_name, std::ref(*state_handle));
    }

    for (const auto& interface_name : body_joint_params.command_interfaces) {
      const auto command_handle = std::find_if(
        command_interfaces_.begin(),
        command_interfaces_.end(),
        [&prefix, &body_name, &joint_name, &interface_name](const auto& interface) {
          return interface.get_prefix_name() == (prefix + "_" + body_name + "_" + joint_name) &&
                 interface.get_interface_name() == interface_name;
        });
      if (command_handle == command_interfaces_.end()) {
        RCLCPP_ERROR(
          logger,
          "[InverseDynamicsMpcController] missing command interface %s for %s_%s",
          interface_name.c_str(),
          body_name.c_str(),
          joint_name.c_str());
        return controller_interface::CallbackReturn::ERROR;
      }
      joint_hw.command.emplace(interface_name, std::ref(*command_handle));
    }

    if (joint_hw.command.count(hardware_interface::HW_IF_EFFORT) == 0 ||
        joint_hw.state.count(hardware_interface::HW_IF_POSITION) == 0 ||
        joint_hw.state.count(hardware_interface::HW_IF_VELOCITY) == 0) {
      RCLCPP_ERROR(
        logger,
        "[InverseDynamicsMpcController] each joint requires effort command plus position and velocity states.");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (parameters_.wrenchEstimation.activate &&
        joint_hw.state.count(hardware_interface::HW_IF_EFFORT) == 0) {
      RCLCPP_ERROR(
        logger,
        "[InverseDynamicsMpcController] wrench estimation requires an effort state for every joint.");
      return controller_interface::CallbackReturn::ERROR;
    }

    robot_hardware_interfaces_[body_name].emplace(joint_name, std::move(joint_hw));
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InverseDynamicsMpcController::on_activate(
  const rclcpp_lifecycle::State&)
{
  const auto hardware_result = configure_hardware_interface();
  if (hardware_result != controller_interface::CallbackReturn::SUCCESS) {
    return hardware_result;
  }

  const auto& model = interface_->getInverseDynamicsMpcModel();
  const vector_t q = current_position_vector();
  const vector_t v = current_velocity_vector();
  estimated_ee_wrench_.setZero();
  wrench_publish_elapsed_ = 0.0;
  observer_residual_norm_ = 0.0;
  jacobian_sigma_min_ = 0.0;
  jacobian_condition_number_ = 0.0;
  relative_projection_error_ = 0.0;
  wrench_estimate_valid_ = false;
  if (wrench_estimator_) {
    try {
      wrench_estimator_->reset(q, v, current_effort_vector());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "[InverseDynamicsMpcController] failed to reset wrench estimator: %s",
        e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  const vector_t tau_nonlinear = interface_->computeNonlinearEffects(q, v);
  last_input_.setZero();
  last_input_.segment(
    static_cast<Eigen::Index>(model.tauOffset()),
    static_cast<Eigen::Index>(model.jointDim())) = tau_nonlinear;
  last_tau_command_ = tau_nonlinear;

  reference_manager_ = interface_->getReferenceManagerPtr();
  policy_performance_acceptable_ = false;
  reset_mpc_warm_start_requested_ = false;
  target_received_ = false;
  received_target_msg_.writeFromNonRT(std::shared_ptr<TargetMsg>{});
  latest_target_receive_time_sec_.writeFromNonRT(0.0);
  virtual_time_ = 0.0;
  const SystemObservation initial_observation = build_observation(virtual_time_);
  reference_manager_->setTargetTrajectories(
    joint_tracking_target_->fromObservation(initial_observation));

  // Hold robot
  apply_hold_command();

  if (mpc_data_.gravity_compensation_only_) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController] gravityCompensationOnly=true; bypassing MPC solver and commanding bounded inverse-dynamics hold.");
    RCLCPP_INFO(get_node()->get_logger(), "[InverseDynamicsMpcController] activated");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  if (solver_type_ == "ddp") {
    mpc_solver_ = std::make_unique<ocs2::GaussNewtonDDP_MPC>(
      interface_->mpcSettings(),
      interface_->ddpSettings(),
      interface_->getRollout(),
      interface_->getOptimalControlProblem(),
      interface_->getInitializer());
  } else if (solver_type_ == "sqp") {
    mpc_solver_ = std::make_unique<ocs2::SqpMpc>(
      interface_->mpcSettings(),
      interface_->sqpSettings(),
      interface_->getOptimalControlProblem(),
      interface_->getInitializer());
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[InverseDynamicsMpcController] unsupported solver type: %s",
      solver_type_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  mpc_solver_->getSolverPtr()->setReferenceManager(reference_manager_);
  mrt_interface_ = std::make_unique<ocs2::MPC_MRT_Interface>(*mpc_solver_);
  mrt_interface_->initRollout(&interface_->getRollout());

  mpc_policy_observer_.reset();
  if (mpc_policy_publisher_) {
    mpc_policy_observer_ = std::make_shared<diagnostics::MpcPolicyPublisher>(
      mpc_policy_publisher_,
      *mpc_solver_,
      get_node()->get_logger(),
      parameters_.policyPublishing.publishRate);
    mrt_interface_->addMrtObserver(mpc_policy_observer_);
  }

  mrt_interface_->setCurrentObservation(initial_observation);
  publish_mpc_observation(initial_observation);

  try {
    check_initializer(initial_observation);

    std::size_t attempts = 0;
    while (!mrt_interface_->initialPolicyReceived() && attempts < 20) {
      const auto solve_start = std::chrono::steady_clock::now();
      mrt_interface_->advanceMpc();
      ++attempts;
      const double solve_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - solve_start).count();
      const double target_frequency =
        static_cast<double>(parameters_.ocs2.mpc.mpcDesiredFrequency);
      RCLCPP_INFO(
        get_node()->get_logger(),
        "[InverseDynamicsMpcController][MPC_TIMING] initial advance | attempt=%zu, elapsed=%.3f ms, target period=%.3f ms, overrun=%s.",
        attempts,
        solve_time_ms,
        1000.0 / target_frequency,
        solve_time_ms * target_frequency > 1000.0 ? "true" : "false");
    }
  } catch (const std::exception& e) {
    // Hold robot
    apply_hold_command();
    execute_mpc_ = false;
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[InverseDynamicsMpcController] initial MPC solve failed; remaining in bounded hold mode: %s",
      e.what());
    RCLCPP_INFO(get_node()->get_logger(), "[InverseDynamicsMpcController] activated in hold fallback");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  if (!mrt_interface_->initialPolicyReceived()) {
    // Hold robot
    apply_hold_command();
    execute_mpc_ = false;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController] no initial MPC policy received; remaining in bounded hold mode.");
    RCLCPP_INFO(get_node()->get_logger(), "[InverseDynamicsMpcController] activated in hold fallback");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // Hold robot
  apply_hold_command();

  execute_mpc_ = true;
  mpc_thread_ = std::thread(&InverseDynamicsMpcController::mpc_loop, this);

  RCLCPP_INFO(get_node()->get_logger(), "[InverseDynamicsMpcController] activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn InverseDynamicsMpcController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
  execute_mpc_ = false;
  reset_mpc_warm_start_requested_ = false;
  target_received_ = false;
  received_target_msg_.writeFromNonRT(std::shared_ptr<TargetMsg>{});
  latest_target_receive_time_sec_.writeFromNonRT(0.0);
  if (mpc_thread_.joinable()) {
    mpc_thread_.join();
  }

  // Hold robot
  apply_hold_command();
  wrench_estimate_valid_ = false;
  wrench_publish_elapsed_ = 0.0;
  robot_hardware_interfaces_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "[InverseDynamicsMpcController] deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
InverseDynamicsMpcController::on_export_reference_interfaces()
{
  reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.emplace_back(
    std::string(get_node()->get_name()),
    std::string("dummy_inverse_dynamics_mpc/") + hardware_interface::HW_IF_EFFORT,
    reference_interfaces_.data());
  return reference_interfaces;
}

controller_interface::InterfaceConfiguration
InverseDynamicsMpcController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto& [body_joint_name, body_joint_params] : parameters_.robot.body_joint_names_map) {
    for (const auto& interface_type : body_joint_params.command_interfaces) {
      conf_names.push_back(parameters_.robot.prefix + "_" + body_joint_name + "/" + interface_type);
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::InterfaceConfiguration
InverseDynamicsMpcController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto& [body_joint_name, body_joint_params] : parameters_.robot.body_joint_names_map) {
    for (const auto& interface_type : body_joint_params.state_interfaces) {
      conf_names.push_back(parameters_.robot.prefix + "_" + body_joint_name + "/" + interface_type);
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
}

void InverseDynamicsMpcController::mpc_loop()
{
  using Clock = std::chrono::steady_clock;
  const double target_frequency =
    static_cast<double>(parameters_.ocs2.mpc.mpcDesiredFrequency);
  const double target_period_ms = 1000.0 / target_frequency;
  const auto log_period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(parameters_.numeric.statusLogPeriod));

  Clock::time_point previous_start{};
  Clock::time_point next_log = Clock::now() + log_period;
  double accumulated_period_ms = 0.0;
  double accumulated_advance_ms = 0.0;
  std::size_t period_samples = 0;
  std::size_t advance_samples = 0;

  while (execute_mpc_) {
    try {
      ocs2::executeAndSleep(
        [&]() {
          const auto iteration_start = Clock::now();
          if (previous_start != Clock::time_point{}) {
            const double period_ms = std::chrono::duration<double, std::milli>(
              iteration_start - previous_start).count();
            accumulated_period_ms += period_ms;
            ++period_samples;
          }
          previous_start = iteration_start;

          if (reset_mpc_warm_start_requested_.exchange(false)) {
            mrt_interface_->resetMpcNode(reference_manager_->getTargetTrajectories());
            policy_performance_acceptable_ = false;
            RCLCPP_WARN_THROTTLE(
              get_node()->get_logger(),
              *get_node()->get_clock(),
              status_log_period_ms_,
              "[InverseDynamicsMpcController] reset MPC warm start after rejected policy; next solve will use the inverse-dynamics initializer.");
          }

          mrt_interface_->advanceMpc();

          const auto iteration_end = Clock::now();
          const double advance_ms = std::chrono::duration<double, std::milli>(
            iteration_end - iteration_start).count();
          accumulated_advance_ms += advance_ms;
          ++advance_samples;

          if (iteration_end >= next_log) {
            const double average_period_ms = period_samples > 0 ?
              accumulated_period_ms / static_cast<double>(period_samples) : target_period_ms;
            const double average_advance_ms = advance_samples > 0 ?
              accumulated_advance_ms / static_cast<double>(advance_samples) : 0.0;
            const double actual_frequency = 1000.0 / average_period_ms;
            RCLCPP_INFO(
              get_node()->get_logger(),
              "[InverseDynamicsMpcController][MPC_TIMING] target=%.2f Hz, actual=%.2f Hz, latest advance=%.3f ms, average advance=%.3f ms, target period=%.3f ms, utilization=%.1f%%.",
              target_frequency,
              actual_frequency,
              advance_ms,
              average_advance_ms,
              target_period_ms,
              100.0 * average_advance_ms / target_period_ms);
            accumulated_period_ms = 0.0;
            accumulated_advance_ms = 0.0;
            period_samples = 0;
            advance_samples = 0;
            next_log = iteration_end + log_period;
          }
        },
        parameters_.ocs2.mpc.mpcDesiredFrequency);
    } catch (const std::exception& e) {
      execute_mpc_ = false;
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "[InverseDynamicsMpcController] MPC loop failed: %s",
        e.what());
    }
  }
}

controller_interface::return_type InverseDynamicsMpcController::update_reference_from_subscribers()
{
  if (mpc_data_.gravity_compensation_only_) {
    return controller_interface::return_type::OK;
  }

  auto target_msg_ptr_ptr = received_target_msg_.readFromRT();
  if (!(target_msg_ptr_ptr && *target_msg_ptr_ptr)) {
    return controller_interface::return_type::OK;
  }

  const double* latest_target_time_sec_ptr = latest_target_receive_time_sec_.readFromRT();
  const double latest_target_time_sec =
    latest_target_time_sec_ptr ? *latest_target_time_sec_ptr : 0.0;
  if (controller_utils::commandIsStale(
      target_received_.load(),
      get_node()->get_clock()->now().seconds(),
      latest_target_time_sec,
      parameters_.numeric.targetTimeout)) {
    return controller_interface::return_type::OK;
  }

  const auto& msg = *(*target_msg_ptr_ptr);
  if (!joint_tracking_target_->supports(msg) && !ee_motion_tracking_target_->supports(msg)) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController] supported command_type values are 'joint_position', 'joint_velocity', 'joint', 'ee_motion_pose', 'ee_motion_twist', and 'ee_motion', got '%s'.",
      msg.command_type.c_str());
    return controller_interface::return_type::OK;
  }

  try {
    if (joint_tracking_target_->supports(msg)) {
      reference_manager_->setTargetTrajectories(joint_tracking_target_->fromMessage(msg));
    } else {
      reference_manager_->setTargetTrajectories(ee_motion_tracking_target_->fromMessage(msg));
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "[InverseDynamicsMpcController] failed to process target: %s",
      e.what());
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type InverseDynamicsMpcController::update_and_write_commands(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  const double period_sec = period.seconds();
  update_wrench_estimate(period_sec);
  if (parameters_.wrenchEstimation.activate && parameters_.wrenchEstimation.publish &&
      realtime_wrench_estimate_publisher_ &&
      std::isfinite(period_sec) && period_sec > 0.0) {
    wrench_publish_elapsed_ += period_sec;
    const double publish_period = 1.0 / parameters_.wrenchEstimation.publishRate;
    if (wrench_publish_elapsed_ >= publish_period) {
      publish_wrench_estimate(time);
      wrench_publish_elapsed_ = std::fmod(wrench_publish_elapsed_, publish_period);
    }
  }

  const SystemObservation observation = build_observation(virtual_time_);
  publish_mpc_observation(observation);

  if (mpc_data_.gravity_compensation_only_) {
    // Hold robot
    apply_hold_command();
    RCLCPP_INFO_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController][HOLD_MODE] gravityCompensationOnly=true; applying bounded inverse-dynamics hold.");
    virtual_time_ += period.seconds();
    return controller_interface::return_type::OK;
  }

  if (!execute_mpc_) {
    // Hold robot
    apply_hold_command();
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController][HOLD_FALLBACK] MPC execution is disabled.");
    virtual_time_ += period.seconds();
    return controller_interface::return_type::OK;
  }

  // Check for stale target
  const double* latest_target_time_sec_ptr = latest_target_receive_time_sec_.readFromRT();
  const double latest_target_time_sec =
    latest_target_time_sec_ptr ? *latest_target_time_sec_ptr : 0.0;
  if (controller_utils::commandIsStale(
      target_received_.load(),
      time.seconds(),
      latest_target_time_sec,
      parameters_.numeric.targetTimeout)) {
    reference_manager_->setTargetTrajectories(joint_tracking_target_->fromObservation(observation));
    policy_performance_acceptable_ = false;
    reset_mpc_warm_start_requested_ = true;
    // Hold robot
    apply_hold_command();
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController][HOLD_FALLBACK] target command timed out: age=%.3f s, timeout=%.3f s.",
      time.seconds() - latest_target_time_sec,
      parameters_.numeric.targetTimeout);
    virtual_time_ += period.seconds();
    return controller_interface::return_type::OK;
  }

  mrt_interface_->setCurrentObservation(observation);

  const bool policy_updated = mrt_interface_->updatePolicy();
  if (policy_updated) {
    const auto& performance = mrt_interface_->getPerformanceIndices();
    const auto& validation = parameters_.numeric.policyValidation;
    policy_performance_acceptable_ = policy_performance_is_acceptable(performance);

    if (!policy_performance_acceptable_.load()) {
      if (validation.activate) {
        reset_mpc_warm_start_requested_ = true;
      }
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        status_log_period_ms_,
        "[InverseDynamicsMpcController] rejecting MPC policy: equalitySSE=%.6g (max %.6g), dynamicsSSE=%.6g (max %.6g), inequalitySSE=%.6g (max %.6g).",
        performance.equalityConstraintsSSE,
        validation.maxEqualityConstraintsSSE,
        performance.dynamicsViolationSSE,
        validation.maxDynamicsViolationSSE,
        performance.inequalityConstraintsSSE,
        validation.maxInequalityConstraintsSSE);
    }
  }

  if (!policy_performance_acceptable_.load()) {
    // Hold robot
    apply_hold_command();
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController][HOLD_FALLBACK] waiting for an acceptable MPC policy.");
    virtual_time_ += period.seconds();
    return controller_interface::return_type::OK;
  }

  const auto policy = mrt_interface_->getPolicy();
  if (policy.timeTrajectory_.empty()) {
    // Hold robot
    apply_hold_command();
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController][HOLD_FALLBACK] received an empty MPC policy.");
    virtual_time_ += period.seconds();
    return controller_interface::return_type::OK;
  }

  vector_t policy_state;
  vector_t policy_input;
  std::size_t planned_mode = 0;
  mrt_interface_->evaluatePolicy(
    observation.time,
    observation.state,
    policy_state,
    policy_input,
    planned_mode);

  const auto& model = interface_->getInverseDynamicsMpcModel();
  if (policy_state.size() != static_cast<Eigen::Index>(model.stateDim()) ||
      policy_input.size() != static_cast<Eigen::Index>(model.inputDim()) ||
      !policy_state.allFinite() || !policy_input.allFinite()) {
    // Hold robot
    apply_hold_command();
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController][HOLD_FALLBACK] sampled MPC policy has invalid dimensions or non-finite values.");
    virtual_time_ += period.seconds();
    return controller_interface::return_type::OK;
  }

  const vector_t command_input = compute_policy_command_input(observation, policy_state, policy_input);

  // Check input bounds and apply torque command
  double rnea_residual = 0.0;
  double input_bound_violation = 0.0;
  if (!policy_input_is_acceptable(
      observation, command_input, rnea_residual, input_bound_violation)) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController][HOLD_FALLBACK] rejecting MPC command | RNEA residual=%.6g Nm, input-bound violation=%.6g.",
      rnea_residual,
      input_bound_violation);
    if (parameters_.numeric.policyValidation.activate) {
      reset_mpc_warm_start_requested_ = true;
    }
    // Hold robot
    apply_hold_command();
  } else {
    apply_torque_command(command_input);
  }

  virtual_time_ += period.seconds();
  return controller_interface::return_type::OK;
}

InverseDynamicsMpcController::SystemObservation
InverseDynamicsMpcController::build_observation(double time_sec) const
{
  const auto& model = interface_->getInverseDynamicsMpcModel();
  SystemObservation observation;
  observation.time = time_sec;
  observation.state = vector_t::Zero(static_cast<Eigen::Index>(model.stateDim()));
  observation.input = last_input_;
  observation.mode = 0;

  observation.state.head(static_cast<Eigen::Index>(model.jointDim())) = current_position_vector();
  observation.state.tail(static_cast<Eigen::Index>(model.jointDim())) = current_velocity_vector();
  return observation;
}

void InverseDynamicsMpcController::check_initializer(const SystemObservation& observation)
{
  const auto& model = interface_->getInverseDynamicsMpcModel();
  std::unique_ptr<ocs2::Initializer> initializer(interface_->getInitializer().clone());
  vector_t input;
  vector_t next_state;
  initializer->compute(
    observation.time,
    observation.state,
    observation.time + parameters_.ocs2.task.sqp.dt,
    input,
    next_state);

  const vector_t numeric_expected_tau = model.hasEeWrenchInput() ?
    interface_->computeInverseDynamicsTorqueWithEeWrench(
      model.getQ(observation.state),
      model.getV(observation.state),
      model.getA(input),
      model.getWrench(input)) :
    interface_->computeInverseDynamicsTorque(
      model.getQ(observation.state),
      model.getV(observation.state),
      model.getA(input));
  const vector_t numeric_residual = numeric_expected_tau - model.getTau(input);
  const auto& problem = interface_->getOptimalControlProblem();
  const auto residual_terms = problem.equalityConstraintPtr->getValue(
    observation.time,
    observation.state,
    input,
    *problem.preComputationPtr);
  if (residual_terms.empty()) {
    throw std::runtime_error("inverse-dynamics equality constraint is missing");
  }

  double bound_violation = 0.0;
  if (interface_->inputLimitsActive()) {
    bound_violation = controller_utils::maxBoundViolation(
      input,
      interface_->inputLowerBounds(),
      interface_->inputUpperBounds());
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "[InverseDynamicsMpcController] initializer check | numeric RNEA residual=%.3e, CppAD residual=%.3e, input-bound violation=%.3e",
    numeric_residual.lpNorm<Eigen::Infinity>(),
    residual_terms.front().lpNorm<Eigen::Infinity>(),
    bound_violation);
}

// Hold robot in its current position using a bounded RNEA nonlinear torque command
void InverseDynamicsMpcController::apply_hold_command()
{
  const auto& model = interface_->getInverseDynamicsMpcModel();
  const std::size_t n = model.jointDim();
  const vector_t q = current_position_vector();
  const vector_t v = current_velocity_vector();
  vector_t a_hold = vector_t::Zero(static_cast<Eigen::Index>(n));

  for (std::size_t i = 0; i < n && i < mpc_data_.hold_velocity_damping_.size(); ++i) {
    a_hold(static_cast<Eigen::Index>(i)) =
      -mpc_data_.hold_velocity_damping_[i] * v(static_cast<Eigen::Index>(i));
  }

  if (interface_->inputLimitsActive()) {
    a_hold = a_hold.cwiseMax(
      interface_->inputLowerBounds().segment(
        static_cast<Eigen::Index>(model.aOffset()),
        static_cast<Eigen::Index>(n)));
    a_hold = a_hold.cwiseMin(
      interface_->inputUpperBounds().segment(
        static_cast<Eigen::Index>(model.aOffset()),
        static_cast<Eigen::Index>(n)));
  }

  vector_t tau = interface_->computeInverseDynamicsTorque(
    q,
    v,
    a_hold);

  if (interface_->inputLimitsActive()) {
    tau = tau.cwiseMax(
      interface_->inputLowerBounds().segment(
        static_cast<Eigen::Index>(model.tauOffset()),
        static_cast<Eigen::Index>(n)));
    tau = tau.cwiseMin(
      interface_->inputUpperBounds().segment(
        static_cast<Eigen::Index>(model.tauOffset()),
        static_cast<Eigen::Index>(n)));
  }

  write_torque_command(tau);

  last_input_.setZero();
  last_input_.segment(
    static_cast<Eigen::Index>(model.aOffset()),
    static_cast<Eigen::Index>(n)) = a_hold;
  last_input_.segment(
    static_cast<Eigen::Index>(model.tauOffset()),
    static_cast<Eigen::Index>(n)) = tau;
  last_tau_command_ = tau;
}

InverseDynamicsMpcController::vector_t InverseDynamicsMpcController::compute_policy_command_input(
  const SystemObservation& observation,
  const vector_t& policy_state,
  const vector_t& policy_input) const
{
  const auto& model = interface_->getInverseDynamicsMpcModel();
  const std::size_t n = model.jointDim();
  vector_t command_input = policy_input;

  // Additional low-level PD feedback gains for torque compensation
  if (low_level_pd_feedback_active_) {
    const vector_t q = model.getQ(observation.state);
    const vector_t v = model.getV(observation.state);
    const vector_t q_nominal = model.getQ(policy_state);
    const vector_t v_nominal = model.getV(policy_state);

    vector_t acceleration_command =
      model.getA(policy_input) +
      low_level_pd_kp_.cwiseProduct(q_nominal - q) +
      low_level_pd_kd_.cwiseProduct(v_nominal - v);

    // Acceleration is a virtual inverse-dynamics input. Keep the feedback
    // correction intact and enforce the physical torque bounds below.
    command_input.segment(
      static_cast<Eigen::Index>(model.aOffset()),
      static_cast<Eigen::Index>(n)) = acceleration_command;

    const vector_t tau_command = model.hasEeWrenchInput() ?
      interface_->computeInverseDynamicsTorqueWithEeWrench(
        q,
        v,
        acceleration_command,
        model.getWrench(command_input)) :
      interface_->computeInverseDynamicsTorque(
        q,
        v,
        acceleration_command);

    command_input.segment(
      static_cast<Eigen::Index>(model.tauOffset()),
      static_cast<Eigen::Index>(n)) = tau_command;
  }

  vector_t tau = command_input.segment(
    static_cast<Eigen::Index>(model.tauOffset()),
    static_cast<Eigen::Index>(n));
  if (last_tau_command_.size() == tau.size()) {
    const double alpha = mpc_data_.command_smoothing_alpha_;
    tau = alpha * tau + (1.0 - alpha) * last_tau_command_;
  }
  if (interface_->inputLimitsActive()) {
    const auto tau_offset = static_cast<Eigen::Index>(model.tauOffset());
    const auto tau_dim = static_cast<Eigen::Index>(n);
    tau = tau.cwiseMax(interface_->inputLowerBounds().segment(tau_offset, tau_dim));
    tau = tau.cwiseMin(interface_->inputUpperBounds().segment(tau_offset, tau_dim));
  }
  command_input.segment(
    static_cast<Eigen::Index>(model.tauOffset()),
    static_cast<Eigen::Index>(n)) = tau;

  return command_input;
}

void InverseDynamicsMpcController::apply_torque_command(const vector_t& command_input)
{
  const auto& model = interface_->getInverseDynamicsMpcModel();
  const std::size_t n = model.jointDim();

  const vector_t tau = command_input.segment(
    static_cast<Eigen::Index>(model.tauOffset()),
    static_cast<Eigen::Index>(n));
  write_torque_command(tau);

  last_input_ = command_input;
  last_tau_command_ = tau;
}

void InverseDynamicsMpcController::write_torque_command(const vector_t& torque)
{
  controller_utils::writeTorqueCommand(
    interface_->getInverseDynamicsMpcModel().dofNames(),
    robot_hardware_interfaces_,
    torque);
}

bool InverseDynamicsMpcController::policy_input_is_acceptable(
  const SystemObservation& observation,
  const vector_t& command_input,
  double& rnea_residual,
  double& input_bound_violation) const
{
  rnea_residual = std::numeric_limits<double>::infinity();
  input_bound_violation = std::numeric_limits<double>::infinity();

  const auto& model = interface_->getInverseDynamicsMpcModel();
  if (observation.state.size() != static_cast<Eigen::Index>(model.stateDim()) ||
      command_input.size() != static_cast<Eigen::Index>(model.inputDim()) ||
      !observation.state.allFinite() || !command_input.allFinite()) {
    return false;
  }

  const vector_t commanded_tau = model.getTau(command_input);

  const vector_t expected_tau = model.hasEeWrenchInput() ?
    interface_->computeInverseDynamicsTorqueWithEeWrench(
      model.getQ(observation.state),
      model.getV(observation.state),
      model.getA(command_input),
      model.getWrench(command_input)) :
    interface_->computeInverseDynamicsTorque(
      model.getQ(observation.state),
      model.getV(observation.state),
      model.getA(command_input));
  rnea_residual = (expected_tau - commanded_tau).lpNorm<Eigen::Infinity>();
  if (!std::isfinite(rnea_residual)) {
    return false;
  }

  input_bound_violation = 0.0;
  if (interface_->inputLimitsActive()) {
    const auto tau_offset = static_cast<Eigen::Index>(model.tauOffset());
    const auto tau_dim = static_cast<Eigen::Index>(model.jointDim());
    input_bound_violation = controller_utils::maxBoundViolation(
      commanded_tau,
      interface_->inputLowerBounds().segment(tau_offset, tau_dim),
      interface_->inputUpperBounds().segment(tau_offset, tau_dim));
  }

  const auto& validation = parameters_.numeric.policyValidation;
  return !validation.activate ||
    (rnea_residual <= validation.maxRneaResidual &&
    input_bound_violation <= validation.inputBoundTolerance);
}

bool InverseDynamicsMpcController::policy_performance_is_acceptable(
  const ocs2::PerformanceIndex& performance) const
{
  const auto& validation = parameters_.numeric.policyValidation;
  return !validation.activate ||
    (std::isfinite(performance.cost) &&
    std::isfinite(performance.equalityConstraintsSSE) &&
    std::isfinite(performance.dynamicsViolationSSE) &&
    std::isfinite(performance.inequalityConstraintsSSE) &&
    performance.equalityConstraintsSSE <= validation.maxEqualityConstraintsSSE &&
    performance.dynamicsViolationSSE <= validation.maxDynamicsViolationSSE &&
    performance.inequalityConstraintsSSE <= validation.maxInequalityConstraintsSSE);
}

void InverseDynamicsMpcController::update_wrench_estimate(double period_sec)
{
  wrench_estimate_valid_ = false;
  if (!parameters_.wrenchEstimation.activate || !wrench_estimator_) {
    return;
  }

  try {
    const auto& estimate = wrench_estimator_->update(
      current_position_vector(),
      current_velocity_vector(),
      current_effort_vector(),
      period_sec);
    estimated_ee_wrench_ = estimate.wrench;
    observer_residual_norm_ = estimate.observer_residual_norm;
    jacobian_sigma_min_ = estimate.jacobian_sigma_min;
    jacobian_condition_number_ = estimate.jacobian_condition_number;
    relative_projection_error_ = estimate.relative_projection_error;
    wrench_estimate_valid_ = estimate.valid;

    if (!wrench_estimate_valid_) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        status_log_period_ms_,
        "[InverseDynamicsMpcController] wrench estimate invalid | residual=%.3e Nm, sigmaMin=%.3e, condition=%.3e, relativeProjectionError=%.3e.",
        observer_residual_norm_,
        jacobian_sigma_min_,
        jacobian_condition_number_,
        relative_projection_error_);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      status_log_period_ms_,
      "[InverseDynamicsMpcController] end-effector wrench estimation failed: %s",
      e.what());
  }
}

void InverseDynamicsMpcController::publish_mpc_observation(const SystemObservation& observation)
{
  controller_utils::publishMpcObservation(observation, realtime_mpc_observation_publisher_);
}

void InverseDynamicsMpcController::publish_wrench_estimate(const rclcpp::Time& stamp)
{
  if (!wrench_estimate_valid_ || !realtime_wrench_estimate_publisher_) {
    return;
  }

  controller_utils::publishWrenchEstimate(
    stamp,
    parameters_.ocs2.task.model_information.endEffectorFrame,
    estimated_ee_wrench_,
    realtime_wrench_estimate_publisher_);
}

InverseDynamicsMpcController::vector_t InverseDynamicsMpcController::current_position_vector() const
{
  return controller_utils::readStateVector(
    interface_->getInverseDynamicsMpcModel().dofNames(),
    robot_hardware_interfaces_,
    hardware_interface::HW_IF_POSITION);
}

InverseDynamicsMpcController::vector_t InverseDynamicsMpcController::current_velocity_vector() const
{
  return controller_utils::readStateVector(
    interface_->getInverseDynamicsMpcModel().dofNames(),
    robot_hardware_interfaces_,
    hardware_interface::HW_IF_VELOCITY);
}

InverseDynamicsMpcController::vector_t InverseDynamicsMpcController::current_effort_vector() const
{
  return controller_utils::readStateVector(
    interface_->getInverseDynamicsMpcModel().dofNames(),
    robot_hardware_interfaces_,
    hardware_interface::HW_IF_EFFORT);
}

}  // namespace dynamics_mpc_controller

PLUGINLIB_EXPORT_CLASS(
  dynamics_mpc_controller::InverseDynamicsMpcController,
  controller_interface::ChainableControllerInterface)
