#include "dynamics_mpc_controller/diagnostics/mpc_policy_publisher.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <rclcpp/logging.hpp>

namespace dynamics_mpc_controller::diagnostics
{

MpcPolicyPublisher::MpcPolicyPublisher(
  Publisher::SharedPtr publisher,
  const ocs2::MPC_BASE& mpc,
  rclcpp::Logger logger,
  double publishRate)
: publisher_(std::move(publisher)),
  mpc_(mpc),
  logger_(std::move(logger)),
  publish_period_(Clock::duration::zero())
{
  if (!publisher_) {
    throw std::invalid_argument("MPC policy publisher must not be null.");
  }
  if (!std::isfinite(publishRate) || publishRate <= 0.0) {
    throw std::invalid_argument("MPC policy publish rate must be finite and positive.");
  }
  publish_period_ = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / publishRate));
  publisher_thread_ = std::thread(&MpcPolicyPublisher::publisherWorker, this);
}

MpcPolicyPublisher::~MpcPolicyPublisher()
{
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    stop_worker_ = true;
  }
  buffer_condition_.notify_one();
  if (publisher_thread_.joinable()) {
    publisher_thread_.join();
  }
}

void MpcPolicyPublisher::modifyBufferedSolution(
  const ocs2::CommandData& command,
  ocs2::PrimalSolution& primalSolution)
{
  const auto now = Clock::now();
  if (!first_publish_ && now - last_publish_time_ < publish_period_) {
    return;
  }

  first_publish_ = false;
  last_publish_time_ = now;
  try {
    auto policy = std::make_unique<PolicyBuffer>();
    policy->command = command;
    policy->primal_solution = primalSolution;
    policy->performance = mpc_.getSolverPtr()->getPerformanceIndeces();
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      policy_buffer_ = std::move(policy);
    }
    buffer_condition_.notify_one();
  } catch (const std::exception& e) {
    RCLCPP_WARN(logger_, "Failed to buffer flattened MPC policy: %s", e.what());
  }
}

void MpcPolicyPublisher::publisherWorker()
{
  while (true) {
    std::unique_ptr<PolicyBuffer> policy;
    {
      std::unique_lock<std::mutex> lock(buffer_mutex_);
      buffer_condition_.wait(lock, [this]() {
        return stop_worker_ || policy_buffer_ != nullptr;
      });
      if (stop_worker_) {
        return;
      }
      policy.swap(policy_buffer_);
    }

    try {
      publisher_->publish(createMessage(
        policy->command,
        policy->primal_solution,
        policy->performance));
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger_, "Failed to publish flattened MPC policy: %s", e.what());
    }
  }
}

MpcPolicyPublisher::Message MpcPolicyPublisher::createMessage(
  const ocs2::CommandData& command,
  const ocs2::PrimalSolution& primalSolution,
  const ocs2::PerformanceIndex& performance) const
{
  if (!primalSolution.controllerPtr_) {
    throw std::runtime_error("Cannot publish an MPC policy without a controller.");
  }

  Message msg;
  msg.init_observation =
    ocs2::ros_msg_conversions::createObservationMsg(command.mpcInitObservation_);
  msg.plan_target_trajectories =
    ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(
      command.mpcTargetTrajectories_);
  msg.mode_schedule =
    ocs2::ros_msg_conversions::createModeScheduleMsg(primalSolution.modeSchedule_);
  msg.performance_indices =
    ocs2::ros_msg_conversions::createPerformanceIndicesMsg(
      command.mpcInitObservation_.time,
      performance);

  switch (primalSolution.controllerPtr_->getType()) {
    case ocs2::ControllerType::FEEDFORWARD:
      msg.controller_type = Message::CONTROLLER_FEEDFORWARD;
      break;
    case ocs2::ControllerType::LINEAR:
      msg.controller_type = Message::CONTROLLER_LINEAR;
      break;
    default:
      throw std::runtime_error("Cannot publish an unknown MPC controller type.");
  }

  const std::size_t trajectory_size = primalSolution.timeTrajectory_.size();
  if (primalSolution.stateTrajectory_.size() != trajectory_size ||
      primalSolution.inputTrajectory_.size() != trajectory_size) {
    throw std::runtime_error("MPC policy trajectories have inconsistent lengths.");
  }

  msg.time_trajectory = primalSolution.timeTrajectory_;
  msg.state_trajectory.reserve(trajectory_size);
  msg.input_trajectory.reserve(trajectory_size);
  msg.data.reserve(trajectory_size);
  msg.post_event_indices.reserve(primalSolution.postEventIndices_.size());

  for (const auto index : primalSolution.postEventIndices_) {
    msg.post_event_indices.push_back(static_cast<std::uint16_t>(index));
  }

  for (std::size_t k = 0; k < trajectory_size; ++k) {
    ocs2_msgs::msg::MpcState state;
    state.value.assign(
      primalSolution.stateTrajectory_[k].data(),
      primalSolution.stateTrajectory_[k].data() + primalSolution.stateTrajectory_[k].size());
    msg.state_trajectory.push_back(std::move(state));

    ocs2_msgs::msg::MpcInput input;
    input.value.assign(
      primalSolution.inputTrajectory_[k].data(),
      primalSolution.inputTrajectory_[k].data() + primalSolution.inputTrajectory_[k].size());
    msg.input_trajectory.push_back(std::move(input));
  }

  ocs2::scalar_array_t controller_times;
  controller_times.reserve(trajectory_size);
  std::vector<std::vector<float>*> controller_data;
  controller_data.reserve(trajectory_size);
  for (const auto time : primalSolution.timeTrajectory_) {
    controller_times.push_back(time);
    msg.data.emplace_back();
    controller_data.push_back(&msg.data.back().data);
  }
  primalSolution.controllerPtr_->flatten(controller_times, controller_data);

  return msg;
}

}  // namespace dynamics_mpc_controller::diagnostics
