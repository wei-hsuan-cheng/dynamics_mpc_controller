#include "dynamics_mpc_controller/visualization/performance_visualization.hpp"

#include <exception>
#include <stdexcept>
#include <utility>

#include <rclcpp/logging.hpp>

namespace dynamics_mpc_controller::visualization
{

PerformanceVisualization::PerformanceVisualization(
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
  const ocs2::PinocchioInterface& pinocchioInterface,
  pinocchio::FrameIndex endEffectorFrameId,
  Settings settings)
: node_(std::move(node)),
  settings_(std::move(settings))
{
  if (!node_) {
    throw std::invalid_argument("PerformanceVisualization node must not be null.");
  }

  if (settings_.trajectory_active) {
    trajectory_visualization_ = std::make_unique<OptimizedStateTrajectoryVisualization>(
      pinocchioInterface,
      endEffectorFrameId,
      *node_,
      settings_.trajectory);
    RCLCPP_INFO(
      node_->get_logger(),
      "[Performance Visualization] Enable optimized state trajectory visualization");
  }

  if (settings_.self_collision_active) {
    if (settings_.self_collision.collision_link_pairs.empty() &&
        settings_.self_collision.collision_object_pairs.empty()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "[Performance Visualization] self-collision visualization is active but no collision pairs are configured; disabling it.");
    } else {
      self_collision_visualization_ = std::make_unique<SelfCollisionVisualization>(
        pinocchioInterface,
        *node_,
        settings_.self_collision);
      RCLCPP_INFO(
        node_->get_logger(),
        "[Performance Visualization] Enable self-collision distance visualization");
    }
  }

  if (settings_.robot_models_active) {
    robot_model_visualization_ = std::make_unique<FutureRobotModelVisualization>(
      pinocchioInterface, *node_, settings_.robot_models);
  }

  if (!trajectory_visualization_ && !self_collision_visualization_ && !robot_model_visualization_) {
    RCLCPP_INFO(node_->get_logger(), "[Performance Visualization] no visualization features active");
    return;
  }

  period_ = std::chrono::duration<double>(1.0 / std::max(1.0, settings_.update_rate));
  running_.store(true);
  has_snapshot_.store(false);
  visualization_thread_ = std::thread(&PerformanceVisualization::visualization_loop, this);

  RCLCPP_INFO(
    node_->get_logger(),
    "[Performance Visualization] started | frame=%s rate=%.3f Hz",
    settings_.trajectory.frame_id.c_str(),
    1.0 / period_.count());
}

PerformanceVisualization::~PerformanceVisualization()
{
  running_.store(false);
  if (visualization_thread_.joinable()) {
    visualization_thread_.join();
  }
}

void PerformanceVisualization::update_visualization(
  const ocs2::vector_t& currentState,
  const ocs2::scalar_array_t& optimizedTimeTrajectory,
  const ocs2::vector_array_t& optimizedStateTrajectory)
{
  if (!has_subscribers_.load(std::memory_order_relaxed)) return;
  if (latest_snapshot_.try_set(Snapshot{currentState, optimizedTimeTrajectory, optimizedStateTrajectory})) {
    has_snapshot_.store(true);
  }
}

void PerformanceVisualization::visualization_loop()
{
  while (running_.load()) {
    std::this_thread::sleep_for(period_);

    try {
      const bool collision_subscribed =
        self_collision_visualization_ && self_collision_visualization_->hasSubscribers();
      const bool trajectory_subscribed = trajectory_visualization_ && trajectory_visualization_->hasSubscribers();
      const bool ghosts_subscribed = robot_model_visualization_ && robot_model_visualization_->hasSubscribers();
      const bool subscribed = collision_subscribed || trajectory_subscribed || ghosts_subscribed;
      has_subscribers_.store(subscribed, std::memory_order_relaxed);
      if (!subscribed || !has_snapshot_.exchange(false)) continue;
      const auto snapshot = latest_snapshot_.try_get();
      if (!snapshot.has_value()) {
        has_snapshot_.store(true);  // the control loop was writing it: read it next period
        continue;
      }
      if (collision_subscribed) {
        self_collision_visualization_->publish(snapshot->state);
      }
      if (trajectory_subscribed) {
        trajectory_visualization_->publish(snapshot->states);
      }
      if (ghosts_subscribed) {
        robot_model_visualization_->publish(snapshot->times, snapshot->states);
      }
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "[Performance Visualization] failed to publish: %s",
        error.what());
    }
  }
}

}  // namespace dynamics_mpc_controller::visualization
