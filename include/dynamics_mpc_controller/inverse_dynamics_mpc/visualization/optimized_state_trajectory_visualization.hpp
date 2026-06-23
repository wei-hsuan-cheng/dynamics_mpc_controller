#ifndef DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__VISUALIZATION__OPTIMIZED_STATE_TRAJECTORY_VISUALIZATION_HPP_
#define DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__VISUALIZATION__OPTIMIZED_STATE_TRAJECTORY_VISUALIZATION_HPP_

#include <chrono>
#include <cstddef>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/multibody/fwd.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace dynamics_mpc_controller::visualization
{

class OptimizedStateTrajectoryVisualization
{
public:
  struct Settings
  {
    std::string marker_topic{"/inverse_dynamics_mpc/visualization/optimizedStateTrajectory"};
    std::string frame_id{"world"};
    double publish_rate_hz{10.0};
    double line_width{0.01};
    double point_scale{0.025};
  };

  OptimizedStateTrajectoryVisualization(
    ocs2::PinocchioInterface pinocchioInterface,
    pinocchio::FrameIndex endEffectorFrameId,
    rclcpp::Node& node,
    Settings settings);

  void publish(const ocs2_msgs::msg::MpcFlattenedController& policy);

private:
  using Message = visualization_msgs::msg::MarkerArray;
  using SteadyClock = std::chrono::steady_clock;

  Message createMessage(const ocs2::vector_array_t& jointPositionTrajectory);

  ocs2::PinocchioInterface pinocchio_interface_;
  pinocchio::FrameIndex end_effector_frame_id_{0};
  rclcpp::Node& node_;
  Settings settings_;
  rclcpp::Publisher<Message>::SharedPtr marker_publisher_;
  SteadyClock::duration publish_period_{};
  SteadyClock::time_point last_publish_time_{};
  bool first_publish_{true};
};

}  // namespace dynamics_mpc_controller::visualization

#endif  // DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__VISUALIZATION__OPTIMIZED_STATE_TRAJECTORY_VISUALIZATION_HPP_
