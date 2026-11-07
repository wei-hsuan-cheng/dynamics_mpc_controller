#ifndef DYNAMICS_MPC_CONTROLLER__VISUALIZATION__FUTURE_ROBOT_MODEL_VISUALIZATION_HPP_
#define DYNAMICS_MPC_CONTROLLER__VISUALIZATION__FUTURE_ROBOT_MODEL_VISUALIZATION_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace dynamics_mpc_controller::visualization
{

/**
 * The robot's URDF visuals at states along the optimized MPC trajectory (future robot ghosts) and at
 * its end (terminal state ghost), as legged_robot_mpc_controller's FutureRobotModelVisualization.
 * Fixed base: the joint positions are the first nq entries of the state.
 */
class FutureRobotModelVisualization
{
public:
  struct Settings
  {
    std::string frame_id{"world"};
    double interval{0.2};
    std::string future_topic{"/performance_visualization/optimizedStateGhosts"};
    std::string terminal_topic{"/performance_visualization/terminalStateGhost"};
  };

  FutureRobotModelVisualization(
    ocs2::PinocchioInterface pinocchioInterface, rclcpp_lifecycle::LifecycleNode& node, Settings settings);

  bool hasSubscribers() const;
  void publish(const ocs2::scalar_array_t& times, const ocs2::vector_array_t& states);

private:
  using Message = visualization_msgs::msg::MarkerArray;
  using Publisher = rclcpp::Publisher<Message>;
  struct Visual
  {
    std::size_t frame_id;
    Eigen::Isometry3d link_to_visual;
    visualization_msgs::msg::Marker marker;
  };

  void initializeVisuals();
  Message emptyMessage(const rclcpp::Time& stamp) const;
  void appendStateMarkers(
    Message& message, const rclcpp::Time& stamp, const ocs2::vector_t& state, const std::string& markerNamespace,
    const std::array<float, 3>& color, double alpha);

  ocs2::PinocchioInterface pinocchio_interface_;
  rclcpp_lifecycle::LifecycleNode& node_;
  Settings settings_;
  std::vector<Visual> visuals_;
  Publisher::SharedPtr future_publisher_;
  Publisher::SharedPtr terminal_publisher_;
};

}  // namespace dynamics_mpc_controller::visualization

#endif  // DYNAMICS_MPC_CONTROLLER__VISUALIZATION__FUTURE_ROBOT_MODEL_VISUALIZATION_HPP_
