#ifndef DYNAMICS_MPC_CONTROLLER__VISUALIZATION__DYNAMIC_OBSTACLE_VISUALIZATION_HPP_
#define DYNAMICS_MPC_CONTROLLER__VISUALIZATION__DYNAMIC_OBSTACLE_VISUALIZATION_HPP_

#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dynamics_mpc_controller/common/constraint/dynamic_obstacles.hpp"

namespace dynamics_mpc_controller::visualization
{

/**
 * The obstacle spheres the MPC was last given, per obstacle: now (alpha by the fade-in weight, so a new
 * obstacle shows faint), the clearance shell (radius + minimumDistance), the predicted path over the
 * horizon, the predicted spheres at the robot-ghost times, and the obstacle id.
 */
class DynamicObstacleVisualization
{
public:
  struct Settings
  {
    std::string frame_id{"world"};
    std::string marker_topic{"/performance_visualization/dynamicObstacleMarkers"};
    double minimum_distance{0.0};
    double horizon{1.0};   // predicted path [s]
    double interval{0.2};  // predicted spheres every interval [s], as the robot ghosts
  };

  DynamicObstacleVisualization(
    std::shared_ptr<const constraint::DynamicObstacles> obstacles,
    rclcpp_lifecycle::LifecycleNode& node,
    Settings settings);

  bool hasSubscribers() const;
  void publish();

private:
  using Message = visualization_msgs::msg::MarkerArray;

  std::shared_ptr<const constraint::DynamicObstacles> obstacles_;
  rclcpp_lifecycle::LifecycleNode& node_;
  Settings settings_;
  rclcpp::Publisher<Message>::SharedPtr publisher_;
};

}  // namespace dynamics_mpc_controller::visualization

#endif  // DYNAMICS_MPC_CONTROLLER__VISUALIZATION__DYNAMIC_OBSTACLE_VISUALIZATION_HPP_
