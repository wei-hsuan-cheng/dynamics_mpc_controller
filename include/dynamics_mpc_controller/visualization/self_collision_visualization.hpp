#ifndef DYNAMICS_MPC_CONTROLLER__VISUALIZATION__SELF_COLLISION_VISUALIZATION_HPP_
#define DYNAMICS_MPC_CONTROLLER__VISUALIZATION__SELF_COLLISION_VISUALIZATION_HPP_

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace dynamics_mpc_controller::visualization
{

class SelfCollisionVisualization
{
public:
  struct Settings
  {
    std::string marker_topic{"/policy_visualization/selfCollisionDistanceMarkers"};
    std::string frame_id{"world"};
    double minimum_distance{0.0};
    std::vector<std::pair<std::string, std::string>> collision_link_pairs;
    std::vector<std::pair<std::size_t, std::size_t>> collision_object_pairs;
  };

  SelfCollisionVisualization(
    ocs2::PinocchioInterface pinocchioInterface,
    rclcpp::Node& node,
    Settings settings);

  void publish(const ocs2_msgs::msg::MpcFlattenedController& policy);

private:
  using Message = visualization_msgs::msg::MarkerArray;

  Message createMessage(const std::vector<ocs2::vector_t>& jointPositionTrajectory);

  ocs2::PinocchioInterface pinocchio_interface_;
  ocs2::PinocchioGeometryInterface geometry_interface_;
  rclcpp::Node& node_;
  Settings settings_;
  rclcpp::Publisher<Message>::SharedPtr marker_publisher_;
};

}  // namespace dynamics_mpc_controller::visualization

#endif  // DYNAMICS_MPC_CONTROLLER__VISUALIZATION__SELF_COLLISION_VISUALIZATION_HPP_
