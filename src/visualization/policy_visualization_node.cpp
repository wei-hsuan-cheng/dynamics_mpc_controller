#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dynamics_mpc_controller/common/pinocchio_utils.hpp"
#include "dynamics_mpc_controller/visualization/optimized_state_trajectory_visualization.hpp"
#include "dynamics_mpc_controller/visualization/self_collision_visualization.hpp"

namespace dynamics_mpc_controller
{
namespace
{

std::vector<std::string> sanitizeJointNames(std::vector<std::string> names)
{
  names.erase(
    std::remove_if(
      names.begin(),
      names.end(),
      [](const std::string& name) {return name.empty();}),
    names.end());
  return names;
}

std::vector<std::pair<std::string, std::string>> parseCollisionLinkPairs(
  rclcpp::Node& node,
  const std::string& prefix)
{
  std::vector<std::pair<std::string, std::string>> collision_link_pairs;
  const auto pair_names = node.declare_parameter<std::vector<std::string>>(
    prefix + ".link_pair_names", std::vector<std::string>{});
  collision_link_pairs.reserve(pair_names.size());

  for (const auto& pair_name : pair_names) {
    if (pair_name.empty()) {
      continue;
    }

    const auto link_a = node.declare_parameter<std::string>(
      prefix + "." + pair_name + ".link_a", "");
    const auto link_b = node.declare_parameter<std::string>(
      prefix + "." + pair_name + ".link_b", "");
    if (link_a.empty() || link_b.empty()) {
      throw std::runtime_error(
        "policy_visualization." + prefix + "." + pair_name +
        " must define non-empty link_a and link_b.");
    }
    collision_link_pairs.emplace_back(link_a, link_b);
  }

  return collision_link_pairs;
}

std::vector<std::pair<std::size_t, std::size_t>> parseCollisionObjectPairs(
  rclcpp::Node& node,
  const std::string& prefix)
{
  std::vector<std::pair<std::size_t, std::size_t>> collision_object_pairs;
  const auto pair_names = node.declare_parameter<std::vector<std::string>>(
    prefix + ".object_pair_names", std::vector<std::string>{});
  collision_object_pairs.reserve(pair_names.size());

  for (const auto& pair_name : pair_names) {
    if (pair_name.empty()) {
      continue;
    }

    const auto object_a = node.declare_parameter<std::int64_t>(
      prefix + "." + pair_name + ".object_a", -1);
    const auto object_b = node.declare_parameter<std::int64_t>(
      prefix + "." + pair_name + ".object_b", -1);
    if (object_a < 0 || object_b < 0) {
      throw std::runtime_error(
        "policy_visualization." + prefix + "." + pair_name +
        " must define non-negative object_a and object_b.");
    }
    collision_object_pairs.emplace_back(
      static_cast<std::size_t>(object_a),
      static_cast<std::size_t>(object_b));
  }

  return collision_object_pairs;
}

}  // namespace

class PolicyVisualizationNode final : public rclcpp::Node
{
public:
  PolicyVisualizationNode()
  : Node("policy_visualization")
  {
    const std::string urdf_file = declare_parameter<std::string>("urdfFile", "");
    const auto remove_joints = sanitizeJointNames(
      declare_parameter<std::vector<std::string>>("removeJoints", std::vector<std::string>{}));
    const std::string end_effector_frame = declare_parameter<std::string>("endEffectorFrame", "");
    const std::string policy_topic = declare_parameter<std::string>(
      "mpcPolicyTopic", "/mpc_policy");
    const std::string frame_id = declare_parameter<std::string>("frameId", "world");
    const bool optimized_state_trajectory_active = declare_parameter<bool>(
      "optimized_state_trajectory.activate", true);
    const bool self_collision_active = declare_parameter<bool>("self_collision.activate", false);
    if (!optimized_state_trajectory_active && !self_collision_active) {
      RCLCPP_INFO(get_logger(), "Policy visualization has no active visualization features.");
      return;
    }

    if (urdf_file.empty()) {
      throw std::runtime_error("policy_visualization.urdfFile must not be empty.");
    }

    if (optimized_state_trajectory_active) {
      if (end_effector_frame.empty()) {
        throw std::runtime_error("policy_visualization.endEffectorFrame must not be empty.");
      }

      visualization::OptimizedStateTrajectoryVisualization::Settings settings;
      settings.marker_topic = declare_parameter<std::string>(
        "optimized_state_trajectory.markerTopic", "/policy_visualization/optimizedStateTrajectory");
      settings.frame_id = frame_id;
      settings.line_width = declare_parameter<double>("optimized_state_trajectory.lineWidth", 0.01);
      settings.point_scale = declare_parameter<double>("optimized_state_trajectory.pointScale", 0.025);

      auto pinocchio_interface = pinocchio_utils::createPinocchioInterface(urdf_file, remove_joints);
      const auto& model = pinocchio_interface.getModel();
      const auto end_effector_frame_id = model.getFrameId(end_effector_frame);
      if (end_effector_frame_id >= model.frames.size()) {
        throw std::runtime_error("Pinocchio model does not contain end-effector frame: " + end_effector_frame);
      }

      optimized_state_trajectory_visualization_ =
        std::make_unique<visualization::OptimizedStateTrajectoryVisualization>(
          std::move(pinocchio_interface),
          end_effector_frame_id,
          *this,
          settings);
    }

    if (self_collision_active) {
      visualization::SelfCollisionVisualization::Settings settings;
      settings.marker_topic = declare_parameter<std::string>(
        "self_collision.markerTopic", "/policy_visualization/selfCollisionDistanceMarkers");
      settings.frame_id = frame_id;
      const double default_minimum_distance = declare_parameter<double>(
        "ocs2.task.selfCollision.minimumDistance", 0.0);
      settings.minimum_distance = declare_parameter<double>(
        "self_collision.minimumDistance", default_minimum_distance);
      settings.collision_link_pairs = parseCollisionLinkPairs(*this, "self_collision");
      settings.collision_object_pairs = parseCollisionObjectPairs(*this, "self_collision");
      if (settings.collision_link_pairs.empty() && settings.collision_object_pairs.empty()) {
        settings.collision_link_pairs = parseCollisionLinkPairs(*this, "ocs2.task.selfCollision");
        settings.collision_object_pairs = parseCollisionObjectPairs(*this, "ocs2.task.selfCollision");
      }

      auto pinocchio_interface = pinocchio_utils::createPinocchioInterface(urdf_file, remove_joints);
      self_collision_visualization_ = std::make_unique<visualization::SelfCollisionVisualization>(
        std::move(pinocchio_interface),
        *this,
        settings);
    }

    policy_subscription_ = create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
      policy_topic,
      rclcpp::QoS(1),
      [this](const ocs2_msgs::msg::MpcFlattenedController::SharedPtr msg) {
        try {
          // Publish visualization components
          if (optimized_state_trajectory_visualization_) {
            optimized_state_trajectory_visualization_->publish(*msg);
          }
          if (self_collision_visualization_) {
            self_collision_visualization_->publish(*msg);
          }
        } catch (const std::exception& error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Failed to publish policy visualization: %s",
            error.what());
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "Policy visualization started | optimizedStateTrajectory=%s selfCollision=%s policy=%s frame=%s eeFrame=%s",
      optimized_state_trajectory_visualization_ ? "true" : "false",
      self_collision_visualization_ ? "true" : "false",
      policy_topic.c_str(),
      frame_id.c_str(),
      end_effector_frame.c_str());
  }

private:
  std::unique_ptr<visualization::OptimizedStateTrajectoryVisualization> optimized_state_trajectory_visualization_;
  std::unique_ptr<visualization::SelfCollisionVisualization> self_collision_visualization_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcFlattenedController>::SharedPtr policy_subscription_;
};

}  // namespace dynamics_mpc_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dynamics_mpc_controller::PolicyVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
