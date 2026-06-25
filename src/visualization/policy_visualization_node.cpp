#include <algorithm>
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
    if (!optimized_state_trajectory_active) {
      RCLCPP_INFO(get_logger(), "Policy visualization has no active visualization features.");
      return;
    }

    visualization::OptimizedStateTrajectoryVisualization::Settings settings;
    settings.marker_topic = declare_parameter<std::string>(
      "optimized_state_trajectory.markerTopic", "/policy_visualization/optimizedStateTrajectory");
    settings.frame_id = frame_id;
    settings.line_width = declare_parameter<double>("optimized_state_trajectory.lineWidth", 0.01);
    settings.point_scale = declare_parameter<double>("optimized_state_trajectory.pointScale", 0.025);

    if (urdf_file.empty()) {
      throw std::runtime_error("policy_visualization.urdfFile must not be empty.");
    }
    if (end_effector_frame.empty()) {
      throw std::runtime_error("policy_visualization.endEffectorFrame must not be empty.");
    }

    auto pinocchio_interface = pinocchio_utils::createPinocchioInterface(urdf_file, remove_joints);
    const auto& model = pinocchio_interface.getModel();
    const auto end_effector_frame_id = model.getFrameId(end_effector_frame);
    if (end_effector_frame_id >= model.frames.size()) {
      throw std::runtime_error("Pinocchio model does not contain end-effector frame: " + end_effector_frame);
    }

    optimized_state_trajectory_visualization_ = std::make_unique<visualization::OptimizedStateTrajectoryVisualization>(
      std::move(pinocchio_interface),
      end_effector_frame_id,
      *this,
      settings);

    policy_subscription_ = create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
      policy_topic,
      rclcpp::QoS(1),
      [this](const ocs2_msgs::msg::MpcFlattenedController::SharedPtr msg) {
        try {
          // Publish visualization components
          optimized_state_trajectory_visualization_->publish(*msg);
          // TODO: Other components can be added in the future
        } catch (const std::exception& error) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Failed to publish optimizedStateTrajectory: %s",
            error.what());
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "Policy visualization started | optimizedStateTrajectory=true policy=%s markers=%s frame=%s eeFrame=%s",
      policy_topic.c_str(),
      settings.marker_topic.c_str(),
      settings.frame_id.c_str(),
      end_effector_frame.c_str());
  }

private:
  std::unique_ptr<visualization::OptimizedStateTrajectoryVisualization> optimized_state_trajectory_visualization_;
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
