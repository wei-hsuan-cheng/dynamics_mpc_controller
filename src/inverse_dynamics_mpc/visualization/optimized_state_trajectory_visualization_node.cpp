#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp/rclcpp.hpp>

#include "dynamics_mpc_controller/inverse_dynamics_mpc/visualization/optimized_state_trajectory_visualization.hpp"
#include "dynamics_mpc_controller/pinocchio_utils.hpp"

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

class OptimizedStateTrajectoryVisualizationNode final : public rclcpp::Node
{
public:
  OptimizedStateTrajectoryVisualizationNode()
  : Node("optimized_state_trajectory_visualization")
  {
    const bool activate = declare_parameter<bool>("activate", true);
    if (!activate) {
      RCLCPP_INFO(get_logger(), "Optimized state trajectory visualization is disabled.");
      return;
    }

    const std::string urdf_file = declare_parameter<std::string>("urdfFile", "");
    const auto remove_joints = sanitizeJointNames(
      declare_parameter<std::vector<std::string>>("removeJoints", std::vector<std::string>{}));
    const std::string end_effector_frame = declare_parameter<std::string>("endEffectorFrame", "");
    const std::string policy_topic = declare_parameter<std::string>(
      "mpcPolicyTopic", "/inverse_dynamics_mpc_policy");

    visualization::OptimizedStateTrajectoryVisualization::Settings settings;
    settings.marker_topic = declare_parameter<std::string>(
      "markerTopic", "/inverse_dynamics_mpc/visualization/optimizedStateTrajectory");
    settings.frame_id = declare_parameter<std::string>("frameId", "world");
    settings.publish_rate_hz = declare_parameter<double>("publishRate", 10.0);
    settings.line_width = declare_parameter<double>("lineWidth", 0.01);
    settings.point_scale = declare_parameter<double>("pointScale", 0.025);

    if (urdf_file.empty()) {
      throw std::runtime_error("optimized_state_trajectory_visualization.urdfFile must not be empty.");
    }
    if (end_effector_frame.empty()) {
      throw std::runtime_error("optimized_state_trajectory_visualization.endEffectorFrame must not be empty.");
    }

    auto pinocchio_interface = pinocchio_utils::createPinocchioInterface(urdf_file, remove_joints);
    const auto& model = pinocchio_interface.getModel();
    const auto end_effector_frame_id = model.getFrameId(end_effector_frame);
    if (end_effector_frame_id >= model.frames.size()) {
      throw std::runtime_error("Pinocchio model does not contain end-effector frame: " + end_effector_frame);
    }

    visualization_ = std::make_unique<visualization::OptimizedStateTrajectoryVisualization>(
      std::move(pinocchio_interface),
      end_effector_frame_id,
      *this,
      settings);

    policy_subscription_ = create_subscription<ocs2_msgs::msg::MpcFlattenedController>(
      policy_topic,
      rclcpp::QoS(1),
      [this](const ocs2_msgs::msg::MpcFlattenedController::SharedPtr msg) {
        try {
          visualization_->publish(*msg);
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
      "Optimized state trajectory visualization started | policy=%s markers=%s frame=%s eeFrame=%s rate=%.2f Hz",
      policy_topic.c_str(),
      settings.marker_topic.c_str(),
      settings.frame_id.c_str(),
      end_effector_frame.c_str(),
      settings.publish_rate_hz);
  }

private:
  std::unique_ptr<visualization::OptimizedStateTrajectoryVisualization> visualization_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcFlattenedController>::SharedPtr policy_subscription_;
};

}  // namespace dynamics_mpc_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dynamics_mpc_controller::OptimizedStateTrajectoryVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
