#include "dynamics_mpc_controller/visualization/optimized_state_trajectory_visualization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace dynamics_mpc_controller::visualization
{
namespace
{

constexpr std::array<float, 3> kStartColor{0.0F, 0.447F, 0.741F};
constexpr std::array<float, 3> kEndColor{0.85F, 0.325F, 0.098F};

std_msgs::msg::ColorRGBA trajectoryColor(double progress)
{
  const float t = static_cast<float>(std::clamp(progress, 0.0, 1.0));
  std_msgs::msg::ColorRGBA color;
  color.r = (1.0F - t) * kStartColor[0] + t * kEndColor[0];
  color.g = (1.0F - t) * kStartColor[1] + t * kEndColor[1];
  color.b = (1.0F - t) * kStartColor[2] + t * kEndColor[2];
  color.a = 1.0F;
  return color;
}

builtin_interfaces::msg::Time toTimeMsg(const rclcpp::Time& time)
{
  const std::int64_t nanoseconds = time.nanoseconds();
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  msg.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return msg;
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& position)
{
  geometry_msgs::msg::Point point;
  point.x = position.x();
  point.y = position.y();
  point.z = position.z();
  return point;
}

}  // namespace

OptimizedStateTrajectoryVisualization::OptimizedStateTrajectoryVisualization(
  ocs2::PinocchioInterface pinocchioInterface,
  pinocchio::FrameIndex endEffectorFrameId,
  rclcpp_lifecycle::LifecycleNode& node,
  Settings settings)
: pinocchio_interface_(std::move(pinocchioInterface)),
  node_(node),
  settings_(std::move(settings))
{
  const auto& model = pinocchio_interface_.getModel();
  if (endEffectorFrameId >= model.frames.size()) {
    throw std::invalid_argument("Optimized state trajectory end-effector frame ID is invalid.");
  }
  frame_names_ = settings_.frame_names;
  if (frame_names_.empty()) {
    frame_names_.push_back(model.frames[endEffectorFrameId].name);
  }
  frame_ids_.reserve(frame_names_.size());
  for (const auto& frame_name : frame_names_) {
    if (!model.existFrame(frame_name)) {
      throw std::invalid_argument(
        "Optimized state trajectory frame does not exist in Pinocchio model: " + frame_name);
    }
    frame_ids_.push_back(model.getFrameId(frame_name));
  }
  if (settings_.frame_id.empty()) {
    throw std::invalid_argument("Optimized state trajectory frame ID must not be empty.");
  }
  if (!std::isfinite(settings_.line_width) || settings_.line_width <= 0.0 ||
      !std::isfinite(settings_.point_scale) || settings_.point_scale <= 0.0) {
    throw std::invalid_argument("Optimized state trajectory marker sizes must be finite and positive.");
  }

  marker_publisher_ = node_.create_publisher<Message>(
    settings_.marker_topic,
    rclcpp::QoS(1).reliable().transient_local());
}

void OptimizedStateTrajectoryVisualization::publish(
  const ocs2_msgs::msg::MpcFlattenedController& policy)
{
  const auto& model = pinocchio_interface_.getModel();
  const Eigen::Index joint_dim = model.nq;
  if (joint_dim <= 0 || policy.state_trajectory.empty()) {
    return;
  }

  ocs2::vector_array_t joint_positions;
  joint_positions.reserve(policy.state_trajectory.size());
  for (const auto& state : policy.state_trajectory) {
    if (state.value.size() < static_cast<std::size_t>(joint_dim)) {
      throw std::runtime_error("Optimized state sample is shorter than Pinocchio model.nq.");
    }

    ocs2::vector_t q(joint_dim);
    for (Eigen::Index i = 0; i < joint_dim; ++i) {
      const double value = static_cast<double>(state.value[static_cast<std::size_t>(i)]);
      if (!std::isfinite(value)) {
        throw std::runtime_error("Optimized state trajectory contains a non-finite joint position.");
      }
      q(i) = value;
    }
    joint_positions.push_back(std::move(q));
  }

  marker_publisher_->publish(createMessage(joint_positions));
}

void OptimizedStateTrajectoryVisualization::publish(
  const ocs2::vector_array_t& stateTrajectory)
{
  const auto joint_positions = extractJointPositionTrajectory(stateTrajectory);
  if (joint_positions.empty()) {
    return;
  }

  marker_publisher_->publish(createMessage(joint_positions));
}

ocs2::vector_array_t OptimizedStateTrajectoryVisualization::extractJointPositionTrajectory(
  const ocs2::vector_array_t& stateTrajectory) const
{
  const auto& model = pinocchio_interface_.getModel();
  const Eigen::Index joint_dim = model.nq;
  if (joint_dim <= 0 || stateTrajectory.empty()) {
    return {};
  }

  ocs2::vector_array_t joint_positions;
  joint_positions.reserve(stateTrajectory.size());
  for (const auto& state : stateTrajectory) {
    if (state.size() < joint_dim) {
      throw std::runtime_error("Optimized state sample is shorter than Pinocchio model.nq.");
    }

    const auto q = state.head(joint_dim).eval();
    if (!q.allFinite()) {
      throw std::runtime_error("Optimized state trajectory contains a non-finite joint position.");
    }
    joint_positions.push_back(q);
  }

  return joint_positions;
}

OptimizedStateTrajectoryVisualization::Message OptimizedStateTrajectoryVisualization::createMessage(
  const ocs2::vector_array_t& jointPositionTrajectory)
{
  Message message;
  const auto stamp = node_.get_clock()->now();

  visualization_msgs::msg::Marker delete_all;
  delete_all.header.frame_id = settings_.frame_id;
  delete_all.header.stamp = toTimeMsg(stamp);
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  message.markers.push_back(std::move(delete_all));

  std::vector<visualization_msgs::msg::Marker> line_markers;
  std::vector<visualization_msgs::msg::Marker> point_markers;
  line_markers.reserve(frame_ids_.size());
  point_markers.reserve(frame_ids_.size());

  for (std::size_t frame_index = 0; frame_index < frame_ids_.size(); ++frame_index) {
    visualization_msgs::msg::Marker line;
    line.header.frame_id = settings_.frame_id;
    line.header.stamp = toTimeMsg(stamp);
    line.ns = "optimizedStateTrajectoryLine_" + frame_names_[frame_index];
    line.id = static_cast<int>(2 * frame_index);
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = settings_.line_width;
    line.points.reserve(jointPositionTrajectory.size());
    line.colors.reserve(jointPositionTrajectory.size());

    visualization_msgs::msg::Marker points;
    points.header = line.header;
    points.ns = "optimizedStateTrajectorySamples_" + frame_names_[frame_index];
    points.id = static_cast<int>(2 * frame_index + 1);
    points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points.action = visualization_msgs::msg::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.scale.x = settings_.point_scale;
    points.scale.y = settings_.point_scale;
    points.scale.z = settings_.point_scale;
    points.points.reserve(jointPositionTrajectory.size());
    points.colors.reserve(jointPositionTrajectory.size());

    line_markers.push_back(std::move(line));
    point_markers.push_back(std::move(points));
  }

  const auto& model = pinocchio_interface_.getModel();
  auto& data = pinocchio_interface_.getData();
  for (std::size_t index = 0; index < jointPositionTrajectory.size(); ++index) {
    pinocchio::forwardKinematics(model, data, jointPositionTrajectory[index]);
    pinocchio::updateFramePlacements(model, data);

    const double progress = jointPositionTrajectory.size() > 1 ?
      static_cast<double>(index) / static_cast<double>(jointPositionTrajectory.size() - 1) : 0.0;
    const auto color = trajectoryColor(progress);

    for (std::size_t frame_index = 0; frame_index < frame_ids_.size(); ++frame_index) {
      const auto point = toPoint(data.oMf[frame_ids_[frame_index]].translation());
      line_markers[frame_index].points.push_back(point);
      line_markers[frame_index].colors.push_back(color);
      point_markers[frame_index].points.push_back(point);
      point_markers[frame_index].colors.push_back(color);
    }
  }

  for (auto& line : line_markers) {
    message.markers.push_back(std::move(line));
  }
  for (auto& points : point_markers) {
    message.markers.push_back(std::move(points));
  }
  return message;
}

}  // namespace dynamics_mpc_controller::visualization
