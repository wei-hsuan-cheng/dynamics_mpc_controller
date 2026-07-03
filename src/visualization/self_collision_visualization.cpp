#include "dynamics_mpc_controller/visualization/self_collision_visualization.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <Eigen/Core>
#include <geometry_msgs/msg/point.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace dynamics_mpc_controller::visualization
{
namespace
{

constexpr double kDistanceEps = 1e-9;
constexpr double kLabelOffsetZ = 0.03;
constexpr double kLabelLateralOffset = 0.015;

constexpr std::array<double, 3> kClearColor{0.173, 0.627, 0.173};
constexpr std::array<double, 3> kNearColor{1.000, 0.498, 0.055};
constexpr std::array<double, 3> kViolationColor{0.839, 0.153, 0.157};

struct CollisionMarkerData
{
  double min_distance{std::numeric_limits<double>::infinity()};
  Eigen::Vector3d first_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d second_point{Eigen::Vector3d::Zero()};
  std::size_t first_object{0};
  std::size_t second_object{0};
  bool valid{false};
};

builtin_interfaces::msg::Time toTimeMsg(const rclcpp::Time& time)
{
  const std::int64_t nanoseconds = time.nanoseconds();
  builtin_interfaces::msg::Time msg;
  msg.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  msg.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return msg;
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

std_msgs::msg::ColorRGBA colorFromArray(const std::array<double, 3>& color, double alpha)
{
  std_msgs::msg::ColorRGBA msg;
  msg.r = static_cast<float>(color[0]);
  msg.g = static_cast<float>(color[1]);
  msg.b = static_cast<float>(color[2]);
  msg.a = static_cast<float>(alpha);
  return msg;
}

std_msgs::msg::ColorRGBA distanceColor(double signedMargin)
{
  if (signedMargin < 0.0) {
    return colorFromArray(kViolationColor, 1.0);
  }
  if (signedMargin < 0.05) {
    return colorFromArray(kNearColor, 1.0);
  }
  return colorFromArray(kClearColor, 1.0);
}

std::string stripCollisionSuffix(std::string name)
{
  const auto separator = name.find_last_of('_');
  if (separator == std::string::npos || separator + 1 >= name.size()) {
    return name;
  }

  const auto suffixIsNumber = std::all_of(
    name.begin() + static_cast<std::string::difference_type>(separator + 1),
    name.end(),
    [](unsigned char c) {return std::isdigit(c);});

  if (suffixIsNumber) {
    name.erase(separator);
  }
  return name;
}

std::string geometryObjectName(const pinocchio::GeometryModel& geometryModel, std::size_t objectIndex)
{
  if (objectIndex >= geometryModel.geometryObjects.size()) {
    return "object_" + std::to_string(objectIndex);
  }

  const auto& name = geometryModel.geometryObjects[objectIndex].name;
  if (name.empty()) {
    return "object_" + std::to_string(objectIndex);
  }
  return stripCollisionSuffix(name);
}

std::string formatDistance(double distance)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << distance;
  return stream.str();
}

visualization_msgs::msg::Marker makeDeleteAllMarker(
  const std::string& frameId,
  const rclcpp::Time& stamp)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frameId;
  marker.header.stamp = toTimeMsg(stamp);
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  return marker;
}

}  // namespace

SelfCollisionVisualization::SelfCollisionVisualization(
  ocs2::PinocchioInterface pinocchioInterface,
  rclcpp_lifecycle::LifecycleNode& node,
  Settings settings)
: pinocchio_interface_(std::move(pinocchioInterface)),
  geometry_interface_(
    pinocchio_interface_,
    settings.collision_link_pairs,
    settings.collision_object_pairs),
  node_(node),
  settings_(std::move(settings))
{
  if (settings_.frame_id.empty()) {
    throw std::invalid_argument("Self-collision visualization frame ID must not be empty.");
  }
  if (settings_.collision_link_pairs.empty() && settings_.collision_object_pairs.empty()) {
    throw std::invalid_argument("Self-collision visualization requires at least one collision pair.");
  }
  if (geometry_interface_.getNumCollisionPairs() == 0) {
    throw std::invalid_argument("Self-collision visualization configured zero Pinocchio collision pairs.");
  }
  if (!std::isfinite(settings_.minimum_distance) || settings_.minimum_distance < 0.0) {
    throw std::invalid_argument("Self-collision visualization minimum distance must be finite and non-negative.");
  }

  marker_publisher_ = node_.create_publisher<Message>(
    settings_.marker_topic,
    rclcpp::QoS(1).reliable().transient_local());
}

void SelfCollisionVisualization::publish(
  const ocs2_msgs::msg::MpcFlattenedController& policy)
{
  const auto& model = pinocchio_interface_.getModel();
  const Eigen::Index joint_dim = model.nq;
  if (joint_dim <= 0 || policy.state_trajectory.empty()) {
    return;
  }

  std::vector<ocs2::vector_t> joint_positions;
  joint_positions.reserve(policy.state_trajectory.size());
  for (const auto& state : policy.state_trajectory) {
    if (state.value.size() < static_cast<std::size_t>(joint_dim)) {
      throw std::runtime_error("Self-collision policy state sample is shorter than Pinocchio model.nq.");
    }

    ocs2::vector_t q(joint_dim);
    for (Eigen::Index i = 0; i < joint_dim; ++i) {
      const double value = static_cast<double>(state.value[static_cast<std::size_t>(i)]);
      if (!std::isfinite(value)) {
        throw std::runtime_error("Self-collision policy state trajectory contains a non-finite joint position.");
      }
      q(i) = value;
    }
    joint_positions.push_back(std::move(q));
  }

  marker_publisher_->publish(createMessage(joint_positions));
}

void SelfCollisionVisualization::publish(const ocs2::vector_t& state)
{
  const ocs2::vector_array_t state_trajectory{state};
  publish(state_trajectory);
}

void SelfCollisionVisualization::publish(const ocs2::vector_array_t& stateTrajectory)
{
  const auto joint_positions = extractJointPositionTrajectory(stateTrajectory);
  if (joint_positions.empty()) {
    return;
  }

  marker_publisher_->publish(createMessage(joint_positions));
}

ocs2::vector_array_t SelfCollisionVisualization::extractJointPositionTrajectory(
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
      throw std::runtime_error("Self-collision state sample is shorter than Pinocchio model.nq.");
    }

    const auto q = state.head(joint_dim).eval();
    if (!q.allFinite()) {
      throw std::runtime_error("Self-collision state trajectory contains a non-finite joint position.");
    }
    joint_positions.push_back(q);
  }

  return joint_positions;
}

SelfCollisionVisualization::Message SelfCollisionVisualization::createMessage(
  const std::vector<ocs2::vector_t>& jointPositionTrajectory)
{
  Message message;
  const auto stamp = node_.get_clock()->now();
  message.markers.push_back(makeDeleteAllMarker(settings_.frame_id, stamp));

  const auto& geometry_model = geometry_interface_.getGeometryModel();
  std::vector<CollisionMarkerData> best_results(geometry_model.collisionPairs.size());

  const auto& model = pinocchio_interface_.getModel();
  auto& data = pinocchio_interface_.getData();
  for (const auto& q : jointPositionTrajectory) {
    pinocchio::forwardKinematics(model, data, q);
    const auto results = geometry_interface_.computeDistances(pinocchio_interface_);
    const std::size_t num_results = std::min(results.size(), geometry_model.collisionPairs.size());
    for (std::size_t i = 0; i < num_results; ++i) {
      if (results[i].min_distance >= best_results[i].min_distance) {
        continue;
      }

      const auto& collision_pair = geometry_model.collisionPairs[i];
      best_results[i].min_distance = results[i].min_distance;
      best_results[i].first_point = Eigen::Vector3d(
        results[i].nearest_points[0][0],
        results[i].nearest_points[0][1],
        results[i].nearest_points[0][2]);
      best_results[i].second_point = Eigen::Vector3d(
        results[i].nearest_points[1][0],
        results[i].nearest_points[1][1],
        results[i].nearest_points[1][2]);
      best_results[i].first_object = collision_pair.first;
      best_results[i].second_object = collision_pair.second;
      best_results[i].valid = true;
    }
  }

  int marker_id = 0;
  for (const auto& result : best_results) {
    if (!result.valid) {
      continue;
    }

    const std::string pair_name =
      geometryObjectName(geometry_model, result.first_object) + " - " +
      geometryObjectName(geometry_model, result.second_object);

    Eigen::Vector3d direction = result.second_point - result.first_point;
    const double distance = direction.norm();
    if (distance > kDistanceEps) {
      direction /= distance;
    } else {
      direction = Eigen::Vector3d::UnitX();
    }

    const double signed_margin = result.min_distance - settings_.minimum_distance;
    const auto color = distanceColor(signed_margin);

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = settings_.frame_id;
    arrow.header.stamp = toTimeMsg(stamp);
    arrow.ns = "selfCollisionArrow";
    arrow.id = marker_id++;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose.orientation.w = 1.0;
    arrow.points.push_back(toPoint(result.first_point));
    arrow.points.push_back(toPoint(result.second_point));
    arrow.scale.x = 0.008;
    arrow.scale.y = 0.016;
    arrow.scale.z = 0.024;
    arrow.color = color;
    arrow.text = pair_name;
    message.markers.push_back(std::move(arrow));

    visualization_msgs::msg::Marker endpoints;
    endpoints.header.frame_id = settings_.frame_id;
    endpoints.header.stamp = toTimeMsg(stamp);
    endpoints.ns = "selfCollisionEndpoints";
    endpoints.id = marker_id++;
    endpoints.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    endpoints.action = visualization_msgs::msg::Marker::ADD;
    endpoints.pose.orientation.w = 1.0;
    endpoints.points.push_back(toPoint(result.first_point));
    endpoints.points.push_back(toPoint(result.second_point));
    endpoints.scale.x = 0.02;
    endpoints.scale.y = 0.02;
    endpoints.scale.z = 0.02;
    endpoints.color = color;
    endpoints.text = pair_name;
    message.markers.push_back(std::move(endpoints));

    visualization_msgs::msg::Marker text;
    text.header.frame_id = settings_.frame_id;
    text.header.stamp = toTimeMsg(stamp);
    text.ns = "selfCollisionText";
    text.id = marker_id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;

    Eigen::Vector3d label_position = 0.5 * (result.first_point + result.second_point);
    Eigen::Vector3d lateral = direction.cross(Eigen::Vector3d::UnitZ());
    if (lateral.norm() < kDistanceEps) {
      lateral = direction.cross(Eigen::Vector3d::UnitY());
    }
    if (lateral.norm() > kDistanceEps) {
      label_position += lateral.normalized() * kLabelLateralOffset;
    }
    label_position.z() += kLabelOffsetZ;

    text.pose.position = toPoint(label_position);
    text.pose.orientation.w = 1.0;
    text.scale.z = 0.03;
    text.color = color;
    text.text = "margin=" + formatDistance(signed_margin);
    message.markers.push_back(std::move(text));
  }

  return message;
}

}  // namespace dynamics_mpc_controller::visualization
