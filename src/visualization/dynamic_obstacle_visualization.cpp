#include "dynamics_mpc_controller/visualization/dynamic_obstacle_visualization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace dynamics_mpc_controller::visualization
{
namespace
{

using visualization_msgs::msg::Marker;

constexpr double kGoldenRatioConjugate = 0.618033988749895;
constexpr double kPathStep = 0.05;      // [s]
constexpr double kLabelOffsetZ = 0.04;  // [m] above the sphere

// A distinct, stable color per obstacle id: hue stepped by the golden ratio.
std::array<float, 3> obstacleColor(int id)
{
  const double hue = std::fmod(0.08 + static_cast<double>(id) * kGoldenRatioConjugate, 1.0) * 6.0;
  const double saturation = 0.75;
  const double value = 0.95;
  const int sector = static_cast<int>(hue) % 6;
  const double f = hue - std::floor(hue);
  const double p = value * (1.0 - saturation);
  const double q = value * (1.0 - saturation * f);
  const double t = value * (1.0 - saturation * (1.0 - f));
  const std::array<std::array<double, 3>, 6> rgb{{
    {value, t, p}, {q, value, p}, {p, value, t}, {p, q, value}, {t, p, value}, {value, p, q}}};
  return {static_cast<float>(rgb[sector][0]), static_cast<float>(rgb[sector][1]), static_cast<float>(rgb[sector][2])};
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& p)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  return point;
}

}  // namespace

DynamicObstacleVisualization::DynamicObstacleVisualization(
  std::shared_ptr<const constraint::DynamicObstacles> obstacles,
  rclcpp_lifecycle::LifecycleNode& node,
  Settings settings)
: obstacles_(std::move(obstacles)), node_(node), settings_(std::move(settings))
{
  if (!obstacles_) {
    throw std::invalid_argument("DynamicObstacleVisualization needs the obstacle set.");
  }
  publisher_ = node_.create_publisher<Message>(settings_.marker_topic, rclcpp::QoS(1).best_effort());
  RCLCPP_INFO(
    node_.get_logger(), "[Performance Visualization] dynamic obstacles on %s", settings_.marker_topic.c_str());
}

bool DynamicObstacleVisualization::hasSubscribers() const
{
  return publisher_->get_subscription_count() + publisher_->get_intra_process_subscription_count() > 0;
}

void DynamicObstacleVisualization::publish()
{
  constraint::DynamicObstacleSnapshot snapshot;
  if (!obstacles_->lastSnapshot(snapshot)) {
    return;
  }
  const rclcpp::Time stamp = node_.now();
  Message message;
  Marker clear;
  clear.header.frame_id = settings_.frame_id;
  clear.header.stamp = stamp;
  clear.action = Marker::DELETEALL;
  message.markers.push_back(clear);

  auto make = [&](const std::string& ns, int id, int type) {
    Marker marker;
    marker.header.frame_id = settings_.frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  };

  int index = 0;
  std::size_t previous_slot = snapshot.slot_ids.size();
  for (const auto& sphere : snapshot.spheres) {
    const auto rgb = obstacleColor(sphere.obstacle_id);
    const double weight = std::clamp(sphere.weight, 0.0, 1.0);
    const Eigen::Vector3d now = snapshot.center(sphere, snapshot.init_time);

    auto current = make("obstacleNow", index, Marker::SPHERE);
    current.pose.position = toPoint(now);
    current.scale.x = current.scale.y = current.scale.z = 2.0 * sphere.radius;
    current.color.r = rgb[0];
    current.color.g = rgb[1];
    current.color.b = rgb[2];
    current.color.a = static_cast<float>(0.2 + 0.6 * weight);
    message.markers.push_back(current);

    auto shell = current;
    shell.ns = "obstacleClearance";
    shell.scale.x = shell.scale.y = shell.scale.z = 2.0 * (sphere.radius + settings_.minimum_distance);
    shell.color.a = static_cast<float>(0.12 * weight);
    message.markers.push_back(shell);

    auto path = make("obstaclePath", index, Marker::LINE_STRIP);
    path.scale.x = 0.004;
    path.color = current.color;
    path.color.a = 0.6F;
    const int path_steps = std::max(1, static_cast<int>(std::lround(settings_.horizon / kPathStep)));
    for (int k = 0; k <= path_steps; ++k) {
      path.points.push_back(toPoint(snapshot.center(sphere, snapshot.init_time + k * kPathStep)));
    }
    message.markers.push_back(path);

    const int ghosts = static_cast<int>(std::floor(settings_.horizon / settings_.interval + 1e-9));
    for (int k = 1; k <= ghosts; ++k) {
      auto future = make("obstaclePrediction", index * 100 + k, Marker::SPHERE);
      future.pose.position = toPoint(snapshot.center(sphere, snapshot.init_time + k * settings_.interval));
      future.scale.x = future.scale.y = future.scale.z = 2.0 * sphere.radius;
      future.color = current.color;
      future.color.a = 0.15F;
      message.markers.push_back(future);
    }

    if (sphere.slot != previous_slot) {
      auto label = make("obstacleId", static_cast<int>(sphere.slot), Marker::TEXT_VIEW_FACING);
      label.pose.position = toPoint(now + Eigen::Vector3d(0.0, 0.0, sphere.radius + kLabelOffsetZ));
      label.scale.z = 0.04;
      label.color.r = label.color.g = label.color.b = 1.0F;
      label.color.a = 1.0F;
      std::ostringstream text;
      text << "obstacle " << sphere.obstacle_id;
      if (weight < 1.0) {
        text << " (" << std::fixed << std::setprecision(0) << 100.0 * weight << "%)";
      }
      label.text = text.str();
      message.markers.push_back(label);
      previous_slot = sphere.slot;
    }
    ++index;
  }
  publisher_->publish(message);
}

}  // namespace dynamics_mpc_controller::visualization
