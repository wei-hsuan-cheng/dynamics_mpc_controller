#include "dynamics_mpc_controller/common/constraint/dynamic_obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <unordered_map>
#include <utility>

namespace dynamics_mpc_controller
{
namespace constraint
{
namespace
{

double stampSeconds(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

bool finite(double value) { return std::isfinite(value); }

}  // namespace

Eigen::Vector3d DynamicObstacleSnapshot::center(const Sphere& sphere, double time) const
{
  const double elapsed = predict ? std::clamp(time - stamp, 0.0, max_prediction_time) : 0.0;
  const Eigen::Vector3d origin = sphere.position + elapsed * sphere.linear_velocity;
  const double angle = sphere.angular_velocity.norm() * elapsed;
  const Eigen::Quaterniond rotation = angle > 1e-12 ?
    Eigen::Quaterniond(Eigen::AngleAxisd(angle, sphere.angular_velocity.normalized())) * sphere.orientation :
    sphere.orientation;
  return origin + rotation * sphere.center_in_object;
}

DynamicObstacles::DynamicObstacles(DynamicObstacleSettings settings)
: settings_(std::move(settings)), slots_(settings_.max_obstacles)
{
}

bool DynamicObstacles::update(const Msg& msg, double receiveTime, std::string* error)
{
  if (!settings_.frame_id.empty() && msg.header.frame_id != settings_.frame_id) {
    if (error) {
      *error = "obstacles in frame '" + msg.header.frame_id + "', expected '" + settings_.frame_id + "'";
    }
    return false;
  }
  for (const auto& obstacle : msg.obstacles) {
    if (obstacle.sphere_centers.empty() || obstacle.sphere_centers.size() != obstacle.sphere_radii.size()) {
      if (error) {
        *error = "obstacle " + std::to_string(obstacle.id) +
          " needs as many sphere_radii as sphere_centers (at least one)";
      }
      return false;
    }
    const auto& p = obstacle.pose.position;
    const auto& o = obstacle.pose.orientation;
    bool ok = finite(p.x) && finite(p.y) && finite(p.z) && finite(o.w) && finite(o.x) && finite(o.y) && finite(o.z);
    ok = ok && (o.w * o.w + o.x * o.x + o.y * o.y + o.z * o.z) > 1e-12;
    const auto& v = obstacle.twist.linear;
    const auto& w = obstacle.twist.angular;
    ok = ok && finite(v.x) && finite(v.y) && finite(v.z) && finite(w.x) && finite(w.y) && finite(w.z);
    for (const auto& c : obstacle.sphere_centers) {
      ok = ok && finite(c.x) && finite(c.y) && finite(c.z);
    }
    for (const double r : obstacle.sphere_radii) {
      ok = ok && finite(r) && r >= 0.0;
    }
    if (!ok) {
      if (error) {
        *error = "obstacle " + std::to_string(obstacle.id) +
          " has a non-finite pose, twist or sphere, a zero quaternion or a negative radius";
      }
      return false;
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = std::make_shared<const Msg>(msg);
  latest_receive_time_ = receiveTime;
  return true;
}

void DynamicObstacles::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_.reset();
}

void DynamicObstacles::takeSnapshot(double initTime, DynamicObstacleSnapshot& out)
{
  out.init_time = initTime;
  out.predict = settings_.predict;
  out.max_prediction_time = settings_.max_prediction_time;
  out.spheres.clear();

  std::shared_ptr<const Msg> msg;
  double receive_time = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg = latest_;
    receive_time = latest_receive_time_;
  }
  if (!msg || initTime - receive_time > settings_.timeout) {
    std::fill(slots_.begin(), slots_.end(), Slot{});
    active_ = 0;
    dropped_ = 0;
    out.stamp = initTime;
    out.slot_ids.assign(slots_.size(), -1);
  } else {
    // A zero stamp means "now": the receive time stands in for it.
    out.stamp = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
      receive_time : stampSeconds(msg->header.stamp);

    // Beyond the slot count, keep the obstacles nearest the frame origin (the arm base).
    const auto& obstacles = msg->obstacles;
    std::vector<std::size_t> order(obstacles.size());
    std::iota(order.begin(), order.end(), 0);
    auto nearest = [&](std::size_t i) {
      const auto& p = obstacles[i].pose.position;
      return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    };
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) { return nearest(a) < nearest(b); });
    if (order.size() > settings_.max_obstacles) {
      dropped_ = order.size() - settings_.max_obstacles;
      order.resize(settings_.max_obstacles);
    } else {
      dropped_ = 0;
    }

    // Slots: an id that stays keeps its slot; ids no longer present free theirs; new ids take free ones.
    std::unordered_map<int, std::size_t> kept;  // id -> index into obstacles
    for (const std::size_t i : order) {
      kept.emplace(obstacles[i].id, i);
    }
    std::vector<std::size_t> slot_obstacle(slots_.size(), obstacles.size());  // slot -> obstacle index
    for (std::size_t s = 0; s < slots_.size(); ++s) {
      if (!slots_[s].used) {
        continue;
      }
      const auto found = kept.find(slots_[s].id);
      if (found == kept.end()) {
        slots_[s] = Slot{};
      } else {
        slot_obstacle[s] = found->second;
        kept.erase(found);
      }
    }
    for (const std::size_t i : order) {
      if (kept.count(obstacles[i].id) == 0) {
        continue;  // already in a slot (or a duplicate id)
      }
      kept.erase(obstacles[i].id);
      for (std::size_t s = 0; s < slots_.size(); ++s) {
        if (!slots_[s].used) {
          slots_[s] = Slot{true, obstacles[i].id, initTime};
          slot_obstacle[s] = i;
          break;
        }
      }
    }

    std::size_t active = 0;
    out.slot_ids.assign(slots_.size(), -1);
    for (std::size_t s = 0; s < slots_.size(); ++s) {
      if (!slots_[s].used) {
        continue;
      }
      ++active;
      out.slot_ids[s] = slots_[s].id;
      const auto& obstacle = obstacles[slot_obstacle[s]];
      const double weight = settings_.fade_in_time > 0.0 ?
        std::clamp((initTime - slots_[s].first_seen) / settings_.fade_in_time, 0.0, 1.0) : 1.0;
      const double inflation = settings_.position_std_scale * std::max(0.0, obstacle.position_std);
      const std::size_t spheres = std::min(settings_.max_spheres, obstacle.sphere_centers.size());
      for (std::size_t j = 0; j < spheres; ++j) {
        DynamicObstacleSnapshot::Sphere sphere;
        sphere.obstacle_id = obstacle.id;
        sphere.slot = s;
        sphere.position = Eigen::Vector3d(obstacle.pose.position.x, obstacle.pose.position.y, obstacle.pose.position.z);
        sphere.orientation = Eigen::Quaterniond(
          obstacle.pose.orientation.w, obstacle.pose.orientation.x, obstacle.pose.orientation.y,
          obstacle.pose.orientation.z).normalized();
        sphere.linear_velocity = Eigen::Vector3d(obstacle.twist.linear.x, obstacle.twist.linear.y, obstacle.twist.linear.z);
        sphere.angular_velocity =
          Eigen::Vector3d(obstacle.twist.angular.x, obstacle.twist.angular.y, obstacle.twist.angular.z);
        const auto& c = obstacle.sphere_centers[j];
        sphere.center_in_object = Eigen::Vector3d(c.x, c.y, c.z);
        sphere.radius = obstacle.sphere_radii[j] + inflation;
        sphere.weight = weight;
        out.spheres.push_back(sphere);
      }
    }
    active_ = active;
  }

  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  last_snapshot_ = out;
  has_snapshot_ = true;
}

bool DynamicObstacles::lastSnapshot(DynamicObstacleSnapshot& out) const
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  if (!has_snapshot_) {
    return false;
  }
  out = last_snapshot_;
  return true;
}

DynamicObstacleSynchronizedModule::DynamicObstacleSynchronizedModule(
  std::shared_ptr<DynamicObstacles> obstacles,
  std::shared_ptr<DynamicObstacleSnapshot> snapshot)
: obstacles_(std::move(obstacles)), snapshot_(std::move(snapshot))
{
}

void DynamicObstacleSynchronizedModule::preSolverRun(
  ocs2::scalar_t initTime,
  ocs2::scalar_t,
  const ocs2::vector_t&,
  const ocs2::ReferenceManagerInterface&)
{
  obstacles_->takeSnapshot(initTime, *snapshot_);
}

}  // namespace constraint
}  // namespace dynamics_mpc_controller
