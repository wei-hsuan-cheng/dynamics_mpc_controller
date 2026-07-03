#ifndef DYNAMICS_MPC_CONTROLLER__VISUALIZATION__PERFORMANCE_VISUALIZATION_HPP_
#define DYNAMICS_MPC_CONTROLLER__VISUALIZATION__PERFORMANCE_VISUALIZATION_HPP_

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <cstddef>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/multibody/fwd.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_box_best_effort.hpp>

#include "dynamics_mpc_controller/visualization/optimized_state_trajectory_visualization.hpp"
#include "dynamics_mpc_controller/visualization/self_collision_visualization.hpp"

namespace dynamics_mpc_controller::visualization
{

class PerformanceVisualization
{
public:
  struct Settings
  {
    bool trajectory_active{true};
    bool self_collision_active{true};
    double update_rate{5.0};
    OptimizedStateTrajectoryVisualization::Settings trajectory;
    SelfCollisionVisualization::Settings self_collision;
  };

  PerformanceVisualization(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const ocs2::PinocchioInterface& pinocchioInterface,
    pinocchio::FrameIndex endEffectorFrameId,
    Settings settings);

  ~PerformanceVisualization();

  void update_visualization(
    const ocs2::vector_t& currentState,
    const ocs2::vector_array_t& optimizedStateTrajectory);

private:
  void visualization_loop();

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  Settings settings_;
  std::unique_ptr<OptimizedStateTrajectoryVisualization> trajectory_visualization_;
  std::unique_ptr<SelfCollisionVisualization> self_collision_visualization_;
  realtime_tools::RealtimeBoxBestEffort<ocs2::vector_t> latest_state_;
  realtime_tools::RealtimeBoxBestEffort<ocs2::vector_array_t> latest_optimized_state_trajectory_;
  std::thread visualization_thread_;
  std::chrono::duration<double> period_{0.2};
  std::atomic_bool running_{false};
  std::atomic_bool has_state_{false};
  std::atomic_bool has_optimized_state_trajectory_{false};
};

template <typename Params>
PerformanceVisualization::Settings makePerformanceVisualizationSettings(const Params& parameters)
{
  const auto& visualization = parameters.ocs2.task.visualization;
  const auto& self_collision = parameters.ocs2.task.selfCollision;

  PerformanceVisualization::Settings settings;
  settings.trajectory_active = visualization.trajectory.activate;
  settings.self_collision_active = visualization.collision.activate;
  settings.update_rate = visualization.update_rate;

  settings.trajectory.frame_id = visualization.frameId;
  settings.trajectory.marker_topic = visualization.trajectory.markerTopic;
  settings.trajectory.frame_names = visualization.trajectory.frame_names;
  settings.trajectory.line_width = visualization.trajectory.lineWidth;
  settings.trajectory.point_scale = visualization.trajectory.pointScale;

  settings.self_collision.frame_id = visualization.frameId;
  settings.self_collision.marker_topic = visualization.collision.markerTopic;
  settings.self_collision.minimum_distance = self_collision.minimumDistance;
  settings.self_collision.collision_link_pairs.reserve(self_collision.link_pair_names.size());
  for (const auto& pair_name : self_collision.link_pair_names) {
    if (pair_name.empty()) {
      continue;
    }
    const auto& entry = self_collision.link_pair_names_map.at(pair_name);
    settings.self_collision.collision_link_pairs.emplace_back(entry.link_a, entry.link_b);
  }
  settings.self_collision.collision_object_pairs.reserve(self_collision.object_pair_names.size());
  for (const auto& pair_name : self_collision.object_pair_names) {
    if (pair_name.empty()) {
      continue;
    }
    const auto& entry = self_collision.object_pair_names_map.at(pair_name);
    settings.self_collision.collision_object_pairs.emplace_back(
      static_cast<std::size_t>(entry.object_a),
      static_cast<std::size_t>(entry.object_b));
  }

  return settings;
}

}  // namespace dynamics_mpc_controller::visualization

#endif  // DYNAMICS_MPC_CONTROLLER__VISUALIZATION__PERFORMANCE_VISUALIZATION_HPP_
