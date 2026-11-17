#ifndef DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__DYNAMIC_OBSTACLES_HPP_
#define DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__DYNAMIC_OBSTACLES_HPP_

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <ocs2_core/Types.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "dynamics_mpc_controller/msg/tracked_obstacle_array.hpp"

// Tracked moving obstacles for the inverse-dynamics MPC: the latest dynamics_mpc_controller/TrackedObstacleArray,
// kept on a fixed number of obstacle slots and predicted over the horizon.
//
// The subscription stores each message with update(). Right before every solve, the synchronized module takes a
// snapshot at the MPC start time, and the obstacle cost reads only that snapshot during the solve:
//   - an obstacle keeps its slot while its id stays in the messages; a new id takes a free slot, a missing id frees
//     its slot; beyond maxObstacles the obstacles closest to the frame origin (the arm base) are kept;
//   - the pose is extrapolated at constant velocity from the message stamp, for at most maxPredictionTime;
//   - radii grow by positionStdScale * position_std;
//   - a new obstacle's weight ramps from 0 to 1 over fadeInTime, so the plan does not jump;
//   - no message for timeout seconds clears every slot.

namespace dynamics_mpc_controller
{
namespace constraint
{

struct DynamicObstacleSettings
{
  std::size_t max_obstacles{4};
  std::size_t max_spheres{2};
  std::string frame_id{"world"};
  double timeout{0.5};
  bool predict{true};
  double max_prediction_time{1.0};
  double fade_in_time{0.2};
  double position_std_scale{2.0};
};

// Obstacle spheres at one MPC start time, with what is needed to predict them at any later time.
struct DynamicObstacleSnapshot
{
  struct Sphere
  {
    int obstacle_id{0};
    std::size_t slot{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};  // object frame origin at the stamp
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d center_in_object{Eigen::Vector3d::Zero()};
    double radius{0.0};  // inflated by positionStdScale * position_std
    double weight{1.0};  // barrier weight, ramps up over fadeInTime
  };

  double init_time{0.0};
  double stamp{0.0};
  bool predict{true};
  double max_prediction_time{1.0};
  std::vector<Sphere> spheres;
  std::vector<int> slot_ids;  // per obstacle slot: the obstacle id, or -1 when empty

  // World position of a sphere's center at time t (constant velocity from the stamp).
  Eigen::Vector3d center(const Sphere& sphere, double time) const;
};

class DynamicObstacles
{
public:
  using Msg = dynamics_mpc_controller::msg::TrackedObstacleArray;

  explicit DynamicObstacles(DynamicObstacleSettings settings);

  const DynamicObstacleSettings& settings() const { return settings_; }

  // Stores the message as the current obstacle set (subscription thread). Returns false (and why) for a message that
  // is rejected: wrong frame, or malformed spheres; the previous set then stays until the timeout.
  bool update(const Msg& msg, double receiveTime, std::string* error);
  void clear();

  // The obstacle set at the MPC start time (MPC thread, before the solve).
  void takeSnapshot(double initTime, DynamicObstacleSnapshot& out);

  // Obstacles in slots at the last snapshot, and how many were dropped for lack of slots.
  std::size_t activeObstacles() const { return active_.load(); }
  std::size_t droppedObstacles() const { return dropped_.load(); }

  // The last snapshot, for visualization. False until the first one.
  bool lastSnapshot(DynamicObstacleSnapshot& out) const;

private:
  struct Slot
  {
    bool used{false};
    int id{0};
    double first_seen{0.0};
  };

  DynamicObstacleSettings settings_;

  std::mutex mutex_;
  std::shared_ptr<const Msg> latest_;
  double latest_receive_time_{0.0};

  mutable std::mutex snapshot_mutex_;
  DynamicObstacleSnapshot last_snapshot_;
  bool has_snapshot_{false};

  std::vector<Slot> slots_;  // MPC thread only
  std::atomic<std::size_t> active_{0};
  std::atomic<std::size_t> dropped_{0};
};

// Refreshes the snapshot the obstacle cost reads, right before every solve. The cost (and its per-thread clones)
// share the snapshot; the solver does not evaluate costs while preSolverRun() runs.
class DynamicObstacleSynchronizedModule final : public ocs2::SolverSynchronizedModule
{
public:
  DynamicObstacleSynchronizedModule(
    std::shared_ptr<DynamicObstacles> obstacles,
    std::shared_ptr<DynamicObstacleSnapshot> snapshot);

  void preSolverRun(
    ocs2::scalar_t initTime,
    ocs2::scalar_t finalTime,
    const ocs2::vector_t& initState,
    const ocs2::ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const ocs2::PrimalSolution&) override {}

private:
  std::shared_ptr<DynamicObstacles> obstacles_;
  std::shared_ptr<DynamicObstacleSnapshot> snapshot_;
};

}  // namespace constraint
}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__DYNAMIC_OBSTACLES_HPP_
