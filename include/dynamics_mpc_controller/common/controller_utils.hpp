#ifndef DYNAMICS_MPC_CONTROLLER__COMMON__CONTROLLER_UTILS_HPP_
#define DYNAMICS_MPC_CONTROLLER__COMMON__CONTROLLER_UTILS_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "dynamics_mpc_controller/common/mpc_data.hpp"

namespace dynamics_mpc_controller::controller_utils
{

using HardwareInterfaceMap = std::map<std::string, std::map<std::string, HWInterfaces>>;
using vector_t = ocs2::vector_t;

enum class HoldLogSeverity
{
  Info,
  Warn,
  Error,
};

std::pair<std::string, std::string> resolveJointName(const std::string& name, bool hasPrefix);

vector_t readStateVector(
  const std::vector<std::string>& dofNames,
  const HardwareInterfaceMap& hardwareInterfaces,
  const std::string& interfaceName);

void writeTorqueCommand(
  const std::vector<std::string>& dofNames,
  HardwareInterfaceMap& hardwareInterfaces,
  const vector_t& torque);

double maxBoundViolation(
  const vector_t& value,
  const vector_t& lowerBound,
  const vector_t& upperBound);

vector_t computeJointSpaceImpedanceTorque(
  const vector_t& position,
  const vector_t& velocity,
  const vector_t& referencePosition,
  const vector_t& stiffness,
  const vector_t& damping);

vector_t resolveNegativeDampingFromStiffness(
  const vector_t& stiffness,
  const vector_t& damping);

pinocchio::SE3 computeEndEffectorPose(
  const ocs2::PinocchioInterface& pinocchioInterface,
  pinocchio::FrameIndex endEffectorFrameId,
  const vector_t& position);

vector_t computeCartesianImpedanceTorque(
  const ocs2::PinocchioInterface& pinocchioInterface,
  pinocchio::FrameIndex endEffectorFrameId,
  const vector_t& position,
  const vector_t& velocity,
  const pinocchio::SE3& referencePose,
  const vector_t& stiffness,
  const vector_t& damping);

bool commandIsStale(
  bool commandReceived,
  double currentTimeSec,
  double latestCommandTimeSec,
  double timeoutSec);

void logHoldCommand(
  rclcpp::Logger logger,
  rclcpp::Clock& clock,
  std::int64_t throttlePeriodMs,
  HoldLogSeverity severity,
  const std::string& controllerName,
  const std::string& reason,
  const std::string& impedanceMode);

void publishMpcObservation(
  const ocs2::SystemObservation& observation,
  const std::shared_ptr<realtime_tools::RealtimePublisher<ocs2_msgs::msg::MpcObservation>>& publisher);

void publishWrenchEstimate(
  const rclcpp::Time& stamp,
  const std::string& frameId,
  const vector_t& wrench,
  const std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>& publisher);

}  // namespace dynamics_mpc_controller::controller_utils

#endif  // DYNAMICS_MPC_CONTROLLER__COMMON__CONTROLLER_UTILS_HPP_
