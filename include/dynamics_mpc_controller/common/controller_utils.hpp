#ifndef DYNAMICS_MPC_CONTROLLER__COMMON__CONTROLLER_UTILS_HPP_
#define DYNAMICS_MPC_CONTROLLER__COMMON__CONTROLLER_UTILS_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <rclcpp/time.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "dynamics_mpc_controller/common/mpc_data.hpp"

namespace dynamics_mpc_controller::controller_utils
{

using HardwareInterfaceMap = std::map<std::string, std::map<std::string, HWInterfaces>>;
using vector_t = ocs2::vector_t;

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
