#include "dynamics_mpc_controller/common/controller_utils.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace dynamics_mpc_controller::controller_utils
{

std::pair<std::string, std::string> resolveJointName(const std::string& name, bool hasPrefix)
{
  std::stringstream ss(name);
  std::string item;
  std::vector<std::string> parts;
  while (std::getline(ss, item, '_')) {
    parts.push_back(item);
  }

  std::string body_name;
  std::string joint_name;
  if (hasPrefix) {
    if (parts.size() >= 3) {
      body_name = parts[1];
      joint_name = parts[2];
      for (std::size_t i = 3; i < parts.size(); ++i) {
        joint_name += "_" + parts[i];
      }
    }
  } else {
    if (parts.size() >= 2) {
      body_name = parts[0];
      joint_name = parts[1];
      for (std::size_t i = 2; i < parts.size(); ++i) {
        joint_name += "_" + parts[i];
      }
    }
  }
  return {body_name, joint_name};
}

vector_t readStateVector(
  const std::vector<std::string>& dofNames,
  const HardwareInterfaceMap& hardwareInterfaces,
  const std::string& interfaceName)
{
  vector_t values(static_cast<Eigen::Index>(dofNames.size()));
  for (std::size_t i = 0; i < dofNames.size(); ++i) {
    const auto [body_name, joint_name] = resolveJointName(dofNames[i], true);
    values(static_cast<Eigen::Index>(i)) =
      hardwareInterfaces.at(body_name)
        .at(joint_name)
        .state.at(interfaceName)
        .get()
        .get_value();
  }
  return values;
}

void writeTorqueCommand(
  const std::vector<std::string>& dofNames,
  HardwareInterfaceMap& hardwareInterfaces,
  const vector_t& torque)
{
  for (std::size_t i = 0; i < dofNames.size(); ++i) {
    const auto [body_name, joint_name] = resolveJointName(dofNames[i], true);
    hardwareInterfaces.at(body_name)
      .at(joint_name)
      .command.at(hardware_interface::HW_IF_EFFORT)
      .get()
      .set_value(torque(static_cast<Eigen::Index>(i)));
  }
}

double maxBoundViolation(
  const vector_t& value,
  const vector_t& lowerBound,
  const vector_t& upperBound)
{
  const vector_t lower_violation = lowerBound - value;
  const vector_t upper_violation = value - upperBound;
  return std::max(0.0, std::max(lower_violation.maxCoeff(), upper_violation.maxCoeff()));
}

vector_t computeJointSpaceImpedanceTorque(
  const vector_t& position,
  const vector_t& velocity,
  const vector_t& referencePosition,
  const vector_t& stiffness,
  const vector_t& damping)
{
  return stiffness.cwiseProduct(referencePosition - position) -
    damping.cwiseProduct(velocity);
}

bool commandIsStale(
  bool commandReceived,
  double currentTimeSec,
  double latestCommandTimeSec,
  double timeoutSec)
{
  if (!commandReceived || timeoutSec <= 0.0) {
    return false;
  }
  if (!std::isfinite(currentTimeSec) || !std::isfinite(latestCommandTimeSec)) {
    return true;
  }
  return (currentTimeSec - latestCommandTimeSec) >= timeoutSec;
}

void publishMpcObservation(
  const ocs2::SystemObservation& observation,
  const std::shared_ptr<realtime_tools::RealtimePublisher<ocs2_msgs::msg::MpcObservation>>& publisher)
{
  if (!publisher) {
    return;
  }

  const auto msg = ocs2::ros_msg_conversions::createObservationMsg(observation);
#if REALTIME_TOOLS_NEW_API
  publisher->try_publish(msg);
#else
  if (publisher->trylock()) {
    publisher->msg_ = msg;
    publisher->unlockAndPublish();
  }
#endif
}

void publishWrenchEstimate(
  const rclcpp::Time& stamp,
  const std::string& frameId,
  const vector_t& wrench,
  const std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>& publisher)
{
  if (!publisher || wrench.size() < 6) {
    return;
  }

  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frameId;
  msg.wrench.force.x = wrench(0);
  msg.wrench.force.y = wrench(1);
  msg.wrench.force.z = wrench(2);
  msg.wrench.torque.x = wrench(3);
  msg.wrench.torque.y = wrench(4);
  msg.wrench.torque.z = wrench(5);
#if REALTIME_TOOLS_NEW_API
  publisher->try_publish(msg);
#else
  if (publisher->trylock()) {
    publisher->msg_ = msg;
    publisher->unlockAndPublish();
  }
#endif
}

}  // namespace dynamics_mpc_controller::controller_utils
