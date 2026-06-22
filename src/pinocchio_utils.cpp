#include "dynamics_mpc_controller/pinocchio_utils.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_pinocchio_interface/urdf.h>
#include <urdf_parser/urdf_parser.h>

namespace dynamics_mpc_controller::pinocchio_utils
{

ocs2::PinocchioInterface createPinocchioInterface(
  const std::string& robotUrdfPath,
  const std::vector<std::string>& jointsToRemove)
{
  const auto urdf_tree = urdf::parseURDFFile(robotUrdfPath);
  if (!urdf_tree) {
    throw std::runtime_error("Failed to parse URDF: " + robotUrdfPath);
  }

  auto model = std::make_shared<urdf::ModelInterface>(*urdf_tree);
  for (auto& [joint_name, joint] : model->joints_) {
    const bool remove_joint =
      std::find(jointsToRemove.begin(), jointsToRemove.end(), joint_name) != jointsToRemove.end();
    if (remove_joint || joint->mimic) {
      joint->type = urdf::Joint::FIXED;
    }
  }

  return ocs2::getPinocchioInterfaceFromUrdfModel(std::move(model));
}

}  // namespace dynamics_mpc_controller::pinocchio_utils
