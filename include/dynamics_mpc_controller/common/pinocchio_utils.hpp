#ifndef DYNAMICS_MPC_CONTROLLER__PINOCCHIO_UTILS_HPP_
#define DYNAMICS_MPC_CONTROLLER__PINOCCHIO_UTILS_HPP_

#include <string>
#include <vector>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace dynamics_mpc_controller::pinocchio_utils
{

ocs2::PinocchioInterface createPinocchioInterface(
  const std::string& robotUrdfPath,
  const std::vector<std::string>& jointsToRemove);

}  // namespace dynamics_mpc_controller::pinocchio_utils

#endif  // DYNAMICS_MPC_CONTROLLER__PINOCCHIO_UTILS_HPP_
