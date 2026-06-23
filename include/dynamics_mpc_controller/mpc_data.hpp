#ifndef DYNAMICS_MPC_CONTROLLER__MPC_DATA_HPP_
#define DYNAMICS_MPC_CONTROLLER__MPC_DATA_HPP_

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace dynamics_mpc_controller
{

struct HWInterfaces
{
  std::unordered_map<
    std::string,
    std::reference_wrapper<hardware_interface::LoanedStateInterface>> state;
  std::unordered_map<
    std::string,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command;
};

struct MPCData
{
  std::string lib_folder_;
  std::string urdf_file_;
  double command_smoothing_alpha_{1.0};
  bool gravity_compensation_only_{false};
  std::vector<double> hold_velocity_damping_;
  std::string target_trajectories_topic_{"/target_trajectories"};
  std::string mpc_observation_topic_{"/inverse_dynamics_mpc_observation"};
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__MPC_DATA_HPP_
