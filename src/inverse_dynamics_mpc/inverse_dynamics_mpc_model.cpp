#include "dynamics_mpc_controller/inverse_dynamics_mpc/inverse_dynamics_mpc_model.hpp"

#include <utility>

namespace dynamics_mpc_controller
{

InverseDynamicsMpcModel::InverseDynamicsMpcModel(
  std::size_t jointDim,
  std::vector<std::string> dofNames,
  std::string endEffectorFrame,
  pinocchio::FrameIndex endEffectorFrameId,
  bool wrenchInRnea,
  bool trackZeroWrench)
: joint_dim_(jointDim),
  dof_names_(std::move(dofNames)),
  end_effector_frame_(std::move(endEffectorFrame)),
  end_effector_frame_id_(endEffectorFrameId),
  wrench_in_rnea_(wrenchInRnea),
  track_zero_wrench_(trackZeroWrench)
{
}

ocs2::vector_t InverseDynamicsMpcModel::getQ(const ocs2::vector_t& state) const
{
  return state.segment(static_cast<Eigen::Index>(qOffset()), static_cast<Eigen::Index>(joint_dim_));
}

ocs2::vector_t InverseDynamicsMpcModel::getV(const ocs2::vector_t& state) const
{
  return state.segment(static_cast<Eigen::Index>(vOffset()), static_cast<Eigen::Index>(joint_dim_));
}

ocs2::vector_t InverseDynamicsMpcModel::getA(const ocs2::vector_t& input) const
{
  return input.segment(static_cast<Eigen::Index>(aOffset()), static_cast<Eigen::Index>(joint_dim_));
}

ocs2::vector_t InverseDynamicsMpcModel::getTau(const ocs2::vector_t& input) const
{
  return input.segment(static_cast<Eigen::Index>(tauOffset()), static_cast<Eigen::Index>(joint_dim_));
}

ocs2::vector_t InverseDynamicsMpcModel::getWrench(const ocs2::vector_t& input) const
{
  if (!wrench_in_rnea_) {
    throw std::logic_error("Cannot read an end-effector wrench from the no-wrench MPC input.");
  }
  return input.segment(static_cast<Eigen::Index>(wrenchOffset()), 6);
}

}  // namespace dynamics_mpc_controller
