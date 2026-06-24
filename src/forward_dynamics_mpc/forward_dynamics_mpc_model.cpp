#include "dynamics_mpc_controller/forward_dynamics_mpc/forward_dynamics_mpc_model.hpp"

#include <utility>

namespace dynamics_mpc_controller
{

ForwardDynamicsMpcModel::ForwardDynamicsMpcModel(
  std::size_t jointDim,
  std::vector<std::string> dofNames,
  std::string endEffectorFrame,
  pinocchio::FrameIndex endEffectorFrameId)
: joint_dim_(jointDim),
  dof_names_(std::move(dofNames)),
  end_effector_frame_(std::move(endEffectorFrame)),
  end_effector_frame_id_(endEffectorFrameId)
{
}

ocs2::vector_t ForwardDynamicsMpcModel::getQ(const ocs2::vector_t& state) const
{
  return state.segment(static_cast<Eigen::Index>(qOffset()), static_cast<Eigen::Index>(joint_dim_));
}

ocs2::vector_t ForwardDynamicsMpcModel::getV(const ocs2::vector_t& state) const
{
  return state.segment(static_cast<Eigen::Index>(vOffset()), static_cast<Eigen::Index>(joint_dim_));
}

ocs2::vector_t ForwardDynamicsMpcModel::getTau(const ocs2::vector_t& input) const
{
  return input.segment(static_cast<Eigen::Index>(tauOffset()), static_cast<Eigen::Index>(joint_dim_));
}

}  // namespace dynamics_mpc_controller
