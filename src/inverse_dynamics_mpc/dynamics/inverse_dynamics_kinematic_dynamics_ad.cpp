#include "dynamics_mpc_controller/inverse_dynamics_mpc/dynamics/inverse_dynamics_kinematic_dynamics_ad.hpp"

namespace dynamics_mpc_controller
{

InverseDynamicsKinematicDynamicsAD::InverseDynamicsKinematicDynamicsAD(
  std::size_t jointDim,
  std::size_t inputDim,
  const std::string& modelName,
  const std::string& modelFolder,
  bool recompileLibraries,
  bool verbose)
: joint_dim_(jointDim), input_dim_(inputDim)
{
  initialize(2 * joint_dim_, input_dim_, modelName, modelFolder, recompileLibraries, verbose);
}

ocs2::ad_vector_t InverseDynamicsKinematicDynamicsAD::systemFlowMap(
  ocs2::ad_scalar_t,
  const ocs2::ad_vector_t& state,
  const ocs2::ad_vector_t& input,
  const ocs2::ad_vector_t&) const
{
  ocs2::ad_vector_t flow(2 * joint_dim_);
  flow.head(static_cast<Eigen::Index>(joint_dim_)) =
    state.segment(static_cast<Eigen::Index>(joint_dim_), static_cast<Eigen::Index>(joint_dim_));
  flow.tail(static_cast<Eigen::Index>(joint_dim_)) =
    input.head(static_cast<Eigen::Index>(joint_dim_));
  return flow;
}

}  // namespace dynamics_mpc_controller
