#include "dynamics_mpc_controller/forward_dynamics_mpc/dynamics/forward_dynamics_aba_dynamics_ad.hpp"

#include <pinocchio/algorithm/aba.hpp>

namespace dynamics_mpc_controller
{

ForwardDynamicsAbaDynamicsAD::ForwardDynamicsAbaDynamicsAD(
  const ocs2::PinocchioInterface& pinocchioInterface,
  std::size_t jointDim,
  const std::string& modelName,
  const std::string& modelFolder,
  bool recompileLibraries,
  bool verbose)
: pinocchio_interface_cpp_ad_(pinocchioInterface.toCppAd()), joint_dim_(jointDim)
{
  initialize(2 * joint_dim_, joint_dim_, modelName, modelFolder, recompileLibraries, verbose);
}

ForwardDynamicsAbaDynamicsAD::ForwardDynamicsAbaDynamicsAD(
  const ForwardDynamicsAbaDynamicsAD& rhs)
: ocs2::SystemDynamicsBaseAD(rhs),
  pinocchio_interface_cpp_ad_(rhs.pinocchio_interface_cpp_ad_),
  joint_dim_(rhs.joint_dim_)
{
}

ocs2::ad_vector_t ForwardDynamicsAbaDynamicsAD::systemFlowMap(
  ocs2::ad_scalar_t,
  const ocs2::ad_vector_t& state,
  const ocs2::ad_vector_t& input,
  const ocs2::ad_vector_t&) const
{
  const auto q = state.head(static_cast<Eigen::Index>(joint_dim_));
  const auto v = state.segment(
    static_cast<Eigen::Index>(joint_dim_), static_cast<Eigen::Index>(joint_dim_));
  const auto tau = input.head(static_cast<Eigen::Index>(joint_dim_));

  const auto& model = pinocchio_interface_cpp_ad_.getModel();
  auto data = pinocchio_interface_cpp_ad_.getData();
  const ocs2::ad_vector_t acceleration = pinocchio::aba(model, data, q, v, tau);

  ocs2::ad_vector_t flow(2 * joint_dim_);
  flow.head(static_cast<Eigen::Index>(joint_dim_)) = v;
  flow.tail(static_cast<Eigen::Index>(joint_dim_)) = acceleration;
  return flow;
}

}  // namespace dynamics_mpc_controller
