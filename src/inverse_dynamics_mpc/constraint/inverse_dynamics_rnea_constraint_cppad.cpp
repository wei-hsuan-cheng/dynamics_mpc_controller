#include "dynamics_mpc_controller/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_constraint_cppad.hpp"

#include <pinocchio/algorithm/rnea.hpp>

namespace dynamics_mpc_controller
{

InverseDynamicsRneaConstraintCppAd::InverseDynamicsRneaConstraintCppAd(
  const ocs2::PinocchioInterface& pinocchioInterface,
  std::size_t jointDim,
  const std::string& modelName,
  const std::string& modelFolder,
  bool recompileLibraries,
  bool verbose)
: ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear),
  pinocchio_interface_cpp_ad_(pinocchioInterface.toCppAd()),
  joint_dim_(jointDim)
{
  initialize(2 * joint_dim_, 2 * joint_dim_, 0, modelName, modelFolder, recompileLibraries, verbose);
}

InverseDynamicsRneaConstraintCppAd::InverseDynamicsRneaConstraintCppAd(
  const InverseDynamicsRneaConstraintCppAd& rhs)
: ocs2::StateInputConstraintCppAd(rhs),
  pinocchio_interface_cpp_ad_(rhs.pinocchio_interface_cpp_ad_),
  joint_dim_(rhs.joint_dim_)
{
}

std::size_t InverseDynamicsRneaConstraintCppAd::getNumConstraints(ocs2::scalar_t) const
{
  return joint_dim_;
}

ocs2::ad_vector_t InverseDynamicsRneaConstraintCppAd::constraintFunction(
  ocs2::ad_scalar_t,
  const ocs2::ad_vector_t& state,
  const ocs2::ad_vector_t& input,
  const ocs2::ad_vector_t&) const
{
  const auto q = state.head(static_cast<Eigen::Index>(joint_dim_));
  const auto v = state.segment(
    static_cast<Eigen::Index>(joint_dim_), static_cast<Eigen::Index>(joint_dim_));
  const auto a = input.head(static_cast<Eigen::Index>(joint_dim_));
  const auto tau = input.segment(
    static_cast<Eigen::Index>(joint_dim_), static_cast<Eigen::Index>(joint_dim_));

  const auto& model = pinocchio_interface_cpp_ad_.getModel();
  auto data = pinocchio_interface_cpp_ad_.getData();
  return pinocchio::rnea(model, data, q, v, a) - tau;
}

}  // namespace dynamics_mpc_controller
