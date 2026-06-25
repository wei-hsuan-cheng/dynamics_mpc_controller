#include "dynamics_mpc_controller/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_with_ee_wrench_constraint_cppad.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace dynamics_mpc_controller
{

InverseDynamicsRneaWithEeWrenchConstraintCppAd::InverseDynamicsRneaWithEeWrenchConstraintCppAd(
  const ocs2::PinocchioInterface& pinocchioInterface,
  pinocchio::FrameIndex endEffectorFrameId,
  std::size_t jointDim,
  const std::string& modelName,
  const std::string& modelFolder,
  bool recompileLibraries,
  bool verbose)
: ocs2::StateInputConstraintCppAd(ocs2::ConstraintOrder::Linear),
  pinocchio_interface_cpp_ad_(pinocchioInterface.toCppAd()),
  end_effector_frame_id_(endEffectorFrameId),
  joint_dim_(jointDim)
{
  initialize(2 * joint_dim_, 2 * joint_dim_ + 6, 0, modelName, modelFolder, recompileLibraries, verbose);
}

InverseDynamicsRneaWithEeWrenchConstraintCppAd::InverseDynamicsRneaWithEeWrenchConstraintCppAd(
  const InverseDynamicsRneaWithEeWrenchConstraintCppAd& rhs)
: ocs2::StateInputConstraintCppAd(rhs),
  pinocchio_interface_cpp_ad_(rhs.pinocchio_interface_cpp_ad_),
  end_effector_frame_id_(rhs.end_effector_frame_id_),
  joint_dim_(rhs.joint_dim_)
{
}

std::size_t InverseDynamicsRneaWithEeWrenchConstraintCppAd::getNumConstraints(ocs2::scalar_t) const
{
  return joint_dim_;
}

ocs2::ad_vector_t InverseDynamicsRneaWithEeWrenchConstraintCppAd::constraintFunction(
  ocs2::ad_scalar_t,
  const ocs2::ad_vector_t& state,
  const ocs2::ad_vector_t& input,
  const ocs2::ad_vector_t&) const
{
  using ad_scalar_t = ocs2::ad_scalar_t;

  const auto q = state.head(static_cast<Eigen::Index>(joint_dim_));
  const auto v = state.segment(
    static_cast<Eigen::Index>(joint_dim_), static_cast<Eigen::Index>(joint_dim_));
  const auto a = input.head(static_cast<Eigen::Index>(joint_dim_));
  const auto tau = input.segment(
    static_cast<Eigen::Index>(joint_dim_), static_cast<Eigen::Index>(joint_dim_));
  const auto wrench = input.segment(static_cast<Eigen::Index>(2 * joint_dim_), 6);

  const auto& model = pinocchio_interface_cpp_ad_.getModel();
  auto data = pinocchio_interface_cpp_ad_.getData();

  const ocs2::ad_vector_t rnea_tau = pinocchio::rnea(model, data, q, v, a);

  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  Eigen::Matrix<ad_scalar_t, 6, Eigen::Dynamic> frame_jacobian(6, model.nv);
  frame_jacobian.setZero();
  pinocchio::getFrameJacobian(
    model,
    data,
    end_effector_frame_id_,
    pinocchio::LOCAL_WORLD_ALIGNED,
    frame_jacobian);

  const ocs2::ad_vector_t ee_tau = frame_jacobian.transpose() * wrench;

  return rnea_tau - ee_tau - tau;
}

}  // namespace dynamics_mpc_controller
