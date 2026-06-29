#include "dynamics_mpc_controller/common/constraint/dynamics_self_collision_constraint.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

namespace dynamics_mpc_controller
{
namespace constraint
{

DynamicsSelfCollisionPinocchioMapping::DynamicsSelfCollisionPinocchioMapping(
  std::size_t jointDim)
: joint_dim_(jointDim)
{
}

DynamicsSelfCollisionPinocchioMapping*
DynamicsSelfCollisionPinocchioMapping::clone() const
{
  return new DynamicsSelfCollisionPinocchioMapping(*this);
}

DynamicsSelfCollisionPinocchioMapping::vector_t
DynamicsSelfCollisionPinocchioMapping::getPinocchioJointPosition(
  const vector_t& state) const
{
  return state.head(static_cast<Eigen::Index>(joint_dim_));
}

DynamicsSelfCollisionPinocchioMapping::vector_t
DynamicsSelfCollisionPinocchioMapping::getPinocchioJointVelocity(
  const vector_t& state,
  const vector_t&) const
{
  return state.segment(
    static_cast<Eigen::Index>(joint_dim_),
    static_cast<Eigen::Index>(joint_dim_));
}

std::pair<
  DynamicsSelfCollisionPinocchioMapping::matrix_t,
  DynamicsSelfCollisionPinocchioMapping::matrix_t>
DynamicsSelfCollisionPinocchioMapping::getOcs2Jacobian(
  const vector_t&,
  const matrix_t& Jq,
  const matrix_t& Jv) const
{
  ocs2::matrix_t dfdx = ocs2::matrix_t::Zero(
    Jq.rows(),
    static_cast<Eigen::Index>(2 * joint_dim_));
  dfdx.leftCols(static_cast<Eigen::Index>(joint_dim_)) = Jq;
  dfdx.rightCols(static_cast<Eigen::Index>(joint_dim_)) = Jv;

  return {dfdx, ocs2::matrix_t{}};
}

std::unique_ptr<ocs2::StateConstraint> createDynamicsSelfCollisionConstraint(
  const ocs2::PinocchioInterface& pinocchioInterface,
  std::size_t jointDim,
  const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
  const std::vector<std::pair<std::size_t, std::size_t>>& collisionObjectPairs,
  ocs2::scalar_t minimumDistance,
  const std::string& modelName,
  const std::string& modelFolder,
  bool recompileLibraries,
  bool verbose)
{
  if (collisionLinkPairs.empty() && collisionObjectPairs.empty()) {
    throw std::runtime_error(
      "[DynamicsSelfCollision] selfCollision.activate is true but no collision pairs are configured.");
  }

  ocs2::PinocchioGeometryInterface geometryInterface(
    pinocchioInterface,
    collisionLinkPairs,
    collisionObjectPairs);

  if (geometryInterface.getNumCollisionPairs() == 0) {
    throw std::runtime_error(
      "[DynamicsSelfCollision] configured self-collision pairs produced zero Pinocchio collision pairs.");
  }

  return std::make_unique<ocs2::SelfCollisionConstraintCppAd>(
    pinocchioInterface,
    DynamicsSelfCollisionPinocchioMapping(jointDim),
    std::move(geometryInterface),
    minimumDistance,
    modelName,
    modelFolder,
    recompileLibraries,
    verbose);
}

DynamicsSelfCollisionDistanceEvaluator::DynamicsSelfCollisionDistanceEvaluator(
  const ocs2::PinocchioInterface& pinocchioInterface,
  const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
  const std::vector<std::pair<std::size_t, std::size_t>>& collisionObjectPairs)
: pinocchio_interface_(pinocchioInterface),
  geometry_interface_(pinocchio_interface_, collisionLinkPairs, collisionObjectPairs)
{
  if (collisionLinkPairs.empty() && collisionObjectPairs.empty()) {
    throw std::runtime_error(
      "[DynamicsSelfCollision] selfCollision.activate is true but no collision pairs are configured.");
  }
  if (geometry_interface_.getNumCollisionPairs() == 0) {
    throw std::runtime_error(
      "[DynamicsSelfCollision] configured self-collision pairs produced zero Pinocchio collision pairs.");
  }
}

std::size_t DynamicsSelfCollisionDistanceEvaluator::numCollisionPairs() const
{
  return geometry_interface_.getNumCollisionPairs();
}

ocs2::scalar_t DynamicsSelfCollisionDistanceEvaluator::computeMinimumDistance(
  const ocs2::vector_t& q)
{
  auto& data = pinocchio_interface_.getData();
  const auto& model = pinocchio_interface_.getModel();
  if (q.size() != model.nq) {
    throw std::invalid_argument(
      "[DynamicsSelfCollision] q dimension does not match the Pinocchio model.");
  }

  pinocchio::forwardKinematics(model, data, q);
  const auto distance_results = geometry_interface_.computeDistances(pinocchio_interface_);
  ocs2::scalar_t minimum_distance = std::numeric_limits<ocs2::scalar_t>::infinity();
  for (const auto& result : distance_results) {
    minimum_distance = std::min(minimum_distance, static_cast<ocs2::scalar_t>(result.min_distance));
  }
  return minimum_distance;
}

}  // namespace constraint
}  // namespace dynamics_mpc_controller
