#include "dynamics_mpc_controller/common/constraint/dynamics_self_collision_constraint.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/augmented_lagrangian/StateAugmentedLagrangian.h>
#include <ocs2_core/penalties/augmented/ModifiedRelaxedBarrierPenalty.h>
#include <ocs2_core/penalties/augmented/SlacknessSquaredHingePenalty.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_core/penalties/penalties/SquaredHingePenalty.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>

namespace dynamics_mpc_controller
{
namespace constraint
{

void validateDynamicsSelfCollisionConstraintSettings(
  const std::string& solverType,
  const DynamicsSelfCollisionConstraintSettings& settings)
{
  const bool supported_implementation =
    settings.implementation == "hard" ||
    settings.implementation == "rbf" ||
    settings.implementation == "al";
  if (!supported_implementation) {
    throw std::runtime_error(
      "[DynamicsSelfCollision] unsupported selfCollision.implementation '" +
      settings.implementation + "'. Expected one of: hard, rbf, al.");
  }

  if (solverType == "ddp" && settings.implementation == "hard") {
    throw std::runtime_error(
      "[DynamicsSelfCollision] selfCollision.implementation='hard' is not supported with solverType='ddp'. "
      "Use 'al' or 'rbf', or switch solverType to 'sqp'.");
  }

  if (solverType == "sqp") {
    return;
  }
  if (solverType == "ddp") {
    return;
  }

  throw std::runtime_error(
    "[DynamicsSelfCollision] unsupported solverType '" + solverType +
    "' for selfCollision. Expected 'ddp' or 'sqp'.");
}

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

void addDynamicsSelfCollisionConstraint(
  ocs2::OptimalControlProblem& problem,
  std::unique_ptr<ocs2::StateConstraint> constraint,
  const std::string& solverType,
  const DynamicsSelfCollisionConstraintSettings& settings)
{
  if (settings.implementation == "hard") {
    problem.stateInequalityConstraintPtr->add("selfCollision", std::move(constraint));
    return;
  }

  if (settings.implementation == "rbf") {
    problem.stateSoftConstraintPtr->add(
      "selfCollision",
      std::make_unique<ocs2::StateSoftConstraint>(
        std::move(constraint),
        std::make_unique<ocs2::RelaxedBarrierPenalty>(
          ocs2::RelaxedBarrierPenalty::Config{settings.mu, settings.delta})));
    return;
  }

  if (settings.implementation == "al") {
    if (solverType == "sqp") {
      if (
        settings.penaltyType == "slacknesssquaredhingepenalty" ||
        settings.penaltyType == "slacknesssquaredhinge" ||
        settings.penaltyType == "hinge")
      {
        problem.stateSoftConstraintPtr->add(
          "selfCollision",
          std::make_unique<ocs2::StateSoftConstraint>(
            std::move(constraint),
            std::make_unique<ocs2::SquaredHingePenalty>(
              ocs2::SquaredHingePenalty::Config{settings.scale, 0.0})));
        return;
      }

      if (
        settings.penaltyType == "modifiedrelaxedbarrierpenalty" ||
        settings.penaltyType == "modifiedrelaxedbarrier" ||
        settings.penaltyType == "barrier")
      {
        problem.stateSoftConstraintPtr->add(
          "selfCollision",
          std::make_unique<ocs2::StateSoftConstraint>(
            std::move(constraint),
            std::make_unique<ocs2::RelaxedBarrierPenalty>(
              ocs2::RelaxedBarrierPenalty::Config{settings.scale, settings.relaxation})));
        return;
      }

      throw std::runtime_error(
        "[DynamicsSelfCollision] unsupported selfCollision.penaltyType '" +
        settings.penaltyType + "' for implementation 'al'.");
    }

    if (
      settings.penaltyType == "slacknesssquaredhingepenalty" ||
      settings.penaltyType == "slacknesssquaredhinge" ||
      settings.penaltyType == "hinge")
    {
      problem.stateInequalityLagrangianPtr->add(
        "selfCollision",
        std::make_unique<ocs2::StateAugmentedLagrangian>(
          std::move(constraint),
          ocs2::augmented::SlacknessSquaredHingePenalty::create(
            {settings.scale, settings.stepSize})));
      return;
    }

    if (
      settings.penaltyType == "modifiedrelaxedbarrierpenalty" ||
      settings.penaltyType == "modifiedrelaxedbarrier" ||
      settings.penaltyType == "barrier")
    {
      problem.stateInequalityLagrangianPtr->add(
        "selfCollision",
        std::make_unique<ocs2::StateAugmentedLagrangian>(
          std::move(constraint),
          ocs2::augmented::ModifiedRelaxedBarrierPenalty::create(
            {settings.scale, settings.relaxation, settings.stepSize})));
      return;
    }

    throw std::runtime_error(
      "[DynamicsSelfCollision] unsupported selfCollision.penaltyType '" +
      settings.penaltyType + "' for implementation 'al'.");
  }

  throw std::runtime_error(
    "[DynamicsSelfCollision] unsupported selfCollision.implementation '" +
    settings.implementation + "'. Expected one of: hard, rbf, al.");
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
