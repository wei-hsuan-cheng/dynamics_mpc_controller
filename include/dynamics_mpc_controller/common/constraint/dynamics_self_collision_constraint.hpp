#ifndef DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__DYNAMICS_SELF_COLLISION_CONSTRAINT_HPP_
#define DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__DYNAMICS_SELF_COLLISION_CONSTRAINT_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

namespace dynamics_mpc_controller
{
namespace constraint
{

class DynamicsSelfCollisionPinocchioMapping final :
  public ocs2::PinocchioStateInputMapping<ocs2::scalar_t>
{
public:
  explicit DynamicsSelfCollisionPinocchioMapping(std::size_t jointDim);
  ~DynamicsSelfCollisionPinocchioMapping() override = default;

  DynamicsSelfCollisionPinocchioMapping* clone() const override;

  vector_t getPinocchioJointPosition(const vector_t& state) const override;
  vector_t getPinocchioJointVelocity(
    const vector_t& state,
    const vector_t& input) const override;
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(
    const vector_t& state,
    const matrix_t& Jq,
    const matrix_t& Jv) const override;

private:
  DynamicsSelfCollisionPinocchioMapping(const DynamicsSelfCollisionPinocchioMapping& rhs) = default;

  std::size_t joint_dim_{0};
};

std::unique_ptr<ocs2::StateConstraint> createDynamicsSelfCollisionConstraint(
  const ocs2::PinocchioInterface& pinocchioInterface,
  std::size_t jointDim,
  const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
  const std::vector<std::pair<std::size_t, std::size_t>>& collisionObjectPairs,
  ocs2::scalar_t minimumDistance,
  const std::string& modelName,
  const std::string& modelFolder,
  bool recompileLibraries,
  bool verbose);

class DynamicsSelfCollisionDistanceEvaluator
{
public:
  DynamicsSelfCollisionDistanceEvaluator(
    const ocs2::PinocchioInterface& pinocchioInterface,
    const std::vector<std::pair<std::string, std::string>>& collisionLinkPairs,
    const std::vector<std::pair<std::size_t, std::size_t>>& collisionObjectPairs);

  std::size_t numCollisionPairs() const;
  ocs2::scalar_t computeMinimumDistance(const ocs2::vector_t& q);

private:
  ocs2::PinocchioInterface pinocchio_interface_;
  ocs2::PinocchioGeometryInterface geometry_interface_;
};

}  // namespace constraint
}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__DYNAMICS_SELF_COLLISION_CONSTRAINT_HPP_
