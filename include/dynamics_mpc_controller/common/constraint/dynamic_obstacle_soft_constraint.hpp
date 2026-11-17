#ifndef DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__DYNAMIC_OBSTACLE_SOFT_CONSTRAINT_HPP_
#define DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__DYNAMIC_OBSTACLE_SOFT_CONSTRAINT_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/multibody/geometry.hpp>

#include "dynamics_mpc_controller/common/constraint/dynamic_obstacles.hpp"

namespace dynamics_mpc_controller
{
namespace constraint
{

// Keeps the arm's collision primitives away from the tracked obstacle spheres:
//
//   cost(t, x) = sum_{g, i} w_i * RBF(h_gi),   h_gi = d(g(q), s_i(t)) - minimumDistance
//
// over the URDF collision primitives g of the configured links and the obstacle spheres s_i of the current snapshot,
// with d the signed coal distance between the primitive and the sphere (center predicted at the node time t, radius
// inflated by the tracker's position std), RBF the relaxed log barrier (mu, delta) and w_i the sphere's fade-in weight.
// Gradient dh/dq = -n' J_p, with n the coal normal from the primitive to the sphere and J_p the Jacobian of the
// primitive's witness point; Hessian Gauss-Newton, RBF''(h) dh' dh, as OCS2's soft constraints.
class DynamicObstacleSoftConstraint final : public ocs2::StateCost
{
public:
  DynamicObstacleSoftConstraint(
    const ocs2::PinocchioInterface& pinocchioInterface,
    std::size_t jointDim,
    const std::vector<std::string>& links,
    ocs2::scalar_t minimumDistance,
    ocs2::scalar_t mu,
    ocs2::scalar_t delta,
    std::shared_ptr<const DynamicObstacleSnapshot> snapshot);

  DynamicObstacleSoftConstraint* clone() const override { return new DynamicObstacleSoftConstraint(*this); }

  bool isActive(ocs2::scalar_t time) const override;

  ocs2::scalar_t getValue(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    const ocs2::TargetTrajectories& targetTrajectories,
    const ocs2::PreComputation& preComputation) const override;

  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    const ocs2::TargetTrajectories& targetTrajectories,
    const ocs2::PreComputation& preComputation) const override;

  std::size_t numPrimitives() const { return geometry_ids_.size(); }

private:
  DynamicObstacleSoftConstraint(const DynamicObstacleSoftConstraint& rhs) = default;

  // Cost value; with derivatives, also its gradient and Gauss-Newton Hessian in the state.
  ocs2::scalar_t evaluate(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    ocs2::vector_t* gradient,
    ocs2::matrix_t* hessian) const;

  mutable ocs2::PinocchioInterface pinocchio_interface_;  // own data per clone, i.e. per solver thread
  std::shared_ptr<const pinocchio::GeometryModel> geometry_model_;
  std::vector<pinocchio::GeomIndex> geometry_ids_;
  std::size_t joint_dim_{0};
  ocs2::scalar_t minimum_distance_{0.0};
  std::shared_ptr<const ocs2::RelaxedBarrierPenalty> penalty_;  // stateless, shared by the clones
  std::shared_ptr<const DynamicObstacleSnapshot> snapshot_;
};

}  // namespace constraint
}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__COMMON__CONSTRAINT__DYNAMIC_OBSTACLE_SOFT_CONSTRAINT_HPP_
