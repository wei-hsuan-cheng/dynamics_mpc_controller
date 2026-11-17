#include "dynamics_mpc_controller/common/constraint/dynamic_obstacle_soft_constraint.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <coal/collision_data.h>
#include <coal/distance.h>
#include <coal/shape/geometric_shapes.h>

#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

namespace dynamics_mpc_controller
{
namespace constraint
{
namespace
{

// coal needs a sphere with some extent; a smaller radius is made up for in the distance.
constexpr double kMinimumSphereRadius = 1.0e-4;

}  // namespace

DynamicObstacleSoftConstraint::DynamicObstacleSoftConstraint(
  const ocs2::PinocchioInterface& pinocchioInterface,
  std::size_t jointDim,
  const std::vector<std::string>& links,
  ocs2::scalar_t minimumDistance,
  ocs2::scalar_t mu,
  ocs2::scalar_t delta,
  std::shared_ptr<const DynamicObstacleSnapshot> snapshot)
: pinocchio_interface_(pinocchioInterface),
  joint_dim_(jointDim),
  minimum_distance_(minimumDistance),
  penalty_(std::make_shared<const ocs2::RelaxedBarrierPenalty>(ocs2::RelaxedBarrierPenalty::Config{mu, delta})),
  snapshot_(std::move(snapshot))
{
  if (!snapshot_) {
    throw std::invalid_argument("[DynamicObstacleSoftConstraint] the obstacle snapshot must not be null.");
  }
  // The URDF collision geometry, as the self-collision constraint builds it; no collision pairs.
  ocs2::PinocchioGeometryInterface geometry_interface(pinocchioInterface, std::vector<std::pair<std::size_t, std::size_t>>{});
  geometry_model_ = std::make_shared<const pinocchio::GeometryModel>(geometry_interface.getGeometryModel());

  const auto& model = pinocchioInterface.getModel();
  for (const auto& link : links) {
    if (link.empty()) {
      continue;
    }
    bool found = false;
    for (pinocchio::GeomIndex g = 0; g < geometry_model_->geometryObjects.size(); ++g) {
      if (model.frames[geometry_model_->geometryObjects[g].parentFrame].name == link) {
        geometry_ids_.push_back(g);
        found = true;
      }
    }
    if (!found) {
      throw std::runtime_error(
        "[DynamicObstacleSoftConstraint] dynamicObstacles.links: link '" + link + "' has no collision geometry.");
    }
  }
  if (geometry_ids_.empty()) {
    throw std::runtime_error("[DynamicObstacleSoftConstraint] dynamicObstacles.links is empty.");
  }
}

bool DynamicObstacleSoftConstraint::isActive(ocs2::scalar_t) const
{
  return !snapshot_->spheres.empty();
}

ocs2::scalar_t DynamicObstacleSoftConstraint::getValue(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::TargetTrajectories&,
  const ocs2::PreComputation&) const
{
  return evaluate(time, state, nullptr, nullptr);
}

ocs2::ScalarFunctionQuadraticApproximation DynamicObstacleSoftConstraint::getQuadraticApproximation(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::TargetTrajectories&,
  const ocs2::PreComputation&) const
{
  ocs2::ScalarFunctionQuadraticApproximation cost;
  cost.f = evaluate(time, state, &cost.dfdx, &cost.dfdxx);
  return cost;
}

ocs2::scalar_t DynamicObstacleSoftConstraint::evaluate(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  ocs2::vector_t* gradient,
  ocs2::matrix_t* hessian) const
{
  const bool derivatives = gradient != nullptr;
  const auto n = static_cast<Eigen::Index>(joint_dim_);
  if (derivatives) {
    gradient->setZero(state.size());
    hessian->setZero(state.size(), state.size());
  }
  const auto& spheres = snapshot_->spheres;
  if (spheres.empty()) {
    return 0.0;
  }

  const auto& model = pinocchio_interface_.getModel();
  auto& data = pinocchio_interface_.getData();
  const ocs2::vector_t q = state.head(n);
  if (derivatives) {
    pinocchio::computeJointJacobians(model, data, q);  // also the forward kinematics
  } else {
    pinocchio::forwardKinematics(model, data, q);
  }

  ocs2::matrix_t joint_jacobian(6, model.nv);
  Eigen::Matrix<double, 3, Eigen::Dynamic> point_jacobian(3, model.nv);
  ocs2::vector_t dhdq(model.nv);
  const coal::DistanceRequest request;
  ocs2::scalar_t cost = 0.0;
  for (const auto g : geometry_ids_) {
    const auto& object = geometry_model_->geometryObjects[g];
    const pinocchio::SE3 placement = data.oMi[object.parentJoint] * object.placement;
    const coal::Transform3s geometry_transform(placement.rotation(), placement.translation());
    if (derivatives) {
      joint_jacobian.setZero();
      pinocchio::getJointJacobian(
        model, data, object.parentJoint, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, joint_jacobian);
    }
    for (const auto& sphere : spheres) {
      if (sphere.weight <= 0.0) {
        continue;
      }
      const double radius = std::max(sphere.radius, kMinimumSphereRadius);
      const coal::Sphere shape(radius);
      const coal::Transform3s sphere_transform(
        coal::Matrix3s::Identity(), snapshot_->center(sphere, time));
      coal::DistanceResult result;
      coal::distance(object.geometry.get(), geometry_transform, &shape, sphere_transform, request, result);
      const double h = result.min_distance + (radius - sphere.radius) - minimum_distance_;
      cost += sphere.weight * penalty_->getValue(time, h);
      if (!derivatives) {
        continue;
      }
      // Moving the primitive's witness point p by dp changes the distance by -n' dp (n from primitive to sphere).
      Eigen::Vector3d normal = result.normal;
      if (!normal.allFinite() || normal.squaredNorm() < 0.25) {
        normal = sphere_transform.getTranslation() - result.nearest_points[0];
        const double length = normal.norm();
        if (length < 1e-9) {
          continue;
        }
        normal /= length;
      }
      const Eigen::Vector3d offset = result.nearest_points[0] - data.oMi[object.parentJoint].translation();
      point_jacobian = joint_jacobian.topRows(3) - ocs2::skewSymmetricMatrix(offset) * joint_jacobian.bottomRows(3);
      dhdq = -point_jacobian.transpose() * normal;
      gradient->head(n) += sphere.weight * penalty_->getDerivative(time, h) * dhdq;
      hessian->topLeftCorner(n, n) += sphere.weight * penalty_->getSecondDerivative(time, h) * dhdq * dhdq.transpose();
    }
  }
  return cost;
}

}  // namespace constraint
}  // namespace dynamics_mpc_controller
