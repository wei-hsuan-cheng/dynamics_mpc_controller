#include "dynamics_mpc_controller/forward_dynamics_mpc/initialization/forward_dynamics_initializer.hpp"

#include <algorithm>
#include <utility>

#include <pinocchio/algorithm/rnea.hpp>

namespace dynamics_mpc_controller
{

ForwardDynamicsInitializer::ForwardDynamicsInitializer(
  const ocs2::PinocchioInterface& pinocchioInterface,
  ForwardDynamicsMpcModel model,
  ocs2::vector_t accelerationLowerBound,
  ocs2::vector_t accelerationUpperBound,
  ocs2::vector_t velocityDamping)
: pinocchio_interface_(pinocchioInterface),
  model_(std::move(model)),
  acceleration_lower_bound_(std::move(accelerationLowerBound)),
  acceleration_upper_bound_(std::move(accelerationUpperBound)),
  velocity_damping_(std::move(velocityDamping))
{
}

ForwardDynamicsInitializer* ForwardDynamicsInitializer::clone() const
{
  return new ForwardDynamicsInitializer(*this);
}

void ForwardDynamicsInitializer::compute(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  ocs2::scalar_t nextTime,
  ocs2::vector_t& input,
  ocs2::vector_t& nextState)
{
  const auto n = static_cast<Eigen::Index>(model_.jointDim());
  const ocs2::scalar_t dt = std::max<ocs2::scalar_t>(0.0, nextTime - time);
  const ocs2::vector_t q = model_.getQ(state);
  const ocs2::vector_t v = model_.getV(state);

  ocs2::vector_t acceleration = (-velocity_damping_.array() * v.array()).matrix();
  acceleration = acceleration.cwiseMax(acceleration_lower_bound_).cwiseMin(acceleration_upper_bound_);

  auto& data = pinocchio_interface_.getData();
  const auto& pinocchio_model = pinocchio_interface_.getModel();
  const ocs2::vector_t torque = pinocchio::rnea(pinocchio_model, data, q, v, acceleration);

  input = torque;

  nextState = state;
  nextState.head(n) = q + dt * v + 0.5 * dt * dt * acceleration;
  nextState.tail(n) = v + dt * acceleration;
}

}  // namespace dynamics_mpc_controller
