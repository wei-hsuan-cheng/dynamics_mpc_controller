#include "dynamics_mpc_controller/inverse_dynamics_mpc/initialization/inverse_dynamics_initializer.hpp"

#include <algorithm>
#include <utility>

#include <pinocchio/algorithm/rnea.hpp>

namespace dynamics_mpc_controller
{

InverseDynamicsInitializer::InverseDynamicsInitializer(
  const ocs2::PinocchioInterface& pinocchioInterface,
  InverseDynamicsMpcModel model)
: pinocchio_interface_(pinocchioInterface),
  model_(std::move(model))
{
}

InverseDynamicsInitializer* InverseDynamicsInitializer::clone() const
{
  return new InverseDynamicsInitializer(*this);
}

void InverseDynamicsInitializer::compute(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  ocs2::scalar_t nextTime,
  ocs2::vector_t& input,
  ocs2::vector_t& nextState)
{
  const auto n = static_cast<Eigen::Index>(model_.jointDim());
  const ocs2::scalar_t dt = std::max<ocs2::scalar_t>(0.0, nextTime - time);
  const ocs2::vector_t q = model_.getJointPosition(state);
  const ocs2::vector_t v = model_.getJointVelocity(state);

  const ocs2::vector_t acceleration = ocs2::vector_t::Zero(n);

  auto& data = pinocchio_interface_.getData();
  const auto& pinocchio_model = pinocchio_interface_.getModel();
  const ocs2::vector_t torque = pinocchio::rnea(pinocchio_model, data, q, v, acceleration);

  input = ocs2::vector_t::Zero(static_cast<Eigen::Index>(model_.inputDim()));
  input.segment(static_cast<Eigen::Index>(model_.aOffset()), n) = acceleration;
  input.segment(static_cast<Eigen::Index>(model_.tauOffset()), n) = torque;

  nextState = state;
  nextState.head(n) = q + dt * v + 0.5 * dt * dt * acceleration;
  nextState.tail(n) = v + dt * acceleration;
}

}  // namespace dynamics_mpc_controller
