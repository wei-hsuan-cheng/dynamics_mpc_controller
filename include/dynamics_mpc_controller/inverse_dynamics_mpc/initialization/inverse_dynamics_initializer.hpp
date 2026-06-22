#ifndef DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INITIALIZATION__INVERSE_DYNAMICS_INITIALIZER_HPP_
#define DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INITIALIZATION__INVERSE_DYNAMICS_INITIALIZER_HPP_

#include <cstddef>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "dynamics_mpc_controller/inverse_dynamics_mpc/inverse_dynamics_mpc_model.hpp"

namespace dynamics_mpc_controller
{

class InverseDynamicsInitializer final : public ocs2::Initializer
{
public:
  InverseDynamicsInitializer(
    const ocs2::PinocchioInterface& pinocchioInterface,
    InverseDynamicsMpcModel model,
    ocs2::vector_t accelerationLowerBound,
    ocs2::vector_t accelerationUpperBound,
    ocs2::vector_t velocityDamping);

  ~InverseDynamicsInitializer() override = default;
  InverseDynamicsInitializer* clone() const override;

  void compute(
    ocs2::scalar_t time,
    const ocs2::vector_t& state,
    ocs2::scalar_t nextTime,
    ocs2::vector_t& input,
    ocs2::vector_t& nextState) override;

private:
  InverseDynamicsInitializer(const InverseDynamicsInitializer& rhs) = default;

  ocs2::PinocchioInterface pinocchio_interface_;
  InverseDynamicsMpcModel model_;
  ocs2::vector_t acceleration_lower_bound_;
  ocs2::vector_t acceleration_upper_bound_;
  ocs2::vector_t velocity_damping_;
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INITIALIZATION__INVERSE_DYNAMICS_INITIALIZER_HPP_
