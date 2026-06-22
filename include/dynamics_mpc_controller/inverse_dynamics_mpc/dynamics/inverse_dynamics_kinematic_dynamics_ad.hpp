#ifndef DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__DYNAMICS__INVERSE_DYNAMICS_KINEMATIC_DYNAMICS_AD_HPP_
#define DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__DYNAMICS__INVERSE_DYNAMICS_KINEMATIC_DYNAMICS_AD_HPP_

#include <cstddef>
#include <string>

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>

namespace dynamics_mpc_controller
{

class InverseDynamicsKinematicDynamicsAD final : public ocs2::SystemDynamicsBaseAD
{
public:
  InverseDynamicsKinematicDynamicsAD(
    std::size_t jointDim,
    std::size_t inputDim,
    const std::string& modelName,
    const std::string& modelFolder = "/tmp/ocs2",
    bool recompileLibraries = true,
    bool verbose = true);

  ~InverseDynamicsKinematicDynamicsAD() override = default;
  InverseDynamicsKinematicDynamicsAD* clone() const override { return new InverseDynamicsKinematicDynamicsAD(*this); }

private:
  InverseDynamicsKinematicDynamicsAD(const InverseDynamicsKinematicDynamicsAD& rhs) = default;

  ocs2::ad_vector_t systemFlowMap(
    ocs2::ad_scalar_t time,
    const ocs2::ad_vector_t& state,
    const ocs2::ad_vector_t& input,
    const ocs2::ad_vector_t& parameters) const override;

  std::size_t joint_dim_{0};
  std::size_t input_dim_{0};
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__DYNAMICS__INVERSE_DYNAMICS_KINEMATIC_DYNAMICS_AD_HPP_
