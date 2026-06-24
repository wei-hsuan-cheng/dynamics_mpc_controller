#ifndef DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__DYNAMICS__FORWARD_DYNAMICS_ABA_DYNAMICS_AD_HPP_
#define DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__DYNAMICS__FORWARD_DYNAMICS_ABA_DYNAMICS_AD_HPP_

#include <cstddef>
#include <string>

#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace dynamics_mpc_controller
{

class ForwardDynamicsAbaDynamicsAD final : public ocs2::SystemDynamicsBaseAD
{
public:
  ForwardDynamicsAbaDynamicsAD(
    const ocs2::PinocchioInterface& pinocchioInterface,
    std::size_t jointDim,
    const std::string& modelName,
    const std::string& modelFolder = "/tmp/ocs2",
    bool recompileLibraries = true,
    bool verbose = true);

  ~ForwardDynamicsAbaDynamicsAD() override = default;
  ForwardDynamicsAbaDynamicsAD* clone() const override { return new ForwardDynamicsAbaDynamicsAD(*this); }

private:
  ForwardDynamicsAbaDynamicsAD(const ForwardDynamicsAbaDynamicsAD& rhs);

  ocs2::ad_vector_t systemFlowMap(
    ocs2::ad_scalar_t time,
    const ocs2::ad_vector_t& state,
    const ocs2::ad_vector_t& input,
    const ocs2::ad_vector_t& parameters) const override;

  ocs2::PinocchioInterfaceCppAd pinocchio_interface_cpp_ad_;
  std::size_t joint_dim_{0};
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__FORWARD_DYNAMICS_MPC__DYNAMICS__FORWARD_DYNAMICS_ABA_DYNAMICS_AD_HPP_
