#ifndef DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__CONSTRAINT__INVERSE_DYNAMICS_RNEA_CONSTRAINT_CPPAD_HPP_
#define DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__CONSTRAINT__INVERSE_DYNAMICS_RNEA_CONSTRAINT_CPPAD_HPP_

#include <cstddef>
#include <string>

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace dynamics_mpc_controller
{

class InverseDynamicsRneaConstraintCppAd final : public ocs2::StateInputConstraintCppAd
{
public:
  InverseDynamicsRneaConstraintCppAd(
    const ocs2::PinocchioInterface& pinocchioInterface,
    std::size_t jointDim,
    const std::string& modelName,
    const std::string& modelFolder = "/tmp/ocs2",
    bool recompileLibraries = true,
    bool verbose = true);

  ~InverseDynamicsRneaConstraintCppAd() override = default;

  InverseDynamicsRneaConstraintCppAd* clone() const override
  {
    return new InverseDynamicsRneaConstraintCppAd(*this);
  }

  std::size_t getNumConstraints(ocs2::scalar_t time) const override;

private:
  InverseDynamicsRneaConstraintCppAd(const InverseDynamicsRneaConstraintCppAd& rhs);

  ocs2::ad_vector_t constraintFunction(
    ocs2::ad_scalar_t time,
    const ocs2::ad_vector_t& state,
    const ocs2::ad_vector_t& input,
    const ocs2::ad_vector_t& parameters) const override;

  ocs2::PinocchioInterfaceCppAd pinocchio_interface_cpp_ad_;
  std::size_t joint_dim_{0};
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__CONSTRAINT__INVERSE_DYNAMICS_RNEA_CONSTRAINT_CPPAD_HPP_
