#ifndef DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_MODEL_HPP_
#define DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_MODEL_HPP_

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <pinocchio/multibody/fwd.hpp>

namespace dynamics_mpc_controller
{

class InverseDynamicsMpcModel
{
public:
  InverseDynamicsMpcModel() = default;

  InverseDynamicsMpcModel(
    std::size_t jointDim,
    std::vector<std::string> dofNames,
    std::string endEffectorFrame,
    pinocchio::FrameIndex endEffectorFrameId,
    bool wrenchInRnea);

  std::size_t jointDim() const { return joint_dim_; }
  std::size_t stateDim() const { return 2 * joint_dim_; }
  std::size_t inputDim() const { return 2 * joint_dim_ + (wrench_in_rnea_ ? 6 : 0); }
  bool hasEeWrenchInput() const { return wrench_in_rnea_; }
  bool wrenchInRnea() const { return wrench_in_rnea_; }

  std::size_t qOffset() const { return 0; }
  std::size_t vOffset() const { return joint_dim_; }
  std::size_t aOffset() const { return 0; }
  std::size_t tauOffset() const { return joint_dim_; }
  std::size_t wrenchOffset() const
  {
    if (!wrench_in_rnea_) {
      throw std::logic_error("The active MPC formulation has no end-effector wrench input.");
    }
    return 2 * joint_dim_;
  }

  const std::vector<std::string>& dofNames() const { return dof_names_; }
  const std::string& endEffectorFrame() const { return end_effector_frame_; }
  pinocchio::FrameIndex endEffectorFrameId() const { return end_effector_frame_id_; }

  ocs2::vector_t getJointPosition(const ocs2::vector_t& state) const;
  ocs2::vector_t getJointVelocity(const ocs2::vector_t& state) const;
  ocs2::vector_t getJointAcceleration(const ocs2::vector_t& input) const;
  ocs2::vector_t getJointTorque(const ocs2::vector_t& input) const;
  ocs2::vector_t getEeWrench(const ocs2::vector_t& input) const;

private:
  std::size_t joint_dim_{0};
  std::vector<std::string> dof_names_;
  std::string end_effector_frame_;
  pinocchio::FrameIndex end_effector_frame_id_{0};
  bool wrench_in_rnea_{false};
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_MODEL_HPP_
