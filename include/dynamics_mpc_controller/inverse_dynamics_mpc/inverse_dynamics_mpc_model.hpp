#ifndef DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_MODEL_HPP_
#define DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_MODEL_HPP_

#include <cstddef>
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
    pinocchio::FrameIndex endEffectorFrameId);

  std::size_t jointDim() const { return joint_dim_; }
  std::size_t stateDim() const { return 2 * joint_dim_; }
  std::size_t inputDim() const { return 2 * joint_dim_ + 6; }

  std::size_t qOffset() const { return 0; }
  std::size_t vOffset() const { return joint_dim_; }
  std::size_t aOffset() const { return 0; }
  std::size_t tauOffset() const { return joint_dim_; }
  std::size_t wrenchOffset() const { return 2 * joint_dim_; }

  const std::vector<std::string>& dofNames() const { return dof_names_; }
  const std::string& endEffectorFrame() const { return end_effector_frame_; }
  pinocchio::FrameIndex endEffectorFrameId() const { return end_effector_frame_id_; }

  ocs2::vector_t getQ(const ocs2::vector_t& state) const;
  ocs2::vector_t getV(const ocs2::vector_t& state) const;
  ocs2::vector_t getA(const ocs2::vector_t& input) const;
  ocs2::vector_t getTau(const ocs2::vector_t& input) const;
  ocs2::vector_t getWrench(const ocs2::vector_t& input) const;

private:
  std::size_t joint_dim_{0};
  std::vector<std::string> dof_names_;
  std::string end_effector_frame_;
  pinocchio::FrameIndex end_effector_frame_id_{0};
};

}  // namespace dynamics_mpc_controller

#endif  // DYNAMICS_MPC_CONTROLLER__INVERSE_DYNAMICS_MPC__INVERSE_DYNAMICS_MPC_MODEL_HPP_
