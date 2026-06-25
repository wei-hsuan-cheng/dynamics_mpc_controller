#include "dynamics_mpc_controller/common/cost/input_tracking_cost.hpp"

#include <stdexcept>
#include <utility>

namespace dynamics_mpc_controller
{
namespace cost
{

InputTrackingCost::InputTrackingCost(ocs2::matrix_t R, std::size_t inputDim)
: R_(std::move(R)),
  input_dim_(inputDim)
{
  const Eigen::Index input_dim = static_cast<Eigen::Index>(input_dim_);
  if (R_.rows() != input_dim || R_.cols() != input_dim) {
    throw std::runtime_error("[InputTrackingCost] R dimension mismatch.");
  }
}

ocs2::vector_t InputTrackingCost::getDesiredInput(
  ocs2::scalar_t time,
  const ocs2::TargetTrajectories& targetTrajectories) const
{
  ocs2::vector_t desired_input = ocs2::vector_t::Zero(static_cast<Eigen::Index>(input_dim_));
  const ocs2::vector_t raw_input = targetTrajectories.getDesiredInput(time);
  if (raw_input.size() == desired_input.size()) {
    desired_input = raw_input;
  }
  return desired_input;
}

ocs2::scalar_t InputTrackingCost::getValue(
  ocs2::scalar_t time,
  const ocs2::vector_t&,
  const ocs2::vector_t& input,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::PreComputation&) const
{
  const ocs2::vector_t input_deviation = input - getDesiredInput(time, targetTrajectories);
  return 0.5 * input_deviation.dot(R_ * input_deviation);
}

ocs2::ScalarFunctionQuadraticApproximation InputTrackingCost::getQuadraticApproximation(
  ocs2::scalar_t time,
  const ocs2::vector_t& state,
  const ocs2::vector_t& input,
  const ocs2::TargetTrajectories& targetTrajectories,
  const ocs2::PreComputation&) const
{
  const ocs2::vector_t input_deviation = input - getDesiredInput(time, targetTrajectories);

  ocs2::ScalarFunctionQuadraticApproximation cost;
  cost.setZero(state.size(), input.size());
  cost.dfduu = R_;
  cost.dfdu = R_ * input_deviation;
  cost.f = 0.5 * input_deviation.dot(cost.dfdu);
  return cost;
}

}  // namespace cost
}  // namespace dynamics_mpc_controller
