#include "dynamics_mpc_controller/forward_dynamics_mpc/forward_dynamics_mpc_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include "dynamics_mpc_controller/common/cost/ee_motion_tracking_cost.hpp"
#include "dynamics_mpc_controller/common/cost/input_tracking_cost.hpp"
#include "dynamics_mpc_controller/common/cost/joint_tracking_cost.hpp"
#include "dynamics_mpc_controller/common/pinocchio_utils.hpp"
#include "dynamics_mpc_controller/forward_dynamics_mpc/dynamics/forward_dynamics_aba_dynamics_ad.hpp"
#include "dynamics_mpc_controller/forward_dynamics_mpc/initialization/forward_dynamics_initializer.hpp"

namespace dynamics_mpc_controller
{
namespace
{

ocs2::IntegratorType toIntegratorType(const std::string& name)
{
  if (name == "ODE45") {
    return ocs2::IntegratorType::ODE45;
  }
  if (name == "RK4") {
    return ocs2::IntegratorType::RK4;
  }
  if (name == "EULER") {
    return ocs2::IntegratorType::EULER;
  }
  throw std::runtime_error("Unsupported integrator type: " + name);
}

ocs2::search_strategy::Type toSearchStrategy(const std::string& name)
{
  if (name == "LINE_SEARCH") {
    return ocs2::search_strategy::Type::LINE_SEARCH;
  }
  if (name == "LEVENBERG_MARQUARDT") {
    return ocs2::search_strategy::Type::LEVENBERG_MARQUARDT;
  }
  throw std::runtime_error("Unsupported DDP strategy: " + name);
}

auto toHessianCorrectionStrategy(const std::string& name)
{
  using Strategy = ocs2::hessian_correction::Strategy;
  if (name == "DIAGONAL_SHIFT") {
    return Strategy::DIAGONAL_SHIFT;
  }
  if (name == "CHOLESKY_MODIFICATION") {
    return Strategy::CHOLESKY_MODIFICATION;
  }
  if (name == "EIGENVALUE_MODIFICATION") {
    return Strategy::EIGENVALUE_MODIFICATION;
  }
  if (name == "GERSHGORIN_MODIFICATION") {
    return Strategy::GERSHGORIN_MODIFICATION;
  }
  throw std::runtime_error("Unsupported hessianCorrectionStrategy: " + name);
}

ocs2::SensitivityIntegratorType toSensitivityIntegratorType(const std::string& name)
{
  if (name == "EULER") {
    return ocs2::SensitivityIntegratorType::EULER;
  }
  if (name == "RK2") {
    return ocs2::SensitivityIntegratorType::RK2;
  }
  if (name == "RK4") {
    return ocs2::SensitivityIntegratorType::RK4;
  }
  throw std::runtime_error("Unsupported SQP integrator type: " + name);
}

std::vector<std::string> getLastJointNames(
  const pinocchio::Model& model,
  std::size_t jointDim)
{
  if (model.names.size() < jointDim) {
    throw std::runtime_error("[ForwardDynamicsMpcInterface] Pinocchio model has fewer names than joints.");
  }
  return std::vector<std::string>(model.names.end() - static_cast<std::ptrdiff_t>(jointDim), model.names.end());
}

ocs2::vector_t vectorFromArray(
  const std::vector<double>& values,
  std::size_t expectedSize,
  double fallback)
{
  ocs2::vector_t out = ocs2::vector_t::Constant(expectedSize, fallback);
  const std::size_t count = std::min(expectedSize, values.size());
  for (std::size_t i = 0; i < count; ++i) {
    out(static_cast<Eigen::Index>(i)) = values[i];
  }
  return out;
}

ocs2::vector_t vectorFromArrayExact(
  const std::vector<double>& values,
  std::size_t expectedSize,
  const std::string& parameterName)
{
  if (values.size() != expectedSize) {
    throw std::runtime_error(
      "[ForwardDynamicsMpcInterface] " + parameterName + " must contain " +
      std::to_string(expectedSize) + " values, got " + std::to_string(values.size()) + ".");
  }
  return Eigen::Map<const ocs2::vector_t>(values.data(), static_cast<Eigen::Index>(values.size()));
}

void validateBounds(
  const ocs2::vector_t& lowerBound,
  const ocs2::vector_t& upperBound,
  const std::string& parameterName)
{
  if ((lowerBound.array() > upperBound.array()).any()) {
    throw std::runtime_error(
      "[ForwardDynamicsMpcInterface] lowerBound exceeds upperBound for " + parameterName + ".");
  }
}

std::unique_ptr<ocs2::LinearStateInputConstraint> createInputBoxConstraint(
  std::size_t stateDim,
  const ocs2::vector_t& lowerBound,
  const ocs2::vector_t& upperBound)
{
  if (lowerBound.size() != upperBound.size()) {
    throw std::runtime_error("[ForwardDynamicsMpcInterface] input limit lower/upper dimensions do not match.");
  }

  const Eigen::Index input_dim = lowerBound.size();
  ocs2::vector_t e = ocs2::vector_t::Zero(2 * input_dim);
  ocs2::matrix_t C = ocs2::matrix_t::Zero(2 * input_dim, static_cast<Eigen::Index>(stateDim));
  ocs2::matrix_t D = ocs2::matrix_t::Zero(2 * input_dim, input_dim);

  for (Eigen::Index i = 0; i < input_dim; ++i) {
    e(i) = -lowerBound(i);
    D(i, i) = 1.0;

    e(input_dim + i) = upperBound(i);
    D(input_dim + i, i) = -1.0;
  }

  return std::make_unique<ocs2::LinearStateInputConstraint>(
    std::move(e), std::move(C), std::move(D));
}

std::unique_ptr<ocs2::LinearStateInputConstraint> createStateBoxConstraint(
  std::size_t inputDim,
  const ocs2::vector_t& lowerBound,
  const ocs2::vector_t& upperBound)
{
  if (lowerBound.size() != upperBound.size()) {
    throw std::runtime_error("[ForwardDynamicsMpcInterface] state limit lower/upper dimensions do not match.");
  }

  const Eigen::Index state_dim = lowerBound.size();
  ocs2::vector_t e = ocs2::vector_t::Zero(2 * state_dim);
  ocs2::matrix_t C = ocs2::matrix_t::Zero(2 * state_dim, state_dim);
  ocs2::matrix_t D = ocs2::matrix_t::Zero(2 * state_dim, static_cast<Eigen::Index>(inputDim));

  for (Eigen::Index i = 0; i < state_dim; ++i) {
    e(i) = -lowerBound(i);
    C(i, i) = 1.0;

    e(state_dim + i) = upperBound(i);
    C(state_dim + i, i) = -1.0;
  }

  return std::make_unique<ocs2::LinearStateInputConstraint>(
    std::move(e), std::move(C), std::move(D));
}

}  // namespace

ForwardDynamicsMpcInterface::ForwardDynamicsMpcInterface(const Params& parameters)
{
  loadSolverSettings(parameters);
  setupPinocchio(parameters);
  setupOptimalControlProblem(parameters);
}

std::shared_ptr<ocs2::ReferenceManagerInterface> ForwardDynamicsMpcInterface::getReferenceManagerPtr() const
{
  return reference_manager_ptr_;
}

void ForwardDynamicsMpcInterface::loadSolverSettings(const Params& parameters)
{
  const auto& mpc = parameters.ocs2.mpc;
  mpc_settings_.coldStart_ = mpc.coldStart;
  mpc_settings_.debugPrint_ = mpc.debugPrint;
  mpc_settings_.solutionTimeWindow_ = mpc.solutionTimeWindow;
  mpc_settings_.timeHorizon_ = mpc.timeHorizon;
  mpc_settings_.mpcDesiredFrequency_ = mpc.mpcDesiredFrequency;
  mpc_settings_.mrtDesiredFrequency_ = mpc.mrtDesiredFrequency;

  const auto& ddp = parameters.ocs2.task.ddp;
  ddp_settings_.algorithm_ = ocs2::ddp::fromAlgorithmName(ddp.algorithm);
  ddp_settings_.nThreads_ = static_cast<std::size_t>(ddp.nThreads);
  ddp_settings_.threadPriority_ = ddp.threadPriority;
  ddp_settings_.maxNumIterations_ = static_cast<std::size_t>(ddp.maxNumIterations);
  ddp_settings_.minRelCost_ = ddp.minRelCost;
  ddp_settings_.constraintTolerance_ = ddp.constraintTolerance;
  ddp_settings_.displayInfo_ = ddp.displayInfo;
  ddp_settings_.displayShortSummary_ = ddp.displayShortSummary;
  ddp_settings_.checkNumericalStability_ = ddp.checkNumericalStability;
  ddp_settings_.debugPrintRollout_ = ddp.debugPrintRollout;
  ddp_settings_.absTolODE_ = ddp.AbsTolODE;
  ddp_settings_.relTolODE_ = ddp.RelTolODE;
  ddp_settings_.maxNumStepsPerSecond_ = static_cast<std::size_t>(ddp.maxNumStepsPerSecond);
  ddp_settings_.timeStep_ = ddp.timeStep;
  ddp_settings_.backwardPassIntegratorType_ = toIntegratorType(ddp.backwardPassIntegratorType);
  ddp_settings_.constraintPenaltyInitialValue_ = ddp.constraintPenaltyInitialValue;
  ddp_settings_.constraintPenaltyIncreaseRate_ = ddp.constraintPenaltyIncreaseRate;
  ddp_settings_.preComputeRiccatiTerms_ = ddp.preComputeRiccatiTerms;
  ddp_settings_.useFeedbackPolicy_ = ddp.useFeedbackPolicy;
  ddp_settings_.strategy_ = toSearchStrategy(ddp.strategy);
  ddp_settings_.lineSearch_.minStepLength = ddp.lineSearch.minStepLength;
  ddp_settings_.lineSearch_.maxStepLength = ddp.lineSearch.maxStepLength;
  ddp_settings_.lineSearch_.hessianCorrectionStrategy =
    toHessianCorrectionStrategy(ddp.lineSearch.hessianCorrectionStrategy);
  ddp_settings_.lineSearch_.hessianCorrectionMultiple = ddp.lineSearch.hessianCorrectionMultiple;

  const auto& sqp = parameters.ocs2.task.sqp;
  sqp_settings_.sqpIteration = static_cast<std::size_t>(sqp.sqpIteration);
  sqp_settings_.deltaTol = sqp.deltaTol;
  sqp_settings_.costTol = sqp.costTol;
  sqp_settings_.g_max = sqp.g_max;
  sqp_settings_.g_min = sqp.g_min;
  sqp_settings_.useFeedbackPolicy = sqp.useFeedbackPolicy;
  sqp_settings_.dt = sqp.dt;
  sqp_settings_.integratorType = toSensitivityIntegratorType(sqp.integratorType);
  sqp_settings_.inequalityConstraintMu = sqp.inequalityConstraintMu;
  sqp_settings_.inequalityConstraintDelta = sqp.inequalityConstraintDelta;
  sqp_settings_.projectStateInputEqualityConstraints = sqp.projectStateInputEqualityConstraints;
  sqp_settings_.extractProjectionMultiplier = sqp.extractProjectionMultiplier;
  sqp_settings_.printSolverStatistics = sqp.printSolverStatistics;
  sqp_settings_.printSolverStatus = sqp.printSolverStatus;
  sqp_settings_.printLinesearch = sqp.printLinesearch;
  sqp_settings_.enableLogging = sqp.enableLogging;
  sqp_settings_.nThreads = static_cast<std::size_t>(sqp.nThreads);
  sqp_settings_.threadPriority = sqp.threadPriority;
  sqp_settings_.logFilePath = sqp.logFilePath;
}

void ForwardDynamicsMpcInterface::setupPinocchio(const Params& parameters)
{
  std::vector<std::string> remove_joint_names;
  for (const auto& joint_name : parameters.ocs2.task.model_information.removeJoints) {
    if (!joint_name.empty()) {
      remove_joint_names.push_back(joint_name);
    }
  }

  pinocchio_interface_ptr_ = std::make_unique<ocs2::PinocchioInterface>(
    pinocchio_utils::createPinocchioInterface(
      parameters.paths.urdfFile,
      remove_joint_names));

  const auto& model = pinocchio_interface_ptr_->getModel();
  if (model.nq != model.nv) {
    throw std::runtime_error(
      "[ForwardDynamicsMpcInterface] This fixed-base forward-dynamics implementation requires model.nq == model.nv.");
  }
  if (model.nv <= 0) {
    throw std::runtime_error("[ForwardDynamicsMpcInterface] Pinocchio model has no velocity DOFs.");
  }

  const auto joint_dim = static_cast<std::size_t>(model.nv);
  const std::string requested_ee_frame_name = parameters.ocs2.task.model_information.endEffectorFrame;
  pinocchio::FrameIndex ee_frame_id{};
  if (requested_ee_frame_name.empty()) {
    ee_frame_id = static_cast<pinocchio::FrameIndex>(model.frames.size() - 1);
    std::cerr << "[ForwardDynamicsMpcInterface] model_information.endEffectorFrame is empty; using last Pinocchio frame '"
              << model.frames[ee_frame_id].name << "'.\n";
  } else {
    if (!model.existFrame(requested_ee_frame_name)) {
      throw std::runtime_error(
        "[ForwardDynamicsMpcInterface] endEffectorFrame '" + requested_ee_frame_name +
        "' does not exist in the Pinocchio model.");
    }
    ee_frame_id = model.getFrameId(requested_ee_frame_name);
  }
  const std::string ee_frame_name = model.frames[ee_frame_id].name;

  forward_dynamics_model_ = ForwardDynamicsMpcModel(
    joint_dim,
    getLastJointNames(model, joint_dim),
    ee_frame_name,
    ee_frame_id);

  std::cerr << "\n #### ForwardDynamicsMpcInterface model:";
  std::cerr << "\n #### =============================================================================";
  std::cerr << "\n #### jointDim: " << forward_dynamics_model_.jointDim();
  std::cerr << "\n #### stateDim: " << forward_dynamics_model_.stateDim();
  std::cerr << "\n #### inputDim: " << forward_dynamics_model_.inputDim();
  std::cerr << "\n #### input: [jointTorque]";
  std::cerr << "\n #### dynamics: ABA(q, v, tau)";
  std::cerr << "\n #### eeFrame: " << forward_dynamics_model_.endEffectorFrame();
  std::cerr << "\n #### =============================================================================\n";
}

void ForwardDynamicsMpcInterface::setupOptimalControlProblem(const Params& parameters)
{
  const std::size_t n = forward_dynamics_model_.jointDim();
  const bool recompile_libraries = parameters.ocs2.task.model_settings.recompileLibraries;

  reference_manager_ptr_ = std::make_shared<ocs2::ReferenceManager>();

  ocs2::vector_t default_position_weights =
    ocs2::vector_t::Zero(static_cast<Eigen::Index>(n));
  ocs2::vector_t default_velocity_weights =
    ocs2::vector_t::Zero(static_cast<Eigen::Index>(n));
  if (parameters.ocs2.task.jointTracking.activate) {
    default_position_weights =
      (vectorFromArray(
        parameters.ocs2.task.jointTracking.position.diagonal,
        n,
        1.0) * parameters.ocs2.task.jointTracking.position.scaling).eval();
    default_velocity_weights =
      (vectorFromArray(
        parameters.ocs2.task.jointTracking.velocity.diagonal,
        n,
        1.0) * parameters.ocs2.task.jointTracking.velocity.scaling).eval();
  }
  ocs2::vector_t ee_motion_pose_weights = ocs2::vector_t::Zero(6);
  ocs2::vector_t ee_motion_twist_weights = ocs2::vector_t::Zero(6);
  if (parameters.ocs2.task.eeMotionTracking.activate) {
    default_ee_motion_twist_frame_ = parameters.ocs2.task.eeMotionTracking.twistFrame;
    if (default_ee_motion_twist_frame_ != "base" && default_ee_motion_twist_frame_ != "ee") {
      throw std::runtime_error("[ForwardDynamicsMpcInterface] eeMotionTracking.twistFrame must be 'base' or 'ee'.");
    }
    ee_motion_pose_weights =
      (vectorFromArray(
        parameters.ocs2.task.eeMotionTracking.pose.diagonal,
        6,
        1.0) * parameters.ocs2.task.eeMotionTracking.pose.scaling).eval();
    ee_motion_twist_weights =
      (vectorFromArray(
        parameters.ocs2.task.eeMotionTracking.twist.diagonal,
        6,
        1.0) * parameters.ocs2.task.eeMotionTracking.twist.scaling).eval();
  }

  ocs2::matrix_t R = ocs2::matrix_t::Zero(
    static_cast<Eigen::Index>(forward_dynamics_model_.inputDim()),
    static_cast<Eigen::Index>(forward_dynamics_model_.inputDim()));
  const ocs2::vector_t tau_weights =
    (vectorFromArray(
      parameters.ocs2.task.inputCost.R.jointTorque.diagonal,
      n,
      1.0) * parameters.ocs2.task.inputCost.R.jointTorque.scaling).eval();
  R.diagonal() = tau_weights;

  std::cerr << "\n #### ForwardDynamicsMpcInterface costs:";
  std::cerr << "\n #### =============================================================================";
  std::cerr << "\n #### jointTracking.activate: "
            << (parameters.ocs2.task.jointTracking.activate ? "true" : "false");
  std::cerr << "\n #### default position weights: " << default_position_weights.transpose();
  std::cerr << "\n #### default velocity weights: " << default_velocity_weights.transpose();
  std::cerr << "\n #### eeMotionTracking.activate: "
            << (parameters.ocs2.task.eeMotionTracking.activate ? "true" : "false");
  std::cerr << "\n #### eeMotionTracking.terminalScaling: "
            << parameters.ocs2.task.eeMotionTracking.terminalScaling;
  std::cerr << "\n #### eeMotionTracking.twistFrame: "
            << default_ee_motion_twist_frame_;
  std::cerr << "\n #### default EE pose weights: " << ee_motion_pose_weights.transpose();
  std::cerr << "\n #### default EE twist weights: " << ee_motion_twist_weights.transpose();
  std::cerr << "\n #### R diagonal: " << R.diagonal().transpose();
  std::cerr << "\n #### =============================================================================\n";

  if (parameters.ocs2.task.jointTracking.activate) {
    problem_.costPtr->add(
      "jointTracking",
      std::make_unique<cost::JointTrackingCost>(
        default_position_weights, default_velocity_weights, n));
  }
  if (parameters.ocs2.task.eeMotionTracking.activate) {
    problem_.costPtr->add(
      "ee_motion_tracking_cost",
      std::make_unique<cost::EeMotionTrackingCost>(
        *pinocchio_interface_ptr_,
        forward_dynamics_model_.endEffectorFrameId(),
        ee_motion_pose_weights,
        ee_motion_twist_weights,
        n));
    if (parameters.ocs2.task.eeMotionTracking.terminalScaling > 0.0) {
      problem_.finalCostPtr->add(
        "ee_motion_terminal_tracking_cost",
        std::make_unique<cost::EeMotionTrackingTerminalCost>(
          *pinocchio_interface_ptr_,
          forward_dynamics_model_.endEffectorFrameId(),
          ee_motion_pose_weights,
          ee_motion_twist_weights,
          n,
          parameters.ocs2.task.eeMotionTracking.terminalScaling));
    }
  }
  problem_.costPtr->add(
    "inputTracking",
    std::make_unique<cost::InputTrackingCost>(
      R, forward_dynamics_model_.inputDim()));

  problem_.dynamicsPtr = std::make_unique<ForwardDynamicsAbaDynamicsAD>(
    *pinocchio_interface_ptr_,
    n,
    "forward_dynamics_aba",
    parameters.paths.libFolder,
    recompile_libraries,
    true);

  const auto& input_limits = parameters.ocs2.task.inputLimits;
  const ocs2::vector_t torque_lower = vectorFromArrayExact(
    input_limits.jointTorque.lowerBound, n, "inputLimits.jointTorque.lowerBound");
  const ocs2::vector_t torque_upper = vectorFromArrayExact(
    input_limits.jointTorque.upperBound, n, "inputLimits.jointTorque.upperBound");
  validateBounds(torque_lower, torque_upper, "joint torque");

  input_limits_active_ = input_limits.activate;
  input_lower_bounds_ = torque_lower;
  input_upper_bounds_ = torque_upper;

  if (input_limits_active_) {
    problem_.inequalityConstraintPtr->add(
      "inputLimits",
      createInputBoxConstraint(
        forward_dynamics_model_.stateDim(),
        input_lower_bounds_,
        input_upper_bounds_));
  }

  const auto& state_limits = parameters.ocs2.task.stateLimits;
  const ocs2::vector_t position_lower = vectorFromArrayExact(
    state_limits.jointPosition.lowerBound, n, "stateLimits.jointPosition.lowerBound");
  const ocs2::vector_t position_upper = vectorFromArrayExact(
    state_limits.jointPosition.upperBound, n, "stateLimits.jointPosition.upperBound");
  const ocs2::vector_t velocity_lower = vectorFromArrayExact(
    state_limits.jointVelocity.lowerBound, n, "stateLimits.jointVelocity.lowerBound");
  const ocs2::vector_t velocity_upper = vectorFromArrayExact(
    state_limits.jointVelocity.upperBound, n, "stateLimits.jointVelocity.upperBound");
  validateBounds(position_lower, position_upper, "joint position");
  validateBounds(velocity_lower, velocity_upper, "joint velocity");

  state_limits_active_ = state_limits.activate;
  state_lower_bounds_.resize(static_cast<Eigen::Index>(forward_dynamics_model_.stateDim()));
  state_upper_bounds_.resize(static_cast<Eigen::Index>(forward_dynamics_model_.stateDim()));
  state_lower_bounds_.head(static_cast<Eigen::Index>(n)) = position_lower;
  state_upper_bounds_.head(static_cast<Eigen::Index>(n)) = position_upper;
  state_lower_bounds_.tail(static_cast<Eigen::Index>(n)) = velocity_lower;
  state_upper_bounds_.tail(static_cast<Eigen::Index>(n)) = velocity_upper;

  if (state_limits_active_) {
    problem_.inequalityConstraintPtr->add(
      "stateLimits",
      createStateBoxConstraint(
        forward_dynamics_model_.inputDim(),
        state_lower_bounds_,
        state_upper_bounds_));
  }

  ocs2::rollout::Settings rollout_settings;
  const auto& rollout = parameters.ocs2.task.rollout;
  rollout_settings.absTolODE = rollout.AbsTolODE;
  rollout_settings.relTolODE = rollout.RelTolODE;
  rollout_settings.maxNumStepsPerSecond = rollout.maxNumStepsPerSecond;
  rollout_settings.timeStep = rollout.timeStep;
  rollout_settings.checkNumericalStability = rollout.checkNumericalStability;
  rollout_settings.integratorType = toIntegratorType(rollout.integratorType);

  rollout_ptr_ = std::make_unique<ocs2::TimeTriggeredRollout>(*problem_.dynamicsPtr, rollout_settings);
  initializer_ptr_ = std::make_unique<ForwardDynamicsInitializer>(
    *pinocchio_interface_ptr_,
    forward_dynamics_model_,
    vectorFromArrayExact(
      parameters.numeric.holdAccelerationLowerBound,
      n,
      "numeric.holdAccelerationLowerBound"),
    vectorFromArrayExact(
      parameters.numeric.holdAccelerationUpperBound,
      n,
      "numeric.holdAccelerationUpperBound"),
    vectorFromArrayExact(
      parameters.numeric.holdVelocityDamping, n, "numeric.holdVelocityDamping"));
}

ocs2::vector_t ForwardDynamicsMpcInterface::computeForwardDynamics(
  const ocs2::vector_t& q,
  const ocs2::vector_t& v,
  const ocs2::vector_t& tau) const
{
  const auto& model = pinocchio_interface_ptr_->getModel();
  auto& data = pinocchio_interface_ptr_->getData();
  if (q.size() != model.nq || v.size() != model.nv || tau.size() != model.nv) {
    throw std::invalid_argument("Forward-dynamics q, v, or tau dimension does not match the Pinocchio model.");
  }
  return pinocchio::aba(model, data, q, v, tau);
}

ocs2::vector_t ForwardDynamicsMpcInterface::computeRneaTorque(
  const ocs2::vector_t& q,
  const ocs2::vector_t& v,
  const ocs2::vector_t& a) const
{
  const auto& model = pinocchio_interface_ptr_->getModel();
  auto& data = pinocchio_interface_ptr_->getData();
  if (q.size() != model.nq || v.size() != model.nv || a.size() != model.nv) {
    throw std::invalid_argument("Inverse-dynamics q, v, or a dimension does not match the Pinocchio model.");
  }
  return pinocchio::rnea(model, data, q, v, a);
}

ocs2::vector_t ForwardDynamicsMpcInterface::computeNonlinearEffects(
  const ocs2::vector_t& q,
  const ocs2::vector_t& v) const
{
  const auto& model = pinocchio_interface_ptr_->getModel();
  auto& data = pinocchio_interface_ptr_->getData();
  if (q.size() != model.nq || v.size() != model.nv) {
    throw std::invalid_argument(
      "Nonlinear-effects q or v dimension does not match the Pinocchio model.");
  }
  return pinocchio::nonLinearEffects(model, data, q, v);
}

ocs2::vector_t ForwardDynamicsMpcInterface::computeGravityCompensation(const ocs2::vector_t& q) const
{
  const std::size_t n = forward_dynamics_model_.jointDim();
  return computeNonlinearEffects(q, ocs2::vector_t::Zero(n));
}

}  // namespace dynamics_mpc_controller
