#include "dynamics_mpc_controller/inverse_dynamics_mpc/inverse_dynamics_mpc_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/SensitivityIntegrator.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include "dynamics_mpc_controller/common/constraint/dynamics_self_collision_constraint.hpp"
#include "dynamics_mpc_controller/common/cost/ee_motion_tracking_cost.hpp"
#include "dynamics_mpc_controller/common/cost/input_tracking_cost.hpp"
#include "dynamics_mpc_controller/common/cost/joint_tracking_cost.hpp"
#include "dynamics_mpc_controller/common/pinocchio_utils.hpp"
#include "dynamics_mpc_controller/inverse_dynamics_mpc/constraint/inverse_dynamics_ee_wrench_tracking_constraint.hpp"
#include "dynamics_mpc_controller/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_constraint_cppad.hpp"
#include "dynamics_mpc_controller/inverse_dynamics_mpc/constraint/inverse_dynamics_rnea_with_ee_wrench_constraint_cppad.hpp"
#include "dynamics_mpc_controller/inverse_dynamics_mpc/dynamics/inverse_dynamics_kinematic_dynamics_ad.hpp"
#include "dynamics_mpc_controller/inverse_dynamics_mpc/initialization/inverse_dynamics_initializer.hpp"

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
    throw std::runtime_error("[InverseDynamicsMpcInterface] Pinocchio model has fewer names than joints.");
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
      "[InverseDynamicsMpcInterface] " + parameterName + " must contain " +
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
      "[InverseDynamicsMpcInterface] lowerBound exceeds upperBound for " + parameterName + ".");
  }
}

std::unique_ptr<ocs2::LinearStateInputConstraint> createInputBoxConstraint(
  std::size_t stateDim,
  const ocs2::vector_t& lowerBound,
  const ocs2::vector_t& upperBound)
{
  if (lowerBound.size() != upperBound.size()) {
    throw std::runtime_error("[InverseDynamicsMpcInterface] input limit lower/upper dimensions do not match.");
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

std::unique_ptr<ocs2::LinearStateInputConstraint> createStateSubvectorBoxConstraint(
  std::size_t inputDim,
  std::size_t stateDim,
  std::size_t stateOffset,
  const ocs2::vector_t& lowerBound,
  const ocs2::vector_t& upperBound)
{
  if (lowerBound.size() != upperBound.size()) {
    throw std::runtime_error("[InverseDynamicsMpcInterface] state limit lower/upper dimensions do not match.");
  }
  if (stateOffset + static_cast<std::size_t>(lowerBound.size()) > stateDim) {
    throw std::runtime_error("[InverseDynamicsMpcInterface] state limit offset exceeds state dimension.");
  }

  const Eigen::Index constrained_dim = lowerBound.size();
  ocs2::vector_t e = ocs2::vector_t::Zero(2 * constrained_dim);
  ocs2::matrix_t C = ocs2::matrix_t::Zero(
    2 * constrained_dim,
    static_cast<Eigen::Index>(stateDim));
  ocs2::matrix_t D = ocs2::matrix_t::Zero(
    2 * constrained_dim,
    static_cast<Eigen::Index>(inputDim));
  const Eigen::Index offset = static_cast<Eigen::Index>(stateOffset);

  for (Eigen::Index i = 0; i < constrained_dim; ++i) {
    e(i) = -lowerBound(i);
    C(i, offset + i) = 1.0;

    e(constrained_dim + i) = upperBound(i);
    C(constrained_dim + i, offset + i) = -1.0;
  }

  return std::make_unique<ocs2::LinearStateInputConstraint>(
    std::move(e), std::move(C), std::move(D));
}

}  // namespace

InverseDynamicsMpcInterface::InverseDynamicsMpcInterface(const Params& parameters)
{
  loadSolverSettings(parameters);
  setupPinocchio(parameters);
  setupOptimalControlProblem(parameters);
}

std::shared_ptr<ocs2::ReferenceManagerInterface> InverseDynamicsMpcInterface::getReferenceManagerPtr() const
{
  return reference_manager_ptr_;
}

void InverseDynamicsMpcInterface::loadSolverSettings(const Params& parameters)
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

void InverseDynamicsMpcInterface::setupPinocchio(const Params& parameters)
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
      "[InverseDynamicsMpcInterface] This first fixed-base inverse-dynamics implementation requires model.nq == model.nv.");
  }
  if (model.nv <= 0) {
    throw std::runtime_error("[InverseDynamicsMpcInterface] Pinocchio model has no velocity DOFs.");
  }

  const auto joint_dim = static_cast<std::size_t>(model.nv);
  const std::string requested_ee_frame_name = parameters.ocs2.task.model_information.endEffectorFrame;
  pinocchio::FrameIndex ee_frame_id{};
  if (requested_ee_frame_name.empty()) {
    ee_frame_id = static_cast<pinocchio::FrameIndex>(model.frames.size() - 1);
    std::cerr << "[InverseDynamicsMpcInterface] model_information.endEffectorFrame is empty; using last Pinocchio frame '"
              << model.frames[ee_frame_id].name << "'.\n";
  } else {
    if (!model.existFrame(requested_ee_frame_name)) {
      throw std::runtime_error(
        "[InverseDynamicsMpcInterface] endEffectorFrame '" + requested_ee_frame_name +
        "' does not exist in the Pinocchio model.");
    }
    ee_frame_id = model.getFrameId(requested_ee_frame_name);
  }
  const std::string ee_frame_name = model.frames[ee_frame_id].name;
  const bool wrench_in_rnea = parameters.ocs2.task.model_settings.wrenchInRNEA;
  const bool track_zero_wrench = parameters.ocs2.task.model_settings.trackZeroWrench;
  if (track_zero_wrench && !wrench_in_rnea) {
    throw std::runtime_error(
      "[InverseDynamicsMpcInterface] model_settings.trackZeroWrench requires wrenchInRNEA=true.");
  }

  inverse_dynamics_model_ = InverseDynamicsMpcModel(
    joint_dim,
    getLastJointNames(model, joint_dim),
    ee_frame_name,
    ee_frame_id,
    wrench_in_rnea,
    track_zero_wrench);

  std::cerr << "\n #### InverseDynamicsMpcInterface model:";
  std::cerr << "\n #### =============================================================================";
  std::cerr << "\n #### jointDim: " << inverse_dynamics_model_.jointDim();
  std::cerr << "\n #### stateDim: " << inverse_dynamics_model_.stateDim();
  std::cerr << "\n #### inputDim: " << inverse_dynamics_model_.inputDim();
  std::cerr << "\n #### input: " <<
    (inverse_dynamics_model_.hasEeWrenchInput() ? "[jointAcceleration, jointTorque, eeWrench]" :
    "[jointAcceleration, jointTorque]");
  if (inverse_dynamics_model_.hasEeWrenchInput()) {
    std::cerr << "\n #### trackZeroWrench: " <<
      (inverse_dynamics_model_.trackZeroWrench() ? "true" : "false");
  }
  std::cerr << "\n #### eeFrame: " << inverse_dynamics_model_.endEffectorFrame();
  std::cerr << "\n #### =============================================================================\n";
}

void InverseDynamicsMpcInterface::setupOptimalControlProblem(const Params& parameters)
{
  const std::size_t n = inverse_dynamics_model_.jointDim();
  const bool has_ee_wrench = inverse_dynamics_model_.hasEeWrenchInput();
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
      throw std::runtime_error("[InverseDynamicsMpcInterface] eeMotionTracking.twistFrame must be 'base' or 'ee'.");
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
    static_cast<Eigen::Index>(inverse_dynamics_model_.inputDim()),
    static_cast<Eigen::Index>(inverse_dynamics_model_.inputDim()));
  const ocs2::vector_t a_weights =
    (vectorFromArray(
      parameters.ocs2.task.inputCost.R.jointAcceleration.diagonal,
      n,
      1.0) * parameters.ocs2.task.inputCost.R.jointAcceleration.scaling).eval();
  const ocs2::vector_t tau_weights =
    (vectorFromArray(
      parameters.ocs2.task.inputCost.R.jointTorque.diagonal,
      n,
      1.0) * parameters.ocs2.task.inputCost.R.jointTorque.scaling).eval();
  R.block(
    static_cast<Eigen::Index>(inverse_dynamics_model_.aOffset()),
    static_cast<Eigen::Index>(inverse_dynamics_model_.aOffset()),
    static_cast<Eigen::Index>(n),
    static_cast<Eigen::Index>(n)) = a_weights.asDiagonal();
  R.block(
    static_cast<Eigen::Index>(inverse_dynamics_model_.tauOffset()),
    static_cast<Eigen::Index>(inverse_dynamics_model_.tauOffset()),
    static_cast<Eigen::Index>(n),
    static_cast<Eigen::Index>(n)) = tau_weights.asDiagonal();
  if (has_ee_wrench) {
    const ocs2::vector_t wrench_weights =
      (vectorFromArray(
        parameters.ocs2.task.inputCost.R.eeWrench.diagonal,
        6,
        1.0) * parameters.ocs2.task.inputCost.R.eeWrench.scaling).eval();
    R.block(
      static_cast<Eigen::Index>(inverse_dynamics_model_.wrenchOffset()),
      static_cast<Eigen::Index>(inverse_dynamics_model_.wrenchOffset()),
      6,
      6) = wrench_weights.asDiagonal();
  }

  std::cerr << "\n #### InverseDynamicsMpcInterface costs:";
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
        inverse_dynamics_model_.endEffectorFrameId(),
        ee_motion_pose_weights,
        ee_motion_twist_weights,
        n));
    if (parameters.ocs2.task.eeMotionTracking.terminalScaling > 0.0) {
      problem_.finalCostPtr->add(
        "ee_motion_terminal_tracking_cost",
        std::make_unique<cost::EeMotionTrackingTerminalCost>(
          *pinocchio_interface_ptr_,
          inverse_dynamics_model_.endEffectorFrameId(),
          ee_motion_pose_weights,
          ee_motion_twist_weights,
          n,
          parameters.ocs2.task.eeMotionTracking.terminalScaling));
    }
  }
  problem_.costPtr->add(
    "inputTracking",
    std::make_unique<cost::InputTrackingCost>(
      R, inverse_dynamics_model_.inputDim()));

  problem_.dynamicsPtr = std::make_unique<InverseDynamicsKinematicDynamicsAD>(
    n,
    inverse_dynamics_model_.inputDim(),
    has_ee_wrench ? "inverse_dynamics_kinematic_dynamics_with_wrench" :
    "inverse_dynamics_kinematic_dynamics_no_wrench",
    parameters.paths.libFolder,
    recompile_libraries,
    true);

  if (has_ee_wrench) {
    problem_.equalityConstraintPtr->add(
      "inverseDynamicsRneaWithEeWrench",
      std::make_unique<InverseDynamicsRneaWithEeWrenchConstraintCppAd>(
        *pinocchio_interface_ptr_,
        inverse_dynamics_model_.endEffectorFrameId(),
        n,
        "inverse_dynamics_rnea_with_ee_wrench",
        parameters.paths.libFolder,
        recompile_libraries,
        true));
    problem_.equalityConstraintPtr->add(
      "eeWrenchTracking",
      std::make_unique<InverseDynamicsEeWrenchTrackingConstraint>(
        inverse_dynamics_model_.stateDim(),
        inverse_dynamics_model_.inputDim(),
        inverse_dynamics_model_.wrenchOffset(),
        *reference_manager_ptr_,
        inverse_dynamics_model_.trackZeroWrench()));
  } else {
    problem_.equalityConstraintPtr->add(
      "inverseDynamicsRnea",
      std::make_unique<InverseDynamicsRneaConstraintCppAd>(
        *pinocchio_interface_ptr_,
        n,
        "inverse_dynamics_rnea_no_wrench",
        parameters.paths.libFolder,
        recompile_libraries,
        true));
  }

  const auto& input_limits = parameters.ocs2.task.inputLimits;
  const ocs2::vector_t acceleration_lower = vectorFromArrayExact(
    input_limits.jointAcceleration.lowerBound, n, "inputLimits.jointAcceleration.lowerBound");
  const ocs2::vector_t acceleration_upper = vectorFromArrayExact(
    input_limits.jointAcceleration.upperBound, n, "inputLimits.jointAcceleration.upperBound");
  const ocs2::vector_t torque_lower = vectorFromArrayExact(
    input_limits.jointTorque.lowerBound, n, "inputLimits.jointTorque.lowerBound");
  const ocs2::vector_t torque_upper = vectorFromArrayExact(
    input_limits.jointTorque.upperBound, n, "inputLimits.jointTorque.upperBound");
  validateBounds(acceleration_lower, acceleration_upper, "joint acceleration");
  validateBounds(torque_lower, torque_upper, "joint torque");

  input_limits_active_ = input_limits.activate;
  input_lower_bounds_.resize(static_cast<Eigen::Index>(inverse_dynamics_model_.inputDim()));
  input_upper_bounds_.resize(static_cast<Eigen::Index>(inverse_dynamics_model_.inputDim()));
  input_lower_bounds_.segment(0, static_cast<Eigen::Index>(n)) = acceleration_lower;
  input_upper_bounds_.segment(0, static_cast<Eigen::Index>(n)) = acceleration_upper;
  input_lower_bounds_.segment(
    static_cast<Eigen::Index>(inverse_dynamics_model_.tauOffset()),
    static_cast<Eigen::Index>(n)) = torque_lower;
  input_upper_bounds_.segment(
    static_cast<Eigen::Index>(inverse_dynamics_model_.tauOffset()),
    static_cast<Eigen::Index>(n)) = torque_upper;
  if (has_ee_wrench) {
    const ocs2::vector_t wrench_lower = vectorFromArrayExact(
      input_limits.eeWrench.lowerBound, 6, "inputLimits.eeWrench.lowerBound");
    const ocs2::vector_t wrench_upper = vectorFromArrayExact(
      input_limits.eeWrench.upperBound, 6, "inputLimits.eeWrench.upperBound");
    validateBounds(wrench_lower, wrench_upper, "end-effector wrench");
    input_lower_bounds_.segment(
      static_cast<Eigen::Index>(inverse_dynamics_model_.wrenchOffset()), 6) = wrench_lower;
    input_upper_bounds_.segment(
      static_cast<Eigen::Index>(inverse_dynamics_model_.wrenchOffset()), 6) = wrench_upper;
  }

  if (input_limits_active_) {
    problem_.inequalityConstraintPtr->add(
      "inputLimits",
      createInputBoxConstraint(
        inverse_dynamics_model_.stateDim(),
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

  state_limits_active_ =
    state_limits.jointPosition.activate || state_limits.jointVelocity.activate;
  state_lower_bounds_.resize(static_cast<Eigen::Index>(inverse_dynamics_model_.stateDim()));
  state_upper_bounds_.resize(static_cast<Eigen::Index>(inverse_dynamics_model_.stateDim()));
  state_lower_bounds_.head(static_cast<Eigen::Index>(n)) = position_lower;
  state_upper_bounds_.head(static_cast<Eigen::Index>(n)) = position_upper;
  state_lower_bounds_.tail(static_cast<Eigen::Index>(n)) = velocity_lower;
  state_upper_bounds_.tail(static_cast<Eigen::Index>(n)) = velocity_upper;

  if (state_limits.jointPosition.activate) {
    problem_.inequalityConstraintPtr->add(
      "jointPositionLimits",
      createStateSubvectorBoxConstraint(
        inverse_dynamics_model_.inputDim(),
        inverse_dynamics_model_.stateDim(),
        0,
        position_lower,
        position_upper));
  }
  if (state_limits.jointVelocity.activate) {
    problem_.inequalityConstraintPtr->add(
      "jointVelocityLimits",
      createStateSubvectorBoxConstraint(
        inverse_dynamics_model_.inputDim(),
        inverse_dynamics_model_.stateDim(),
        n,
        velocity_lower,
        velocity_upper));
  }

  const auto& self_collision = parameters.ocs2.task.selfCollision;
  self_collision_hard_stop_active_ = false;
  self_collision_hard_stop_distance_ = 0.0;
  self_collision_distance_evaluator_.reset();
  if (self_collision.activate) {
    if (self_collision.hardStopDistance > self_collision.minimumDistance) {
      throw std::runtime_error(
        "selfCollision.hardStopDistance must be less than or equal to selfCollision.minimumDistance.");
    }

    std::vector<std::pair<std::string, std::string>> collision_link_pairs;
    collision_link_pairs.reserve(self_collision.link_pair_names.size());
    for (const auto& pair_name : self_collision.link_pair_names) {
      const auto& entry = self_collision.link_pair_names_map.at(pair_name);
      collision_link_pairs.emplace_back(entry.link_a, entry.link_b);
    }

    std::vector<std::pair<std::size_t, std::size_t>> collision_object_pairs;
    collision_object_pairs.reserve(self_collision.object_pair_names.size());
    for (const auto& pair_name : self_collision.object_pair_names) {
      const auto& entry = self_collision.object_pair_names_map.at(pair_name);
      collision_object_pairs.emplace_back(
        static_cast<std::size_t>(entry.object_a),
        static_cast<std::size_t>(entry.object_b));
    }

    problem_.stateInequalityConstraintPtr->add(
      "selfCollision",
      constraint::createDynamicsSelfCollisionConstraint(
        *pinocchio_interface_ptr_,
        n,
        collision_link_pairs,
        collision_object_pairs,
        self_collision.minimumDistance,
        "inverse_dynamics_self_collision",
        parameters.paths.libFolder,
        recompile_libraries,
        true));

    self_collision_hard_stop_active_ = self_collision.hardStopDistance > 0.0;
    self_collision_hard_stop_distance_ = self_collision.hardStopDistance;
    self_collision_distance_evaluator_ =
      std::make_unique<constraint::DynamicsSelfCollisionDistanceEvaluator>(
      *pinocchio_interface_ptr_,
      collision_link_pairs,
      collision_object_pairs);
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
  initializer_ptr_ = std::make_unique<InverseDynamicsInitializer>(
    *pinocchio_interface_ptr_,
    inverse_dynamics_model_);
}

double InverseDynamicsMpcInterface::computeMinimumSelfCollisionDistance(
  const ocs2::vector_t& q) const
{
  if (!self_collision_distance_evaluator_) {
    return std::numeric_limits<double>::infinity();
  }
  return self_collision_distance_evaluator_->computeMinimumDistance(q);
}

InverseDynamicsEvaluation InverseDynamicsMpcInterface::evaluateInverseDynamics(
  const ocs2::vector_t& q,
  const ocs2::vector_t& v,
  const ocs2::vector_t& a) const
{
  const auto& model = pinocchio_interface_ptr_->getModel();
  auto& data = pinocchio_interface_ptr_->getData();
  if (q.size() != model.nq || v.size() != model.nv || a.size() != model.nv) {
    throw std::invalid_argument("Inverse-dynamics q, v, or a dimension does not match the Pinocchio model.");
  }

  InverseDynamicsEvaluation evaluation;
  evaluation.rnea_torque = pinocchio::rnea(model, data, q, v, a);

  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  evaluation.ee_jacobian = ocs2::matrix_t::Zero(6, model.nv);
  pinocchio::getFrameJacobian(
    model,
    data,
    inverse_dynamics_model_.endEffectorFrameId(),
    pinocchio::LOCAL_WORLD_ALIGNED,
    evaluation.ee_jacobian);

  return evaluation;
}

ocs2::vector_t InverseDynamicsMpcInterface::computeInverseDynamicsTorque(
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

ocs2::vector_t InverseDynamicsMpcInterface::computeInverseDynamicsTorqueWithEeWrench(
  const ocs2::vector_t& q,
  const ocs2::vector_t& v,
  const ocs2::vector_t& a,
  const ocs2::vector_t& eeWrenchEnvOnRobot) const
{
  if (eeWrenchEnvOnRobot.size() != 6) {
    throw std::invalid_argument("End-effector wrench must contain six values.");
  }
  const auto evaluation = evaluateInverseDynamics(q, v, a);
  return evaluation.rnea_torque - evaluation.ee_jacobian.transpose() * eeWrenchEnvOnRobot;
}

ocs2::vector_t InverseDynamicsMpcInterface::computeNonlinearEffects(
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

ocs2::vector_t InverseDynamicsMpcInterface::computeGravityCompensation(const ocs2::vector_t& q) const
{
  const std::size_t n = inverse_dynamics_model_.jointDim();
  return computeNonlinearEffects(q, ocs2::vector_t::Zero(n));
}

}  // namespace dynamics_mpc_controller
