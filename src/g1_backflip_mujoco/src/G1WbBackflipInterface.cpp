#include <pinocchio/fwd.hpp>  // must come before Eigen / OCS2 headers that pull in pinocchio

#include "g1_backflip_mujoco/G1WbBackflipInterface.h"

#include <algorithm>
#include <iostream>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include <humanoid_common_mpc/HumanoidCostConstraintFactory.h>
#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include <humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h>
#include <humanoid_wb_mpc/WBMpcPreComputation.h>
#include <humanoid_wb_mpc/constraint/ZeroAccelerationConstraintCppAd.h>
#include <humanoid_wb_mpc/dynamics/WBAccelDynamicsAD.h>
#include <humanoid_wb_mpc/end_effector/PinocchioEndEffectorDynamicsCppAd.h>

#include "g1_backflip_mujoco/common/LoadVector.h"
#include "g1_backflip_mujoco/constraint/BaseHeightConstraint.h"
#include "g1_backflip_mujoco/constraint/FootClearanceConstraint.h"
#include "g1_backflip_mujoco/constraint/JointTorqueLimitConstraint.h"
#include "g1_backflip_mujoco/cost/SymmetryCost.h"
#include "g1_backflip_mujoco/cost/TrackingCost.h"

namespace g1_backflip {

using namespace ocs2;
using namespace ocs2::humanoid;

namespace {
void checkFile(const std::string& file, const std::string& what) {
  if (!boost::filesystem::exists(boost::filesystem::path(file))) {
    throw std::invalid_argument("[G1WbBackflipInterface] " + what + " not found: " + file);
  }
}

/** Diagonal Q (state 58) / R (input 35) from the "weights" block. */
std::pair<matrix_t, matrix_t> loadWeights(const boost::property_tree::ptree& pt, size_t numJoints) {
  vector_t q(12 + 2 * numJoints);
  q << loadVectorString(pt, "weights.basePose", 6), loadVectorString(pt, "weights.joints", numJoints),
      loadVectorString(pt, "weights.baseVelocity", 6), loadVectorString(pt, "weights.jointVelocities", numJoints);
  const vector_t wrench = loadVectorString(pt, "weights.wrench", 6);
  vector_t r(6 * N_CONTACTS + numJoints);
  for (size_t i = 0; i < N_CONTACTS; ++i) {
    r.segment<6>(6 * i) = wrench;
  }
  r.tail(numJoints) = loadVectorString(pt, "weights.jointAccelerations", numJoints);
  return {q.asDiagonal(), r.asDiagonal()};
}
}  // namespace

G1WbBackflipInterface::G1WbBackflipInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& solverPrefix)
    : taskFile_(taskFile) {
  checkFile(taskFile, "task file");
  checkFile(urdfFile, "URDF file");
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  bool verbose = false;
  loadData::loadPtreeValue(pt, verbose, "interface.verbose", false);

  modelSettingsPtr_ = std::make_unique<ModelSettings>(taskFile, urdfFile, "wb_mpc_", verbose);
  loadData::loadPtreeValue(pt, modelSettingsPtr_->modelFolderCppAd, "model_settings.modelFolderCppAd", verbose);
  const ModelSettings& modelSettings = *modelSettingsPtr_;

  sqpSettings_ = sqp::loadSettings(taskFile, solverPrefix + ".multiple_shooting", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, solverPrefix + ".mpc", verbose);

  // model: Pinocchio from the URDF (root flip_base) + 6-DoF contact frames on the soles
  pinocchioInterfacePtr_ = std::make_unique<PinocchioInterface>(createCustomPinocchioInterface(taskFile, urdfFile, modelSettings));
  mpcRobotModelPtr_ = std::make_unique<WBAccelMpcRobotModel<scalar_t>>(modelSettings);
  mpcRobotModelADPtr_ = std::make_unique<WBAccelMpcRobotModel<ad_scalar_t>>(modelSettings);
  {
    pinocchio::Data data(pinocchioInterfacePtr_->getModel());
    totalMass_ = pinocchio::computeTotalMass(pinocchioInterfacePtr_->getModel());
  }

  // reference manager with a fixed (backflip) mode schedule; the swing planner is only used by the pre-computation
  auto swingTrajectoryPlanner =
      std::make_unique<SwingTrajectoryPlanner>(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), N_CONTACTS);
  auto gaitSchedule = std::make_shared<GaitSchedule>(ModeSchedule({}, {ModeNumber::STANCE}),
                                                     ModeSequenceTemplate({0.0, 1.0}, {ModeNumber::STANCE}), 0.0);
  referenceManagerPtr_ = std::make_shared<BackflipReferenceManager>(gaitSchedule, std::move(swingTrajectoryPlanner),
                                                                    *pinocchioInterfacePtr_, *mpcRobotModelPtr_);
  referenceManagerPtr_->setArmSwingReferenceActive(false);

  HumanoidCostConstraintFactory factory(taskFile, taskFile, *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelPtr_,
                                        *mpcRobotModelADPtr_, modelSettings, verbose);

  problemPtr_ = std::make_unique<OptimalControlProblem>();
  problemPtr_->dynamicsPtr = std::make_unique<WBAccelDynamicsAD>(*pinocchioInterfacePtr_, *mpcRobotModelADPtr_, "dynamics", modelSettings);

  // cost
  auto [Q, R] = loadWeights(pt, mpcRobotModelPtr_->getJointDim());
  // base velocity weights x baseVelocityScale, input weights x inputScale (the online MPC tracks the plan's momentum
  // build-up and its contact forces / joint accelerations more tightly instead of re-optimising them)
  {
    const size_t nj = mpcRobotModelPtr_->getJointDim();
    Q.block(6 + nj, 6 + nj, 6, 6) *= pt.get<double>(solverPrefix + ".baseVelocityScale", 1.0);
    R *= pt.get<double>(solverPrefix + ".inputScale", 1.0);
  }
  // flight: base pose / velocity weights x flightBaseScale, joint position / velocity weights x flightJointScale
  matrix_t QFlight = Q;
  {
    const size_t nj = mpcRobotModelPtr_->getJointDim();
    const scalar_t baseScale = pt.get<double>(solverPrefix + ".flightBaseScale");
    const scalar_t jointScale = pt.get<double>(solverPrefix + ".flightJointScale");
    QFlight.block(0, 0, 6, 6) *= baseScale;
    QFlight.block(6 + nj, 6 + nj, 6, 6) *= baseScale;
    QFlight.block(6, 6, nj, nj) *= jointScale;
    QFlight.block(12 + nj, 12 + nj, nj, nj) *= jointScale;
  }
  problemPtr_->costPtr->add("tracking", std::make_unique<StateInputTrackingCost>(
                                            Q, QFlight, R, 6 * N_CONTACTS, *referenceManagerPtr_,
                                            pt.get<double>(solverPrefix + ".liftoffForceWeight", pt.get<double>("weights.liftoffForceWeight")),
                                            pt.get<double>("weights.liftoffRampTime")));
  problemPtr_->finalCostPtr->add("finalTracking",
                                 std::make_unique<FinalStateTrackingCost>(pt.get<double>("weights.finalWeightScale") * Q));

  // left / right symmetry of the joints (positions and velocities)
  {
    const auto& names = modelSettings.mpcModelJointNames;
    const size_t nj = names.size();
    std::vector<SymmetryCost::Pair> pairs;
    for (size_t l = 0; l < nj; ++l) {
      if (names[l].rfind("left_", 0) != 0) {
        continue;
      }
      const std::string mirrored = "right_" + names[l].substr(5);
      const auto it = std::find(names.begin(), names.end(), mirrored);
      if (it == names.end()) {
        continue;
      }
      const size_t r = static_cast<size_t>(std::distance(names.begin(), it));
      const bool pitchJoint = names[l].find("pitch") != std::string::npos || names[l].find("knee") != std::string::npos ||
                              names[l].find("elbow") != std::string::npos;
      const double sign = pitchJoint ? 1.0 : -1.0;
      pairs.push_back({6 + l, 6 + r, sign});
      pairs.push_back({12 + nj + l, 12 + nj + r, sign});
    }
    problemPtr_->stateCostPtr->add("symmetry", std::make_unique<SymmetryCost>(pairs, pt.get<double>("weights.symmetry")));
  }

  // soft constraints of the upstream factory
  problemPtr_->stateSoftConstraintPtr->add("jointLimits", factory.getJointLimitsConstraint());
  problemPtr_->stateSoftConstraintPtr->add("footCollision", factory.getFootCollisionConstraint());

  // joint torque limits (inverse dynamics of the whole-body model)
  if (pt.get<bool>("torqueLimits.enable")) {
    const auto& model = pinocchioInterfacePtr_->getModel();
    const vector_t limits = pt.get<double>("torqueLimits.scale") * model.effortLimit.tail(mpcRobotModelPtr_->getJointDim());
    RelaxedBarrierPenalty::Config barrier;
    barrier.mu = pt.get<double>("torqueLimits.mu");
    barrier.delta = pt.get<double>("torqueLimits.delta");
    problemPtr_->softConstraintPtr->add(
        "jointTorqueLimits", std::make_unique<StateInputSoftConstraint>(
                                 std::make_unique<JointTorqueLimitConstraint>(limits, *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, modelSettings),
                                 std::make_unique<RelaxedBarrierPenalty>(barrier)));
  }

  // contacts
  const ModelSettings::FootConstraintConfig& footCfg = modelSettings.footConstraintConfig;
  for (size_t i = 0; i < N_CONTACTS; i++) {
    const std::string& footName = modelSettings.contactNames[i];
    PinocchioEndEffectorDynamicsCppAd eeDynamics(*pinocchioInterfacePtr_, *mpcRobotModelADPtr_, {footName}, footName,
                                                 modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd,
                                                 modelSettings.verboseCppAd);

    problemPtr_->softConstraintPtr->add(footName + "_frictionForceCone", factory.getFrictionForceConeConstraint(i));
    problemPtr_->softConstraintPtr->add(footName + "_contactMomentXY",
                                        factory.getContactMomentXYConstraint(i, footName + "_contact_moment_XY_constraint"));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroWrench", factory.getZeroWrenchConstraint(i));

    // in the air: the foot stays above the floor
    RelaxedBarrierPenalty::Config clearanceBarrier;
    clearanceBarrier.mu = pt.get<double>("footClearance.mu");
    clearanceBarrier.delta = pt.get<double>("footClearance.delta");
    problemPtr_->stateSoftConstraintPtr->add(
        footName + "_clearance",
        std::make_unique<StateSoftConstraint>(
            std::make_unique<FootClearanceConstraint>(*referenceManagerPtr_, eeDynamics, static_cast<int>(i),
                                                      pt.get<double>("footClearance.minHeight")),
            std::make_unique<RelaxedBarrierPenalty>(clearanceBarrier)));

    // stance: zero foot acceleration with velocity / height / orientation feedback (as WBMpcInterface)
    EndEffectorDynamicsAccelerationsConstraint::Config config;
    config.b.setZero(6);
    config.Ax.setZero(6, 6);
    config.Av.setIdentity(6, 6);
    config.Aa.setIdentity(6, 6);
    if (!numerics::almost_eq(footCfg.positionErrorGain_z, 0.0)) {
      config.Ax(2, 2) = footCfg.positionErrorGain_z;
    }
    if (!numerics::almost_eq(footCfg.orientationErrorGain, 0.0)) {
      config.Ax.block(3, 3, 3, 3) = matrix_t::Identity(3, 3) * footCfg.orientationErrorGain;
    }
    config.Av.block(0, 0, 2, 2) = matrix_t::Identity(2, 2) * footCfg.linearVelocityErrorGain_xy;
    config.Av(2, 2) = footCfg.linearVelocityErrorGain_z;
    config.Av.block(3, 3, 3, 3) = matrix_t::Identity(3, 3) * footCfg.angularVelocityErrorGain;
    config.Aa.block(0, 0, 2, 2) = matrix_t::Identity(2, 2) * footCfg.linearAccelerationErrorGain_xy;
    config.Aa(2, 2) = footCfg.linearAccelerationErrorGain_z;
    config.Aa.block(3, 3, 3, 3) = matrix_t::Identity(3, 3) * footCfg.angularAccelerationErrorGain;
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroAcceleration",
                                            std::make_unique<ZeroAccelerationConstraintCppAd>(*referenceManagerPtr_, eeDynamics, i, config));
  }

  // realistic crouch: the pelvis stays above minHeight
  {
    RelaxedBarrierPenalty::Config barrier;
    barrier.mu = pt.get<double>("pelvisHeight.mu");
    barrier.delta = pt.get<double>("pelvisHeight.delta");
    problemPtr_->stateSoftConstraintPtr->add(
        "pelvisHeight", std::make_unique<StateSoftConstraint>(std::make_unique<BaseHeightConstraint>(pt.get<double>("pelvisHeight.minHeight")),
                                                              std::make_unique<RelaxedBarrierPenalty>(barrier)));
  }

  // knees stay above the floor (landing out of the tuck)
  {
    RelaxedBarrierPenalty::Config kneeBarrier;
    kneeBarrier.mu = pt.get<double>("kneeClearance.mu");
    kneeBarrier.delta = pt.get<double>("kneeClearance.delta");
    for (const std::string frame : {"left_knee_link", "right_knee_link"}) {
      PinocchioEndEffectorDynamicsCppAd kneeKinematics(*pinocchioInterfacePtr_, *mpcRobotModelADPtr_, {frame}, frame,
                                                       modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd,
                                                       modelSettings.verboseCppAd);
      problemPtr_->stateSoftConstraintPtr->add(
          frame + "_clearance",
          std::make_unique<StateSoftConstraint>(
              std::make_unique<FootClearanceConstraint>(*referenceManagerPtr_, kneeKinematics, -1, pt.get<double>("kneeClearance.minHeight")),
              std::make_unique<RelaxedBarrierPenalty>(kneeBarrier)));
    }
  }

  problemPtr_->preComputationPtr = std::make_unique<WBMpcPreComputation>(
      *pinocchioInterfacePtr_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), *mpcRobotModelPtr_);

  initializerPtr_ = std::make_unique<BackflipInitializer>(*referenceManagerPtr_, N_CONTACTS);
}

}  // namespace g1_backflip
