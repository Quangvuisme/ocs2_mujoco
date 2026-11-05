#include <pinocchio/fwd.hpp>  // must come before Eigen / OCS2 headers that pull in pinocchio

#include "g1_perceptive_mujoco/G1Interface.h"

#include <iostream>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/info_parser.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include "g1_perceptive_mujoco/LeggedRobotPreComputation.h"
#include "g1_perceptive_mujoco/constraint/FrictionConeConstraint.h"
#include "g1_perceptive_mujoco/constraint/NormalVelocityConstraintCppAd.h"
#include "g1_perceptive_mujoco/constraint/FootRollConstraint.h"
#include "g1_perceptive_mujoco/constraint/HeelVelocityConstraint.h"
#include "g1_perceptive_mujoco/constraint/JointLimitsConstraint.h"
#include "g1_perceptive_mujoco/constraint/TerrainConstraints.h"
#include "g1_perceptive_mujoco/constraint/ZeroForceConstraint.h"
#include "g1_perceptive_mujoco/constraint/ZeroVelocityConstraintCppAd.h"
#include "g1_perceptive_mujoco/cost/LeggedRobotQuadraticTrackingCost.h"
#include "g1_perceptive_mujoco/dynamics/LeggedRobotDynamicsAD.h"

namespace g1_perceptive {

namespace {
void checkFile(const std::string& file, const std::string& what) {
  if (!boost::filesystem::exists(boost::filesystem::path(file))) {
    throw std::invalid_argument("[G1Interface] " + what + " not found: " + file);
  }
}
}  // namespace

G1Interface::G1Interface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                         std::shared_ptr<TerrainMapBuffer> terrainBuffer)
    : terrainBuffer_(std::move(terrainBuffer)) {
  checkFile(taskFile, "task file");
  checkFile(urdfFile, "URDF file");
  checkFile(referenceFile, "reference file");

  bool verbose = false;
  ocs2::loadData::loadCppDataType(taskFile, "g1_interface.verbose", verbose);

  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc", verbose);
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp", verbose);
  rolloutSettings_ = ocs2::rollout::loadSettings(taskFile, "rollout", verbose);
  ocs2::loadData::loadCppDataType(taskFile, "g1_interface.contactHeight", contactHeight_);
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    ocs2::loadData::loadPtreeValue(pt, jointLimitMargin_, "jointLimits.margin", verbose);
    ocs2::loadData::loadPtreeValue(pt, jointLimitMu_, "jointLimits.mu", verbose);
    ocs2::loadData::loadPtreeValue(pt, jointLimitDelta_, "jointLimits.delta", verbose);
  }
  perceptionSettings_ = loadPerceptionSettings(taskFile, "perception", verbose);

  setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void G1Interface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                           const std::string& referenceFile, bool verbose) {
  // model
  pinocchioInterfacePtr_ = std::make_unique<ocs2::PinocchioInterface>(
      ocs2::centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames));
  centroidalModelInfo_ = ocs2::centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, ocs2::centroidal_model::loadCentroidalType(taskFile),
      ocs2::centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
      modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);

  initialState_.setZero(centroidalModelInfo_.stateDim);
  ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  // reference manager: gait + terrain-aware swing + footholds + posture
  auto swingTrajectoryPlanner = std::make_shared<SwingTrajectoryPlanner>(
      loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), centroidalModelInfo_.numThreeDofContacts);
  referenceManagerPtr_ = std::make_shared<PerceptiveReferenceManager>(
      centroidalModelInfo_, loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner),
      *pinocchioInterfacePtr_, initialState_, perceptionSettings_, terrainBuffer_);

  problemPtr_ = std::make_unique<ocs2::OptimalControlProblem>();

  // dynamics
  problemPtr_->dynamicsPtr = std::make_unique<LeggedRobotDynamicsAD>(*pinocchioInterfacePtr_, centroidalModelInfo_,
                                                                     "dynamics", modelSettings_);

  // cost
  problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, verbose));

  // contact constraints
  ocs2::scalar_t frictionCoefficient = 0.7;
  ocs2::RelaxedBarrierPenalty::Config frictionBarrier;
  std::tie(frictionCoefficient, frictionBarrier) = loadFrictionConeSettings(taskFile, verbose);

  const auto infoCppAd = centroidalModelInfo_.toCppAd();
  const ocs2::CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
  auto velocityUpdateCallback = [&infoCppAd](const ocs2::ad_vector_t& state, ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
    const ocs2::ad_vector_t q = ocs2::centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
    updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
  };

  // contact point kinematics; contacts {0, 1} are the toes, {2, 3} the heels of {left, right}
  const size_t numContacts = centroidalModelInfo_.numThreeDofContacts;
  std::vector<std::unique_ptr<ocs2::PinocchioEndEffectorKinematicsCppAd>> contactKinematics;
  for (size_t i = 0; i < numContacts; i++) {
    const std::string& footName = modelSettings_.contactNames3DoF[i];
    contactKinematics.emplace_back(std::make_unique<ocs2::PinocchioEndEffectorKinematicsCppAd>(
        *pinocchioInterfacePtr_, pinocchioMappingCppAd, std::vector<std::string>{footName}, centroidalModelInfo_.stateDim,
        centroidalModelInfo_.inputDim, velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
        modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
  }

  const auto& ps = perceptionSettings_;
  for (size_t i = 0; i < numContacts; i++) {
    const std::string& footName = modelSettings_.contactNames3DoF[i];
    const auto& eeKinematics = *contactKinematics[i];
    const bool isHeel = i >= numContacts / 2;

    // friction cone (relaxed barrier: DDP does not enforce hard inequalities)
    FrictionConeConstraint::Config frictionConfig(frictionCoefficient);
    problemPtr_->softConstraintPtr->add(
        footName + "_frictionCone",
        std::make_unique<ocs2::StateInputSoftConstraint>(
            std::make_unique<FrictionConeConstraint>(*referenceManagerPtr_, std::move(frictionConfig), i, centroidalModelInfo_),
            std::make_unique<ocs2::RelaxedBarrierPenalty>(frictionBarrier)));

    problemPtr_->equalityConstraintPtr->add(
        footName + "_zeroForce", std::make_unique<ZeroForceConstraint>(*referenceManagerPtr_, i, centroidalModelInfo_));

    // stance: toe velocity zero (3), heel velocity perpendicular to the toe-heel axis zero (2) -> the foot is fixed
    // except for a roll about its sole axis, which the ankle roll constraint below removes
    if (isHeel) {
      problemPtr_->equalityConstraintPtr->add(
          footName + "_zeroVelocity",
          std::make_unique<HeelVelocityConstraint>(*referenceManagerPtr_, eeKinematics, *contactKinematics[i - numContacts / 2], i));
    } else {
      EndEffectorLinearConstraint::Config zeroVelocityConfig;
      zeroVelocityConfig.b.setZero(3);
      zeroVelocityConfig.Av.setIdentity(3, 3);
      problemPtr_->equalityConstraintPtr->add(
          footName + "_zeroVelocity",
          std::make_unique<ZeroVelocityConstraintCppAd>(*referenceManagerPtr_, eeKinematics, i, zeroVelocityConfig));
    }

    // swing: z velocity (and position, via positionErrorGain) along the terrain-aware swing spline
    problemPtr_->equalityConstraintPtr->add(
        footName + "_normalVelocity", std::make_unique<NormalVelocityConstraintCppAd>(*referenceManagerPtr_, eeKinematics, i));

    // perception: stance toe / heel inside its foothold region
    if (ps.enableFootPlacement) {
      problemPtr_->stateSoftConstraintPtr->add(
          footName + "_footPlacement",
          std::make_unique<ocs2::StateSoftConstraint>(
              std::make_unique<FootPlacementConstraint>(*referenceManagerPtr_, eeKinematics, i,
                                                        static_cast<size_t>(std::max(3, ps.foothold.numVertices))),
              std::make_unique<ocs2::RelaxedBarrierPenalty>(
                  ocs2::RelaxedBarrierPenalty::Config(ps.footPlacementMu, ps.footPlacementDelta))));
    }

    // perception: swing toe / heel above the terrain
    if (ps.enableSwingClearance) {
      problemPtr_->stateSoftConstraintPtr->add(
          footName + "_swingClearance",
          std::make_unique<ocs2::StateSoftConstraint>(
              std::make_unique<FootClearanceConstraint>(*referenceManagerPtr_, eeKinematics, i),
              std::make_unique<ocs2::RelaxedBarrierPenalty>(
                  ocs2::RelaxedBarrierPenalty::Config(ps.swingClearanceMu, ps.swingClearanceDelta))));
    }
  }

  // perception: swing ankles and shins (ankle -> knee) above the terrain, so they do not sweep step noses
  if (ps.enableShankClearance) {
    for (const std::string side : {"left", "right"}) {
      const int toeIndex = side == "left" ? 0 : 1;  // contact flags of the leg
      const std::string knee = side + "_knee_link";
      const std::string ankle = side + "_ankle_pitch_link";
      ocs2::PinocchioEndEffectorKinematicsCppAd ankleKinematics(
          *pinocchioInterfacePtr_, pinocchioMappingCppAd, {ankle}, centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
          velocityUpdateCallback, side + "_ankle", modelSettings_.modelFolderCppAd, modelSettings_.recompileLibrariesCppAd,
          modelSettings_.verboseCppAd);
      problemPtr_->stateSoftConstraintPtr->add(
          side + "_ankleClearance",
          std::make_unique<ocs2::StateSoftConstraint>(
              std::make_unique<FootClearanceConstraint>(*referenceManagerPtr_, ankleKinematics, toeIndex, ps.ankleClearance),
              std::make_unique<ocs2::RelaxedBarrierPenalty>(
                  ocs2::RelaxedBarrierPenalty::Config(ps.swingClearanceMu, ps.swingClearanceDelta))));
      ocs2::PinocchioEndEffectorKinematicsCppAd kneeAnkleKinematics(
          *pinocchioInterfacePtr_, pinocchioMappingCppAd, {knee, ankle}, centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
          velocityUpdateCallback, side + "_shank", modelSettings_.modelFolderCppAd, modelSettings_.recompileLibrariesCppAd,
          modelSettings_.verboseCppAd);
      problemPtr_->stateSoftConstraintPtr->add(
          side + "_shankClearance",
          std::make_unique<ocs2::StateSoftConstraint>(
              std::make_unique<ShankClearanceConstraint>(*referenceManagerPtr_, kneeAnkleKinematics, std::vector<double>{0.3, 0.7},
                                                         ps.shankRadius, toeIndex),
              std::make_unique<ocs2::RelaxedBarrierPenalty>(
                  ocs2::RelaxedBarrierPenalty::Config(ps.shankClearanceMu, ps.shankClearanceDelta))));
    }
  }

  // joint position limits from the URDF (minus a margin), relaxed barrier
  if (jointLimitMargin_ >= 0.0) {
    const auto& model = pinocchioInterfacePtr_->getModel();
    const auto n = static_cast<Eigen::Index>(centroidalModelInfo_.actuatedDofNum);
    const ocs2::vector_t lower = model.lowerPositionLimit.tail(n).array() + jointLimitMargin_;
    const ocs2::vector_t upper = model.upperPositionLimit.tail(n).array() - jointLimitMargin_;
    problemPtr_->stateSoftConstraintPtr->add(
        "jointLimits", std::make_unique<ocs2::StateSoftConstraint>(
                           std::make_unique<JointLimitsConstraint>(centroidalModelInfo_, lower, upper),
                           std::make_unique<ocs2::RelaxedBarrierPenalty>(ocs2::RelaxedBarrierPenalty::Config(jointLimitMu_, jointLimitDelta_))));
  }

  // ankle roll: zero joint velocity (the toe-heel line contact gives no support about the sole axis)
  const auto& pinocchioModel = pinocchioInterfacePtr_->getModel();
  for (const auto& jointName : modelSettings_.footRollJointNames) {
    if (!pinocchioModel.existJointName(jointName)) {
      throw std::invalid_argument("[G1Interface] foot roll joint not found in the URDF: " + jointName);
    }
    const auto& joint = pinocchioModel.joints[pinocchioModel.getJointId(jointName)];
    const size_t jointIndex = static_cast<size_t>(joint.idx_v()) - 6;  // first 6 velocities: floating base
    problemPtr_->equalityConstraintPtr->add(jointName + "_footRoll",
                                            std::make_unique<FootRollConstraint>(jointIndex, centroidalModelInfo_));
  }

  // pre-computation (swing z-velocity constraint configuration)
  problemPtr_->preComputationPtr = std::make_unique<LeggedRobotPreComputation>(
      *pinocchioInterfacePtr_, centroidalModelInfo_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_);

  rolloutPtr_ = std::make_unique<ocs2::TimeTriggeredRollout>(*problemPtr_->dynamicsPtr, rolloutSettings_);

  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_ = std::make_unique<LeggedRobotInitializer>(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum);
}

std::shared_ptr<GaitSchedule> G1Interface::loadGaitSchedule(const std::string& file, bool verbose) const {
  const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);
  if (verbose) {
    std::cerr << "Initial mode schedule:\n" << initModeSchedule << "Default mode sequence template:\n"
              << defaultModeSequenceTemplate;
  }
  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings_.phaseTransitionStanceTime);
}

ocs2::matrix_t G1Interface::initializeInputCostWeight(const std::string& taskFile,
                                                                    const ocs2::CentroidalModelInfo& info) {
  const size_t totalContactDim = 3 * info.numThreeDofContacts;

  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto q = ocs2::centroidal_model::getGeneralizedCoordinates(initialState_, centroidalModelInfo_);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  ocs2::matrix_t baseToFeetJacobians(totalContactDim, info.actuatedDofNum);
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    ocs2::matrix_t jacobianWorldToContactPointInWorldFrame = ocs2::matrix_t::Zero(6, info.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]),
                                pinocchio::LOCAL_WORLD_ALIGNED, jacobianWorldToContactPointInWorldFrame);
    baseToFeetJacobians.block(3 * i, 0, 3, info.actuatedDofNum) =
        jacobianWorldToContactPointInWorldFrame.block(0, 6, 3, info.actuatedDofNum);
  }

  ocs2::matrix_t R_taskspace(totalContactDim + totalContactDim, totalContactDim + totalContactDim);
  ocs2::loadData::loadEigenMatrix(taskFile, "R", R_taskspace);

  ocs2::matrix_t R = ocs2::matrix_t::Zero(info.inputDim, info.inputDim);
  R.topLeftCorner(totalContactDim, totalContactDim) = R_taskspace.topLeftCorner(totalContactDim, totalContactDim);
  R.bottomRightCorner(info.actuatedDofNum, info.actuatedDofNum) =
      baseToFeetJacobians.transpose() * R_taskspace.bottomRightCorner(totalContactDim, totalContactDim) * baseToFeetJacobians;
  return R;
}

std::unique_ptr<ocs2::StateInputCost> G1Interface::getBaseTrackingCost(const std::string& taskFile,
                                                                                     const ocs2::CentroidalModelInfo& info,
                                                                                     bool verbose) {
  ocs2::matrix_t Q(info.stateDim, info.stateDim);
  ocs2::loadData::loadEigenMatrix(taskFile, "Q", Q);
  ocs2::matrix_t R = initializeInputCostWeight(taskFile, info);
  if (verbose) {
    std::cerr << "Q:\n" << Q << "\nR:\n" << R << "\n";
  }
  return std::make_unique<LeggedRobotStateInputQuadraticCost>(std::move(Q), std::move(R), info, *referenceManagerPtr_);
}

std::pair<ocs2::scalar_t, ocs2::RelaxedBarrierPenalty::Config> G1Interface::loadFrictionConeSettings(
    const std::string& taskFile, bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "frictionConeSoftConstraint.";
  ocs2::scalar_t frictionCoefficient = 1.0;
  ocs2::RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  ocs2::loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
  ocs2::loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  ocs2::loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  return {frictionCoefficient, barrierPenaltyConfig};
}

}  // namespace g1_perceptive
