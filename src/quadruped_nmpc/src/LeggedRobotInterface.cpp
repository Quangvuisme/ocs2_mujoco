#include <iostream>
#include <memory>
#include <string>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <quadruped_nmpc/LeggedRobotInterface.h>
#include <quadruped_nmpc/LeggedRobotPreComputation.h>
#include <quadruped_nmpc/constraint/FrictionConeConstraint.h>
#include <quadruped_nmpc/constraint/NormalVelocityConstraintCppAd.h>
#include <quadruped_nmpc/constraint/ZeroForceConstraint.h>
#include <quadruped_nmpc/constraint/ZeroVelocityConstraintCppAd.h>
#include <quadruped_nmpc/cost/LeggedRobotQuadraticTrackingCost.h>
#include <quadruped_nmpc/dynamics/LeggedRobotDynamicsAD.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <ocs2_centroidal_model/FactoryFunctions.h>


namespace quadruped_nmpc {
    LeggedRobotInterface::LeggedRobotInterface(const std::string &taskFile,
                                               const std::string &urdfFile,
                                               const std::string &referenceFile,
                                               bool useHardFrictionConeConstraint)
        : useHardFrictionConeConstraint_(useHardFrictionConeConstraint) {
        // check that task file exists
        const boost::filesystem::path taskFilePath(taskFile);
        if (exists(taskFilePath)) {
            std::cerr << "[LeggedRobotInterface] Loading task file: " << taskFilePath
                    << std::endl;
        } else {
            throw std::invalid_argument("[LeggedRobotInterface] Task file not found: " +
                                        taskFilePath.string());
        }
        // check that urdf file exists
        boost::filesystem::path urdfFilePath(urdfFile);
        if (exists(urdfFilePath)) {
            std::cerr << "[LeggedRobotInterface] Loading Pinocchio model from: "
                    << urdfFilePath << std::endl;
        } else {
            throw std::invalid_argument("[LeggedRobotInterface] URDF file not found: " +
                                        urdfFilePath.string());
        }
        // check that targetCommand file exists
        const boost::filesystem::path referenceFilePath(referenceFile);
        if (exists(referenceFilePath)) {
            std::cerr << "[LeggedRobotInterface] Loading target command settings from: "
                    << referenceFilePath << std::endl;
        } else {
            throw std::invalid_argument(
                "[LeggedRobotInterface] targetCommand file not found: " +
                referenceFilePath.string());
        }

        bool verbose;
        ocs2::loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose",
                                  verbose);

        // load setting from loading file
        modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
        mpcSettings_ = ocs2::mpc::loadSettings(taskFile, "mpc", verbose);
        ddpSettings_ = ocs2::ddp::loadSettings(taskFile, "ddp", verbose);
        sqpSettings_ = ocs2::sqp::loadSettings(taskFile, "sqp", verbose);
        ipmSettings_ = ocs2::ipm::loadSettings(taskFile, "ipm", verbose);
        rolloutSettings_ = ocs2::rollout::loadSettings(taskFile, "rollout", verbose);

        // OptimalConrolProblem
        setupOptimalConrolProblem(taskFile, urdfFile, referenceFile, verbose);

        // initial state
        initialState_.setZero(centroidalModelInfo_.stateDim);
        ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
    }


    void LeggedRobotInterface::setupOptimalConrolProblem(
        const std::string &taskFile,
        const std::string &urdfFile,
        const std::string &referenceFile,
        bool verbose) {
        // PinocchioInterface
        pinocchioInterfacePtr_ = std::make_unique<ocs2::PinocchioInterface>(
            ocs2::centroidal_model::createPinocchioInterface(
                urdfFile, modelSettings_.jointNames));

        // CentroidalModelInfo
        centroidalModelInfo_ = ocs2::centroidal_model::createCentroidalModelInfo(
            *pinocchioInterfacePtr_, ocs2::centroidal_model::loadCentroidalType(taskFile),
            ocs2::centroidal_model::loadDefaultJointState(
                pinocchioInterfacePtr_->getModel().nq - 6, referenceFile),
            modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);

        // Swing trajectory planner
        auto swingTrajectoryPlanner = std::make_unique<SwingTrajectoryPlanner>(
            loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose),
            4);

        // Mode schedule manager
        referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(
            loadGaitSchedule(referenceFile, verbose),
            std::move(swingTrajectoryPlanner));

        // Optimal control problem
        problemPtr_ = std::make_unique<ocs2::OptimalControlProblem>();

        // Dynamics
        bool useAnalyticalGradientsDynamics = false;
        ocs2::loadData::loadCppDataType(
            taskFile, "legged_robot_interface.useAnalyticalGradientsDynamics",
            useAnalyticalGradientsDynamics);
        std::unique_ptr<ocs2::SystemDynamicsBase> dynamicsPtr;
        if (useAnalyticalGradientsDynamics) {
            throw std::runtime_error(
                "[LeggedRobotInterface::setupOptimalConrolProblem] The analytical "
                "dynamics class is not yet implemented!");
        }
        const std::string modelName = "dynamics";
        dynamicsPtr = std::make_unique<LeggedRobotDynamicsAD>(*pinocchioInterfacePtr_,
                                                              centroidalModelInfo_, modelName,
                                                              modelSettings_);

        problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

        // Cost terms
        problemPtr_->costPtr->add(
            "baseTrackingCost",
            getBaseTrackingCost(taskFile, centroidalModelInfo_, false));

        // Constraint terms
        // friction cone settings
        ocs2::scalar_t frictionCoefficient = 0.7;
        ocs2::RelaxedBarrierPenalty::Config barrierPenaltyConfig;
        std::tie(frictionCoefficient, barrierPenaltyConfig) =
                loadFrictionConeSettings(taskFile, verbose);

        bool useAnalyticalGradientsConstraints = false;
        ocs2::loadData::loadCppDataType(
            taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints",
            useAnalyticalGradientsConstraints);
        for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
            const std::string &footName = modelSettings_.contactNames3DoF[i];

            std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t> > eeKinematicsPtr;
            if (useAnalyticalGradientsConstraints) {
                throw std::runtime_error(
                    "[LeggedRobotInterface::setupOptimalConrolProblem] The analytical "
                    "end-effector linear constraint is not implemented!");
            }
            const auto infoCppAd = centroidalModelInfo_.toCppAd();
            const ocs2::CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(
                infoCppAd);
            auto velocityUpdateCallback =
                    [&infoCppAd](const ocs2::ad_vector_t &state,
                                 ocs2::PinocchioInterfaceCppAd &pinocchioInterfaceAd) {
                const ocs2::ad_vector_t q =
                        ocs2::centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
                updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
            };
            eeKinematicsPtr.reset(new ocs2::PinocchioEndEffectorKinematicsCppAd(
                *pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
                centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
                modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

            if (useHardFrictionConeConstraint_) {
                problemPtr_->inequalityConstraintPtr->add(
                    footName + "_frictionCone",
                    getFrictionConeConstraint(i, frictionCoefficient));
            } else {
                problemPtr_->softConstraintPtr->add(
                    footName + "_frictionCone",
                    getFrictionConeSoftConstraint(i, frictionCoefficient,
                                                  barrierPenaltyConfig));
            }
            problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce",
                                                    getZeroForceConstraint(i));
            problemPtr_->equalityConstraintPtr->add(
                footName + "_zeroVelocity",
                getZeroVelocityConstraint(*eeKinematicsPtr, i,
                                          useAnalyticalGradientsConstraints));
            problemPtr_->equalityConstraintPtr->add(
                footName + "_normalVelocity",
                getNormalVelocityConstraint(*eeKinematicsPtr, i,
                                            useAnalyticalGradientsConstraints));
        }

        // Pre-computation
        problemPtr_->preComputationPtr = std::make_unique<LeggedRobotPreComputation>(
            *pinocchioInterfacePtr_, centroidalModelInfo_,
            *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_);

        // Rollout
        rolloutPtr_ = std::make_unique<ocs2::TimeTriggeredRollout>(
            *problemPtr_->dynamicsPtr, rolloutSettings_);

        // Initialization
        constexpr bool extendNormalizedMomentum = true;
        initializerPtr_ = std::make_unique<LeggedRobotInitializer>(
            centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum);
    }


    std::shared_ptr<GaitSchedule> LeggedRobotInterface::loadGaitSchedule(
        const std::string &file,
        bool verbose) const {
        const auto initModeSchedule =
                loadModeSchedule(file, "initialModeSchedule", false);
        const auto defaultModeSequenceTemplate =
                loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

        const auto defaultGait = [&] {
            Gait gait{};
            gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
            // Events: from time -> phase
            std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1,
                          defaultModeSequenceTemplate.switchingTimes.end() - 1,
                          [&](double eventTime) {
                              gait.eventPhases.push_back(eventTime / gait.duration);
                          });
            // Modes:
            gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
            return gait;
        }();

        // display
        if (verbose) {
            std::cerr << "\n#### Modes Schedule: ";
            std::cerr << "\n#### "
                    "============================================================="
                    "================\n";
            std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
            std::cerr << "Default Modes Sequence Template: \n"
                    << defaultModeSequenceTemplate;
            std::cerr << "#### "
                    "============================================================="
                    "================\n";
        }

        return std::make_shared<GaitSchedule>(
            initModeSchedule, defaultModeSequenceTemplate,
            modelSettings_.phaseTransitionStanceTime);
    }


    ocs2::matrix_t LeggedRobotInterface::initializeInputCostWeight(
        const std::string &taskFile,
        const ocs2::CentroidalModelInfo &info) {
        const size_t totalContactDim = 3 * info.numThreeDofContacts;

        ocs2::vector_t initialState(centroidalModelInfo_.stateDim);
        ocs2::loadData::loadEigenMatrix(taskFile, "initialState", initialState);

        const auto &model = pinocchioInterfacePtr_->getModel();
        auto &data = pinocchioInterfacePtr_->getData();
        const auto q = ocs2::centroidal_model::getGeneralizedCoordinates(
            initialState, centroidalModelInfo_);
        computeJointJacobians(model, data, q);
        updateFramePlacements(model, data);

        ocs2::matrix_t baseToFeetJacobians(totalContactDim, info.actuatedDofNum);
        for (size_t i = 0; i < info.numThreeDofContacts; i++) {
            ocs2::matrix_t jacobianWorldToContactPointInWorldFrame =
                    ocs2::matrix_t::Zero(6, info.generalizedCoordinatesNum);
            getFrameJacobian(
                model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]),
                pinocchio::LOCAL_WORLD_ALIGNED,
                jacobianWorldToContactPointInWorldFrame);

            baseToFeetJacobians.block(3 * i, 0, 3, info.actuatedDofNum) =
                    jacobianWorldToContactPointInWorldFrame.block(0, 6, 3,
                                                                  info.actuatedDofNum);
        }

        ocs2::matrix_t R_taskspace(totalContactDim + totalContactDim,
                             totalContactDim + totalContactDim);
        ocs2::loadData::loadEigenMatrix(taskFile, "R", R_taskspace);

        ocs2::matrix_t R = ocs2::matrix_t::Zero(info.inputDim, info.inputDim);
        // Contact Forces
        R.topLeftCorner(totalContactDim, totalContactDim) =
                R_taskspace.topLeftCorner(totalContactDim, totalContactDim);
        // Joint velocities
        R.bottomRightCorner(info.actuatedDofNum, info.actuatedDofNum) =
                baseToFeetJacobians.transpose() *
                R_taskspace.bottomRightCorner(totalContactDim, totalContactDim) *
                baseToFeetJacobians;
        return R;
    }


    std::unique_ptr<ocs2::StateInputCost> LeggedRobotInterface::getBaseTrackingCost(
        const std::string &taskFile,
        const ocs2::CentroidalModelInfo &info,
        bool verbose) {
        ocs2::matrix_t Q(info.stateDim, info.stateDim);
        ocs2::loadData::loadEigenMatrix(taskFile, "Q", Q);
        ocs2::matrix_t R = initializeInputCostWeight(taskFile, info);

        if (verbose) {
            std::cerr << "\n #### Base Tracking Cost Coefficients: ";
            std::cerr << "\n #### "
                    "============================================================="
                    "================\n";
            std::cerr << "Q:\n" << Q << "\n";
            std::cerr << "R:\n" << R << "\n";
            std::cerr << " #### "
                    "============================================================="
                    "================\n";
        }

        return std::make_unique<LeggedRobotStateInputQuadraticCost>(
            std::move(Q), std::move(R), info, *referenceManagerPtr_);
    }


    std::pair<ocs2::scalar_t, ocs2::RelaxedBarrierPenalty::Config>
    LeggedRobotInterface::loadFrictionConeSettings(const std::string &taskFile,
                                                   bool verbose) const {
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        const std::string prefix = "frictionConeSoftConstraint.";

        ocs2::scalar_t frictionCoefficient = 1.0;
        ocs2::RelaxedBarrierPenalty::Config barrierPenaltyConfig;
        if (verbose) {
            std::cerr << "\n #### Friction Cone Settings: ";
            std::cerr << "\n #### "
                    "============================================================="
                    "================\n";
        }
        ocs2::loadData::loadPtreeValue(pt, frictionCoefficient,
                                 prefix + "frictionCoefficient", verbose);
        ocs2::loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
        ocs2::loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta",
                                 verbose);
        if (verbose) {
            std::cerr << " #### "
                    "============================================================="
                    "================\n";
        }

        return {frictionCoefficient, std::move(barrierPenaltyConfig)};
    }


    std::unique_ptr<ocs2::StateInputConstraint>
    LeggedRobotInterface::getFrictionConeConstraint(size_t contactPointIndex,
                                                    ocs2::scalar_t frictionCoefficient) {
        FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
        return std::make_unique<FrictionConeConstraint>(
            *referenceManagerPtr_, std::move(frictionConeConConfig),
            contactPointIndex, centroidalModelInfo_);
    }


    std::unique_ptr<ocs2::StateInputCost>
    LeggedRobotInterface::getFrictionConeSoftConstraint(
        size_t contactPointIndex,
        ocs2::scalar_t frictionCoefficient,
        const ocs2::RelaxedBarrierPenalty::Config &barrierPenaltyConfig) {
        return std::make_unique<ocs2::StateInputSoftConstraint>(
            getFrictionConeConstraint(contactPointIndex, frictionCoefficient),
            std::make_unique<ocs2::RelaxedBarrierPenalty>(barrierPenaltyConfig));
    }


    std::unique_ptr<ocs2::StateInputConstraint>
    LeggedRobotInterface::getZeroForceConstraint(size_t contactPointIndex) {
        return std::make_unique<ZeroForceConstraint>(
            *referenceManagerPtr_, contactPointIndex, centroidalModelInfo_);
    }


    std::unique_ptr<ocs2::StateInputConstraint>
    LeggedRobotInterface::getZeroVelocityConstraint(
        const ocs2::EndEffectorKinematics<ocs2::scalar_t> &eeKinematics,
        size_t contactPointIndex,
        bool useAnalyticalGradients) {
        auto eeZeroVelConConfig = [](ocs2::scalar_t positionErrorGain) {
            EndEffectorLinearConstraint::Config config;
            config.b.setZero(3);
            config.Av.setIdentity(3, 3);
            if (!ocs2::numerics::almost_eq(positionErrorGain, 0.0)) {
                config.Ax.setZero(3, 3);
                config.Ax(2, 2) = positionErrorGain;
            }
            return config;
        };

        if (useAnalyticalGradients) {
            throw std::runtime_error(
                "[LeggedRobotInterface::getZeroVelocityConstraint] The analytical "
                "end-effector zero velocity constraint is not implemented!");
        }
        return std::make_unique<ZeroVelocityConstraintCppAd>(
            *referenceManagerPtr_, eeKinematics, contactPointIndex,
            eeZeroVelConConfig(modelSettings_.positionErrorGain));
    }


    std::unique_ptr<ocs2::StateInputConstraint>
    LeggedRobotInterface::getNormalVelocityConstraint(
        const ocs2::EndEffectorKinematics<ocs2::scalar_t> &eeKinematics,
        size_t contactPointIndex,
        bool useAnalyticalGradients) {
        if (useAnalyticalGradients) {
            throw std::runtime_error(
                "[LeggedRobotInterface::getNormalVelocityConstraint] The analytical "
                "end-effector normal velocity constraint is not implemented!");
        }
        return std::make_unique<NormalVelocityConstraintCppAd>(
            *referenceManagerPtr_, eeKinematics, contactPointIndex);
    }
} // namespace quadruped_nmpc
