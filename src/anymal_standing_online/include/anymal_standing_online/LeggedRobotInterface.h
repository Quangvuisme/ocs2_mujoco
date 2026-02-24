#pragma once

// ocs2
#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ipm/IpmSettings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/SqpSettings.h>

#include "anymal_standing_online/common/ModelSettings.h"
#include "anymal_standing_online/initialization/LeggedRobotInitializer.h"
#include "anymal_standing_online/reference_manager/SwitchedModelReferenceManager.h"

/**
 * LeggedRobotInterface class
 * General interface for mpc implementation on the legged robot model
 */
namespace anymal_standing_online {
    class LeggedRobotInterface final : public ocs2::RobotInterface {
    public:
        /**
        * Constructor
        *
        * @throw Invalid argument error if input task file or urdf file does not exist.
        *
        * @param [in] taskFile: The absolute path to the configuration file for the MPC.
        * @param [in] urdfFile: The absolute path to the URDF file for the robot.
        * @param [in] referenceFile: The absolute path to the reference configuration file.
        * @param [in] useHardFrictionConeConstraint: Which to use hard or soft friction cone constraints.
        */
        LeggedRobotInterface(const std::string &taskFile, const std::string &urdfFile,
                             const std::string &referenceFile,
                             bool useHardFrictionConeConstraint = false);

        ~LeggedRobotInterface() override = default;

        const ocs2::OptimalControlProblem &getOptimalControlProblem() const override { return *problemPtr_; }

        const ModelSettings &modelSettings() const { return modelSettings_; }
        const ocs2::ddp::Settings &ddpSettings() const { return ddpSettings_; }
        const ocs2::mpc::Settings &mpcSettings() const { return mpcSettings_; }
        const ocs2::rollout::Settings &rolloutSettings() const { return rolloutSettings_; }
        const ocs2::sqp::Settings &sqpSettings() { return sqpSettings_; }
        const ocs2::ipm::Settings &ipmSettings() { return ipmSettings_; }

        const ocs2::vector_t &getInitialState() const { return initialState_; }
        const ocs2::RolloutBase &getRollout() const { return *rolloutPtr_; }
        ocs2::PinocchioInterface &getPinocchioInterface() { return *pinocchioInterfacePtr_; }
        const ocs2::CentroidalModelInfo &getCentroidalModelInfo() const { return centroidalModelInfo_; }

        std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const {
            return referenceManagerPtr_;
        }

        const LeggedRobotInitializer &getInitializer() const override { return *initializerPtr_; }

        std::shared_ptr<ocs2::ReferenceManagerInterface> getReferenceManagerPtr() const override {
            return referenceManagerPtr_;
        }

    private:
        void setupOptimalConrolProblem(const std::string &taskFile, const std::string &urdfFile,
                                       const std::string &referenceFile, bool verbose);

        std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string &file, bool verbose) const;

        std::unique_ptr<ocs2::StateInputCost> getBaseTrackingCost(const std::string &taskFile,
                                                            const ocs2::CentroidalModelInfo &info, bool verbose);

        ocs2::matrix_t initializeInputCostWeight(const std::string &taskFile, const ocs2::CentroidalModelInfo &info);

        std::pair<ocs2::scalar_t, ocs2::RelaxedBarrierPenalty::Config> loadFrictionConeSettings(
            const std::string &taskFile, bool verbose) const;

        std::unique_ptr<ocs2::StateInputConstraint> getFrictionConeConstraint(
            size_t contactPointIndex, ocs2::scalar_t frictionCoefficient);

        std::unique_ptr<ocs2::StateInputCost> getFrictionConeSoftConstraint(
            size_t contactPointIndex, ocs2::scalar_t frictionCoefficient,
            const ocs2::RelaxedBarrierPenalty::Config &barrierPenaltyConfig);

        std::unique_ptr<ocs2::StateInputConstraint> getZeroForceConstraint(size_t contactPointIndex);

        std::unique_ptr<ocs2::StateInputConstraint> getZeroVelocityConstraint(
            const ocs2::EndEffectorKinematics<ocs2::scalar_t> &eeKinematics,
            size_t contactPointIndex, bool useAnalyticalGradients);

        std::unique_ptr<ocs2::StateInputConstraint> getNormalVelocityConstraint(
            const ocs2::EndEffectorKinematics<ocs2::scalar_t> &eeKinematics,
            size_t contactPointIndex, bool useAnalyticalGradients);

        ModelSettings modelSettings_;
        ocs2::ddp::Settings ddpSettings_;
        ocs2::mpc::Settings mpcSettings_;
        ocs2::sqp::Settings sqpSettings_;
        ocs2::ipm::Settings ipmSettings_;
        const bool useHardFrictionConeConstraint_;

        std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterfacePtr_;
        ocs2::CentroidalModelInfo centroidalModelInfo_;

        std::unique_ptr<ocs2::OptimalControlProblem> problemPtr_;
        std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

        ocs2::rollout::Settings rolloutSettings_;
        std::unique_ptr<ocs2::RolloutBase> rolloutPtr_;
        std::unique_ptr<LeggedRobotInitializer> initializerPtr_;

        ocs2::vector_t initialState_;
    };
}
