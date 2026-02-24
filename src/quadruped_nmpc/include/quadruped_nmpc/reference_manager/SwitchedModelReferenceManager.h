#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "quadruped_nmpc/foot_planner/SwingTrajectoryPlanner.h"
#include "quadruped_nmpc/gait/GaitSchedule.h"
#include "quadruped_nmpc/gait/MotionPhaseDefinition.h"
#include "quadruped_nmpc/gait/ModeSequenceTemplate.h"

namespace quadruped_nmpc {
    /**
    * Manages the ModeSchedule and the TargetTrajectories for switched model.
    */
    class SwitchedModelReferenceManager : public ocs2::ReferenceManager {
    public:
        SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                      std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                      const std::string& gaitFile = "");

        ~SwitchedModelReferenceManager() override = default;

        void setModeSchedule(const ocs2::ModeSchedule &modeSchedule) override;

        contact_flag_t getContactFlags(ocs2::scalar_t time) const;

        const std::shared_ptr<GaitSchedule> &getGaitSchedule() { return gaitSchedulePtr_; }

        const std::shared_ptr<SwingTrajectoryPlanner> &getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

        /**
         * Set the current gait by name. Gait names must match those defined in gait.info.
         * @param gaitName: Name of the gait (e.g., "stance", "trot", "pace")
         * @param currentTime: Current simulation time for gait transition timing
         */
        void setGait(const std::string& gaitName, ocs2::scalar_t currentTime);
        
        /**
         * Get the current gait name
         */
        const std::string& getCurrentGaitName() const { return currentGaitName_; }
        
        /**
         * Load gait definitions from file
         */
        void loadGaitDefinitions(const std::string& gaitFile);

    private:
        void modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t &initState,
                              ocs2::TargetTrajectories &targetTrajectories,
                              ocs2::ModeSchedule &modeSchedule) override;

        std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
        std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
        
        // Gait definitions loaded from file
        std::string gaitFile_;
        std::string currentGaitName_ = "stance";
        bool gaitChangeRequested_ = false;
        std::string requestedGaitName_;
        ocs2::scalar_t gaitChangeTime_ = 0.0;
    };
}
