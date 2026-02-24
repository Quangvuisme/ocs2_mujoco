#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "anymal_standing_online/foot_planner/SwingTrajectoryPlanner.h"
#include "anymal_standing_online/gait/GaitSchedule.h"
#include "anymal_standing_online/gait/MotionPhaseDefinition.h"

namespace anymal_standing_online {
    /**
    * Manages the ModeSchedule and the TargetTrajectories for switched model.
    */
    class SwitchedModelReferenceManager : public ocs2::ReferenceManager {
    public:
        SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                      std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

        ~SwitchedModelReferenceManager() override = default;

        void setModeSchedule(const ocs2::ModeSchedule &modeSchedule) override;

        contact_flag_t getContactFlags(ocs2::scalar_t time) const;

        const std::shared_ptr<GaitSchedule> &getGaitSchedule() { return gaitSchedulePtr_; }

        const std::shared_ptr<SwingTrajectoryPlanner> &getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

    private:
        void modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t &initState,
                              ocs2::TargetTrajectories &targetTrajectories,
                              ocs2::ModeSchedule &modeSchedule) override;

        std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
        std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
    };
}
