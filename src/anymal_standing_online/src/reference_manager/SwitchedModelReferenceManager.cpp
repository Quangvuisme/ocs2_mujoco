#include <anymal_standing_online/reference_manager/SwitchedModelReferenceManager.h>


namespace anymal_standing_online {
    SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                 std::shared_ptr<SwingTrajectoryPlanner>
                                                                 swingTrajectoryPtr)
        : ReferenceManager(ocs2::TargetTrajectories(), ocs2::ModeSchedule()),
          gaitSchedulePtr_(std::move(gaitSchedulePtr)),
          swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) {
    }


    void SwitchedModelReferenceManager::setModeSchedule(const ocs2::ModeSchedule &modeSchedule) {
        ReferenceManager::setModeSchedule(modeSchedule);
        gaitSchedulePtr_->setModeSchedule(modeSchedule);
    }


    contact_flag_t SwitchedModelReferenceManager::getContactFlags(ocs2::scalar_t time) const {
        return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
    }


    void SwitchedModelReferenceManager::modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime,
                                                         const ocs2::vector_t &initState,
                                                         ocs2::TargetTrajectories &targetTrajectories,
                                                         ocs2::ModeSchedule &modeSchedule) {
        const auto timeHorizon = finalTime - initTime;
        modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

        const ocs2::scalar_t terrainHeight = 0.0;
        swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
    }
} // namespace anymal_standing_online
