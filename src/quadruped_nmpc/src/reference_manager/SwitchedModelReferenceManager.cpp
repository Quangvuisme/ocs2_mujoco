#include <quadruped_nmpc/reference_manager/SwitchedModelReferenceManager.h>
#include <iostream>


namespace quadruped_nmpc {
    SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                 std::shared_ptr<SwingTrajectoryPlanner>
                                                                 swingTrajectoryPtr,
                                                                 const std::string& gaitFile)
        : ReferenceManager(ocs2::TargetTrajectories(), ocs2::ModeSchedule()),
          gaitSchedulePtr_(std::move(gaitSchedulePtr)),
          swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
          gaitFile_(gaitFile) {
    }


    void SwitchedModelReferenceManager::setModeSchedule(const ocs2::ModeSchedule &modeSchedule) {
        ReferenceManager::setModeSchedule(modeSchedule);
        gaitSchedulePtr_->setModeSchedule(modeSchedule);
    }


    contact_flag_t SwitchedModelReferenceManager::getContactFlags(ocs2::scalar_t time) const {
        return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
    }


    void SwitchedModelReferenceManager::loadGaitDefinitions(const std::string& gaitFile) {
        gaitFile_ = gaitFile;
        std::cout << "[RefManager] Loaded gait definitions from: " << gaitFile << std::endl;
    }
    
    
    void SwitchedModelReferenceManager::setGait(const std::string& gaitName, ocs2::scalar_t currentTime) {
        if (gaitFile_.empty()) {
            std::cerr << "[RefManager] Cannot change gait: gait file not loaded" << std::endl;
            return;
        }
        
        if (gaitName == currentGaitName_) {
            return;  // Already in this gait
        }
        
        try {
            // Load the new gait template from file
            ModeSequenceTemplate newTemplate = loadModeSequenceTemplate(gaitFile_, gaitName, false);
            
            // Schedule the gait change
            const ocs2::scalar_t gaitSwitchTime = currentTime + 0.1;  // Switch after 100ms
            const ocs2::scalar_t finalTime = gaitSwitchTime + 10.0;   // Plan 10 seconds ahead
            
            gaitSchedulePtr_->insertModeSequenceTemplate(newTemplate, gaitSwitchTime, finalTime);
            
            currentGaitName_ = gaitName;
            std::cout << "[RefManager] Gait changed to: " << gaitName << " at time " << gaitSwitchTime << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[RefManager] Failed to change gait to " << gaitName << ": " << e.what() << std::endl;
        }
    }


    void SwitchedModelReferenceManager::modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime,
                                                         const ocs2::vector_t &initState,
                                                         ocs2::TargetTrajectories &targetTrajectories,
                                                         ocs2::ModeSchedule &modeSchedule) {
        const auto timeHorizon = finalTime - initTime;
        modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

        // Debug: Print mode schedule info
        static int debugCount = 0;
        if (++debugCount % 100 == 0) {  // Print every 100 calls (~1 second at 100Hz)
            std::cout << "[RefManager] ModeSchedule at t=" << initTime 
                      << " modes=[";
            for (size_t i = 0; i < std::min(modeSchedule.modeSequence.size(), size_t(5)); ++i) {
                std::cout << modeSchedule.modeSequence[i];
                if (i < std::min(modeSchedule.modeSequence.size(), size_t(5)) - 1) std::cout << ",";
            }
            std::cout << "] gait=" << currentGaitName_ << std::endl;
        }

        const ocs2::scalar_t terrainHeight = 0.0;
        swingTrajectoryPtr_->update(modeSchedule, terrainHeight);
    }
} // namespace quadruped_nmpc
