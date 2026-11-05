#include "g1_perceptive_mujoco/reference_manager/SwitchedModelReferenceManager.h"

#include <iostream>

namespace g1_perceptive {

SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
    : ReferenceManager(ocs2::TargetTrajectories(), ocs2::ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) {}

void SwitchedModelReferenceManager::setModeSchedule(const ocs2::ModeSchedule& modeSchedule) {
  ReferenceManager::setModeSchedule(modeSchedule);
  gaitSchedulePtr_->setModeSchedule(modeSchedule);
}

contact_flag_t SwitchedModelReferenceManager::getContactFlags(ocs2::scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

void SwitchedModelReferenceManager::requestGait(const std::string& gaitName) {
  std::lock_guard<std::mutex> lock(gaitMutex_);
  requestedGaitName_ = gaitName;
}

std::string SwitchedModelReferenceManager::getCurrentGaitName() const {
  std::lock_guard<std::mutex> lock(gaitMutex_);
  return currentGaitName_;
}

ocs2::ModeSchedule SwitchedModelReferenceManager::updateModeSchedule(ocs2::scalar_t initTime, ocs2::scalar_t finalTime) {
  std::string requested;
  {
    std::lock_guard<std::mutex> lock(gaitMutex_);
    requested.swap(requestedGaitName_);
  }
  if (!requested.empty() && requested != getCurrentGaitName()) {
    if (gaitFile_.empty()) {
      std::cerr << "[ReferenceManager] Cannot change gait to '" << requested << "': no gait file loaded\n";
    } else {
      try {
        const auto modeSequenceTemplate = loadModeSequenceTemplate(gaitFile_, requested, false);
        const ocs2::scalar_t switchTime = initTime + 0.1;
        gaitSchedulePtr_->insertModeSequenceTemplate(modeSequenceTemplate, switchTime, finalTime + 10.0);
        std::lock_guard<std::mutex> lock(gaitMutex_);
        currentGaitName_ = requested;
        std::cout << "[ReferenceManager] Gait -> " << requested << " at t=" << switchTime << "\n";
      } catch (const std::exception& e) {
        std::cerr << "[ReferenceManager] Failed to load gait '" << requested << "': " << e.what() << "\n";
      }
    }
  }

  const auto timeHorizon = finalTime - initTime;
  return gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
}

void SwitchedModelReferenceManager::modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime,
                                                     const ocs2::vector_t& /*initState*/,
                                                     ocs2::TargetTrajectories& /*targetTrajectories*/,
                                                     ocs2::ModeSchedule& modeSchedule) {
  modeSchedule = updateModeSchedule(initTime, finalTime);
  swingTrajectoryPtr_->update(modeSchedule, contactHeight_);
}

}  // namespace g1_perceptive
