#pragma once

#include <mutex>

#include <humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h>

namespace g1_backflip {

/** Mode schedule of a backflip: both feet down until lift-off, flight until touchdown, both feet down after. */
inline ocs2::ModeSchedule backflipModeSchedule(ocs2::scalar_t liftoffTime, ocs2::scalar_t touchdownTime) {
  using namespace ocs2::humanoid;
  return ocs2::ModeSchedule({liftoffTime, touchdownTime}, {ModeNumber::STANCE, ModeNumber::FLY, ModeNumber::STANCE});
}

/**
 * wb_humanoid_mpc's reference manager with a fixed mode schedule instead of the walking gait scheduler: the schedule
 * is set from outside (thread-safe) and used as is in every solve. Arm swing references are off.
 */
class BackflipReferenceManager final : public ocs2::humanoid::SwitchedModelReferenceManager {
 public:
  using ocs2::humanoid::SwitchedModelReferenceManager::SwitchedModelReferenceManager;

  /** Thread-safe; takes effect at the next solve. */
  void setBackflipSchedule(const ocs2::ModeSchedule& schedule) {
    std::lock_guard<std::mutex> lock(scheduleMutex_);
    schedule_ = schedule;
  }

 protected:
  void modifyReferences(ocs2::scalar_t /*initTime*/, ocs2::scalar_t /*finalTime*/, const ocs2::vector_t& /*initState*/,
                        size_t /*initMode*/, ocs2::TargetTrajectories& /*targetTrajectories*/, ocs2::ModeSchedule& modeSchedule) override {
    {
      std::lock_guard<std::mutex> lock(scheduleMutex_);
      modeSchedule = schedule_;
    }
    getSwingTrajectoryPlanner()->update(modeSchedule, 0.0);
    modeSchedule_ = modeSchedule;
  }

 private:
  std::mutex scheduleMutex_;
  ocs2::ModeSchedule schedule_{{}, {ocs2::humanoid::ModeNumber::STANCE}};
};

}  // namespace g1_backflip
