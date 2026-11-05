#pragma once

#include <mutex>
#include <string>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include "g1_perceptive_mujoco/foot_planner/SwingTrajectoryPlanner.h"
#include "g1_perceptive_mujoco/gait/GaitSchedule.h"
#include "g1_perceptive_mujoco/gait/ModeSequenceTemplate.h"
#include "g1_perceptive_mujoco/gait/MotionPhaseDefinition.h"

namespace g1_perceptive {

/**
 * Manages the ModeSchedule and the TargetTrajectories for the switched (contact) model.
 *
 * Gait changes can be requested from any thread with requestGait(). They are applied inside
 * modifyReferences(), i.e. on the MPC thread right before a solve, so the gait schedule is never
 * modified while the solver reads it.
 */
class SwitchedModelReferenceManager : public ocs2::ReferenceManager {
 public:
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr);

  ~SwitchedModelReferenceManager() override = default;

  void setModeSchedule(const ocs2::ModeSchedule& modeSchedule) override;

  contact_flag_t getContactFlags(ocs2::scalar_t time) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

  /** File with the gait templates (see config/command/gait.info). */
  void loadGaitDefinitions(const std::string& gaitFile) { gaitFile_ = gaitFile; }

  /** Thread-safe: queues a gait change (name as in gait.info), applied at the next MPC iteration. */
  void requestGait(const std::string& gaitName);

  /** Name of the gait that is currently scheduled. */
  std::string getCurrentGaitName() const;

  /** Height of the contact frames above the (flat) ground while in contact: start / end of the swing splines. */
  void setContactHeight(ocs2::scalar_t height) { contactHeight_ = height; }

 protected:
  void modifyReferences(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t& initState,
                        ocs2::TargetTrajectories& targetTrajectories, ocs2::ModeSchedule& modeSchedule) override;

  /** Applies a pending gait request and returns the mode schedule over the extended horizon. */
  ocs2::ModeSchedule updateModeSchedule(ocs2::scalar_t initTime, ocs2::scalar_t finalTime);

 private:
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;

  ocs2::scalar_t contactHeight_ = 0.0;
  std::string gaitFile_;
  mutable std::mutex gaitMutex_;
  std::string currentGaitName_ = "stance";
  std::string requestedGaitName_;
};

}  // namespace g1_perceptive
