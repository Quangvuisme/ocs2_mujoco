#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <mujoco/mujoco.h>

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_sqp/SqpMpc.h>

#include "g1_backflip_mujoco/BackflipPlanner.h"
#include "g1_backflip_mujoco/G1WbBackflipInterface.h"
#include "g1_backflip_mujoco/controller/MujocoG1Bridge.h"

namespace g1_backflip {

struct ControllerSettings {
  double holdKp = 300.0;  // [Nm/rad] joint PD that holds the start posture until the MPC runs
  double holdKd = 8.0;
  ocs2::vector_t jointKp;  // joint PD around the MPC solution (MPC joints)
  ocs2::vector_t jointKd;
  double wristKp = 20.0;  // joints that are not in the MPC (wrists): PD to zero
  double wristKd = 0.5;
  double touchdownWindow = 0.12;  // [s] the MPC touchdown follows the measured one within this of the plan
};

ControllerSettings loadControllerSettings(const std::string& file, size_t numJoints);

/**
 * Backflip controller of the 29-DoF G1 in MuJoCo, built on the whole-body NMPC of wb_humanoid_mpc.
 *
 *   t < mpcStartTime    : joint PD holds the start posture (the robot starts standing on the plan's footprint)
 *   t >= mpcStartTime   : OCS2 SQP NMPC (whole-body dynamics) tracks the offline plan, which starts at flipStartTime
 *                         (stand, crouch, push, flight, landing, stand) and holds its first / last state outside it
 *   every physics step  : joint torques of the MPC solution (inverse dynamics of its wrenches and joint accelerations,
 *                         as upstream's WBMpcMrtJointController) + joint PD to the MPC's joint positions / velocities
 *
 * Lift-off / touchdown: the mode schedule follows the measured events (both feet off the floor for 5 ms / a foot
 * touches the floor) within +-touchdownWindow of the plan; the post-landing targets are shifted with the touchdown.
 */
class BackflipController {
 public:
  BackflipController(const std::string& taskFile, const std::string& urdfFile, const mjModel* model, BackflipPlan plan,
                     double mpcStartTime, double flipStartTime);

  /** Initial MuJoCo pose: the plan's first state (feet on the ground). */
  void setInitialPose(const mjModel* m, mjData* d);

  /** Computes d->ctrl for the next physics step. Call with the simulation locked. */
  void preStep(const mjModel* m, mjData* d);

  /** Runs one MPC iteration if the MPC is active. Returns true if a solve was performed. */
  bool advanceMpc();

  /** Call after the simulation was reset. */
  void reset();

  bool isMpcActive() const { return mpcActive_.load(); }
  double mpcFrequency() const { return interface_->mpcSettings().mpcDesiredFrequency_; }
  double lastSolveDuration() const { return lastSolveDuration_.load(); }
  const MujocoG1Bridge& bridge() const { return *bridge_; }
  const BackflipPlan& plan() const { return plan_; }
  double flipStartTime() const { return flipStartTime_; }
  double scheduledTouchdownTime() const { return touchdownTime_; }

  /** Planned robot pitch at time t (unwrapped, 0 upright, -2 pi after the flip). */
  double plannedPitch(double t) const;
  /** Last measured MPC state, MPC policy state and applied joint torques (MPC joints). */
  const ocs2::vector_t& measuredState() const { return measuredState_; }
  const ocs2::vector_t& desiredState() const { return desiredState_; }
  const ocs2::vector_t& appliedTorques() const { return appliedTorques_; }
  const ocs2::vector_t& desiredInput() const { return desiredInput_; }

  /** Number of feet (0..2) with a sole sphere touching the floor. */
  int feetInContact(const mjModel* m, const mjData* d) const;

 private:
  void startMpc(double time, const ocs2::vector_t& state);
  ocs2::TargetTrajectories buildTargets(double touchdownShift) const;
  void updateLiftoff(double time, int feetInContact);
  void updateTouchdown(double time, int feetInContact);
  void applySchedule();

  ControllerSettings settings_;
  std::unique_ptr<G1WbBackflipInterface> interface_;
  std::unique_ptr<ocs2::SqpMpc> mpc_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt_;
  std::unique_ptr<MujocoG1Bridge> bridge_;

  BackflipPlan plan_;
  double mpcStartTime_;
  double flipStartTime_;
  double liftoffTime_ = 0.0;    // scheduled (absolute)
  double touchdownTime_ = 0.0;  // scheduled (absolute)
  bool liftoffDetected_ = false;
  bool touchdownDetected_ = false;
  double feetOffSince_ = -1.0;
  std::vector<int> footBodies_;  // left, right ankle roll links

  std::mutex mpcMutex_;
  std::atomic<bool> mpcActive_{false};
  std::atomic<double> lastSolveDuration_{0.0};
  ocs2::vector_t holdJoints_;
  ocs2::vector_t measuredState_, desiredState_, desiredInput_, appliedTorques_;
};

}  // namespace g1_backflip
