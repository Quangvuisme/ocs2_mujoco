#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <mujoco/mujoco.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include "g1_walking_mujoco/FSMController.h"
#include "g1_walking_mujoco/G1Interface.h"
#include "g1_walking_mujoco/controller/LegPostureIk.h"
#include "g1_walking_mujoco/controller/MujocoG1Bridge.h"

namespace g1_walking {

struct ControllerSettings {
  double standUpKp = 150.0;         // [Nm/rad] PD stand-up
  double standUpKd = 5.0;           // [Nms/rad]
  double standUpDuration = 1.5;     // [s]
  double wbcJointKp = 500.0;        // acceleration-level joint PD of the RNEA whole-body control
  double wbcJointKd = 50.0;
  double commandHorizon = 1.0;      // [s] length of the velocity command trajectory
  double maxCommandLead = 0.10;     // [m] the integrated walking command may lead / lag the base by at most this
  double maxCommandYawLead = 0.20;  // [rad]
};

ControllerSettings loadControllerSettings(const std::string& file, const std::string& prefix, bool verbose);

/**
 * Glue between MuJoCo and the G1 MPC:
 *  - FSM (PASSIVE / PD_CONTROL / STANDING / WALKING) driven by keyboard, gamepad or a script,
 *  - measured state from the simulator, command trajectories from the FSM targets,
 *  - MPC policy evaluation + RNEA whole-body torques every physics step.
 *
 * Threads: preStep() runs in the physics thread (simulation locked). advanceMpc() runs either in
 * a separate real-time MPC thread or, in lock-step mode, in the physics thread itself. The FSM can
 * be driven from any thread through withFsm().
 */
class G1Controller {
 public:
  struct Paths {
    std::string taskFile;
    std::string urdfFile;
    std::string referenceFile;
    std::string gaitFile;
  };

  G1Controller(const Paths& paths, const mjModel* model);

  /** Computes d->ctrl for the next physics step. Call with the simulation locked. */
  void preStep(const mjModel* m, mjData* d);

  /** Runs one MPC iteration if the MPC is active. Returns true if a solve was performed. */
  bool advanceMpc();

  /** Call after the simulation was reset. */
  void onSimulationReset();

  /** Thread-safe access to the FSM. */
  template <typename F>
  auto withFsm(F&& f) {
    std::lock_guard<std::mutex> lock(fsmMutex_);
    return f(fsm_);
  }

  bool isMpcActive() const { return mpcActive_.load(); }
  double mpcFrequency() const { return interface_->mpcSettings().mpcDesiredFrequency_; }
  G1Interface& interface() { return *interface_; }
  SwitchedModelReferenceManager& referenceManager() { return *interface_->getSwitchedModelReferenceManagerPtr(); }
  const MujocoG1Bridge& bridge() const { return *bridge_; }
  const ControllerSettings& settings() const { return settings_; }
  const ocs2::vector_t& defaultJointPositions() const { return defaultJoints_; }

  /** Timing of the last MPC solve [s]. */
  double lastSolveDuration() const { return lastSolveDuration_.load(); }

 private:
  void startMpc(double time, const ocs2::vector_t& state, FSMState fsmState);
  void stopMpc();
  /** Integrates the walking velocity command, kept within maxCommandLead of the measured pose. */
  void updateWalkAnchor(double time, const ocs2::vector_t& state, const WalkingTarget& walk);
  ocs2::TargetTrajectories buildCommand(double time, const ocs2::vector_t& state, FSMState fsmState,
                                        const WalkingTarget& walk, const StandingTarget& stand) const;
  ocs2::vector_t wholeBodyTorques(const ocs2::vector_t& desiredState, const ocs2::vector_t& desiredInput,
                                  const ocs2::vector_t& measuredRbdState) const;

  ControllerSettings settings_;
  std::unique_ptr<G1Interface> interface_;
  std::unique_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt_;
  std::unique_ptr<ocs2::CentroidalModelRbdConversions> rbdConversions_;
  std::unique_ptr<MujocoG1Bridge> bridge_;
  std::unique_ptr<LegPostureIk> postureIk_;  // joint posture reference for the commanded pelvis height

  std::mutex fsmMutex_;
  FSMController fsm_;
  FSMState lastFsmState_ = FSMState::PASSIVE;

  std::mutex mpcMutex_;  // serialises solves with (re)initialisation
  std::atomic<bool> mpcActive_{false};
  std::atomic<double> lastSolveDuration_{0.0};

  // standing hold pose (x, y, yaw) captured when entering STANDING
  Eigen::Vector3d standPose_ = Eigen::Vector3d::Zero();

  // integrated walking command (x, y, yaw); the command trajectory starts here
  Eigen::Vector3d walkAnchor_ = Eigen::Vector3d::Zero();
  double walkAnchorTime_ = 0.0;
  bool walkAnchorValid_ = false;

  // PD stand-up interpolation
  ocs2::vector_t standUpStart_;
  double standUpStartTime_ = 0.0;

  ocs2::vector_t defaultJoints_;
  double defaultBaseHeight_ = 0.0;  // pelvis height of the default posture (initialState)
  ocs2::vector_t pGains_, dGains_;
};

}  // namespace g1_walking
