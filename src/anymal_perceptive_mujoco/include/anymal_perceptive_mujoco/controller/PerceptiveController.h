#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <mujoco/mujoco.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include "anymal_perceptive_mujoco/FSMController.h"
#include "anymal_perceptive_mujoco/PerceptiveLeggedInterface.h"
#include "anymal_perceptive_mujoco/controller/MujocoAnymalBridge.h"

namespace anymal_perceptive {

struct ControllerSettings {
  double standUpKp = 150.0;         // [Nm/rad] PD stand-up
  double standUpKd = 5.0;           // [Nms/rad]
  double standUpDuration = 1.5;     // [s]
  double wbcJointKp = 500.0;        // acceleration-level joint PD of the RNEA whole-body control
  double wbcJointKd = 50.0;
  double commandHorizon = 1.0;      // [s] length of the velocity command trajectory
};

ControllerSettings loadControllerSettings(const std::string& file, const std::string& prefix, bool verbose);

/**
 * Glue between MuJoCo and the perceptive MPC:
 *  - FSM (PASSIVE / PD_CONTROL / STANDING / WALKING) driven by keyboard, gamepad or a script,
 *  - measured state from the simulator, command trajectories from the FSM targets,
 *  - MPC policy evaluation + RNEA whole-body torques every physics step.
 *
 * Threads: preStep() runs in the physics thread (simulation locked). advanceMpc() runs either in
 * a separate real-time MPC thread or, in lock-step mode, in the physics thread itself. The FSM can
 * be driven from any thread through withFsm().
 */
class PerceptiveController {
 public:
  struct Paths {
    std::string taskFile;
    std::string urdfFile;
    std::string referenceFile;
    std::string gaitFile;
  };

  PerceptiveController(const Paths& paths, const mjModel* model);

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
  std::shared_ptr<TerrainMapBuffer> terrainBuffer() const { return terrainBuffer_; }
  PerceptiveLeggedInterface& interface() { return *interface_; }
  PerceptiveReferenceManager& referenceManager() { return *interface_->getPerceptiveReferenceManagerPtr(); }
  const MujocoAnymalBridge& bridge() const { return *bridge_; }
  const ControllerSettings& settings() const { return settings_; }
  const ocs2::vector_t& defaultJointPositions() const { return defaultJoints_; }

  /** Timing of the last MPC solve [s]. */
  double lastSolveDuration() const { return lastSolveDuration_.load(); }

 private:
  void startMpc(double time, const ocs2::vector_t& state, FSMState fsmState);
  void stopMpc();
  ocs2::TargetTrajectories buildCommand(double time, const ocs2::vector_t& state, FSMState fsmState,
                                        const WalkingTarget& walk, const StandingTarget& stand) const;
  ocs2::vector_t wholeBodyTorques(const ocs2::vector_t& desiredState, const ocs2::vector_t& desiredInput,
                                  const ocs2::vector_t& measuredRbdState) const;

  ControllerSettings settings_;
  std::shared_ptr<TerrainMapBuffer> terrainBuffer_;
  std::unique_ptr<PerceptiveLeggedInterface> interface_;
  std::unique_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt_;
  std::unique_ptr<ocs2::CentroidalModelRbdConversions> rbdConversions_;
  std::unique_ptr<MujocoAnymalBridge> bridge_;

  std::mutex fsmMutex_;
  FSMController fsm_;
  FSMState lastFsmState_ = FSMState::PASSIVE;

  std::mutex mpcMutex_;  // serialises solves with (re)initialisation
  std::atomic<bool> mpcActive_{false};
  std::atomic<double> lastSolveDuration_{0.0};

  // standing hold pose (x, y, yaw) captured when entering STANDING
  Eigen::Vector3d standPose_ = Eigen::Vector3d::Zero();

  // PD stand-up interpolation
  ocs2::vector_t standUpStart_;
  double standUpStartTime_ = 0.0;

  ocs2::vector_t defaultJoints_;
  ocs2::vector_t pGains_, dGains_;
};

}  // namespace anymal_perceptive
