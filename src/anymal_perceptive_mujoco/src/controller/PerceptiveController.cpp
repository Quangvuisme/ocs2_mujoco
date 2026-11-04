#include <pinocchio/fwd.hpp>  // must come before Eigen / OCS2 headers that pull in pinocchio

#include "anymal_perceptive_mujoco/controller/PerceptiveController.h"

#include <chrono>
#include <cmath>
#include <iostream>

#include <pinocchio/multibody/model.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>

#include "anymal_perceptive_mujoco/common/utils.h"
#include "anymal_perceptive_mujoco/gait/MotionPhaseDefinition.h"

namespace anymal_perceptive {

namespace {
Eigen::Matrix2d yawRotation(double yaw) {
  Eigen::Matrix2d R;
  R << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
  return R;
}
}  // namespace

ControllerSettings loadControllerSettings(const std::string& file, const std::string& prefix, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(file, pt);
  ControllerSettings s;
  const std::string p = prefix + ".";
  ocs2::loadData::loadPtreeValue(pt, s.standUpKp, p + "standUpKp", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.standUpKd, p + "standUpKd", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.standUpDuration, p + "standUpDuration", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.wbcJointKp, p + "wbcJointKp", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.wbcJointKd, p + "wbcJointKd", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.commandHorizon, p + "commandHorizon", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.maxCommandLead, p + "maxCommandLead", verbose);
  ocs2::loadData::loadPtreeValue(pt, s.maxCommandYawLead, p + "maxCommandYawLead", verbose);
  return s;
}

PerceptiveController::PerceptiveController(const Paths& paths, const mjModel* model)
    : settings_(loadControllerSettings(paths.taskFile, "controller", false)),
      terrainBuffer_(std::make_shared<TerrainMapBuffer>()) {
  interface_ = std::make_unique<PerceptiveLeggedInterface>(paths.taskFile, paths.urdfFile, paths.referenceFile, terrainBuffer_);
  referenceManager().loadGaitDefinitions(paths.gaitFile);

  const auto& info = interface_->getCentroidalModelInfo();
  rbdConversions_ = std::make_unique<ocs2::CentroidalModelRbdConversions>(interface_->getPinocchioInterface(), info);

  mpc_ = std::make_unique<ocs2::GaussNewtonDDP_MPC>(interface_->mpcSettings(), interface_->ddpSettings(),
                                                    interface_->getRollout(), interface_->getOptimalControlProblem(),
                                                    interface_->getInitializer());
  mpc_->getSolverPtr()->setReferenceManager(interface_->getReferenceManagerPtr());
  mrt_ = std::make_unique<ocs2::MPC_MRT_Interface>(*mpc_);
  mrt_->initRollout(&interface_->getRollout());

  // actuated joints in Pinocchio order (the root is a 6-DoF composite joint)
  const auto& pm = interface_->getPinocchioInterface().getModel();
  std::vector<std::string> jointNames(info.actuatedDofNum);
  for (int j = 1; j < static_cast<int>(pm.njoints); ++j) {
    if (pm.joints[j].nq() == 1) {
      jointNames.at(pm.joints[j].idx_q() - 6) = pm.names[j];
    }
  }
  bridge_ = std::make_unique<MujocoAnymalBridge>(model, jointNames);

  defaultJoints_ = ocs2::centroidal_model::getJointAngles(interface_->getInitialState(), info);
  pGains_ = ocs2::vector_t::Zero(info.generalizedCoordinatesNum);
  dGains_ = ocs2::vector_t::Zero(info.generalizedCoordinatesNum);
  pGains_.tail(info.actuatedDofNum).setConstant(settings_.wbcJointKp);
  dGains_.tail(info.actuatedDofNum).setConstant(settings_.wbcJointKd);
  standUpStart_ = defaultJoints_;

  fsm_.setGaitChangeCallback([this](GaitType gait) { referenceManager().requestGait(gaitTypeToString(gait)); });
}

void PerceptiveController::onSimulationReset() {
  stopMpc();
  bridge_->resetYaw();
  withFsm([](FSMController& fsm) {
    fsm.requestStateChange(FSMState::PASSIVE);
    fsm.update(0.0);
    return 0;
  });
  lastFsmState_ = FSMState::PASSIVE;
}

void PerceptiveController::preStep(const mjModel* m, mjData* d) {
  const double t = d->time;
  FSMState state;
  WalkingTarget walk;
  StandingTarget stand;
  {
    std::lock_guard<std::mutex> lock(fsmMutex_);
    state = fsm_.getCurrentState();
    walk = fsm_.getWalkingTarget();
    stand = fsm_.getTarget();
  }

  const ocs2::vector_t rbd = bridge_->rbdState(d);  // every step: keeps the yaw unwrapping continuous
  const size_t nj = bridge_->numJoints();
  const ocs2::vector_t q = rbd.segment(6, nj);
  const ocs2::vector_t dq = rbd.tail(nj);
  ocs2::vector_t tau = ocs2::vector_t::Zero(nj);

  const auto isMpcState = [](FSMState s) { return s == FSMState::STANDING || s == FSMState::WALKING; };

  if (state != lastFsmState_) {
    if (state == FSMState::PD_CONTROL || state == FSMState::RECOVERY) {
      standUpStart_ = q;
      standUpStartTime_ = t;
    }
    if (state == FSMState::STANDING) {
      standPose_ << rbd(3), rbd(4), rbd(0);
    }
    if (!isMpcState(state) && isMpcState(lastFsmState_)) {
      stopMpc();
    }
    lastFsmState_ = state;
  }

  if (isMpcState(state)) {
    const ocs2::vector_t x = rbdConversions_->computeCentroidalStateFromRbdModel(rbd);
    if (!mpcActive_) {
      startMpc(t, x, state);
    }
    if (state == FSMState::WALKING) {
      updateWalkAnchor(t, x, walk);
    } else {
      walkAnchorValid_ = false;
    }
    referenceManager().setCommandTrajectories(buildCommand(t, x, state, walk, stand));

    const auto& info = interface_->getCentroidalModelInfo();
    ocs2::SystemObservation observation;
    observation.time = t;
    observation.state = x;
    observation.input = weightCompensatingInput(info, {true, true, true, true});
    observation.mode = ModeNumber::STANCE;
    mrt_->setCurrentObservation(observation);

    if (mrt_->initialPolicyReceived()) {
      mrt_->updatePolicy();
      ocs2::vector_t desiredState, desiredInput;
      size_t mode;
      mrt_->evaluatePolicy(t, x, desiredState, desiredInput, mode);
      tau = wholeBodyTorques(desiredState, desiredInput, rbd).tail(nj);
    } else {  // hold the posture until the first policy arrives
      tau = settings_.standUpKp * (defaultJoints_ - q) - settings_.standUpKd * dq;
    }
  } else if (state == FSMState::PD_CONTROL || state == FSMState::RECOVERY) {
    const double alpha = std::clamp((t - standUpStartTime_) / settings_.standUpDuration, 0.0, 1.0);
    const double s = 0.5 * (1.0 - std::cos(M_PI * alpha));
    const ocs2::vector_t qTarget = standUpStart_ + s * (defaultJoints_ - standUpStart_);
    tau = settings_.standUpKp * (qTarget - q) - settings_.standUpKd * dq;
  }

  bridge_->setJointTorques(m, d, tau);
}

void PerceptiveController::startMpc(double time, const ocs2::vector_t& state, FSMState fsmState) {
  std::lock_guard<std::mutex> lock(mpcMutex_);
  WalkingTarget walk;
  StandingTarget stand;
  {
    std::lock_guard<std::mutex> fsmLock(fsmMutex_);
    walk = fsm_.getWalkingTarget();
    stand = fsm_.getTarget();
  }
  if (fsmState == FSMState::STANDING) {
    standPose_ << state(6), state(7), state(9);
  }
  walkAnchorValid_ = false;
  if (fsmState == FSMState::WALKING) {
    updateWalkAnchor(time, state, walk);
  }
  const auto command = buildCommand(time, state, fsmState, walk, stand);
  referenceManager().setCommandTrajectories(command);
  mrt_->reset();
  mrt_->resetMpcNode(command);

  const auto& info = interface_->getCentroidalModelInfo();
  ocs2::SystemObservation observation;
  observation.time = time;
  observation.state = state;
  observation.input = weightCompensatingInput(info, {true, true, true, true});
  observation.mode = ModeNumber::STANCE;
  mrt_->setCurrentObservation(observation);
  mpcActive_ = true;
  std::cout << "[Controller] MPC started at t=" << time << " (" << fsmStateToString(fsmState) << ")\n";
}

void PerceptiveController::stopMpc() {
  std::lock_guard<std::mutex> lock(mpcMutex_);
  if (mpcActive_) {
    std::cout << "[Controller] MPC stopped\n";
  }
  mpcActive_ = false;
}

bool PerceptiveController::advanceMpc() {
  if (!mpcActive_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mpcMutex_);
  if (!mpcActive_) {
    return false;
  }
  const auto start = std::chrono::steady_clock::now();
  try {
    mrt_->advanceMpc();
  } catch (const std::exception& e) {
    // A failed solve leaves the solver's internal trajectories inconsistent: restart it cold from the
    // next observation. The MRT keeps executing the last valid policy meanwhile.
    std::string what = e.what();
    what = what.substr(0, what.find('\n'));
    std::cerr << "[Controller] MPC iteration failed (" << what << "), restarting the solver\n";
    mrt_->resetMpcNode(referenceManager().getTargetTrajectories());
    return false;
  }
  lastSolveDuration_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  return true;
}

void PerceptiveController::updateWalkAnchor(double time, const ocs2::vector_t& state, const WalkingTarget& walk) {
  const Eigen::Vector3d measured(state(6), state(7), state(9));
  if (!walkAnchorValid_ || settings_.maxCommandLead <= 0.0) {
    walkAnchor_ = measured;
    walkAnchorTime_ = time;
    walkAnchorValid_ = true;
    return;
  }
  const double dt = std::clamp(time - walkAnchorTime_, 0.0, 0.1);
  walkAnchorTime_ = time;
  walkAnchor_.z() += walk.yaw_rate * dt;
  walkAnchor_.head<2>() += yawRotation(walkAnchor_.z()) * Eigen::Vector2d(walk.vx, walk.vy) * dt;
  // position feedback: do not let the command run away from (or fall behind) the robot
  const Eigen::Vector2d lead = walkAnchor_.head<2>() - measured.head<2>();
  if (lead.norm() > settings_.maxCommandLead) {
    walkAnchor_.head<2>() = measured.head<2>() + settings_.maxCommandLead * lead.normalized();
  }
  walkAnchor_.z() = measured.z() + std::clamp(walkAnchor_.z() - measured.z(), -settings_.maxCommandYawLead,
                                              settings_.maxCommandYawLead);
}

ocs2::TargetTrajectories PerceptiveController::buildCommand(double time, const ocs2::vector_t& state, FSMState fsmState,
                                                            const WalkingTarget& walk, const StandingTarget& stand) const {
  const auto& info = interface_->getCentroidalModelInfo();
  ocs2::vector_t nominal = interface_->getInitialState();
  nominal.head(6).setZero();
  ocs2::centroidal_model::getJointAngles(nominal, info) = defaultJoints_;
  const ocs2::vector_t zeroInput = ocs2::vector_t::Zero(info.inputDim);

  if (fsmState != FSMState::WALKING) {
    // hold the pose captured when entering STANDING; StandingTarget x/y/yaw are offsets, z the height above ground
    ocs2::vector_t x = nominal;
    const Eigen::Vector2d offset = yawRotation(standPose_.z()) * Eigen::Vector2d(stand.x, stand.y);
    x(6) = standPose_.x() + offset.x();
    x(7) = standPose_.y() + offset.y();
    x(8) = stand.z;
    x(9) = standPose_.z() + stand.yaw;
    return ocs2::TargetTrajectories({time}, {x}, {zeroInput});
  }

  // walking: integrate the velocity command (heading frame) from the walking anchor
  constexpr int kNumNodes = 6;
  const double dt = settings_.commandHorizon / (kNumNodes - 1);
  const Eigen::Vector2d vLocal(walk.vx, walk.vy);
  Eigen::Vector2d position = walkAnchorValid_ ? Eigen::Vector2d(walkAnchor_.head<2>()) : Eigen::Vector2d(state(6), state(7));
  double yaw = walkAnchorValid_ ? walkAnchor_.z() : state(9);
  ocs2::TargetTrajectories command;
  for (int k = 0; k < kNumNodes; ++k) {
    if (k > 0) {
      position += yawRotation(yaw + 0.5 * walk.yaw_rate * dt) * vLocal * dt;
      yaw += walk.yaw_rate * dt;
    }
    ocs2::vector_t x = nominal;
    const Eigen::Vector2d vWorld = yawRotation(yaw) * vLocal;
    x(0) = vWorld.x();
    x(1) = vWorld.y();
    x(6) = position.x();
    x(7) = position.y();
    x(8) = walk.z;
    x(9) = yaw;
    command.timeTrajectory.push_back(time + k * dt);
    command.stateTrajectory.push_back(std::move(x));
    command.inputTrajectory.push_back(zeroInput);
  }
  return command;
}

ocs2::vector_t PerceptiveController::wholeBodyTorques(const ocs2::vector_t& desiredState, const ocs2::vector_t& desiredInput,
                                                      const ocs2::vector_t& measuredRbdState) const {
  const auto& info = interface_->getCentroidalModelInfo();
  const ocs2::vector_t jointAccelerations = ocs2::vector_t::Zero(info.actuatedDofNum);
  return rbdConversions_->computeRbdTorqueFromCentroidalModelPD(desiredState, desiredInput, jointAccelerations,
                                                                measuredRbdState, pGains_, dGains_);
}

}  // namespace anymal_perceptive
