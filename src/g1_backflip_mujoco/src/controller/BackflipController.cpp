#include <pinocchio/fwd.hpp>  // must come before Eigen / OCS2 headers that pull in pinocchio

#include "g1_backflip_mujoco/controller/BackflipController.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LinearInterpolation.h>

#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h>

#include "g1_backflip_mujoco/common/LoadVector.h"

namespace g1_backflip {

using namespace ocs2;

ControllerSettings loadControllerSettings(const std::string& file, size_t numJoints) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(file, pt);
  ControllerSettings s;
  s.holdKp = pt.get<double>("controller.holdKp");
  s.holdKd = pt.get<double>("controller.holdKd");
  s.jointKp = loadVectorString(pt, "controller.jointKp", numJoints);
  s.jointKd = loadVectorString(pt, "controller.jointKd", numJoints);
  s.wristKp = pt.get<double>("controller.wristKp");
  s.wristKd = pt.get<double>("controller.wristKd");
  s.touchdownWindow = pt.get<double>("controller.touchdownWindow");
  return s;
}

BackflipController::BackflipController(const std::string& taskFile, const std::string& urdfFile, const mjModel* model, BackflipPlan plan,
                                       double mpcStartTime, double flipStartTime)
    : plan_(std::move(plan)), mpcStartTime_(mpcStartTime), flipStartTime_(flipStartTime) {
  interface_ = std::make_unique<G1WbBackflipInterface>(taskFile, urdfFile, "online");
  const auto& ms = interface_->modelSettings();
  settings_ = loadControllerSettings(taskFile, ms.mpc_joint_dim);

  mpc_ = std::make_unique<SqpMpc>(interface_->mpcSettings(), interface_->sqpSettings(), interface_->getOptimalControlProblem(),
                                  interface_->getInitializer());
  mpc_->getSolverPtr()->setReferenceManager(interface_->getReferenceManagerPtr());
  mrt_ = std::make_unique<MPC_MRT_Interface>(*mpc_);

  std::vector<std::string> extraJoints;
  for (const auto& name : ms.fullJointNames) {
    if (std::find(ms.mpcModelJointNames.begin(), ms.mpcModelJointNames.end(), name) == ms.mpcModelJointNames.end()) {
      extraJoints.push_back(name);
    }
  }
  bridge_ = std::make_unique<MujocoG1Bridge>(model, ms.mpcModelJointNames, extraJoints);
  footBodies_ = {mj_name2id(model, mjOBJ_BODY, "left_ankle_roll_link"), mj_name2id(model, mjOBJ_BODY, "right_ankle_roll_link")};

  holdJoints_ = plan_.solution.stateTrajectory_.front().segment(6, ms.mpc_joint_dim);
  liftoffTime_ = flipStartTime_ + plan_.liftoffTime;
  touchdownTime_ = flipStartTime_ + plan_.touchdownTime;
}

void BackflipController::setInitialPose(const mjModel* m, mjData* d) {
  mj_resetData(m, d);
  bridge_->setPose(d, plan_.solution.stateTrajectory_.front());
  mj_forward(m, d);
}

void BackflipController::reset() {
  std::lock_guard<std::mutex> lock(mpcMutex_);
  mpcActive_ = false;
  liftoffTime_ = flipStartTime_ + plan_.liftoffTime;
  touchdownTime_ = flipStartTime_ + plan_.touchdownTime;
  liftoffDetected_ = false;
  touchdownDetected_ = false;
  feetOffSince_ = -1.0;
  bridge_->resetUnwrapping();
}

double BackflipController::plannedPitch(double t) const {
  const auto& sol = plan_.solution;
  return LinearInterpolation::interpolate(t - flipStartTime_, sol.timeTrajectory_, sol.stateTrajectory_)(5);
}

int BackflipController::feetInContact(const mjModel* m, const mjData* d) const {
  bool foot[2] = {false, false};
  for (int c = 0; c < d->ncon; ++c) {
    const mjContact& con = d->contact[c];
    for (int k = 0; k < 2; ++k) {
      const int body = m->geom_bodyid[con.geom[k]];
      if (m->geom_bodyid[con.geom[1 - k]] != 0) {
        continue;
      }
      for (int f = 0; f < 2; ++f) {
        foot[f] = foot[f] || body == footBodies_[f];
      }
    }
  }
  return static_cast<int>(foot[0]) + static_cast<int>(foot[1]);
}

TargetTrajectories BackflipController::buildTargets(double touchdownShift) const {
  const auto& sol = plan_.solution;
  const double td = plan_.touchdownTime;
  constexpr double kDt = 0.01, kEps = 1e-4;
  TargetTrajectories targets;
  const auto push = [&](double time, double tau) {
    targets.timeTrajectory.push_back(time);
    targets.stateTrajectory.push_back(LinearInterpolation::interpolate(tau, sol.timeTrajectory_, sol.stateTrajectory_));
    targets.inputTrajectory.push_back(LinearInterpolation::interpolate(tau, sol.timeTrajectory_, sol.inputTrajectory_));
  };
  // up to the (shifted) touchdown
  const double flightEnd = td + std::min(touchdownShift, 0.0);
  for (double tau = 0.0; tau < flightEnd - kEps; tau += kDt) {
    push(flipStartTime_ + tau, tau);
  }
  push(flipStartTime_ + flightEnd - kEps, flightEnd - kEps);
  // landing, shifted with the touchdown
  for (double tau = td + kEps; tau < plan_.endTime; tau += kDt) {
    push(flipStartTime_ + tau + touchdownShift, tau);
  }
  push(flipStartTime_ + plan_.endTime + touchdownShift, plan_.endTime);
  return targets;
}

void BackflipController::applySchedule() {
  auto referenceManager = interface_->getReferenceManagerPtr();
  referenceManager->setBackflipSchedule(backflipModeSchedule(liftoffTime_, touchdownTime_));
  referenceManager->setTargetTrajectories(buildTargets(touchdownTime_ - flipStartTime_ - plan_.touchdownTime));
}

void BackflipController::startMpc(double time, const vector_t& state) {
  std::lock_guard<std::mutex> lock(mpcMutex_);
  applySchedule();
  mrt_->resetMpcNode(buildTargets(touchdownTime_ - flipStartTime_ - plan_.touchdownTime));
  SystemObservation observation;
  observation.time = time;
  observation.state = state;
  observation.input = vector_t::Zero(interface_->getMpcRobotModel().getInputDim());
  observation.mode = humanoid::ModeNumber::STANCE;
  mrt_->setCurrentObservation(observation);
  mpcActive_ = true;
  std::cout << "[Controller] MPC started at t=" << time << " s, backflip at t=" << flipStartTime_ << " s\n";
}

void BackflipController::updateLiftoff(double t, int feet) {
  if (liftoffDetected_) {
    return;
  }
  const double planned = flipStartTime_ + plan_.liftoffTime;
  if (t > planned) {
    liftoffDetected_ = true;  // the planned lift-off came first: keep the schedule
    return;
  }
  if (t < planned - settings_.touchdownWindow) {
    return;
  }
  if (feet > 0) {
    feetOffSince_ = -1.0;
    return;
  }
  if (feetOffSince_ < 0.0) {
    feetOffSince_ = t;
  }
  if (t - feetOffSince_ >= 0.005) {
    liftoffDetected_ = true;
    liftoffTime_ = t;
    std::cout << "[Controller] lift-off at t=" << t << " s (planned " << planned << " s)\n";
    applySchedule();
  }
}

void BackflipController::updateTouchdown(double t, int feet) {
  if (touchdownDetected_) {
    return;
  }
  const double planned = flipStartTime_ + plan_.touchdownTime;
  if (t > planned + settings_.touchdownWindow) {
    touchdownDetected_ = true;  // give up adapting: keep the schedule
    return;
  }
  if (t < planned - settings_.touchdownWindow) {
    return;
  }
  double scheduled = touchdownTime_;
  if (feet >= 1) {
    touchdownDetected_ = true;
    scheduled = t;
    std::cout << "[Controller] touchdown at t=" << t << " s (planned " << planned << " s)\n";
  } else if (t > touchdownTime_ - 0.005) {
    scheduled = t + 0.01;  // still in the air: postpone the touchdown
  }
  if (std::abs(scheduled - touchdownTime_) > 1e-4) {
    touchdownTime_ = scheduled;
    applySchedule();
  }
}

void BackflipController::preStep(const mjModel* m, mjData* d) {
  const double t = d->time;
  auto& model = interface_->getMpcRobotModel();
  const size_t nj = model.getJointDim();
  const vector_t x = bridge_->mpcState(d);  // every step: keeps the Euler unwrapping continuous
  const vector_t q = x.segment(6, nj);
  const vector_t dq = x.tail(nj);
  vector_t tau;

  if (t < mpcStartTime_) {
    tau = settings_.holdKp * (holdJoints_ - q) - settings_.holdKd * dq;
  } else {
    if (!mpcActive_) {
      startMpc(t, x);
    }
    const int feet = feetInContact(m, d);
    if (t > flipStartTime_) {
      updateLiftoff(t, feet);
    }
    if (t > liftoffTime_) {
      updateTouchdown(t, feet);
    }
    SystemObservation observation;
    observation.time = t;
    observation.state = x;
    observation.mode = backflipModeSchedule(liftoffTime_, touchdownTime_).modeAtTime(t);
    observation.input = vector_t::Zero(model.getInputDim());
    mrt_->setCurrentObservation(observation);

    if (mrt_->initialPolicyReceived()) {
      mrt_->updatePolicy();
      vector_t xd, ud;
      size_t mode;
      mrt_->evaluatePolicy(t, x, xd, ud, mode);
      // the SQP (multiple shooting) holds every input constant over its interval: use that input, not the linear
      // interpolation of evaluatePolicy (which lags rising push forces by half an interval)
      {
        const auto& policy = mrt_->getPolicy();
        const auto& times = policy.timeTrajectory_;
        const auto it = std::upper_bound(times.begin(), times.end(), t);
        if (it != times.begin() && it != times.end()) {
          ud = policy.inputTrajectory_[std::distance(times.begin(), it) - 1];
        }
      }
      // a foot can only push: no pulling normal force, and nothing at all while the foot is in the air in the schedule
      {
        const auto flags = humanoid::modeNumber2StanceLeg(observation.mode);
        for (size_t i = 0; i < N_CONTACTS; ++i) {
          if (!flags[i] || ud(6 * i + 2) < 0.0) {
            ud.segment<6>(6 * i).setZero();
          }
        }
      }
      desiredState_ = xd;
      desiredInput_ = ud;
      // inverse dynamics at the measured state with the MPC's wrenches and joint accelerations: the contact Jacobians of
      // the actual configuration map the planned foot wrenches (with the desired state, a foot starts to lift during
      // the explosive push and the momentum falls short of the plan)
      tau = humanoid::computeJointTorques<scalar_t>(x, ud, interface_->getPinocchioInterface(), model);
      tau += settings_.jointKp.cwiseProduct(model.getJointAngles(xd) - q) + settings_.jointKd.cwiseProduct(model.getJointVelocities(xd, ud) - dq);
    } else {
      tau = settings_.holdKp * (holdJoints_ - q) - settings_.holdKd * dq;
    }
  }
  measuredState_ = x;
  appliedTorques_ = tau;
  const vector_t tauExtra = -settings_.wristKp * bridge_->extraJointPositions(d) - settings_.wristKd * bridge_->extraJointVelocities(d);
  bridge_->setTorques(m, d, tau, tauExtra);
}

bool BackflipController::advanceMpc() {
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
    std::string what = e.what();
    what = what.substr(0, what.find('\n'));
    std::cerr << "[Controller] MPC iteration failed (" << what << ")\n";
    return false;
  }
  lastSolveDuration_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  return true;
}

}  // namespace g1_backflip
