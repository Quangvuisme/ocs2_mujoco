/**
 * @file main_roadster_mujoco.cpp
 * @brief Roadster kickflip with cold-gas (compressed air) jets, OCS2 + MuJoCo
 *
 * A 2 t sports car drives at 18 m/s towards a row of parked vehicles, lifts off on 12 on/off air jets,
 * does a full barrel roll over the row and lands on its wheels.
 *
 * 1. Plan:  the whole flight is optimized offline with OCS2 SLQ on a planar roll model
 *           (jet limits and clearance over the row as soft constraints).
 * 2. Fly:   in MuJoCo, OCS2 NMPC (GaussNewtonDDP_MPC + MPC_MRT_Interface) tracks the plan at 50 Hz and
 *           lands the car; its feedback policy runs at 100 Hz. A pitch/yaw hold, a bounded least-squares
 *           jet allocation and sigma-delta on/off valves turn the NMPC output into the 12 jet forces.
 * 3. Show:  live in a GLFW window, or offscreen into a video/GIF with --record (ffmpeg).
 *
 * Usage:
 *   roadster_stunt_mujoco                       live window (SPACE pause, R restart, L loop, F follow, ESC exit)
 *   roadster_stunt_mujoco --headless            simulate once and print the touchdown report
 *   roadster_stunt_mujoco --record out.gif      render offscreen to out.gif / out.mp4 (needs ffmpeg)
 *       [--width 960 --height 540 --scale 640 --fps 25 --slowmo 0.5]   (render size, output width, rate, slow motion of the roll)
 *   --log flight.csv                            also write the flight (state, plan, NMPC input) to a CSV file
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ddp/SLQ.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <roadster_stunt/ClearanceConstraint.h>
#include <roadster_stunt/JetController.h>
#include <roadster_stunt/RoadsterInterface.h>
#include <roadster_stunt/StuntReference.h>
#include <roadster_stunt/package_path.h>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

using namespace ocs2;
using namespace roadster_stunt;

// ============================================================================
// Settings
// ============================================================================
struct SimulationSettings {
  scalar_t settleTime = 0.5;
  scalar_t rolloutTime = 2.5;
  scalar_t brakeDamping = 1500.0;
  bool gustEnable = false;
  scalar_t gustStart = 1.0;
  scalar_t gustDuration = 0.15;
  scalar_t gustForceY = 4000.0;
  scalar_t gustTorqueX = 6000.0;
};

SimulationSettings loadSimulationSettings(const std::string& taskFile) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  SimulationSettings s;
  loadData::loadPtreeValue(pt, s.settleTime, "simulation.settleTime", false);
  loadData::loadPtreeValue(pt, s.rolloutTime, "simulation.rolloutTime", false);
  loadData::loadPtreeValue(pt, s.brakeDamping, "simulation.brakeDamping", false);
  loadData::loadPtreeValue(pt, s.gustEnable, "simulation.gust.enable", false);
  loadData::loadPtreeValue(pt, s.gustStart, "simulation.gust.start", false);
  loadData::loadPtreeValue(pt, s.gustDuration, "simulation.gust.duration", false);
  loadData::loadPtreeValue(pt, s.gustForceY, "simulation.gust.forceY", false);
  loadData::loadPtreeValue(pt, s.gustTorqueX, "simulation.gust.torqueX", false);
  return s;
}

JetController::Settings loadJetSettings(const std::string& taskFile) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  JetController::Settings s;
  loadData::loadPtreeValue(pt, s.pitchNaturalFrequency, "lowLevel.pitchNaturalFrequency", false);
  loadData::loadPtreeValue(pt, s.yawNaturalFrequency, "lowLevel.yawNaturalFrequency", false);
  loadData::loadPtreeValue(pt, s.onOffValves, "lowLevel.onOffValves", false);
  return s;
}

// ============================================================================
// Offline plan (OCS2 SLQ)
// ============================================================================
PrimalSolution computeStuntPlan(const std::string& taskFile) {
  RoadsterInterface planInterface(taskFile, "plan", false);
  const auto& roadster = planInterface.roadsterParameters();
  const auto& stunt = planInterface.stuntParameters();

  SLQ solver(planInterface.ddpSettings(), planInterface.getRollout(), planInterface.getOptimalControlProblem(),
             planInterface.getInitializer());
  solver.getReferenceManager().setTargetTrajectories(designStuntGuide(roadster, stunt, 0.01));

  const auto start = std::chrono::steady_clock::now();
  solver.run(0.0, planInterface.getInitialState(), stunt.flightTime_);
  const scalar_t ms = std::chrono::duration<scalar_t, std::milli>(std::chrono::steady_clock::now() - start).count();
  PrimalSolution plan = solver.primalSolution(stunt.flightTime_);

  // report
  const ClearanceConstraint clearance(roadster, stunt);
  scalar_t apex = 0.0, minMargin = 1e9, impulse = 0.0;
  for (size_t k = 0; k < plan.timeTrajectory_.size(); ++k) {
    const scalar_t t = plan.timeTrajectory_[k];
    apex = std::max(apex, plan.stateTrajectory_[k](Z));
    if (clearance.minHeight(t) > 1.0) {
      minMargin = std::min(minMargin, plan.stateTrajectory_[k](Z) - clearance.minHeight(t));
    }
    if (k + 1 < plan.timeTrajectory_.size()) {
      impulse += 1e3 * plan.inputTrajectory_[k].sum() * (plan.timeTrajectory_[k + 1] - t);
    }
  }
  const vector_t& xT = plan.stateTrajectory_.back();
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "[Plan] OCS2 SLQ solved in " << ms << " ms: flight " << stunt.flightTime_ << " s, apex " << apex
            << " m, min clearance margin " << minMargin << " m, air " << impulse / 1e3 << " kN s\n";
  std::cout << "[Plan] end (flare): y " << xT(Y) << " m, z " << xT(Z) << " m, vz " << xT(VZ) << " m/s, roll "
            << xT(ROLL) * 180.0 / M_PI << " deg\n";
  return plan;
}

// ============================================================================
// Closed-loop simulation (MuJoCo + OCS2 NMPC)
// ============================================================================
enum class Phase { SETTLE, RUNUP, FLIGHT, ROLLOUT, DONE };

const char* phaseName(Phase p) {
  switch (p) {
    case Phase::SETTLE:
      return "SETTLE";
    case Phase::RUNUP:
      return "RUN-UP";
    case Phase::FLIGHT:
      return "FLIGHT (OCS2 NMPC)";
    case Phase::ROLLOUT:
      return "LANDED";
    default:
      return "DONE";
  }
}

struct TouchdownReport {
  bool landed = false;
  scalar_t time = 0.0;  // after launch
  scalar_t x = 0.0;
  scalar_t y = 0.0;
  scalar_t sinkRate = 0.0;
  scalar_t rollErrorDeg = 0.0;
  scalar_t pitchDeg = 0.0;
  scalar_t apex = 0.0;
  bool hitObstacle = false;
  scalar_t airUsed = 0.0;
  bool finalUpright = false;
};

class StuntSimulation {
 public:
  static constexpr scalar_t CONTROL_DT = 0.01;  // jet control / policy evaluation [s]
  static constexpr int MPC_EVERY = 2;           // control ticks per NMPC solve (50 Hz)

  StuntSimulation(const std::string& modelPath, const std::string& taskFile, const PrimalSolution& plan)
      : nmpcInterface_(taskFile, "nmpc", false), sim_(loadSimulationSettings(taskFile)), tank_(0.0) {
    char error[1000] = "Could not load model";
    m_ = mj_loadXML(modelPath.c_str(), nullptr, error, sizeof(error));
    if (!m_) {
      throw std::runtime_error(std::string("[StuntSimulation] Load model error: ") + error);
    }
    d_ = mj_makeData(m_);
    stepsPerControl_ = static_cast<int>(std::lround(CONTROL_DT / m_->opt.timestep));

    roadster_ = nmpcInterface_.roadsterParameters();
    stunt_ = nmpcInterface_.stuntParameters();
    tank_ = AirTank(roadster_.impulseBudget_);
    planEnd_ = plan.timeTrajectory_.back();

    // ids
    car_ = mj_name2id(m_, mjOBJ_BODY, "car");
    floor_ = mj_name2id(m_, mjOBJ_GEOM, "floor");
    for (const char* n : {"fl", "fr", "rl", "rr"}) {
      tires_.push_back(mj_name2id(m_, mjOBJ_GEOM, (std::string("tire_") + n).c_str()));
      spinDofs_.push_back(m_->jnt_dofadr[mj_name2id(m_, mjOBJ_JOINT, (std::string("spin_") + n).c_str())]);
    }
    bodyGeoms_ = {mj_name2id(m_, mjOBJ_GEOM, "chassis"), mj_name2id(m_, mjOBJ_GEOM, "cabin_col")};
    for (int i = 0; i < 5; ++i) {
      obstacles_.push_back(mj_name2id(m_, mjOBJ_GEOM, ("obstacle_" + std::to_string(i)).c_str()));
    }
    for (int i = 0; i < m_->nu; ++i) {
      const int site = m_->actuator_trnid[2 * i];
      plumes_.push_back(mj_name2id(m_, mjOBJ_GEOM, (std::string("plume_") + mj_id2name(m_, mjOBJ_SITE, site)).c_str()));
    }
    spinDamping_ = m_->dof_damping[spinDofs_[0]];

    // jets: allocation matrix from the model, about the CoM of the whole car
    mj_forward(m_, d_);
    vector_t fMax(m_->nu);
    for (int i = 0; i < m_->nu; ++i) {
      fMax(i) = m_->actuator_ctrlrange[2 * i + 1];
    }
    jets_ = std::make_unique<JetController>(allocationMatrix(), fMax, roadster_, loadJetSettings(taskFile));

    // NMPC: track the plan, warm-started by the plan's inputs
    targets_ = planToTargets(plan, roadster_, stunt_, 0.01, 2.0);
    nmpcInterface_.setInitializer(std::make_unique<TargetInputInitializer>(targets_));
    mpc_ = std::make_unique<GaussNewtonDDP_MPC>(nmpcInterface_.mpcSettings(), nmpcInterface_.ddpSettings(),
                                                nmpcInterface_.getRollout(), nmpcInterface_.getOptimalControlProblem(),
                                                nmpcInterface_.getInitializer());
    mpc_->getSolverPtr()->setReferenceManager(nmpcInterface_.getReferenceManagerPtr());
    mrt_ = std::make_unique<MPC_MRT_Interface>(*mpc_);

    // plan path in the world (x = launch speed * t) for drawing
    for (size_t k = 0; k < plan.timeTrajectory_.size(); k += 4) {
      const auto& x = plan.stateTrajectory_[k];
      planPath_.push_back({stunt_.launchSpeed_ * plan.timeTrajectory_[k], x(Y), x(Z)});
    }

    reset();
  }

  /** Writes the flight (state, plan reference, NMPC input) to a CSV file. */
  void openLog(const std::string& path) {
    log_.open(path);
    log_ << "t,x,y,z,vy,vz,roll,roll_rate,y_ref,z_ref,vy_ref,vz_ref,roll_ref,roll_rate_ref,"
            "lift_L,lift_R,roof_L,roof_R,mpc_ms,air_left\n";
  }

  ~StuntSimulation() {
    mj_deleteData(d_);
    mj_deleteModel(m_);
  }

  void reset() {
    mj_resetData(m_, d_);
    for (int dof : spinDofs_) {
      m_->dof_damping[dof] = spinDamping_;
    }
    mj_forward(m_, d_);
    phase_ = Phase::SETTLE;
    time_ = 0.0;
    launchTime_ = rolloutStart_ = 0.0;
    flightTick_ = 0;
    lastRoll_ = 0.0;
    lastInput_ = vector_t::Zero(INPUT_DIM);
    jetForces_ = vector_t::Zero(m_->nu);
    jetImpulse_ = vector_t::Zero(m_->nu);
    jetImpulseTime_ = 0.0;
    jets_->reset();
    tank_.reset();
    mrt_->resetMpcNode(targets_);
    report_ = TouchdownReport();
    mpcSolveMs_ = 0.0;
  }

  /** Advances one control period. */
  void step() {
    if (phase_ == Phase::DONE) {
      return;
    }
    updatePhase();
    if (phase_ == Phase::FLIGHT) {
      flightControl();
    } else {
      jetForces_.setZero();
    }

    for (int k = 0; k < stepsPerControl_; ++k) {
      const vector_t applied = tank_.draw(jetForces_, m_->opt.timestep);
      for (int i = 0; i < m_->nu; ++i) {
        d_->ctrl[i] = applied(i);
      }
      jetImpulse_ += applied * m_->opt.timestep;
      jetImpulseTime_ += m_->opt.timestep;
      applyGust();
      mj_step(m_, d_);
      checkContacts();
    }
    time_ += CONTROL_DT;
  }

  /** Average jet force over the time since the last call, as a fraction of each jet's maximum. */
  vector_t consumeJetLevels() {
    vector_t level = vector_t::Zero(m_->nu);
    if (jetImpulseTime_ > 0.0) {
      for (int i = 0; i < m_->nu; ++i) {
        level(i) = jetImpulse_(i) / jetImpulseTime_ / m_->actuator_ctrlrange[2 * i + 1];
      }
    }
    jetImpulse_.setZero();
    jetImpulseTime_ = 0.0;
    return level;
  }

  /** NMPC predicted path in the world over its horizon (empty outside the flight). */
  std::vector<std::array<scalar_t, 3>> predictedPath() const {
    std::vector<std::array<scalar_t, 3>> path;
    if (phase_ != Phase::FLIGHT || !mrt_->initialPolicyReceived()) {
      return path;
    }
    const auto& policy = mrt_->getPolicy();
    const scalar_t tNow = time_ - launchTime_;
    for (size_t k = 0; k < policy.timeTrajectory_.size(); k += 3) {
      const scalar_t t = policy.timeTrajectory_[k];
      if (t < tNow) {
        continue;
      }
      const auto& x = policy.stateTrajectory_[k];
      path.push_back({d_->xpos[3 * car_] + d_->qvel[0] * (t - tNow), x(Y), x(Z)});
    }
    return path;
  }

  void printReport() const {
    std::cout << std::fixed << std::setprecision(2);
    if (!report_.landed) {
      std::cout << "[Result] the car did not land" << std::endl;
      return;
    }
    std::cout << "[Result] touchdown " << report_.time << " s after launch at x = " << report_.x << " m (y = " << report_.y
              << " m)\n";
    std::cout << "[Result] sink rate " << report_.sinkRate << " m/s, roll error " << report_.rollErrorDeg << " deg, pitch "
              << report_.pitchDeg << " deg, apex " << report_.apex << " m\n";
    std::cout << "[Result] hit an obstacle: " << (report_.hitObstacle ? "YES" : "no") << ", wheels down at the end: "
              << (report_.finalUpright ? "yes" : "NO") << ", air used " << report_.airUsed / 1e3 << " of "
              << roadster_.impulseBudget_ / 1e3 << " kN s" << std::endl;
  }

  mjModel* model() { return m_; }
  mjData* data() { return d_; }
  int carId() const { return car_; }
  Phase phase() const { return phase_; }
  scalar_t time() const { return time_; }
  scalar_t flightTime() const { return phase_ >= Phase::FLIGHT ? time_ - launchTime_ : 0.0; }
  scalar_t airFraction() const { return tank_.fraction(); }
  scalar_t rollDeg() const { return lastRoll_ * 180.0 / M_PI; }
  scalar_t mpcSolveMs() const { return mpcSolveMs_; }
  const TouchdownReport& report() const { return report_; }
  const std::vector<int>& plumes() const { return plumes_; }
  const std::vector<std::array<scalar_t, 3>>& planPath() const { return planPath_; }

 private:
  Eigen::MatrixXd allocationMatrix() const {
    Eigen::Matrix3d R;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        R(r, c) = d_->xmat[9 * car_ + 3 * r + c];
      }
    }
    const Eigen::Map<const Eigen::Vector3d> origin(d_->xpos + 3 * car_);
    const Eigen::Map<const Eigen::Vector3d> subtreeCom(d_->subtree_com + 3 * car_);
    const Eigen::Vector3d com = R.transpose() * (subtreeCom - origin);
    Eigen::MatrixXd A(6, m_->nu);
    for (int i = 0; i < m_->nu; ++i) {
      const int site = m_->actuator_trnid[2 * i];
      const Eigen::Vector3d p = Eigen::Map<const Eigen::Vector3d>(m_->site_pos + 3 * site) - com;
      const Eigen::Map<const Eigen::Vector3d> f(m_->actuator_gear + 6 * i);
      A.col(i) << f, p.cross(f);
    }
    return A;
  }

  void updatePhase() {
    if (phase_ == Phase::SETTLE && time_ >= sim_.settleTime) {
      // run-up: the car rolls towards the launch line at the launch speed
      phase_ = Phase::RUNUP;
      d_->qvel[0] = stunt_.launchSpeed_;
      for (int dof : spinDofs_) {
        d_->qvel[dof] = stunt_.launchSpeed_ / 0.35;
      }
    } else if (phase_ == Phase::RUNUP && d_->xpos[3 * car_] >= 0.0) {
      phase_ = Phase::FLIGHT;
      launchTime_ = time_;
      flightTick_ = 0;
      std::cout << "[Sim] launch at t = " << time_ << " s, speed " << d_->qvel[0] << " m/s" << std::endl;
    } else if (phase_ == Phase::FLIGHT) {
      const scalar_t tp = time_ - launchTime_;
      if ((tp > 0.3 && groundContact_) || tp > planEnd_ + 3.0) {
        land();
      }
    } else if (phase_ == Phase::ROLLOUT && time_ - rolloutStart_ >= sim_.rolloutTime) {
      phase_ = Phase::DONE;
      report_.airUsed = roadster_.impulseBudget_ - tank_.remaining();
      report_.finalUpright = d_->xmat[9 * car_ + 8] > 0.9;
    }
  }

  vector_t measureState() {
    vector_t x(STATE_DIM);
    const mjtNum* R = d_->xmat + 9 * car_;
    // roll of the body z axis in the y-z plane, unwrapped to stay continuous through the full turn
    scalar_t roll = std::atan2(-R[5], R[8]);
    roll += 2.0 * M_PI * std::round((lastRoll_ - roll) / (2.0 * M_PI));
    lastRoll_ = roll;
    // world angular velocity about x from the body angular velocity of the free joint
    const scalar_t rollRate = R[0] * d_->qvel[3] + R[1] * d_->qvel[4] + R[2] * d_->qvel[5];
    x << d_->xpos[3 * car_ + 1], d_->xpos[3 * car_ + 2], d_->qvel[1], d_->qvel[2], roll, rollRate;
    return x;
  }

  void flightControl() {
    const scalar_t tp = time_ - launchTime_;
    const vector_t x = measureState();
    report_.apex = std::max(report_.apex, x(Z));

    if (flightTick_++ % MPC_EVERY == 0) {
      SystemObservation observation;
      observation.time = tp;
      observation.state = x;
      observation.input = lastInput_;
      mrt_->setCurrentObservation(observation);
      const auto start = std::chrono::steady_clock::now();
      mrt_->advanceMpc();
      mpcSolveMs_ = std::chrono::duration<scalar_t, std::milli>(std::chrono::steady_clock::now() - start).count();
      mrt_->updatePolicy();
    }

    vector_t xOpt, u;
    size_t mode;
    mrt_->evaluatePolicy(tp, x, xOpt, u, mode);
    lastInput_ = u;

    Matrix3 R;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        R(r, c) = d_->xmat[9 * car_ + 3 * r + c];
      }
    }
    const Vector3 omega(d_->qvel[3], d_->qvel[4], d_->qvel[5]);
    jetForces_ = jets_->compute(u, R, omega);

    if (log_.is_open()) {
      const vector_t xRef = targets_.getDesiredState(tp);
      log_ << tp << ',' << d_->xpos[3 * car_];
      for (Eigen::Index i = 0; i < x.size(); ++i) log_ << ',' << x(i);
      for (Eigen::Index i = 0; i < xRef.size(); ++i) log_ << ',' << xRef(i);
      for (Eigen::Index i = 0; i < u.size(); ++i) log_ << ',' << u(i);
      log_ << ',' << mpcSolveMs_ << ',' << tank_.remaining() << '\n';
    }
  }

  void land() {
    phase_ = Phase::ROLLOUT;
    rolloutStart_ = time_;
    jetForces_.setZero();
    for (int dof : spinDofs_) {
      m_->dof_damping[dof] = sim_.brakeDamping;  // brake on the wheels
    }
    const mjtNum* R = d_->xmat + 9 * car_;
    report_.landed = true;
    report_.time = time_ - launchTime_;
    report_.x = d_->xpos[3 * car_];
    report_.y = d_->xpos[3 * car_ + 1];
    report_.sinkRate = std::abs(d_->qvel[2]);
    report_.rollErrorDeg = std::abs(std::remainder(std::atan2(-R[5], R[8]), 2.0 * M_PI)) * 180.0 / M_PI;
    report_.pitchDeg = std::asin(std::clamp(-R[6], -1.0, 1.0)) * 180.0 / M_PI;
    std::cout << "[Sim] touchdown at t = " << time_ << " s" << std::endl;
  }

  void checkContacts() {
    groundContact_ = false;
    for (int i = 0; i < d_->ncon; ++i) {
      const int g1 = d_->contact[i].geom1;
      const int g2 = d_->contact[i].geom2;
      const auto isCar = [&](int g) {
        return std::find(tires_.begin(), tires_.end(), g) != tires_.end() ||
               std::find(bodyGeoms_.begin(), bodyGeoms_.end(), g) != bodyGeoms_.end();
      };
      const auto isObstacle = [&](int g) { return std::find(obstacles_.begin(), obstacles_.end(), g) != obstacles_.end(); };
      if ((g1 == floor_ && isCar(g2)) || (g2 == floor_ && isCar(g1))) {
        groundContact_ = true;
      }
      if ((isObstacle(g1) && isCar(g2)) || (isObstacle(g2) && isCar(g1))) {
        report_.hitObstacle = true;
      }
    }
  }

  void applyGust() {
    mju_zero(d_->xfrc_applied + 6 * car_, 6);
    if (!sim_.gustEnable || phase_ != Phase::FLIGHT) {
      return;
    }
    const scalar_t tp = time_ - launchTime_;
    if (tp >= sim_.gustStart && tp < sim_.gustStart + sim_.gustDuration) {
      const mjtNum* R = d_->xmat + 9 * car_;
      d_->xfrc_applied[6 * car_ + 1] = sim_.gustForceY;
      for (int r = 0; r < 3; ++r) {
        d_->xfrc_applied[6 * car_ + 3 + r] = R[3 * r] * sim_.gustTorqueX;  // body x torque in the world frame
      }
    }
  }

  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  int stepsPerControl_ = 10;

  RoadsterInterface nmpcInterface_;
  RoadsterParameters roadster_;
  StuntParameters stunt_;
  SimulationSettings sim_;
  TargetTrajectories targets_;
  std::unique_ptr<GaussNewtonDDP_MPC> mpc_;
  std::unique_ptr<MPC_MRT_Interface> mrt_;
  std::unique_ptr<JetController> jets_;
  AirTank tank_;
  scalar_t planEnd_ = 0.0;

  int car_ = -1, floor_ = -1;
  std::vector<int> tires_, bodyGeoms_, obstacles_, spinDofs_, plumes_;
  scalar_t spinDamping_ = 0.0;

  Phase phase_ = Phase::SETTLE;
  scalar_t time_ = 0.0, launchTime_ = 0.0, rolloutStart_ = 0.0;
  long flightTick_ = 0;
  scalar_t lastRoll_ = 0.0;
  bool groundContact_ = false;
  vector_t lastInput_, jetForces_, jetImpulse_;
  scalar_t jetImpulseTime_ = 0.0;
  scalar_t mpcSolveMs_ = 0.0;
  TouchdownReport report_;
  std::vector<std::array<scalar_t, 3>> planPath_;
  std::ofstream log_;
};

// ============================================================================
// Rendering
// ============================================================================
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
GLFWwindow* window = nullptr;

bool paused = false;
bool loopAnimation = true;
bool followCar = true;
bool restartRequested = false;
bool buttonLeft = false, buttonMiddle = false, buttonRight = false;
double lastx = 0, lasty = 0;
mjModel* viewModel = nullptr;

void keyboard(GLFWwindow* w, int key, int scancode, int act, int mods) {
  if (act != GLFW_PRESS) {
    return;
  }
  switch (key) {
    case GLFW_KEY_ESCAPE:
      glfwSetWindowShouldClose(w, GLFW_TRUE);
      break;
    case GLFW_KEY_SPACE:
      paused = !paused;
      break;
    case GLFW_KEY_R:
      restartRequested = true;
      break;
    case GLFW_KEY_L:
      loopAnimation = !loopAnimation;
      std::cout << "Loop: " << (loopAnimation ? "ON" : "OFF") << std::endl;
      break;
    case GLFW_KEY_F:
      followCar = !followCar;
      std::cout << "Follow camera: " << (followCar ? "ON" : "OFF") << std::endl;
      break;
  }
}

void mouseButton(GLFWwindow* w, int button, int act, int mods) {
  buttonLeft = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  buttonMiddle = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
  buttonRight = glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  glfwGetCursorPos(w, &lastx, &lasty);
}

void mouseMove(GLFWwindow* w, double xpos, double ypos) {
  if (!buttonLeft && !buttonMiddle && !buttonRight) {
    return;
  }
  const double dx = xpos - lastx;
  const double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;
  int width, height;
  glfwGetWindowSize(w, &width, &height);
  const bool shift = glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
  mjtMouse action = mjMOUSE_ZOOM;
  if (buttonRight) {
    action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (buttonLeft) {
    action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  mjv_moveCamera(viewModel, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow* w, double xoffset, double yoffset) {
  mjv_moveCamera(viewModel, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void resetCamera() {
  mjv_defaultFreeCamera(viewModel, &cam);
  cam.type = mjCAMERA_FREE;
  cam.azimuth = 72.0;
  cam.elevation = -8.0;
  cam.distance = 21.0;
  cam.lookat[0] = -6.0;
  cam.lookat[1] = 0.0;
  cam.lookat[2] = 2.0;
}

/** Smoothly keeps the car and the row in view. */
void updateFollowCamera(StuntSimulation& sim, scalar_t dt) {
  if (!followCar) {
    return;
  }
  const mjtNum* p = sim.data()->xpos + 3 * sim.carId();
  const scalar_t targetX = p[0] + 4.0;
  const scalar_t targetZ = 1.5 + 0.45 * std::max(0.0, p[2] - 0.5);
  const scalar_t a = 1.0 - std::exp(-dt / 0.35);
  cam.lookat[0] += a * (targetX - cam.lookat[0]);
  cam.lookat[1] += a * (0.0 - cam.lookat[1]);
  cam.lookat[2] += a * (targetZ - cam.lookat[2]);
}

void addSegment(const std::array<scalar_t, 3>& from, const std::array<scalar_t, 3>& to, scalar_t width, const float rgba[4]) {
  if (scn.ngeom >= scn.maxgeom) {
    return;
  }
  mjvGeom* g = scn.geoms + scn.ngeom++;
  const mjtNum size[3] = {width, width, width};
  mjv_initGeom(g, mjGEOM_CAPSULE, size, nullptr, nullptr, rgba);
  mjv_connector(g, mjGEOM_CAPSULE, width, from.data(), to.data());
  g->category = mjCAT_DECOR;
}

void renderScene(StuntSimulation& sim, const mjrRect& viewport, const vector_t& jetLevels, scalar_t rate) {
  mjModel* m = sim.model();
  // cold-gas plumes: opacity follows the average jet force since the last frame
  for (size_t i = 0; i < sim.plumes().size(); ++i) {
    const scalar_t level = std::clamp(jetLevels(i), 0.0, 1.0);
    m->geom_rgba[4 * sim.plumes()[i] + 3] = level > 0.02 ? static_cast<float>(0.12 + 0.5 * level) : 0.0f;
  }

  mjv_updateScene(m, sim.data(), &opt, nullptr, &cam, mjCAT_ALL, &scn);

  // offline plan (blue) and the NMPC prediction over its horizon (orange)
  const float planRgba[4] = {0.35f, 0.65f, 1.0f, 0.55f};
  const float mpcRgba[4] = {1.0f, 0.55f, 0.1f, 0.95f};
  const auto& plan = sim.planPath();
  if (sim.phase() == Phase::RUNUP || sim.phase() == Phase::FLIGHT) {
    for (size_t k = 1; k < plan.size(); ++k) {
      addSegment(plan[k - 1], plan[k], 0.04, planRgba);
    }
  }
  const auto path = sim.predictedPath();
  for (size_t k = 1; k < path.size(); ++k) {
    addSegment(path[k - 1], path[k], 0.07, mpcRgba);
  }

  mjr_render(viewport, &scn, &con);

  char title[200];
  snprintf(title, sizeof(title), "Roadster kickflip on cold-gas jets  |  OCS2 SLQ plan + OCS2 NMPC  |  MuJoCo");
  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, title, nullptr, &con);

  char labels[300], values[300];
  snprintf(labels, sizeof(labels), "Phase\nTime after launch\nRoll\nAir left\nNMPC solve\nPlayback");
  snprintf(values, sizeof(values), "%s\n%.2f s\n%.0f deg\n%.0f %%\n%.1f ms\n%.2fx", phaseName(sim.phase()), sim.flightTime(),
           sim.rollDeg(), 100.0 * sim.airFraction(), sim.mpcSolveMs(), rate);
  mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, labels, values, &con);

  char legend[200];
  snprintf(legend, sizeof(legend), "blue: offline plan   orange: NMPC horizon");
  mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMRIGHT, viewport, legend, nullptr, &con);
}

bool initGraphics(mjModel* m, int width, int height, bool visible) {
  if (!glfwInit()) {
    std::cerr << "Could not initialize GLFW" << std::endl;
    return false;
  }
  glfwWindowHint(GLFW_VISIBLE, visible ? GLFW_TRUE : GLFW_FALSE);
  window = glfwCreateWindow(width, height, "Roadster Stunt - OCS2 NMPC + MuJoCo", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return false;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  viewModel = m;
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);
  opt.flags[mjVIS_TRANSPARENT] = 0;
  opt.geomgroup[3] = 0;  // hide collision proxies
  mjv_makeScene(m, &scn, 4000);
  mjr_makeContext(m, &con, width >= 900 ? mjFONTSCALE_150 : mjFONTSCALE_100);
  resetCamera();

  if (visible) {
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouseMove);
    glfwSetMouseButtonCallback(window, mouseButton);
    glfwSetScrollCallback(window, scroll);
  }
  return true;
}

void cleanupGraphics() {
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  if (window) {
    glfwDestroyWindow(window);
  }
  glfwTerminate();
}

/** Slow motion is applied while the car is in the air and has not finished the roll. */
scalar_t playbackRate(const StuntSimulation& sim, scalar_t slowmo) {
  return sim.phase() == Phase::FLIGHT && sim.rollDeg() < 355.0 ? slowmo : 1.0;
}

// ============================================================================
// Modes
// ============================================================================
int runHeadless(StuntSimulation& sim) {
  while (sim.phase() != Phase::DONE) {
    sim.step();
  }
  sim.printReport();
  return sim.report().landed && !sim.report().hitObstacle && sim.report().finalUpright ? 0 : 2;
}

int runRecord(StuntSimulation& sim, const std::string& output, int width, int height, int fps, scalar_t slowmo, int outWidth) {
  if (!initGraphics(sim.model(), width, height, false)) {
    return 1;
  }
  if (width > sim.model()->vis.global.offwidth || height > sim.model()->vis.global.offheight) {
    std::cerr << "[Record] size exceeds the model's offscreen buffer" << std::endl;
    return 1;
  }
  mjr_setBuffer(mjFB_OFFSCREEN, &con);

  // ffmpeg: raw RGB frames on stdin, flipped and scaled to the output width; a GIF gets its own palette
  const bool gif = output.size() > 4 && output.substr(output.size() - 4) == ".gif";
  char filter[512];
  snprintf(filter, sizeof(filter), "vflip,scale=%d:-2:flags=lanczos", outWidth > 0 ? outWidth : width);
  char cmd[1024];
  if (gif) {
    snprintf(cmd, sizeof(cmd),
             "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i - -vf "
             "\"%s,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle\" "
             "-loop 0 \"%s\"",
             width, height, fps, filter, output.c_str());
  } else {
    snprintf(cmd, sizeof(cmd), "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i - -vf %s -pix_fmt yuv420p \"%s\"",
             width, height, fps, filter, output.c_str());
  }
  FILE* pipe = popen(cmd, "w");
  if (!pipe) {
    std::cerr << "[Record] could not start ffmpeg" << std::endl;
    return 1;
  }

  const mjrRect viewport = {0, 0, width, height};
  std::vector<unsigned char> rgb(3 * width * height);
  const scalar_t frameDt = 1.0 / fps;
  int frames = 0;

  // skip most of the settling on the wheels
  while (sim.phase() == Phase::SETTLE && sim.time() < 0.3) {
    sim.step();
  }
  scalar_t simAhead = 0.0;
  scalar_t holdAfterDone = 0.6;
  while (holdAfterDone > 0.0) {
    const scalar_t rate = playbackRate(sim, slowmo);
    simAhead += frameDt * rate;
    while (simAhead >= StuntSimulation::CONTROL_DT - 1e-9 && sim.phase() != Phase::DONE) {
      sim.step();
      simAhead -= StuntSimulation::CONTROL_DT;
    }
    if (sim.phase() == Phase::DONE) {
      holdAfterDone -= frameDt;
    }
    updateFollowCamera(sim, frameDt * rate);
    renderScene(sim, viewport, sim.consumeJetLevels(), rate);
    mjr_readPixels(rgb.data(), nullptr, viewport, &con);
    fwrite(rgb.data(), 1, rgb.size(), pipe);
    ++frames;
  }
  const int status = pclose(pipe);
  cleanupGraphics();
  std::cout << "[Record] " << frames << " frames (" << frames / static_cast<scalar_t>(fps) << " s at " << fps << " fps) -> " << output
            << (status == 0 ? "" : "  (ffmpeg FAILED)") << std::endl;
  sim.printReport();
  return status == 0 ? 0 : 1;
}

int runInteractive(StuntSimulation& sim, scalar_t slowmo) {
  if (!initGraphics(sim.model(), 1400, 800, true)) {
    return 1;
  }
  std::cout << "\n========================================" << std::endl;
  std::cout << "  SPACE : pause / resume" << std::endl;
  std::cout << "  R     : restart the stunt" << std::endl;
  std::cout << "  L     : toggle loop" << std::endl;
  std::cout << "  F     : toggle follow camera" << std::endl;
  std::cout << "  Mouse : rotate / zoom / pan" << std::endl;
  std::cout << "  ESC   : exit" << std::endl;
  std::cout << "========================================" << std::endl;

  auto last = std::chrono::steady_clock::now();
  scalar_t simAhead = 0.0, doneFor = 0.0;
  bool reported = false;
  while (!glfwWindowShouldClose(window)) {
    const auto now = std::chrono::steady_clock::now();
    const scalar_t frameDt = std::min(0.05, std::chrono::duration<scalar_t>(now - last).count());
    last = now;

    if (restartRequested || (loopAnimation && doneFor > 2.0)) {
      sim.reset();
      resetCamera();
      restartRequested = false;
      reported = false;
      doneFor = simAhead = 0.0;
    }

    const scalar_t rate = playbackRate(sim, slowmo);
    if (!paused) {
      simAhead += frameDt * rate;
      while (simAhead >= StuntSimulation::CONTROL_DT && sim.phase() != Phase::DONE) {
        sim.step();
        simAhead -= StuntSimulation::CONTROL_DT;
      }
      if (sim.phase() == Phase::DONE) {
        doneFor += frameDt;
        if (!reported) {
          sim.printReport();
          reported = true;
        }
      }
      updateFollowCamera(sim, frameDt * rate);
    }

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    renderScene(sim, viewport, sim.consumeJetLevels(), paused ? 0.0 : rate);
    glfwSwapBuffers(window);
    glfwPollEvents();
  }
  cleanupGraphics();
  return 0;
}

// ============================================================================
// Main Function
// ============================================================================
int main(int argc, char** argv) {
  std::string record, logFile;
  bool headless = false;
  int width = 960, height = 540, fps = 25, outWidth = 0;
  scalar_t slowmo = 1.0;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::invalid_argument("missing value after " + arg);
      }
      return argv[++i];
    };
    if (arg == "--record") {
      record = next();
    } else if (arg == "--log") {
      logFile = next();
    } else if (arg == "--headless") {
      headless = true;
    } else if (arg == "--width") {
      width = std::stoi(next());
    } else if (arg == "--height") {
      height = std::stoi(next());
    } else if (arg == "--scale") {
      outWidth = std::stoi(next());
    } else if (arg == "--fps") {
      fps = std::stoi(next());
    } else if (arg == "--slowmo") {
      slowmo = std::stod(next());
    } else {
      std::cerr << "Usage: " << argv[0] << " [--headless] [--record out.gif|out.mp4] [--log flight.csv] [--width W] [--height H] [--scale OUT_W] [--fps F] [--slowmo S]"
                << std::endl;
      return 1;
    }
  }

  std::cout << "========================================" << std::endl;
  std::cout << "   Roadster Kickflip on Cold-Gas Jets" << std::endl;
  std::cout << "   OCS2 plan + OCS2 NMPC in MuJoCo" << std::endl;
  std::cout << "========================================" << std::endl;

  const std::string packagePath = roadster_stunt::getPath();
  const std::string taskFile = packagePath + "/config/task.info";
  const std::string modelPath = packagePath + "/robots/roadster.xml";

  try {
    std::cout << "\n[1] Planning the stunt offline (OCS2 SLQ)..." << std::endl;
    const PrimalSolution plan = computeStuntPlan(taskFile);

    std::cout << "\n[2] Setting up MuJoCo and the OCS2 NMPC..." << std::endl;
    StuntSimulation sim(modelPath, taskFile, plan);
    if (!logFile.empty()) {
      sim.openLog(logFile);
    }

    std::cout << "\n[3] Flying..." << std::endl;
    if (!record.empty()) {
      return runRecord(sim, record, width, height, fps, slowmo, outWidth);
    }
    if (headless) {
      return runHeadless(sim);
    }
    return runInteractive(sim, slowmo);
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] " << e.what() << std::endl;
    return 1;
  }
}
