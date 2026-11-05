/**
 * @file main_g1_backflip.cpp
 * @brief Standing backflip of the 29-DoF Unitree G1 in MuJoCo with the whole-body NMPC of wb_humanoid_mpc
 *        (OCS2 SQP + HPIPM), plain C++ (no ROS).
 *
 *  1. Plan : the whole manoeuvre (stand, crouch, push, flight with tuck, landing, stand) is optimised offline with
 *            OCS2 SQP on the whole-body dynamics (wb_humanoid_mpc's model), tracking a hand-designed keyframe guide.
 *  2. Run  : in MuJoCo, the whole-body SQP NMPC (100 Hz) tracks the plan; joint torques from the MPC's inverse
 *            dynamics + joint PD every 1 ms. The touchdown in the MPC follows the measured one.
 *
 * Modes
 *   g1_backflip_mujoco                        MuJoCo GUI (lock-step: the physics waits for the MPC)
 *   g1_backflip_mujoco --headless             scripted lock-step run, prints a report (exit code 0 = landed upright)
 *   g1_backflip_mujoco --record out.gif|mp4   same, rendered offscreen (ffmpeg)
 *
 * Options
 *   --flip-time T     start of the backflip plan [s] (default 1.5)
 *   --mpc-start T     MPC takes over from the posture hold [s] (default 0.5)
 *   --duration S      simulated time [s] (default: end of the plan + 2 s)
 *   --log FILE.csv    measured / planned trajectory
 *   --plan-csv FILE   the offline plan (time, state, input)
 *   --task FILE       another task.info
 *   --push T,FX,FZ    0.1 s force [N] on the pelvis at time T (x forward, z up)
 *   --record FILE     with --width --height --fps --playback (simulated seconds per video second) --scale
 */

#include <pinocchio/fwd.hpp>  // must come before Eigen / OCS2 headers that pull in pinocchio

#include <atomic>
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

#include <mujoco/mujoco.h>

#include "g1_backflip_mujoco/BackflipPlanner.h"
#include "g1_backflip_mujoco/controller/BackflipController.h"
#include "g1_backflip_mujoco/package_path.h"

#ifdef G1_BACKFLIP_WITH_GUI
#include <GLFW/glfw3.h>

#include "glfw_adapter.h"
#include "simulate.h"
#endif

using namespace g1_backflip;

namespace {

struct Options {
  std::string scene;
  bool headless = false;
  double flipTime = 1.5;
  double mpcStart = 0.5;
  double duration = -1.0;
  std::string logFile;
  std::string planCsv;
  std::string taskFile;
  std::string record;
  int width = 960;
  int height = 540;
  int fps = 30;
  int scale = 0;
  double playback = 1.0;
  double pushTime = -1.0;
  Eigen::Vector2d pushForce = Eigen::Vector2d::Zero();
};

void printUsage() {
  std::cout << "Usage: g1_backflip_mujoco [scene.xml] [--headless] [--flip-time T] [--mpc-start T] [--duration S]\n"
               "                          [--log FILE.csv] [--plan-csv FILE.csv] [--task task.info] [--push T,FX,FZ]\n"
               "                          [--record FILE.gif|mp4] [--width W] [--height H] [--fps F] [--playback X] [--scale W]\n";
}

bool parseOptions(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* what) -> std::string {
      if (i + 1 >= argc) {
        throw std::invalid_argument(std::string("missing value for ") + what);
      }
      return argv[++i];
    };
    if (a == "--headless") {
      o.headless = true;
    } else if (a == "--flip-time") {
      o.flipTime = std::stod(next("--flip-time"));
    } else if (a == "--mpc-start") {
      o.mpcStart = std::stod(next("--mpc-start"));
    } else if (a == "--duration") {
      o.duration = std::stod(next("--duration"));
    } else if (a == "--log") {
      o.logFile = next("--log");
    } else if (a == "--plan-csv") {
      o.planCsv = next("--plan-csv");
    } else if (a == "--task") {
      o.taskFile = next("--task");
    } else if (a == "--record") {
      o.record = next("--record");
      o.headless = true;
    } else if (a == "--width") {
      o.width = std::stoi(next("--width"));
    } else if (a == "--height") {
      o.height = std::stoi(next("--height"));
    } else if (a == "--fps") {
      o.fps = std::stoi(next("--fps"));
    } else if (a == "--scale") {
      o.scale = std::stoi(next("--scale"));
    } else if (a == "--playback") {
      o.playback = std::stod(next("--playback"));
    } else if (a == "--push") {
      const std::string v = next("--push");
      double t = 0, fx = 0, fz = 0;
      if (std::sscanf(v.c_str(), "%lf,%lf,%lf", &t, &fx, &fz) != 3) {
        throw std::invalid_argument("--push expects T,FX,FZ");
      }
      o.pushTime = t;
      o.pushForce << fx, fz;
    } else if (a == "--help" || a == "-h") {
      printUsage();
      return false;
    } else if (!a.empty() && a[0] != '-') {
      o.scene = a;
    } else {
      throw std::invalid_argument("unknown option " + a);
    }
  }
  return true;
}

/** True for a contact between a robot geom that is not a foot sphere and the floor (e.g. a knee or a hand). */
bool isBodyGroundContact(const mjModel* m, const mjContact& c, int robotRoot, const char** robotBody) {
  for (int k = 0; k < 2; ++k) {
    const int g = c.geom[k];
    const int other = c.geom[1 - k];
    const int body = m->geom_bodyid[g];
    if (m->body_rootid[body] != robotRoot || m->geom_bodyid[other] != 0 || m->geom_type[g] == mjGEOM_SPHERE) {
      continue;
    }
    *robotBody = mj_id2name(m, mjOBJ_BODY, body);
    return true;
  }
  return false;
}

/** True if the pelvis, torso, head or an arm touches the floor (a fall; knees and feet do not count). */
bool upperBodyOnGround(const mjModel* m, const mjData* d, int robotRoot) {
  for (int c = 0; c < d->ncon; ++c) {
    const char* body = nullptr;
    if (isBodyGroundContact(m, d->contact[c], robotRoot, &body) && body != nullptr) {
      const std::string name(body);
      if (name.find("knee") == std::string::npos && name.find("ankle") == std::string::npos &&
          name.find("hip") == std::string::npos) {
        return true;
      }
    }
  }
  return false;
}

std::string phaseLabel(const BackflipController& c, double t) {
  const double tau = t - c.flipStartTime();
  const auto& p = c.plan();
  if (!c.isMpcActive()) return "posture hold";
  if (tau < 0.0) return "NMPC: stand";
  if (tau < p.liftoffTime) return "NMPC: crouch + push";
  if (t < c.scheduledTouchdownTime()) return "NMPC: flight";
  if (tau < p.endTime) return "NMPC: landing";
  return "NMPC: stand";
}

// =============================================================================================
// Offscreen recorder (hidden GLFW window, ffmpeg pipe)
// =============================================================================================
#ifdef G1_BACKFLIP_WITH_GUI
class Recorder {
 public:
  bool open(const mjModel* m, const Options& o) {
    width_ = o.width;
    height_ = o.height;
    if (width_ > m->vis.global.offwidth || height_ > m->vis.global.offheight) {
      std::cerr << "[Record] " << width_ << "x" << height_ << " exceeds the model's offscreen buffer\n";
      return false;
    }
    if (!glfwInit()) {
      std::cerr << "[Record] could not initialise GLFW (no display? use xvfb-run)\n";
      return false;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(width_, height_, "recorder", nullptr, nullptr);
    if (window_ == nullptr) {
      std::cerr << "[Record] could not create an OpenGL context\n";
      glfwTerminate();
      return false;
    }
    glfwMakeContextCurrent(window_);
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);
    opt_.geomgroup[3] = 0;  // hide collision proxies
    mjv_makeScene(m, &scn_, 20000);
    mjr_makeContext(m, &con_, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_OFFSCREEN, &con_);
    cam_.type = mjCAMERA_FREE;
    cam_.distance = 3.4;
    cam_.azimuth = 70.0;  // side view, the robot faces +x (to the right)
    cam_.elevation = -8.0;

    const bool gif = o.record.size() > 4 && o.record.substr(o.record.size() - 4) == ".gif";
    char filter[256];
    std::snprintf(filter, sizeof(filter), "vflip,scale=%d:-2:flags=lanczos", o.scale > 0 ? o.scale : width_);
    char cmd[1024];
    if (gif) {
      std::snprintf(cmd, sizeof(cmd),
                    "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i - -vf "
                    "\"%s,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:"
                    "bayer_scale=3:diff_mode=rectangle\" -loop 0 \"%s\"",
                    width_, height_, o.fps, filter, o.record.c_str());
    } else {
      std::snprintf(cmd, sizeof(cmd),
                    "ffmpeg -y -loglevel error -f rawvideo -pix_fmt rgb24 -s %dx%d -r %d -i - -vf %s -pix_fmt yuv420p \"%s\"",
                    width_, height_, o.fps, filter, o.record.c_str());
    }
    pipe_ = popen(cmd, "w");
    if (pipe_ == nullptr) {
      std::cerr << "[Record] could not start ffmpeg\n";
      return false;
    }
    rgb_.resize(3 * width_ * height_);
    return true;
  }

  void frame(const mjModel* m, mjData* d, const Eigen::Vector3d& base, const std::string& label, const std::string& info) {
    const double target[3] = {base.x(), base.y(), 0.75};
    for (int k = 0; k < 3; ++k) {
      cam_.lookat[k] = initialized_ ? 0.95 * cam_.lookat[k] + 0.05 * target[k] : target[k];
    }
    initialized_ = true;
    mjv_updateScene(m, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    const mjrRect viewport = {0, 0, width_, height_};
    mjr_render(viewport, &scn_, &con_);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, "G1 29-DoF backflip, wb_humanoid_mpc NMPC", label.c_str(), &con_);
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, info.c_str(), nullptr, &con_);
    mjr_readPixels(rgb_.data(), nullptr, viewport, &con_);
    std::fwrite(rgb_.data(), 1, rgb_.size(), pipe_);
    ++frames_;
  }

  int close(int fps, const std::string& output) {
    int status = 0;
    if (pipe_ != nullptr) {
      status = pclose(pipe_);
      pipe_ = nullptr;
    }
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    if (window_ != nullptr) {
      glfwDestroyWindow(window_);
    }
    glfwTerminate();
    std::cout << "[Record] " << frames_ << " frames (" << frames_ / static_cast<double>(fps) << " s) -> " << output
              << (status == 0 ? "" : "  (ffmpeg FAILED)") << "\n";
    return status;
  }

 private:
  GLFWwindow* window_ = nullptr;
  mjvCamera cam_;
  mjvOption opt_;
  mjvScene scn_;
  mjrContext con_;
  FILE* pipe_ = nullptr;
  std::vector<unsigned char> rgb_;
  int width_ = 0, height_ = 0, frames_ = 0;
  bool initialized_ = false;
};
#endif

// =============================================================================================
// Headless: deterministic lock-step run (the physics waits for every MPC solve)
// =============================================================================================
int runHeadless(const Options& o, mjModel* m, mjData* d, BackflipController& controller) {
  const double dt = m->opt.timestep;
  const double mpcPeriod = 1.0 / controller.mpcFrequency();
  const double duration = o.duration > 0.0 ? o.duration : o.flipTime + controller.plan().endTime + 2.0;
  controller.setInitialPose(m, d);

#ifdef G1_BACKFLIP_WITH_GUI
  Recorder recorder;
  const bool recording = !o.record.empty();
  if (recording && !recorder.open(m, o)) {
    return 1;
  }
  double nextFrame = 0.0;
#else
  if (!o.record.empty()) {
    std::cerr << "--record needs the GUI build (GLFW + OpenGL)\n";
    return 1;
  }
#endif

  std::ofstream log;
  if (!o.logFile.empty()) {
    log.open(o.logFile);
    log << "t,x,y,z,pitch,pitch_plan,roll,yaw,vx,vz,feet,solve_ms,com_vx,com_vz,Lx,Ly,Lz,saturated,Fz_meas,Fz_des";
    for (const char* j : {"hipL", "kneeL", "ankleL", "waistP", "shoulderL", "elbowL"}) {
      log << "," << j << "," << j << "_des," << j << "_tau";
    }
    log << "\n";
  }

  const int pelvis = mj_name2id(m, mjOBJ_BODY, "pelvis");
  const int robotRoot = m->body_rootid[pelvis];
  double nextMpc = 0.0, nextLog = 0.0, nextPrint = 0.0;
  bool mpcWasActive = false;
  double solveTimeSum = 0.0, solveTimeMax = 0.0, maxPitchError = 0.0, apex = 0.0;
  double liftoffMeasured = -1.0, touchdownMeasured = -1.0, feetOffSince = -1.0;
  int numSolves = 0, bodyContactSteps = 0;
  double firstContactTime = -1.0;
  std::string firstContactBody;
  bool fell = false;
  const auto wallStart = std::chrono::steady_clock::now();

  while (d->time < duration) {
    const double t = d->time;

    controller.preStep(m, d);
    if (controller.isMpcActive() && (!mpcWasActive || t >= nextMpc - 1e-9)) {
      if (controller.advanceMpc()) {
        solveTimeSum += controller.lastSolveDuration();
        solveTimeMax = std::max(solveTimeMax, controller.lastSolveDuration());
        ++numSolves;
      }
      nextMpc = t + mpcPeriod;
    }
    mpcWasActive = controller.isMpcActive();

    mju_zero(d->xfrc_applied + 6 * pelvis, 6);
    if (o.pushTime >= 0.0 && t >= o.pushTime && t < o.pushTime + 0.1) {
      d->xfrc_applied[6 * pelvis + 0] = o.pushForce.x();
      d->xfrc_applied[6 * pelvis + 2] = o.pushForce.y();
    }

    mj_step(m, d);

    // ---- monitoring ----
    const Eigen::Vector3d base = controller.bridge().basePosition(d);
    const Eigen::Vector3d euler = controller.bridge().lastEulerZyx();
    const int feet = controller.feetInContact(m, d);
    const double tau = t - o.flipTime;
    // lift-off: both feet off the floor for 50 ms; touchdown: the first foot contact after it
    if (tau > 0.0 && liftoffMeasured < 0.0) {
      if (feet > 0) {
        feetOffSince = -1.0;
      } else if (feetOffSince < 0.0) {
        feetOffSince = t;
      } else if (t - feetOffSince >= 0.05) {
        liftoffMeasured = feetOffSince;
      }
    }
    if (liftoffMeasured > 0.0 && feet > 0 && touchdownMeasured < 0.0) {
      touchdownMeasured = t;
    }
    if (liftoffMeasured > 0.0) {
      apex = std::max(apex, base.z());
    }
    if (controller.isMpcActive() && tau > 0.0 && tau < controller.plan().endTime) {
      maxPitchError = std::max(maxPitchError, std::abs(euler(2) - controller.plannedPitch(t)));
    }
    for (int c = 0; c < d->ncon; ++c) {
      const char* body = nullptr;
      if (isBodyGroundContact(m, d->contact[c], robotRoot, &body)) {
        if (firstContactTime < 0.0) {
          firstContactTime = d->time;
          firstContactBody = body != nullptr ? body : "?";
        }
        ++bodyContactSteps;
        break;
      }
    }
    if (base.z() < 0.18 || upperBodyOnGround(m, d, robotRoot)) {
      fell = true;
      std::cout << "[Headless] FALL at t=" << t << " pelvis z=" << base.z() << " pitch=" << euler(2)
                << (base.z() >= 0.18 ? " (upper body on the floor)" : "") << "\n";
      break;
    }
    if (log.is_open() && t >= nextLog - 1e-9) {
      log << std::fixed << std::setprecision(4) << t << "," << base.x() << "," << base.y() << "," << base.z() << "," << euler(2)
          << "," << (tau > 0.0 ? controller.plannedPitch(t) : 0.0) << "," << euler(1) << "," << euler(0) << "," << d->qvel[0]
          << "," << d->qvel[2] << "," << feet << "," << 1e3 * controller.lastSolveDuration();
      mj_subtreeVel(m, d);
      log << "," << d->subtree_linvel[3 * pelvis] << "," << d->subtree_linvel[3 * pelvis + 2] << "," << d->subtree_angmom[3 * pelvis]
          << "," << d->subtree_angmom[3 * pelvis + 1] << "," << d->subtree_angmom[3 * pelvis + 2];
      std::string saturated;
      for (int a = 0; a < m->nu; ++a) {
        if (m->actuator_ctrllimited[a] && std::abs(d->ctrl[a]) >= m->actuator_ctrlrange[2 * a + 1] - 1e-6) {
          saturated += std::string(saturated.empty() ? "" : "|") + mj_id2name(m, mjOBJ_ACTUATOR, a);
        }
      }
      log << "," << (saturated.empty() ? "-" : saturated);
      double fz = 0.0;
      for (int c = 0; c < d->ncon; ++c) {
        const int b1 = m->geom_bodyid[d->contact[c].geom[0]], b2 = m->geom_bodyid[d->contact[c].geom[1]];
        if (b1 == 0 || b2 == 0) {
          mjtNum f[6];
          mj_contactForce(m, d, c, f);
          fz += f[0] * d->contact[c].frame[2];  // normal force, world z component of the normal
        }
      }
      const auto& ud = controller.desiredInput();
      log << "," << std::abs(fz) << "," << (ud.size() ? ud(2) + ud(8) : 0.0);
      const auto& xm = controller.measuredState();
      const auto& xd = controller.desiredState();
      const auto& tq = controller.appliedTorques();
      for (int j : {0, 3, 4, 14, 15, 18}) {
        log << "," << (xm.size() ? xm(6 + j) : 0.0) << "," << (xd.size() ? xd(6 + j) : 0.0) << "," << (tq.size() ? tq(j) : 0.0);
      }
      log << "\n";
      nextLog += 0.005;
    }
    if (t >= nextPrint - 1e-9) {
      std::cout << std::fixed << std::setprecision(2) << "[t=" << std::setw(5) << t << "] " << std::setw(20) << phaseLabel(controller, t)
                << " x=" << std::setw(5) << base.x() << " z=" << base.z() << " pitch=" << std::setw(6) << euler(2)
                << " roll=" << std::setw(5) << euler(1) << " feet=" << feet << " solve=" << std::setprecision(1)
                << 1e3 * controller.lastSolveDuration() << "ms\n";
      nextPrint += (tau > -0.2 && tau < controller.plan().endTime) ? 0.05 : 0.5;
    }
#ifdef G1_BACKFLIP_WITH_GUI
    if (recording && d->time >= nextFrame) {
      char label[256], info[256];
      std::snprintf(label, sizeof(label), "t=%.2f s  %s", d->time, phaseLabel(controller, d->time).c_str());
      std::snprintf(info, sizeof(info), "pitch %+5.0f deg   pelvis height %.2f m", euler(2) * 180.0 / M_PI, base.z());
      recorder.frame(m, d, base, label, info);
      nextFrame += o.playback / o.fps;
    }
#endif
  }

  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
  const Eigen::Vector3d base = controller.bridge().basePosition(d);
  const Eigen::Vector3d euler = controller.bridge().lastEulerZyx();
  const double finalPitchError = euler(2) + 2.0 * M_PI;  // flip_base roll = robot pitch
  const bool flipped = std::abs(finalPitchError) < 0.25 && std::abs(euler(1)) < 0.25;
  const bool standing = base.z() > 0.6 && controller.feetInContact(m, d) == 2;
  const bool success = !fell && flipped && standing && bodyContactSteps == 0;
  std::cout << "\n================ Headless report ================\n"
            << std::fixed << std::setprecision(3) << "  simulated        : " << d->time << " s (wall " << wall << " s)\n"
            << "  lift-off         : " << (liftoffMeasured > 0 ? liftoffMeasured - o.flipTime : -1.0) << " s (plan "
            << controller.plan().liftoffTime << ")\n"
            << "  touchdown        : " << (touchdownMeasured > 0 ? touchdownMeasured - o.flipTime : -1.0) << " s (plan "
            << controller.plan().touchdownTime << ")\n"
            << "  apex pelvis z    : " << apex << " m\n"
            << "  max |pitch - plan|: " << maxPitchError << " rad\n"
            << "  final pitch      : " << euler(2) << " rad (" << finalPitchError * 180.0 / M_PI << " deg from a full turn), roll "
            << euler(1) << " rad\n"
            << "  final pelvis     : (" << base.transpose() << ")\n"
            << "  MPC solves       : " << numSolves << "  avg " << (numSolves ? 1e3 * solveTimeSum / numSolves : 0.0) << " ms  max "
            << 1e3 * solveTimeMax << " ms\n"
            << "  non-foot contact : " << bodyContactSteps * dt << " s"
            << (firstContactTime >= 0.0 ? " (first: " + firstContactBody + " at t=" + std::to_string(firstContactTime) + ")" : "") << "\n"
            << "  result           : " << (success ? "SUCCESS (full backflip, landed and standing)" : "FAILURE") << "\n"
            << "=================================================\n";

#ifdef G1_BACKFLIP_WITH_GUI
  if (recording && recorder.close(o.fps, o.record) != 0) {
    return 1;
  }
#endif
  return success ? 0 : 2;
}

// =============================================================================================
// Interactive GUI (lock-step: the physics waits for the MPC, so it runs slower than real time)
// =============================================================================================
#ifdef G1_BACKFLIP_WITH_GUI
std::atomic<bool> g_exit{false};

void physicsThread(mujoco::Simulate* sim, mjModel* m, mjData* d, BackflipController& controller, const std::string& scenePath) {
  sim->Load(m, d, scenePath.c_str());
  const double mpcPeriod = 1.0 / controller.mpcFrequency();
  double nextMpc = 0.0;
  double lastTime = d->time;
  using Clock = std::chrono::steady_clock;

  while (!g_exit && !sim->exitrequest.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
    if (sim->m_ == nullptr || sim->d_ == nullptr) {
      continue;
    }
    // the GUI reset (Backspace) restarts the backflip
    if (sim->d_->time < lastTime - 1e-9 || (sim->d_->time == 0.0 && lastTime > 0.0)) {
      controller.reset();
      controller.setInitialPose(sim->m_, sim->d_);
      nextMpc = 0.0;
    }
    sim->Sync();
    if (sim->run) {
      // as many lock-step physics steps as fit into one frame (~16 ms of wall time)
      const auto start = Clock::now();
      const double slowdown = 100.0 / sim->percentRealTime[sim->real_time_index];
      const double simBudget = 0.016 / slowdown;
      const double simStart = sim->d_->time;
      while (sim->d_->time - simStart < simBudget && Clock::now() - start < std::chrono::milliseconds(30)) {
        controller.preStep(sim->m_, sim->d_);
        if (controller.isMpcActive() && sim->d_->time >= nextMpc - 1e-9) {
          controller.advanceMpc();
          nextMpc = sim->d_->time + mpcPeriod;
        }
        mj_step(sim->m_, sim->d_);
      }
      sim->AddToHistory();
    } else {
      mj_forward(sim->m_, sim->d_);
    }
    lastTime = sim->d_->time;
  }
}

int runGui(const Options& o, mjModel* m, mjData* d, BackflipController& controller) {
  controller.setInitialPose(m, d);
  mjvCamera cam;
  mjvOption opt;
  mjvPerturb pert;
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultPerturb(&pert);
  cam.type = mjCAMERA_FREE;
  cam.lookat[0] = -0.1;
  cam.lookat[2] = 0.75;
  cam.distance = 3.4;
  cam.azimuth = 70;
  cam.elevation = -8;
  opt.geomgroup[3] = 0;

  auto sim = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &cam, &opt, &pert, false);
  std::cout << "\n========================================\n"
            << "  The backflip starts at t=" << o.flipTime << " s (simulation time).\n"
            << "  The physics waits for every MPC solve: slower than real time.\n"
            << "  MuJoCo window: Space pause, Backspace reset (= flip again), Esc quit\n"
            << "========================================\n";
  std::thread physics(physicsThread, sim.get(), m, d, std::ref(controller), o.scene);
  sim->RenderLoop();
  g_exit = true;
  sim->exitrequest.store(1);
  physics.join();
  return 0;
}
#endif

}  // namespace

int main(int argc, char** argv) {
  Options o;
  const std::string pkg = getPackagePath();
  o.scene = pkg + "/robots/g1_29dof/scene.xml";
  try {
    if (!parseOptions(argc, argv, o)) {
      return 0;
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    printUsage();
    return 1;
  }
  const std::string taskFile = o.taskFile.empty() ? pkg + "/config/task.info" : o.taskFile;
  const std::string urdfFile = pkg + "/robots/g1_29dof/urdf/g1_29dof_flip.urdf";

  std::cout << "========================================\n"
            << "  Unitree G1 (29 DoF) backflip: whole-body NMPC of wb_humanoid_mpc in MuJoCo (no ROS)\n"
            << "  scene: " << o.scene << "\n"
            << "  task : " << taskFile << "\n"
            << "========================================\n";

  char error[1000] = "";
  mjModel* m = mj_loadXML(o.scene.c_str(), nullptr, error, sizeof(error));
  if (m == nullptr) {
    std::cerr << "Could not load " << o.scene << ": " << error << "\n";
    return 1;
  }
  mjData* d = mj_makeData(m);

  int status = 0;
  try {
    std::cout << "\n[1] Planning the backflip offline (OCS2 SQP, whole-body dynamics, 23 MPC joints)...\n";
    BackflipPlan plan;
    {
      G1WbBackflipInterface planInterface(taskFile, urdfFile, "plan");
      BackflipGuide guide(taskFile, planInterface.getPinocchioInterface(), planInterface.modelSettings().contactNames);
      plan = computeBackflipPlan(planInterface, guide);
    }
    if (!o.planCsv.empty()) {
      writePlanCsv(plan, o.planCsv);
    }

    std::cout << "\n[2] Running the NMPC in MuJoCo...\n";
    BackflipController controller(taskFile, urdfFile, m, std::move(plan), o.mpcStart, o.flipTime);
    if (o.headless) {
      status = runHeadless(o, m, d, controller);
    } else {
#ifdef G1_BACKFLIP_WITH_GUI
      status = runGui(o, m, d, controller);
#else
      std::cerr << "This build has no GUI (GLFW/OpenGL not found); use --headless\n";
      status = 1;
#endif
    }
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] " << e.what() << "\n";
    status = 1;
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return status;
}
