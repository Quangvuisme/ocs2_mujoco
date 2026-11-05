/**
 * @file main_g1_mujoco.cpp
 * @brief Centroidal NMPC for the Unitree G1 humanoid (12 leg joints) in MuJoCo, plain C++ (no ROS).
 *
 * OCS2 SLQ MPC on the single rigid body centroidal model with 4 point contacts (toe + heel of every
 * foot), RNEA whole-body torques with joint PD, perfect state estimation from the simulator.
 *
 * Modes
 *   g1_walking_mujoco [scene.xml]                 interactive MuJoCo GUI (keyboard / Xbox)
 *   g1_walking_mujoco --headless [options]        scripted run in lock-step (deterministic), prints a report
 *   g1_walking_mujoco --headless --record out.gif same, rendered offscreen to a GIF / MP4 (ffmpeg)
 *
 * Headless options
 *   --duration S      simulated time [s]               (default 20)
 *   --vx V            forward speed command [m/s]      (default 0.3)
 *   --vy V            lateral speed command [m/s]      (default 0)
 *   --yaw-rate W      yaw rate command [rad/s]         (default 0; non-zero disables the heading hold)
 *   --goal-x X        stop and stand once the base passes x = X; success criterion
 *   --gait NAME       gait from config/command/gait.info (default walk)
 *   --log FILE.csv    base trajectory log
 *   --record FILE     render to FILE (.gif or .mp4) with --width/--height/--fps/--scale/--playback
 *   --task FILE       use another task.info
 *   --base-height H   pelvis height [m] (default: FSM default)
 *   --walk-start T    time of the switch from STANDING to WALKING [s] (default 3.0)
 *   --push T,FX,FY    apply a 0.1 s force (FX, FY) [N] on the pelvis at time T
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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include "g1_walking_mujoco/KeyboardController.h"
#include "g1_walking_mujoco/XboxController.h"
#include "g1_walking_mujoco/controller/G1Controller.h"
#include "g1_walking_mujoco/package_path.h"

#ifdef G1_WALKING_WITH_GUI
#include <GLFW/glfw3.h>

#include "glfw_adapter.h"
#include "simulate.h"
#endif

using namespace g1_walking;

namespace {

struct Options {
  std::string scene;
  bool headless = false;
  double duration = 20.0;
  double vx = 0.3;
  double vy = 0.0;
  double yawRate = 0.0;
  double goalX = -1.0;
  std::string gait = "walk";
  std::string logFile;
  std::string record;
  int width = 960;
  int height = 540;
  int fps = 30;
  int scale = 640;
  double playback = 1.0;  // simulated seconds per video second
  std::string taskFile;   // overrides config/mpc/task.info
  double baseHeight = -1.0;  // pelvis height [m] (< 0: FSM default)
  double walkStart = 3.0; // [s] scripted switch STANDING -> WALKING
  double pushTime = -1.0; // [s] lateral push on the pelvis
  Eigen::Vector2d pushForce = Eigen::Vector2d::Zero();
};

void printUsage() {
  std::cout << "Usage: g1_walking_mujoco [scene.xml] [--headless] [--duration S] [--vx V] [--vy V] [--yaw-rate W]\n"
               "                         [--goal-x X] [--gait NAME] [--log FILE.csv] [--record FILE.gif|mp4]\n"
               "                         [--width W] [--height H] [--fps F] [--scale W] [--playback X]\n"
               "                         [--task task.info] [--base-height H] [--walk-start T] [--push T,FX,FY]\n";
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
    } else if (a == "--duration") {
      o.duration = std::stod(next("--duration"));
    } else if (a == "--vx") {
      o.vx = std::stod(next("--vx"));
    } else if (a == "--vy") {
      o.vy = std::stod(next("--vy"));
    } else if (a == "--yaw-rate") {
      o.yawRate = std::stod(next("--yaw-rate"));
    } else if (a == "--goal-x") {
      o.goalX = std::stod(next("--goal-x"));
    } else if (a == "--gait") {
      o.gait = next("--gait");
    } else if (a == "--log") {
      o.logFile = next("--log");
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
    } else if (a == "--base-height") {
      o.baseHeight = std::stod(next("--base-height"));
    } else if (a == "--walk-start") {
      o.walkStart = std::stod(next("--walk-start"));
    } else if (a == "--push") {
      const std::string v = next("--push");
      double t = 0, fx = 0, fy = 0;
      if (std::sscanf(v.c_str(), "%lf,%lf,%lf", &t, &fx, &fy) != 3) {
        throw std::invalid_argument("--push expects T,FX,FY");
      }
      o.pushTime = t;
      o.pushForce << fx, fy;
    } else if (a == "--task") {
      o.taskFile = next("--task");
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

/** Robot in the default posture at (0, 0), feet 1 cm above the ground. */
void setInitialPose(const mjModel* m, mjData* d, G1Controller& controller) {
  mj_resetData(m, d);
  controller.bridge().setPose(d, Eigen::Vector3d(0.0, 0.0, 0.745), 0.0, controller.defaultJointPositions());
  mj_forward(m, d);
}

/** True for a contact between a robot geom that is not a foot sphere and a world geom (e.g. a knee on the floor). */
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

// =============================================================================================
// Offscreen recorder (hidden GLFW window, ffmpeg pipe)
// =============================================================================================
#ifdef G1_WALKING_WITH_GUI
class Recorder {
 public:
  bool open(const mjModel* m, const Options& o) {
    width_ = o.width;
    height_ = o.height;
    if (width_ > m->vis.global.offwidth || height_ > m->vis.global.offheight) {
      std::cerr << "[Record] " << width_ << "x" << height_ << " exceeds the model's offscreen buffer ("
                << m->vis.global.offwidth << "x" << m->vis.global.offheight << ")\n";
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
    cam_.distance = 3.0;
    cam_.azimuth = 135.0;
    cam_.elevation = -15.0;

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

  void frame(const mjModel* m, mjData* d, const Eigen::Vector3d& base, const std::string& label) {
    // smooth follow camera
    for (int k = 0; k < 3; ++k) {
      const double target = k == 2 ? std::max(base.z() - 0.15, 0.3) : base(k);
      cam_.lookat[k] = initialized_ ? 0.9 * cam_.lookat[k] + 0.1 * target : target;
    }
    initialized_ = true;
    mjv_updateScene(m, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    const mjrRect viewport = {0, 0, width_, height_};
    mjr_render(viewport, &scn_, &con_);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, "Unitree G1 (12 DoF) centroidal NMPC", label.c_str(), &con_);
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
// Headless: deterministic lock-step run (physics waits for every MPC solve)
// =============================================================================================
int runHeadless(const Options& o, mjModel* m, mjData* d, G1Controller& controller) {
  const double dt = m->opt.timestep;
  const double mpcPeriod = 1.0 / controller.mpcFrequency();
  const double fsmPeriod = 0.01;

  setInitialPose(m, d, controller);

#ifdef G1_WALKING_WITH_GUI
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
    log << "t,mode,x,y,z,yaw,pitch,roll,vx,vy,solve_ms\n";
  }

  // script
  const double tStand = 1.6;         // PD stand-up -> MPC double support
  const double tWalk = o.walkStart;  // -> walking (gait, zero velocity)
  const double tGo = tWalk + 1.5;    // -> velocity command
  double tGoal = -1.0;               // time the goal was reached

  const int pelvis = mj_name2id(m, mjOBJ_BODY, "pelvis");
  const int robotRoot = m->body_rootid[pelvis];
  double nextMpc = 0.0, nextFsm = 0.0, nextLog = 0.0, nextPrint = 0.0;
  bool mpcWasActive = false;
  bool fell = false;
  double maxX = 0.0, solveTimeSum = 0.0, solveTimeMax = 0.0;
  int numSolves = 0;
  int bodyContactSteps = 0;
  double firstContactTime = -1.0;
  std::string firstContactBody;
  const Eigen::Vector3d zyxStart = controller.bridge().baseEulerZyx(d);
  const auto wallStart = std::chrono::steady_clock::now();

  controller.withFsm([](FSMController& f) {
    f.requestStateChange(FSMState::PD_CONTROL);
    return 0;
  });

  int stage = 0;
  while (d->time < o.duration) {
    const double t = d->time;
    const Eigen::Vector3d base = controller.bridge().basePosition(d);
    maxX = std::max(maxX, base.x());

    // ---- script ----
    if (stage == 0 && t >= tStand) {
      controller.withFsm([](FSMController& f) {
        f.requestStateChange(FSMState::STANDING);
        return 0;
      });
      stage = 1;
    } else if (stage == 1 && t >= tWalk) {
      controller.withFsm([&](FSMController& f) {
        f.requestStateChange(FSMState::WALKING);
        return 0;
      });
      stage = 2;
    } else if (stage == 2 && t >= tWalk + 0.05) {
      controller.referenceManager().requestGait(o.gait);
      stage = 3;
    } else if (stage == 3 && t >= tGo) {
      controller.withFsm([&](FSMController& f) {
        f.getWalkingTargetMutable().vx = o.vx;
        f.getWalkingTargetMutable().vy = o.vy;
        f.getWalkingTargetMutable().yaw_rate = o.yawRate;
        return 0;
      });
      stage = 4;
    } else if (stage == 4 && o.goalX > 0.0 && base.x() >= o.goalX) {
      controller.withFsm([](FSMController& f) {
        f.getWalkingTargetMutable().vx = 0.0;
        f.getWalkingTargetMutable().vy = 0.0;
        f.getWalkingTargetMutable().yaw_rate = 0.0;
        return 0;
      });
      tGoal = t;
      stage = 5;
      std::cout << "[Headless] goal x=" << o.goalX << " reached at t=" << t << " s\n";
    } else if (stage == 5 && t >= tGoal + 2.0) {
      controller.withFsm([](FSMController& f) {
        f.requestStateChange(FSMState::STANDING);
        return 0;
      });
      stage = 6;
    } else if (stage == 6 && t >= tGoal + 4.0) {
      break;
    }

    if (o.baseHeight > 0.0) {
      controller.withFsm([&](FSMController& f) {
        f.getTargetMutable().z = o.baseHeight;
        f.getWalkingTargetMutable().z = o.baseHeight;
        return 0;
      });
    }
    // operator: hold the start heading while walking straight
    if (stage == 4 && o.yawRate == 0.0) {
      const Eigen::Vector3d zyxNow = controller.bridge().baseEulerZyx(d);
      controller.withFsm([&](FSMController& f) {
        f.getWalkingTargetMutable().yaw_rate = std::clamp(-1.0 * (zyxNow(0) - zyxStart(0)), -0.3, 0.3);
        return 0;
      });
    }

    // ---- FSM ----
    if (t >= nextFsm - 1e-9) {
      controller.withFsm([&](FSMController& f) {
        f.update(fsmPeriod);
        return 0;
      });
      nextFsm += fsmPeriod;
    }

    // ---- control + MPC (lock-step) ----
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

    // ---- disturbance ----
    mju_zero(d->xfrc_applied + 6 * pelvis, 6);
    if (o.pushTime >= 0.0 && t >= o.pushTime && t < o.pushTime + 0.1) {
      d->xfrc_applied[6 * pelvis + 0] = o.pushForce.x();
      d->xfrc_applied[6 * pelvis + 1] = o.pushForce.y();
    }

    mj_step(m, d);

    // ---- monitoring ----
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
    const Eigen::Vector3d zyx = controller.bridge().baseEulerZyx(d);
    if (controller.isMpcActive() && (base.z() < 0.5 || std::abs(zyx(1)) > 0.6 || std::abs(zyx(2)) > 0.6)) {
      fell = true;
      std::cout << "[Headless] FALL at t=" << t << " base=(" << base.transpose() << ") pitch=" << zyx(1)
                << " roll=" << zyx(2) << "\n";
      break;
    }
    const std::string mode = controller.withFsm([](FSMController& f) { return fsmStateToString(f.getCurrentState()); });
    if (log.is_open() && t >= nextLog - 1e-9) {
      log << std::fixed << std::setprecision(4) << t << "," << mode << "," << base.x() << "," << base.y() << "," << base.z()
          << "," << zyx(0) << "," << zyx(1) << "," << zyx(2) << "," << d->qvel[0] << "," << d->qvel[1] << ","
          << 1e3 * controller.lastSolveDuration() << "\n";
      nextLog += 0.02;
    }
    if (t >= nextPrint - 1e-9) {
      std::cout << std::fixed << std::setprecision(2) << "[t=" << std::setw(5) << t << "] " << std::setw(10) << mode
                << " x=" << std::setw(5) << base.x() << " y=" << std::setw(5) << base.y() << " z=" << base.z()
                << " yaw=" << std::setw(5) << zyx(0) << " pitch=" << std::setw(5) << zyx(1) << " roll=" << std::setw(5)
                << zyx(2) << " vx=" << std::setw(5) << d->qvel[0] << " solve=" << std::setprecision(1)
                << 1e3 * controller.lastSolveDuration() << "ms\n";
      nextPrint += 1.0;
    }
#ifdef G1_WALKING_WITH_GUI
    if (recording && d->time >= nextFrame) {
      char label[256];
      std::snprintf(label, sizeof(label), "t=%.1fs  %s  v=%.2f m/s", d->time, mode.c_str(), d->qvel[0]);
      recorder.frame(m, d, controller.bridge().basePosition(d), label);
      nextFrame += o.playback / o.fps;
    }
#endif
  }

  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
  const Eigen::Vector3d base = controller.bridge().basePosition(d);
  const bool reached = o.goalX <= 0.0 || maxX >= o.goalX;
  std::cout << "\n================ Headless report ================\n"
            << std::fixed << std::setprecision(3) << "  simulated        : " << d->time << " s (wall " << wall << " s)\n"
            << "  final base       : (" << base.transpose() << ")\n"
            << "  max x reached    : " << maxX << " m" << (o.goalX > 0.0 ? "  (goal " + std::to_string(o.goalX) + ")" : "")
            << "\n"
            << "  MPC solves       : " << numSolves << "  avg " << (numSolves ? 1e3 * solveTimeSum / numSolves : 0.0)
            << " ms  max " << 1e3 * solveTimeMax << " ms\n"
            << "  non-foot contact : " << bodyContactSteps * dt << " s"
            << (firstContactTime >= 0.0 ? " (first: " + firstContactBody + " at t=" + std::to_string(firstContactTime) + ")" : "")
            << "\n"
            << "  fell             : " << (fell ? "YES" : "no") << "\n"
            << "  result           : " << (!fell && reached ? "SUCCESS" : "FAILURE") << "\n"
            << "=================================================\n";

#ifdef G1_WALKING_WITH_GUI
  if (recording && recorder.close(o.fps, o.record) != 0) {
    return 1;
  }
#endif
  return !fell && reached ? 0 : 2;
}

// =============================================================================================
// Interactive GUI
// =============================================================================================
#ifdef G1_WALKING_WITH_GUI
std::atomic<bool> g_exit{false};

void inputThread(G1Controller& controller) {
  XboxController xbox;
  KeyboardController keyboard;
  const double dt = 0.01;
  int printCount = 0;
  while (!g_exit) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    float lx = 0, ly = 0, rx = 0, ry = 0;
    int modeRequest = -1;
    bool start = false, back = false, gaitSwitch = false;
    int heightAdjust = 0;
    if (xbox.isConnected()) {
      const auto in = xbox.getInput();
      lx = in.lx, ly = in.ly, rx = in.rx, ry = in.ry;
      modeRequest = xbox.getModeChangeRequest();
      start = xbox.wasStartPressed();
      back = xbox.wasBackPressed();
      gaitSwitch = xbox.wasGaitSwitchPressed();
      heightAdjust = xbox.getHeightAdjustRequest();
    }
    if (keyboard.isConnected()) {
      const auto in = keyboard.getInput();
      if (std::abs(in.lx) > 0.01f || std::abs(in.ly) > 0.01f || std::abs(in.rx) > 0.01f || std::abs(in.ry) > 0.01f) {
        lx = in.lx, ly = in.ly, rx = in.rx, ry = in.ry;
      }
      const int kbMode = keyboard.getModeChangeRequest();
      modeRequest = kbMode >= 0 ? kbMode : modeRequest;
      start |= keyboard.wasStartPressed();
      back |= keyboard.wasBackPressed();
      gaitSwitch |= keyboard.wasGaitSwitchPressed();
      heightAdjust += keyboard.getHeightAdjustRequest();
    }

    controller.withFsm([&](FSMController& fsm) {
      static const FSMState modes[4] = {FSMState::PASSIVE, FSMState::PD_CONTROL, FSMState::STANDING, FSMState::WALKING};
      if (modeRequest >= 0 && modeRequest < 4) {
        fsm.requestStateChange(modes[modeRequest]);
      }
      if (start) {
        fsm.toggleMpc();
      }
      if (back) {
        fsm.emergencyStop();
      }
      if (gaitSwitch && fsm.getCurrentState() == FSMState::WALKING) {
        fsm.nextGait();
      }
      if (heightAdjust != 0) {
        fsm.adjustHeight(0.02 * heightAdjust);
      }
      if (fsm.getCurrentState() == FSMState::WALKING) {
        fsm.updateWalkingFromXbox(lx, ly, rx, ry, dt);
      }
      fsm.update(dt);
      if (++printCount % 100 == 0) {
        std::cout << "[FSM] " << fsm.getStatusString() << " | MPC solve " << std::fixed << std::setprecision(1)
                  << 1e3 * controller.lastSolveDuration() << " ms" << std::endl;
      }
      return 0;
    });
  }
}

void mpcThread(G1Controller& controller, mujoco::Simulate* sim) {
  const auto period = std::chrono::duration<double>(1.0 / controller.mpcFrequency());
  auto next = std::chrono::steady_clock::now();
  while (!g_exit) {
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    if (sim->run && controller.isMpcActive()) {
      controller.advanceMpc();
    }
    std::this_thread::sleep_until(next);
    if (std::chrono::steady_clock::now() > next + std::chrono::milliseconds(100)) {
      next = std::chrono::steady_clock::now();  // fell behind: do not try to catch up
    }
  }
}

void physicsThread(mujoco::Simulate* sim, mjModel* m, mjData* d, G1Controller& controller,
                   const std::string& scenePath) {
  sim->Load(m, d, scenePath.c_str());

  using Clock = std::chrono::steady_clock;
  auto syncCpu = Clock::now();
  mjtNum syncSim = 0;
  double lastTime = d->time;

  while (!g_exit && !sim->exitrequest.load()) {
    if (sim->run && sim->busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
    if (sim->m_ == nullptr || sim->d_ == nullptr) {
      continue;
    }
    // the GUI reset (Backspace) puts the robot back into the XML pose
    if (sim->d_->time < lastTime - 1e-9 || (sim->d_->time == 0.0 && lastTime > 0.0)) {
      controller.onSimulationReset();
      setInitialPose(sim->m_, sim->d_, controller);
    }
    lastTime = sim->d_->time;

    sim->Sync();

    if (sim->run) {
      const auto startCpu = Clock::now();
      const double slowdown = 100.0 / sim->percentRealTime[sim->real_time_index];
      const double elapsedSim = sim->d_->time - syncSim;
      const bool misaligned =
          std::abs(std::chrono::duration<double>(startCpu - syncCpu).count() / slowdown - elapsedSim) > 0.1;
      if (elapsedSim < 0 || syncCpu.time_since_epoch().count() == 0 || misaligned || sim->speed_changed) {
        syncCpu = startCpu;
        syncSim = sim->d_->time;
        sim->speed_changed = false;
        controller.preStep(sim->m_, sim->d_);
        mj_step(sim->m_, sim->d_);
      } else {
        const double refreshTime = 0.7 / sim->refresh_rate;
        const mjtNum prevSim = sim->d_->time;
        while (std::chrono::duration<double>((sim->d_->time - syncSim) * slowdown) < Clock::now() - syncCpu &&
               Clock::now() - startCpu < std::chrono::duration<double>(refreshTime)) {
          controller.preStep(sim->m_, sim->d_);
          mj_step(sim->m_, sim->d_);
          if (sim->d_->time < prevSim) {
            break;
          }
        }
      }
      sim->AddToHistory();
      lastTime = sim->d_->time;
    } else {
      mj_forward(sim->m_, sim->d_);
      sim->speed_changed = true;
    }
  }
}

int runGui(const Options& o, mjModel* m, mjData* d, G1Controller& controller) {
  setInitialPose(m, d, controller);

  mjvCamera cam;
  mjvOption opt;
  mjvPerturb pert;
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultPerturb(&pert);
  cam.type = mjCAMERA_TRACKING;
  cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "pelvis");
  cam.distance = 3.0;
  cam.azimuth = 135;
  cam.elevation = -15;
  opt.geomgroup[3] = 0;

  auto sim = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &cam, &opt, &pert, false);

  std::cout << "\n========================================\n"
            << "  Keyboard (terminal) / Xbox controls\n"
            << "    1 / A   PASSIVE\n"
            << "    2 / B   PD stand-up\n"
            << "    3 / X   STANDING (MPC)\n"
            << "    4 / Y   WALKING  (MPC, walk gait)\n"
            << "    W/S A/D forward / lateral velocity, J/L yaw rate\n"
            << "    G / RB  next gait (walk -> step -> quick step),  -/= height\n"
            << "  MuJoCo window: Space pause, Backspace reset, Esc quit\n"
            << "========================================\n";

  std::thread physics(physicsThread, sim.get(), m, d, std::ref(controller), o.scene);
  std::thread mpc(mpcThread, std::ref(controller), sim.get());
  std::thread input(inputThread, std::ref(controller));

  sim->RenderLoop();

  g_exit = true;
  sim->exitrequest.store(1);
  physics.join();
  mpc.join();
  input.join();
  return 0;
}
#endif

}  // namespace

int main(int argc, char** argv) {
  Options o;
  const std::string pkg = getPackagePath();
  o.scene = pkg + "/robots/g1/scene.xml";
  try {
    if (!parseOptions(argc, argv, o)) {
      return 0;
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    printUsage();
    return 1;
  }

  G1Controller::Paths paths;
  paths.taskFile = o.taskFile.empty() ? pkg + "/config/mpc/task.info" : o.taskFile;
  paths.urdfFile = pkg + "/robots/g1/urdf/g1_12dof.urdf";
  paths.referenceFile = pkg + "/config/command/reference.info";
  paths.gaitFile = pkg + "/config/command/gait.info";

  std::cout << "========================================\n"
            << "  Unitree G1 (12 DoF) centroidal NMPC (OCS2 + MuJoCo, no ROS)\n"
            << "  scene: " << o.scene << "\n"
            << "  task : " << paths.taskFile << "\n"
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
    G1Controller controller(paths, m);
    if (o.headless) {
      status = runHeadless(o, m, d, controller);
    } else {
#ifdef G1_WALKING_WITH_GUI
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
