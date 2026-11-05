/**
 * @file main_g1_perceptive.cpp
 * @brief Perceptive NMPC for the Unitree G1 humanoid (12 leg DoF) in MuJoCo, plain C++ (no ROS).
 *
 * The elevation map is ray cast straight from the MuJoCo scene (MujocoElevationMapper), the OCS2
 * centroidal MPC places both soles (toe + heel) on steppable terrain, lifts the swing feet over
 * obstacles and keeps the pelvis at a height above the terrain.
 *
 * Modes
 *   g1_perceptive_mujoco [scene.xml]                 interactive MuJoCo GUI (keyboard / Xbox)
 *   g1_perceptive_mujoco --headless [options]        scripted run in lock-step (deterministic), prints a report
 *   g1_perceptive_mujoco --headless --record out.gif same, rendered offscreen to a GIF / MP4 (ffmpeg)
 *
 * Headless options
 *   --duration S      simulated time [s]               (default 60)
 *   --vx V            forward speed command [m/s]      (default 0.3)
 *   --goal-x X        stop and stand once the base passes x = X; success criterion
 *   --gait NAME       gait from config/command/gait.info (default walk)
 *   --log FILE.csv    base trajectory log
 *   --trace FILE.csv  detailed trace every 10 ms: base, toe / heel positions, next planned footholds, MPC-predicted
 *                     toe / heel positions 0.2 s ahead, joint angles and torques
 *   --record FILE     render to FILE (.gif or .mp4) with --width/--height/--fps/--scale/--playback
 *   --no-map          do not draw the elevation map
 *   --blind           do not give the map to the MPC (flat-ground assumption), for comparison
 *   --task FILE       use another task.info
 *   --base-height H   pelvis height above the terrain [m] (default: FSM default 0.73 standing / 0.72 walking)
 *   --walk-start T    time of the switch from STANDING to WALKING [s] (default 3.0)
 *   --start-y Y       initial lateral position [m] (default 0)
 *   --report-hits     print every collision of a leg / the body (not a foot) with the terrain
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

#include "g1_perceptive_mujoco/KeyboardController.h"
#include "g1_perceptive_mujoco/XboxController.h"
#include "g1_perceptive_mujoco/controller/G1Controller.h"
#include "g1_perceptive_mujoco/package_path.h"
#include "g1_perceptive_mujoco/terrain/MujocoElevationMapper.h"
#include "g1_perceptive_mujoco/visualization/TerrainVisualizer.h"

#ifdef G1_PERCEPTIVE_WITH_GUI
#include <GLFW/glfw3.h>

#include "glfw_adapter.h"
#include "simulate.h"
#endif

using namespace g1_perceptive;

namespace {

struct Options {
  std::string scene;
  bool headless = false;
  double duration = 60.0;
  double vx = 0.3;
  double goalX = -1.0;
  std::string gait = "walk";
  std::string logFile;
  std::string traceFile;
  std::string record;
  int width = 960;
  int height = 540;
  int fps = 30;
  int scale = 640;
  double playback = 1.0;  // simulated seconds per video second
  int mapStride = 2;      // draw every n-th elevation map cell
  bool showMap = true;
  bool blind = false;     // do not feed the elevation map to the MPC (flat-ground assumption), for comparison
  std::string taskFile;   // overrides config/mpc/task.info
  double baseHeight = -1.0;  // base height above the terrain [m] (< 0: FSM default)
  double walkStart = 3.0; // [s] scripted switch STANDING -> WALKING
  double startY = 0.0;    // [m] initial lateral position
  bool reportHits = false;  // print every leg/body-terrain collision
};

void printUsage() {
  std::cout << "Usage: g1_perceptive_mujoco [scene.xml] [--headless] [--duration S] [--vx V] [--goal-x X]\n"
               "                                [--gait NAME] [--log FILE.csv] [--record FILE.gif|mp4]\n"
               "                                [--width W] [--height H] [--fps F] [--scale W] [--playback X] [--no-map] [--map-stride N]\n"
               "                                [--blind] [--task task.info] [--base-height H] [--walk-start T] [--start-y Y] [--report-hits]\n";
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
    } else if (a == "--goal-x") {
      o.goalX = std::stod(next("--goal-x"));
    } else if (a == "--gait") {
      o.gait = next("--gait");
    } else if (a == "--log") {
      o.logFile = next("--log");
    } else if (a == "--trace") {
      o.traceFile = next("--trace");
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
    } else if (a == "--map-stride") {
      o.mapStride = std::max(1, std::stoi(next("--map-stride")));
    } else if (a == "--playback") {
      o.playback = std::stod(next("--playback"));
    } else if (a == "--no-map") {
      o.showMap = false;
    } else if (a == "--blind") {
      o.blind = true;
    } else if (a == "--base-height") {
      o.baseHeight = std::stod(next("--base-height"));
    } else if (a == "--report-hits") {
      o.reportHits = true;
    } else if (a == "--start-y") {
      o.startY = std::stod(next("--start-y"));
    } else if (a == "--walk-start") {
      o.walkStart = std::stod(next("--walk-start"));
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

/** Robot in the default posture at (0, y), feet 1 cm above the ground. */
void setInitialPose(const mjModel* m, mjData* d, G1Controller& controller, double y = 0.0) {
  mj_resetData(m, d);
  controller.bridge().setPose(d, Eigen::Vector3d(0.0, y, 0.745), 0.0, controller.defaultJointPositions());
  mj_forward(m, d);
}

/**
 * True for a contact between a robot geom that is not a sole sphere and a world (terrain) geom. Contacts
 * closer than 6 cm to a sole sphere of the same body are grazes of the foot itself and are not counted.
 */
bool isBodyTerrainContact(const mjModel* m, const mjData* d, const mjContact& c, int robotRoot, const char** robotBody) {
  for (int k = 0; k < 2; ++k) {
    const int g = c.geom[k];
    const int other = c.geom[1 - k];
    const int body = m->geom_bodyid[g];
    if (m->body_rootid[body] != robotRoot || m->geom_bodyid[other] != 0 || m->geom_type[g] == mjGEOM_SPHERE) {
      continue;
    }
    for (int f = 0; f < m->ngeom; ++f) {  // foot sphere on the same body
      if (m->geom_bodyid[f] == body && m->geom_type[f] == mjGEOM_SPHERE) {
        const double dx = c.pos[0] - d->geom_xpos[3 * f], dy = c.pos[1] - d->geom_xpos[3 * f + 1],
                     dz = c.pos[2] - d->geom_xpos[3 * f + 2];
        if (dx * dx + dy * dy + dz * dz < 0.06 * 0.06) {
          return false;
        }
      }
    }
    *robotBody = mj_id2name(m, mjOBJ_BODY, body);
    return true;
  }
  return false;
}

/** Height of the base above the terrain under it. */
double heightAboveTerrain(const Eigen::Vector3d& base, const std::shared_ptr<const TerrainMap>& map) {
  return map ? base.z() - map->smoothHeightAt(base.head<2>()) : base.z();
}

// =============================================================================================
// Offscreen recorder (hidden GLFW window, ffmpeg pipe)
// =============================================================================================
#ifdef G1_PERCEPTIVE_WITH_GUI
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
    cam_.distance = 3.4;
    cam_.azimuth = 125.0;
    cam_.elevation = -18.0;

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

  void frame(const mjModel* m, mjData* d, const Eigen::Vector3d& base, const PerceptiveReferenceManager::Plan& plan,
             const TerrainVisualizer& visualizer, const std::string& label) {
    // smooth follow camera
    for (int k = 0; k < 3; ++k) {
      const double target = k == 2 ? std::max(base.z() - 0.15, 0.3) : base(k);
      cam_.lookat[k] = initialized_ ? 0.9 * cam_.lookat[k] + 0.1 * target : target;
    }
    initialized_ = true;
    mjv_updateScene(m, d, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    visualizer.append(plan, &scn_);
    const mjrRect viewport = {0, 0, width_, height_};
    mjr_render(viewport, &scn_, &con_);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, "Unitree G1 perceptive NMPC", label.c_str(), &con_);
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
// Headless: deterministic lock-step run (physics waits for every MPC solve and map update)
// =============================================================================================
int runHeadless(const Options& o, mjModel* m, mjData* d, G1Controller& controller, MujocoElevationMapper& mapper) {
  const double dt = m->opt.timestep;
  const double mpcPeriod = 1.0 / controller.mpcFrequency();
  const double mapPeriod = 1.0 / mapper.config().updateRate;
  const double fsmPeriod = 0.01;

  setInitialPose(m, d, controller, o.startY);
  TerrainVisualizer visualizer;
  visualizer.options().showMap = o.showMap;
  visualizer.options().mapStride = o.mapStride;

#ifdef G1_PERCEPTIVE_WITH_GUI
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

  std::ofstream trace;
  const char* contactSites[4] = {"l_foot_toe", "r_foot_toe", "l_foot_heel", "r_foot_heel"};
  if (!o.traceFile.empty()) {
    trace.open(o.traceFile);
    trace << "t,mode,x,y,z,yaw,pitch,roll";
    for (const char* s : contactSites) {
      trace << "," << s << "_x," << s << "_y," << s << "_z";
    }
    for (const char* s : contactSites) {
      trace << ",plan_" << s << "_x,plan_" << s << "_y,plan_" << s << "_z";
    }
    for (const char* s : contactSites) {
      trace << ",pred_" << s << "_x,pred_" << s << "_y,pred_" << s << "_z";
    }
    for (int j = 0; j < m->nu; ++j) {
      trace << ",q_" << mj_id2name(m, mjOBJ_JOINT, m->actuator_trnid[2 * j]);
    }
    for (int j = 0; j < m->nu; ++j) {
      trace << ",tau_" << mj_id2name(m, mjOBJ_JOINT, m->actuator_trnid[2 * j]);
    }
    trace << ",solve_ms\n";
  }
  double nextTrace = 0.0;

  std::ofstream log;
  if (!o.logFile.empty()) {
    log.open(o.logFile);
    log << "t,mode,x,y,z,yaw,pitch,roll,vx,height_above_terrain,terrain_height,solve_ms\n";
  }

  // script
  const double tStand = 1.6;    // PD stand-up -> MPC stance
  const double tWalk = o.walkStart;  // -> walking (gait, zero velocity)
  const double tGo = tWalk + 1.3;    // -> forward velocity
  double tGoal = -1.0;          // time the goal was reached

  double nextMpc = 0.0, nextMap = 0.0, nextFsm = 0.0, nextLog = 0.0, nextPrint = 0.0;
  bool mpcWasActive = false;
  bool fell = false;
  double maxX = 0.0, solveTimeSum = 0.0, solveTimeMax = 0.0, mapTimeSum = 0.0;
  int numSolves = 0, numMaps = 0;
  const int robotRoot = m->body_rootid[mj_name2id(m, mjOBJ_BODY, "pelvis")];
  int bodyCollisionSteps = 0;
  double firstCollisionTime = -1.0;
  std::string firstCollisionBody;
  bool wasColliding = false;
  std::shared_ptr<const TerrainMap> latestMap;  // ground truth for monitoring, also in --blind runs
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
        return 0;
      });
      stage = 4;
    } else if (stage == 4 && o.goalX > 0.0 && base.x() >= o.goalX) {
      controller.withFsm([](FSMController& f) {
        f.getWalkingTargetMutable().vx = 0.0;
        return 0;
      });
      tGoal = t;
      stage = 5;
      std::cout << "[Headless] goal x=" << o.goalX << " reached at t=" << t << " s\n";
    } else if (stage == 5 && t >= tGoal + 1.5) {
      controller.withFsm([](FSMController& f) {
        f.requestStateChange(FSMState::STANDING);
        return 0;
      });
      stage = 6;
    } else if (stage == 6 && t >= tGoal + 3.5) {
      break;
    }

    // operator: keep to the centre line of the course (lateral offset and heading)
    if (o.baseHeight > 0.0) {
      controller.withFsm([&](FSMController& f) {
        f.getTargetMutable().z = o.baseHeight;
        f.getWalkingTargetMutable().z = o.baseHeight;
        return 0;
      });
    }
    if (stage == 4) {
      const Eigen::Vector3d zyxNow = controller.bridge().baseEulerZyx(d);
      controller.withFsm([&](FSMController& f) {
        auto& w = f.getWalkingTargetMutable();
        w.vy = std::clamp(-0.5 * (base.y() - o.startY), -0.15, 0.15);
        w.yaw_rate = std::clamp(-1.0 * zyxNow(0), -0.3, 0.3);
        return 0;
      });
    }

    // ---- perception ----
    if (t >= nextMap - 1e-9) {
      mapper.captureState(d);
      const auto map = mapper.update();
      if (!o.blind) {
        controller.terrainBuffer()->set(map);
      }
      latestMap = map;
      mapTimeSum += mapper.lastUpdateDuration();
      ++numMaps;
      nextMap += mapPeriod;
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

    mj_step(m, d);

    // ---- monitoring ----
    bool colliding = false;
    for (int c = 0; c < d->ncon; ++c) {
      const char* body = nullptr;
      if (isBodyTerrainContact(m, d, d->contact[c], robotRoot, &body)) {
        if (firstCollisionTime < 0.0) {
          firstCollisionTime = d->time;
          firstCollisionBody = body != nullptr ? body : "?";
        }
        if (o.reportHits && !wasColliding) {
          const auto& con = d->contact[c];
          const int robotGeom = m->body_rootid[m->geom_bodyid[con.geom[0]]] == robotRoot ? con.geom[0] : con.geom[1];
          std::cout << "[HIT] t=" << std::setprecision(3) << d->time << " " << (body != nullptr ? body : "?") << " geom#"
                    << robotGeom << " (type " << m->geom_type[robotGeom] << ", size " << m->geom_size[3 * robotGeom] << ")"
                    << " at ("
                    << con.pos[0] << ", " << con.pos[1] << ", " << con.pos[2] << ")\n";
        }
        colliding = true;
        ++bodyCollisionSteps;
        break;
      }
    }
    wasColliding = colliding;
    const Eigen::Vector3d zyx = controller.bridge().baseEulerZyx(d);
    const double clearance = heightAboveTerrain(base, latestMap);
    if (controller.isMpcActive() && (clearance < 0.5 || std::abs(zyx(1)) > 0.6 || std::abs(zyx(2)) > 0.6)) {
      fell = true;
      std::cout << "[Headless] FALL at t=" << t << " base=(" << base.transpose() << ") height above terrain="
                << clearance << " pitch=" << zyx(1) << " roll=" << zyx(2) << "\n";
      break;
    }
    const std::string mode = controller.withFsm([](FSMController& f) { return fsmStateToString(f.getCurrentState()); });
    if (log.is_open() && t >= nextLog - 1e-9) {
      log << std::fixed << std::setprecision(4) << t << "," << mode << "," << base.x() << "," << base.y() << "," << base.z()
          << "," << zyx(0) << "," << zyx(1) << "," << zyx(2) << "," << d->qvel[0] << "," << clearance << ","
          << base.z() - clearance << "," << 1e3 * controller.lastSolveDuration() << "\n";
      nextLog += 0.02;
    }
    if (trace.is_open() && t >= nextTrace - 1e-9) {
      trace << std::fixed << std::setprecision(4) << t << "," << mode << "," << base.x() << "," << base.y() << ","
            << base.z() << "," << zyx(0) << "," << zyx(1) << "," << zyx(2);
      for (const char* s : contactSites) {
        const int id = mj_name2id(m, mjOBJ_SITE, s);
        trace << "," << d->site_xpos[3 * id] << "," << d->site_xpos[3 * id + 1] << "," << d->site_xpos[3 * id + 2];
      }
      const auto plan = controller.referenceManager().getPlan();
      for (size_t c = 0; c < 4; ++c) {
        const Foothold* next = nullptr;  // first planned foothold of the contact in the horizon
        for (const auto& [leg, f] : plan.footholds) {
          if (leg == c) {
            next = &f;
            break;
          }
        }
        if (next != nullptr) {
          trace << "," << next->position.x() << "," << next->position.y() << "," << next->position.z();
        } else {
          trace << ",nan,nan,nan";
        }
      }
      const auto predicted = controller.predictedContactPositions(t + 0.2);
      for (size_t c = 0; c < 4; ++c) {
        if (c < predicted.size()) {
          trace << "," << predicted[c].x() << "," << predicted[c].y() << "," << predicted[c].z();
        } else {
          trace << ",nan,nan,nan";
        }
      }
      for (int j = 0; j < m->nu; ++j) {
        trace << "," << d->qpos[m->jnt_qposadr[m->actuator_trnid[2 * j]]];
      }
      for (int j = 0; j < m->nu; ++j) {
        trace << "," << d->ctrl[j];
      }
      trace << "," << 1e3 * controller.lastSolveDuration() << "\n";
      nextTrace += 0.01;
    }
    if (t >= nextPrint - 1e-9) {
      const auto plan = controller.referenceManager().getPlan();
      const double zRef = plan.baseTrajectory.empty() ? 0.0 : plan.baseTrajectory.front().z();
      std::cout << std::fixed << std::setprecision(2) << "[t=" << std::setw(5) << t << "] " << std::setw(10) << mode
                << " x=" << std::setw(5) << base.x() << " y=" << std::setw(5) << base.y() << " z=" << base.z()
                << " h=" << clearance << " zref=" << zRef << " pitch=" << std::setw(5) << zyx(1) << " vx=" << std::setw(5)
                << d->qvel[0]
                << " solve=" << std::setprecision(1) << 1e3 * controller.lastSolveDuration() << "ms"
                << " map=" << 1e3 * mapper.lastUpdateDuration() << "ms\n";
      nextPrint += 1.0;
    }
#ifdef G1_PERCEPTIVE_WITH_GUI
    if (recording && d->time >= nextFrame) {
      char label[256];
      std::snprintf(label, sizeof(label), "t=%.1fs  %s  v=%.2f m/s", d->time, mode.c_str(), d->qvel[0]);
      recorder.frame(m, d, controller.bridge().basePosition(d), controller.referenceManager().getPlan(), visualizer, label);
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
            << "  map updates      : " << numMaps << "  avg " << (numMaps ? 1e3 * mapTimeSum / numMaps : 0.0) << " ms\n"
            << "  leg/body hits    : " << bodyCollisionSteps * dt << " s of contact"
            << (firstCollisionTime >= 0.0 ? " (first: " + firstCollisionBody + " at t=" + std::to_string(firstCollisionTime) + ")" : "")
            << "\n"
            << "  fell             : " << (fell ? "YES" : "no") << "\n"
            << "  result           : " << (!fell && reached ? "SUCCESS" : "FAILURE") << "\n"
            << "=================================================\n";

#ifdef G1_PERCEPTIVE_WITH_GUI
  if (recording && recorder.close(o.fps, o.record) != 0) {
    return 1;
  }
#endif
  return !fell && reached ? 0 : 2;
}

// =============================================================================================
// Interactive GUI
// =============================================================================================
#ifdef G1_PERCEPTIVE_WITH_GUI
std::atomic<bool> g_exit{false};
std::atomic<bool> g_showMap{true};
std::atomic<bool> g_blind{false};

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
      if (keyboard.wasMapTogglePressed()) {
        g_showMap = !g_showMap;
        std::cout << "[GUI] elevation map display " << (g_showMap ? "ON" : "OFF") << std::endl;
      }
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

void mappingThread(G1Controller& controller, MujocoElevationMapper& mapper, mujoco::Simulate* sim) {
  const auto period = std::chrono::duration<double>(1.0 / mapper.config().updateRate);
  auto next = std::chrono::steady_clock::now();
  while (!g_exit) {
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    bool ready = false;
    {
      std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
      if (sim->m_ != nullptr && sim->d_ != nullptr) {
        mapper.captureState(sim->d_);
        ready = true;
      }
    }
    if (ready) {
      auto map = mapper.update();
      if (!g_blind) {
        controller.terrainBuffer()->set(std::move(map));
      }
    }
    std::this_thread::sleep_until(next);
    if (std::chrono::steady_clock::now() > next + std::chrono::milliseconds(200)) {
      next = std::chrono::steady_clock::now();
    }
  }
}

void physicsThread(mujoco::Simulate* sim, mjModel* m, mjData* d, G1Controller& controller,
                   const std::string& scenePath) {
  sim->Load(m, d, scenePath.c_str());

  mjvScene userScene;
  mjv_defaultScene(&userScene);
  mjv_makeScene(m, &userScene, 12000);
  TerrainVisualizer visualizer;
  {
    std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
    sim->user_scn = &userScene;
  }

  using Clock = std::chrono::steady_clock;
  auto syncCpu = Clock::now();
  mjtNum syncSim = 0;
  auto lastVis = Clock::now();
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

    // decorations: elevation map, footholds, base reference
    if (Clock::now() - lastVis > std::chrono::milliseconds(50)) {
      lastVis = Clock::now();
      visualizer.options().showMap = g_showMap;
      visualizer.options().showFootholds = g_showMap;
      visualizer.options().showBaseTrajectory = g_showMap;
      visualizer.draw(controller.referenceManager().getPlan(), &userScene);
    }

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

  {
    std::lock_guard<mujoco::SimulateMutex> lock(sim->mtx);
    sim->user_scn = nullptr;
  }
  mjv_freeScene(&userScene);
}

int runGui(const Options& o, mjModel* m, mjData* d, G1Controller& controller, MujocoElevationMapper& mapper) {
  setInitialPose(m, d, controller);

  mjvCamera cam;
  mjvOption opt;
  mjvPerturb pert;
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultPerturb(&pert);
  cam.type = mjCAMERA_TRACKING;
  cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "pelvis");
  cam.distance = 3.2;
  cam.azimuth = 125;
  cam.elevation = -22;
  opt.geomgroup[3] = 0;
  g_showMap = o.showMap;
  g_blind = o.blind;

  auto sim = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(), &cam, &opt, &pert, false);

  std::cout << "\n========================================\n"
            << "  Keyboard (terminal) / Xbox controls\n"
            << "    1 / A   PASSIVE\n"
            << "    2 / B   PD stand-up\n"
            << "    3 / X   STANDING (MPC)\n"
            << "    4 / Y   WALKING  (MPC, walk gait)\n"
            << "    W/S A/D forward / lateral velocity, J/L yaw rate\n"
            << "    G / RB  next gait,  -/= height,  M map display\n"
            << "  MuJoCo window: Space pause, Backspace reset, Esc quit\n"
            << "========================================\n";

  std::thread physics(physicsThread, sim.get(), m, d, std::ref(controller), o.scene);
  std::thread mpc(mpcThread, std::ref(controller), sim.get());
  std::thread mapping(mappingThread, std::ref(controller), std::ref(mapper), sim.get());
  std::thread input(inputThread, std::ref(controller));

  sim->RenderLoop();

  g_exit = true;
  sim->exitrequest.store(1);
  physics.join();
  mpc.join();
  mapping.join();
  input.join();
  return 0;
}
#endif

}  // namespace

int main(int argc, char** argv) {
  Options o;
  const std::string pkg = getPackagePath();
  o.scene = pkg + "/robots/g1/scene_terrain.xml";
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
            << "  Unitree G1 (12 DoF) perceptive NMPC (OCS2 + MuJoCo, no ROS)\n"
            << "  scene: " << o.scene << "\n"
            << "  task : " << paths.taskFile << (o.blind ? "\n  BLIND: the MPC assumes flat ground" : "") << "\n"
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
    MujocoElevationMapper mapper(m, loadElevationMapperConfig(paths.taskFile, "elevation_mapper", false));
    if (o.headless) {
      status = runHeadless(o, m, d, controller, mapper);
    } else {
#ifdef G1_PERCEPTIVE_WITH_GUI
      status = runGui(o, m, d, controller, mapper);
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
