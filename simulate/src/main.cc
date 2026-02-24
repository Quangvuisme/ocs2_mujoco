// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <filesystem>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include <mujoco/mujoco.h>
#include "mujoco/glfw_adapter.h"
#include "mujoco/simulate.h"
#include "mujoco/array_safety.h"
#include "cartpole_bridge/cartpole_bridge.h"
#include "quadrotor_bridge/quadrotor_bridge.h"
#include "quadruped_bridge/quadruped_bridge.h"
#include "ros/CameraNode.hpp"
#include <pthread.h>
#include "yaml-cpp/yaml.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C"
{
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

namespace
{
  namespace mj = ::mujoco;
  namespace mju = ::mujoco::sample_util;

  // constants
  const double syncMisalign = 0.1;       // maximum mis-alignment before re-sync (simulation seconds)
  const double simRefreshFraction = 0.7; // fraction of refresh available for simulation
  const int kErrorLength = 1024;         // load error string length

  // model and data
  mjModel *m = nullptr;
  mjData *d = nullptr;

  // control noise variables
  mjtNum *ctrlnoise = nullptr;

  struct SimulationConfig
  {
    std::string robot = "go2";
    std::string robot_scene = "scene.xml";

    int domain_id = 1;
    std::string interface = "lo";

    int print_scene_information = 1;

  } config;

  using Seconds = std::chrono::duration<double>;

  //---------------------------------------- plugin handling -----------------------------------------

  // return the path to the directory containing the current executable
  // used to determine the location of auto-loaded plugin libraries
  std::string getExecutableDir()
  {
#if defined(_WIN32) || defined(__CYGWIN__)
    constexpr char kPathSep = '\\';
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      DWORD buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
        if (written < buf_size)
        {
          success = true;
        }
        else if (written == buf_size)
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
        else
        {
          std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
          return "";
        }
      }
      return realpath.get();
    }();
#else
    constexpr char kPathSep = '/';
#if defined(__APPLE__)
    std::unique_ptr<char[]> buf(nullptr);
    {
      std::uint32_t buf_size = 0;
      _NSGetExecutablePath(nullptr, &buf_size);
      buf.reset(new char[buf_size]);
      if (!buf)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }
      if (_NSGetExecutablePath(buf.get(), &buf_size))
      {
        std::cerr << "unexpected error from _NSGetExecutablePath\n";
      }
    }
    const char *path = buf.get();
#else
    const char *path = "/proc/self/exe";
#endif
    std::string realpath = [&]() -> std::string
    {
      std::unique_ptr<char[]> realpath(nullptr);
      std::uint32_t buf_size = 128;
      bool success = false;
      while (!success)
      {
        realpath.reset(new (std::nothrow) char[buf_size]);
        if (!realpath)
        {
          std::cerr << "cannot allocate memory to store executable path\n";
          return "";
        }

        std::size_t written = readlink(path, realpath.get(), buf_size);
        if (written < buf_size)
        {
          realpath.get()[written] = '\0';
          success = true;
        }
        else if (written == -1)
        {
          if (errno == EINVAL)
          {
            // path is already not a symlink, just use it
            return path;
          }

          std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
          return "";
        }
        else
        {
          // realpath is too small, grow and retry
          buf_size *= 2;
        }
      }
      return realpath.get();
    }();
#endif

    if (realpath.empty())
    {
      return "";
    }

    for (std::size_t i = realpath.size() - 1; i > 0; --i)
    {
      if (realpath.c_str()[i] == kPathSep)
      {
        return realpath.substr(0, i);
      }
    }

    // don't scan through the entire file system's root
    return "";
  }

  // scan for libraries in the plugin directory to load additional plugins
  void scanPluginLibraries()
  {
    // check and print plugins that are linked directly into the executable
    int nplugin = mjp_pluginCount();
    if (nplugin)
    {
      std::printf("Built-in plugins:\n");
      for (int i = 0; i < nplugin; ++i)
      {
        std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
      }
    }

    // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
    const std::string sep = "\\";
#else
    const std::string sep = "/";
#endif

    // try to open the ${EXECDIR}/plugin directory
    // ${EXECDIR} is the directory containing the simulate binary itself
    const std::string executable_dir = getExecutableDir();
    if (executable_dir.empty())
    {
      return;
    }

    const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(), +[](const char *filename, int first, int count)
                            {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        } });
  }

  //------------------------------------------- simulation -------------------------------------------

  mjModel *LoadModel(const char *file, mj::Simulate &sim)
  {
    // this copy is needed so that the mju::strlen call below compiles
    char filename[mj::Simulate::kMaxFilenameLength];
    mju::strcpy_arr(filename, file);

    // make sure filename is not empty
    if (!filename[0])
    {
      return nullptr;
    }

    // load and compile
    char loadError[kErrorLength] = "";
    mjModel *mnew = 0;
    if (mju::strlen_arr(filename) > 4 &&
        !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                      mju::sizeof_arr(filename) - mju::strlen_arr(filename) + 4))
    {
      mnew = mj_loadModel(filename, nullptr);
      if (!mnew)
      {
        mju::strcpy_arr(loadError, "could not load binary model");
      }
    }
    else
    {
      mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
      // remove trailing newline character from loadError
      if (loadError[0])
      {
        int error_length = mju::strlen_arr(loadError);
        if (loadError[error_length - 1] == '\n')
        {
          loadError[error_length - 1] = '\0';
        }
      }
    }

    mju::strcpy_arr(sim.load_error, loadError);

    if (!mnew)
    {
      std::printf("%s\n", loadError);
      return nullptr;
    }

    // compiler warning: print and pause
    if (loadError[0])
    {
      // mj_forward() below will print the warning message
      std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
      sim.run = 0;
    }

    return mnew;
  }

  // simulate in background thread (while rendering in main thread)
  void PhysicsLoop(mj::Simulate &sim)
  {
    // cpu-sim syncronization point
    std::chrono::time_point<mj::Simulate::Clock> syncCPU;
    mjtNum syncSim = 0;

    // ChannelFactory::Instance()->Init(0);
    // UnitreeDds ud(d);

    // run until asked to exit
    while (!sim.exitrequest.load())
    {
      if (sim.droploadrequest.load())
      {
        sim.LoadMessage(sim.dropfilename);
        mjModel *mnew = LoadModel(sim.dropfilename, sim);
        sim.droploadrequest.store(false);

        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.dropfilename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      if (sim.uiloadrequest.load())
      {
        sim.uiloadrequest.fetch_sub(1);
        sim.LoadMessage(sim.filename);
        mjModel *mnew = LoadModel(sim.filename, sim);
        mjData *dnew = nullptr;
        if (mnew)
          dnew = mj_makeData(mnew);
        if (dnew)
        {
          sim.Load(mnew, dnew, sim.filename);

          mj_deleteData(d);
          mj_deleteModel(m);

          m = mnew;
          d = dnew;
          mj_forward(m, d);

          // allocate ctrlnoise
          free(ctrlnoise);
          ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
          mju_zero(ctrlnoise, m->nu);
        }
        else
        {
          sim.LoadMessageClear();
        }
      }

      // sleep for 1 ms or yield, to let main thread run
      //  yield results in busy wait - which has better timing but kills battery life
      if (sim.run && sim.busywait)
      {
        std::this_thread::yield();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      {
        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        // run only if model is present
        if (m)
        {
          // running
          if (sim.run)
          {
            bool stepped = false;

            // record cpu time at start of iteration
            const auto startCPU = mj::Simulate::Clock::now();

            // elapsed CPU and simulation time since last sync
            const auto elapsedCPU = startCPU - syncCPU;
            double elapsedSim = d->time - syncSim;

            // inject noise
            if (sim.ctrl_noise_std)
            {
              // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
              mjtNum rate = mju_exp(-m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
              mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

              for (int i = 0; i < m->nu; i++)
              {
                // update noise
                ctrlnoise[i] = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

                // apply noise
                d->ctrl[i] = ctrlnoise[i];
              }
            }

            // requested slow-down factor
            double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

            // misalignment condition: distance from target sim time is bigger than syncmisalign
            bool misaligned =
                mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim) > syncMisalign;

            // out-of-sync (for any reason): reset sync times, step
            if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
                misaligned || sim.speed_changed)
            {
              // re-sync
              syncCPU = startCPU;
              syncSim = d->time;
              sim.speed_changed = false;

              // run single step, let next iteration deal with timing
              mj_step(m, d);
              stepped = true;
            }

            // in-sync: step until ahead of cpu
            else
            {
              bool measured = false;
              mjtNum prevSim = d->time;

              double refreshTime = simRefreshFraction / sim.refresh_rate;

              // step while sim lags behind cpu and within refreshTime
              while (Seconds((d->time - syncSim) * slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                     mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime))
              {
                // measure slowdown before first step
                if (!measured && elapsedSim)
                {
                  sim.measured_slowdown =
                      std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                  measured = true;
                }

                // call mj_step
                mj_step(m, d);
                stepped = true;

                // break if reset
                if (d->time < prevSim)
                {
                  break;
                }
              }
            }

            // save current state to history buffer
            if (stepped)
            {
              sim.AddToHistory();
            }
          }

          // paused
          else
          {
            // run mj_forward, to update rendering and joint sliders
            mj_forward(m, d);
            sim.speed_changed = true;
          }
        }
      } // release std::lock_guard<std::mutex>
    }
  }
} // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename)
{
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr)
  {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m)
      d = mj_makeData(m);
    if (d)
    {
      sim->Load(m, d, filename);
      mj_forward(m, d);

      // allocate ctrlnoise
      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      mju_zero(ctrlnoise, m->nu);
    }
    else
    {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);

  exit(0);
}

void *Mujoco2RosBridgeThread(void *arg){

  std::cout << "Loading up RosBridge " << std::endl;

  // Wait for mujoco data
  while (1)
  {
    if (d)
    {
      std::cout << "Mujoco data is loaded. " << std::endl;
      break;
    }
    usleep(500000);
  }

  #ifdef USE_ROS2
  std::cout << "Initializing camera Node ..." << std::endl;
  auto ros_interface = std::make_shared<CameraNode>();
  ros_interface->SetMujocoData(d,m);
  ((mj::Simulate*) arg)->setRosNode(ros_interface);
  ros_interface->Run();
  #endif

  pthread_exit(NULL);
}



void *cartpoleBridgeThread(void *arg)
{
  // Wait for Mujoco data initialization
  while (true)
  {
    if (d)
    {
      std::cout << "CartPole: Mujoco data is prepared." << std::endl;
      break;
    }
    usleep(500000);
  }

  CartPoleBridge cartpole_bridge;
  
  if (!cartpole_bridge.init(m, d))
  {
    std::cerr << "CartPole: Failed to initialize UDP bridge" << std::endl;
    pthread_exit(NULL);
  }

  std::cout << "CartPole: UDP bridge running with dedicated threads (500 Hz)" << std::endl;

  // Just apply commands in main loop, threads handle send/receive
  while (true)
  {
    cartpole_bridge.apply_command(d);
    usleep(1000); // 1ms - just for applying commands
  }

  pthread_exit(NULL);
}

void *quadrotorBridgeThread(void *arg)
{
  // Wait for Mujoco data initialization
  while (true)
  {
    if (d)
    {
      std::cout << "Quadrotor: Mujoco data is prepared." << std::endl;
      break;
    }
    usleep(500000);
  }

  QuadrotorBridge quadrotor_bridge;
  
  if (!quadrotor_bridge.Initialize(m, d))
  {
    std::cerr << "Quadrotor: Failed to initialize UDP bridge" << std::endl;
    pthread_exit(NULL);
  }

  std::cout << "Quadrotor: UDP bridge running with dedicated threads (500 Hz)" << std::endl;

  // Bridge runs in separate threads
  while (true)
  {
    usleep(10000); // 10ms - just keep thread alive
  }

  pthread_exit(NULL);
}

void *quadrupedBridgeThread(void *arg)
{
  // Wait for Mujoco data initialization
  while (true)
  {
    if (d)
    {
      std::cout << "Quadruped: Mujoco data is prepared." << std::endl;
      break;
    }
    usleep(500000);
  }

  QuadrupedBridge quadruped_bridge;
  
  if (!quadruped_bridge.Initialize(m, d))
  {
    std::cerr << "Quadruped: Failed to initialize UDP bridge" << std::endl;
    pthread_exit(NULL);
  }

  std::cout << "Quadruped: UDP bridge running with dedicated threads (500 Hz)" << std::endl;

  // Bridge runs in separate threads
  while (true)
  {
    usleep(10000); // 10ms - just keep thread alive
  }

  pthread_exit(NULL);
}

//------------------------------------------ main --------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void _mj_rosettaError(const char *msg)
{
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char **argv)
{
  if (argc < 2) {
      std::cerr << "Error: Missing arguments.\n";
      std::cerr << "Usage: ./mujoco_ocs <robot>\n";
      std::cerr << "Supported robots: anymal_c, go1, go2, cartpole, cartpole_down, quadrotor\n";
      return 1;
  }

  std::string versionArg = argv[1];
  std::string robotScene = "scene_" + versionArg + "_real.xml";

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg)
  {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version())
  {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false);

  // Load simulation configuration
  YAML::Node yaml_node = YAML::LoadFile("../config.yaml");
  config.robot = versionArg;
  config.robot_scene = robotScene;
  config.domain_id = yaml_node["domain_id"].as<int>();
  config.interface = yaml_node["interface"].as<std::string>();
  config.print_scene_information = yaml_node["print_scene_information"].as<int>();

  yaml_node.~Node();

  // Resolve absolute path to scene file
  std::filesystem::path exe_path = std::filesystem::canonical(argv[0]);
  // go three levels up: build → simulate → project root
  std::filesystem::path proj_dir = exe_path.parent_path().parent_path().parent_path();

  std::filesystem::path scene_path;
  if (std::filesystem::path(config.robot_scene).is_relative()) {
      scene_path = proj_dir / "robots" / config.robot / config.robot_scene;
  } else {
      scene_path = std::filesystem::path(config.robot_scene);  // already absolute
  }

  // Optional: print resolved path
  std::cout << "[INFO] Loading scene file: " << scene_path << std::endl;

  // Choose file to load
  const char* filename = scene_path.c_str();

  // Start appropriate bridge thread based on robot type
  pthread_t bridge_thread;
  int rc;
  
  if (config.robot == "cartpole" || config.robot == "cartpole_down")
  {
    // CartPole UDP bridge
    rc = pthread_create(&bridge_thread, NULL, cartpoleBridgeThread, NULL);
    if (rc != 0)
    {
      std::cout << "Error: unable to create CartPole UDP thread, " << rc << std::endl;
      exit(-1);
    }
    std::cout << "CartPole bridge thread started" << std::endl;
  }
  else if (config.robot == "quadrotor")
  {
    // Quadrotor UDP bridge
    rc = pthread_create(&bridge_thread, NULL, quadrotorBridgeThread, NULL);
    if (rc != 0)
    {
      std::cout << "Error: unable to create Quadrotor UDP thread, " << rc << std::endl;
      exit(-1);
    }
    std::cout << "Quadrotor bridge thread started" << std::endl;
  }
  else if (config.robot == "anymal" || config.robot == "anymal_c" || config.robot == "go1" || config.robot == "go2" || config.robot == "quadruped")
  {
    // Quadruped UDP bridge for ANYmal, Go1, Go2, etc.
    rc = pthread_create(&bridge_thread, NULL, quadrupedBridgeThread, NULL);
    if (rc != 0)
    {
      std::cout << "Error: unable to create Quadruped UDP thread, " << rc << std::endl;
      exit(-1);
    }
    std::cout << "Quadruped bridge thread started" << std::endl;
  }
  else
  {
    std::cerr << "Error: Unsupported robot type '" << config.robot << "'" << std::endl;
    std::cerr << "Supported robots: cartpole, cartpole_down, quadrotor, anymal, anymal_c, go1, go2, quadruped" << std::endl;
    exit(-1);
  }

  pthread_t ros_thread;
  rc = pthread_create(&ros_thread, NULL, Mujoco2RosBridgeThread, sim.get());

  if (rc != 0)
  {
    std::cout << "Error:unable to create Ros Bridge thread," << rc << std::endl;
    exit(-1);
  }

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);
  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  pthread_exit(NULL);
  return 0;
}
