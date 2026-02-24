#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <iomanip>
#include <atomic>
#include <mutex>

#include "ocs2_cartpole/CartpoleUDP.h"
#include "ocs2_cartpole/CartPoleInterface.h"

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/integration/SystemEventHandler.h>
#include <ocs2_mpc/SystemObservation.h>

using namespace ocs2_cartpole;

constexpr float PI = 3.14159265359f;

/**
 * Normalize angle to [-π, π]
 */
float normalizeAngle(float angle) {
  while (angle > PI) {
    angle -= 2.0f * PI;
  }
  while (angle < -PI) {
    angle += 2.0f * PI;
  }
  return angle;
}

// Global variables for MRT-MPC communication
std::atomic<bool> mpcRunning{true};
std::atomic<bool> mpcUpdated{false};
std::mutex policyMutex;
ocs2::PrimalSolution latestPolicy;
ocs2::SystemObservation latestObservation;

// Global pointers for MPC thread access
ocs2::GaussNewtonDDP_MPC* globalMpcPtr = nullptr;
const ocs2::vector_t* globalFinalGoal = nullptr;
const ocs2::mpc::Settings* globalMpcSettings = nullptr;

/**
 * MPC Thread: Runs at 10Hz, solves optimization
 */
void mpcThread() {
  const double mpcPeriod = 0.1;  // 10Hz
  
  while (mpcRunning) {
    auto loopStart = std::chrono::steady_clock::now();
    
    // Get current observation
    ocs2::SystemObservation obs;
    {
      std::lock_guard<std::mutex> lock(policyMutex);
      obs = latestObservation;
    }
    
    // Set target trajectory
    ocs2::TargetTrajectories targetTrajectories;
    const double horizonEnd = obs.time + globalMpcSettings->timeHorizon_;
    targetTrajectories.timeTrajectory = {obs.time, horizonEnd};
    targetTrajectories.stateTrajectory = {*globalFinalGoal, *globalFinalGoal};
    targetTrajectories.inputTrajectory = {
        ocs2::vector_t::Zero(INPUT_DIM),
        ocs2::vector_t::Zero(INPUT_DIM)
    };
    globalMpcPtr->getSolverPtr()->getReferenceManager().setTargetTrajectories(targetTrajectories);
    
    try {
      // Run MPC optimization
      bool success = globalMpcPtr->run(obs.time, obs.state);
      
      if (success) {
        // Get policy and update
        ocs2::PrimalSolution policy;
        const double queryTime = obs.time;
        globalMpcPtr->getSolverPtr()->getPrimalSolution(queryTime, &policy);
        
        {
          std::lock_guard<std::mutex> lock(policyMutex);
          latestPolicy = policy;
          mpcUpdated = true;
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "[MPC] Optimization failed: " << e.what() << std::endl;
    }
    
    // Sleep to maintain 10Hz
    auto loopEnd = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(loopEnd - loopStart).count();
    double sleepTime = mpcPeriod - elapsed;
    if (sleepTime > 0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(sleepTime));
    }
  }
}

/**
 * Interpolate control from policy at given time
 */
float interpolateControl(const ocs2::PrimalSolution& policy, double time) {
  if (policy.timeTrajectory_.empty() || policy.inputTrajectory_.empty()) {
    return 0.0f;
  }
  
  // Find time interval
  auto it = std::lower_bound(policy.timeTrajectory_.begin(), 
                             policy.timeTrajectory_.end(), time);
  
  if (it == policy.timeTrajectory_.begin()) {
    return static_cast<float>(policy.inputTrajectory_.front()(0));
  }
  
  if (it == policy.timeTrajectory_.end()) {
    return static_cast<float>(policy.inputTrajectory_.back()(0));
  }
  
  // Linear interpolation
  size_t idx = std::distance(policy.timeTrajectory_.begin(), it);
  double t1 = policy.timeTrajectory_[idx - 1];
  double t2 = policy.timeTrajectory_[idx];
  double u1 = policy.inputTrajectory_[idx - 1](0);
  double u2 = policy.inputTrajectory_[idx](0);
  
  double alpha = (time - t1) / (t2 - t1);
  return static_cast<float>(u1 + alpha * (u2 - u1));
}

int main(int argc, char** argv) {
  std::cout << "=" << std::string(58, '=') << "=" << std::endl;
  std::cout << "CartPole Upright Stabilization - NMPC via UDP" << std::endl;
  std::cout << "=" << std::string(58, '=') << "=" << std::endl;

  // Get config file path and library folder
  // Default to source directory config if running from different location
  std::string config_file = "src/ocs2_cartpole_upright/config/mpc/task.info";
  std::string library_folder = "/tmp/ocs2_cartpole_upright";
  
  if (argc > 1) {
    config_file = argv[1];
  }
  if (argc > 2) {
    library_folder = argv[2];
  }

  // Initialize UDP interface
  CartPoleUDPInterface udp;
  if (!udp.initialize()) {
    std::cerr << "[ERROR] Failed to initialize UDP interface" << std::endl;
    return 1;
  }

  // Build CartPole Interface
  std::cout << "\n[*] Building NMPC solver..." << std::endl;
  std::unique_ptr<CartPoleInterface> cartPoleInterfacePtr;
  try {
    cartPoleInterfacePtr =
        std::make_unique<CartPoleInterface>(config_file, library_folder, true);
  } catch (const std::exception& e) {
    std::cerr << "[ERROR] Failed to build NMPC: " << e.what() << std::endl;
    return 1;
  }

  // Create GaussNewtonDDP_MPC (like ROS example)
  std::cout << "[*] Creating GaussNewtonDDP_MPC solver..." << std::endl;
  auto mpcPtr = std::make_unique<ocs2::GaussNewtonDDP_MPC>(
      cartPoleInterfacePtr->mpcSettings(),
      cartPoleInterfacePtr->ddpSettings(),
      cartPoleInterfacePtr->getRollout(),
      cartPoleInterfacePtr->getOptimalControlProblem(),
      cartPoleInterfacePtr->getInitializer());

  // Get initial state and target
  const ocs2::vector_t initialState = cartPoleInterfacePtr->getInitialState();
  const ocs2::vector_t finalGoal = cartPoleInterfacePtr->getInitialTarget();
  
  std::cout << "\n[*] Initial state: [" << initialState.transpose() << "]" << std::endl;
  std::cout << "[*] Target state:  [" << finalGoal.transpose() << "]" << std::endl;

  std::cout << "\n[*] Starting MRT-MPC control loop..." << std::endl;
  std::cout << "    MPC frequency: 10 Hz (optimization every 100ms)" << std::endl;
  std::cout << "    MRT frequency: 100 Hz (control @ 10ms with interpolation)" << std::endl;
  std::cout << "    Target: pole angle = 0 (upright), cart position = 0 (centered)"
            << std::endl;
  std::cout << "    Press Ctrl+C to stop\n" << std::endl;

  const auto& mpcSettings = cartPoleInterfacePtr->mpcSettings();
  const double max_force =
      static_cast<double>(cartPoleInterfacePtr->getParameters().maxInput_);

  // Initialize observation
  latestObservation.state.resize(STATE_DIM);
  latestObservation.input.resize(INPUT_DIM);
  latestObservation.input.setZero();
  latestObservation.time = 0.0;
  
  // Initialize shared policy with upright goal
  latestPolicy.timeTrajectory_ = {0.0};
  latestPolicy.stateTrajectory_ = {finalGoal};
  latestPolicy.inputTrajectory_ = {ocs2::vector_t::Zero(INPUT_DIM)};

  // Reset MPC
  mpcPtr->reset();
  
  // Set initial target
  ocs2::TargetTrajectories initTargetTrajectories;
  initTargetTrajectories.timeTrajectory.push_back(0.0);
  initTargetTrajectories.stateTrajectory.push_back(finalGoal);
  initTargetTrajectories.inputTrajectory.push_back(ocs2::vector_t::Zero(INPUT_DIM));
  mpcPtr->getSolverPtr()->getReferenceManager().setTargetTrajectories(initTargetTrajectories);

  // Set global pointers for MPC thread
  globalMpcPtr = mpcPtr.get();
  globalFinalGoal = &finalGoal;
  globalMpcSettings = &mpcSettings;

  // Start MPC thread
  std::thread mpcThreadHandle(mpcThread);
  
  std::cout << "[*] MPC thread started at 10Hz" << std::endl;
  std::cout << "[*] MRT loop starting at 100Hz..." << std::endl;

  uint32_t step = 0;
  ocs2::vector_t currentState(STATE_DIM);
  const double mrtDt = 0.01;  // 100Hz control rate

  try {
    while (true) {
      CartPoleState state;

      // Receive state from simulator
      if (!udp.receiveState(state, 1000)) {
        if (step == 0) {
          std::cout << "[WARN] Waiting for simulator data..." << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      step++;

      // Convert to OCS2 state vector [theta, x, theta_dot, x_dot]
      // MuJoCo: theta=0 is upright, same as OCS2 convention
      // NO negation needed - direct mapping
      currentState << static_cast<double>(state.pole_angle),
          static_cast<double>(state.cart_pos),
          static_cast<double>(state.pole_anglevel),
          static_cast<double>(state.cart_vel);

      // Update observation for MPC thread
      {
        std::lock_guard<std::mutex> lock(policyMutex);
        latestObservation.time = static_cast<double>(state.timestamp);
        latestObservation.state = currentState;
      }

      // Interpolate control from latest policy
      float optimal_force = 0.0f;
      {
        std::lock_guard<std::mutex> lock(policyMutex);
        if (mpcUpdated) {
          optimal_force = interpolateControl(latestPolicy, latestObservation.time);
        }
      }
      
      // Clamp to actuator limits
      optimal_force = std::max(-static_cast<float>(max_force),
                               std::min(static_cast<float>(max_force), optimal_force));

      // Send command
      CartPoleCommand cmd(optimal_force, state.timestamp);
      if (!udp.sendCommand(cmd)) {
        std::cerr << "[ERROR] Failed to send command" << std::endl;
        break;
      }

      // Print status every 10 steps (~0.1s)
      if (step % 10 == 0) {
        std::cout << std::fixed << std::setprecision(3) << "[" << step
                  << "] t: " << latestObservation.time
                  << " | state: [" << currentState(0) << ", " << currentState(1) 
                  << ", " << currentState(2) << ", " << currentState(3) << "]"
                  << " | F: " << optimal_force << " N"
                  << " | MPC: " << (mpcUpdated ? "OK" : "WAIT") << std::endl;
      }

      // MRT control rate: 100Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } catch (const std::exception& e) {
    std::cerr << "\n[ERROR] " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "\n[ERROR] Unknown exception occurred" << std::endl;
  }

  // Stop MPC thread
  mpcRunning = false;
  if (mpcThreadHandle.joinable()) {
    mpcThreadHandle.join();
  }

  udp.close();
  std::cout << "\n[*] Control loop stopped" << std::endl;
  std::cout << "[*] Total states received: " << udp.getStateCount() << std::endl;
  std::cout << "=" << std::string(58, '=') << "=" << std::endl;

  return 0;
}