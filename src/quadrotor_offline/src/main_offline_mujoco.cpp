/**
 * @file main_offline_mujoco.cpp
 * @brief Offline quadrotor trajectory optimization with MuJoCo visualization
 * 
 * This program:
 * 1. Loads quadrotor parameters and computes optimal trajectory using OCS2 (iLQR)
 * 2. Visualizes the pre-computed trajectory in MuJoCo (no real-time control)
 * 
 * The trajectory is computed offline and then replayed in the MuJoCo visualizer.
 */

#include <iostream>
#include <memory>
#include <string>
#include <iomanip>
#include <fstream>
#include <thread>
#include <chrono>
#include <cmath>

#include <ocs2_ddp/ILQR.h>
#include <ocs2_ddp/SLQ.h>
#include <quadrotor_offline/QuadrotorInterface.h>
#include <quadrotor_offline/package_path.h>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

using namespace ocs2;
using namespace quadrotor_offline;

// ============================================================================
// MuJoCo Global Variables
// ============================================================================
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

GLFWwindow* window = nullptr;

// Mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// Animation control
bool paused = false;
bool loop_animation = true;
double playback_speed = 1.0;

// ============================================================================
// GLFW Callbacks
// ============================================================================
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            case GLFW_KEY_SPACE:
                paused = !paused;
                std::cout << (paused ? "[PAUSED]" : "[PLAYING]") << std::endl;
                break;
            case GLFW_KEY_L:
                loop_animation = !loop_animation;
                std::cout << "Loop: " << (loop_animation ? "ON" : "OFF") << std::endl;
                break;
            case GLFW_KEY_UP:
                playback_speed = std::min(4.0, playback_speed * 1.5);
                std::cout << "Speed: " << playback_speed << "x" << std::endl;
                break;
            case GLFW_KEY_DOWN:
                playback_speed = std::max(0.1, playback_speed / 1.5);
                std::cout << "Speed: " << playback_speed << "x" << std::endl;
                break;
            case GLFW_KEY_R:
                // Reset view
                cam.azimuth = 135;
                cam.elevation = -30;
                cam.distance = 8.0;
                cam.lookat[0] = 1.5;
                cam.lookat[1] = 1.5;
                cam.lookat[2] = 2.5;
                std::cout << "[VIEW RESET]" << std::endl;
                break;
        }
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// ============================================================================
// MuJoCo Functions
// ============================================================================
bool initMuJoCo(const std::string& modelPath) {
    char error[1000] = "Could not load model";
    m = mj_loadXML(modelPath.c_str(), 0, error, 1000);
    if (!m) {
        std::cerr << "Load model error: " << error << std::endl;
        return false;
    }
    
    std::cout << "Model loaded successfully!" << std::endl;
    std::cout << "  Bodies: " << m->nbody << std::endl;
    std::cout << "  Geoms: " << m->ngeom << std::endl;

    d = mj_makeData(m);

    if (!glfwInit()) {
        std::cerr << "Could not initialize GLFW" << std::endl;
        return false;
    }

    window = glfwCreateWindow(1400, 900, "Quadrotor Offline NMPC - MuJoCo Visualization", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // Visual options
    opt.flags[mjVIS_CONVEXHULL] = 0;
    opt.flags[mjVIS_TEXTURE] = 1;
    opt.flags[mjVIS_JOINT] = 0;
    opt.flags[mjVIS_ACTUATOR] = 0;
    opt.flags[mjVIS_CAMERA] = 0;
    opt.flags[mjVIS_LIGHT] = 1;
    opt.flags[mjVIS_TRANSPARENT] = 1;

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // Initial camera view
    cam.azimuth = 135;
    cam.elevation = -30;
    cam.distance = 8.0;
    cam.lookat[0] = 1.5;
    cam.lookat[1] = 1.5;
    cam.lookat[2] = 2.5;

    // Install callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    return true;
}

void cleanupMuJoCo() {
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    if (d) mj_deleteData(d);
    if (m) mj_deleteModel(m);
    if (window) glfwDestroyWindow(window);
    glfwTerminate();
}

void setQuadrotorState(const vector_t& state) {
    // State: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
    
    // Position
    d->qpos[0] = state(0);  // x
    d->qpos[1] = state(1);  // y
    d->qpos[2] = state(2);  // z
    
    // Convert Euler angles (roll, pitch, yaw) to quaternion
    double roll = state(3);
    double pitch = state(4);
    double yaw = state(5);
    
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    d->qpos[3] = cr * cp * cy + sr * sp * sy;  // qw
    d->qpos[4] = sr * cp * cy - cr * sp * sy;  // qx
    d->qpos[5] = cr * sp * cy + sr * cp * sy;  // qy
    d->qpos[6] = cr * cp * sy - sr * sp * cy;  // qz
    
    // Velocities
    d->qvel[0] = state(6);   // vx
    d->qvel[1] = state(7);   // vy
    d->qvel[2] = state(8);   // vz
    d->qvel[3] = state(9);   // wx
    d->qvel[4] = state(10);  // wy
    d->qvel[5] = state(11);  // wz
}

void render(double simTime, double totalTime) {
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    
    // Overlay text
    char status[200];
    snprintf(status, sizeof(status), 
             "Time: %.2f / %.2f s  |  Speed: %.1fx  |  %s  |  Loop: %s",
             simTime, totalTime, playback_speed,
             paused ? "PAUSED" : "PLAYING",
             loop_animation ? "ON" : "OFF");
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, status, NULL, &con);
    
    char controls[200];
    snprintf(controls, sizeof(controls), 
             "Controls: SPACE=Pause  L=Loop  UP/DOWN=Speed  R=Reset View  ESC=Exit");
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, controls, NULL, &con);
    
    glfwSwapBuffers(window);
    glfwPollEvents();
}

// ============================================================================
// Main Function
// ============================================================================
int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "   Quadrotor Offline NMPC Demo" << std::endl;
    std::cout << "   with MuJoCo Visualization" << std::endl;
    std::cout << "========================================" << std::endl;

    // Paths
    std::string packagePath = quadrotor_offline::getPath();
    std::string taskFile = packagePath + "/config/task.info";
    std::string libFolder = packagePath + "/auto_generated";
    std::string modelPath = packagePath + "/robots/quadrotor.xml";

    std::cout << "\n[INFO] Package path: " << packagePath << std::endl;
    std::cout << "[INFO] Task file: " << taskFile << std::endl;
    std::cout << "[INFO] Model file: " << modelPath << std::endl;

    // ========================================================================
    // Step 1: Create Quadrotor interface and load parameters
    // ========================================================================
    std::cout << "\n[1] Creating Quadrotor interface..." << std::endl;
    auto quadrotorInterface = std::make_unique<QuadrotorInterface>(taskFile, libFolder, false);

    vector_t initialState = quadrotorInterface->getInitialState();
    vector_t targetState = quadrotorInterface->getInitialTarget();

    std::cout << "\n[2] Initial state:" << std::endl;
    std::cout << "    Position: [" << initialState(0) << ", " 
              << initialState(1) << ", " << initialState(2) << "]" << std::endl;
    std::cout << "    Attitude: [roll=" << initialState(3) 
              << ", pitch=" << initialState(4) << ", yaw=" << initialState(5) << "]" << std::endl;

    std::cout << "\n[3] Target state:" << std::endl;
    std::cout << "    Position: [" << targetState(0) << ", " 
              << targetState(1) << ", " << targetState(2) << "]" << std::endl;

    // ========================================================================
    // Step 2: Setup and run iLQR optimization
    // ========================================================================
    std::cout << "\n[4] Creating iLQR solver..." << std::endl;
    auto ddpSettings = quadrotorInterface->ddpSettings();
    ddpSettings.algorithm_ = ddp::Algorithm::ILQR;
    ddpSettings.maxNumIterations_ = 100;
    ddpSettings.minRelCost_ = 1e-12;
    ddpSettings.displayInfo_ = false;
    ddpSettings.displayShortSummary_ = true;

    ILQR solver(
        ddpSettings,
        quadrotorInterface->getRollout(),
        quadrotorInterface->getOptimalControlProblem(),
        quadrotorInterface->getInitializer());

    // Set target trajectory
    TargetTrajectories targetTrajectories;
    targetTrajectories.timeTrajectory = {0.0};
    targetTrajectories.stateTrajectory = {targetState};
    targetTrajectories.inputTrajectory = {vector_t::Zero(INPUT_DIM)};
    solver.getReferenceManager().setTargetTrajectories(targetTrajectories);

    // Run optimization
    const double timeHorizon = 15.0;
    std::cout << "\n[5] Running offline trajectory optimization..." << std::endl;
    std::cout << "    Time horizon: " << timeHorizon << " seconds" << std::endl;

    PrimalSolution solution;
    try {
        solver.run(0.0, initialState, timeHorizon);
        std::cout << "[SUCCESS] Optimization completed!" << std::endl;
        solution = solver.primalSolution(timeHorizon);
        
        std::cout << "\n[6] Solution statistics:" << std::endl;
        std::cout << "    Trajectory points: " << solution.timeTrajectory_.size() << std::endl;
        std::cout << "    Duration: " << solution.timeTrajectory_.back() << " s" << std::endl;

        // Print final state
        const auto& finalState = solution.stateTrajectory_.back();
        std::cout << "\n    Final position: [" << finalState(0) << ", " 
                  << finalState(1) << ", " << finalState(2) << "]" << std::endl;
        
        // Calculate position error
        double posError = sqrt(pow(finalState(0) - targetState(0), 2) +
                               pow(finalState(1) - targetState(1), 2) +
                               pow(finalState(2) - targetState(2), 2));
        std::cout << "    Position error: " << posError << " m" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Optimization failed: " << e.what() << std::endl;
        return 1;
    }

    // ========================================================================
    // Step 3: Initialize MuJoCo and visualize trajectory
    // ========================================================================
    std::cout << "\n[7] Initializing MuJoCo visualization..." << std::endl;
    if (!initMuJoCo(modelPath)) {
        std::cerr << "[ERROR] Failed to initialize MuJoCo" << std::endl;
        return 1;
    }
    std::cout << "[SUCCESS] MuJoCo initialized!" << std::endl;

    std::cout << "\n========================================" << std::endl;
    std::cout << "        Visualization Controls" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  SPACE     : Pause/Resume" << std::endl;
    std::cout << "  L         : Toggle loop mode" << std::endl;
    std::cout << "  UP/DOWN   : Adjust playback speed" << std::endl;
    std::cout << "  R         : Reset camera view" << std::endl;
    std::cout << "  Mouse     : Rotate/Zoom/Pan camera" << std::endl;
    std::cout << "  ESC       : Exit" << std::endl;
    std::cout << "========================================" << std::endl;

    // ========================================================================
    // Step 4: Animation loop - replay pre-computed trajectory
    // ========================================================================
    std::cout << "\n[8] Starting trajectory visualization..." << std::endl;

    size_t idx = 0;
    const double dt = 0.01;  // 100 Hz display rate
    double simulationTime = 0.0;
    const double totalTime = solution.timeTrajectory_.back();

    auto lastFrameTime = std::chrono::high_resolution_clock::now();

    while (!glfwWindowShouldClose(window)) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        double frameTime = std::chrono::duration<double>(currentTime - lastFrameTime).count();
        lastFrameTime = currentTime;

        if (!paused) {
            // Advance simulation time based on playback speed
            simulationTime += frameTime * playback_speed;

            // Find the closest trajectory point
            while (idx < solution.timeTrajectory_.size() - 1 && 
                   solution.timeTrajectory_[idx + 1] <= simulationTime) {
                idx++;
            }

            // Interpolate state for smooth animation
            if (idx < solution.timeTrajectory_.size() - 1) {
                double t0 = solution.timeTrajectory_[idx];
                double t1 = solution.timeTrajectory_[idx + 1];
                double alpha = (simulationTime - t0) / (t1 - t0);
                alpha = std::clamp(alpha, 0.0, 1.0);
                
                vector_t interpolatedState = (1.0 - alpha) * solution.stateTrajectory_[idx] +
                                              alpha * solution.stateTrajectory_[idx + 1];
                setQuadrotorState(interpolatedState);
            } else {
                setQuadrotorState(solution.stateTrajectory_.back());
            }

            // Loop or stop at the end
            if (simulationTime >= totalTime) {
                if (loop_animation) {
                    simulationTime = 0.0;
                    idx = 0;
                } else {
                    simulationTime = totalTime;
                }
            }
        }

        // Update MuJoCo physics (forward kinematics)
        mj_forward(m, d);

        // Render
        render(simulationTime, totalTime);

        // Cap frame rate
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // ========================================================================
    // Cleanup
    // ========================================================================
    std::cout << "\n[9] Cleaning up..." << std::endl;
    cleanupMuJoCo();

    std::cout << "\n========================================" << std::endl;
    std::cout << "   Demo Complete!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
