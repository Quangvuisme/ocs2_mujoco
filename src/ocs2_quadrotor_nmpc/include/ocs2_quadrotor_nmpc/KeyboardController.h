#pragma once

#include <atomic>
#include <thread>
#include <termios.h>
#include <unistd.h>

namespace ocs2_cartpole_quadrotor {

/**
 * @brief Keyboard input state (same interface as XboxInput)
 * 
 * Keyboard mapping for OCS2 Quadrotor:
 * - W/S: Forward/Backward (ly)
 * - A/D: Left/Right (lx)
 * - I/K: Up/Down height (ry)
 * - J/L: Yaw left/right (rx)
 * - Space: Reset all values to zero
 * - 1-4: Target selection (same as A,B,X,Y buttons)
 * - Enter: Start button equivalent
 */
struct KeyboardInput {
    float lx = 0.0f;
    float ly = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
};

/**
 * @brief Keyboard controller interface for OCS2 Quadrotor control
 */
class KeyboardController {
public:
    KeyboardController();
    ~KeyboardController();
    
    bool isConnected() const { return running_.load(); }
    KeyboardInput getInput();
    
    int getTargetChangeRequest();
    bool wasStartPressed();
    
private:
    std::thread inputThread_;
    std::atomic<bool> running_{false};
    
    std::atomic<float> lx_{0.0f}, ly_{0.0f};
    std::atomic<float> rx_{0.0f}, ry_{0.0f};
    std::atomic<int> targetChangeRequest_{-1};
    std::atomic<bool> startPressed_{false};
    
    struct termios oldSettings_;
    struct termios newSettings_;
    
    float sensitivity_ = 0.1f;
    float decayRate_ = 0.95f;
    
    void runThread();
    float clamp(float value, float minVal, float maxVal);
};

}  // namespace ocs2_cartpole_quadrotor
