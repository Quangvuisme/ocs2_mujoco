#pragma once

#include <atomic>
#include <thread>
#include <termios.h>
#include <unistd.h>

namespace anymal_walking_online {

/**
 * @brief Keyboard input state for ANYmal walking control
 * 
 * Keyboard mapping for ANYmal Walking:
 * 
 * Movement (both Standing and Walking mode):
 * - W/S: Forward/Backward (ly)
 * - A/D: Left/Right strafe (lx)
 * - I/K: Up/Down height (ry)
 * - J/L: Yaw left/right (rx)
 * 
 * Mode selection:
 * - 1: PASSIVE mode
 * - 2: PD_CONTROL mode
 * - 3: STANDING mode (MPC)
 * - 4: WALKING mode (MPC + Gait)
 * 
 * Controls:
 * - Enter: Toggle MPC on/off
 * - Q/Backspace: Emergency stop
 * - G: Switch gait (in walking mode)
 * - -/=: Height preset down/up
 * - Space: Reset stick values to zero
 */
struct KeyboardInput {
    float lx = 0.0f;
    float ly = 0.0f;
    float rx = 0.0f;
    float ry = 0.0f;
    float lt = 0.0f;
    float rt = 0.0f;
};

/**
 * @brief Keyboard controller interface for ANYmal walking control
 */
class KeyboardController {
public:
    KeyboardController();
    ~KeyboardController();
    
    bool isConnected() const { return running_.load(); }
    KeyboardInput getInput();
    
    // Mode change request (-1 = no change, 0=PASSIVE, 1=PD_CONTROL, 2=STANDING, 3=WALKING)
    int getModeChangeRequest();
    
    bool wasStartPressed();
    bool wasBackPressed();
    bool wasGaitSwitchPressed();
    
    // Height adjustment (-1 = down, 0 = none, 1 = up)
    int getHeightAdjustRequest();
    
private:
    std::thread inputThread_;
    std::atomic<bool> running_{false};
    
    std::atomic<float> lx_{0.0f}, ly_{0.0f};
    std::atomic<float> rx_{0.0f}, ry_{0.0f};
    std::atomic<float> lt_{0.0f}, rt_{0.0f};
    
    std::atomic<int> modeChangeRequest_{-1};
    std::atomic<bool> startPressed_{false};
    std::atomic<bool> backPressed_{false};
    std::atomic<bool> gaitSwitchPressed_{false};
    std::atomic<int> heightAdjustRequest_{0};
    
    struct termios oldSettings_;
    struct termios newSettings_;
    
    float sensitivity_ = 0.1f;
    float decayRate_ = 0.95f;
    
    void runThread();
    float clamp(float value, float minVal, float maxVal);
};

}  // namespace anymal_walking_online
