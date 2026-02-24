#pragma once

#include <atomic>
#include <thread>
#include <termios.h>
#include <unistd.h>

namespace anymal_standing_online {

/**
 * @brief Keyboard input state for ANYmal standing control
 * 
 * Keyboard mapping for ANYmal Standing:
 * - W/S: Forward/Backward (ly)
 * - A/D: Left/Right strafe (lx)
 * - I/K: Up/Down height (ry)
 * - J/L: Yaw left/right (rx)
 * - 1: STANDING mode (A button)
 * - 2: PASSIVE mode (B button)
 * - 3: RECOVERY mode (X button)
 * - 4/R: Reset to default pose (Y button)
 * - Enter: Start/Toggle MPC
 * - Backspace/Q: Emergency stop (Back button)
 * - -/=: Height preset down/up (LB/RB)
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
 * @brief Keyboard controller interface for ANYmal standing control
 */
class KeyboardController {
public:
    KeyboardController();
    ~KeyboardController();
    
    bool isConnected() const { return running_.load(); }
    KeyboardInput getInput();
    
    // Mode change request (-1 = no change, 0=PASSIVE, 1=STANDING, 2=RECOVERY)
    int getModeChangeRequest();
    
    bool wasStartPressed();
    bool wasBackPressed();
    bool wasResetPressed();
    
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
    std::atomic<bool> resetPressed_{false};
    std::atomic<int> heightAdjustRequest_{0};
    
    struct termios oldSettings_;
    struct termios newSettings_;
    
    float sensitivity_ = 0.1f;
    float decayRate_ = 0.95f;
    
    void runThread();
    float clamp(float value, float minVal, float maxVal);
};

}  // namespace anymal_standing_online
