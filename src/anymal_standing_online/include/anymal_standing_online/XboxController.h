#ifndef ANYMAL_STANDING_XBOX_CONTROLLER_H
#define ANYMAL_STANDING_XBOX_CONTROLLER_H

#include <atomic>
#include <string>
#include <thread>
#include <linux/input.h>

namespace anymal_standing_online {

/**
 * @brief Xbox controller input for ANYmal standing control
 * 
 * Joystick mapping for ANYmal:
 * - Left stick Y (ly): Move target X position (forward/backward)
 * - Left stick X (lx): Move target Y position (left/right)
 * - Right stick Y (ry): Adjust target height (up/down)
 * - Right stick X (rx): Adjust target yaw (rotation)
 * 
 * Buttons:
 * - A: Switch to STANDING mode
 * - B: Switch to PASSIVE mode  
 * - X: Switch to RECOVERY mode
 * - Y: Reset to default pose
 * - Start: Toggle MPC on/off
 * - Back: Emergency stop
 * - LB: Decrease height preset
 * - RB: Increase height preset
 */
struct XboxInput {
    float lx = 0.0f;  // Left stick X (-1 to 1)
    float ly = 0.0f;  // Left stick Y (-1 to 1)
    float rx = 0.0f;  // Right stick X (-1 to 1)
    float ry = 0.0f;  // Right stick Y (-1 to 1)
    float lt = 0.0f;  // Left trigger (0 to 1)
    float rt = 0.0f;  // Right trigger (0 to 1)
    
    // Button states (momentary)
    bool btn_a = false;
    bool btn_b = false;
    bool btn_x = false;
    bool btn_y = false;
    bool btn_start = false;
    bool btn_back = false;
    bool btn_lb = false;
    bool btn_rb = false;
    
    void reset() {
        lx = ly = rx = ry = lt = rt = 0.0f;
        btn_a = btn_b = btn_x = btn_y = false;
        btn_start = btn_back = btn_lb = btn_rb = false;
    }
};

class XboxController {
public:
    XboxController();
    ~XboxController();
    
    // Check if controller is connected
    bool isConnected() const { return fd_ >= 0; }
    
    // Get current input state
    XboxInput getInput();
    
    // Get mode change request (-1 = no change, 0=PASSIVE, 1=STANDING, 2=RECOVERY)
    int getModeChangeRequest();
    
    // Check if start button was pressed (and clear the flag)
    bool wasStartPressed();
    
    // Check if back button was pressed (emergency stop)
    bool wasBackPressed();
    
    // Check if Y button pressed (reset pose)
    bool wasResetPressed();
    
    // Get height adjustment (-1 = down, 0 = none, 1 = up)
    int getHeightAdjustRequest();
    
private:
    void runThread();
    std::string detectDevicePath(const std::string& nameHint);
    float applyDeadband(float value, float deadband = 0.08f);
    
    int fd_ = -1;
    std::thread inputThread_;
    std::atomic<bool> running_{false};
    
    // Current input state (protected by atomic operations)
    std::atomic<float> lx_{0.0f};
    std::atomic<float> ly_{0.0f};
    std::atomic<float> rx_{0.0f};
    std::atomic<float> ry_{0.0f};
    std::atomic<float> lt_{0.0f};
    std::atomic<float> rt_{0.0f};
    
    // Button press events (latched, cleared on read)
    std::atomic<int> modeChangeRequest_{-1};
    std::atomic<bool> startPressed_{false};
    std::atomic<bool> backPressed_{false};
    std::atomic<bool> resetPressed_{false};
    std::atomic<int> heightAdjustRequest_{0};
    
    // Low-pass filter alpha
    float alpha_ = 0.8f;
};

}  // namespace anymal_standing_online

#endif  // ANYMAL_STANDING_XBOX_CONTROLLER_H
