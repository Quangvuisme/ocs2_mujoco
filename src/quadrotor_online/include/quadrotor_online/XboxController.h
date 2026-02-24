#ifndef QUADROTOR_XBOX_CONTROLLER_H
#define QUADROTOR_XBOX_CONTROLLER_H

#include <atomic>
#include <string>
#include <thread>
#include <linux/input.h>

namespace quadrotor_online {

/**
 * @brief Xbox controller input for quadrotor control
 * 
 * Joystick mapping for quadrotor:
 * - Left stick Y (ly): Forward/Backward velocity (target x)
 * - Left stick X (lx): Left/Right velocity (target y)
 * - Right stick Y (ry): Up/Down velocity (target z)
 * - Right stick X (rx): Yaw rate
 * 
 * Buttons:
 * - A: Select target 1
 * - B: Select target 2
 * - X: Select target 3
 * - Y: Select target 4
 * - Start: Toggle run/pause
 */
struct XboxInput {
    float lx = 0.0f;  // Left stick X (-1 to 1)
    float ly = 0.0f;  // Left stick Y (-1 to 1)
    float rx = 0.0f;  // Right stick X (-1 to 1)
    float ry = 0.0f;  // Right stick Y (-1 to 1)
    
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
        lx = ly = rx = ry = 0.0f;
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
    
    // Get target change request (-1 = no change, 0-3 = target index)
    int getTargetChangeRequest();
    
    // Check if start button was pressed (and clear the flag)
    bool wasStartPressed();
    
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
    
    // Button press events (latched, cleared on read)
    std::atomic<int> targetChangeRequest_{-1};
    std::atomic<bool> startPressed_{false};
    
    // Low-pass filter alpha
    float alpha_ = 0.8f;
};

}  // namespace quadrotor_online

#endif  // QUADROTOR_XBOX_CONTROLLER_H
