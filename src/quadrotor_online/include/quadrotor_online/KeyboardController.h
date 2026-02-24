#pragma once

#include <atomic>
#include <thread>
#include <termios.h>
#include <unistd.h>

namespace quadrotor_online {

/**
 * @brief Keyboard input state (same interface as XboxInput)
 * 
 * Keyboard mapping for Quadrotor:
 * - W/S: Forward/Backward (ly)
 * - A/D: Left/Right (lx)
 * - I/K: Up/Down height (ry)
 * - J/L: Yaw left/right (rx)
 * - Space: Reset all values to zero
 * - 1-4: Target selection (same as A,B,X,Y buttons)
 * - Enter: Start button equivalent
 */
struct KeyboardInput {
    float lx = 0.0f;   // Left stick X (-1 to 1) - A/D keys
    float ly = 0.0f;   // Left stick Y (-1 to 1) - W/S keys
    float rx = 0.0f;   // Right stick X (-1 to 1) - J/L keys (yaw)
    float ry = 0.0f;   // Right stick Y (-1 to 1) - I/K keys (height)
};

/**
 * @brief Keyboard controller interface for Quadrotor control
 */
class KeyboardController {
public:
    KeyboardController();
    ~KeyboardController();
    
    bool isConnected() const { return running_.load(); }
    KeyboardInput getInput();
    
    // Button events (compatible with XboxController interface)
    int getTargetChangeRequest();  // 1=0, 2=1, 3=2, 4=3, -1 if none
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

}  // namespace quadrotor_online
