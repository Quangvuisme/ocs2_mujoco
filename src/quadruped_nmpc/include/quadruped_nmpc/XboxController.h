#pragma once

#include <atomic>
#include <thread>
#include <string>
#include <linux/input.h>

namespace quadruped_nmpc {

/**
 * @brief Xbox controller input state
 */
struct XboxInput {
    float lx = 0.0f;   // Left stick X (-1 to 1)
    float ly = 0.0f;   // Left stick Y (-1 to 1)
    float rx = 0.0f;   // Right stick X (-1 to 1)
    float ry = 0.0f;   // Right stick Y (-1 to 1)
};

/**
 * @brief Xbox controller interface for Quadruped control
 */
class XboxController {
public:
    XboxController();
    ~XboxController();
    
    bool isConnected() const { return fd_ >= 0; }
    XboxInput getInput();
    
    // Button events
    int getTargetChangeRequest();  // A=0, B=1, X=2, Y=3, -1 if none
    bool wasStartPressed();
    
private:
    int fd_ = -1;
    std::thread inputThread_;
    std::atomic<bool> running_{false};
    
    std::atomic<float> lx_{0.0f}, ly_{0.0f};
    std::atomic<float> rx_{0.0f}, ry_{0.0f};
    std::atomic<int> targetChangeRequest_{-1};
    std::atomic<bool> startPressed_{false};
    
    float alpha_ = 0.3f;  // Low-pass filter
    
    void runThread();
    std::string detectDevicePath(const std::string& nameHint);
    float applyDeadband(float value, float deadband = 0.1f);
};

}  // namespace quadruped_nmpc
