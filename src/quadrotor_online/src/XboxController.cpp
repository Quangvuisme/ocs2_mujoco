#include <quadrotor_online/XboxController.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <regex>
#include <chrono>
#include <cmath>

namespace quadrotor_online {

XboxController::XboxController() {
    // Try to detect Xbox controller
    std::vector<std::string> controllerNames = {
        "Microsoft Xbox Controller",           // USB wired Xbox controller
        "Xbox One Wireless Controller",        // Xbox One wireless
        "Microsoft Xbox Series S|X Controller", // Series S|X
        "Xbox Wireless Controller",            // Generic wireless
        "Microsoft X-Box One pad"              // Legacy
    };
    
    std::string devicePath;
    for (const auto& name : controllerNames) {
        devicePath = detectDevicePath(name);
        if (!devicePath.empty()) {
            std::cout << "[Xbox] Found controller: " << name << std::endl;
            break;
        }
    }
    
    if (devicePath.empty()) {
        std::cerr << "[Xbox] No Xbox controller found. Joystick control disabled." << std::endl;
        return;
    }
    
    fd_ = open(devicePath.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "[Xbox] Failed to open device: " << devicePath << std::endl;
        return;
    }
    
    std::cout << "[Xbox] Connected to: " << devicePath << std::endl;
    
    running_ = true;
    inputThread_ = std::thread(&XboxController::runThread, this);
}

XboxController::~XboxController() {
    running_ = false;
    if (inputThread_.joinable()) {
        inputThread_.join();
    }
    if (fd_ >= 0) {
        close(fd_);
    }
}

void XboxController::runThread() {
    struct input_event ev;
    float raw_lx = 0, raw_ly = 0, raw_rx = 0, raw_ry = 0;
    
    while (running_) {
        if (fd_ < 0) {
            // Try to reconnect
            std::this_thread::sleep_for(std::chrono::seconds(2));
            std::vector<std::string> controllerNames = {
                "Microsoft Xbox Controller",
                "Xbox One Wireless Controller",
                "Microsoft Xbox Series S|X Controller"
            };
            for (const auto& name : controllerNames) {
                std::string devicePath = detectDevicePath(name);
                if (!devicePath.empty()) {
                    fd_ = open(devicePath.c_str(), O_RDONLY | O_NONBLOCK);
                    if (fd_ >= 0) {
                        std::cout << "[Xbox] Reconnected!" << std::endl;
                        break;
                    }
                }
            }
            continue;
        }
        
        ssize_t bytes = read(fd_, &ev, sizeof(ev));
        if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data available, sleep a bit
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            // Error - controller disconnected?
            std::cerr << "[Xbox] Controller disconnected" << std::endl;
            close(fd_);
            fd_ = -1;
            continue;
        }
        
        if (bytes < (ssize_t)sizeof(ev)) continue;
        
        // Handle buttons
        if (ev.type == EV_KEY) {
            switch (ev.code) {
                case BTN_A:  // A button
                    if (ev.value == 1) targetChangeRequest_.store(0);
                    break;
                case BTN_B:  // B button
                    if (ev.value == 1) targetChangeRequest_.store(1);
                    break;
                case BTN_X:  // X button
                    if (ev.value == 1) targetChangeRequest_.store(2);
                    break;
                case BTN_Y:  // Y button
                    if (ev.value == 1) targetChangeRequest_.store(3);
                    break;
                case BTN_START:  // Start button
                    if (ev.value == 1) startPressed_.store(true);
                    break;
            }
        }
        // Handle analog sticks
        else if (ev.type == EV_ABS) {
            float normalized = ev.value / 32767.0f;
            
            switch (ev.code) {
                case ABS_X:  // Left stick X
                    raw_lx = applyDeadband(-normalized);
                    lx_.store(alpha_ * raw_lx + (1.0f - alpha_) * lx_.load());
                    break;
                case ABS_Y:  // Left stick Y
                    raw_ly = applyDeadband(-normalized);
                    ly_.store(alpha_ * raw_ly + (1.0f - alpha_) * ly_.load());
                    break;
                case ABS_RX:  // Right stick X
                    raw_rx = applyDeadband(-normalized);
                    rx_.store(alpha_ * raw_rx + (1.0f - alpha_) * rx_.load());
                    break;
                case ABS_RY:  // Right stick Y
                    raw_ry = applyDeadband(-normalized);
                    ry_.store(alpha_ * raw_ry + (1.0f - alpha_) * ry_.load());
                    break;
            }
        }
    }
}

XboxInput XboxController::getInput() {
    XboxInput input;
    input.lx = lx_.load();
    input.ly = ly_.load();
    input.rx = rx_.load();
    input.ry = ry_.load();
    return input;
}

int XboxController::getTargetChangeRequest() {
    return targetChangeRequest_.exchange(-1);
}

bool XboxController::wasStartPressed() {
    return startPressed_.exchange(false);
}

float XboxController::applyDeadband(float value, float deadband) {
    if (std::fabs(value) < deadband)
        return 0.0f;
    float scaled = (value - std::copysign(deadband, value)) / (1.0f - deadband);
    return scaled;
}

std::string XboxController::detectDevicePath(const std::string& nameHint) {
    std::ifstream devices("/proc/bus/input/devices");
    if (!devices.is_open()) return "";

    std::string line;
    std::string eventName;
    bool found = false;

    while (std::getline(devices, line)) {
        if (line.rfind("N: Name=", 0) == 0 && line.find(nameHint) != std::string::npos) {
            found = true;
        } else if (found && line.rfind("H: Handlers=", 0) == 0) {
            std::regex re("event[0-9]+");
            std::smatch match;
            if (std::regex_search(line, match, re)) {
                eventName = match.str(0);
                break;
            }
        }
    }

    if (!eventName.empty()) {
        return "/dev/input/" + eventName;
    }

    return "";
}

}  // namespace quadrotor_online
