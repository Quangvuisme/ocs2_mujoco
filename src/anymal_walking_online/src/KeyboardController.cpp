#include "anymal_walking_online/KeyboardController.h"
#include <iostream>
#include <sys/select.h>
#include <cmath>

namespace anymal_walking_online {

KeyboardController::KeyboardController() {
    tcgetattr(fileno(stdin), &oldSettings_);
    newSettings_ = oldSettings_;
    newSettings_.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(fileno(stdin), TCSANOW, &newSettings_);
    
    running_ = true;
    inputThread_ = std::thread(&KeyboardController::runThread, this);
    
    std::cout << "[Keyboard] ANYmal Walking Controller initialized" << std::endl;
    std::cout << "[Keyboard] Controls:" << std::endl;
    std::cout << "  W/S: Forward/Backward velocity" << std::endl;
    std::cout << "  A/D: Left/Right strafe" << std::endl;
    std::cout << "  I/K: Height adjustment" << std::endl;
    std::cout << "  J/L: Yaw rate" << std::endl;
    std::cout << "  1: PASSIVE mode" << std::endl;
    std::cout << "  2: PD_CONTROL mode" << std::endl;
    std::cout << "  3: STANDING mode (MPC)" << std::endl;
    std::cout << "  4: WALKING mode (MPC + Gait)" << std::endl;
    std::cout << "  Enter: Toggle MPC" << std::endl;
    std::cout << "  Q: Emergency stop" << std::endl;
    std::cout << "  G: Switch gait" << std::endl;
    std::cout << "  -/=: Height preset down/up" << std::endl;
    std::cout << "  Space: Reset stick values" << std::endl;
}

KeyboardController::~KeyboardController() {
    running_ = false;
    if (inputThread_.joinable()) {
        inputThread_.join();
    }
    tcsetattr(fileno(stdin), TCSANOW, &oldSettings_);
}

void KeyboardController::runThread() {
    fd_set set;
    struct timeval timeout;
    
    float local_lx = 0.0f, local_ly = 0.0f;
    float local_rx = 0.0f, local_ry = 0.0f;
    
    while (running_) {
        FD_ZERO(&set);
        FD_SET(fileno(stdin), &set);
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;
        
        int res = select(fileno(stdin) + 1, &set, NULL, NULL, &timeout);
        
        bool keyPressed = false;
        
        if (res > 0) {
            char seq[3];
            ssize_t nread = read(fileno(stdin), &seq[0], 1);
            
            if (nread > 0) {
                if (seq[0] == '\x1b') {
                    read(fileno(stdin), &seq[1], 1);
                    if (seq[1] == '[') {
                        read(fileno(stdin), &seq[2], 1);
                        switch (seq[2]) {
                            case 'A': local_ly = clamp(local_ly + sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                            case 'B': local_ly = clamp(local_ly - sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                            case 'C': local_lx = clamp(local_lx + sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                            case 'D': local_lx = clamp(local_lx - sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                        }
                    }
                } else {
                    char c = seq[0];
                    
                    switch (c) {
                        // WASD for movement
                        case 'w': case 'W': local_ly = clamp(local_ly + sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                        case 's': case 'S': local_ly = clamp(local_ly - sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                        case 'a': case 'A': local_lx = clamp(local_lx + sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                        case 'd': case 'D': local_lx = clamp(local_lx - sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                        
                        // IJKL for yaw and height
                        case 'i': case 'I': local_ry = clamp(local_ry + sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                        case 'k': case 'K': local_ry = clamp(local_ry - sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                        case 'j': case 'J': local_rx = clamp(local_rx + sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                        case 'l': case 'L': local_rx = clamp(local_rx - sensitivity_, -1.0f, 1.0f); keyPressed = true; break;
                        
                        // Mode selection: 1=PASSIVE, 2=PD_CONTROL, 3=STANDING, 4=WALKING
                        case '1': modeChangeRequest_.store(0); break;  // PASSIVE
                        case '2': modeChangeRequest_.store(1); break;  // PD_CONTROL
                        case '3': modeChangeRequest_.store(2); break;  // STANDING
                        case '4': modeChangeRequest_.store(3); break;  // WALKING
                        
                        // Toggle MPC
                        case '\n': case '\r': startPressed_.store(true); break;
                        
                        // Emergency stop
                        case 'q': case 'Q': case 127: backPressed_.store(true); break;
                        
                        // Gait switch
                        case 'g': case 'G': gaitSwitchPressed_.store(true); break;
                        
                        // Height preset adjustment
                        case '-': case '_': heightAdjustRequest_.store(-1); break;
                        case '=': case '+': heightAdjustRequest_.store(1); break;
                        
                        // Reset stick values
                        case ' ':
                            local_lx = local_ly = local_rx = local_ry = 0.0f;
                            keyPressed = true;
                            break;
                    }
                }
            }
        }
        
        if (!keyPressed) {
            local_lx *= decayRate_; local_ly *= decayRate_;
            local_rx *= decayRate_; local_ry *= decayRate_;
            if (std::fabs(local_lx) < 0.01f) local_lx = 0.0f;
            if (std::fabs(local_ly) < 0.01f) local_ly = 0.0f;
            if (std::fabs(local_rx) < 0.01f) local_rx = 0.0f;
            if (std::fabs(local_ry) < 0.01f) local_ry = 0.0f;
        }
        
        lx_.store(local_lx); ly_.store(local_ly);
        rx_.store(local_rx); ry_.store(local_ry);
    }
}

KeyboardInput KeyboardController::getInput() {
    KeyboardInput input;
    input.lx = lx_.load(); input.ly = ly_.load();
    input.rx = rx_.load(); input.ry = ry_.load();
    input.lt = lt_.load(); input.rt = rt_.load();
    return input;
}

int KeyboardController::getModeChangeRequest() { return modeChangeRequest_.exchange(-1); }
bool KeyboardController::wasStartPressed() { return startPressed_.exchange(false); }
bool KeyboardController::wasBackPressed() { return backPressed_.exchange(false); }
bool KeyboardController::wasGaitSwitchPressed() { return gaitSwitchPressed_.exchange(false); }
int KeyboardController::getHeightAdjustRequest() { return heightAdjustRequest_.exchange(0); }

float KeyboardController::clamp(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

}  // namespace anymal_walking_online
