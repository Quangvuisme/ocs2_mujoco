#include "quadruped_nmpc/KeyboardController.h"
#include <iostream>
#include <sys/select.h>
#include <cmath>

namespace quadruped_nmpc {

KeyboardController::KeyboardController() {
    // Setup terminal for non-blocking input
    tcgetattr(fileno(stdin), &oldSettings_);
    newSettings_ = oldSettings_;
    newSettings_.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(fileno(stdin), TCSANOW, &newSettings_);
    
    running_ = true;
    inputThread_ = std::thread(&KeyboardController::runThread, this);
    
    std::cout << "[Keyboard] Controller initialized" << std::endl;
    std::cout << "[Keyboard] Controls:" << std::endl;
    std::cout << "  W/S: Forward/Backward" << std::endl;
    std::cout << "  A/D: Left/Right strafe" << std::endl;
    std::cout << "  I/K: Up/Down (height)" << std::endl;
    std::cout << "  J/L: Yaw left/right" << std::endl;
    std::cout << "  1-4: Select target" << std::endl;
    std::cout << "  Enter: Start/Toggle" << std::endl;
    std::cout << "  Space: Reset values" << std::endl;
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
    char c;
    
    float local_lx = 0.0f, local_ly = 0.0f;
    float local_rx = 0.0f, local_ry = 0.0f;
    
    while (running_) {
        FD_ZERO(&set);
        FD_SET(fileno(stdin), &set);
        
        timeout.tv_sec = 0;
        timeout.tv_usec = 50000;  // 50ms timeout
        
        int res = select(fileno(stdin) + 1, &set, NULL, NULL, &timeout);
        
        bool keyPressed = false;
        
        if (res > 0) {
            char seq[3];
            ssize_t nread = read(fileno(stdin), &seq[0], 1);
            
            if (nread > 0) {
                // Handle arrow keys (escape sequences)
                if (seq[0] == '\x1b') {
                    read(fileno(stdin), &seq[1], 1);
                    if (seq[1] == '[') {
                        read(fileno(stdin), &seq[2], 1);
                        switch (seq[2]) {
                            case 'A': // Up arrow
                                local_ly = clamp(local_ly + sensitivity_, -1.0f, 1.0f);
                                keyPressed = true;
                                break;
                            case 'B': // Down arrow
                                local_ly = clamp(local_ly - sensitivity_, -1.0f, 1.0f);
                                keyPressed = true;
                                break;
                            case 'C': // Right arrow
                                local_lx = clamp(local_lx + sensitivity_, -1.0f, 1.0f);
                                keyPressed = true;
                                break;
                            case 'D': // Left arrow
                                local_lx = clamp(local_lx - sensitivity_, -1.0f, 1.0f);
                                keyPressed = true;
                                break;
                        }
                    }
                } else {
                    c = seq[0];
                    
                    switch (c) {
                        // WASD for XY movement
                        case 'w': case 'W':
                            local_ly = clamp(local_ly + sensitivity_, -1.0f, 1.0f);
                            keyPressed = true;
                            break;
                        case 's': case 'S':
                            local_ly = clamp(local_ly - sensitivity_, -1.0f, 1.0f);
                            keyPressed = true;
                            break;
                        case 'a': case 'A':
                            local_lx = clamp(local_lx + sensitivity_, -1.0f, 1.0f);
                            keyPressed = true;
                            break;
                        case 'd': case 'D':
                            local_lx = clamp(local_lx - sensitivity_, -1.0f, 1.0f);
                            keyPressed = true;
                            break;
                        
                        // IJKL for yaw and height
                        case 'i': case 'I':
                            local_ry = clamp(local_ry + sensitivity_, -1.0f, 1.0f);
                            keyPressed = true;
                            break;
                        case 'k': case 'K':
                            local_ry = clamp(local_ry - sensitivity_, -1.0f, 1.0f);
                            keyPressed = true;
                            break;
                        case 'j': case 'J':
                            local_rx = clamp(local_rx + sensitivity_, -1.0f, 1.0f);
                            keyPressed = true;
                            break;
                        case 'l': case 'L':
                            local_rx = clamp(local_rx - sensitivity_, -1.0f, 1.0f);
                            keyPressed = true;
                            break;
                        
                        // Target selection (1-4 maps to 0-3)
                        case '1':
                            targetChangeRequest_.store(0);
                            break;
                        case '2':
                            targetChangeRequest_.store(1);
                            break;
                        case '3':
                            targetChangeRequest_.store(2);
                            break;
                        case '4':
                            targetChangeRequest_.store(3);
                            break;
                        
                        // Start button
                        case '\n': case '\r':
                            startPressed_.store(true);
                            break;
                        
                        // Space to reset
                        case ' ':
                            local_lx = 0.0f;
                            local_ly = 0.0f;
                            local_rx = 0.0f;
                            local_ry = 0.0f;
                            keyPressed = true;
                            break;
                    }
                }
            }
        }
        
        // Apply decay when no key is pressed
        if (!keyPressed) {
            local_lx *= decayRate_;
            local_ly *= decayRate_;
            local_rx *= decayRate_;
            local_ry *= decayRate_;
            
            // Zero out small values
            if (std::fabs(local_lx) < 0.01f) local_lx = 0.0f;
            if (std::fabs(local_ly) < 0.01f) local_ly = 0.0f;
            if (std::fabs(local_rx) < 0.01f) local_rx = 0.0f;
            if (std::fabs(local_ry) < 0.01f) local_ry = 0.0f;
        }
        
        // Update atomic values
        lx_.store(local_lx);
        ly_.store(local_ly);
        rx_.store(local_rx);
        ry_.store(local_ry);
    }
}

KeyboardInput KeyboardController::getInput() {
    KeyboardInput input;
    input.lx = lx_.load();
    input.ly = ly_.load();
    input.rx = rx_.load();
    input.ry = ry_.load();
    return input;
}

int KeyboardController::getTargetChangeRequest() {
    return targetChangeRequest_.exchange(-1);
}

bool KeyboardController::wasStartPressed() {
    return startPressed_.exchange(false);
}

float KeyboardController::clamp(float value, float minVal, float maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

}  // namespace quadruped_nmpc
