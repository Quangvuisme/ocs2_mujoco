#ifndef CARTPOLE_BRIDGE_H
#define CARTPOLE_BRIDGE_H

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <mujoco/mujoco.h>

// CRC32 calculation
inline uint32_t cartpole_crc32(const void* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    
    for (size_t i = 0; i < length; i++) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

struct CartPoleState {
    float cart_pos;        // x (index 0)
    float pole_angle;      // θ (index 1)
    float cart_vel;        // ẋ (index 2)
    float pole_anglevel;   // θ̇ (index 3)
    float timestamp;
    uint32_t crc;
};

struct CartPoleCommand {
    float force;           // F
    float timestamp;
    uint32_t crc;
};

class CartPoleBridge {
private:
    int sock_state_;
    int sock_cmd_;
    struct sockaddr_in addr_state_;
    struct sockaddr_in addr_cmd_;
    
    bool initialized_;
    std::atomic<bool> running_;
    
    const int STATE_PORT = 9001;
    const int CMD_PORT = 9002;
    const char* CONTROLLER_IP = "127.0.0.1";
    
    CartPoleCommand last_cmd_;
    
    mjModel* mj_model_;
    mjData* mj_data_;
    
    std::thread state_publish_thread_;
    std::thread cmd_receive_thread_;

    void PublishStateLoop() {
        printf("CartPole: State publish thread started (500 Hz)\n");
        while (running_) {
            if (mj_data_ && initialized_) {
                CartPoleState state;
                
                // Get sensor data (in order defined in XML)
                state.cart_pos = mj_data_->sensordata[0];
                state.pole_angle = mj_data_->sensordata[1];
                state.cart_vel = mj_data_->sensordata[2];
                state.pole_anglevel = mj_data_->sensordata[3];
                state.timestamp = static_cast<float>(mj_data_->time);
                
                // Calculate CRC
                state.crc = cartpole_crc32(&state, sizeof(CartPoleState) - sizeof(uint32_t));
                
                // Send UDP packet
                sendto(sock_state_, &state, sizeof(state), 0,
                       (struct sockaddr*)&addr_state_, sizeof(addr_state_));
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 500 Hz
        }
        printf("CartPole: State publish thread stopped\n");
    }
    
    void ReceiveCommandLoop() {
        printf("CartPole: Command receive thread started\n");
        while (running_) {
            if (initialized_) {
                CartPoleCommand recv_cmd;
                ssize_t n = recv(sock_cmd_, &recv_cmd, sizeof(recv_cmd), 0);
                
                if (n == sizeof(CartPoleCommand)) {
                    // Verify CRC
                    uint32_t calc_crc = cartpole_crc32(&recv_cmd, sizeof(CartPoleCommand) - sizeof(uint32_t));
                    if (calc_crc == recv_cmd.crc) {
                        last_cmd_ = recv_cmd;
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 500 Hz
        }
        printf("CartPole: Command receive thread stopped\n");
    }

public:
    CartPoleBridge() : sock_state_(-1), sock_cmd_(-1), initialized_(false), 
                       running_(false), mj_model_(nullptr), mj_data_(nullptr) {
        last_cmd_.force = 0.0f;
        last_cmd_.timestamp = 0.0f;
        last_cmd_.crc = 0;
    }
    
    ~CartPoleBridge() {
        running_ = false;
        
        if (state_publish_thread_.joinable())
            state_publish_thread_.join();
        if (cmd_receive_thread_.joinable())
            cmd_receive_thread_.join();
            
        if (sock_state_ >= 0) close(sock_state_);
        if (sock_cmd_ >= 0) close(sock_cmd_);
    }
    
    bool init(mjModel* m, mjData* d) {
        mj_model_ = m;
        mj_data_ = d;
        
        // Create state broadcast socket
        sock_state_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_state_ < 0) {
            printf("CartPole: Failed to create state socket\n");
            return false;
        }
        
        memset(&addr_state_, 0, sizeof(addr_state_));
        addr_state_.sin_family = AF_INET;
        addr_state_.sin_port = htons(STATE_PORT);
        inet_pton(AF_INET, CONTROLLER_IP, &addr_state_.sin_addr);
        
        // Create command receive socket
        sock_cmd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_cmd_ < 0) {
            printf("CartPole: Failed to create command socket\n");
            return false;
        }
        
        // Set non-blocking
        int flags = fcntl(sock_cmd_, F_GETFL, 0);
        fcntl(sock_cmd_, F_SETFL, flags | O_NONBLOCK);
        
        memset(&addr_cmd_, 0, sizeof(addr_cmd_));
        addr_cmd_.sin_family = AF_INET;
        addr_cmd_.sin_addr.s_addr = INADDR_ANY;
        addr_cmd_.sin_port = htons(CMD_PORT);
        
        if (bind(sock_cmd_, (struct sockaddr*)&addr_cmd_, sizeof(addr_cmd_)) < 0) {
            printf("CartPole: Failed to bind command socket\n");
            return false;
        }
        
        // FLUSH OLD DATA FROM COMMAND SOCKET BUFFER
        // Reduce buffer size to prevent accumulation (default ~212KB -> 2KB)
        int recv_buf_size = 2048;
        setsockopt(sock_cmd_, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size));
        
        printf("CartPole: Flushing old command data from socket buffer...\n");
        int flushed_count = 0;
        auto flush_start = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - flush_start).count() < 1.0) {
            CartPoleCommand dummy_cmd;
            ssize_t n = recv(sock_cmd_, &dummy_cmd, sizeof(dummy_cmd), 0);
            if (n > 0) {
                flushed_count++;
            } else if (flushed_count > 0) {
                // Got some data before, now buffer empty
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                break;
            }
        }
        printf("CartPole: Flushed %d old command packets\n", flushed_count);
        
        initialized_ = true;
        running_ = true;
        
        // Start threads
        state_publish_thread_ = std::thread(&CartPoleBridge::PublishStateLoop, this);
        cmd_receive_thread_ = std::thread(&CartPoleBridge::ReceiveCommandLoop, this);
        
        printf("CartPole UDP Bridge initialized (state:%d, cmd:%d)\n", STATE_PORT, CMD_PORT);
        return true;
    }
    
    void apply_command(mjData* d) {
        if (!initialized_) return;
        
        // Apply force to cart (actuator index 0)
        d->ctrl[0] = last_cmd_.force;
    }
};

#endif // CARTPOLE_BRIDGE_H
