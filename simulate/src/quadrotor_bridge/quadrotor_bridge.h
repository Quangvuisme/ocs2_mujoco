#ifndef QUADROTOR_BRIDGE_H
#define QUADROTOR_BRIDGE_H

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
#include <cmath>
#include <Eigen/Dense>

// Quadrotor State: [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
// Command: [Fz, Mx, My, Mz]
// Motor allocation: [F1, F2, F3, F4] from [Fz, Mx, My, Mz]
//
// Quadrotor configuration (X-configuration):
//     Motor 1 (Front, +X)
//          ^
//          |
// Motor 4 -+- Motor 2 (Right, +Y)
// (Left)   |
//          |
//     Motor 3 (Back, -X)
//
// Arm length from center to motor: L = 0.1265 m
//
// Allocation matrix (standard X-configuration):
// [F1]   [1   0      -1/L   -k] [Fz ]
// [F2] = [1  -1/L     0      k] [Mx ]
// [F3]   [1   0       1/L   -k] [My ]
// [F4]   [1   1/L     0      k] [Mz ]
//
// Where k = torque constant (moment per thrust)

struct QuadrotorState {
    uint64_t timestamp;  // nanoseconds
    double state[12];    // [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
} __attribute__((packed));

struct QuadrotorCommand {
    uint64_t timestamp;  // nanoseconds
    double input[4];     // [Fz, Mx, My, Mz]
} __attribute__((packed));

// Quaternion to Euler angles (XYZ convention)
inline void quat2euler(const mjtNum quat[4], double& roll, double& pitch, double& yaw) {
    double qw = quat[0];
    double qx = quat[1];
    double qy = quat[2];
    double qz = quat[3];
    
    // Roll (X-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (Y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp);
    else
        pitch = std::asin(sinp);
    
    // Yaw (Z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// Euler to quaternion (XYZ convention)
inline void euler2quat(float roll, float pitch, float yaw, float quat[4]) {
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    
    quat[0] = cr * cp * cy + sr * sp * sy;  // w
    quat[1] = sr * cp * cy - cr * sp * sy;  // x
    quat[2] = cr * sp * cy + sr * cp * sy;  // y
    quat[3] = cr * cp * sy - sr * sp * cy;  // z
}

class QuadrotorBridge {
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
    
    QuadrotorCommand last_cmd_;
    
    mjModel* mj_model_;
    mjData* mj_data_;
    
    std::thread state_publish_thread_;
    std::thread cmd_receive_thread_;
    
    // Quadrotor parameters for allocation matrix
    const double ARM_LENGTH = 0.1265;  // meters (distance from center to motor)
    const double K_TORQUE = 0.01;      // torque constant (Nm per N of thrust)
    
    // Convert OCS2 control [Fz, Mx, My, Mz] to 4 motor forces [F1, F2, F3, F4]
    void allocateMotorForces(const double control[4], double motor_forces[4]) {
        double Fz = control[0];
        double Mx = control[1];
        double My = control[2];
        double Mz = control[3];
        
        double L_inv = 1.0 / ARM_LENGTH;
        
        // Allocation matrix (X-configuration):
        // F1 = Fz/4 + 0*Mx      - My/(4*L) - Mz/(4*k)
        // F2 = Fz/4 - Mx/(4*L)  + 0*My     + Mz/(4*k)
        // F3 = Fz/4 + 0*Mx      + My/(4*L) - Mz/(4*k)
        // F4 = Fz/4 + Mx/(4*L)  + 0*My     + Mz/(4*k)
        
        // motor_forces[0] = 0.25 * Fz - 0.25 * My * L_inv - 0.25 * Mz / K_TORQUE;  // Motor 1 (Front, +X)
        // motor_forces[1] = 0.25 * Fz - 0.25 * Mx * L_inv + 0.25 * Mz / K_TORQUE;  // Motor 2 (Right, +Y)
        // motor_forces[2] = 0.25 * Fz + 0.25 * My * L_inv - 0.25 * Mz / K_TORQUE;  // Motor 3 (Back, -X)
        // motor_forces[3] = 0.25 * Fz + 0.25 * Mx * L_inv + 0.25 * Mz / K_TORQUE;  // Motor 4 (Left, -Y)

        motor_forces[0] = 0.25 * Fz + 0.25 * My * L_inv - 0.25 * Mz / K_TORQUE;  // Motor 1 (Front, +X)
        motor_forces[1] = 0.25 * Fz - 0.25 * Mx * L_inv + 0.25 * Mz / K_TORQUE;  // Motor 2 (Right, +Y)
        motor_forces[2] = 0.25 * Fz - 0.25 * My * L_inv - 0.25 * Mz / K_TORQUE;  // Motor 3 (Back, -X)
        motor_forces[3] = 0.25 * Fz + 0.25 * Mx * L_inv + 0.25 * Mz / K_TORQUE;  // Motor 4 (Left, -Y)

        
        // Clamp motor forces to valid range [0, 10] N
        for (int i = 0; i < 4; i++) {
            if (motor_forces[i] < 0.0) motor_forces[i] = 0.0;
            if (motor_forces[i] > 10.0) motor_forces[i] = 10.0;
        }
    }

    void PublishStateLoop() {
        printf("Quadrotor: State publish thread started (500 Hz)\n");
        
        // Get body index - MuJoCo API is mj_name2id(model, obj_type, name)
        int base_body_id = mj_name2id(mj_model_, mjOBJ_BODY, "base");
        if (base_body_id < 0) {
            printf("Quadrotor: WARNING - 'base' body not found, using body 1\n");
            base_body_id = 1;  // fallback to body 1 (usually the first dynamic body)
        }
        
        // Get joint address for qvel (freejoint has 6 DOF)
        int joint_id = mj_name2id(mj_model_, mjOBJ_JOINT, "base_joint");
        int qvel_addr = 0;
        if (joint_id >= 0) {
            qvel_addr = mj_model_->jnt_dofadr[joint_id];
        }
        printf("Quadrotor: Joint qvel address: %d\n", qvel_addr);
        
        while (running_) {
            if (mj_data_ && initialized_) {
                QuadrotorState state;
                state.timestamp = static_cast<uint64_t>(mj_data_->time * 1e9);
                
                // Position [0-2] - from body position (world frame)
                state.state[0] = mj_data_->xpos[base_body_id * 3 + 0];  // x
                state.state[1] = mj_data_->xpos[base_body_id * 3 + 1];  // y
                state.state[2] = mj_data_->xpos[base_body_id * 3 + 2];  // z
                
                // Quaternion -> Euler angles
                double roll, pitch, yaw;
                quat2euler(mj_data_->xquat + base_body_id * 4, roll, pitch, yaw);
                state.state[3] = roll;      // [3]
                state.state[4] = pitch;     // [4]
                state.state[5] = yaw;       // [5]
                
                // Velocity from qvel (generalized velocities)
                // For freejoint: qvel[0:3] = linear velocity (world frame)
                //                qvel[3:6] = angular velocity (body frame) - OCS2 needs this!
                state.state[6] = mj_data_->qvel[qvel_addr + 0];  // vx (world frame)
                state.state[7] = mj_data_->qvel[qvel_addr + 1];  // vy (world frame)
                state.state[8] = mj_data_->qvel[qvel_addr + 2];  // vz (world frame)
                
                // Angular velocity in body frame (what OCS2 expects)
                state.state[9]  = mj_data_->qvel[qvel_addr + 3];  // wx (body frame)
                state.state[10] = mj_data_->qvel[qvel_addr + 4];  // wy (body frame)
                state.state[11] = mj_data_->qvel[qvel_addr + 5];  // wz (body frame)
                
                // Send state
                int n = sendto(sock_state_, (const char*)&state, sizeof(state), 0,
                              (struct sockaddr*)&addr_state_, sizeof(addr_state_));
                if (n < 0) {
                    perror("sendto");
                }
                
                // Sleep to achieve 500 Hz
                std::this_thread::sleep_for(std::chrono::microseconds(2000));
            }
        }
        printf("Quadrotor: State publish thread ended\n");
    }

    void ReceiveCommandLoop() {
        printf("Quadrotor: Command receive thread started\n");
        printf("Quadrotor: Using DIRECT force application (like quadrotor_online)\n");
        
        // Get quadrotor body ID for direct force application (same as online version)
        int base_body_id = mj_name2id(mj_model_, mjOBJ_BODY, "base");
        if (base_body_id < 0) {
            // Try "quadrotor" as alternative name
            base_body_id = mj_name2id(mj_model_, mjOBJ_BODY, "quadrotor");
        }
        if (base_body_id < 0) {
            printf("Quadrotor: WARNING - Body not found, using body 1\n");
            base_body_id = 1;
        }
        printf("Quadrotor: Using body ID=%d for force application\n", base_body_id);
        
        // Flush old commands at startup
        printf("Quadrotor: Flushing old commands...\n");
        int flushed = 0;
        struct sockaddr_in dummy_addr;
        socklen_t dummy_len = sizeof(dummy_addr);
        char dummy_buf[256];
        while (recvfrom(sock_cmd_, dummy_buf, sizeof(dummy_buf), 0,
                       (struct sockaddr*)&dummy_addr, &dummy_len) > 0) {
            flushed++;
        }
        printf("Quadrotor: Flushed %d old command packets\n", flushed);
        
        while (running_) {
            struct sockaddr_in src_addr;
            socklen_t src_len = sizeof(src_addr);
            QuadrotorCommand cmd;
            QuadrotorCommand latest_cmd;
            bool has_cmd = false;
            int cmd_count = 0;
            
            // Read ALL available commands and only use the LATEST one
            while (true) {
                int n = recvfrom(sock_cmd_, (char*)&cmd, sizeof(cmd), MSG_DONTWAIT,
                               (struct sockaddr*)&src_addr, &src_len);
                if (n == sizeof(cmd)) {
                    latest_cmd = cmd;
                    has_cmd = true;
                    cmd_count++;
                } else {
                    break;
                }
            }
            
            if (has_cmd) {
                last_cmd_ = latest_cmd;
                
                if (cmd_count > 1) {
                    static int skip_warn = 0;
                    if (++skip_warn % 50 == 0) {
                        printf("[Bridge] Skipped %d old commands (delay detected)\n", cmd_count - 1);
                    }
                }
                
                // ============================================================
                // DIRECT force application (same as quadrotor_online)
                // Control: [Fz, Mx, My, Mz]
                // ============================================================
                double Fz = latest_cmd.input[0];  // Thrust force along body Z
                double Mx = latest_cmd.input[1];  // Roll moment
                double My = latest_cmd.input[2];  // Pitch moment
                double Mz = latest_cmd.input[3];  // Yaw moment
                
                // Get current quaternion for body-to-world transformation
                double qw = mj_data_->xquat[base_body_id * 4 + 0];
                double qx = mj_data_->xquat[base_body_id * 4 + 1];
                double qy = mj_data_->xquat[base_body_id * 4 + 2];
                double qz = mj_data_->xquat[base_body_id * 4 + 3];
                
                // Body z-axis in world frame (thrust direction)
                double bz_x = 2.0 * (qx * qz + qw * qy);
                double bz_y = 2.0 * (qy * qz - qw * qx);
                double bz_z = 1.0 - 2.0 * (qx * qx + qy * qy);
                
                // Apply thrust force along body Z axis (in world frame)
                mj_data_->xfrc_applied[6 * base_body_id + 0] = Fz * bz_x;
                mj_data_->xfrc_applied[6 * base_body_id + 1] = Fz * bz_y;
                mj_data_->xfrc_applied[6 * base_body_id + 2] = Fz * bz_z;
                
                // Rotation matrix for torque transformation (body to world)
                double R00 = 1.0 - 2.0 * (qy * qy + qz * qz);
                double R01 = 2.0 * (qx * qy - qz * qw);
                double R02 = 2.0 * (qx * qz + qy * qw);
                double R10 = 2.0 * (qx * qy + qz * qw);
                double R11 = 1.0 - 2.0 * (qx * qx + qz * qz);
                double R12 = 2.0 * (qy * qz - qx * qw);
                double R20 = 2.0 * (qx * qz - qy * qw);
                double R21 = 2.0 * (qy * qz + qx * qw);
                double R22 = 1.0 - 2.0 * (qx * qx + qy * qy);
                
                // Apply torques transformed to world frame
                mj_data_->xfrc_applied[6 * base_body_id + 3] = R00 * Mx + R01 * My + R02 * Mz;
                mj_data_->xfrc_applied[6 * base_body_id + 4] = R10 * Mx + R11 * My + R12 * Mz;
                mj_data_->xfrc_applied[6 * base_body_id + 5] = R20 * Mx + R21 * My + R22 * Mz;
                
                // Debug print every 100 commands
                static int print_counter = 0;
                if (++print_counter % 100 == 0) {
                    printf("[Bridge] Direct: Fz=%.3f Mx=%.4f My=%.4f Mz=%.4f\n",
                           Fz, Mx, My, Mz);
                    printf("[Bridge] World Force: Fx=%.3f Fy=%.3f Fz=%.3f\n",
                           Fz * bz_x, Fz * bz_y, Fz * bz_z);
                }
            }
            
            // Small sleep to avoid busy loop
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        printf("Quadrotor: Command receive thread ended\n");
    }

public:
    QuadrotorBridge() : sock_state_(-1), sock_cmd_(-1), initialized_(false), 
                        running_(false), mj_model_(nullptr), mj_data_(nullptr) {
        std::memset(&last_cmd_, 0, sizeof(last_cmd_));
    }

    ~QuadrotorBridge() {
        Shutdown();
    }

    bool Initialize(mjModel* m, mjData* d) {
        mj_model_ = m;
        mj_data_ = d;
        
        // Create state socket (send)
        sock_state_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_state_ < 0) {
            perror("socket");
            return false;
        }
        
        std::memset(&addr_state_, 0, sizeof(addr_state_));
        addr_state_.sin_family = AF_INET;
        addr_state_.sin_port = htons(STATE_PORT);
        inet_aton(CONTROLLER_IP, &addr_state_.sin_addr);
        
        // Create command socket (receive)
        sock_cmd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_cmd_ < 0) {
            perror("socket");
            close(sock_state_);
            return false;
        }
        
        // Set small receive buffer to avoid delay (only keep ~5 packets)
        int rcvbuf_size = 256;  // Small buffer for real-time
        setsockopt(sock_cmd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size));
        
        // Set non-blocking
        int flags = fcntl(sock_cmd_, F_GETFL, 0);
        fcntl(sock_cmd_, F_SETFL, flags | O_NONBLOCK);
        
        struct sockaddr_in addr_bind;
        std::memset(&addr_bind, 0, sizeof(addr_bind));
        addr_bind.sin_family = AF_INET;
        addr_bind.sin_port = htons(CMD_PORT);
        addr_bind.sin_addr.s_addr = INADDR_ANY;
        
        if (bind(sock_cmd_, (struct sockaddr*)&addr_bind, sizeof(addr_bind)) < 0) {
            perror("bind");
            close(sock_state_);
            close(sock_cmd_);
            return false;
        }
        
        initialized_ = true;
        running_ = true;
        
        state_publish_thread_ = std::thread(&QuadrotorBridge::PublishStateLoop, this);
        cmd_receive_thread_ = std::thread(&QuadrotorBridge::ReceiveCommandLoop, this);
        
        printf("Quadrotor Bridge initialized\n");
        printf("  State broadcast to %s:%d\n", CONTROLLER_IP, STATE_PORT);
        printf("  Command listen on port %d\n", CMD_PORT);
        
        return true;
    }

    void Shutdown() {
        running_ = false;
        
        if (state_publish_thread_.joinable()) {
            state_publish_thread_.join();
        }
        if (cmd_receive_thread_.joinable()) {
            cmd_receive_thread_.join();
        }
        
        if (sock_state_ >= 0) close(sock_state_);
        if (sock_cmd_ >= 0) close(sock_cmd_);
        
        printf("Quadrotor Bridge shutdown\n");
    }
};

#endif  // QUADROTOR_BRIDGE_H
