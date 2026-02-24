#ifndef QUADRUPED_BRIDGE_H
#define QUADRUPED_BRIDGE_H

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
#include <mutex>
#include <mujoco/mujoco.h>
#include <cmath>
#include <Eigen/Dense>

/**
 * @brief UDP Bridge for Quadruped Robot (ANYmal-style)
 * 
 * Bridge sends RAW MuJoCo data - controller does all mapping!
 * 
 * State format (36D RBD state):
 *   [euler(3), pos(3), jointPos(12), angVel_world(3), linVel(3), jointVel(12)]
 *   - euler: roll, pitch, yaw (rad)
 *   - pos: base position x, y, z (m)
 *   - jointPos: 12 joint angles (rad) - MuJoCo order: LF, RF, LH, RH
 *   - angVel_world: angular velocity in WORLD frame (rad/s)
 *   - linVel: linear velocity in world frame (m/s)
 *   - jointVel: 12 joint velocities (rad/s) - MuJoCo order
 * 
 * Command format (12D joint torques):
 *   [tau_0, tau_1, ..., tau_11] (Nm) - MuJoCo order
 * 
 * MuJoCo/URDF Joint order:
 *   LF: HAA(0), HFE(1), KFE(2)
 *   RF: HAA(3), HFE(4), KFE(5)
 *   LH: HAA(6), HFE(7), KFE(8)
 *   RH: HAA(9), HFE(10), KFE(11)
 * 
 * NOTE: Controller maps MuJoCo<->OCS2 order like anymal_walking_online
 */

// State packet: 36D RBD state + timestamp
struct QuadrupedState {
    uint64_t timestamp;     // nanoseconds
    double state[36];       // RBD state
} __attribute__((packed));

// Command packet: 12 joint torques + timestamp
struct QuadrupedCommand {
    uint64_t timestamp;     // nanoseconds
    double torque[12];      // joint torques (Nm)
} __attribute__((packed));

// Quaternion to Euler angles (ZYX convention - same as OCS2)
inline void quatToEulerZYX(const mjtNum quat[4], double& roll, double& pitch, double& yaw) {
    double qw = quat[0];
    double qx = quat[1];
    double qy = quat[2];
    double qz = quat[3];
    
    // Roll (X-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (Y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0)
        pitch = std::copysign(M_PI / 2.0, sinp);
    else
        pitch = std::asin(sinp);
    
    // Yaw (Z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

// Convert body frame angular velocity to world frame
inline void bodyToWorldAngVel(const mjtNum quat[4], 
                               double wx_body, double wy_body, double wz_body,
                               double& wx_world, double& wy_world, double& wz_world) {
    double qw = quat[0];
    double qx = quat[1];
    double qy = quat[2];
    double qz = quat[3];
    
    // Rotation matrix from body to world (quaternion to rotation matrix)
    double R00 = 1.0 - 2.0 * (qy * qy + qz * qz);
    double R01 = 2.0 * (qx * qy - qz * qw);
    double R02 = 2.0 * (qx * qz + qy * qw);
    double R10 = 2.0 * (qx * qy + qz * qw);
    double R11 = 1.0 - 2.0 * (qx * qx + qz * qz);
    double R12 = 2.0 * (qy * qz - qx * qw);
    double R20 = 2.0 * (qx * qz - qy * qw);
    double R21 = 2.0 * (qy * qz + qx * qw);
    double R22 = 1.0 - 2.0 * (qx * qx + qy * qy);
    
    // omega_world = R * omega_body
    wx_world = R00 * wx_body + R01 * wy_body + R02 * wz_body;
    wy_world = R10 * wx_body + R11 * wy_body + R12 * wz_body;
    wz_world = R20 * wx_body + R21 * wy_body + R22 * wz_body;
}

class QuadrupedBridge {
private:
    int sock_state_;
    int sock_cmd_;
    struct sockaddr_in addr_state_;
    struct sockaddr_in addr_cmd_;
    
    bool initialized_;
    std::atomic<bool> running_;
    
    const int STATE_PORT = 9101;    // Different from quadrotor (9001)
    const int CMD_PORT = 9102;      // Different from quadrotor (9002)
    const char* CONTROLLER_IP = "127.0.0.1";
    
    QuadrupedCommand last_cmd_;
    std::mutex cmd_mutex_;
    
    mjModel* mj_model_;
    mjData* mj_data_;
    
    std::thread state_publish_thread_;
    std::thread cmd_receive_thread_;
    
    // Joint mapping from MuJoCo actuator index to OCS2 convention
    // ANYmal joint order: LF_HAA, LF_HFE, LF_KFE, RF_HAA, RF_HFE, RF_KFE,
    //                     LH_HAA, LH_HFE, LH_KFE, RH_HAA, RH_HFE, RH_KFE
    std::vector<int> mujoco_to_ocs2_joint_;  // MuJoCo actuator idx -> OCS2 joint idx
    std::vector<int> ocs2_to_mujoco_joint_;  // OCS2 joint idx -> MuJoCo actuator idx
    
    // Sensor indices for joint position/velocity (if using sensors)
    int base_body_id_ = -1;
    int base_joint_id_ = -1;
    int qvel_addr_ = 0;
    int qpos_addr_ = 0;
    
    // Continuous yaw tracking
    double continuous_yaw_ = 0.0;
    bool yaw_initialized_ = false;
    
    double unwrapYaw(double newYaw, double prevYaw) {
        double diff = newYaw - prevYaw;
        while (diff > M_PI) { newYaw -= 2.0 * M_PI; diff = newYaw - prevYaw; }
        while (diff < -M_PI) { newYaw += 2.0 * M_PI; diff = newYaw - prevYaw; }
        return newYaw;
    }
    
    void BuildJointMapping() {
        // OCS2 legged_robot joint order (from reference.info):
        // LF(0-2), LH(3-5), RF(6-8), RH(9-11)
        const std::vector<std::string> ocs2_joint_names = {
            "LF_HAA", "LF_HFE", "LF_KFE",   // 0-2
            "LH_HAA", "LH_HFE", "LH_KFE",   // 3-5 (not RF!)
            "RF_HAA", "RF_HFE", "RF_KFE",   // 6-8 (not LH!)
            "RH_HAA", "RH_HFE", "RH_KFE"    // 9-11
        };
        
        mujoco_to_ocs2_joint_.resize(mj_model_->nu, -1);
        ocs2_to_mujoco_joint_.resize(12, -1);
        
        for (int mj_idx = 0; mj_idx < mj_model_->nu; ++mj_idx) {
            const char* name = mj_id2name(mj_model_, mjOBJ_ACTUATOR, mj_idx);
            if (!name) continue;
            
            std::string actuator_name(name);
            
            // Find matching OCS2 joint
            for (size_t ocs2_idx = 0; ocs2_idx < ocs2_joint_names.size(); ++ocs2_idx) {
                if (actuator_name.find(ocs2_joint_names[ocs2_idx]) != std::string::npos) {
                    mujoco_to_ocs2_joint_[mj_idx] = ocs2_idx;
                    ocs2_to_mujoco_joint_[ocs2_idx] = mj_idx;
                    printf("  Joint mapping: MuJoCo[%d](%s) -> OCS2[%zu](%s)\n",
                           mj_idx, name, ocs2_idx, ocs2_joint_names[ocs2_idx].c_str());
                    break;
                }
            }
        }
        
        // Check for unmapped joints
        for (size_t i = 0; i < 12; ++i) {
            if (ocs2_to_mujoco_joint_[i] < 0) {
                printf("  WARNING: OCS2 joint %zu (%s) not mapped!\n", 
                       i, ocs2_joint_names[i].c_str());
            }
        }
    }
    
    void PublishStateLoop() {
        printf("Quadruped: State publish thread started (500 Hz)\n");
        
        // Get base body and joint info
        base_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "base");
        if (base_body_id_ < 0) {
            base_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "trunk");
        }
        if (base_body_id_ < 0) {
            printf("Quadruped: WARNING - base body not found, using body 1\n");
            base_body_id_ = 1;
        }
        
        // Get freejoint address for velocities
        for (int i = 0; i < mj_model_->njnt; ++i) {
            if (mj_model_->jnt_type[i] == mjJNT_FREE) {
                qvel_addr_ = mj_model_->jnt_dofadr[i];
                qpos_addr_ = mj_model_->jnt_qposadr[i];
                printf("Quadruped: Found freejoint at qpos=%d, qvel=%d\n", qpos_addr_, qvel_addr_);
                break;
            }
        }
        
        while (running_) {
            if (mj_data_ && initialized_) {
                QuadrupedState state;
                state.timestamp = static_cast<uint64_t>(mj_data_->time * 1e9);
                
                // ========================================================
                // Part 1: Base orientation (Euler angles) - indices 0-2
                // ========================================================
                double roll, pitch, yaw;
                quatToEulerZYX(mj_data_->xquat + base_body_id_ * 4, roll, pitch, yaw);
                
                // Apply continuous yaw tracking
                if (!yaw_initialized_) {
                    continuous_yaw_ = yaw;
                    yaw_initialized_ = true;
                } else {
                    continuous_yaw_ = unwrapYaw(yaw, continuous_yaw_);
                }
                
                state.state[0] = roll;
                state.state[1] = pitch;
                state.state[2] = continuous_yaw_;
                
                // ========================================================
                // Part 2: Base position - indices 3-5
                // ========================================================
                state.state[3] = mj_data_->xpos[base_body_id_ * 3 + 0];  // x
                state.state[4] = mj_data_->xpos[base_body_id_ * 3 + 1];  // y
                state.state[5] = mj_data_->xpos[base_body_id_ * 3 + 2];  // z
                
                // ========================================================
                // Part 3: Joint positions - indices 6-17 (MuJoCo order)
                // MuJoCo order: LF(0-2), RF(3-5), LH(6-8), RH(9-11)
                // ========================================================
                for (int mj_idx = 0; mj_idx < 12; ++mj_idx) {
                    int joint_id = mj_model_->actuator_trnid[mj_idx * 2];
                    int jnt_qpos = mj_model_->jnt_qposadr[joint_id];
                    state.state[6 + mj_idx] = mj_data_->qpos[jnt_qpos];
                }
                
                // ========================================================
                // Part 4: Base angular velocity (WORLD frame!) - indices 18-20
                // ========================================================
                double wx_body = mj_data_->qvel[qvel_addr_ + 3];
                double wy_body = mj_data_->qvel[qvel_addr_ + 4];
                double wz_body = mj_data_->qvel[qvel_addr_ + 5];
                
                // Convert body frame to world frame (critical for OCS2!)
                double wx_world, wy_world, wz_world;
                bodyToWorldAngVel(mj_data_->xquat + base_body_id_ * 4,
                                  wx_body, wy_body, wz_body,
                                  wx_world, wy_world, wz_world);
                
                state.state[18] = wx_world;
                state.state[19] = wy_world;
                state.state[20] = wz_world;
                
                // ========================================================
                // Part 5: Base linear velocity - indices 21-23
                // ========================================================
                state.state[21] = mj_data_->qvel[qvel_addr_ + 0];  // vx
                state.state[22] = mj_data_->qvel[qvel_addr_ + 1];  // vy
                state.state[23] = mj_data_->qvel[qvel_addr_ + 2];  // vz
                
                // ========================================================
                // Part 6: Joint velocities - indices 24-35 (MuJoCo order)
                // ========================================================
                for (int mj_idx = 0; mj_idx < 12; ++mj_idx) {
                    int joint_id = mj_model_->actuator_trnid[mj_idx * 2];
                    int jnt_dof = mj_model_->jnt_dofadr[joint_id];
                    state.state[24 + mj_idx] = mj_data_->qvel[jnt_dof];
                }
                
                // Send state
                int n = sendto(sock_state_, (const char*)&state, sizeof(state), 0,
                              (struct sockaddr*)&addr_state_, sizeof(addr_state_));
                if (n < 0) {
                    perror("sendto");
                }
                
                // 500 Hz
                std::this_thread::sleep_for(std::chrono::microseconds(2000));
            }
        }
        printf("Quadruped: State publish thread ended\n");
    }
    
    void ReceiveCommandLoop() {
        printf("Quadruped: Command receive thread started\n");
        
        // Flush old commands
        printf("Quadruped: Flushing old commands...\n");
        int flushed = 0;
        struct sockaddr_in dummy_addr;
        socklen_t dummy_len = sizeof(dummy_addr);
        char dummy_buf[256];
        while (recvfrom(sock_cmd_, dummy_buf, sizeof(dummy_buf), 0,
                       (struct sockaddr*)&dummy_addr, &dummy_len) > 0) {
            flushed++;
        }
        printf("Quadruped: Flushed %d old command packets\n", flushed);
        
        while (running_) {
            struct sockaddr_in src_addr;
            socklen_t src_len = sizeof(src_addr);
            QuadrupedCommand cmd;
            QuadrupedCommand latest_cmd;
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
                {
                    std::lock_guard<std::mutex> lock(cmd_mutex_);
                    last_cmd_ = latest_cmd;
                }
                
                if (cmd_count > 1) {
                    static int skip_warn = 0;
                    if (++skip_warn % 50 == 0) {
                        printf("[QuadrupedBridge] Skipped %d old commands\n", cmd_count - 1);
                    }
                }
                
                // Apply torques to MuJoCo actuators (MuJoCo order directly)
                for (int mj_idx = 0; mj_idx < 12 && mj_idx < mj_model_->nu; ++mj_idx) {
                    mj_data_->ctrl[mj_idx] = latest_cmd.torque[mj_idx];
                }
                
                // Debug print every 500 commands (~1 second at 500Hz)
                static int print_counter = 0;
                if (++print_counter % 500 == 0) {
                    printf("[QuadrupedBridge] Torques: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
                           latest_cmd.torque[0], latest_cmd.torque[1], latest_cmd.torque[2],
                           latest_cmd.torque[3], latest_cmd.torque[4], latest_cmd.torque[5],
                           latest_cmd.torque[6], latest_cmd.torque[7], latest_cmd.torque[8],
                           latest_cmd.torque[9], latest_cmd.torque[10], latest_cmd.torque[11]);
                }
            }
            
            // Small sleep
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        printf("Quadruped: Command receive thread ended\n");
    }

public:
    QuadrupedBridge() : sock_state_(-1), sock_cmd_(-1), initialized_(false),
                        running_(false), mj_model_(nullptr), mj_data_(nullptr) {
        std::memset(&last_cmd_, 0, sizeof(last_cmd_));
    }
    
    ~QuadrupedBridge() {
        Shutdown();
    }
    
    bool Initialize(mjModel* m, mjData* d) {
        mj_model_ = m;
        mj_data_ = d;
        
        printf("\n========================================\n");
        printf("  Quadruped UDP Bridge Initialization\n");
        printf("========================================\n");
        
        // Build joint mapping
        printf("Building joint mapping...\n");
        BuildJointMapping();
        
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
        
        // Set small receive buffer
        int rcvbuf_size = 256;
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
        
        state_publish_thread_ = std::thread(&QuadrupedBridge::PublishStateLoop, this);
        cmd_receive_thread_ = std::thread(&QuadrupedBridge::ReceiveCommandLoop, this);
        
        printf("\nQuadruped Bridge initialized successfully!\n");
        printf("  State broadcast to %s:%d\n", CONTROLLER_IP, STATE_PORT);
        printf("  Command listen on port %d\n", CMD_PORT);
        printf("  State format: 36D RBD [euler(3), pos(3), joints(12), angVel(3), linVel(3), jointVel(12)]\n");
        printf("  Command format: 12D torques\n");
        printf("========================================\n\n");
        
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
        
        printf("Quadruped Bridge shutdown\n");
    }
    
    // Get last received command (for debugging)
    QuadrupedCommand getLastCommand() {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        return last_cmd_;
    }
};

#endif  // QUADRUPED_BRIDGE_H
