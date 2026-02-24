#pragma once

#include <cstdint>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

namespace quadruped_nmpc {

/**
 * @brief State packet from MuJoCo bridge
 * Format MUST match QuadrupedState in quadruped_bridge.h
 * 
 * Bridge format:
 *   uint64_t timestamp;     // nanoseconds
 *   double state[36];       // RBD state
 * 
 * state[36] layout:
 *   [0-2]:   euler (roll, pitch, yaw)
 *   [3-5]:   pos (x, y, z)
 *   [6-17]:  jointPos (12 joints) - MuJoCo order: LF, RF, LH, RH
 *   [18-20]: angVel_world (wx, wy, wz) 
 *   [21-23]: linVel (vx, vy, vz)
 *   [24-35]: jointVel (12 joints) - MuJoCo order: LF, RF, LH, RH
 * 
 * MuJoCo/URDF joint order:
 *   LF: HAA(0), HFE(1), KFE(2)
 *   RF: HAA(3), HFE(4), KFE(5)
 *   LH: HAA(6), HFE(7), KFE(8)
 *   RH: HAA(9), HFE(10), KFE(11)
 * 
 * OCS2 joint order (for MPC):
 *   LF: HAA(0), HFE(1), KFE(2)
 *   LH: HAA(3), HFE(4), KFE(5)
 *   RF: HAA(6), HFE(7), KFE(8)
 *   RH: HAA(9), HFE(10), KFE(11)
 * 
 * NOTE: Bridge sends MuJoCo order. Controller maps MuJoCo<->OCS2!
 */

// Raw packet matching bridge exactly
struct RawStatePacket {
    uint64_t timestamp;     // nanoseconds
    double state[36];       // RBD state
} __attribute__((packed));

// Raw command packet matching bridge exactly  
struct RawCommandPacket {
    uint64_t timestamp;     // nanoseconds
    double torque[12];      // joint torques
} __attribute__((packed));

// Parsed state with convenient fields (MuJoCo order from bridge)
struct StatePacket {
    double timestamp;           // seconds
    double roll, pitch, yaw;    // euler angles (rad)
    double x, y, z;             // base position (m)
    double jointPos[12];        // joint positions (rad) - MuJoCo order: LF, RF, LH, RH
    double angVelWorld[3];      // angular velocity world frame (rad/s)
    double linVel[3];           // linear velocity world frame (m/s)
    double jointVel[12];        // joint velocities (rad/s) - MuJoCo order: LF, RF, LH, RH
};

// Command packet (MuJoCo order to bridge)
struct CommandPacket {
    double timestamp;           // seconds
    double jointTorques[12];    // joint torques (Nm) - MuJoCo order: LF, RF, LH, RH
};

/**
 * @brief UDP Communication class for Quadruped NMPC
 */
class QuadrupedUDP {
public:
    QuadrupedUDP();
    ~QuadrupedUDP();
    
    bool initialize(int rxPort = 9101, int txPort = 9102);
    bool receiveState(StatePacket& pkt);
    void sendCommand(const CommandPacket& cmd);
    void shutdown();
    
    int getRxCount() const { return rxCount_; }
    int getTxCount() const { return txCount_; }
    
private:
    int rxSocket_ = -1;
    int txSocket_ = -1;
    int rxPort_ = 9101;
    int txPort_ = 9102;
    struct sockaddr_in txAddr_;
    
    std::vector<char> rxBuffer_;
    int rxCount_ = 0;
    int txCount_ = 0;
};

}  // namespace quadruped_nmpc
