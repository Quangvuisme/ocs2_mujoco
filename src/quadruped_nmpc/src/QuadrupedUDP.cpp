#include "quadruped_nmpc/QuadrupedUDP.h"
#include <cmath>

namespace quadruped_nmpc {

QuadrupedUDP::QuadrupedUDP() : rxBuffer_(512) {}

QuadrupedUDP::~QuadrupedUDP() { shutdown(); }

bool QuadrupedUDP::initialize(int rxPort, int txPort) {
    rxPort_ = rxPort;
    txPort_ = txPort;
    
    // Create RX socket
    rxSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (rxSocket_ < 0) {
        std::cerr << "[QuadrupedUDP] Failed to create RX socket\n";
        return false;
    }
    
    // Non-blocking
    int flags = fcntl(rxSocket_, F_GETFL, 0);
    fcntl(rxSocket_, F_SETFL, flags | O_NONBLOCK);
    
    // Bind
    struct sockaddr_in rxAddr;
    std::memset(&rxAddr, 0, sizeof(rxAddr));
    rxAddr.sin_family = AF_INET;
    rxAddr.sin_addr.s_addr = INADDR_ANY;
    rxAddr.sin_port = htons(rxPort_);
    
    if (bind(rxSocket_, (struct sockaddr*)&rxAddr, sizeof(rxAddr)) < 0) {
        std::cerr << "[QuadrupedUDP] Failed to bind on port " << rxPort_ << "\n";
        close(rxSocket_);
        rxSocket_ = -1;
        return false;
    }
    
    // TX socket
    txSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (txSocket_ < 0) {
        std::cerr << "[QuadrupedUDP] Failed to create TX socket\n";
        close(rxSocket_);
        rxSocket_ = -1;
        return false;
    }
    
    // Setup TX address
    std::memset(&txAddr_, 0, sizeof(txAddr_));
    txAddr_.sin_family = AF_INET;
    txAddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
    txAddr_.sin_port = htons(txPort_);
    
    // Small buffer to avoid delay
    int recv_buf_size = 4096;
    setsockopt(rxSocket_, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size));
    
    // Flush old data
    std::cout << "[QuadrupedUDP] Flushing old packets...\n";
    int flushed = 0;
    auto flush_start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - flush_start).count() < 0.5) {
        char dummy[512];
        int n = recvfrom(rxSocket_, dummy, sizeof(dummy), 0, nullptr, nullptr);
        if (n > 0) flushed++;
        else if (flushed > 0) break;
    }
    std::cout << "[QuadrupedUDP] Flushed " << flushed << " packets\n";
    
    std::cout << "[QuadrupedUDP] Initialized: RX=" << rxPort_ << ", TX=" << txPort_ << "\n";
    std::cout << "[QuadrupedUDP] RawStatePacket size: " << sizeof(RawStatePacket) << " bytes\n";
    std::cout << "[QuadrupedUDP] RawCommandPacket size: " << sizeof(RawCommandPacket) << " bytes\n";
    return true;
}

bool QuadrupedUDP::receiveState(StatePacket& pkt) {
    if (rxSocket_ < 0) return false;
    
    struct sockaddr_in srcAddr;
    socklen_t srcLen = sizeof(srcAddr);
    
    int n = recvfrom(rxSocket_, rxBuffer_.data(), rxBuffer_.size(), 0,
                     (struct sockaddr*)&srcAddr, &srcLen);
    
    if (n != sizeof(RawStatePacket)) {
        // Debug: print actual received size periodically
        static int debug_count = 0;
        if (n > 0 && debug_count++ % 1000 == 0) {
            std::cerr << "[QuadrupedUDP] Received " << n << " bytes, expected " 
                      << sizeof(RawStatePacket) << "\n";
        }
        return false;
    }
    
    RawStatePacket* raw = reinterpret_cast<RawStatePacket*>(rxBuffer_.data());
    
    // Convert nanoseconds to seconds
    pkt.timestamp = static_cast<double>(raw->timestamp) / 1e9;
    
    // Euler angles (from bridge: roll=0, pitch=1, yaw=2)
    pkt.roll = raw->state[0];
    pkt.pitch = raw->state[1];
    pkt.yaw = raw->state[2];
    
    // Position (from bridge: x=3, y=4, z=5)
    pkt.x = raw->state[3];
    pkt.y = raw->state[4];
    pkt.z = raw->state[5];
    
    // Joint positions (from bridge: indices 6-17) - MuJoCo order: LF, RF, LH, RH
    // NOTE: main.cpp maps MuJoCo->OCS2 in statePacketToRbdState()
    for (int i = 0; i < 12; i++) {
        pkt.jointPos[i] = raw->state[6 + i];
    }
    
    // Angular velocity already in world frame from bridge (indices 18-20)
    pkt.angVelWorld[0] = raw->state[18];
    pkt.angVelWorld[1] = raw->state[19];
    pkt.angVelWorld[2] = raw->state[20];
    
    // Linear velocity (from bridge: indices 21-23)
    pkt.linVel[0] = raw->state[21];
    pkt.linVel[1] = raw->state[22];
    pkt.linVel[2] = raw->state[23];
    
    // Joint velocities (from bridge: indices 24-35) - MuJoCo order: LF, RF, LH, RH
    // NOTE: main.cpp maps MuJoCo->OCS2 in statePacketToRbdState()
    for (int i = 0; i < 12; i++) {
        pkt.jointVel[i] = raw->state[24 + i];
    }
    
    rxCount_++;
    
    // Debug print every 500 packets
    if (rxCount_ % 500 == 0) {
        std::cout << "[QuadrupedUDP] RX#" << rxCount_ 
                  << " t=" << pkt.timestamp 
                  << " z=" << pkt.z 
                  << " yaw=" << pkt.yaw << "\n";
    }
    
    return true;
}

void QuadrupedUDP::sendCommand(const CommandPacket& cmd) {
    if (txSocket_ < 0) return;
    
    RawCommandPacket raw;
    // Convert seconds to nanoseconds
    raw.timestamp = static_cast<uint64_t>(cmd.timestamp * 1e9);
    
    // Torques are in MuJoCo order - bridge expects MuJoCo order: LF, RF, LH, RH
    // NOTE: main.cpp maps OCS2->MuJoCo in extractJointTorques() before calling sendCommand()
    for (int i = 0; i < 12; i++) {
        raw.torque[i] = cmd.jointTorques[i];
    }
    
    int sent = sendto(txSocket_, &raw, sizeof(raw), 0,
                      (struct sockaddr*)&txAddr_, sizeof(txAddr_));
    
    if (sent > 0) {
        txCount_++;
    }
    
    // Debug print every 500 packets
    if (txCount_ % 500 == 0) {
        std::cout << "[QuadrupedUDP] TX#" << txCount_ 
                  << " torques=[" << cmd.jointTorques[0] << ", " 
                  << cmd.jointTorques[1] << ", " << cmd.jointTorques[2] << "...]\n";
    }
}

void QuadrupedUDP::shutdown() {
    std::cout << "[QuadrupedUDP] Shutdown (RX=" << rxCount_ << ", TX=" << txCount_ << ")\n";
    if (rxSocket_ >= 0) { close(rxSocket_); rxSocket_ = -1; }
    if (txSocket_ >= 0) { close(txSocket_); txSocket_ = -1; }
}

}  // namespace quadruped_nmpc
