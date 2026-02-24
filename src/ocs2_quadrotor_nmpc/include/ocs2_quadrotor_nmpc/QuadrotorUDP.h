#pragma once

#include <cstdint>
#include <cstring>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2_cartpole_quadrotor {

// UDP Communication for Quadrotor
// RX: State from simulator (24 bytes header + 48 bytes state = 72 bytes)
// TX: Command to simulator (24 bytes header + 16 bytes input = 40 bytes)

class QuadrotorUDP {
 public:
  QuadrotorUDP();
  ~QuadrotorUDP();

  // Initialize UDP sockets
  // rxPort: port to receive state from simulator (9001)
  // txPort: port to send command to simulator (9002)
  void initialize(int rxPort = 9001, int txPort = 9002);

  // Receive state from simulator
  // Returns true if data received successfully
  bool receiveState(double& time,
                    ocs2::vector_t& state);

  // Send command to simulator
  void sendCommand(double time, const ocs2::vector_t& input);

  void shutdown();

 private:
  struct StatePacket {
    uint64_t timestamp;  // 8 bytes
    double state[12];    // 96 bytes (12 doubles)
  } __attribute__((packed));

  struct CommandPacket {
    uint64_t timestamp;  // 8 bytes
    double input[4];     // 32 bytes (4 doubles)
  } __attribute__((packed));

  int rxSocket_ = -1;
  int txSocket_ = -1;
  int rxPort_ = 9001;
  int txPort_ = 9002;

  // Buffer for receiving
  std::vector<uint8_t> rxBuffer_;
  
  // Statistics
  uint64_t rxCount_ = 0;
  uint64_t txCount_ = 0;
};

}  // namespace ocs2_cartpole_quadrotor
