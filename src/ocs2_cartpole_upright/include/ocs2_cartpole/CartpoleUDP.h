#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace ocs2_cartpole {

/**
 * CartPole UDP Interface for communication with MuJoCo simulator
 * 
 * State Reception: Listening on 0.0.0.0:9001
 *   - CartPoleStatePacket (24 bytes)
 *   - Fields: cart_pos, pole_angle, cart_vel, pole_anglevel, timestamp, crc32
 * 
 * Command Transmission: Sending to 127.0.0.1:9002
 *   - CartPoleCommandPacket (12 bytes)
 *   - Fields: force, timestamp, crc32
 */

constexpr int STATE_PORT = 9001;
constexpr int CMD_PORT = 9002;
constexpr int STATE_BUFFER_SIZE = 24;  // 5 floats + 1 uint32_t
constexpr int CMD_BUFFER_SIZE = 12;    // 2 floats + 1 uint32_t

/**
 * CartPole state packet structure
 */
struct CartPoleState {
  float cart_pos;          // [m] cart position
  float pole_angle;        // [rad] pole angle from vertical
  float cart_vel;          // [m/s] cart velocity
  float pole_anglevel;     // [rad/s] pole angular velocity
  float timestamp;         // [s] simulation timestamp
  uint32_t crc;            // CRC32 checksum

  CartPoleState() : cart_pos(0.0f), pole_angle(0.0f), cart_vel(0.0f), 
                    pole_anglevel(0.0f), timestamp(0.0f), crc(0) {}
};

/**
 * CartPole command packet structure
 */
struct CartPoleCommand {
  float force;             // [N] applied force
  float timestamp;         // [s] command timestamp
  uint32_t crc;            // CRC32 checksum

  CartPoleCommand(float f = 0.0f, float t = 0.0f) 
    : force(f), timestamp(t), crc(0) {}
};

/**
 * CartPole UDP Interface Class
 */
class CartPoleUDPInterface {
 public:
  CartPoleUDPInterface();
  ~CartPoleUDPInterface();

  /**
   * Initialize UDP sockets
   * @param state_port Port for receiving state (default 9001)
   * @param cmd_port Port for sending commands (default 9002)
   * @param controller_ip IP address to send commands to (default 127.0.0.1)
   * @return true if initialization successful
   */
  bool initialize(int state_port = STATE_PORT, int cmd_port = CMD_PORT,
                  const std::string& controller_ip = "127.0.0.1");

  /**
   * Receive state from simulator
   * @param state Output state structure
   * @param timeout_ms Receive timeout in milliseconds
   * @return true if state received successfully
   */
  bool receiveState(CartPoleState& state, int timeout_ms = 1000);

  /**
   * Send command to simulator
   * @param command Command to send
   * @return true if sent successfully
   */
  bool sendCommand(const CartPoleCommand& command);

  /**
   * Close UDP connections
   */
  void close();

  /**
   * Check if interface is initialized
   */
  bool isInitialized() const { return initialized_; }

  /**
   * Get number of states received
   */
  uint32_t getStateCount() const { return state_count_; }

 private:
  /**
   * Calculate CRC32 checksum
   */
  static uint32_t calculateCRC32(const uint8_t* data, size_t length);

  /**
   * Normalize angle to [-π, π]
   */
  static float normalizeAngle(float angle);

  int state_sock_;
  int cmd_sock_;
  bool initialized_;
  uint32_t state_count_;
  std::string controller_ip_;
  int cmd_port_;
};

}  // namespace ocs2_cartpole
