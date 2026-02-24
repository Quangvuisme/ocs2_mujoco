#include "ocs2_cartpole/CartpoleUDP.h"

#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <math.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ocs2_cartpole {

// CRC32 lookup table
static uint32_t crc32_table[256];

void init_crc32_table() {
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t crc = i;
    for (int j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xEDB88320;
      } else {
        crc >>= 1;
      }
    }
    crc32_table[i] = crc;
  }
}

uint32_t CartPoleUDPInterface::calculateCRC32(const uint8_t* data,
                                               size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; i++) {
    uint8_t index = (crc ^ data[i]) & 0xFF;
    crc = (crc >> 8) ^ crc32_table[index];
  }
  return crc ^ 0xFFFFFFFF;
}

float CartPoleUDPInterface::normalizeAngle(float angle) {
  constexpr float PI = 3.14159265359f;
  while (angle > PI) {
    angle -= 2.0f * PI;
  }
  while (angle < -PI) {
    angle += 2.0f * PI;
  }
  return angle;
}

CartPoleUDPInterface::CartPoleUDPInterface()
    : state_sock_(-1), cmd_sock_(-1), initialized_(false), 
      state_count_(0), controller_ip_("127.0.0.1"), cmd_port_(CMD_PORT) {
  init_crc32_table();
}

CartPoleUDPInterface::~CartPoleUDPInterface() {
  close();
}

bool CartPoleUDPInterface::initialize(int state_port, int cmd_port,
                                      const std::string& controller_ip) {
  // Create state receiver socket
  state_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (state_sock_ < 0) {
    std::cerr << "[CartPoleUDP] Failed to create state socket" << std::endl;
    return false;
  }

  // Set socket timeout
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 500000;  // 500ms timeout
  if (setsockopt(state_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    std::cerr << "[CartPoleUDP] Failed to set socket timeout" << std::endl;
    close();
    return false;
  }

  // Set small receive buffer to prevent old data accumulation
  // Default is ~212KB (~1000 packets), reduce to ~2KB (~10-20 packets)
  int rcvbuf_size = 2048;
  if (setsockopt(state_sock_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
    std::cerr << "[CartPoleUDP] Warning: Failed to set socket buffer size" << std::endl;
  }

  // Bind state socket
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(state_port);

  if (bind(state_sock_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    std::cerr << "[CartPoleUDP] Failed to bind state socket to port " 
              << state_port << std::endl;
    close();
    return false;
  }

  // Create command sender socket
  cmd_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (cmd_sock_ < 0) {
    std::cerr << "[CartPoleUDP] Failed to create command socket" << std::endl;
    close();
    return false;
  }

  controller_ip_ = controller_ip;
  cmd_port_ = cmd_port;
  initialized_ = true;

  std::cout << "[CartPoleUDP] Interface initialized" << std::endl;
  std::cout << "  State receiver: 0.0.0.0:" << state_port << std::endl;
  std::cout << "  Command sender: " << controller_ip << ":" << cmd_port
            << std::endl;

  // FLUSH OLD DATA FROM SOCKET BUFFER
  std::cout << "[CartPoleUDP] Flushing old data from socket buffer..." << std::endl;
  
  // Set very short timeout for flushing
  struct timeval flush_tv;
  flush_tv.tv_sec = 0;
  flush_tv.tv_usec = 10000;  // 10ms timeout
  setsockopt(state_sock_, SOL_SOCKET, SO_RCVTIMEO, &flush_tv, sizeof(flush_tv));
  
  uint8_t flush_buffer[STATE_BUFFER_SIZE];
  int flushed_count = 0;
  auto flush_start = std::chrono::steady_clock::now();
  
  while (true) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - flush_start).count();
    if (elapsed > 500) break;  // Flush for max 500ms
    
    int received = recv(state_sock_, flush_buffer, STATE_BUFFER_SIZE, 0);
    if (received > 0) {
      flushed_count++;
    } else {
      break;  // No more data
    }
  }
  
  std::cout << "  Flushed " << flushed_count << " old packets" << std::endl;
  
  // Restore normal timeout
  struct timeval normal_tv;
  normal_tv.tv_sec = 0;
  normal_tv.tv_usec = 500000;  // 500ms timeout
  setsockopt(state_sock_, SOL_SOCKET, SO_RCVTIMEO, &normal_tv, sizeof(normal_tv));

  return true;
}

bool CartPoleUDPInterface::receiveState(CartPoleState& state, int timeout_ms) {
  if (!initialized_) {
    std::cerr << "[CartPoleUDP] Interface not initialized" << std::endl;
    return false;
  }

  uint8_t buffer[STATE_BUFFER_SIZE];
  struct sockaddr_in addr;
  socklen_t addr_len = sizeof(addr);

  int received = recvfrom(state_sock_, buffer, STATE_BUFFER_SIZE, 0,
                          (struct sockaddr*)&addr, &addr_len);

  if (received != STATE_BUFFER_SIZE) {
    if (received < 0) {
      // Timeout is expected, not an error
      return false;
    }
    std::cerr << "[CartPoleUDP] Received invalid state packet size: " 
              << received << " (expected " << STATE_BUFFER_SIZE << ")"
              << std::endl;
    return false;
  }

  // Parse state packet
  float* float_data = reinterpret_cast<float*>(buffer);
  uint32_t crc_recv = *reinterpret_cast<uint32_t*>(buffer + 20);

  // Verify CRC
  uint32_t crc_calc = calculateCRC32(buffer, 20);
  if (crc_calc != crc_recv) {
    std::cerr << "[CartPoleUDP] CRC mismatch in state packet" << std::endl;
    return false;
  }

  // Extract state
  state.cart_pos = float_data[0];
  state.pole_angle = normalizeAngle(float_data[1]);
  state.cart_vel = float_data[2];
  state.pole_anglevel = float_data[3];
  state.timestamp = float_data[4];
  state.crc = crc_recv;

  state_count_++;
  return true;
}

bool CartPoleUDPInterface::sendCommand(const CartPoleCommand& command) {
  if (!initialized_) {
    std::cerr << "[CartPoleUDP] Interface not initialized" << std::endl;
    return false;
  }

  uint8_t buffer[CMD_BUFFER_SIZE];
  float* float_data = reinterpret_cast<float*>(buffer);

  // Pack command data (without CRC)
  float_data[0] = command.force;
  float_data[1] = command.timestamp;

  // Calculate CRC
  uint32_t crc = calculateCRC32(buffer, 8);
  uint32_t* crc_ptr = reinterpret_cast<uint32_t*>(buffer + 8);
  *crc_ptr = crc;

  // Send command
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(cmd_port_);
  addr.sin_addr.s_addr = inet_addr(controller_ip_.c_str());

  if (sendto(cmd_sock_, buffer, CMD_BUFFER_SIZE, 0, (struct sockaddr*)&addr,
             sizeof(addr)) < 0) {
    std::cerr << "[CartPoleUDP] Failed to send command" << std::endl;
    return false;
  }

  return true;
}

void CartPoleUDPInterface::close() {
  if (state_sock_ >= 0) {
    ::close(state_sock_);
    state_sock_ = -1;
  }
  if (cmd_sock_ >= 0) {
    ::close(cmd_sock_);
    cmd_sock_ = -1;
  }
  initialized_ = false;
}

}  // namespace ocs2_cartpole
