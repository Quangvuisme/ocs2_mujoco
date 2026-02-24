#include "ocs2_quadrotor_nmpc/QuadrotorUDP.h"

#include <iostream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <chrono>
#include <thread>

namespace ocs2_cartpole_quadrotor {

QuadrotorUDP::QuadrotorUDP() : rxBuffer_(sizeof(StatePacket)) {}

QuadrotorUDP::~QuadrotorUDP() { shutdown(); }

void QuadrotorUDP::initialize(int rxPort, int txPort) {
  rxPort_ = rxPort;
  txPort_ = txPort;

  // Create RX socket (receive state)
  rxSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (rxSocket_ < 0) {
    std::cerr << "[QuadrotorUDP] Failed to create RX socket\n";
    return;
  }

  // Set socket to non-blocking
  int flags = fcntl(rxSocket_, F_GETFL, 0);
  fcntl(rxSocket_, F_SETFL, flags | O_NONBLOCK);

  // Bind RX socket
  struct sockaddr_in rxAddr;
  std::memset(&rxAddr, 0, sizeof(rxAddr));
  rxAddr.sin_family = AF_INET;
  rxAddr.sin_addr.s_addr = INADDR_ANY;
  rxAddr.sin_port = htons(rxPort_);

  if (bind(rxSocket_, (struct sockaddr*)&rxAddr, sizeof(rxAddr)) < 0) {
    std::cerr << "[QuadrotorUDP] Failed to bind RX socket on port " << rxPort_ << "\n";
    close(rxSocket_);
    rxSocket_ = -1;
    return;
  }

  // Create TX socket (send command)
  txSocket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (txSocket_ < 0) {
    std::cerr << "[QuadrotorUDP] Failed to create TX socket\n";
    close(rxSocket_);
    rxSocket_ = -1;
    return;
  }

  // FLUSH OLD DATA FROM RX SOCKET BUFFER (like PID controller)
  // Reduce buffer size to prevent accumulation
  int recv_buf_size = 4096;
  setsockopt(rxSocket_, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size));
  
  std::cout << "[QuadrotorUDP] Flushing old state data from socket buffer...\n";
  int flushed_count = 0;
  auto flush_start = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - flush_start).count() < 1.0) {
    char dummy[256];
    int n = recvfrom(rxSocket_, dummy, sizeof(dummy), 0, nullptr, nullptr);
    if (n > 0) {
      flushed_count++;
    } else if (flushed_count > 0) {
      // Buffer empty after getting some data
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      break;
    }
  }
  std::cout << "[QuadrotorUDP] Flushed " << flushed_count << " old packets\n";

  std::cout << "[QuadrotorUDP] Initialized successfully\n"
            << "  RX listening on port " << rxPort_ << "\n"
            << "  TX sending to port " << txPort_ << " (127.0.0.1)\n";
}

bool QuadrotorUDP::receiveState(double& time, ocs2::vector_t& state) {
  if (rxSocket_ < 0) return false;

  struct sockaddr_in srcAddr;
  socklen_t srcLen = sizeof(srcAddr);

  int nbytes = recvfrom(rxSocket_, rxBuffer_.data(), rxBuffer_.size(), 0,
                        (struct sockaddr*)&srcAddr, &srcLen);

  if (nbytes != sizeof(StatePacket)) {
    return false;  // No data or incomplete packet
  }

  StatePacket* packet = (StatePacket*)rxBuffer_.data();
  time = static_cast<double>(packet->timestamp) / 1e9;  // Convert to seconds

  state.setZero(12);
  for (int i = 0; i < 12; i++) {
    state(i) = packet->state[i];
  }

  rxCount_++;
  return true;
}

void QuadrotorUDP::sendCommand(double time, const ocs2::vector_t& input) {
  if (txSocket_ < 0) return;

  CommandPacket packet;
  packet.timestamp = static_cast<uint64_t>(time * 1e9);  // Convert to nanoseconds

  for (int i = 0; i < 4 && i < input.rows(); i++) {
    packet.input[i] = input(i);
  }

  struct sockaddr_in txAddr;
  std::memset(&txAddr, 0, sizeof(txAddr));
  txAddr.sin_family = AF_INET;
  txAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
  txAddr.sin_port = htons(txPort_);

  sendto(txSocket_, (const char*)&packet, sizeof(packet), 0,
         (const struct sockaddr*)&txAddr, sizeof(txAddr));

  txCount_++;
}

void QuadrotorUDP::shutdown() {
  if (rxSocket_ >= 0) {
    close(rxSocket_);
    rxSocket_ = -1;
  }
  if (txSocket_ >= 0) {
    close(txSocket_);
    txSocket_ = -1;
  }
  std::cout << "[QuadrotorUDP] Shutdown complete (RX: " << rxCount_
            << ", TX: " << txCount_ << ")\n";
}

}  // namespace ocs2_cartpole_quadrotor
