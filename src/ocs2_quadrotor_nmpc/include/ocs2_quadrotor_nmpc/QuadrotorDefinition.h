#pragma once

#include <cstddef>

namespace ocs2_cartpole_quadrotor {

constexpr size_t STATE_DIM = 12;  // [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
constexpr size_t INPUT_DIM = 4;   // [Fz, Mx, My, Mz]

}  // namespace ocs2_cartpole_quadrotor
