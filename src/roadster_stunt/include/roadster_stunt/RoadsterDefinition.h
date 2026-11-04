#pragma once

#include <cstddef>

namespace roadster_stunt {

/**
 * Planar roll model of the car in flight (the y-z plane across the road; x is decoupled, x = launchSpeed * t).
 *
 * State  [y, z, vy, vz, roll, rollRate]               (m, m, m/s, m/s, rad, rad/s)
 * Input  [lift_L, lift_R, roof_L, roof_R]             (kN, front + rear jet of one side summed)
 */
constexpr size_t STATE_DIM = 6;
constexpr size_t INPUT_DIM = 4;

enum StateIndex : size_t { Y = 0, Z = 1, VY = 2, VZ = 3, ROLL = 4, ROLL_RATE = 5 };
enum InputIndex : size_t { LIFT_L = 0, LIFT_R = 1, ROOF_L = 2, ROOF_R = 3 };

/** Number of jets on the car (MuJoCo actuator order: 4 lift, 4 roof, 4 side). */
constexpr size_t NUM_JETS = 12;

}  // namespace roadster_stunt
