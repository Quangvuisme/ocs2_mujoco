# Quadrotor NMPC Controller with OCS2

Quadrotor Nonlinear Model Predictive Control using OCS2 framework with UDP communication bridge to MuJoCo simulator.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    Quadrotor NMPC System                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────────┐         ┌──────────────────┐             │
│  │  MuJoCo Sim      │◄────────►│  UDP Bridge      │             │
│  │ (50 Hz physics)  │          │ (500 Hz state TX)│             │
│  │                  │          │ (RX commands)    │             │
│  └──────────────────┘         └────────┬─────────┘             │
│                                        │ Port 9001 (RX)        │
│                                        │ Port 9002 (TX)        │
│                                        ▼                       │
│  ┌──────────────────────────────────────────────┐              │
│  │  Quadrotor NMPC Controller                   │              │
│  │  ┌────────────────────────────────────────┐  │              │
│  │  │ Control Loop (100 Hz)                  │  │              │
│  │  │ - RX state                             │  │              │
│  │  │ - Interpolate MPC solution             │  │              │
│  │  │ - TX command to simulator              │  │              │
│  │  └────────────────────────────────────────┘  │              │
│  │  ┌────────────────────────────────────────┐  │              │
│  │  │ MPC Optimization Thread (10 Hz)        │  │              │
│  │  │ - Update target trajectory             │  │              │
│  │  │ - Run DDP/SLQ solver                   │  │              │
│  │  │ - Store solution for interpolation     │  │              │
│  │  └────────────────────────────────────────┘  │              │
│  └──────────────────────────────────────────────┘              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## System Specifications

### State Vector (12D)
```
x = [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]ᵀ
```

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0-2 | x,y,z | Position in world frame | m |
| 3-5 | roll, pitch, yaw | Euler angles (XYZ convention) | rad |
| 6-8 | vx, vy, vz | Linear velocity in world frame | m/s |
| 9-11 | wx, wy, wz | Angular velocity (body frame) | rad/s |

### Input Vector (4D)
```
u = [Fz, Mx, My, Mz]ᵀ
```

| Index | Symbol | Description | Unit |
|-------|--------|-------------|------|
| 0 | Fz | Vertical thrust force | N |
| 1-3 | Mx, My, Mz | Moment/torque commands | N·m |

### Quadrotor Parameters

From OCS2 example configuration:

```
Mass (m):           0.546 kg
Inertia Ixx (Iyy):  2.32e-3 kg·m²
Inertia Izz:        3.0e-4 kg·m²
Arm length:         0.1265 m
```

## Project Structure

```
src/ocs2_quadrotor_nmpc/
├── CMakeLists.txt                    # Build configuration
├── package.xml                       # ROS2 package metadata
├── include/ocs2_quadrotor_nmpc/
│   ├── QuadrotorParameters.h          # Parameter loading
│   ├── QuadrotorDynamics.h            # System dynamics declaration
│   ├── QuadrotorInterface.h           # OCS2 problem setup
│   └── QuadrotorUDP.h                 # UDP communication
├── src/
│   ├── QuadrotorDynamics.cpp          # Dynamics implementation
│   ├── QuadrotorInterface.cpp         # OCP builder
│   ├── QuadrotorUDP.cpp               # UDP socket handling
│   └── main_udp_nmpc.cpp              # Main control loop
└── config/mpc/
    └── task.info                      # MPC configuration

robots/quadrotor/
├── quadrotor.urdf                     # URDF model (Pinocchio validation)
└── quadrotor.xml                      # MuJoCo scene file

simulate/src/quadrotor_bridge/
└── quadrotor_bridge.h                 # MuJoCo ↔ Controller bridge
```

## File Descriptions

### QuadrotorParameters.h
- Loads quadrotor physical parameters from `task.info`
- Parameters: mass, inertia tensors (Ixx, Iyy, Izz), gravity
- Used by dynamics and interface components

### QuadrotorDynamics.h/cpp
- Implements system dynamics: `ẋ = f(x, u)`
- Non-linear rigid body dynamics with rotation matrix
- Attitude dynamics: Euler angle rates computed from quaternion mapping
- Force/moment propagated through inertia matrix
- Compatible with OCS2 AD differentiation (uses scalar_t)
- **Note**: Hardcoded for efficiency; Pinocchio validates URDF at init

### QuadrotorInterface.h/cpp
- Builds OCS2 OptimalControlProblem
- Loads URDF model via Pinocchio (validation only)
- Sets up cost functions: Q (state cost), R (input cost)
- Creates MPC solver with SLQ algorithm
- Returns configured problem for solver
- Reference trajectory: hovering at z=1m

### QuadrotorUDP.h/cpp
- UDP socket management (non-blocking, dual-thread)
- RX: Receives state from MuJoCo simulator (9001, 500 Hz)
  - Packet: timestamp(8B) + state[12](96B) = 104 bytes
- TX: Sends commands to MuJoCo simulator (9002)
  - Packet: timestamp(8B) + input[4](32B) = 40 bytes
- Statistics tracking: RX/TX packet counts

### main_udp_nmpc.cpp
- **Control Loop (100 Hz)**:
  - Receives state from simulator
  - Interpolates stored MPC solution
  - Sends command to simulator
  - Maintains real-time guarantees
  
- **MPC Thread (10 Hz)**:
  - Runs optimization in background
  - Updates target trajectory (hovering at origin)
  - Stores solution for control loop interpolation
  
- **Signal Handling**:
  - SIGINT (Ctrl+C): Clean shutdown
  - Gracefully closes UDP sockets

### quadrotor_bridge.h
- MuJoCo-side UDP bridge (runs in simulator threads)
- **State Publish (500 Hz)**:
  - Reads body state from MuJoCo: position, quaternion, velocities
  - Converts quaternion → Euler angles (XYZ convention)
  - Packs into UDP packet, sends to controller
  
- **Command Receive**:
  - Listens for command packets
  - Applies forces/torques to quadrotor base body
  - Uses MuJoCo's `xfrc_applied` mechanism

- **Thread Management**:
  - Separate threads for publish and receive
  - Non-blocking sockets with periodic sleep

### task.info
MPC solver configuration:

**DDP Settings**:
- Algorithm: SLQ (Sequential Linear Quadratic)
- Max iterations: 5 per cycle
- Time horizon: 5.0 s
- ODE integration: RK45 with adaptive timestep
- Tolerance: 1e-5 (absolute), 1e-3 (relative)

**MPC Settings**:
- Solution time limit: 50 ms (for 10 Hz)
- Prediction horizon: 5.0 seconds

**Cost Matrices**:
- Q: State cost weights [1, 1, 10, 0.1, 0.1, 0.1, 0.5, 0.5, 1.0, 0.1, 0.1, 0.1]
  - Higher weight on Z position (altitude)
  - Lower weight on angles (allow rotation)
- Qf: Terminal state cost (2x state cost)
- R: Input cost [0.1, 0.01, 0.01, 0.01]
  - Penalizes excessive thrust and torques

### quadrotor.urdf
- Pinocchio URDF model
- Floating-base quadrotor with 4 visualization arm frames
- Mass: 0.546 kg
- Inertia tensors match MuJoCo specs
- Used for model validation (dynamics hardcoded in C++)

### quadrotor.xml
- MuJoCo scene description
- Gravity: 9.8 m/s²
- Timestep: 0.001 s (1 kHz physics)
- Quadrotor base body with 4 arm visualization frames
- Sensor definitions for state feedback (position, velocity, angles)

## Building

```bash
# In workspace root
colcon build --packages-select ocs2_quadrotor_nmpc

# Or with verbose output
colcon build --packages-select ocs2_quadrotor_nmpc -v
```

## Running

### Terminal 1: Start MuJoCo Simulator
```bash
cd /path/to/workspace
./simulate/build/mujoco_ocs quadrotor
```

Expected output:
```
Quadrotor: Mujoco data is prepared.
Quadrotor Bridge initialized
  State broadcast to 127.0.0.1:9001
  Command listen on port 9002
Quadrotor: UDP bridge running with dedicated threads (500 Hz)
```

### Terminal 2: Start NMPC Controller
```bash
# First source the environment
source install/setup.bash

# Run controller
./install/ocs2_quadrotor_nmpc/bin/ocs2_quadrotor_nmpc_udp \
  ./src/ocs2_quadrotor_nmpc/config/mpc
```

Expected output:
```
====================================================
     Quadrotor NMPC Controller (OCS2 + UDP)
====================================================
Config directory: ./src/ocs2_quadrotor_nmpc/config/mpc
Control frequency: 100 Hz
MPC frequency: 10 Hz
UDP RX port: 9001
UDP TX port: 9002
====================================================

[QuadrotorUDP] Initialized successfully
  RX listening on port 9001
  TX sending to port 9002 (127.0.0.1)
[QuadrotorInterface] Successfully initialized!
  State dim: 12 [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
  Input dim: 4 [Fz, Mx, My, Mz]
  Time horizon: 5.0 s
  Mass: 0.546 kg
  Gravity: 9.8 m/s^2
[Control] t=0.0s, state=[0 0 1], input=[5.34628 0 0 0], loops/s=100
[MPC] Optimization cycle completed
```

## Control Flow Details

### Initialization Phase
1. **MuJoCo Simulator** loads `quadrotor.xml`, initializes physics
2. **Quadrotor Bridge** waits for simulator data readiness
3. Once ready, bridge launches:
   - State publish thread (500 Hz)
   - Command receive thread (non-blocking)
4. **NMPC Controller** initializes:
   - Loads configuration from `task.info`
   - Creates OCS2 problem with dynamics and costs
   - Launches MPC optimization thread
   - Starts control loop

### Runtime Cycle

**Control Loop (runs every 10 ms @ 100 Hz):**
```
┌─────────────────────────────────┐
│ 1. Try receive state from sim   │ (~1-2 ms)
│    Port 9001 (UDP)              │
└───────────┬─────────────────────┘
            ▼
┌─────────────────────────────────┐
│ 2. Check MPC ready              │ (<1 ms)
│    Lock mutex, get last solution│
└───────────┬─────────────────────┘
            ▼
┌─────────────────────────────────┐
│ 3. Interpolate control input    │ (<1 ms)
│    Use stored MPC solution      │
└───────────┬─────────────────────┘
            ▼
┌─────────────────────────────────┐
│ 4. Send command to simulator    │ (<1 ms)
│    Port 9002 (UDP)              │
└───────────┬─────────────────────┘
            ▼
┌─────────────────────────────────┐
│ 5. Sleep remaining time         │ (≈7-8 ms)
│    Target 10 ms cycle (100 Hz)  │
└─────────────────────────────────┘
```

**MPC Thread (runs every 100 ms @ 10 Hz):**
```
┌─────────────────────────────────┐
│ 1. Update target trajectory     │
│    (hover at origin or setpoint)│
└───────────┬─────────────────────┘
            ▼
┌─────────────────────────────────┐
│ 2. Run DDP/SLQ optimization     │
│    Max 50 ms for convergence    │
└───────────┬─────────────────────┘
            ▼
┌─────────────────────────────────┐
│ 3. Lock mutex, store solution   │ (<1 ms)
│    Available for control loop   │
└───────────┬─────────────────────┘
            ▼
┌─────────────────────────────────┐
│ 4. Sleep remaining time         │ (≈40-50 ms)
│    Target 100 ms cycle (10 Hz)  │
└─────────────────────────────────┘
```

## Dynamics Equations

### Translational Dynamics
Position derivatives equal velocity:
```
ẋ = vx,  ẏ = vy,  ż = vz
```

Acceleration in world frame (from forces):
```
vx_dot = (sin(yaw)·sin(roll) + cos(yaw)·sin(pitch)·cos(roll))·Fz / m
vy_dot = (-cos(yaw)·sin(roll) + sin(yaw)·sin(pitch)·cos(roll))·Fz / m
vz_dot = (cos(pitch)·cos(roll))·Fz / m - g
```

### Rotational Dynamics
Euler angle rates from angular velocity (via quaternion mapping):
```
roll_dot  = T[0,0]·wx + T[0,1]·wy + T[0,2]·wz
pitch_dot = T[1,0]·wx + T[1,1]·wy + T[1,2]·wz
yaw_dot   = T[2,0]·wx + T[2,1]·wy + T[2,2]·wz
```

Where T is transformation matrix from quaternion.

Angular accelerations (Euler equations):
```
wx_dot = Mx / Ixx
wy_dot = My / Iyy
wz_dot = Mz / Izz
```

For quadrotor, coupling terms are negligible since Ixx ≈ Iyy >> Izz.

## Cost Function

### Quadratic Cost
```
J = ∫[0 to T] (x^T Q x + u^T R u) dt + x_f^T Qf x_f
```

**State Cost (Q)**: Penalizes deviations from reference trajectory
- Altitude (z): weight 10 (maintain height)
- Position (x,y): weight 1 (allow lateral motion)
- Angles: weight 0.1 (allow attitude freedom)
- Velocities: weight 0.5-1.0 (smooth motion)

**Input Cost (R)**: Penalizes control effort
- Thrust: weight 0.1
- Torques: weight 0.01 (agile rotation)

**Terminal Cost (Qf)**: Soft constraint at horizon end
- 2x state cost to encourage convergence

## Tuning Guidelines

### Cost Weights
- Increase Q for tighter reference tracking
- Increase R to reduce energy consumption
- Adjust Qf for terminal convergence

### Solver Settings
- `maxNumIterations`: More iterations = better solution but slower
- `timeStep`: Smaller timestep = more accurate but slower
- `AbsTolODE`, `RelTolODE`: Tighter tolerance = better accuracy but slower

### Horizon Length
- Longer horizon = better long-term predictions but slower
- Typical: 5-10 seconds for hover tasks

## Troubleshooting

### Build Errors

**`error: 'getMappingFromLocalAngularVelocityToEulerAnglesXyzDerivative' was not declared`**
- Missing OCS2 robotic_tools library
- Add to CMakeLists: `ocs2_robotic_tools`

**`undefined reference to QuadrotorDynamics::...`**
- Missing object file in linker
- Check CMakeLists source files are listed

### Runtime Issues

**No state received from simulator**
- Check simulator running: `ps aux | grep mujoco_ocs`
- Verify UDP ports 9001, 9002 not blocked
- Check firewall settings

**Controller not updating**
- MPC thread may be slow, check convergence
- Verify `max_solution_time = 50ms` in config
- Monitor CPU usage

**Simulation unstable (quadrotor diverges)**
- Reduce thrust input limit in quadrotor.xml
- Increase state costs Q
- Reduce time horizon
- Check Pinocchio inertia matches MuJoCo

## Comparison with CartPole

| Aspect | CartPole | Quadrotor |
|--------|----------|-----------|
| DOF | 2 | 6 |
| State dim | 4 | 12 |
| Input dim | 1 | 4 |
| Dynamics | Linear with gravity | Nonlinear attitudes |
| Attitude | Angle + angle rate | Euler angles + quaternion |
| Control focus | Swing-up + stabilize | Altitude + heading |
| Typical horizon | 5 s | 5 s |
| MPC freq | 10 Hz | 10 Hz |
| Real-time freq | 100 Hz | 100 Hz |

## Future Enhancements

1. **Tracking Control**: Modify reference trajectory for waypoint following
2. **Constraints**: Add inequality constraints (velocity bounds, angle limits)
3. **State Estimation**: Add Kalman filter for noisy measurements
4. **Adaptive**: Online cost weighting based on performance
5. **Multi-phase**: Different horizons for takeoff/cruise/landing
6. **Robustness**: Include model uncertainty and disturbances
7. **Simulation comparison**: Validate against analytical solutions

## References

- OCS2 Documentation: [github.com/leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2)
- Pinocchio: [stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio)
- MuJoCo: [deepmind/mujoco](https://github.com/deepmind/mujoco)

## Author & License

Quadrotor NMPC Controller - Based on OCS2 and CartPole examples

See LICENSE file for details (typically BSD 3-Clause)
