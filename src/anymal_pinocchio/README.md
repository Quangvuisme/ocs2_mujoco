# anymal_pinocchio

ANYmal MuJoCo simulation with continuous Pinocchio calculations printing.

## Description

This package combines:
- **MuJoCo GUI**: Full interactive simulation with ANYmal C robot
- **PD Control**: Standing control using joint-level PD
- **Pinocchio Calculations**: Continuous printing of robot dynamics

## Pinocchio Calculations Printed

Every 500ms (configurable), the following are computed and printed:

1. **Configuration State**: Base pose, joint positions/velocities
2. **Forward Kinematics**: Foot positions in world frame
3. **Center of Mass**: Position and velocity
4. **Jacobians**: Foot Jacobian matrices
5. **Mass Matrix**: Inertia matrix M(q)
6. **Gravity Compensation**: g(q) vector
7. **Inverse Dynamics (RNEA)**: Required torques τ = M*a + C*v + g
8. **Coriolis Forces**: C(q,v)*v

## Build

```bash
cd ~/ros2_ws_ocs2
colcon build --packages-select anymal_pinocchio
source install/setup.bash
```

## Run

```bash
ros2 run anymal_pinocchio anymal_pinocchio_gui
```

## Controls

- **Space**: Play/Pause simulation
- **Mouse**: Rotate/Zoom/Pan camera
- **Tab**: Toggle UI panels
- **ESC**: Exit

## Dependencies

- ocs2_pinocchio_interface
- ocs2_robotic_assets (for ANYmal URDF)
- pinocchio
- mujoco
- glfw3

## Notes

- Requires `robots/anymal_c/scene_anymal_c_real.xml` in workspace root
- Uses ANYmal C URDF from ocs2_robotic_assets
- Print interval can be adjusted via `PRINT_INTERVAL` constant
