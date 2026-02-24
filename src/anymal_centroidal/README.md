# anymal_centroidal

ANYmal MuJoCo GUI with OCS2 Centroidal Model - Test convention without controller.

## Purpose

This package tests the OCS2 Centroidal Model convention by comparing:
1. **Pinocchio FreeFlyer**: Standard Pinocchio with quaternion base (19D q, 18D v)
2. **OCS2 Centroidal**: OCS2's Pinocchio with SphericalZYX base (18D q, 18D v)

## Key Differences

| Aspect | Pinocchio FreeFlyer | OCS2 Centroidal |
|--------|---------------------|-----------------|
| Base position | q[0:3] | q[0:3] |
| Base orientation | q[3:7] quaternion (x,y,z,w) | q[3:6] euler ZYX (yaw,pitch,roll) |
| Joints start | q[7:19] | q[6:18] |
| Joint order | LF, LH, RF, RH (alphabetical) | LF, LH, RF, RH (alphabetical) |
| Total q dim | 19 | 18 |
| Total v dim | 18 | 18 |

## Joint Order Mapping

```
MuJoCo/URDF order:  LF(0-2), RF(3-5), LH(6-8), RH(9-11)
OCS2/Pinocchio:     LF(0-2), LH(3-5), RF(6-8), RH(9-11)
```

## Build

```bash
cd ~/ros2_ws_ocs2
colcon build --packages-select anymal_centroidal
```

## Run

```bash
./build/anymal_centroidal/anymal_centroidal_gui
```

## Output

The console prints comparison between:
- Forward Kinematics (foot positions)
- Center of Mass
- Centroidal Dynamics (momentum)
- Gravity Compensation (RNEA)
- RBD State Conversions
