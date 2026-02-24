## Build

```bash
cd ~/ros2_ws_ocs2
colcon build --packages-select anymal_standing_online
```

## Run

```bash
source ./install/setup.bash
./build/anymal_standing_online/anymal_standing_mujoco
```