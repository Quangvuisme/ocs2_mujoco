# ROS2 Launch Files for Robot Visualization

## Prerequisites

Ensure you have the following ROS2 packages installed:

```bash
sudo apt update
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-rviz2
```

## Usage

All launch scripts are located in `/home/quangvd7/ros2_ws_ocs2/scripts/` and reference robot URDFs from `/home/quangvd7/ros2_ws_ocs2/robots/`.

### 1. Launch GO1 Robot in RViz2

```bash
cd /home/quangvd7/ros2_ws_ocs2/scripts
ros2 launch launch_go1_rviz.py
```

### 2. Launch GO2 Robot in RViz2

```bash
cd /home/quangvd7/ros2_ws_ocs2/scripts
ros2 launch launch_go2_rviz.py
```

### 3. Launch ANYmal C Robot in RViz2

```bash
cd /home/quangvd7/ros2_ws_ocs2/scripts
ros2 launch launch_anymal_c_rviz.py
```

## Features

Each launch file will:
- Load the robot URDF model
- Start `robot_state_publisher` to publish the robot transforms
- Start `joint_state_publisher_gui` for interactive joint control
- Launch RViz2 for visualization

## Optional: With Simulation Time

If using with Gazebo or simulation:

```bash
ros2 launch launch_go1_rviz.py use_sim_time:=true
ros2 launch launch_go2_rviz.py use_sim_time:=true
ros2 launch launch_anymal_c_rviz.py use_sim_time:=true
```

## RViz Configuration

The launch files will look for RViz config files in:
- `go1/rviz/go1.rviz`
- `go2/rviz/go2.rviz`
- `anymal_c/rviz/anymal_c.rviz`

If these files don't exist, RViz2 will open with default settings. You can manually add:
- **RobotModel** display to visualize the robot
- **TF** display to see the coordinate frames
- Set **Fixed Frame** to `base_link` or appropriate frame

Save your RViz configuration to the paths above for automatic loading next time.

## Troubleshooting

### Issue: RViz2 crashes with "symbol lookup error: libpthread.so.0"

This is caused by a conflict with snap packages. **Solutions:**

**Option 1: Use the fixed launch files (already included)**
```bash
ros2 launch launch_go1_rviz.py
```

**Option 2: Run RViz2 manually in a separate terminal**
```bash
# Terminal 1 - Launch without RViz
cd /home/quangvd7/ros2_ws_ocs2/robots/go1
ros2 launch launch_go1_rviz.py use_rviz:=false

# Terminal 2 - Start RViz2 manually
LD_PRELOAD= rviz2
```

**Option 3: Uninstall snap RViz2 and use apt version**
```bash
sudo snap remove rviz2  # if installed via snap
sudo apt install ros-humble-rviz2
```

### Issue: Mesh files not loading (green spheres instead of robot model)

The launch files automatically convert relative mesh paths to absolute paths. If you still see issues:

1. **Verify mesh files exist:**
   ```bash
   ls ~/ros2_ws_ocs2/robots/go1/go1_urdf/meshes/*.dae
   ls ~/ros2_ws_ocs2/robots/go2/go2_urdf/dae/*.dae
   ls ~/ros2_ws_ocs2/robots/anymal_c/anymal_c_urdf/meshes/*.dae
   ```

2. **In RViz2:** The robot model should load automatically. If not, try:
   - Remove and re-add the **RobotModel** display
   - Check the status message in the display panel
   - Set **Fixed Frame** to `trunk` (for GO1) or `base_link` (for GO2/ANYmal)

## Verification Script

A Python script is provided to verify UDP communication without MuJoCo:

```bash
cd ~/ros2_ws_ocs2/scripts
python3 verify_udp_quadruped.py --mode passive --duration 30
python3 verify_udp_quadruped.py --mode pd --duration 30
python3 verify_udp_quadruped.py --mode both
```

### Issue: "No transform from [link] to [base_link]"
- Make sure `joint_state_publisher_gui` is running and publishing joint states

### Issue: Robot model not showing
- Check that URDF file paths are correct
- Verify mesh files (.dae, .obj, .stl) exist in the specified directories
- In RViz, add **RobotModel** display and set the topic to `/robot_description`
- Set **Fixed Frame** to `trunk` or `base_link`

### Issue: Launch file permission denied
- Make sure the launch files are executable:
```bash
chmod +x launch_*.py
```
