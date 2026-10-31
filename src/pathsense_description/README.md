# pathsense_description

URDF model + Gazebo Harmonic simulation + keyboard teleop for the **PathSense stair-climbing indoor service robot**
(tracked base in the MEC235 style, two retractable tracked climbing arms, LiDAR, 2 cameras, IMU, 6 ToF edge sensors, barometer).

Target: **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**. Primitive shapes only, so there are no mesh files to install.

## 1. Install dependencies (once)

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                 ros-jazzy-joint-state-publisher-gui ros-jazzy-teleop-twist-keyboard ros-jazzy-rviz2
```

## 2. Build

```bash
mkdir -p ~/pathsense_ws/src
cp -r pathsense_description ~/pathsense_ws/src/
cd ~/pathsense_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Run

### A. Look at the model in RViz (sliders for wheels and arms)
```bash
ros2 launch pathsense_description display.launch.py
```

### B. Gazebo + RViz, drive with the keyboard
Terminal 1:
```bash
ros2 launch pathsense_description gazebo.launch.py
```
Terminal 2 (driving):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
`i` forward, `,` back, `j`/`l` turn on the spot, `k` stop, `q`/`z` change speed (start around 0.3 m/s).

Terminal 3 (climbing arms):
```bash
ros2 run pathsense_description arm_teleop_keyboard.py
```
`w`/`s` both arms up/down, `e`/`d` left arm, `r`/`f` right arm,
`1` drive pose, `2` flat, `3` press down (lifts the front), `4` retract, `5` vertical.

The teleop window must have keyboard focus (click on that terminal before pressing keys).

Options: `rviz:=false`, `use_tof:=false` (lighter on slow laptops), `x:= y:= yaw:=` spawn pose, `world:=/path/to/world.sdf`.

### C. RViz only (no Gazebo), still driven by the keyboard
```bash
ros2 launch pathsense_description rviz_teleop.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # new terminal
ros2 run pathsense_description arm_teleop_keyboard.py    # new terminal
```

## Topics (Gazebo)

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | track drive (skid steer) |
| `/arm_left/cmd_pos`, `/arm_right/cmd_pos` | std_msgs/Float64 | arm angle, rad (0.40 = drive pose, 3.0 = retracted, negative = press down) |
| `/odom`, `/tf` | Odometry, TF | `odom -> base_footprint` |
| `/joint_states` | JointState | wheels + arms |
| `/scan` | LaserScan | 2D LiDAR, 360°, 12 m |
| `/camera/front/image_raw`, `/camera/stair/image_raw` (+ `camera_info`) | Image | forward and stair cameras |
| `/imu` | Imu | BNO055 stand-in |
| `/air_pressure` | FluidPressure | BMP390 stand-in (floor change) |
| `/tof/front_left` … `/tof/side_right` | LaserScan (1 ray) | VL53L1X edge sensors |

## Files

```
urdf/pathsense.urdf.xacro   main model: dimensions at the top, body, tracks, wheels, arms, sensors
urdf/materials.xacro        colour palette (from the concept render)
urdf/macros.xacro           inertia + geometry helpers (grip pads, wheel holes, rivets)
urdf/gazebo.xacro           Gazebo plugins: drive, arm controllers, sensors, friction
config/gz_bridge.yaml       Gazebo <-> ROS 2 topic bridge
worlds/stair_test.sdf       room with a 5-step staircase (150 mm rise) and landing
launch/*.launch.py          display / gazebo / rviz_teleop
scripts/arm_teleop_keyboard.py   keyboard control for the climbing arms
scripts/rviz_teleop_sim.py       kinematic simulator for RViz-only driving
```

## Model notes

- Size 508 × 393 × 249 mm (body + tracks, without LiDAR/antennas), 160 mm track wheels, 104 mm arm end wheels, arm length 260 mm, about 5.9 kg.
- Tracks are simulated as 4 driven wheels per side (2 visible + 2 hidden support wheels) under a static belt. This drives, turns and bumps up step edges well, but the belt itself does not move and the arm belts are not driven, so stair climbing in Gazebo is approximate: lift the front with the arms (`3`) and drive forward.
- Cameras are simulated at 640×360 to stay light; the real ones are 1080p.
- If Gazebo shows nothing or crashes on a laptop without a proper GPU, try `export LIBGL_ALWAYS_SOFTWARE=1` before launching.
