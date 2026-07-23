# Ubuntu Webots + ROS 2

This project includes a ROS 2 bridge controller for the legacy hexapod:

- `controllers/legacy_hexapod_ros2_bridge/legacy_hexapod_ros2_bridge.py`
- `worlds/legacy_hexapod_ros2_bridge_test.wbt`

## Recommended stack

- Ubuntu 22.04 with ROS 2 Humble
- Webots R2025a

## Install ROS 2 Python dependencies

```bash
sudo apt update
sudo apt install -y ros-humble-rclpy ros-humble-sensor-msgs ros-humble-geometry-msgs
```

If `sensor_msgs` and `geometry_msgs` are already present in your ROS installation, the command above is enough.

## Run

From the project root, run the checked launcher. It validates the ROS Python
imports, sources Humble, and starts Webots:

```bash
./run_ros2_sim.sh
```

The default `lite` sensor mode keeps locomotion responsive and publishes joint
states, IMU, and odometry. Enable the computationally expensive lidar and depth
camera only when needed:

```bash
HEXAPOD_SENSOR_MODE=full ./run_ros2_sim.sh
```

Do not open `legacy_hexapod_ros2_bridge_test.wbt` directly and do not select
`legacy_hexapod_ros2_bridge` in the Webots controller menu. The Snap build of
Webots cannot import ROS 2 from an internal controller. Always use
`./run_ros2_sim.sh`, which starts the bridge as an external controller.

The robot intentionally stands still until `/cmd_vel` is published. Keep the
command publisher running while testing movement.

## ROS 2 topics

Published:

- `/joint_states`
- `/scan` (full sensor mode)
- `/imu/data_raw`
- `/camera/depth/image_raw` (full sensor mode)
- `/odom` (GPS/IMU ground-truth pose and measured body velocity)

Subscribed:

- `/cmd_vel`

## Quick test

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /scan
```

Command the hexapod with:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08}, angular: {z: 0.0}}" -r 10
```

Keep the publisher running: the controller deliberately stops after 0.75 s
without a command. Use a negative `linear.x` value to walk backward, and press
Ctrl+C to stop publishing.

Turn in place with:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.4}}" -r 10
```

## Closed-loop path demos

`hexapod_path_demo.py` follows paths using live `/odom` feedback and publishes
combined linear/angular commands. Generated paths are rejected if their centre
line leaves the default safe arena region (`|x|` or `|y|` above 0.75 m).

```bash
source /opt/ros/humble/setup.bash
cd extracted/webots_hexapod

# Return to the arena centre.
./hexapod_path_demo.py goto --target-x 0 --target-y 0 --tolerance 0.04

# Relative paths starting from the robot's current pose.
./hexapod_path_demo.py circle --radius 0.20
./hexapod_path_demo.py figure8 --radius 0.12
./hexapod_path_demo.py square --radius 0.15
./hexapod_path_demo.py s_curve --radius 0.12
```

Only one `/cmd_vel` publisher should run at a time. Stop any manual
`ros2 topic pub` command before starting a path demo.

## Labeled obstacle course

`worlds/legacy_hexapod_ros2_bridge_test.wbt` now contains a 4 m x 4 m
modular test track. Each section has a numbered floor label and every obstacle
also has a descriptive name in the Webots scene tree:

1. `RAMP 20 DEG`: 20-degree ascent, platform, and descent.
2. `30 / 60 / 90 mm STEPS`: progressive stair-height test.
3. `UNEVEN PLATES`: alternating tilted footholds.
4. `SPEED BUMPS`: repeated transverse rounded obstacles.
5. `SLALOM`: posts for turning and combined-trajectory tests.
6. `WALL OBSTACLE CORRIDOR`: narrow passage with wall protrusions.
7. `LOW BEAM`: overhead-clearance obstacle.
8. `ROCK FIELD`: irregular blocks with varied size and orientation.
9. `BALANCE BEAM`: narrow raised route with edge markers.

The robot starts in the marked `START` zone away from the arena walls. Track
geometry is defined in `protos/HexapodTestTrack.proto`; label rendering is
defined in `protos/TrackLabel.proto`, so the course can be reused in another
world with one `HexapodTestTrack {}` node.

## Notes

- This is a first-pass bridge intended to get Webots sensors and gait control into ROS 2 quickly.
- The controller uses a stable alternating-tripod gait with a 62% stance phase,
  smooth swing-leg lift, differential turning, and a command watchdog.
- If Webots cannot import `rclpy`, make sure you launched it from a terminal where ROS 2 was sourced.
- `run_ros2_sim.sh` runs the bridge as an external Webots controller. This is
  required for the Snap build of Webots, whose sandbox cannot import ROS from
  `/opt/ros/humble` directly. The launcher uses port 1237 by default; override
  it with `WEBOTS_PORT=1238 ./run_ros2_sim.sh` if needed.
- `legacy_hexapod_load_test.wbt` loads the model without a controller;
  `legacy_hexapod_controller_test.wbt` runs the non-ROS autonomous gait.

## Run Webots on Windows and ROS 2 in a VM

This is the recommended setup when the VM has poor 3D acceleration. Keep the
ROS 2 bridge in Ubuntu and connect it over Webots' external-controller TCP
protocol, so ROS 2 DDS does not need to cross the VM boundary.

1. Copy this project directory to Windows and install the same Webots release.
2. In Windows Webots Preferences > Network, allow the Ubuntu VM address (or
   leave the incoming-address list empty), then allow inbound TCP port 1237 in
   Windows Defender Firewall.
3. In PowerShell, start Windows Webots on port 1237. The robot controller in
   the opened world must stay set to `<extern>`:

```powershell
.\start_webots_windows.ps1
```
4. In Ubuntu, connect the ROS 2 bridge to the Windows IP:

```bash
./connect_ros2_to_windows_webots.sh 192.168.1.20 1237
```

Use the Windows host's bridged/host-only adapter address, not `127.0.0.1`.
Test it first with `ping <windows-ip>`. Full perception remains optional:

```bash
HEXAPOD_SENSOR_MODE=full ./connect_ros2_to_windows_webots.sh 192.168.1.20 1237
```
- The repository is a standalone Webots project, not a colcon package; no
  `colcon build` step is required.
