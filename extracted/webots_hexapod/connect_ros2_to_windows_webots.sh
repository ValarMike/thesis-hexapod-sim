#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 <windows-ip> [webots-port]" >&2
  echo "Example: $0 192.168.1.20 1237" >&2
  exit 2
fi

windows_ip="$1"
webots_port="${2:-1237}"
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
controller="$project_dir/controllers/legacy_hexapod_ros2_bridge/legacy_hexapod_ros2_bridge.py"
ros_setup=/opt/ros/humble/setup.bash

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS 2 Humble was not found at $ros_setup." >&2
  exit 1
fi

set +u
source "$ros_setup"
set -u

webots_home="${WEBOTS_HOME:-}"
if [[ -z "$webots_home" && -x /snap/webots/current/usr/share/webots/webots-controller ]]; then
  webots_home=/snap/webots/current/usr/share/webots
fi

if [[ -z "$webots_home" || ! -x "$webots_home/webots-controller" ]]; then
  echo "Could not locate webots-controller; set WEBOTS_HOME first." >&2
  exit 1
fi

python3 - <<'PY'
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
print("ROS 2 Python dependencies: OK")
PY

echo "Connecting ROS 2 controller to Webots at $windows_ip:$webots_port"
echo "Target robot: legacy_hexapod_ros2"
WEBOTS_HOME="$webots_home" "$webots_home/webots-controller" \
  --protocol=tcp \
  --ip-address="$windows_ip" \
  --port="$webots_port" \
  --robot-name=legacy_hexapod_ros2 \
  "$controller"
