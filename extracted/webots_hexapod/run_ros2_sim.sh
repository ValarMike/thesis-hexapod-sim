#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_setup=/opt/ros/humble/setup.bash
world="$project_dir/worlds/legacy_hexapod_ros2_bridge_test.wbt"
controller="$project_dir/controllers/legacy_hexapod_ros2_bridge/legacy_hexapod_ros2_bridge.py"
webots_port="${WEBOTS_PORT:-1237}"

if [[ ! -f "$ros_setup" ]]; then
  echo "ROS 2 Humble was not found at $ros_setup." >&2
  echo "Install ros-humble-desktop, then run this script again." >&2
  exit 1
fi

if ! command -v webots >/dev/null 2>&1; then
  echo "Webots was not found on PATH." >&2
  exit 1
fi

# ROS 2's generated setup scripts may probe unset variables. Temporarily
# disable nounset while sourcing, then restore this launcher's strict mode.
set +u
source "$ros_setup"
set -u

python3 - <<'PY'
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
print("ROS 2 Python dependencies: OK")
PY

webots_home="${WEBOTS_HOME:-}"
if [[ -z "$webots_home" && -x /snap/webots/current/usr/share/webots/webots-controller ]]; then
  webots_home=/snap/webots/current/usr/share/webots
fi

if [[ -z "$webots_home" || ! -x "$webots_home/webots-controller" ]]; then
  echo "Could not locate webots-controller for the external ROS 2 controller." >&2
  echo "Set WEBOTS_HOME to the Webots installation directory." >&2
  exit 1
fi

echo "Starting $world"
webots --port="$webots_port" "$@" "$world" &
webots_pid=$!

cleanup() {
  if kill -0 "$webots_pid" 2>/dev/null; then
    kill "$webots_pid" 2>/dev/null || true
    wait "$webots_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Connecting external ROS 2 controller"
WEBOTS_HOME="$webots_home" "$webots_home/webots-controller" \
  --port="$webots_port" --robot-name=legacy_hexapod_ros2 "$controller"
