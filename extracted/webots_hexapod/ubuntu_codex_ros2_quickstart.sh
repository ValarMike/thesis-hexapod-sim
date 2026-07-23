#!/usr/bin/env bash
set -euo pipefail

echo "[1/6] Install base tools..."
sudo apt update
sudo apt install -y curl git build-essential python3-pip python3-venv

echo "[2/6] Install Codex CLI..."
curl -fsSL https://chatgpt.com/codex/install.sh | sh

echo "[3/6] Install ROS 2 Humble dependencies for this Webots project..."
sudo apt install -y \
  software-properties-common \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-rclpy \
  ros-humble-desktop \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "[4/6] Initialize rosdep..."
  sudo rosdep init
fi
rosdep update

echo "[5/6] Add ROS 2 environment to ~/.bashrc if needed..."
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

echo "[6/6] Done."
echo
echo "Next:"
echo "  source /opt/ros/humble/setup.bash"
echo "  cd ~/hexapod_ws/src/webots_hexapod"
echo "  codex"
echo
echo "Inside Codex, try:"
echo '  "Read this Webots + ROS2 project and help me run the hexapod simulation."'
