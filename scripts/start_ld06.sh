#!/usr/bin/env bash
set -euo pipefail

# 开机自启：LD06 激光雷达驱动（无RViz）
# 使用 `ros2 launch ldlidar_stl_ros2 ld06.launch.py` 启动。

ROS_DISTRO_DEFAULT="humble"
WORKSPACE_DIR="/home/zhengtuo/nailong_ros2_ws"

# 如需覆盖串口或其他参数，请在此处修改，或通过launch参数覆盖：
# 该launch默认端口为 /dev/ttyUSB0，波特率 230400
PORT_NAME_OVERRIDE=""   # 示例："/dev/ld06" 或 "/dev/ttyUSB0"；为空表示使用launch默认

: "${ROS_DOMAIN_ID:=0}"

log() { echo "[ld06] $*"; }

# 1) 载入ROS与工作区环境
if [[ -f "/opt/ros/${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}/setup.bash" ]]; then
  source "/opt/ros/${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}/setup.bash"
else
  echo "未找到ROS环境：/opt/ros/${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}/setup.bash" >&2
  exit 1
fi
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
else
  echo "未找到工作区安装环境：${WORKSPACE_DIR}/install/setup.bash。请先构建：colcon build" >&2
  exit 1
fi

log "ROS_DISTRO=${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

# 2) 启动LD06（无RViz）
if [[ -n "${PORT_NAME_OVERRIDE}" ]]; then
  exec ros2 launch ldlidar_stl_ros2 ld06.launch.py port_name:=${PORT_NAME_OVERRIDE}
else
  exec ros2 launch ldlidar_stl_ros2 ld06.launch.py
fi
