#!/usr/bin/env bash
set -euo pipefail

# 开机自启：底盘 base_driver（无RViz）
# 使用 `ros2 launch base_driver base_driver.launch.py` 启动，支持串口与波特率参数。

# 配置：如需修改，编辑下方变量
ROS_DISTRO_DEFAULT="humble"
WORKSPACE_DIR="/home/zhengtuo/nailong_ros2_ws"
SERIAL_PORT="/dev/ttyACM0"
BAUDRATE="115200"

# 可选：设置ROS域ID，避免与其他机器人冲突（0-255）
: "${ROS_DOMAIN_ID:=0}"

log() { echo "[chassis] $*"; }

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

# 2) 打印信息
log "ROS_DISTRO=${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
log "serial_port=${SERIAL_PORT}, baudrate=${BAUDRATE}"

# 3) 启动底盘驱动（无RViz）
exec ros2 launch base_driver base_driver.launch.py \
  serial_port:=${SERIAL_PORT} \
  baudrate:=${BAUDRATE}
