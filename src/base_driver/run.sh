#!/bin/bash
# ROS2 Humble Base Driver 快速启动脚本

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}ROS2 Humble Base Driver启动向导${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查环境
echo -e "\n${YELLOW}[检查环境]${NC}"

# 检查ROS2安装
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ 未检测到ROS2环境${NC}"
    echo "请先执行: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓ ROS2版本: ${ROS_DISTRO}${NC}"
fi

# 检查libserial
if ! pkg-config --exists libserial; then
    echo -e "${RED}❌ libserial库未安装${NC}"
    echo "请先执行: sudo apt-get install libserial-dev"
    exit 1
else
    echo -e "${GREEN}✓ libserial已安装${NC}"
fi

# 检查串口设备
echo -e "\n${YELLOW}[检查串口设备]${NC}"
SERIAL_DEVICES=$(ls /dev/tty{ACM,USB,S}* 2>/dev/null)
if [ -z "$SERIAL_DEVICES" ]; then
    echo -e "${YELLOW}⚠ 未检测到串口设备${NC}"
    echo "可用的设备: $SERIAL_DEVICES"
else
    echo -e "${GREEN}✓ 检测到串口设备:${NC}"
    echo "$SERIAL_DEVICES"
fi

# 编译项目
echo -e "\n${YELLOW}[编译项目]${NC}"
cd /home/zhengtuo/nailong_ros2_ws
source install/setup.bash
colcon build --packages-select base_driver

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ 编译失败${NC}"
    exit 1
fi
echo -e "${GREEN}✓ 编译成功${NC}"

# 运行节点
echo -e "\n${YELLOW}[启动节点]${NC}"
source install/setup.bash
echo -e "${GREEN}启动base_driver_node...${NC}"
ros2 run base_driver base_driver_node --ros-args -p serial_port:=/dev/ttyACM0
