# Base Driver - ROS2 Humble 版本

一个为 ROS2 Humble 优化的机器人底盘驱动程序，提供串口通信、电池管理、里程计计算等功能。

## ✨ 特性

- **ROS2 Humble 原生支持** - 完全重写为现代ROS2风格
- **优化的串口通信** - 改进的错误处理和自动重连机制
- **完整的传感器支持** - 电池监测、电机控制、舵机控制、灯光控制
- **里程计计算** - 实时计算机器人位置和速度，发布TF坐标变换
- **系统监测** - CPU和RAM使用率监测
- **日志记录** - 结构化的日志输出便于调试

## 📋 依赖

### 系统依赖
- Ubuntu 22.04 LTS (Jammy)
- ROS2 Humble
- C++17 编译器

### 库依赖
```bash
sudo apt-get update
sudo apt-get install libserial-dev
```

### ROS2 包依赖
- rclcpp
- std_msgs
- sensor_msgs
- geometry_msgs
- nav_msgs
- tf2_ros

## 🔧 编译安装

```bash
# 进入工作空间
cd ~/nailong_ros2_ws

# Source ROS2环境
source /opt/ros/humble/setup.bash

# 编译base_driver包
colcon build --packages-select base_driver

# Source工作空间
source install/setup.bash
```

## 🚀 快速开始

### 基本运行
```bash
ros2 run base_driver base_driver_node
```

### 带参数运行
```bash
ros2 run base_driver base_driver_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baudrate:=115200 \
  -p battery_voltage_min:=24.0 \
  -p battery_voltage_max:=27.0
```

### 使用Launch文件（推荐）
创建 `launch/base_driver.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_driver',
            executable='base_driver_node',
            name='uart_protocol_node',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baudrate': 115200,
                'battery_voltage_min': 24.0,
                'battery_voltage_max': 27.0,
                'system_stats_rate_hz': 1.0,
                'max_linear_speed': 0.8,
                'max_angular_speed': 1.2,
            }]
        ),
    ])
```

运行:
```bash
ros2 launch base_driver base_driver.launch.py
```

## 📡 话题接口

### 订阅的话题 (输入)

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 (linear.x, angular.z) |
| `/servo_cmd` | `sensor_msgs/JointState` | 舵机控制命令 |
| `/light_cmd` | `std_msgs/Float32MultiArray` | 灯光控制 [enable, brightness] |
| `/motor_enable` | `std_msgs/UInt8` | 电机启停 (0=禁用, 1=启用, 0xFF=切换) |

### 发布的话题 (输出)

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/robot_status` | `std_msgs/Float32MultiArray` | 机器人状态 [voltage, speedL, speedR, CPU, RAM, ...] |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态 (轮子和舵机速度) |
| `/battery_voltage` | `std_msgs/Float32` | 电池电压 (V) |
| `/battery_percentage` | `std_msgs/Float32` | 电池百分比 (%) |
| `/device_online` | `std_msgs/Bool` | 设备在线状态 |
| `/odom` | `nav_msgs/Odometry` | 里程计数据 |
| `/light_status` | `std_msgs/Float32MultiArray` | 灯光状态 [enabled, brightness] |
| `/cpu_usage` | `std_msgs/Float32` | MCU CPU使用率 (%) |
| `/ram_usage` | `std_msgs/Float32` | MCU RAM使用率 (%) |
| `/system_cpu_usage` | `std_msgs/Float32` | 系统CPU使用率 (%) |
| `/system_ram_usage` | `std_msgs/Float32` | 系统RAM使用率 (%) |

## 🔋 配置参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `serial_port` | string | `/dev/ttyACM0` | 串口设备路径 |
| `baudrate` | int | `115200` | 波特率 |
| `battery_voltage_min` | double | `24.0` | 电池最小电压(V) |
| `battery_voltage_max` | double | `27.0` | 电池最大电压(V) |
| `system_stats_rate_hz` | double | `1.0` | 系统统计频率(Hz) |
| `max_linear_speed` | double | `0.8` | 最大线速度(m/s) |
| `max_angular_speed` | double | `1.2` | 最大角速度(rad/s) |

## 🐛 故障排查

### 连接问题

**问题：无法打开串口设备**
```
错误: No permission to access serial device: /dev/ttyACM0
```

**解决方案：**
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER
# 重启或新建终端
newgrp dialout
```

**问题：找不到串口设备**
```bash
# 检查连接的设备
ls -la /dev/tty*
```

### 通信问题

**问题：CRC校验失败**
- 检查波特率设置是否匹配
- 检查USB线是否接触良好
- 查看MCU的日志输出

**问题：心跳超时**
- 检查MCU是否正常运行
- 查看串口波特率
- 确认MCU固件正确

## 📊 主要改进点 (vs ROS1版本)

1. **现代C++** - 使用 unique_ptr、智能引用等现代特性
2. **更好的错误处理** - 错误计数机制避免频繁断连
3. **结构化日志** - RCLCPP_ERROR/WARN/INFO/DEBUG 日志宏
4. **参数系统** - ROS2原生参数管理
5. **TF2支持** - 使用现代TF2坐标变换库
6. **资源管理** - 自动化的资源清理

## 📚 文档

- [REFACTORING_NOTES.md](REFACTORING_NOTES.md) - 详细的重构说明
- [run.sh](run.sh) - 快速启动脚本

## 📝 许可证

MIT License

## 👨‍💻 作者

维护者: zhengtuo (24320131@hdu.edu.cn)

## 🔗 相关资源

- [ROS2 官方文档](https://docs.ros.org/en/humble/)
- [libserial 文档](https://github.com/crayzeewulf/libserial)
