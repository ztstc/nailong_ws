# ROS 2 Humble 迁移计划

本项目从ROS 1迁移到ROS 2 Humble的详细步骤。

## 项目结构分析

本项目包含以下模块：

### 1. inertial_nav
- **描述**: 惯性导航系统
- **主要文件**: `inertial_path_node.cpp`
- **需要更改**: 
  - Node类初始化
  - Publisher/Subscriber API
  - Launch文件格式

### 2. lakibeam_ws
- **描述**: 独立的ROS工作空间
- **状态**: 需要递归处理该工作空间

### 3. navi_demo01
- **描述**: 导航演示
- **文件**: 
  - Launch文件（多个）
  - Configuration文件
  - Map文件
- **需要更改**: 
  - 所有.launch → .launch.py
  - 参数格式更新
  - 导航栈节点更新

### 4. open_cv
- **描述**: OpenCV视觉处理模块
- **主要文件**: 
  - `open_cv_gray.cpp`
  - `open_cv_green_laser.cpp`
  - `open_cv_red.cpp`
  - `open_cv_trace.cpp`
- **需要更改**: 
  - ROS API更新
  - 节点初始化
  - Launch文件转换

### 5. ros_uart_protocol
- **描述**: UART协议通信
- **主要文件**: 
  - `uart_protocol_ros.cpp`
  - `uart_ros_node.cpp`
- **需要更改**: 
  - 通信API
  - 参数系统
  - Launch文件

### 6. urdf01 & urdf02
- **描述**: URDF模型和控制
- **需要更改**: 
  - Launch文件格式
  - Xacro文件兼容性检查

---

## 迁移步骤

### 第一阶段：准备工作
- [x] 创建 `ros2-humble` 分支
- [x] 创建文档和迁移指南

### 第二阶段：构建系统更新
- [ ] 更新顶层 `CMakeLists.txt`
- [ ] 更新所有子包的 `CMakeLists.txt`
- [ ] 更新所有 `package.xml` 文件

### 第三阶段：代码迁移
- [ ] 迁移 inertial_nav
- [ ] 迁移 open_cv 模块
- [ ] 迁移 ros_uart_protocol
- [ ] 迁移 urdf modules

### 第四阶段：Launch文件转换
- [ ] 转换 navi_demo01 launch文件
- [ ] 转换 open_cv launch文件
- [ ] 转换 ros_uart_protocol launch文件
- [ ] 转换 urdf launch文件

### 第五阶段：测试和验证
- [ ] 编译检查
- [ ] 运行时验证
- [ ] 功能测试

---

## 关键迁移点

### 1. ROS消息头文件
| ROS1 | ROS2 |
|------|------|
| `#include <sensor_msgs/Image.h>` | `#include <sensor_msgs/msg/image.hpp>` |
| `#include <geometry_msgs/Twist.h>` | `#include <geometry_msgs/msg/twist.hpp>` |

### 2. 节点创建
```cpp
// ROS1
ros::NodeHandle nh;

// ROS2
auto node = std::make_shared<rclcpp::Node>("node_name");
```

### 3. Publisher/Subscriber
```cpp
// ROS1风格的订阅
ros::Subscriber sub = nh.subscribe("topic", 10, callback);

// ROS2风格
auto sub = node->create_subscription<MessageType>(
    "topic", 10, callback);
```

### 4. Launch文件示例转换

**ROS1:**
```xml
<launch>
  <node pkg="pkg_name" type="node_exec" name="node_name" output="screen"/>
</launch>
```

**ROS2:**
```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_name',
            executable='node_exec',
            name='node_name',
            output='screen'
        )
    ])
```

---

## 依赖项更新

### ROS系统依赖
- `rosdep install --from-paths src --ignore-src -r -y`

### Python Launch工具
- `sudo apt install ros-humble-launch-ros`
- `sudo apt install ros-humble-launch-xml`

---

## 编译命令

```bash
# 清理之前的构建
rm -rf build install log

# 使用colcon构建
colcon build --symlink-install

# 激活环境
source install/setup.bash
```

---

## 验证清单

- [ ] 所有包都成功编译
- [ ] 没有编译警告
- [ ] 所有节点都能启动
- [ ] 话题发布/订阅工作正常
- [ ] 服务调用工作正常
- [ ] 参数设置工作正常
- [ ] Launch文件启动正常

---

## 常见问题处理

### 编译错误: 找不到ament_cmake
**解决**: 确保安装了ROS 2 Humble
```bash
source /opt/ros/humble/setup.bash
```

### 导入错误: No module named 'launch'
**解决**: 安装launch-ros
```bash
sudo apt install ros-humble-launch-ros python3-launch-ros-pygments
```

### Node类找不到方法
**解决**: 检查ROS2版本，某些API在不同版本中可能有差异

---

## 进度跟踪

- 创建时间: 2025-12-16
- 计划完成时间: 2025-12-31
- 当前状态: 已创建ros2-humble分支，开始代码迁移

