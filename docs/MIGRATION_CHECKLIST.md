# ROS2 Humble 迁移检查清单

使用本清单追踪项目的ROS2迁移进度。

## 项目概览
- **项目名称**: nailong_ws
- **源分支**: main (ROS1)
- **目标分支**: ros2-humble
- **开始日期**: 2025-12-16
- **目标完成日期**: 2025-12-31

---

## 1. 准备阶段 ✅

- [x] 创建git分支: `ros2-humble`
- [x] 创建docs文件夹
- [x] 编写ROS1 vs ROS2指南
- [x] 编写代码迁移示例
- [x] 编写快速参考指南

---

## 2. 包级别迁移 - inertial_nav

### 配置文件
- [ ] 更新 `package.xml` (格式2 → 3)
- [ ] 更新 `CMakeLists.txt`
  - [ ] 替换 `find_package(catkin ...)` 为 `find_package(ament_cmake ...)`
  - [ ] 替换 `catkin_package()` 为 `ament_package()`
  - [ ] 添加 `install()` 指令

### C++代码
- [ ] `inertial_path_node.cpp`
  - [ ] 替换 `#include <ros/ros.h>` 为 `#include <rclcpp/rclcpp.hpp>`
  - [ ] 替换消息头文件路径 (`.h` → `.hpp`)
  - [ ] 将类继承自 `rclcpp::Node`
  - [ ] 替换 `ros::NodeHandle` 的初始化
  - [ ] 更新参数声明/获取方式
  - [ ] 更新订阅创建方式
  - [ ] 更新发布创建方式
  - [ ] 更新定时器创建方式
  - [ ] 替换日志宏 (ROS_* → RCLCPP_*)
  - [ ] 更新 main() 函数
  - [ ] 更新所有回调函数签名

### Launch文件
- [ ] 转换 `inertial_nav.launch` (XML → Python)
  - [ ] 创建 `inertial_nav.launch.py`
  - [ ] 删除/备份 `inertial_nav.launch`

### 测试
- [ ] 编译成功
- [ ] 无编译警告
- [ ] 运行检查

---

## 3. 包级别迁移 - open_cv

### 配置文件
- [ ] 更新 `package.xml`
- [ ] 更新 `CMakeLists.txt`
  - [ ] 替换构建工具链为 ament_cmake
  - [ ] 添加 `rclcpp` 依赖

### C++代码 - open_cv_gray.cpp
- [ ] 更新头文件包含
- [ ] 将 `ImageConverter` 继承自 `rclcpp::Node`
- [ ] 更新节点初始化
- [ ] 更新参数管理
- [ ] 更新image_transport订阅和发布
- [ ] 替换日志宏
- [ ] 更新main函数
- [ ] 编译测试

### C++代码 - open_cv_green_laser.cpp
- [ ] 重复上述步骤

### C++代码 - open_cv_red.cpp
- [ ] 重复上述步骤

### C++代码 - open_cv_trace.cpp
- [ ] 重复上述步骤

### Launch文件
- [ ] 转换 `cam_gray.launch` → `cam_gray.launch.py`
- [ ] 转换 `cam_green_laser.launch` → `cam_green_laser.launch.py`
- [ ] 转换 `cam_red_square.launch` → `cam_red_square.launch.py`
- [ ] 转换 `trace.launch` → `trace.launch.py`

---

## 4. 包级别迁移 - ros_uart_protocol

### 配置文件
- [ ] 更新 `package.xml`
- [ ] 更新 `CMakeLists.txt`

### 头文件
- [ ] `include/ros_uart_protocol/uart_protocol_ros.h`
  - [ ] 更新ROS相关头文件
  - [ ] 添加 `rclcpp::Logger` 成员

### C++代码
- [ ] `uart_protocol_ros.cpp`
  - [ ] 替换 `#include <ros/console.h>` 为 `#include <rclcpp/rclcpp.hpp>`
  - [ ] 更新日志使用方式
  - [ ] 接受 logger 参数
  - [ ] 编译测试

- [ ] `uart_ros_node.cpp`
  - [ ] 更新头文件
  - [ ] 创建 `UARTNode` 类继承 `rclcpp::Node`
  - [ ] 实现节点初始化
  - [ ] 创建service/publisher等
  - [ ] 更新main函数
  - [ ] 编译测试

### Launch文件
- [ ] 转换 `launch_node.launch` → `launch_node.launch.py`
- [ ] 转换 `uvc_cam.launch` → `uvc_cam.launch.py`

---

## 5. 包级别迁移 - navi_demo01

### 配置文件
- [ ] 更新 `package.xml`
- [ ] 更新 `CMakeLists.txt` (如果有cpp代码)

### Launch文件
- [ ] 转换 `amcl.launch` → `amcl.launch.py`
- [ ] 转换 `gmapping_joy.launch` → `gmapping_joy.launch.py`
- [ ] 转换 `gmapping.launch` → `gmapping.launch.py`
- [ ] 转换 `joy.launch` → `joy.launch.py`
- [ ] 转换 `map_amcl.launch` → `map_amcl.launch.py`
- [ ] 转换 `movebase.launch` → `movebase.launch.py`
- [ ] 转换 `navi.launch` → `navi.launch.py`

### 参数文件
- [ ] 检查YAML配置文件是否需要更新
- [ ] 更新参数格式（如有变化）

---

## 6. 包级别迁移 - urdf01

### 配置文件
- [ ] 更新 `package.xml`
- [ ] 更新 `CMakeLists.txt`

### Launch文件
- [ ] 转换 `demo01_helloworld.launch` → `.launch.py`
- [ ] 转换 `demo02_link.launch` → `.launch.py`
- [ ] 转换 `demo03_joint.launch` → `.launch.py`
- [ ] 转换 `demo04_base_footprint.launch` → `.launch.py`
- [ ] 转换 `demo07_control.launch` → `.launch.py`
- [ ] 转换 `nailong_model.launch` → `.launch.py`

### URDF/Xacro文件
- [ ] 检查兼容性
- [ ] 更新ROS1特定的语法

---

## 7. 包级别迁移 - urdf02

### 配置文件
- [ ] 更新 `package.xml`
- [ ] 更新 `CMakeLists.txt`

### Launch文件
- [ ] 转换所有 `.launch` 文件

### URDF/Xacro文件
- [ ] 检查兼容性
- [ ] 更新语法

---

## 8. 包级别迁移 - lakibeam_ws

**注**: 这是一个独立的工作空间，可能需要特殊处理

- [ ] 检查其内部结构
- [ ] 决定是否集成到主工作空间
- [ ] 如果独立，按相同流程迁移
- [ ] 或者从主工作空间中移除

---

## 9. 顶层配置

### 根 CMakeLists.txt
- [ ] 更新为ROS2兼容版本

### 根 package.xml
- [ ] 如果存在，更新为格式3
- [ ] 更新依赖声明

---

## 10. 编译和测试

### 初始编译
- [ ] 清理旧构建目录
  ```bash
  rm -rf build install log
  ```
- [ ] 第一次全编译
  ```bash
  colcon build --symlink-install
  ```
- [ ] 解决编译错误
- [ ] 验证无编译警告

### 功能测试
- [ ] 逐个运行节点
  ```bash
  ros2 run package_name executable
  ```
- [ ] 验证话题发布/订阅
  ```bash
  ros2 topic list
  ros2 topic echo /topic_name
  ```
- [ ] 验证服务（如有）
  ```bash
  ros2 service list
  ros2 service call /service_name ...
  ```
- [ ] 验证参数（如有）
  ```bash
  ros2 param list
  ros2 param get /node_name param_name
  ```

### Launch文件测试
- [ ] 测试每个 `.launch.py` 文件
  ```bash
  ros2 launch package_name launch_file.launch.py
  ```
- [ ] 检查所有节点启动成功
- [ ] 检查参数加载正确

---

## 11. 文档更新

- [ ] 更新README.md
- [ ] 更新安装说明
- [ ] 添加ROS2特定的说明
- [ ] 更新依赖列表

---

## 12. 提交和备份

- [ ] 提交所有更改到 `ros2-humble` 分支
  ```bash
  git add .
  git commit -m "Migrate project to ROS2 Humble"
  git push origin ros2-humble
  ```
- [ ] 创建发布标签
- [ ] 保留 main 分支作为ROS1版本备份

---

## 常见问题排查

### 编译错误

| 错误 | 原因 | 解决方案 |
|------|------|--------|
| `find_package(catkin)` not found | 仍使用ROS1构建系统 | 替换为 `find_package(ament_cmake)` |
| `#include <ros/ros.h>` not found | 头文件路径错误 | 替换为 `#include <rclcpp/rclcpp.hpp>` |
| `NodeHandle` not declared | 未继承rclcpp::Node | 将类继承自rclcpp::Node或创建node成员 |
| `ament_package()` not defined | CMakeLists.txt缺少依赖 | 添加 `find_package(ament_cmake REQUIRED)` |
| Image message not found | 消息头文件路径错误 | 检查消息头文件包含路径 |

### 运行时错误

| 错误 | 原因 | 解决方案 |
|------|------|--------|
| `Could not find ROS package` | package.xml或CMakeLists有问题 | 检查构建和安装步骤 |
| `Topic not found` | 订阅或发布配置错误 | 使用 `ros2 topic list` 检查 |
| `Parameter not found` | 未声明参数 | 在节点初始化时调用 `declare_parameter()` |
| `Callback not called` | 回调绑定错误 | 检查 `std::bind()` 和 `std::placeholders` |

---

## 进度统计

```
总包数: 8 (inertial_nav, open_cv, ros_uart_protocol, navi_demo01, 
           urdf01, urdf02, lakibeam_ws, 顶层)

完成: [ ] / 8
进度: 0%
```

更新此表格以跟踪整体进度。

---

## 签名和日期

- 迁移开始: 2025-12-16
- 预计完成: 2025-12-31
- 实际完成: ___________
- 执行人: ___________
- 审核人: ___________

---

## 附注

在迁移过程中记录任何特殊问题或注意事项：

```
[在此添加笔记]
```

