# ROS1 到 ROS2 Humble 快速参考

## 快速对比表

### 头文件和命名空间

```cpp
// ROS1
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
```

### 节点和初始化

```cpp
// ROS1
int main(int argc, char** argv) {
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh;
    // ...
    ros::spin();
    return 0;
}

// ROS2
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 订阅和发布

```cpp
// ROS1
ros::Subscriber sub = nh.subscribe("topic", 10, callback);
ros::Publisher pub = nh.advertise<MsgType>("topic", 10);

// ROS2
auto sub = node->create_subscription<MsgType>(
    "topic", 10, 
    std::bind(&Class::callback, this, std::placeholders::_1));
auto pub = node->create_publisher<MsgType>("topic", 10);
```

### 参数

```cpp
// ROS1
double value;
nh.getParam("param_name", value);
nh.setParam("param_name", 3.14);

// ROS2
this->declare_parameter("param_name", 3.14);
double value = this->get_parameter("param_name").as_double();
this->set_parameter(rclcpp::Parameter("param_name", 3.14));
```

### 定时器

```cpp
// ROS1
ros::Timer timer = nh.createTimer(ros::Duration(0.1), 
                                  &Class::timerCallback, this);

// ROS2
auto timer = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&Class::timerCallback, this));
```

### 日志

```cpp
// ROS1
ROS_INFO("Message: %s", msg.c_str());
ROS_ERROR("Error occurred");

// ROS2
RCLCPP_INFO(this->get_logger(), "Message: %s", msg.c_str());
RCLCPP_ERROR(this->get_logger(), "Error occurred");
```

---

## API映射表

| 功能 | ROS1 | ROS2 |
|------|------|------|
| **节点初始化** | `ros::init()` | `rclcpp::init()` |
| **创建节点** | `ros::NodeHandle` | `rclcpp::Node` |
| **销毁节点** | `ros::shutdown()` | `rclcpp::shutdown()` |
| **旋转循环** | `ros::spin()` | `rclcpp::spin()` |
| **创建发布者** | `advertise<>()` | `create_publisher<>()` |
| **创建订阅者** | `subscribe()` | `create_subscription<>()` |
| **发布消息** | `pub.publish(msg)` | `pub->publish(msg)` |
| **服务器** | `advertiseService()` | `create_service<>()` |
| **服务客户端** | `service::call()` | `create_client<>()` |
| **定时器** | `createTimer()` | `create_wall_timer()` |
| **参数读取** | `getParam()` | `get_parameter()` |
| **参数设置** | `setParam()` | `set_parameter()` |
| **日志调试** | `ROS_DEBUG()` | `RCLCPP_DEBUG()` |
| **日志信息** | `ROS_INFO()` | `RCLCPP_INFO()` |
| **日志警告** | `ROS_WARN()` | `RCLCPP_WARN()` |
| **日志错误** | `ROS_ERROR()` | `RCLCPP_ERROR()` |
| **日志致命** | `ROS_FATAL()` | `RCLCPP_FATAL()` |

---

## 消息类型映射

### 常用消息前缀变化

```
ROS1:  sensor_msgs/Image          →  ROS2: sensor_msgs/msg/Image
ROS1:  geometry_msgs/Twist        →  ROS2: geometry_msgs/msg/Twist
ROS1:  nav_msgs/Odometry          →  ROS2: nav_msgs/msg/Odometry
ROS1:  std_msgs/String            →  ROS2: std_msgs/msg/String
ROS1:  std_msgs/Int32             →  ROS2: std_msgs/msg/Int32
ROS1:  tf/TransformListener       →  ROS2: tf2_ros/TransformListener
```

### 头文件变化

```cpp
// ROS1
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// ROS2
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
```

---

## 构建和编译

### 编译命令变化

```bash
# ROS1
catkin_make
catkin_make install
rosbag record -a

# ROS2
colcon build
colcon build --symlink-install
ros2 bag record -a

# 激活环境
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### CMakeLists.txt关键部分

```cmake
# ROS1
find_package(catkin REQUIRED)
catkin_package()

# ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
ament_target_dependencies(target_name rclcpp)
ament_package()
```

---

## 回调函数变化

### 订阅回调

```cpp
// ROS1
void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());
}

// ROS2 - 使用std::bind
void callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
}

// 绑定回调
auto sub = node->create_subscription<std_msgs::msg::String>(
    "topic", 10,
    std::bind(&MyClass::callback, this, std::placeholders::_1));
```

### 定时器回调

```cpp
// ROS1
void timerCallback(const ros::TimerEvent& event) {
    // event.current_real, event.last_real, 等
}

// ROS2 - 不传递事件参数
void timerCallback() {
    // 不直接获取事件信息
}
```

---

## 常见陷阱和解决方案

### 1. 头文件路径错误
```cpp
// 错误
#include <sensor_msgs/Image.h>
// 正确
#include <sensor_msgs/msg/image.hpp>
```

### 2. 日志宏需要logger
```cpp
// 错误
RCLCPP_INFO("message");
// 正确
RCLCPP_INFO(this->get_logger(), "message");
```

### 3. 消息发布
```cpp
// ROS1
msg.data = "hello";
pub.publish(msg);

// ROS2
auto msg = std::make_shared<std_msgs::msg::String>();
msg->data = "hello";
pub->publish(*msg);
```

### 4. 参数声明顺序
```cpp
// 错误
double value = this->get_parameter("param").as_double();

// 正确
this->declare_parameter("param", 0.0);
double value = this->get_parameter("param").as_double();
```

### 5. 时间表示
```cpp
// ROS1
ros::Duration(0.1)
ros::Rate(10)

// ROS2
std::chrono::milliseconds(100)
std::chrono::duration<double>(0.1)
```

---

## 话题命名约定

ROS2使用更严格的命名规则：

```bash
# ROS1 - 接受多种格式
/robot/odometry
/robot/cmd_vel
robot_odometry

# ROS2 - 统一使用小写和下划线
/robot/odometry
/robot/cmd_vel
robot_odometry

# ROS2中避免
/Robot/Odometry    # 使用大写
/robot-odometry    # 使用连字符
robot odom         # 使用空格
```

---

## 依赖项声明

### ROS1 package.xml
```xml
<buildtool_depend>catkin</buildtool_depend>
<depend>roscpp</depend>
<depend>sensor_msgs</depend>
```

### ROS2 package.xml
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
```

---

## 调试技巧

### 检查编译错误
```bash
# 清理重新编译
rm -rf build install log
colcon build --symlink-install

# 使用详细输出
colcon build --event-handlers console_direct+
```

### 运行时检查
```bash
# 列出所有节点
ros2 node list

# 列出所有话题
ros2 topic list

# 查看话题信息
ros2 topic echo /topic_name

# 查看节点信息
ros2 node info /node_name
```

### 常见命令
```bash
# 运行节点
ros2 run package_name executable_name

# 启动launch文件
ros2 launch package_name launch_file.launch.py

# 设置参数
ros2 param set /node_name param_name value

# 获取参数
ros2 param get /node_name param_name
```

---

## 文档和资源

- [ROS2官方迁移指南](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Migration-Guide.html)
- [rclcpp API文档](https://docs.ros2.org/latest/api/rclcpp/)
- [ROS2 Humble文档](https://docs.ros.org/en/humble/)
- [ROS Discourse - 迁移讨论](https://discourse.ros.org/)

