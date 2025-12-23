# ROS1 vs ROS2 迁移指南

本指南帮助初学者理解ROS1和ROS2的主要区别，以及如何将ROS1代码迁移到ROS2 Humble。

## 目录
1. [核心架构差异](#核心架构差异)
2. [编程API变化](#编程api变化)
3. [构建系统](#构建系统)
4. [Launch文件](#launch文件)
5. [消息和服务](#消息和服务)
6. [迁移检查清单](#迁移检查清单)

---

## 核心架构差异

### ROS1特点
- **单主节点(Master)**: 所有节点必须连接到中央ROS Master
- **同步通信**: 基于TCP/UDP，依赖于单点Master
- **实时性有限**: 不是实时操作系统

### ROS2特点
- **分布式架构**: 无中央Master，基于DDS(Data Distribution Service)
- **异步通信**: 基于发布-订阅模型，支持多种通信中间件
- **实时性支持**: 可支持硬实时应用
- **更好的可扩展性**: 支持多机器协作

---

## 编程API变化

### 1. 节点初始化

**ROS1:**
```cpp
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    
    // 创建发布者
    ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 10);
    
    ros::spin();
    return 0;
}
```

**ROS2:**
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("my_node");
    
    // 创建发布者
    auto pub = node->create_publisher<std_msgs::msg::String>("chatter", 10);
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 2. 发布者/订阅者

**ROS1:**
```cpp
// 发布
ros::Publisher pub = nh.advertise<std_msgs::String>("topic", 10);
std_msgs::String msg;
msg.data = "Hello";
pub.publish(msg);

// 订阅
void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}
ros::Subscriber sub = nh.subscribe("topic", 10, callback);
```

**ROS2:**
```cpp
// 发布
auto pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
auto msg = std::make_shared<std_msgs::msg::String>();
msg->data = "Hello";
pub->publish(*msg);

// 订阅
auto sub = node->create_subscription<std_msgs::msg::String>(
    "topic", 10,
    [](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(node->get_logger(), "I heard: [%s]", msg->data.c_str());
    });
```

### 3. 服务/客户端

**ROS1:**
```cpp
// 服务
bool serviceCallback(example_pkg::MyService::Request& req,
                     example_pkg::MyService::Response& res) {
    res.result = req.input * 2;
    return true;
}
ros::ServiceServer server = nh.advertiseService("service", serviceCallback);

// 客户端
example_pkg::MyService srv;
srv.request.input = 5;
ros::service::call("service", srv);
```

**ROS2:**
```cpp
// 服务
auto callback = [](const std::shared_ptr<example_pkg::srv::MyService::Request> req,
                   std::shared_ptr<example_pkg::srv::MyService::Response> res) {
    res->result = req->input * 2;
};
auto server = node->create_service<example_pkg::srv::MyService>("service", callback);

// 客户端
auto client = node->create_client<example_pkg::srv::MyService>("service");
auto request = std::make_shared<example_pkg::srv::MyService::Request>();
request->input = 5;
auto result = client->async_send_request(request);
```

### 4. 日志记录

**ROS1:**
```cpp
ROS_DEBUG("Debug message");
ROS_INFO("Info message");
ROS_WARN("Warning message");
ROS_ERROR("Error message");
ROS_FATAL("Fatal message");
```

**ROS2:**
```cpp
RCLCPP_DEBUG(node->get_logger(), "Debug message");
RCLCPP_INFO(node->get_logger(), "Info message");
RCLCPP_WARN(node->get_logger(), "Warning message");
RCLCPP_ERROR(node->get_logger(), "Error message");
RCLCPP_FATAL(node->get_logger(), "Fatal message");
```

### 5. 参数管理

**ROS1:**
```cpp
ros::NodeHandle nh("~");
std::string param_value;
nh.getParam("param_name", param_value);
nh.setParam("param_name", "new_value");
```

**ROS2:**
```cpp
auto param = node->declare_parameter("param_name", "default_value");
auto param_value = node->get_parameter("param_name").as_string();
node->set_parameter(rclcpp::Parameter("param_name", "new_value"));
```

---

## 构建系统

### ROS1 CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(example_pkg)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node ${catkin_LIBRARIES})
```

### ROS2 CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(example_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC include)
ament_target_dependencies(my_node rclcpp std_msgs)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

---

## Launch文件

### ROS1 .launch (XML格式)
```xml
<?xml version="1.0"?>
<launch>
  <arg name="use_sim_time" default="false"/>
  
  <node pkg="example_pkg" type="my_node" name="my_node" output="screen">
    <param name="param_name" value="param_value"/>
    <remap from="input" to="output"/>
  </node>
</launch>
```

### ROS2 .launch.py (Python格式)
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock'
        ),
        
        Node(
            package='example_pkg',
            executable='my_node',
            name='my_node',
            output='screen',
            parameters=[{'param_name': 'param_value'}],
            remappings=[('input', 'output')]
        )
    ])
```

---

## 消息和服务

### ROS1 消息定义 (.msg)
```
# 文件: msg/MyMessage.msg
string name
int32 age
float64 score
```

### ROS2 消息定义 (.msg)
与ROS1相同，但在CMakeLists.txt中的配置不同：

**ROS1:**
```cmake
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES MyMessage.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
```

**ROS2:**
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"
)
ament_package()
```

---

## 迁移检查清单

### 代码级别
- [ ] 替换 `#include <ros/ros.h>` 为 `#include <rclcpp/rclcpp.hpp>`
- [ ] 更改 `ros::NodeHandle` 为 `rclcpp::Node`
- [ ] 更新发布者创建方式
- [ ] 更新订阅者创建和回调方式
- [ ] 更新服务创建和调用方式
- [ ] 替换所有日志调用（ROS_INFO → RCLCPP_INFO）
- [ ] 更新参数管理代码
- [ ] 替换 `ros::spin()` 为 `rclcpp::spin()`

### 配置文件
- [ ] 更新 CMakeLists.txt（catkin → ament_cmake）
- [ ] 更新 package.xml（格式版本 1 → 2）
- [ ] 转换所有 .launch 文件为 .launch.py

### 构建和测试
- [ ] 确保 C++ 标准 ≥ 17
- [ ] 使用 `colcon build` 代替 `catkin_make`
- [ ] 检查所有依赖项都已安装

---

## 常见问题解答

**Q: 为什么ROS2没有Master？**
A: ROS2使用DDS中间件，支持分布式发现，不需要单点Master。

**Q: ROS2性能更好吗？**
A: 是的，DDS提供了更低的延迟和更高的吞吐量。

**Q: 我可以混合使用ROS1和ROS2吗？**
A: 可以，通过 `ros1_bridge` 包实现，但不推荐用于生产环境。

**Q: 学习ROS2需要多长时间？**
A: 如果你已了解ROS1，通常2-4周即可掌握主要概念。

---

## 参考资源

- [ROS2官方迁移指南](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Migration-Guide.html)
- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [rclcpp API文档](https://docs.ros2.org/latest/api/rclcpp/)
- [ROS Discourse论坛](https://discourse.ros.org/)
