# ROS2 Humble 代码迁移示例

本文档展示本项目中具体代码的迁移过程。

## 1. 惯性导航 (inertial_nav) 迁移

### 原始ROS1代码结构
```cpp
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class InertialPathNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber cmd_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Timer control_timer_;

public:
    InertialPathNode() : nh_(), private_nh_("~") {
        // 参数读取
        private_nh_.param("odom_topic", odom_topic_, std::string("odom"));
        
        // 订阅和发布
        odom_sub_ = nh_.subscribe(odom_topic_, 50, 
                                  &InertialPathNode::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
        
        // 控制定时器
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                         &InertialPathNode::controlTimerCallback, 
                                         this);
    }
};
```

### ROS2迁移版本
```cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

class InertialPathNode : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

public:
    InertialPathNode() : rclcpp::Node("inertial_path_node") {
        // 声明和获取参数
        this->declare_parameter<std::string>("odom_topic", "odom");
        std::string odom_topic = this->get_parameter("odom_topic")
                                      .as_string();
        
        // 创建订阅
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 50,
            std::bind(&InertialPathNode::odomCallback, this, 
                     std::placeholders::_1));
        
        // 创建发布者
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10);
        
        // 创建定时器（使用chrono::milliseconds）
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_)),
            std::bind(&InertialPathNode::controlTimerCallback, this));
    }
    
private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // 回调实现
    }
    
    void controlTimerCallback() {
        // 定时器回调实现
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InertialPathNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 关键变化
| ROS1 | ROS2 | 说明 |
|------|------|------|
| `ros::NodeHandle` | `rclcpp::Node` | 节点基类改变 |
| `nh_.param()` | `declare_parameter()` + `get_parameter()` | 参数管理方式改变 |
| `nh_.subscribe()` | `create_subscription()` | 订阅API改变 |
| `nh_.advertise()` | `create_publisher()` | 发布API改变 |
| `nh_.createTimer()` | `create_wall_timer()` | 定时器API改变 |
| `ros::Timer` | `rclcpp::TimerBase` | 定时器类型改变 |
| `ros::Duration` | `std::chrono` | 时间表示改变 |

---

## 2. OpenCV图像处理迁移

### 原始ROS1代码
```cpp
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter() : it_(nh_) {
        std::string in_topic = nh_.param<std::string>("input_topic", 
                                                      "camera/image_raw");
        image_sub_ = it_.subscribe(in_topic, 1, 
                                  &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise(out_topic, 1);
        ROS_INFO("ImageConverter initialized.");
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, 
                                        sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}
```

### ROS2迁移版本
```cpp
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

class ImageConverter : public rclcpp::Node {
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter() : rclcpp::Node("image_converter") {
        // 创建image_transport
        it_ = std::make_shared<image_transport::ImageTransport>(
            std::shared_ptr<rclcpp::Node>(this, [](void*) {}));
        
        // 声明参数
        this->declare_parameter<std::string>("input_topic", 
                                            "camera/image_raw");
        this->declare_parameter<std::string>("output_topic", 
                                            "output_image");
        
        std::string in_topic = this->get_parameter("input_topic")
                                    .as_string();
        std::string out_topic = this->get_parameter("output_topic")
                                     .as_string();
        
        // 创建订阅和发布
        image_sub_ = it_->subscribe(in_topic, 1,
                                   std::bind(&ImageConverter::imageCb, 
                                            this, std::placeholders::_1));
        image_pub_ = it_->advertise(out_topic, 1);
        
        RCLCPP_INFO(this->get_logger(), "ImageConverter initialized.");
    }

private:
    void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, 
                                        sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", 
                        e.what());
            return;
        }
        // 图像处理逻辑
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 关键变化
- 继承 `rclcpp::Node` 而不是仅使用 `NodeHandle`
- `ROS_INFO` → `RCLCPP_INFO(this->get_logger(), ...)`
- 回调参数类型改变：`ConstPtr` → `ConstSharedPtr`
- `std::bind` + `std::placeholders` 用于绑定成员函数

---

## 3. UART协议通信迁移

### 原始ROS1代码片段
```cpp
#include <ros/console.h>

bool UARTProtocolROS::connect() {
    try {
        // ... 连接逻辑 ...
        ROS_ERROR("串口设备不存在: %s", port_.c_str());
        ROS_INFO("已连接到串口: %s", port_.c_str());
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("串口连接异常: %s", e.what());
        return false;
    }
}
```

### ROS2迁移版本
```cpp
#include <rclcpp/rclcpp.hpp>

class UARTProtocolROS {
private:
    std::shared_ptr<rclcpp::Logger> logger_;

public:
    UARTProtocolROS(const std::string& port, int baudrate,
                    std::shared_ptr<rclcpp::Logger> logger)
        : port_(port), baudrate_(baudrate), logger_(logger) {}
    
    bool connect() {
        try {
            // ... 连接逻辑 ...
            RCLCPP_ERROR(*logger_, "串口设备不存在: %s", port_.c_str());
            RCLCPP_INFO(*logger_, "已连接到串口: %s", port_.c_str());
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(*logger_, "串口连接异常: %s", e.what());
            return false;
        }
    }
};

// 在Node类中使用
class UARTNode : public rclcpp::Node {
    std::unique_ptr<UARTProtocolROS> uart_;
    
public:
    UARTNode() : rclcpp::Node("uart_node") {
        uart_ = std::make_unique<UARTProtocolROS>(
            port, baudrate, 
            std::make_shared<rclcpp::Logger>(this->get_logger()));
    }
};
```

---

## 4. Launch文件迁移

### ROS1 XML格式 (navi.launch)
```xml
<?xml version="1.0"?>
<launch>
  <arg name="use_sim_time" default="false"/>
  <arg name="map_file" default="map.yaml"/>
  
  <param name="use_sim_time" value="$(arg use_sim_time)"/>
  
  <node pkg="inertial_nav" type="inertial_path_node" name="inertial_nav" 
        output="screen">
    <param name="odom_topic" value="/odom"/>
    <param name="cmd_vel_topic" value="/cmd_vel"/>
  </node>
  
  <node pkg="rviz" type="rviz" name="rviz" 
        args="-d $(find inertial_nav)/config/navi.rviz"/>
</launch>
```

### ROS2 Python格式 (navi.launch.py)
```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map_file', default='map.yaml')
    
    # 获取包的共享目录
    pkg_dir = FindPackageShare('inertial_nav').find('inertial_nav')
    rviz_config = os.path.join(pkg_dir, 'config', 'navi.rviz')
    
    # 设置模拟时间
    set_use_sim_time_env = SetEnvironmentVariable('use_sim_time', use_sim_time)
    
    # 惯性导航节点
    inertial_nav_node = Node(
        package='inertial_nav',
        executable='inertial_path_node',
        name='inertial_nav',
        output='screen',
        parameters=[
            {'odom_topic': '/odom'},
            {'cmd_vel_topic': '/cmd_vel'}
        ]
    )
    
    # RViz可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value='map.yaml',
            description='Map file name'
        ),
        set_use_sim_time_env,
        inertial_nav_node,
        rviz_node
    ])
```

### 关键变化
- `.launch` (XML) → `.launch.py` (Python)
- `<node>` → `Node()`
- `$(find pkg)` → `FindPackageShare('pkg').find('pkg')`
- `<param>` → `parameters=[]`
- `<remap>` → `remappings=[]`

---

## 5. CMakeLists.txt迁移

### ROS1 CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(inertial_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(inertial_path_node src/inertial_path_node.cpp)
target_link_libraries(inertial_path_node ${catkin_LIBRARIES})
```

### ROS2 CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(inertial_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(inertial_path_node src/inertial_path_node.cpp)
target_include_directories(inertial_path_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(inertial_path_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2)

install(TARGETS inertial_path_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
```

---

## 6. package.xml迁移

### ROS1 package.xml
```xml
<?xml version="1.0"?>
<package format="2">
  <name>inertial_nav</name>
  <version>0.0.0</version>
  <description>The inertial_nav package</description>
  
  <maintainer email="author@example.com">Author Name</maintainer>
  <license>BSD</license>
  
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf</depend>
  
  <export>
  </export>
</package>
```

### ROS2 package.xml
```xml
<?xml version="1.0"?>
<package format="3">
  <name>inertial_nav</name>
  <version>0.0.0</version>
  <description>The inertial_nav package</description>
  
  <maintainer email="author@example.com">Author Name</maintainer>
  <license>BSD</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2</depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 7. 日志宏对比

| ROS1 | ROS2 | 用途 |
|------|------|------|
| `ROS_DEBUG(...)` | `RCLCPP_DEBUG(logger, ...)` | 调试信息 |
| `ROS_INFO(...)` | `RCLCPP_INFO(logger, ...)` | 一般信息 |
| `ROS_WARN(...)` | `RCLCPP_WARN(logger, ...)` | 警告 |
| `ROS_ERROR(...)` | `RCLCPP_ERROR(logger, ...)` | 错误 |
| `ROS_FATAL(...)` | `RCLCPP_FATAL(logger, ...)` | 严重错误 |

**使用示例:**
```cpp
// ROS2中使用日志
RCLCPP_INFO(this->get_logger(), "Node started");
RCLCPP_WARN(this->get_logger(), "Low battery: %d%%", battery_level);
RCLCPP_ERROR(this->get_logger(), "Failed to connect to: %s", device_name);
```

---

## 下一步

1. 逐个修改每个包的CMakeLists.txt和package.xml
2. 将C++代码中的ROS1 API替换为ROS2 API
3. 转换所有.launch文件为.launch.py
4. 使用colcon构建和测试
