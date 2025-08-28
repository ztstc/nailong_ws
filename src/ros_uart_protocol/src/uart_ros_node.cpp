#include "ros_uart_protocol/uart_protocol_ros.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class UARTRosNode {
public:
    //UARTRosNode() : nh_("~"),
    UARTRosNode() : nh_(),
        x_(0.0), y_(0.0), theta_(0.0), last_odom_time_(ros::Time::now())
    {
        // 获取参数
        std::string port;
        int baudrate;
        nh_.param<std::string>("serial_port", port, "/dev/ttyACM0");  // 默认串口 也可能是 "/dev/ttyACM0"
        nh_.param("baudrate", baudrate, 115200);
        
        // 初始化协议处理器
        protocol_ = std::make_unique<UARTProtocolROS>(port, baudrate);
        
        // 连接串口
        if (!protocol_->connect()) {
            ROS_FATAL("Failed to connect to serial device");
            ros::shutdown();
            return;
        }
        
        // 创建订阅器
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &UARTRosNode::cmdVelCallback, this);
        servo_cmd_sub_ = nh_.subscribe("servo_cmd", 10, &UARTRosNode::servoCmdCallback, this);
        
        // 创建发布器
        status_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("robot_status", 10);
        joint_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);
        motor_speed_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("motor_speed", 10);
        battery_pub_ = nh_.advertise<std_msgs::Float32>("battery_voltage", 10);
        device_online_pub_ = nh_.advertise<std_msgs::Bool>("device_online", 10);
        // 添加CPU和RAM发布器
        cpu_usage_pub_ = nh_.advertise<std_msgs::Float32>("cpu_usage", 10);
        ram_usage_pub_ = nh_.advertise<std_msgs::Float32>("ram_usage", 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
        
        // 创建定时器
        status_timer_ = nh_.createTimer(ros::Duration(0.033), &UARTRosNode::statusTimerCallback, this); //30Hz
        request_timer_ = nh_.createTimer(ros::Duration(1.0), &UARTRosNode::requestTimerCallback, this); //1Hz
        online_check_timer_ = nh_.createTimer(ros::Duration(0.1), &UARTRosNode::onlineCheckTimerCallback, this); //10Hz
        
        ROS_INFO("UART ROS node initialized");
    }
    
    ~UARTRosNode() {
        protocol_->disconnect();
    }
    
private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 将线速度和角速度转换为左右轮速
        // 这里需要根据机器人参数进行转换
        float linear = msg->linear.x;
        float angular = msg->angular.z;
        
        // 示例转换公式 (需要根据实际机器人参数调整)
        float wheel_separation = 0.425; // 轮间距
        float wheel_radius = 0.085;    // 轮半径
        float left_speed = (linear - angular * wheel_separation / 2.0) / wheel_radius;
        float right_speed = (linear + angular * wheel_separation / 2.0) / wheel_radius;
        
        // 发送运动命令
        protocol_->sendMotionCommand(
            static_cast<float>(left_speed), // 缩放因子
            static_cast<float>(right_speed)
        );
    }
    
    void servoCmdCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i].find("servo") != std::string::npos) {
                // 提取舵机ID (假设名称格式为 "servoX")
                int servo_id = std::stoi(msg->name[i].substr(5));
                float angle_rad = msg->position[i];
                int16_t angle_deg = static_cast<int16_t>(angle_rad * 180.0 / M_PI);
                
                // 发送舵机命令
                protocol_->sendServoCommand(servo_id, angle_deg);
            }
        }
    }
    
    void statusTimerCallback(const ros::TimerEvent&) {
        // 获取最新状态
        auto status = protocol_->getLatestStatus();
        
        // 发布状态消息
        std_msgs::Float32MultiArray status_msg;
        status_msg.data = {
            status.batteryVoltage,
            status.speedL,
            status.speedR,
            static_cast<float>(status.CPU),
            static_cast<float>(status.RAM),
            status.motorCurrentL,
            status.motorCurrentR,
            status.motorCurrent
        };
        status_pub_.publish(status_msg);
        
        // 发布电池电压，过滤掉0.0，发布上一次有效值
        std_msgs::Float32 battery_msg;
        if (status.batteryVoltage > 0.01) {
            last_valid_battery_voltage_ = status.batteryVoltage;
        }
        battery_msg.data = last_valid_battery_voltage_;
        battery_pub_.publish(battery_msg);
        
        // 发布关节状态
        sensor_msgs::JointState joint_msg;
        joint_msg.header.stamp = ros::Time::now();
        
        // 添加轮子关节
        joint_msg.name.push_back("wheel_left_joint");
        joint_msg.name.push_back("wheel_right_joint");
        joint_msg.position.push_back(0.0);  // 轮子位置不重要
        joint_msg.position.push_back(0.0);
        joint_msg.velocity.push_back(status.speedL * M_PI / 30.0);  // RPM to rad/s
        joint_msg.velocity.push_back(status.speedR * M_PI / 30.0);
        
        // 添加舵机关节
        for (int i = 0; i < 10; ++i) {
            if (status.onlineservoid[i] != 0) {
                std::string joint_name = "servo" + std::to_string(i);
                joint_msg.name.push_back(joint_name);
                joint_msg.position.push_back(status.servo[i] * M_PI / 180.0);
                joint_msg.velocity.push_back(0.0);  // 舵机速度未知
            }
        }
        joint_pub_.publish(joint_msg);
        
        // 发布CPU使用率
        std_msgs::Float32 cpu_msg;
        cpu_msg.data = static_cast<float>(status.CPU);
        cpu_usage_pub_.publish(cpu_msg);
        
        // 发布RAM使用率
        std_msgs::Float32 ram_msg;
        ram_msg.data = static_cast<float>(status.RAM);
        ram_usage_pub_.publish(ram_msg);
        
        // 计算里程计
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_odom_time_).toSec();
        last_odom_time_ = current_time;

        // 轮速（单位：rad/s）
        double v_l = status.speedL * M_PI / 30.0 * wheel_radius; // 左轮线速度
        double v_r = status.speedR * M_PI / 30.0 * wheel_radius; // 右轮线速度
        double v = (v_r + v_l) / 2.0;
        double omega = (v_r - v_l) / wheel_separation;

        // 积分更新位姿
        double delta_x = v * cos(theta_) * dt;
        double delta_y = v * sin(theta_) * dt;
        double delta_theta = omega * dt;
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        // 发布里程计消息
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        //odom.child_frame_id = "base_link";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = omega;
        odom_pub_.publish(odom);

        // 发布tf变换
        static tf::TransformBroadcaster odom_broadcaster;
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        // 请求电机速度更新
        protocol_->requestMotorSpeed();
    }
    
    void requestTimerCallback(const ros::TimerEvent&) {
        // 定期请求状态更新
        protocol_->requestStatus();
    }

    void onlineCheckTimerCallback(const ros::TimerEvent&) {
        std_msgs::Bool msg;
        msg.data = protocol_->isDeviceOnline();
        device_online_pub_.publish(msg);
    }
    
    ros::NodeHandle nh_;
    std::unique_ptr<UARTProtocolROS> protocol_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber servo_cmd_sub_;
    ros::Publisher status_pub_;
    ros::Publisher joint_pub_;
    ros::Publisher motor_speed_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher device_online_pub_;
    ros::Publisher cpu_usage_pub_;
    ros::Publisher ram_usage_pub_;
    ros::Publisher odom_pub_;
    ros::Timer status_timer_;
    ros::Timer request_timer_;
    ros::Timer online_check_timer_;

    // 里程计相关成员变量
    double x_, y_, theta_;
    ros::Time last_odom_time_;

    // 机器人参数（可根据实际情况调整）
    const double wheel_separation = 0.425;
    const double wheel_radius = 0.085;

    // 记录上一次有效的电池电压
    float last_valid_battery_voltage_ = 0.0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "uart_protocol_node");
    UARTRosNode node;
    ros::spin();
    return 0;
}