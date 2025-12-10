#include "ros_uart_protocol/uart_protocol_ros.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <clocale>

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
        nh_.param("battery_voltage_min", battery_v_min_, 24.0);
        nh_.param("battery_voltage_max", battery_v_max_, 27.0);
        nh_.param("system_stats_rate_hz", system_stats_rate_hz_, 1.0);

        // 线速度/角速度限幅参数（仍可在上位机配置）
        nh_.param("max_linear_speed", max_linear_speed_, 0.8);      // m/s
        nh_.param("max_angular_speed", max_angular_speed_, 1.2);    // rad/s
        
        // 初始化协议处理器
        protocol_ = std::make_unique<UARTProtocolROS>(port, baudrate);
        
        // 连接串口
        if (!protocol_->connect()) {
            ROS_FATAL("无法连接到串口设备");
            ros::shutdown();
            return;
        }
        
        // 创建订阅器
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10, &UARTRosNode::cmdVelCallback, this);
        servo_cmd_sub_ = nh_.subscribe("servo_cmd", 10, &UARTRosNode::servoCmdCallback, this);
        light_cmd_sub_ = nh_.subscribe("light_cmd", 10, &UARTRosNode::lightCmdCallback, this);
        motor_enable_sub_ = nh_.subscribe("motor_enable", 10, &UARTRosNode::motorEnableCallback, this);
        
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
        // 新增：电池电量（百分比）与本机系统资源占用话题
        battery_percent_pub_ = nh_.advertise<std_msgs::Float32>("battery_percentage", 10);
        system_cpu_pub_ = nh_.advertise<std_msgs::Float32>("system_cpu_usage", 10);
        system_ram_pub_ = nh_.advertise<std_msgs::Float32>("system_ram_usage", 10);
        // 新增：灯光状态发布器
        light_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("light_status", 10);
        
        // 创建定时器
        status_timer_ = nh_.createTimer(ros::Duration(0.02), &UARTRosNode::statusTimerCallback, this); //30Hz
        request_timer_ = nh_.createTimer(ros::Duration(1.0), &UARTRosNode::requestTimerCallback, this); //1Hz
        online_check_timer_ = nh_.createTimer(ros::Duration(0.1), &UARTRosNode::onlineCheckTimerCallback, this); //10Hz
        // 新增：系统CPU/RAM统计定时器
        double stats_period = (system_stats_rate_hz_ > 0.0) ? (1.0 / system_stats_rate_hz_) : 1.0;
        system_stats_timer_ = nh_.createTimer(ros::Duration(stats_period), &UARTRosNode::systemStatsTimerCallback, this);
        
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

        // 先对线速度和角速度进行限幅
        if (max_linear_speed_ > 0.0f) {
            linear = std::max(-max_linear_speed_, std::min(max_linear_speed_, static_cast<double>(linear)));
        }
        if (max_angular_speed_ > 0.0f) {
            angular = std::max(-max_angular_speed_, std::min(max_angular_speed_, static_cast<double>(angular)));
        }
        
        // 示例转换公式 (需要根据实际机器人参数调整)
        float wheel_separation = 0.425; // 轮间距
        float wheel_radius = 0.085;    // 轮半径
        float left_speed = 10*(linear - angular * wheel_separation / 2.0) / wheel_radius;
        float right_speed = 10*(linear + angular * wheel_separation / 2.0) / wheel_radius;

        // 再对左右轮RPM进行限幅：
        // 1）优先使用 MCU 上报的 speedLimit（见 RobotStatus::speedLimit，单位 RPM）
        // 2）如果 MCU 未设置（speedLimit==0），则不主动限速，由底盘自行限幅
        auto status = protocol_->getLatestStatus();
        double rpm_limit = static_cast<double>(status.speedLimit);
        if (rpm_limit > 0.0) {
            left_speed  = std::max(-rpm_limit, std::min(rpm_limit, static_cast<double>(left_speed)));
            right_speed = std::max(-rpm_limit, std::min(rpm_limit, static_cast<double>(right_speed)));
        }
        
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
    
    void lightCmdCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        // msg->data[0]: 1=开, 0=关, 0xFF=切换
        // msg->data[1]: 亮度 0-100
        if (msg->data.size() >= 2) {
            uint8_t enable = static_cast<uint8_t>(msg->data[0]);
            uint8_t brightness = static_cast<uint8_t>(msg->data[1]);
            protocol_->sendLightControl(enable, brightness);
        }
    }

    void motorEnableCallback(const std_msgs::UInt8::ConstPtr& msg) {
        // msg->data: 0=禁用, 1=启用, 0xFF=切换
        protocol_->sendMotorEnable(msg->data);
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

        // 新增：发布电池电量百分比（线性映射：battery_voltage_min -> 0%，battery_voltage_max -> 100%）
        if (battery_v_max_ < battery_v_min_) std::swap(battery_v_min_, battery_v_max_);
        std_msgs::Float32 battery_pct_msg;
        if (last_valid_battery_voltage_ > 0.01) {
            double pct = (last_valid_battery_voltage_ - battery_v_min_) / std::max(1e-6, (battery_v_max_ - battery_v_min_));
            pct = std::max(0.0, std::min(1.0, pct));
            battery_pct_msg.data = static_cast<float>(pct * 100.0);
        } else {
            battery_pct_msg.data = 0.0f;
        }
        battery_percent_pub_.publish(battery_pct_msg);
        
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

        // 发布灯光状态
        std_msgs::Float32MultiArray light_msg;
        light_msg.data = {
            static_cast<float>(status.lightEnabled ? 1.0f : 0.0f),
            static_cast<float>(status.lightBrightness)
        };
        light_pub_.publish(light_msg);
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

    // 新增：系统资源统计发布
    void systemStatsTimerCallback(const ros::TimerEvent&) {
        double cpu = readSystemCPUPercent();
        double ram = readSystemRAMPercent();

        if (std::isfinite(cpu)) {
            std_msgs::Float32 msg; msg.data = static_cast<float>(cpu);
            system_cpu_pub_.publish(msg);
        }
        if (std::isfinite(ram)) {
            std_msgs::Float32 msg; msg.data = static_cast<float>(ram);
            system_ram_pub_.publish(msg);
        }
    }

    // 从 /proc/stat 计算CPU占用率（百分比）。首次调用返回NaN以建立基线。
    double readSystemCPUPercent() {
        std::ifstream fs("/proc/stat");
        if (!fs.is_open()) return std::numeric_limits<double>::quiet_NaN();
        std::string line;
        std::getline(fs, line);
        fs.close();
        if (line.rfind("cpu ", 0) != 0) return std::numeric_limits<double>::quiet_NaN();

        std::istringstream iss(line);
        std::string cpu_label; // "cpu"
        uint64_t user=0, nice=0, system=0, idle=0, iowait=0, irq=0, softirq=0, steal=0, guest=0, guest_nice=0;
        iss >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal >> guest >> guest_nice;

        uint64_t idle_all = idle + iowait;
        uint64_t non_idle = user + nice + system + irq + softirq + steal;
        uint64_t total = idle_all + non_idle;

        if (!has_prev_cpu_) {
            prev_total_jiffies_ = static_cast<double>(total);
            prev_idle_jiffies_ = static_cast<double>(idle_all);
            has_prev_cpu_ = true;
            return std::numeric_limits<double>::quiet_NaN();
        }

        double totald = static_cast<double>(total) - prev_total_jiffies_;
        double idled  = static_cast<double>(idle_all) - prev_idle_jiffies_;
        prev_total_jiffies_ = static_cast<double>(total);
        prev_idle_jiffies_  = static_cast<double>(idle_all);

        if (totald <= 0.0) return std::numeric_limits<double>::quiet_NaN();
        double cpu_pct = (totald - idled) / totald * 100.0;
        if (cpu_pct < 0.0) cpu_pct = 0.0;
        if (cpu_pct > 100.0) cpu_pct = 100.0;
        return cpu_pct;
    }

    // 从 /proc/meminfo 计算RAM占用率（百分比）
    double readSystemRAMPercent() {
        std::ifstream fs("/proc/meminfo");
        if (!fs.is_open()) return std::numeric_limits<double>::quiet_NaN();
        std::string line;
        uint64_t mem_total = 0, mem_available = 0;
        while (std::getline(fs, line)) {
            if (line.rfind("MemTotal:", 0) == 0) {
                std::istringstream iss(line.substr(9));
                iss >> mem_total; // kB
            } else if (line.rfind("MemAvailable:", 0) == 0) {
                std::istringstream iss(line.substr(13));
                iss >> mem_available; // kB
            }
            if (mem_total && mem_available) break;
        }
        fs.close();
        if (mem_total == 0) return std::numeric_limits<double>::quiet_NaN();
        double used = static_cast<double>(mem_total - mem_available);
        double pct = used / static_cast<double>(mem_total) * 100.0;
        if (pct < 0.0) pct = 0.0;
        if (pct > 100.0) pct = 100.0;
        return pct;
    }
    
    ros::NodeHandle nh_;
    std::unique_ptr<UARTProtocolROS> protocol_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber servo_cmd_sub_;
    ros::Subscriber light_cmd_sub_;
    ros::Subscriber motor_enable_sub_;
    ros::Publisher status_pub_;
    ros::Publisher joint_pub_;
    ros::Publisher motor_speed_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher device_online_pub_;
    ros::Publisher cpu_usage_pub_;
    ros::Publisher ram_usage_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher battery_percent_pub_;
    ros::Publisher system_cpu_pub_;
    ros::Publisher system_ram_pub_;
    ros::Publisher light_pub_;
    ros::Timer status_timer_;
    ros::Timer request_timer_;
    ros::Timer online_check_timer_;
    ros::Timer system_stats_timer_;

    // 里程计相关成员变量
    double x_, y_, theta_;
    ros::Time last_odom_time_;

    // 机器人参数（可根据实际情况调整）
    const double wheel_separation = 0.425;
    const double wheel_radius = 0.085;

    // 限速参数（线速度/角速度由上位机约束，轮速 RPM 由底盘 MCU 的 speedLimit 字段约束）
    double max_linear_speed_ = 0.8;   // m/s
    double max_angular_speed_ = 1.2;  // rad/s

    // 记录上一次有效的电池电压
    float last_valid_battery_voltage_ = 0.0;

    // 电池百分比映射参数
    double battery_v_min_ = 24.0;
    double battery_v_max_ = 27.0;

    // 系统资源统计参数与状态
    double system_stats_rate_hz_ = 1.0;
    bool has_prev_cpu_ = false;
    double prev_total_jiffies_ = 0.0;
    double prev_idle_jiffies_ = 0.0;
};

int main(int argc, char** argv) {
    // 设置本地化，确保中文日志可正确输出
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "uart_protocol_node");
    UARTRosNode node;
    ros::spin();
    return 0;
}