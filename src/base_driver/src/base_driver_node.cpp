#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "base_driver/uart_protocol_ros2.h"
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>

class UARTRosNode : public rclcpp::Node {
public:
    UARTRosNode()
        : rclcpp::Node("uart_protocol_node"),
          x_(0.0), y_(0.0), theta_(0.0),
          last_odom_time_(this->now()),
          wheel_separation(0.425),
          wheel_radius(0.085),
          max_linear_speed_(0.8),
          max_angular_speed_(1.2),
          last_valid_battery_voltage_(0.0),
          battery_v_min_(24.0),
          battery_v_max_(27.0),
          system_stats_rate_hz_(1.0),
          has_prev_cpu_(false),
          prev_total_jiffies_(0.0),
          prev_idle_jiffies_(0.0) {
        
        // Declare and get parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 115200);
        this->declare_parameter<double>("battery_voltage_min", 24.0);
        this->declare_parameter<double>("battery_voltage_max", 27.0);
        this->declare_parameter<double>("system_stats_rate_hz", 1.0);
        this->declare_parameter<double>("max_linear_speed", 0.8);
        this->declare_parameter<double>("max_angular_speed", 1.2);
        
        std::string port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        battery_v_min_ = this->get_parameter("battery_voltage_min").as_double();
        battery_v_max_ = this->get_parameter("battery_voltage_max").as_double();
        system_stats_rate_hz_ = this->get_parameter("system_stats_rate_hz").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        
        // Initialize protocol handler
        protocol_ = std::make_unique<UARTProtocolROS2>(port, baudrate, this->get_logger());
        
        // Connect to serial port
        if (!protocol_->connect()) {
            RCLCPP_WARN(this->get_logger(), 
                "Failed to connect to serial device '%s'. "
                "Available devices: /dev/ttyS* (check with 'ls /dev/tty*'). "
                "Please connect a USB serial device or specify correct port with -p serial_port:=/dev/ttyXX", 
                port.c_str());
            // Don't shutdown - keep running and wait for device
        }
        
        // Create subscribers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&UARTRosNode::cmdVelCallback, this, std::placeholders::_1));
        servo_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "servo_cmd", 10, std::bind(&UARTRosNode::servoCmdCallback, this, std::placeholders::_1));
        light_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "light_cmd", 10, std::bind(&UARTRosNode::lightCmdCallback, this, std::placeholders::_1));
        motor_enable_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
            "motor_enable", 10, std::bind(&UARTRosNode::motorEnableCallback, this, std::placeholders::_1));
        
        // Create publishers
        status_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("robot_status", 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        motor_speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("motor_speed", 10);
        battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_voltage", 10);
        device_online_pub_ = this->create_publisher<std_msgs::msg::Bool>("device_online", 10);
        cpu_usage_pub_ = this->create_publisher<std_msgs::msg::Float32>("cpu_usage", 10);
        ram_usage_pub_ = this->create_publisher<std_msgs::msg::Float32>("ram_usage", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        battery_percent_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery_percentage", 10);
        system_cpu_pub_ = this->create_publisher<std_msgs::msg::Float32>("system_cpu_usage", 10);
        system_ram_pub_ = this->create_publisher<std_msgs::msg::Float32>("system_ram_usage", 10);
        light_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("light_status", 10);
        
        // Create timers
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&UARTRosNode::statusTimerCallback, this));
        request_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&UARTRosNode::requestTimerCallback, this));
        online_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&UARTRosNode::onlineCheckTimerCallback, this));
        
        double stats_period = (system_stats_rate_hz_ > 0.0) ? (1.0 / system_stats_rate_hz_) : 1.0;
        system_stats_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(stats_period),
            std::bind(&UARTRosNode::systemStatsTimerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "UART ROS2 node initialized");
    }
    
    ~UARTRosNode() {
        protocol_->disconnect();
    }
    
private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        float linear = msg->linear.x;
        float angular = msg->angular.z;

        if (max_linear_speed_ > 0.0f) {
            linear = std::max(-max_linear_speed_, std::min(max_linear_speed_, static_cast<double>(linear)));
        }
        if (max_angular_speed_ > 0.0f) {
            angular = std::max(-max_angular_speed_, std::min(max_angular_speed_, static_cast<double>(angular)));
        }
        
        float left_speed = 10 * (linear - angular * wheel_separation / 2.0) / wheel_radius;
        float right_speed = 10 * (linear + angular * wheel_separation / 2.0) / wheel_radius;

        auto status = protocol_->getLatestStatus();
        double rpm_limit = static_cast<double>(status.speedLimit);
        if (rpm_limit > 0.0) {
            left_speed = std::max(-rpm_limit, std::min(rpm_limit, static_cast<double>(left_speed)));
            right_speed = std::max(-rpm_limit, std::min(rpm_limit, static_cast<double>(right_speed)));
        }
        
        protocol_->sendMotionCommand(left_speed, right_speed);
    }
    
    void servoCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i].find("servo") != std::string::npos) {
                int servo_id = std::stoi(msg->name[i].substr(5));
                float angle_rad = msg->position[i];
                int16_t angle_deg = static_cast<int16_t>(angle_rad * 180.0 / M_PI);
                protocol_->sendServoCommand(servo_id, angle_deg);
            }
        }
    }
    
    void lightCmdCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            uint8_t enable = static_cast<uint8_t>(msg->data[0]);
            uint8_t brightness = static_cast<uint8_t>(msg->data[1]);
            protocol_->sendLightControl(enable, brightness);
        }
    }

    void motorEnableCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
        protocol_->sendMotorEnable(msg->data);
    }
    
    void statusTimerCallback() {
        auto status = protocol_->getLatestStatus();
        
        // Publish robot status
        auto status_msg = std_msgs::msg::Float32MultiArray();
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
        status_pub_->publish(status_msg);
        
        // Publish battery voltage
        auto battery_msg = std_msgs::msg::Float32();
        if (status.batteryVoltage > 0.01) {
            last_valid_battery_voltage_ = status.batteryVoltage;
        }
        battery_msg.data = last_valid_battery_voltage_;
        battery_pub_->publish(battery_msg);

        // Publish battery percentage
        if (battery_v_max_ < battery_v_min_) std::swap(battery_v_min_, battery_v_max_);
        auto battery_pct_msg = std_msgs::msg::Float32();
        if (last_valid_battery_voltage_ > 0.01) {
            double pct = (last_valid_battery_voltage_ - battery_v_min_) / std::max(1e-6, (battery_v_max_ - battery_v_min_));
            pct = std::max(0.0, std::min(1.0, pct));
            battery_pct_msg.data = static_cast<float>(pct * 100.0);
        } else {
            battery_pct_msg.data = 0.0f;
        }
        battery_percent_pub_->publish(battery_pct_msg);
        
        // Publish joint states
        auto joint_msg = sensor_msgs::msg::JointState();
        joint_msg.header.stamp = this->now();
        
        joint_msg.name.push_back("wheel_left_joint");
        joint_msg.name.push_back("wheel_right_joint");
        joint_msg.position.push_back(0.0);
        joint_msg.position.push_back(0.0);
        joint_msg.velocity.push_back(status.speedL * M_PI / 30.0);
        joint_msg.velocity.push_back(status.speedR * M_PI / 30.0);
        
        for (int i = 0; i < 10; ++i) {
            if (status.onlineservoid[i] != 0) {
                std::string joint_name = "servo" + std::to_string(i);
                joint_msg.name.push_back(joint_name);
                joint_msg.position.push_back(status.servo[i] * M_PI / 180.0);
                joint_msg.velocity.push_back(0.0);
            }
        }
        joint_pub_->publish(joint_msg);
        
        // Publish CPU and RAM usage
        auto cpu_msg = std_msgs::msg::Float32();
        cpu_msg.data = static_cast<float>(status.CPU);
        cpu_usage_pub_->publish(cpu_msg);
        
        auto ram_msg = std_msgs::msg::Float32();
        ram_msg.data = static_cast<float>(status.RAM);
        ram_usage_pub_->publish(ram_msg);
        
        // Calculate and publish odometry
        auto current_time = this->now();
        double dt = (current_time - last_odom_time_).seconds();
        last_odom_time_ = current_time;

        double v_l = status.speedL * M_PI / 30.0 * wheel_radius;
        double v_r = status.speedR * M_PI / 30.0 * wheel_radius;
        double v = (v_r + v_l) / 2.0;
        double omega = (v_r - v_l) / wheel_separation;

        double delta_x = v * cos(theta_) * dt;
        double delta_y = v * sin(theta_) * dt;
        double delta_theta = omega * dt;
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        
        odom.twist.twist.linear.x = v;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = omega;
        odom_pub_->publish(odom);

        // Broadcast TF
        static auto tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();
        tf_broadcaster->sendTransform(odom_trans);

        protocol_->requestMotorSpeed();

        // Publish light status
        auto light_msg = std_msgs::msg::Float32MultiArray();
        light_msg.data = {
            static_cast<float>(status.lightEnabled ? 1.0f : 0.0f),
            static_cast<float>(status.lightBrightness)
        };
        light_pub_->publish(light_msg);
    }
    
    void requestTimerCallback() {
        protocol_->requestStatus();
    }

    void onlineCheckTimerCallback() {
        auto msg = std_msgs::msg::Bool();
        msg.data = protocol_->isDeviceOnline();
        device_online_pub_->publish(msg);
    }

    void systemStatsTimerCallback() {
        double cpu = readSystemCPUPercent();
        double ram = readSystemRAMPercent();

        if (std::isfinite(cpu)) {
            auto msg = std_msgs::msg::Float32();
            msg.data = static_cast<float>(cpu);
            system_cpu_pub_->publish(msg);
        }
        if (std::isfinite(ram)) {
            auto msg = std_msgs::msg::Float32();
            msg.data = static_cast<float>(ram);
            system_ram_pub_->publish(msg);
        }
    }

    double readSystemCPUPercent() {
        std::ifstream fs("/proc/stat");
        if (!fs.is_open()) return std::numeric_limits<double>::quiet_NaN();
        std::string line;
        std::getline(fs, line);
        fs.close();
        if (line.rfind("cpu ", 0) != 0) return std::numeric_limits<double>::quiet_NaN();

        std::istringstream iss(line);
        std::string cpu_label;
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
        double idled = static_cast<double>(idle_all) - prev_idle_jiffies_;
        prev_total_jiffies_ = static_cast<double>(total);
        prev_idle_jiffies_ = static_cast<double>(idle_all);

        if (totald <= 0.0) return std::numeric_limits<double>::quiet_NaN();
        double cpu_pct = (totald - idled) / totald * 100.0;
        if (cpu_pct < 0.0) cpu_pct = 0.0;
        if (cpu_pct > 100.0) cpu_pct = 100.0;
        return cpu_pct;
    }

    double readSystemRAMPercent() {
        std::ifstream fs("/proc/meminfo");
        if (!fs.is_open()) return std::numeric_limits<double>::quiet_NaN();
        std::string line;
        uint64_t mem_total = 0, mem_available = 0;
        while (std::getline(fs, line)) {
            if (line.rfind("MemTotal:", 0) == 0) {
                std::istringstream iss(line.substr(9));
                iss >> mem_total;
            } else if (line.rfind("MemAvailable:", 0) == 0) {
                std::istringstream iss(line.substr(13));
                iss >> mem_available;
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
    
    std::unique_ptr<UARTProtocolROS2> protocol_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr servo_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr light_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr motor_enable_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr device_online_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cpu_usage_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ram_usage_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_percent_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr system_cpu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr system_ram_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr light_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr request_timer_;
    rclcpp::TimerBase::SharedPtr online_check_timer_;
    rclcpp::TimerBase::SharedPtr system_stats_timer_;

    double x_, y_, theta_;
    rclcpp::Time last_odom_time_;

    const double wheel_separation;
    const double wheel_radius;

    double max_linear_speed_;
    double max_angular_speed_;

    float last_valid_battery_voltage_;
    double battery_v_min_;
    double battery_v_max_;

    double system_stats_rate_hz_;
    bool has_prev_cpu_;
    double prev_total_jiffies_;
    double prev_idle_jiffies_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UARTRosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
