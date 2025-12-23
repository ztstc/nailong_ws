#ifndef UART_PROTOCOL_ROS2_H
#define UART_PROTOCOL_ROS2_H

#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <vector>
#include <memory>

using namespace LibSerial;

// 协议配置
constexpr uint16_t PROTOCOL_HEADER = 0xAA55;
constexpr uint16_t PROTOCOL_FOOTER = 0x0D0A;
constexpr int HEARTBEAT_INTERVAL_MS = 300;
constexpr int HEARTBEAT_TIMEOUT_MS = 1000;
constexpr int MAX_FRAME_LENGTH = 256;
constexpr int RECONNECT_INTERVAL_MS = 2000;
constexpr int MAX_CONSECUTIVE_ERRORS = 5;

// 命令字定义
enum class ProtocolCmd : uint8_t {
    CMD_MOTION_CONTROL = 0x01,
    CMD_HEARTBEAT = 0x02,
    CMD_HEARTBEAT_ACK = 0x03,
    CMD_STATUS_FEEDBACK = 0x04,
    CMD_ERROR_REPORT = 0x05,
    CMD_REQUEST_STATUS = 0x06,
    CMD_SERVO_CONTROL = 0x07,
    CMD_REQUEST_MOTOR_SPEED = 0x08,
    CMD_MOTOR_SPEED_FEEDBACK = 0x09,
    CMD_LIGHT_CONTROL = 0x0A,
    CMD_MOTOR_ENABLE = 0x0B
};

// 接收状态机
enum class RxState {
    FRAME_HEADER1,
    FRAME_HEADER2,
    FRAME_LENGTH,
    FRAME_CMD,
    FRAME_DATA,
    FRAME_CRC,
    FRAME_TAIL1,
    FRAME_TAIL2
};

// 运动控制结构
#pragma pack(push, 1)
struct MotionCmd {
    float target_speedL;
    float target_speedR;
};
#pragma pack(pop)

// 舵机控制结构
#pragma pack(push, 1)
struct ServoCmd {
    uint8_t servo_id;
    int16_t angle;
    uint16_t time_ms;
};
#pragma pack(pop)

// 机器人状态信息 (根据嵌入式端定义)
#pragma pack(push, 1)
struct RobotStatus {
    bool Ready;
    bool motorEN;
    bool servoEN;
    uint8_t onlineservoid[10];   // 在线舵机ID
    uint16_t req_servo[10];      // 请求舵机角度 (0-270°)
    uint16_t servo[10];          // 实际舵机角度 (0-270°)
    uint8_t speedLimit;          // 最大速度(RPM)
    float req_speedL;            // 请求左轮速度(RPM)
    float req_speedR;            // 请求右轮速度(RPM)
    float speedL;                // 实际左轮速度(RPM)
    float speedR;                // 实际右轮速度(RPM)
    float batteryVoltage;        // 电池电压(V)
    float motorCurrentL;         // 左电机电流(A)
    float motorCurrentR;         // 右电机电流(A)
    float motorCurrent;          // 总电流(A)
    uint8_t CPU;                 // MCU CPU使用率(%)
    uint8_t RAM;                 // MCU RAM使用率(%)
    uint8_t error_flags;         // 错误标志
    bool lightEnabled;           // 灯光启用状态
    uint8_t lightBrightness;     // 灯光亮度(0-100%)
};
#pragma pack(pop)

class UARTProtocolROS2 {
public:
    UARTProtocolROS2(const std::string& port, int baudrate,
                     rclcpp::Logger logger = rclcpp::get_logger("uart_protocol"));
    ~UARTProtocolROS2();
    
    // 连接管理
    bool connect();
    void disconnect();
    bool isConnected() const;
    
    // 数据发送
    void sendMotionCommand(float left_speed, float right_speed);
    void sendServoCommand(uint8_t servo_id, int16_t angle);
    void sendServoCommand(uint8_t servo_id, int16_t angle, uint16_t time_ms);
    void requestStatus();
    void requestMotorSpeed();
    void sendHeartbeat();
    void sendLightControl(uint8_t enable, uint8_t brightness);
    void sendMotorEnable(uint8_t enable);
    
    // 状态获取
    RobotStatus getLatestStatus() const;
    bool isDeviceOnline() const;
    
private:
    // 线程函数
    void receiveThreadFunc();
    void heartbeatThreadFunc();
    void sendThreadFunc();
    
    // 协议处理
    void processFrame(uint8_t cmd, const std::vector<uint8_t>& data);
    void sendProtocolFrame(ProtocolCmd cmd, const void* data, uint8_t data_len);
    uint8_t crc8_update(uint8_t crc, uint8_t data);
    
    // 错误处理
    void handleError(const std::string& error_msg);
    void resetConnection();
    
    // 串口对象
    std::unique_ptr<SerialPort> serial_port_;
    std::string port_;
    int baudrate_;
    rclcpp::Logger logger_;
    
    // 线程控制
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    std::thread heartbeat_thread_;
    std::thread send_thread_;
    
    // 数据队列
    std::queue<std::vector<uint8_t>> tx_queue_;
    mutable std::mutex tx_mutex_;
    
    // 状态同步
    std::condition_variable cv_;
    mutable std::mutex status_mutex_;
    RobotStatus latest_status_{};
    
    // 心跳控制
    std::atomic<int> missed_beats_{0};
    std::mutex heartbeat_mutex_;
    std::condition_variable heartbeat_cv_;
    bool heartbeat_ack_received_{false};
    
    // 错误计数
    std::atomic<int> consecutive_errors_{0};
    
    // 重连间隔
    std::chrono::steady_clock::time_point last_reconnect_attempt_;
};

#endif // UART_PROTOCOL_ROS2_H