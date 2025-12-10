#ifndef UART_PROTOCOL_ROS_H
#define UART_PROTOCOL_ROS_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <vector>

// 协议配置
#define PROTOCOL_HEADER         0xAA55
#define PROTOCOL_FOOTER         0x0D0A
#define HEARTBEAT_INTERVAL_MS   300
#define HEARTBEAT_TIMEOUT_MS    1000
#define MAX_FRAME_LENGTH        256
#define RECONNECT_INTERVAL_MS   2000

// 命令字定义
enum ProtocolCmd {
    CMD_MOTION_CONTROL = 0x01,
    CMD_HEARTBEAT = 0x02,
    CMD_HEARTBEAT_ACK = 0x03,
    CMD_STATUS_FEEDBACK = 0x04,
    CMD_ERROR_REPORT = 0x05,
    CMD_REQUEST_STATUS = 0x06,
    CMD_SERVO_CONTROL = 0x07,
    CMD_REQUEST_MOTOR_SPEED = 0x08,
    CMD_MOTOR_SPEED_FEEDBACK = 0x09,
    CMD_LIGHT_CONTROL = 0x0A,        // 灯光控制
    CMD_MOTOR_ENABLE = 0x0B          // 电机启停控制
};

// 接收状态机
enum RxState {
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
};
#pragma pack(pop)

// 机器人状态信息 (根据嵌入式端定义)
#pragma pack(push, 1)
struct RobotStatus {
    bool Ready;
    bool motorEN;
    bool servoEN;
    uint8_t onlineservoid[10];   // 在线舵机ID
    uint8_t req_servo[10];       // 请求舵机角度
    uint8_t servo[10];           // 实际舵机角度
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

class UARTProtocolROS {
public:
    UARTProtocolROS(const std::string& port, int baudrate);
    ~UARTProtocolROS();
    
    // 连接管理
    bool connect();
    void disconnect();
    bool isConnected() const;
    
    // 数据发送
    void sendMotionCommand(float left_speed, float right_speed);
    void sendServoCommand(uint8_t servo_id, int16_t angle);
    void requestStatus();
    void requestMotorSpeed();
    void sendHeartbeat();
    void sendLightControl(uint8_t enable, uint8_t brightness);
    void sendMotorEnable(uint8_t enable);
    
    // 状态获取
    RobotStatus getLatestStatus() const;

    // 添加设备在线状态检查方法
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
    
    // 串口对象
    serial::Serial serial_port_;
    std::string port_;
    int baudrate_;
    
    // 线程控制
    std::atomic<bool> running_{false};
    std::thread receive_thread_;
    std::thread heartbeat_thread_;
    std::thread send_thread_;
    
    // 数据队列
    std::queue<uint8_t> rx_queue_;
    std::queue<std::vector<uint8_t>> tx_queue_;
    mutable std::mutex rx_mutex_;
    mutable std::mutex tx_mutex_;
    
    // 状态同步
    std::condition_variable cv_;
    std::mutex status_mutex_;
    RobotStatus latest_status_;
    
    // 心跳控制
    std::atomic<int> missed_beats_{0};
    std::mutex heartbeat_mutex_;
    std::condition_variable heartbeat_cv_;
    bool heartbeat_ack_received_{false};

    // 添加重连相关方法
    void tryReconnect();
    
    // 添加重连计数器
    std::atomic<int> reconnect_attempts_{0};
    static constexpr int MAX_RECONNECT_ATTEMPTS = 0;  // 0表示无限重试
    
    // 添加上次重连时间记录
    std::chrono::steady_clock::time_point last_reconnect_attempt_;
};

#endif // UART_PROTOCOL_ROS_H