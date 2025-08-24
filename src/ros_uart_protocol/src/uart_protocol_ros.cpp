#include "ros_uart_protocol/uart_protocol_ros.h"
#include <ros/console.h>
#include <chrono>
#include <cstring>

using namespace std::chrono;

UARTProtocolROS::UARTProtocolROS(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate) {
    // 初始化串口对象
    serial_port_.setPort(port);
    serial_port_.setBaudrate(baudrate);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serial_port_.setTimeout(timeout);
}

UARTProtocolROS::~UARTProtocolROS() {
    disconnect();
}

bool UARTProtocolROS::connect() {
    try {
        // 如果已经连接，先断开
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
        
        serial_port_.open();
        if (!serial_port_.isOpen()) {
            ROS_ERROR("Failed to open serial port: %s", port_.c_str());
            return false;
        }
        
        running_ = true;
        
        // 启动线程
        receive_thread_ = std::thread(&UARTProtocolROS::receiveThreadFunc, this);
        heartbeat_thread_ = std::thread(&UARTProtocolROS::heartbeatThreadFunc, this);
        send_thread_ = std::thread(&UARTProtocolROS::sendThreadFunc, this);
        
        ROS_INFO("Connected to serial port: %s", port_.c_str());
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("Serial connection error: %s", e.what());
        running_ = false;
        return false;
    }
}

void UARTProtocolROS::disconnect() {
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        running_ = false;
    }
    
    // 先通知所有线程退出
    cv_.notify_all();
    heartbeat_cv_.notify_all();
    
    // 等待所有线程结束
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
    if (send_thread_.joinable()) {
        send_thread_.join();
    }
    
    // 最后关闭串口
    try {
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error closing serial port: %s", e.what());
    }
    
    ROS_INFO("Disconnected from serial port: %s", port_.c_str());
}

bool UARTProtocolROS::isConnected() const {
    return serial_port_.isOpen() && running_;
}

// CRC8计算 (多项式0x07)
uint8_t UARTProtocolROS::crc8_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0x07;
        } else {
            crc = (crc << 1);
        }
        crc &= 0xFF;  // 确保结果为8位
    }
    return crc;
}

void UARTProtocolROS::receiveThreadFunc() {
    RxState rx_state = FRAME_HEADER1;
    std::vector<uint8_t> rx_buffer;
    uint8_t expected_length = 0;
    uint8_t current_cmd = 0;
    uint8_t calculated_crc = 0;

    uint8_t data_bytes_received = 0;

    while (running_) {
        if (!isConnected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        try {
            // 检查可用数据
            size_t bytes_avail = serial_port_.available();
            if (bytes_avail == 0) {
                std::this_thread::sleep_for(milliseconds(5));
                continue;
            }

            // 读取数据
            std::vector<uint8_t> data(bytes_avail);
            size_t bytes_read = serial_port_.read(data.data(), bytes_avail);
            
            // 处理接收到的数据
            for (size_t i = 0; i < bytes_read; ++i) {
                uint8_t rx_byte = data[i];
            
                switch (rx_state) {
                    case FRAME_HEADER1:
                        calculated_crc = 0;  // 重置CRC
                        if (rx_byte == 0xAA) {
                            rx_state = FRAME_HEADER2;
                            rx_buffer.clear();
                            rx_buffer.push_back(rx_byte);
                        }
                        break;
                        
                    case FRAME_HEADER2:
                        if (rx_byte == 0x55) {
                            rx_state = FRAME_LENGTH;
                            rx_buffer.push_back(rx_byte);
                        } else {
                            rx_state = FRAME_HEADER1;
                        }
                        break;
                        
                    case FRAME_LENGTH:
                        expected_length = rx_byte;
                        calculated_crc = crc8_update(0, rx_byte);  // 从长度开始计算CRC
                        rx_buffer.push_back(rx_byte);
                        
                        if (expected_length <= MAX_FRAME_LENGTH - 4) {
                            rx_state = FRAME_CMD;
                        } else {
                            ROS_WARN("Invalid frame length: %d", expected_length);
                            rx_state = FRAME_HEADER1;
                        }
                        break;
                        
                    case FRAME_CMD:
                        current_cmd = rx_byte;
                        calculated_crc = crc8_update(calculated_crc, rx_byte);
                        rx_buffer.push_back(rx_byte);
                        
                        if (expected_length > 0) {
                            rx_state = FRAME_DATA;
                        } else {
                            rx_state = FRAME_CRC;
                        }
                        break;
                        
                    case FRAME_DATA:
                        rx_buffer.push_back(rx_byte);
                        calculated_crc = crc8_update(calculated_crc, rx_byte);
                        data_bytes_received++;  // 关键修复：独立计数
                    
                        // 修复判断条件：仅检查数据域长度
                        if (data_bytes_received >= expected_length) {
                            rx_state = FRAME_CRC;
                        }
                        break;
                        // if (rx_buffer.size() >= expected_length) {
                        //     rx_state = FRAME_CRC;
                        // }
                        // break;
                        
                    case FRAME_CRC:
                        rx_buffer.push_back(rx_byte);
                        if (rx_byte == calculated_crc) {
                            rx_state = FRAME_TAIL1;
                        } else {
                            ROS_WARN("CRC mismatch: expected 0x%02X, got 0x%02X, cmd: 0x%02X, len: %d", 
                                     calculated_crc, rx_byte, current_cmd, expected_length);
                            rx_state = FRAME_HEADER1;
                        }
                        break;
                        
                    case FRAME_TAIL1:
                        if (rx_byte == 0x0D) {
                            rx_state = FRAME_TAIL2;
                        } else {
                            rx_state = FRAME_HEADER1;
                        }
                        break;
                        
                    case FRAME_TAIL2:
                        if (rx_byte == 0x0A) {
                            rx_buffer.push_back(rx_byte);
                            // 关键修复：只传递数据域部分 (去掉帧头4B+CRC1B+帧尾2B)
                            if (rx_buffer.size() >= 4 + expected_length) {
                                std::vector<uint8_t> payload(
                                    rx_buffer.begin() + 4, 
                                    rx_buffer.begin() + 4 + expected_length
                                );
                                processFrame(current_cmd, payload);
                            }
                        }
                        rx_state = FRAME_HEADER1;
                        data_bytes_received = 0;  // 重置计数器
                        break;
                }
            }
        }
        catch (const serial::IOException& e) {
            ROS_ERROR("Serial IO error: %s", e.what());
            disconnect();  // 触发重连机制
        }
        catch (const serial::SerialException& e) {
            ROS_ERROR("Serial exception: %s", e.what());
            disconnect();  // 触发重连机制
        }
        catch (const std::exception& e) {
            ROS_ERROR("Unexpected error in receive thread: %s", e.what());
            disconnect();  // 触发重连机制
        }
    }
}

void UARTProtocolROS::processFrame(uint8_t cmd, const std::vector<uint8_t>& data) {
    switch (cmd) {
        case CMD_HEARTBEAT_ACK:
            {
                std::lock_guard<std::mutex> lock(heartbeat_mutex_);
                heartbeat_ack_received_ = true;
                heartbeat_cv_.notify_one();
            }
            break;
            
        case CMD_STATUS_FEEDBACK:
            if (data.size() == sizeof(RobotStatus)) {
                RobotStatus status;
                memcpy(&status, data.data(), sizeof(RobotStatus));
                
                std::lock_guard<std::mutex> lock(status_mutex_);
                latest_status_ = status;
            }
            break;
            
        case CMD_MOTOR_SPEED_FEEDBACK:
            if (data.size() == 8) {
                float speedL, speedR;
                memcpy(&speedL, data.data(), sizeof(float));
                memcpy(&speedR, data.data() + sizeof(float), sizeof(float));
                
                std::lock_guard<std::mutex> lock(status_mutex_);
                latest_status_.speedL = speedL;
                latest_status_.speedR = speedR;
            }
            break;
            
        case CMD_ERROR_REPORT:
            if (!data.empty()) {
                ROS_ERROR("Device reported error: 0x%02X", data[0]);
            }
            break;
            
        default:
            ROS_DEBUG("Received unknown command: 0x%02X", cmd);
            break;
    }
}

void UARTProtocolROS::heartbeatThreadFunc() {
    while (running_) {
        if (!isConnected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // 发送心跳
        try {
            sendHeartbeat();
        } catch (const std::exception& e) {
            ROS_ERROR("Error sending heartbeat: %s", e.what());
            continue;
        }
        
        {
            std::unique_lock<std::mutex> lock(heartbeat_mutex_);
            heartbeat_ack_received_ = false;
            
            bool ack_received = heartbeat_cv_.wait_for(lock, 
                std::chrono::milliseconds(HEARTBEAT_TIMEOUT_MS),
                [this] { return heartbeat_ack_received_ || !running_; });
                
            if (!ack_received && running_) {
                int current_missed = ++missed_beats_;
                if (current_missed >= 3) {
                    ROS_WARN("Heartbeat timeout (%d missed beats), initiating reconnect...", current_missed);
                    try {
                        if (serial_port_.isOpen()) {
                            serial_port_.close();
                        }
                    } catch (...) {}
                    continue;
                }
            } else {
                missed_beats_ = 0;
            }
        }
        
        if (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_INTERVAL_MS));
        }
    }
}

bool UARTProtocolROS::isDeviceOnline() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(heartbeat_mutex_));
    // 如果丢失心跳次数小于3次，则认为设备在线
    return missed_beats_ < 3;
}

void UARTProtocolROS::tryReconnect() {
    if (!running_) return;
    
    static auto last_attempt = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    
    // 限制重连频率
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_attempt).count() < RECONNECT_INTERVAL_MS) {
        return;
    }
    
    last_attempt = now;
    
    // 确保串口完全关闭
    try {
        if (serial_port_.isOpen()) {
            serial_port_.close();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (...) {}
    
    ROS_INFO("Attempting to reconnect to %s...", port_.c_str());
    
    // 尝试重新连接
    try {
        serial_port_.open();
        if (serial_port_.isOpen()) {
            ROS_INFO("Reconnection successful");
            missed_beats_ = 0;
            
            // 重新初始化线程
            if (!receive_thread_.joinable()) {
                receive_thread_ = std::thread(&UARTProtocolROS::receiveThreadFunc, this);
            }
            if (!heartbeat_thread_.joinable()) {
                heartbeat_thread_ = std::thread(&UARTProtocolROS::heartbeatThreadFunc, this);
            }
            if (!send_thread_.joinable()) {
                send_thread_ = std::thread(&UARTProtocolROS::sendThreadFunc, this);
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Reconnection failed: %s", e.what());
    }
}

void UARTProtocolROS::sendThreadFunc() {
    while (running_) {
        std::unique_lock<std::mutex> lock(tx_mutex_);
        bool has_data = cv_.wait_for(lock, 
            std::chrono::milliseconds(100),
            [this] { return !tx_queue_.empty() || !running_; });
            
        if (!running_) break;
        if (!has_data) continue;
        
        while (!tx_queue_.empty() && running_) {
            auto data = std::move(tx_queue_.front());
            tx_queue_.pop();
            lock.unlock();
            
            try {
                if (serial_port_.isOpen()) {
                    serial_port_.write(data);
                    serial_port_.flush();
                }
            } catch (const std::exception& e) {
                ROS_ERROR("Serial write error: %s", e.what());
                // 标记需要重连
                try {
                    if (serial_port_.isOpen()) {
                        serial_port_.close();
                    }
                } catch (...) {}
            }
            
            lock.lock();
        }
    }
}

void UARTProtocolROS::sendProtocolFrame(ProtocolCmd cmd, const void* data, uint8_t data_len) {
    // 计算帧长度 (header + len + cmd + data + crc + footer)
    uint16_t frame_len = 2 + 1 + 1 + data_len + 1 + 2;
    std::vector<uint8_t> frame(frame_len);
    uint8_t* ptr = frame.data();
    
    // 帧头
    *ptr++ = 0xAA;
    *ptr++ = 0x55;
    
    // 长度
    *ptr++ = data_len;
    
    // 命令字
    *ptr++ = static_cast<uint8_t>(cmd);
    
    // 数据域
    if (data_len > 0 && data) {
        memcpy(ptr, data, data_len);
        ptr += data_len;
    }
    
    // 计算CRC (从长度字节开始，包含数据域)
    uint8_t crc = 0;
    const uint8_t* crc_start = frame.data() + 2;  // 跳过帧头
    crc = crc8_update(crc, data_len);  // 长度
    crc = crc8_update(crc, static_cast<uint8_t>(cmd));  // 命令
    for (int i = 0; i < data_len; i++) {  // 数据
        crc = crc8_update(crc, static_cast<const uint8_t*>(data)[i]);
    }
    *ptr++ = crc;
    
    // 帧尾
    *ptr++ = 0x0D;
    *ptr++ = 0x0A;
    
    // 加入发送队列
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        tx_queue_.push(frame);
    }
    cv_.notify_one();
}

void UARTProtocolROS::sendMotionCommand(float left_speed, float right_speed) {
    MotionCmd cmd = {left_speed, right_speed};
    sendProtocolFrame(CMD_MOTION_CONTROL, &cmd, sizeof(MotionCmd));
}

void UARTProtocolROS::sendServoCommand(uint8_t servo_id, int16_t angle) {
    ServoCmd cmd = {servo_id, angle};
    sendProtocolFrame(CMD_SERVO_CONTROL, &cmd, sizeof(ServoCmd));
}

void UARTProtocolROS::requestStatus() {
    sendProtocolFrame(CMD_REQUEST_STATUS, nullptr, 0);
}

void UARTProtocolROS::sendHeartbeat() {
    sendProtocolFrame(CMD_HEARTBEAT, nullptr, 0);
}

void UARTProtocolROS::requestMotorSpeed() {
    sendProtocolFrame(CMD_REQUEST_MOTOR_SPEED, nullptr, 0);
}

RobotStatus UARTProtocolROS::getLatestStatus() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(status_mutex_));
    return latest_status_;
}