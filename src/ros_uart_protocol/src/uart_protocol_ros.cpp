// Cleaned and corrected implementation
#include "ros_uart_protocol/uart_protocol_ros.h"
#include <ros/console.h>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <errno.h>

using namespace std::chrono;

UARTProtocolROS::UARTProtocolROS(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate) {
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
        if (serial_port_.isOpen()) {
            serial_port_.close();
        }
        if (access(port_.c_str(), F_OK) != 0) {
            ROS_ERROR("串口设备不存在: %s", port_.c_str());
            return false;
        }
        if (access(port_.c_str(), R_OK | W_OK) != 0) {
            ROS_ERROR("无权访问串口设备: %s（请将当前用户加入dialout组或配置udev权限）", port_.c_str());
            return false;
        }
        serial_port_.open();
        if (!serial_port_.isOpen()) {
            ROS_ERROR("打开串口失败: %s", port_.c_str());
            return false;
        }
        running_ = true;
        receive_thread_ = std::thread(&UARTProtocolROS::receiveThreadFunc, this);
        heartbeat_thread_ = std::thread(&UARTProtocolROS::heartbeatThreadFunc, this);
        send_thread_ = std::thread(&UARTProtocolROS::sendThreadFunc, this);
        ROS_INFO("已连接到串口: %s", port_.c_str());
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("串口连接异常: %s。常见原因：设备未插入或被占用、权限不足、波特率不匹配。", e.what());
        running_ = false;
        return false;
    }
}

void UARTProtocolROS::disconnect() {
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        running_ = false;
    }
    cv_.notify_all();
    heartbeat_cv_.notify_all();
    if (receive_thread_.joinable()) receive_thread_.join();
    if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
    if (send_thread_.joinable()) send_thread_.join();
    try {
        if (serial_port_.isOpen()) serial_port_.close();
    } catch (const std::exception& e) {
        ROS_ERROR("关闭串口时发生错误: %s", e.what());
    }
    ROS_INFO("已断开串口: %s", port_.c_str());
}

bool UARTProtocolROS::isConnected() const {
    return serial_port_.isOpen() && running_;
}

uint8_t UARTProtocolROS::crc8_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (crc << 1) ^ 0x07;
        else crc = (crc << 1);
        crc &= 0xFF;
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
        if (!isConnected()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }
        try {
            size_t bytes_avail = serial_port_.available();
            if (bytes_avail == 0) { std::this_thread::sleep_for(milliseconds(5)); continue; }
            std::vector<uint8_t> data(bytes_avail);
            size_t bytes_read = serial_port_.read(data.data(), bytes_avail);
            for (size_t i = 0; i < bytes_read; ++i) {
                uint8_t rx_byte = data[i];
                switch (rx_state) {
                    case FRAME_HEADER1:
                        calculated_crc = 0;
                        if (rx_byte == 0xAA) { rx_state = FRAME_HEADER2; rx_buffer.clear(); rx_buffer.push_back(rx_byte);} break;
                    case FRAME_HEADER2:
                        if (rx_byte == 0x55) { rx_state = FRAME_LENGTH; rx_buffer.push_back(rx_byte);} else { rx_state = FRAME_HEADER1;} break;
                    case FRAME_LENGTH:
                        expected_length = rx_byte; calculated_crc = crc8_update(0, rx_byte); rx_buffer.push_back(rx_byte);
                        if (expected_length <= MAX_FRAME_LENGTH - 4) rx_state = FRAME_CMD; else { ROS_WARN("无效的帧长度: %d", expected_length); rx_state = FRAME_HEADER1; }
                        break;
                    case FRAME_CMD:
                        current_cmd = rx_byte; calculated_crc = crc8_update(calculated_crc, rx_byte); rx_buffer.push_back(rx_byte);
                        rx_state = (expected_length > 0) ? FRAME_DATA : FRAME_CRC; break;
                    case FRAME_DATA:
                        rx_buffer.push_back(rx_byte); calculated_crc = crc8_update(calculated_crc, rx_byte); data_bytes_received++;
                        if (data_bytes_received >= expected_length) rx_state = FRAME_CRC; break;
                    case FRAME_CRC:
                        rx_buffer.push_back(rx_byte);
                        if (rx_byte == calculated_crc) rx_state = FRAME_TAIL1; else { ROS_WARN("CRC校验失败: 期望0x%02X, 实际0x%02X, 命令: 0x%02X, 长度: %d", calculated_crc, rx_byte, current_cmd, expected_length); rx_state = FRAME_HEADER1; }
                        break;
                    case FRAME_TAIL1:
                        rx_state = (rx_byte == 0x0D) ? FRAME_TAIL2 : FRAME_HEADER1; break;
                    case FRAME_TAIL2:
                        if (rx_byte == 0x0A) {
                            rx_buffer.push_back(rx_byte);
                            if (rx_buffer.size() >= 4 + expected_length) {
                                std::vector<uint8_t> payload(rx_buffer.begin() + 4, rx_buffer.begin() + 4 + expected_length);
                                processFrame(current_cmd, payload);
                            }
                        }
                        rx_state = FRAME_HEADER1; data_bytes_received = 0; break;
                }
            }
        } catch (const serial::IOException& e) { ROS_ERROR("串口IO错误: %s", e.what()); disconnect(); }
          catch (const serial::SerialException& e) { ROS_ERROR("串口异常: %s", e.what()); disconnect(); }
          catch (const std::exception& e) { ROS_ERROR("接收线程未预期的错误: %s", e.what()); disconnect(); }
    }
}

void UARTProtocolROS::processFrame(uint8_t cmd, const std::vector<uint8_t>& data) {
    switch (cmd) {
        case CMD_HEARTBEAT_ACK: { std::lock_guard<std::mutex> lock(heartbeat_mutex_); heartbeat_ack_received_ = true; heartbeat_cv_.notify_one(); } break;
        case CMD_STATUS_FEEDBACK:
            if (data.size() == sizeof(RobotStatus)) { RobotStatus status; memcpy(&status, data.data(), sizeof(RobotStatus)); std::lock_guard<std::mutex> lock(status_mutex_); latest_status_ = status; }
            break;
        case CMD_MOTOR_SPEED_FEEDBACK:
            if (data.size() == 8) { float speedL, speedR; memcpy(&speedL, data.data(), sizeof(float)); memcpy(&speedR, data.data() + sizeof(float), sizeof(float)); std::lock_guard<std::mutex> lock(status_mutex_); latest_status_.speedL = speedL; latest_status_.speedR = speedR; }
            break;
        case CMD_ERROR_REPORT:
            if (!data.empty()) ROS_ERROR("设备上报错误: 0x%02X", data[0]);
            break;
        default:
            ROS_DEBUG("收到未知命令: 0x%02X", cmd);
            break;
    }
}

void UARTProtocolROS::heartbeatThreadFunc() {
    while (running_) {
        if (!isConnected()) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }
        try { sendHeartbeat(); } catch (const std::exception& e) { ROS_ERROR("发送心跳出错: %s", e.what()); continue; }
        {
            std::unique_lock<std::mutex> lock(heartbeat_mutex_);
            heartbeat_ack_received_ = false;
            bool ack_received = heartbeat_cv_.wait_for(lock, std::chrono::milliseconds(HEARTBEAT_TIMEOUT_MS), [this]{ return heartbeat_ack_received_ || !running_; });
            if (!ack_received && running_) {
                int current_missed = ++missed_beats_;
                if (current_missed >= 3) {
                    ROS_WARN("心跳超时（连续丢失%d次），正在尝试重连...", current_missed);
                    try { if (serial_port_.isOpen()) serial_port_.close(); } catch (...) {}
                    continue;
                }
            } else { missed_beats_ = 0; }
        }
        if (running_) std::this_thread::sleep_for(std::chrono::milliseconds(HEARTBEAT_INTERVAL_MS));
    }
}

bool UARTProtocolROS::isDeviceOnline() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(heartbeat_mutex_));
    return serial_port_.isOpen() && running_ && (missed_beats_ < 3);
}

void UARTProtocolROS::tryReconnect() {
    if (!running_) return;
    static auto last_attempt = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_attempt).count() < RECONNECT_INTERVAL_MS) return;
    last_attempt = now;
    try { if (serial_port_.isOpen()) { serial_port_.close(); std::this_thread::sleep_for(std::chrono::milliseconds(100)); } } catch (...) {}
    ROS_INFO("正在尝试重新连接到 %s...", port_.c_str());
    try {
        serial_port_.open();
        if (serial_port_.isOpen()) {
            ROS_INFO("重连成功");
            missed_beats_ = 0;
            if (!receive_thread_.joinable()) receive_thread_ = std::thread(&UARTProtocolROS::receiveThreadFunc, this);
            if (!heartbeat_thread_.joinable()) heartbeat_thread_ = std::thread(&UARTProtocolROS::heartbeatThreadFunc, this);
            if (!send_thread_.joinable()) send_thread_ = std::thread(&UARTProtocolROS::sendThreadFunc, this);
        }
    } catch (const std::exception& e) { ROS_ERROR("重连失败: %s", e.what()); }
}

void UARTProtocolROS::sendThreadFunc() {
    while (running_) {
        std::unique_lock<std::mutex> lock(tx_mutex_);
        bool has_data = cv_.wait_for(lock, std::chrono::milliseconds(100), [this]{ return !tx_queue_.empty() || !running_; });
        if (!running_) break; if (!has_data) continue;
        while (!tx_queue_.empty() && running_) {
            auto data = std::move(tx_queue_.front());
            tx_queue_.pop();
            lock.unlock();
            try { if (serial_port_.isOpen()) { serial_port_.write(data); serial_port_.flush(); } }
            catch (const std::exception& e) { ROS_ERROR("串口写入错误: %s", e.what()); try { if (serial_port_.isOpen()) serial_port_.close(); } catch (...) {} }
            lock.lock();
        }
    }
}

void UARTProtocolROS::sendProtocolFrame(ProtocolCmd cmd, const void* data, uint8_t data_len) {
    uint16_t frame_len = 2 + 1 + 1 + data_len + 1 + 2;
    std::vector<uint8_t> frame(frame_len);
    uint8_t* ptr = frame.data();
    *ptr++ = 0xAA; *ptr++ = 0x55; *ptr++ = data_len; *ptr++ = static_cast<uint8_t>(cmd);
    if (data_len > 0 && data) { memcpy(ptr, data, data_len); ptr += data_len; }
    uint8_t crc = 0; crc = crc8_update(crc, data_len); crc = crc8_update(crc, static_cast<uint8_t>(cmd));
    for (int i = 0; i < data_len; i++) crc = crc8_update(crc, static_cast<const uint8_t*>(data)[i]);
    *ptr++ = crc; *ptr++ = 0x0D; *ptr++ = 0x0A;
    { std::lock_guard<std::mutex> lock(tx_mutex_); tx_queue_.push(frame); }
    cv_.notify_one();
}

void UARTProtocolROS::sendMotionCommand(float left_speed, float right_speed) { MotionCmd cmd = {left_speed, right_speed}; sendProtocolFrame(CMD_MOTION_CONTROL, &cmd, sizeof(MotionCmd)); }
void UARTProtocolROS::sendServoCommand(uint8_t servo_id, int16_t angle) { ServoCmd cmd = {servo_id, angle}; sendProtocolFrame(CMD_SERVO_CONTROL, &cmd, sizeof(ServoCmd)); }
void UARTProtocolROS::requestStatus() { sendProtocolFrame(CMD_REQUEST_STATUS, nullptr, 0); }
void UARTProtocolROS::sendHeartbeat() { sendProtocolFrame(CMD_HEARTBEAT, nullptr, 0); }
void UARTProtocolROS::requestMotorSpeed() { sendProtocolFrame(CMD_REQUEST_MOTOR_SPEED, nullptr, 0); }
void UARTProtocolROS::sendLightControl(uint8_t enable, uint8_t brightness) {
    uint8_t data[2] = {enable, brightness};
    sendProtocolFrame(CMD_LIGHT_CONTROL, data, 2);
}
void UARTProtocolROS::sendMotorEnable(uint8_t enable) {
    sendProtocolFrame(CMD_MOTOR_ENABLE, &enable, 1);
}
RobotStatus UARTProtocolROS::getLatestStatus() const { std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(status_mutex_)); return latest_status_; }