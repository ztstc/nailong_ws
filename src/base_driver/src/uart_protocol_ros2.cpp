// ROS2 Humble compatible implementation with optimized UART logic
#include "base_driver/uart_protocol_ros2.h"
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <errno.h>
#include <iomanip>
#include <sstream>
#include <libserial/SerialPortConstants.h>

using namespace std::chrono;
using namespace LibSerial;

UARTProtocolROS2::UARTProtocolROS2(const std::string& port, int baudrate, rclcpp::Logger logger)
    : port_(port), baudrate_(baudrate), logger_(logger), last_reconnect_attempt_(steady_clock::now()) {
    try {
        serial_port_ = std::make_unique<SerialPort>();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Failed to create SerialPort object: %s", e.what());
        serial_port_ = nullptr;
    }
}

UARTProtocolROS2::~UARTProtocolROS2() {
    disconnect();
}

bool UARTProtocolROS2::connect() {
    try {
        if (!serial_port_) {
            RCLCPP_ERROR(logger_, "SerialPort object not initialized");
            return false;
        }
        
        try {
            if (serial_port_->IsOpen()) {
                serial_port_->Close();
            }
        } catch (...) {
            // Ignore close errors
        }
        
        if (access(port_.c_str(), F_OK) != 0) {
            RCLCPP_ERROR(logger_, "Serial device does not exist: %s (available: ls /dev/tty*)", port_.c_str());
            return false;
        }
        if (access(port_.c_str(), R_OK | W_OK) != 0) {
            RCLCPP_ERROR(logger_, "No permission to access serial device: %s. Add user to dialout: sudo usermod -a -G dialout $USER", port_.c_str());
            return false;
        }
        
        try {
            RCLCPP_INFO(logger_, "Opening serial port: %s", port_.c_str());
            serial_port_->Open(port_, std::ios_base::in | std::ios_base::out);
            RCLCPP_INFO(logger_, "Successfully opened serial port: %s", port_.c_str());
        } catch (const LibSerial::OpenFailed& e) {
            RCLCPP_ERROR(logger_, "Failed to open serial port '%s': OpenFailed - %s", port_.c_str(), e.what());
            return false;
        } catch (const LibSerial::AlreadyOpen& e) {
            RCLCPP_WARN(logger_, "Serial port already open: %s", port_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to open serial port '%s': %s", port_.c_str(), e.what());
            return false;
        }
        
        try {
            serial_port_->SetBaudRate(BaudRate::BAUD_115200);
            serial_port_->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            serial_port_->SetParity(Parity::PARITY_NONE);
            serial_port_->SetStopBits(StopBits::STOP_BITS_1);
            serial_port_->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Failed to configure serial port: %s", e.what());
            return false;
        }
        running_ = true;
        consecutive_errors_ = 0;
        
        // Only start threads if connection succeeded
        if (serial_port_ && serial_port_->IsOpen()) {
            receive_thread_ = std::thread(&UARTProtocolROS2::receiveThreadFunc, this);
            heartbeat_thread_ = std::thread(&UARTProtocolROS2::heartbeatThreadFunc, this);
            send_thread_ = std::thread(&UARTProtocolROS2::sendThreadFunc, this);
            RCLCPP_INFO(logger_, "Connected to serial port: %s at %d baud", port_.c_str(), baudrate_);
        } else {
            RCLCPP_WARN(logger_, "Serial port not yet opened, starting in retry mode");
            // Start a reconnect attempt thread
            std::thread reconnect_thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                if (!isConnected()) {
                    this->resetConnection();
                }
            });
            reconnect_thread.detach();
        }
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Serial connection exception: %s. Common causes: device not found, already in use, insufficient permissions, baudrate mismatch", e.what());
        running_ = false;
        return false;
    }
}

void UARTProtocolROS2::disconnect() {
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
        if (serial_port_) {
            try { 
                if (serial_port_->IsOpen()) {
                    serial_port_->Close();
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(logger_, "Warning closing serial port: %s", e.what());
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error closing serial port: %s", e.what());
    }
    RCLCPP_INFO(logger_, "Disconnected from serial port: %s", port_.c_str());
}

bool UARTProtocolROS2::isConnected() const {
    try {
        return serial_port_ && serial_port_->IsOpen() && running_;
    } catch (...) {
        return false;
    }
}

uint8_t UARTProtocolROS2::crc8_update(uint8_t crc, uint8_t data) {
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 0x80) crc = (crc << 1) ^ 0x07;
        else crc = (crc << 1);
        crc &= 0xFF;
    }
    return crc;
}

void UARTProtocolROS2::receiveThreadFunc() {
    RxState rx_state = RxState::FRAME_HEADER1;
    std::vector<uint8_t> rx_buffer;
    uint8_t expected_length = 0;
    uint8_t current_cmd = 0;
    uint8_t calculated_crc = 0;
    uint8_t data_bytes_received = 0;
    
    while (running_) {
        if (!isConnected()) {
            std::this_thread::sleep_for(milliseconds(100));
            continue;
        }
        try {
            uint8_t rx_byte = 0;
            try {
                serial_port_->ReadByte(rx_byte, 10);  // 10ms timeout
            } catch (const std::exception&) {
                // Timeout or read error - just continue
                std::this_thread::sleep_for(milliseconds(5));
                continue;
            }
            
            consecutive_errors_ = 0;  // Reset error counter on successful read
            
            switch (rx_state) {
                case RxState::FRAME_HEADER1:
                    if (rx_byte == 0xAA) {
                        calculated_crc = 0;
                        rx_state = RxState::FRAME_HEADER2;
                        rx_buffer.clear();
                        rx_buffer.push_back(rx_byte);
                    }
                    break;
                case RxState::FRAME_HEADER2:
                    if (rx_byte == 0x55) {
                        rx_state = RxState::FRAME_LENGTH;
                        rx_buffer.push_back(rx_byte);
                    } else {
                        rx_state = RxState::FRAME_HEADER1;
                    }
                    break;
                case RxState::FRAME_LENGTH:
                    expected_length = rx_byte;
                    calculated_crc = crc8_update(0, rx_byte);
                    rx_buffer.push_back(rx_byte);
                    if (expected_length <= MAX_FRAME_LENGTH - 4) {
                        rx_state = RxState::FRAME_CMD;
                    } else {
                        RCLCPP_WARN(logger_, "Invalid frame length: %d", expected_length);
                        rx_state = RxState::FRAME_HEADER1;
                    }
                    break;
                case RxState::FRAME_CMD:
                    current_cmd = rx_byte;
                    calculated_crc = crc8_update(calculated_crc, rx_byte);
                    rx_buffer.push_back(rx_byte);
                    rx_state = (expected_length > 0) ? RxState::FRAME_DATA : RxState::FRAME_CRC;
                    break;
                case RxState::FRAME_DATA:
                    rx_buffer.push_back(rx_byte);
                    calculated_crc = crc8_update(calculated_crc, rx_byte);
                    data_bytes_received++;
                    if (data_bytes_received >= expected_length) {
                        rx_state = RxState::FRAME_CRC;
                    }
                    break;
                case RxState::FRAME_CRC:
                    rx_buffer.push_back(rx_byte);
                    if (rx_byte == calculated_crc) {
                        rx_state = RxState::FRAME_TAIL1;
                    } else {
                        RCLCPP_WARN(logger_, "CRC check failed: expected 0x%02X, got 0x%02X, cmd 0x%02X, len %d",
                                   calculated_crc, rx_byte, current_cmd, expected_length);
                        rx_state = RxState::FRAME_HEADER1;
                    }
                    break;
                case RxState::FRAME_TAIL1:
                    rx_state = (rx_byte == 0x0D) ? RxState::FRAME_TAIL2 : RxState::FRAME_HEADER1;
                    break;
                case RxState::FRAME_TAIL2:
                    if (rx_byte == 0x0A) {
                        rx_buffer.push_back(rx_byte);
                        if (rx_buffer.size() >= static_cast<size_t>(4 + expected_length)) {
                            std::vector<uint8_t> payload(rx_buffer.begin() + 4, rx_buffer.begin() + 4 + expected_length);
                            processFrame(current_cmd, payload);
                        }
                    }
                    rx_state = RxState::FRAME_HEADER1;
                    data_bytes_received = 0;
                    break;
            }
        } catch (const std::exception& e) {
            handleError(std::string("Unexpected error in receive thread: ") + e.what());
        }
    }
}

void UARTProtocolROS2::processFrame(uint8_t cmd, const std::vector<uint8_t>& data) {
    switch (static_cast<ProtocolCmd>(cmd)) {
        case ProtocolCmd::CMD_HEARTBEAT_ACK: {
            std::lock_guard<std::mutex> lock(heartbeat_mutex_);
            heartbeat_ack_received_ = true;
            heartbeat_cv_.notify_one();
            break;
        }
        case ProtocolCmd::CMD_STATUS_FEEDBACK:
            if (data.size() == sizeof(RobotStatus)) {
                RobotStatus status;
                memcpy(&status, data.data(), sizeof(RobotStatus));
                std::lock_guard<std::mutex> lock(status_mutex_);
                latest_status_ = status;
            } else {
                RCLCPP_WARN(logger_, "Status payload size mismatch: expected %zu, got %zu",
                           sizeof(RobotStatus), data.size());
            }
            break;
        case ProtocolCmd::CMD_MOTOR_SPEED_FEEDBACK:
            if (data.size() == 8) {
                float speedL, speedR;
                memcpy(&speedL, data.data(), sizeof(float));
                memcpy(&speedR, data.data() + sizeof(float), sizeof(float));
                std::lock_guard<std::mutex> lock(status_mutex_);
                latest_status_.speedL = speedL;
                latest_status_.speedR = speedR;
            }
            break;
        case ProtocolCmd::CMD_ERROR_REPORT:
            if (!data.empty()) {
                RCLCPP_ERROR(logger_, "Device error reported: 0x%02X", data[0]);
            }
            break;
        default:
            RCLCPP_DEBUG(logger_, "Received unknown command: 0x%02X", cmd);
            break;
    }
}

void UARTProtocolROS2::heartbeatThreadFunc() {
    while (running_) {
        if (!isConnected()) {
            std::this_thread::sleep_for(milliseconds(100));
            continue;
        }
        try {
            sendHeartbeat();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger_, "Error sending heartbeat: %s", e.what());
            continue;
        }
        
        {
            std::unique_lock<std::mutex> lock(heartbeat_mutex_);
            heartbeat_ack_received_ = false;
            bool ack_received = heartbeat_cv_.wait_for(lock, milliseconds(HEARTBEAT_TIMEOUT_MS),
                                                       [this] { return heartbeat_ack_received_ || !running_; });
            if (!ack_received && running_) {
                int current_missed = ++missed_beats_;
                if (current_missed >= 3) {
                    RCLCPP_WARN(logger_, "Heartbeat timeout (missed %d times), attempting to reconnect...", current_missed);
                    try {
                        if (serial_port_ && serial_port_->IsOpen()) {
                            serial_port_->Close();
                        }
                    } catch (...) {}
                    continue;
                }
            } else {
                missed_beats_ = 0;
            }
        }
        if (running_) {
            std::this_thread::sleep_for(milliseconds(HEARTBEAT_INTERVAL_MS));
        }
    }
}

bool UARTProtocolROS2::isDeviceOnline() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(heartbeat_mutex_));
    try {
        return serial_port_ && serial_port_->IsOpen() && running_ && (missed_beats_ < 3);
    } catch (...) {
        return false;
    }
}

void UARTProtocolROS2::handleError(const std::string& error_msg) {
    RCLCPP_ERROR(logger_, "%s", error_msg.c_str());
    consecutive_errors_++;
    if (consecutive_errors_ >= MAX_CONSECUTIVE_ERRORS) {
        RCLCPP_WARN(logger_, "Too many consecutive errors (%d), disconnecting", consecutive_errors_.load());
        resetConnection();
    }
}

void UARTProtocolROS2::resetConnection() {
    auto now = steady_clock::now();
    auto elapsed = duration_cast<milliseconds>(now - last_reconnect_attempt_).count();
    
    if (elapsed < RECONNECT_INTERVAL_MS) {
        return;
    }
    last_reconnect_attempt_ = now;
    
    try {
        if (serial_port_ && serial_port_->IsOpen()) {
            serial_port_->Close();
            std::this_thread::sleep_for(milliseconds(100));
        }
    } catch (...) {}
    
    RCLCPP_INFO(logger_, "Attempting to reconnect to %s...", port_.c_str());
    try {
        serial_port_->Open(port_, std::ios_base::in | std::ios_base::out);
        if (serial_port_->IsOpen()) {
            RCLCPP_INFO(logger_, "Reconnection successful");
            consecutive_errors_ = 0;
            missed_beats_ = 0;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Reconnection failed: %s", e.what());
    }
}

void UARTProtocolROS2::sendThreadFunc() {
    while (running_) {
        std::unique_lock<std::mutex> lock(tx_mutex_);
        bool has_data = cv_.wait_for(lock, milliseconds(100), [this] { return !tx_queue_.empty() || !running_; });
        if (!running_) break;
        if (!has_data) continue;
        
        while (!tx_queue_.empty() && running_) {
            auto data = std::move(tx_queue_.front());
            tx_queue_.pop();
            lock.unlock();
            try {
                if (serial_port_ && serial_port_->IsOpen()) {
                    serial_port_->Write(data);
                    serial_port_->DrainWriteBuffer();
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(logger_, "Serial write error: %s", e.what());
                try {
                    if (serial_port_ && serial_port_->IsOpen()) {
                        serial_port_->Close();
                    }
                } catch (...) {}
            }
            lock.lock();
        }
    }
}

void UARTProtocolROS2::sendProtocolFrame(ProtocolCmd cmd, const void* data, uint8_t data_len) {
    uint16_t frame_len = 2 + 1 + 1 + data_len + 1 + 2;
    std::vector<uint8_t> frame(frame_len);
    uint8_t* ptr = frame.data();
    
    *ptr++ = 0xAA;
    *ptr++ = 0x55;
    *ptr++ = data_len;
    *ptr++ = static_cast<uint8_t>(cmd);
    
    if (data_len > 0 && data) {
        memcpy(ptr, data, data_len);
        ptr += data_len;
    }
    
    uint8_t crc = 0;
    crc = crc8_update(crc, data_len);
    crc = crc8_update(crc, static_cast<uint8_t>(cmd));
    for (int i = 0; i < data_len; i++) {
        crc = crc8_update(crc, static_cast<const uint8_t*>(data)[i]);
    }
    
    *ptr++ = crc;
    *ptr++ = 0x0D;
    *ptr++ = 0x0A;
    
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        tx_queue_.push(frame);
    }
    cv_.notify_one();
}

void UARTProtocolROS2::sendMotionCommand(float left_speed, float right_speed) {
    MotionCmd cmd = {left_speed, right_speed};
    sendProtocolFrame(ProtocolCmd::CMD_MOTION_CONTROL, &cmd, sizeof(MotionCmd));
}

void UARTProtocolROS2::sendServoCommand(uint8_t servo_id, int16_t angle) {
    sendServoCommand(servo_id, angle, static_cast<uint16_t>(1000));
}

void UARTProtocolROS2::sendServoCommand(uint8_t servo_id, int16_t angle, uint16_t time_ms) {
    ServoCmd cmd = {servo_id, angle, time_ms};
    sendProtocolFrame(ProtocolCmd::CMD_SERVO_CONTROL, &cmd, sizeof(ServoCmd));
}

void UARTProtocolROS2::requestStatus() {
    sendProtocolFrame(ProtocolCmd::CMD_REQUEST_STATUS, nullptr, 0);
}

void UARTProtocolROS2::sendHeartbeat() {
    sendProtocolFrame(ProtocolCmd::CMD_HEARTBEAT, nullptr, 0);
}

void UARTProtocolROS2::requestMotorSpeed() {
    sendProtocolFrame(ProtocolCmd::CMD_REQUEST_MOTOR_SPEED, nullptr, 0);
}

void UARTProtocolROS2::sendLightControl(uint8_t enable, uint8_t brightness) {
    uint8_t data[2] = {enable, brightness};
    sendProtocolFrame(ProtocolCmd::CMD_LIGHT_CONTROL, data, 2);
}

void UARTProtocolROS2::sendMotorEnable(uint8_t enable) {
    sendProtocolFrame(ProtocolCmd::CMD_MOTOR_ENABLE, &enable, 1);
}

RobotStatus UARTProtocolROS2::getLatestStatus() const {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(status_mutex_));
    return latest_status_;
}