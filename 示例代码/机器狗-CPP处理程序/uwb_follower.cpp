/**
 * ============================================================================
 * UWB单基站跟随套件 - 机器狗跟随库（实现文件）
 * ============================================================================
 * 
 * 文件名：uwb_follower.cpp
 * 
 * 编译方法：
 *     g++ -c uwb_follower.cpp -o uwb_follower.o -std=c++11
 * 
 * 与主程序链接：
 *     g++ main.cpp uwb_follower.o -o dog_follow -std=c++11 -lm
 * 
 * ============================================================================
 */

#include "uwb_follower.h"
#include <iostream>
#include <cstring>

// Linux串口头文件
#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#endif

// ============================================================================
// 工具函数实现
// ============================================================================

uint16_t U16HighLowByteSwap(uint16_t value) {
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
}

uint32_t U32HighLowByteSwap(uint32_t value) {
    return ((value & 0x000000FF) << 24) |
           ((value & 0x0000FF00) << 8)  |
           ((value & 0x00FF0000) >> 8)  |
           ((value & 0xFF000000) >> 24);
}

// ============================================================================
// AngleSmoother 实现
// ============================================================================

AngleSmoother::AngleSmoother(double threshold)
    : threshold_(threshold), prev_angle_(0), prev_distance_(0), 
      initialized_(false), suppressed_count_(0) {}

void AngleSmoother::process(double angle, double distance, 
                            double& out_angle, double& out_distance) {
    if (!initialized_) {
        prev_angle_ = angle;
        prev_distance_ = distance;
        out_angle = angle;
        out_distance = distance;
        initialized_ = true;
        return;
    }
    
    double delta_angle = angle - prev_angle_;
    
    if (delta_angle > threshold_) {
        out_angle = prev_angle_;
        out_distance = prev_distance_ + 2.0;
        suppressed_count_++;
    } else if (delta_angle < -threshold_) {
        out_angle = prev_angle_;
        out_distance = prev_distance_ - 2.0;
        suppressed_count_++;
    } else {
        out_angle = angle;
        out_distance = distance;
    }
    
    prev_angle_ = angle;
    prev_distance_ = distance;
}

void AngleSmoother::reset() {
    initialized_ = false;
    suppressed_count_ = 0;
}

// ============================================================================
// MedianFilter 实现
// ============================================================================

MedianFilter::MedianFilter(int window_size)
    : window_size_(window_size) {}

double MedianFilter::update(double value) {
    buffer_.push_back(value);
    if (buffer_.size() > (size_t)window_size_) {
        buffer_.pop_front();
    }
    
    if (buffer_.size() < (size_t)window_size_) {
        return value;
    }
    
    std::vector<double> sorted_values(buffer_.begin(), buffer_.end());
    std::sort(sorted_values.begin(), sorted_values.end());
    return sorted_values[sorted_values.size() / 2];
}

void MedianFilter::reset() {
    buffer_.clear();
}

// ============================================================================
// MedianFilter2D 实现
// ============================================================================

MedianFilter2D::MedianFilter2D(int window_size)
    : filter_x_(window_size), filter_y_(window_size) {}

void MedianFilter2D::update(double x, double y, double& out_x, double& out_y) {
    out_x = filter_x_.update(x);
    out_y = filter_y_.update(y);
}

void MedianFilter2D::reset() {
    filter_x_.reset();
    filter_y_.reset();
}

// ============================================================================
// SimpleEKF2D 实现
// ============================================================================

SimpleEKF2D::SimpleEKF2D(double process_noise, double measurement_noise)
    : process_noise_(process_noise), measurement_noise_(measurement_noise),
      initialized_(false), last_time_ms_(0) {
    x_[0] = x_[1] = x_[2] = x_[3] = 0.0;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_[i][j] = (i == j) ? 100.0 : 0.0;
        }
    }
}

uint64_t SimpleEKF2D::getCurrentTimeMs() {
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch());
    return ms.count();
}

void SimpleEKF2D::update(double x, double y, 
                         double& out_x, double& out_y,
                         double& out_vx, double& out_vy) {
    uint64_t current_time = getCurrentTimeMs();
    
    if (!initialized_) {
        x_[0] = x;
        x_[1] = y;
        x_[2] = 0.0;
        x_[3] = 0.0;
        last_time_ms_ = current_time;
        initialized_ = true;
        
        out_x = x;
        out_y = y;
        out_vx = 0.0;
        out_vy = 0.0;
        return;
    }
    
    double dt = (current_time - last_time_ms_) / 1000.0;
    if (dt <= 0) dt = 0.1;
    if (dt > 1.0) dt = 1.0;
    last_time_ms_ = current_time;
    
    // 预测步骤
    double x_pred[4];
    x_pred[0] = x_[0] + x_[2] * dt;
    x_pred[1] = x_[1] + x_[3] * dt;
    x_pred[2] = x_[2];
    x_pred[3] = x_[3];
    
    double F[4][4] = {
        {1, 0, dt, 0},
        {0, 1, 0, dt},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    
    double P_pred[4][4];
    double temp[4][4];
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            temp[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                temp[i][j] += F[i][k] * P_[k][j];
            }
        }
    }
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_pred[i][j] = process_noise_ * ((i == j) ? 1.0 : 0.0);
            if (i >= 2 && j >= 2 && i == j) {
                P_pred[i][j] *= 2.0;
            }
            for (int k = 0; k < 4; k++) {
                P_pred[i][j] += temp[i][k] * F[j][k];
            }
        }
    }
    
    // 更新步骤
    double z[2] = {x, y};
    double y_residual[2];
    y_residual[0] = z[0] - x_pred[0];
    y_residual[1] = z[1] - x_pred[1];
    
    double S[2][2];
    S[0][0] = P_pred[0][0] + measurement_noise_;
    S[0][1] = P_pred[0][1];
    S[1][0] = P_pred[1][0];
    S[1][1] = P_pred[1][1] + measurement_noise_;
    
    double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (std::abs(det) < 1e-10) det = 1e-10;
    
    double S_inv[2][2];
    S_inv[0][0] = S[1][1] / det;
    S_inv[0][1] = -S[0][1] / det;
    S_inv[1][0] = -S[1][0] / det;
    S_inv[1][1] = S[0][0] / det;
    
    double K[4][2];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            K[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                K[i][j] += P_pred[i][k] * S_inv[k][j];
            }
        }
    }
    
    for (int i = 0; i < 4; i++) {
        x_[i] = x_pred[i] + K[i][0] * y_residual[0] + K[i][1] * y_residual[1];
    }
    
    double I_KH[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            I_KH[i][j] = (i == j) ? 1.0 : 0.0;
            if (j < 2) {
                I_KH[i][j] -= K[i][j];
            }
        }
    }
    
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P_[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                P_[i][j] += I_KH[i][k] * P_pred[k][j];
            }
        }
    }
    
    out_x = x_[0];
    out_y = x_[1];
    out_vx = x_[2];
    out_vy = x_[3];
}

void SimpleEKF2D::reset() {
    initialized_ = false;
}

// ============================================================================
// PhysicalConstraintChecker 实现
// ============================================================================

PhysicalConstraintChecker::PhysicalConstraintChecker(double max_velocity)
    : max_velocity_(max_velocity), initialized_(false), 
      last_x_(0), last_y_(0), last_time_ms_(0) {}

uint64_t PhysicalConstraintChecker::getCurrentTimeMs() {
    auto now = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch());
    return ms.count();
}

bool PhysicalConstraintChecker::check(double x, double y, double distance, double azimuth) {
    if (distance < UWBConfig::MIN_DISTANCE || distance > UWBConfig::MAX_DISTANCE) {
        return false;
    }
    
    if (azimuth < UWBConfig::MIN_AZIMUTH || azimuth > UWBConfig::MAX_AZIMUTH) {
        return false;
    }
    
    if (initialized_) {
        uint64_t current_time = getCurrentTimeMs();
        double dt = (current_time - last_time_ms_) / 1000.0;
        
        if (dt > 0) {
            double dx = x - last_x_;
            double dy = y - last_y_;
            double velocity = std::sqrt(dx*dx + dy*dy) / dt;
            
            if (velocity > max_velocity_) {
                return false;
            }
        }
    }
    
    last_x_ = x;
    last_y_ = y;
    last_time_ms_ = getCurrentTimeMs();
    initialized_ = true;
    
    return true;
}

void PhysicalConstraintChecker::reset() {
    initialized_ = false;
}

// ============================================================================
// UWBFrameParser 实现
// ============================================================================

UWBFrameParser::UWBFrameParser() : buffer_index_(0) {}

bool UWBFrameParser::feedByte(uint8_t byte, UWBRawData& out_data) {
    buffer_[buffer_index_++] = byte;
    
    if (buffer_index_ >= sizeof(buffer_)) {
        buffer_index_ = 0;
        return false;
    }
    
    if (buffer_index_ >= 4) {
        bool header_found = true;
        for (int i = 0; i < 4; i++) {
            if (buffer_[buffer_index_ - 4 + i] != 0xFF) {
                header_found = false;
                break;
            }
        }
        
        if (header_found && buffer_index_ == 4) {
            // 帧头找到
        } else if (!header_found && buffer_index_ < 6) {
            if (buffer_index_ > 3) {
                memmove(buffer_, buffer_ + buffer_index_ - 3, 3);
                buffer_index_ = 3;
            }
        }
    }
    
    if (buffer_index_ >= UWBConfig::FRAME_SIZE) {
        if (buffer_[0] == 0xFF && buffer_[1] == 0xFF && 
            buffer_[2] == 0xFF && buffer_[3] == 0xFF) {
            
            uint8_t xor_check = 0;
            for (int i = 0; i < UWBConfig::FRAME_SIZE - 1; i++) {
                xor_check ^= buffer_[i];
            }
            
            if (xor_check == buffer_[UWBConfig::FRAME_SIZE - 1]) {
                uint16_t cmd = (static_cast<uint16_t>(buffer_[8]) << 8) | 
                               static_cast<uint16_t>(buffer_[9]);
                
                if (cmd == CMD_LOCATION) {
                    // 注意：必须先转换为uint32_t再位移，否则uint8_t左移24位会溢出
                    out_data.anchor_id = U32HighLowByteSwap(
                        (static_cast<uint32_t>(buffer_[12]) << 24) | 
                        (static_cast<uint32_t>(buffer_[13]) << 16) |
                        (static_cast<uint32_t>(buffer_[14]) << 8) | 
                        static_cast<uint32_t>(buffer_[15]));
                    
                    out_data.tag_id = U32HighLowByteSwap(
                        (static_cast<uint32_t>(buffer_[16]) << 24) | 
                        (static_cast<uint32_t>(buffer_[17]) << 16) |
                        (static_cast<uint32_t>(buffer_[18]) << 8) | 
                        static_cast<uint32_t>(buffer_[19]));
                    
                    out_data.distance_cm = U32HighLowByteSwap(
                        (static_cast<uint32_t>(buffer_[20]) << 24) | 
                        (static_cast<uint32_t>(buffer_[21]) << 16) |
                        (static_cast<uint32_t>(buffer_[22]) << 8) | 
                        static_cast<uint32_t>(buffer_[23]));
                    
                    out_data.azimuth_deg = static_cast<int16_t>(
                        U16HighLowByteSwap(
                            (static_cast<uint16_t>(buffer_[24]) << 8) | 
                            static_cast<uint16_t>(buffer_[25])));
                    
                    out_data.elevation_deg = static_cast<int16_t>(
                        U16HighLowByteSwap(
                            (static_cast<uint16_t>(buffer_[26]) << 8) | 
                            static_cast<uint16_t>(buffer_[27])));
                    
                    out_data.is_valid = true;
                    
                    buffer_index_ = 0;
                    return true;
                }
            }
        }
        
        memmove(buffer_, buffer_ + 1, buffer_index_ - 1);
        buffer_index_--;
    }
    
    out_data.is_valid = false;
    return false;
}

void UWBFrameParser::reset() {
    buffer_index_ = 0;
}

// ============================================================================
// UWBDataProcessor 实现
// ============================================================================

UWBDataProcessor::UWBDataProcessor() : total_count_(0), filtered_count_(0) {}

UWB2DData UWBDataProcessor::process(const UWBRawData& raw) {
    UWB2DData result;
    result.is_valid = false;
    
    if (!raw.is_valid) {
        return result;
    }
    
    total_count_++;
    
    double distance = static_cast<double>(raw.distance_cm);
    double azimuth = static_cast<double>(raw.azimuth_deg);
    
    // Step 1: 物理约束检查（初步检查）
    if (distance < UWBConfig::MIN_DISTANCE || distance > UWBConfig::MAX_DISTANCE) {
        return result;
    }
    if (azimuth < UWBConfig::MIN_AZIMUTH || azimuth > UWBConfig::MAX_AZIMUTH) {
        return result;
    }
    
    // Step 2: 角度突变抑制
    double smoothed_azimuth, smoothed_distance;
    angle_smoother_.process(azimuth, distance, smoothed_azimuth, smoothed_distance);
    
    // 坐标转换
    double azimuth_rad = smoothed_azimuth * M_PI / 180.0;
    double raw_x = smoothed_distance * std::sin(azimuth_rad);
    double raw_y = smoothed_distance * std::cos(azimuth_rad);
    
    // Step 3: 物理约束检查（速度检查）
    if (!constraint_checker_.check(raw_x, raw_y, smoothed_distance, smoothed_azimuth)) {
        return result;
    }
    
    // Step 4: 中位数滤波
    double median_x, median_y;
    median_filter_.update(raw_x, raw_y, median_x, median_y);
    
    // Step 5: EKF状态估计
    double ekf_x, ekf_y, ekf_vx, ekf_vy;
    ekf_.update(median_x, median_y, ekf_x, ekf_y, ekf_vx, ekf_vy);
    
    result.x = ekf_x;
    result.y = ekf_y;
    result.vx = ekf_vx;
    result.vy = ekf_vy;
    result.distance_cm = smoothed_distance;
    result.azimuth_deg = smoothed_azimuth;
    result.is_valid = true;
    
    filtered_count_++;
    return result;
}

void UWBDataProcessor::reset() {
    angle_smoother_.reset();
    constraint_checker_.reset();
    median_filter_.reset();
    ekf_.reset();
    total_count_ = 0;
    filtered_count_ = 0;
}

// ============================================================================
// UWBFollower 实现
// ============================================================================

UWBFollower::UWBFollower() : serial_fd_(-1), has_raw_data_(false) {}

UWBFollower::~UWBFollower() {
    close();
}

bool UWBFollower::init(const std::string& port, int baudrate) {
#ifdef __linux__
    serial_fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        std::cerr << "错误：无法打开串口 " << port 
                  << " (" << strerror(errno) << ")" << std::endl;
        return false;
    }
    
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    if (tcgetattr(serial_fd_, &tty) != 0) {
        std::cerr << "错误：无法获取串口属性" << std::endl;
        ::close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    speed_t baud;
    switch (baudrate) {
        case 9600: baud = B9600; break;
        case 19200: baud = B19200; break;
        case 38400: baud = B38400; break;
        case 57600: baud = B57600; break;
        case 115200: baud = B115200; break;
        default: baud = B115200;
    }
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | 
                     INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
        std::cerr << "错误：无法设置串口属性" << std::endl;
        ::close(serial_fd_);
        serial_fd_ = -1;
        return false;
    }
    
    return true;
#else
    std::cerr << "错误：此平台不支持串口" << std::endl;
    return false;
#endif
}

void UWBFollower::close() {
#ifdef __linux__
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
#endif
}

bool UWBFollower::isConnected() const {
    return serial_fd_ >= 0;
}

UWB2DData UWBFollower::getData() {
    UWB2DData result;
    result.is_valid = false;
    
#ifdef __linux__
    if (serial_fd_ < 0) {
        return result;
    }
    
    uint8_t buffer[128];
    int n = ::read(serial_fd_, buffer, sizeof(buffer));
    
    if (n > 0) {
        for (int i = 0; i < n; i++) {
            UWBRawData raw_data;
            if (parser_.feedByte(buffer[i], raw_data)) {
                last_raw_data_ = raw_data;
                has_raw_data_ = true;
                result = processor_.process(raw_data);
                if (result.is_valid) {
                    return result;
                }
            }
        }
    }
#endif
    
    return result;
}

UWBRawData UWBFollower::getRawData() {
    if (has_raw_data_) {
        return last_raw_data_;
    }
    return UWBRawData();
}

UWB2DData UWBFollower::processRawData(const UWBRawData& raw) {
    return processor_.process(raw);
}

void UWBFollower::reset() {
    parser_.reset();
    processor_.reset();
    has_raw_data_ = false;
}

int UWBFollower::getTotalCount() const {
    return processor_.getTotalCount();
}

int UWBFollower::getFilteredCount() const {
    return processor_.getFilteredCount();
}

int UWBFollower::getSuppressedCount() const {
    return processor_.getSuppressedCount();
}
