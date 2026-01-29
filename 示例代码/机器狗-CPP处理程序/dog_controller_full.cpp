/**
 * ============================================================================
 * UWB单基站跟随套件 - 机器狗跟随控制完整库（实现文件）
 * ============================================================================
 * 
 * 文件名：dog_controller_full.cpp
 * 
 * 功能说明：
 *     整合 UWB数据接收、协议解析、滤波处理 和 跟随控制 的一体化库实现
 *     无需其他依赖
 * 
 * 编译方法：
 *     g++ your_main.cpp dog_controller_full.cpp -o your_program -std=c++11 -lm
 * 
 * ============================================================================
 */

#include "dog_controller_full.h"
#include <cstring>
#include <cstdio>

// Linux串口头文件
#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#endif

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
    
    // 缓冲区溢出保护
    if (buffer_index_ >= sizeof(buffer_)) {
        buffer_index_ = 0;
        return false;
    }
    
    // 检查是否有帧头（4个0xFF）
    if (buffer_index_ >= 4) {
        if (buffer_[0] != 0xFF || buffer_[1] != 0xFF || 
            buffer_[2] != 0xFF || buffer_[3] != 0xFF) {
            // 寻找可能的帧头起始位置
            int shift = 1;
            for (int i = 1; i < buffer_index_ - 3; i++) {
                if (buffer_[i] == 0xFF && buffer_[i+1] == 0xFF && 
                    buffer_[i+2] == 0xFF && buffer_[i+3] == 0xFF) {
                    shift = i;
                    break;
                }
            }
            if (shift > 0) {
                memmove(buffer_, buffer_ + shift, buffer_index_ - shift);
                buffer_index_ -= shift;
            }
        }
    }
    
    // 检查是否收到完整帧
    if (buffer_index_ >= UWBConfig::FRAME_SIZE) {
        if (buffer_[0] == 0xFF && buffer_[1] == 0xFF && 
            buffer_[2] == 0xFF && buffer_[3] == 0xFF) {
            
            // 异或校验
            uint8_t xor_check = 0;
            for (int i = 0; i < UWBConfig::FRAME_SIZE - 1; i++) {
                xor_check ^= buffer_[i];
            }
            
            if (xor_check == buffer_[UWBConfig::FRAME_SIZE - 1]) {
                // 大端序直接读取
                uint16_t cmd = (static_cast<uint16_t>(buffer_[8]) << 8) | 
                               static_cast<uint16_t>(buffer_[9]);
                
                if (cmd == CMD_LOCATION) {
                    out_data.anchor_id = 
                        (static_cast<uint32_t>(buffer_[12]) << 24) | 
                        (static_cast<uint32_t>(buffer_[13]) << 16) |
                        (static_cast<uint32_t>(buffer_[14]) << 8) | 
                        static_cast<uint32_t>(buffer_[15]);
                    
                    out_data.tag_id = 
                        (static_cast<uint32_t>(buffer_[16]) << 24) | 
                        (static_cast<uint32_t>(buffer_[17]) << 16) |
                        (static_cast<uint32_t>(buffer_[18]) << 8) | 
                        static_cast<uint32_t>(buffer_[19]);
                    
                    out_data.distance_cm = 
                        (static_cast<uint32_t>(buffer_[20]) << 24) | 
                        (static_cast<uint32_t>(buffer_[21]) << 16) |
                        (static_cast<uint32_t>(buffer_[22]) << 8) | 
                        static_cast<uint32_t>(buffer_[23]);
                    
                    out_data.azimuth_deg = static_cast<int16_t>(
                        (static_cast<uint16_t>(buffer_[24]) << 8) | 
                        static_cast<uint16_t>(buffer_[25]));
                    
                    out_data.elevation_deg = static_cast<int16_t>(
                        (static_cast<uint16_t>(buffer_[26]) << 8) | 
                        static_cast<uint16_t>(buffer_[27]));
                    
                    out_data.is_valid = true;
                    buffer_index_ = 0;
                    return true;
                }
            }
        }
        
        // 帧无效，丢弃第一个字节继续寻找
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
// DogController 实现
// ============================================================================

DogController::DogController()
    : serial_fd_(-1)
    , has_raw_data_(false)
    , min_distance_(FollowConfig::MIN_DISTANCE)
    , max_linear_speed_(FollowConfig::MAX_LINEAR_SPEED)
    , max_angular_speed_(FollowConfig::MAX_ANGULAR_SPEED)
    , lost_frame_count_(0)
    , is_following_(false)
    , radial_at_max_speed_(false)
    , angular_at_max_speed_(false)
    , prev_radial_speed_(0.0)
    , prev_angular_speed_(0.0)
    , first_frame_(true)
    , total_frames_(0)
    , valid_frames_(0)
    , debug_mode_(false) {
}

DogController::~DogController() {
    close();
}

bool DogController::init(const std::string& port, int baudrate) {
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
    
    std::cout << "串口 " << port << " 初始化成功 (波特率=" << baudrate << ")" << std::endl;
    return true;
#else
    std::cerr << "错误：此平台不支持串口" << std::endl;
    return false;
#endif
}

void DogController::close() {
#ifdef __linux__
    if (serial_fd_ >= 0) {
        ::close(serial_fd_);
        serial_fd_ = -1;
    }
#endif
}

bool DogController::isConnected() const {
    return serial_fd_ >= 0;
}

UWB2DData DogController::readUWBData() {
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

UWB2DData DogController::getUWBData() {
    return readUWBData();
}

DogVelocity DogController::getVelocity() {
    // 获取UWB数据
    UWB2DData uwb_data = readUWBData();
    total_frames_++;
    
    // 计算速度
    return computeVelocity(uwb_data);
}

DogVelocity DogController::computeVelocity(const UWB2DData& uwb_data) {
    DogVelocity result;
    result.is_valid = false;
    result.is_following = is_following_;
    
    // 保存额外信息
    result.distance_cm = uwb_data.distance_cm;
    result.azimuth_deg = uwb_data.azimuth_deg;
    result.human_vx = uwb_data.vx;
    result.human_vy = uwb_data.vy;
    
    // 检查数据有效性
    if (!uwb_data.is_valid) {
        lost_frame_count_++;
        if (lost_frame_count_ > FollowConfig::MAX_LOST_FRAMES) {
            if (is_following_) {
                if (debug_mode_) {
                    std::cout << "[跟随] 目标丢失，停止跟随" << std::endl;
                }
                is_following_ = false;
            }
            return result;
        }
        // 短暂丢失，保持上一次的控制
        result.is_valid = true;
        result.vx = prev_radial_speed_;
        result.omega = prev_angular_speed_;
        return result;
    }
    
    // 数据有效，重置丢失计数
    lost_frame_count_ = 0;
    valid_frames_++;
    
    // 获取当前距离和角度
    double now_distance = uwb_data.distance_cm;
    double now_angle = uwb_data.azimuth_deg;
    
    // 计算距离和角度差
    double distance_diff = now_distance - min_distance_;
    
    double angle_diff = now_angle - FollowConfig::TARGET_ANGLE;
    double angle_sign = (angle_diff >= 0) ? 1.0 : -1.0;
    angle_diff = std::abs(angle_diff);

    // 获取EKF估计的人的速度
    double human_vx = uwb_data.vx;
    double human_vy = uwb_data.vy;
    
    // 计算人的径向速度
    double human_radial_speed = 0.0;
    if (human_vy > 0) {
        human_radial_speed = std::sqrt(human_vx * human_vx + human_vy * human_vy);
    } else {
        human_radial_speed = FollowConfig::HUMAN_STANDARD_SPEED * 100;
    }
    
    // 计算人的角向速度
    double human_angular_speed = 0.0;
    if (now_distance > 10.0) {
        if ((human_vx > 0) ^ (angle_sign > 0)) {
            human_angular_speed = FollowConfig::HUMAN_ANGULAR_SPEED * 57.3;
        } else {
            human_angular_speed = std::abs(human_vx) / now_distance * 57.3;
        }
    }
    
    // ============== 安全检查 ==============
    
    // 人太近，停止
    if (now_distance < FollowConfig::STOP_DISTANCE) {
        if (is_following_ && debug_mode_) {
            std::cout << "[跟随] 人太近 (" << now_distance << "cm)，停止" << std::endl;
        }
        is_following_ = false;
        radial_at_max_speed_ = false;
        angular_at_max_speed_ = false;
        result.is_valid = true;
        return result;
    }
    
    // 人太远，停止
    if (now_distance > FollowConfig::LOST_DISTANCE) {
        if (is_following_ && debug_mode_) {
            std::cout << "[跟随] 人太远 (" << now_distance << "cm)，停止" << std::endl;
        }
        is_following_ = false;
        radial_at_max_speed_ = false;
        angular_at_max_speed_ = false;
        return result;
    }
    
    // 开始跟随
    if (!is_following_) {
        if (debug_mode_) {
            std::cout << "[跟随] 检测到目标，开始跟随 (距离=" 
                      << now_distance << "cm, 角度=" 
                      << now_angle << "°)" << std::endl;
        }
        is_following_ = true;
    }
    
    double x_speed = 0.0;
    double angular_speed = 0.0;
    
    // ============== 径向运动控制 ==============
            
    if (distance_diff > 0) {
        // 计算二次函数系数
        double radial_coeff = FollowConfig::RADIAL_COEFF_A * distance_diff * distance_diff
                            + FollowConfig::RADIAL_COEFF_C;
        
        // 机器狗的径向速度 = 人的径向速度 × 系数
        double radial_speed = (human_radial_speed * radial_coeff) / 100.0;
        
        // 检查是否超过最大速度
        if (radial_speed > max_linear_speed_) {
            radial_speed = max_linear_speed_;
            radial_at_max_speed_ = true;
        } else if (radial_at_max_speed_) {
            if (now_distance < FollowConfig::DISTANCE_DECEL_FACTOR * min_distance_) {
                radial_at_max_speed_ = false;
            } else {
                radial_speed = max_linear_speed_;
            }
        }
        
        x_speed = radial_speed;
    } else {
        radial_at_max_speed_ = false;
    }
    
    // ============== 角向运动控制 ==============

    if (angle_diff > FollowConfig::MIN_ANGLE) {
        // 计算二次函数系数
        double angular_coeff = FollowConfig::ANGULAR_COEFF_A * angle_diff * angle_diff
                             + FollowConfig::ANGULAR_COEFF_C;
        
        // 机器狗的角速度 = 人的角速度 × 系数
        double angular_speed_cmd = human_angular_speed * angular_coeff;
        
        // 转换为rad/s
        angular_speed_cmd = angular_speed_cmd / 57.3;
        
        // 检查是否超过最大角速度
        if (angular_speed_cmd > max_angular_speed_) {
            angular_speed_cmd = max_angular_speed_;
            angular_at_max_speed_ = true;
        } else if (angular_at_max_speed_) {
            if (angle_diff < FollowConfig::ANGLE_DECEL_FACTOR * FollowConfig::MIN_ANGLE) {
                angular_at_max_speed_ = false;
            } else {
                angular_speed_cmd = max_angular_speed_;
            }
        }
        
        angular_speed = angular_speed_cmd * angle_sign;
    } else {
        angular_at_max_speed_ = false;
    }
    
    // ============== 速度平滑处理 ==============
    
    if (first_frame_) {
        prev_radial_speed_ = x_speed;
        prev_angular_speed_ = angular_speed;
        first_frame_ = false;
    } else {
        x_speed = FollowConfig::RADIAL_SMOOTH_ALPHA * x_speed 
                + (1.0 - FollowConfig::RADIAL_SMOOTH_ALPHA) * prev_radial_speed_;
        angular_speed = FollowConfig::ANGULAR_SMOOTH_ALPHA * angular_speed 
                      + (1.0 - FollowConfig::ANGULAR_SMOOTH_ALPHA) * prev_angular_speed_;
        
        prev_radial_speed_ = x_speed;
        prev_angular_speed_ = angular_speed;
    }
    
    // 输出结果
    result.vx = x_speed;
    result.vy = 0.0;  // 侧向速度暂不使用
    result.omega = angular_speed;
    result.is_valid = true;
    result.is_following = is_following_;
    
    if (debug_mode_) {
        printf("[调试] 距离=%.0fcm 角度=%.1f° | 速度: vx=%.2f vy=%.2f omega=%.2f\n",
               now_distance, now_angle, result.vx, result.vy, result.omega);
    }
    
    return result;
}

void DogController::reset() {
    parser_.reset();
    processor_.reset();
    has_raw_data_ = false;
    lost_frame_count_ = 0;
    is_following_ = false;
    radial_at_max_speed_ = false;
    angular_at_max_speed_ = false;
    prev_radial_speed_ = 0.0;
    prev_angular_speed_ = 0.0;
    first_frame_ = true;
    total_frames_ = 0;
    valid_frames_ = 0;
}

bool DogController::isFollowing() const {
    return is_following_;
}

void DogController::setFollowDistance(double distance_cm) {
    min_distance_ = distance_cm;
}

void DogController::setMaxSpeed(double max_linear, double max_angular) {
    max_linear_speed_ = max_linear;
    max_angular_speed_ = max_angular;
}

void DogController::setDebugMode(bool enable) {
    debug_mode_ = enable;
}

int DogController::getTotalFrames() const {
    return total_frames_;
}

int DogController::getValidFrames() const {
    return valid_frames_;
}

int DogController::getFilteredFrames() const {
    return processor_.getFilteredCount();
}

double DogController::clamp(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
