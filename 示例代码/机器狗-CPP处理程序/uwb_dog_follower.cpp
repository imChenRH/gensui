/**
 * ============================================================================
 * UWB单基站跟随套件 - 机器狗跟随程序（C++版）
 * ============================================================================
 * 
 * 功能说明：
 *     接收UWB基站数据，进行数据滤波处理，输出用于机器狗跟随的X、Y坐标值
 *     
 * 参考资料：
 *     - STM32F1-Usart文件夹（数据帧解析）
 *     - uwb_filtered_visualizer_2d.py（滤波算法）
 * 
 * 坐标转换（忽略仰角，直线距离=平地距离）：
 *     X = Distance × sin(Azimuth)
 *     Y = Distance × cos(Azimuth)
 * 
 * 滤波流程：
 *     Step 1: 物理约束检查 - 范围限制 + 速度限制
 *     Step 2: 角度突变抑制 - 抑制人员遮挡导致的角度突变（阈值10°）
 *     Step 3: 中位数滤波 - 消除尖峰异常值（窗口=5）
 *     Step 4: EKF状态估计 - 位置平滑 + 速度估计
 * 
 * 编译方法（Linux/机器狗平台）：
 *     g++ -o uwb_dog_follower uwb_dog_follower.cpp -std=c++11 -lm
 * 
 * 使用方法：
 *     ./uwb_dog_follower /dev/ttyUSB0
 * 
 * 作者：Copilot
 * 日期：2026-01-27
 * ============================================================================
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include <deque>
#include <algorithm>
#include <chrono>

// Linux串口头文件
#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#endif

// ============================================================================
// 配置参数
// ============================================================================

// 物理约束参数
const double MIN_DISTANCE = 5.0;       // 最小距离 (cm)
const double MAX_DISTANCE = 5000.0;    // 最大距离 (cm)
const double MAX_VELOCITY = 300.0;     // 最大速度 (cm/s)
const double MIN_AZIMUTH = -180.0;     // 最小方位角 (度)
const double MAX_AZIMUTH = 180.0;      // 最大方位角 (度)

// 中位数滤波参数
const int MEDIAN_WINDOW_SIZE = 5;      // 中位数滤波窗口大小

// 角度突变抑制参数
const double ANGLE_THRESHOLD = 10.0;   // 角度突变阈值（度）

// EKF参数
const double EKF_PROCESS_NOISE = 0.5;      // EKF过程噪声
const double EKF_MEASUREMENT_NOISE = 1.0;  // EKF测量噪声

// 串口参数
const int BAUDRATE = 115200;
const int FRAME_SIZE = 37;             // 数据帧长度

// ============================================================================
// 数据结构定义（参考STM32代码）
// ============================================================================

// UWB原始数据结构
struct UWBRawData {
    uint32_t anchor_id;     // 基站ID
    uint32_t tag_id;        // 标签ID
    uint32_t distance_cm;   // 距离 (cm)
    int16_t azimuth_deg;    // 方位角 (度)
    int16_t elevation_deg;  // 仰角 (度)，本程序忽略
    bool is_valid;          // 数据有效标志
};

// 处理后的2D坐标数据
struct UWB2DData {
    double x;               // X坐标 (cm)
    double y;               // Y坐标 (cm)
    double vx;              // X方向速度 (cm/s)
    double vy;              // Y方向速度 (cm/s)
    double distance_cm;     // 距离 (cm)
    double azimuth_deg;     // 方位角 (度)
    bool is_valid;          // 数据有效标志
};

// ============================================================================
// 字节序转换函数（参考STM32代码 dataoperation.c）
// ============================================================================

/**
 * 16位大端序转小端序
 */
uint16_t U16HighLowByteSwap(uint16_t value) {
    return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF);
}

/**
 * 32位大端序转小端序
 */
uint32_t U32HighLowByteSwap(uint32_t value) {
    return ((value & 0x000000FF) << 24) |
           ((value & 0x0000FF00) << 8)  |
           ((value & 0x00FF0000) >> 8)  |
           ((value & 0xFF000000) >> 24);
}

// ============================================================================
// 角度突变抑制器（参考UWB code优化技术）
// ============================================================================

class AngleSmoother {
public:
    /**
     * 角度突变抑制器
     * 
     * 抑制规则（当角度变化超过阈值时）：
     * - αk - αk-1 > threshold:  αexport = αk-1, dexport = dk-1 + 2
     * - αk - αk-1 < -threshold: αexport = αk-1, dexport = dk-1 - 2
     * - |αk - αk-1| ≤ threshold: αexport = αk, dexport = dk
     */
    AngleSmoother(double threshold = ANGLE_THRESHOLD)
        : threshold_(threshold), prev_angle_(0), prev_distance_(0), 
          initialized_(false), suppressed_count_(0) {}
    
    /**
     * 处理角度和距离，抑制角度突变
     * @param angle 当前角度 αk
     * @param distance 当前距离 dk
     * @param out_angle 输出角度 αexport
     * @param out_distance 输出距离 dexport
     */
    void process(double angle, double distance, 
                 double& out_angle, double& out_distance) {
        // 首次数据，直接输出
        if (!initialized_) {
            prev_angle_ = angle;
            prev_distance_ = distance;
            out_angle = angle;
            out_distance = distance;
            initialized_ = true;
            return;
        }
        
        // 计算角度变化
        double delta_angle = angle - prev_angle_;
        
        if (delta_angle > threshold_) {
            // 角度正向突变超过阈值
            out_angle = prev_angle_;
            out_distance = prev_distance_ + 2.0;
            suppressed_count_++;
        } else if (delta_angle < -threshold_) {
            // 角度负向突变超过阈值
            out_angle = prev_angle_;
            out_distance = prev_distance_ - 2.0;
            suppressed_count_++;
        } else {
            // 角度变化在阈值范围内，正常输出
            out_angle = angle;
            out_distance = distance;
        }
        
        // 更新上一时刻的值（使用原始UWB数据）
        prev_angle_ = angle;
        prev_distance_ = distance;
    }
    
    void reset() {
        initialized_ = false;
        suppressed_count_ = 0;
    }
    
    int getSuppressedCount() const { return suppressed_count_; }

private:
    double threshold_;
    double prev_angle_;
    double prev_distance_;
    bool initialized_;
    int suppressed_count_;
};

// ============================================================================
// 中位数滤波器
// ============================================================================

class MedianFilter {
public:
    MedianFilter(int window_size = MEDIAN_WINDOW_SIZE)
        : window_size_(window_size) {}
    
    /**
     * 更新并返回滤波值
     */
    double update(double value) {
        buffer_.push_back(value);
        if (buffer_.size() > (size_t)window_size_) {
            buffer_.pop_front();
        }
        
        if (buffer_.size() < (size_t)window_size_) {
            return value;
        }
        
        // 计算中位数
        std::vector<double> sorted_values(buffer_.begin(), buffer_.end());
        std::sort(sorted_values.begin(), sorted_values.end());
        return sorted_values[sorted_values.size() / 2];
    }
    
    void reset() {
        buffer_.clear();
    }

private:
    int window_size_;
    std::deque<double> buffer_;
};

// ============================================================================
// 二维中位数滤波器
// ============================================================================

class MedianFilter2D {
public:
    MedianFilter2D(int window_size = MEDIAN_WINDOW_SIZE)
        : filter_x_(window_size), filter_y_(window_size) {}
    
    void update(double x, double y, double& out_x, double& out_y) {
        out_x = filter_x_.update(x);
        out_y = filter_y_.update(y);
    }
    
    void reset() {
        filter_x_.reset();
        filter_y_.reset();
    }

private:
    MedianFilter filter_x_;
    MedianFilter filter_y_;
};

// ============================================================================
// 简化版2D扩展卡尔曼滤波器 (EKF 2D)
// ============================================================================

class SimpleEKF2D {
public:
    /**
     * 二维EKF
     * 状态向量: [x, y, vx, vy]
     * 使用恒速运动模型
     */
    SimpleEKF2D(double process_noise = EKF_PROCESS_NOISE,
                double measurement_noise = EKF_MEASUREMENT_NOISE)
        : process_noise_(process_noise), measurement_noise_(measurement_noise),
          initialized_(false), last_time_ms_(0) {
        // 初始化状态向量 [x, y, vx, vy]
        x_[0] = x_[1] = x_[2] = x_[3] = 0.0;
        
        // 初始化协方差矩阵（对角线100）
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P_[i][j] = (i == j) ? 100.0 : 0.0;
            }
        }
    }
    
    /**
     * 更新EKF状态
     * @param x 测量的X坐标
     * @param y 测量的Y坐标
     * @param out_x 滤波后X坐标
     * @param out_y 滤波后Y坐标
     * @param out_vx 估计的X速度
     * @param out_vy 估计的Y速度
     */
    void update(double x, double y, 
                double& out_x, double& out_y,
                double& out_vx, double& out_vy) {
        
        uint64_t current_time = getCurrentTimeMs();
        
        // 初始化
        if (!initialized_) {
            x_[0] = x;
            x_[1] = y;
            x_[2] = 0.0;  // vx
            x_[3] = 0.0;  // vy
            last_time_ms_ = current_time;
            initialized_ = true;
            
            out_x = x;
            out_y = y;
            out_vx = 0.0;
            out_vy = 0.0;
            return;
        }
        
        // 计算时间间隔（秒）
        double dt = (current_time - last_time_ms_) / 1000.0;
        if (dt <= 0) dt = 0.1;
        if (dt > 1.0) dt = 1.0;  // 限制最大时间间隔
        last_time_ms_ = current_time;
        
        // === 预测步骤 ===
        // 状态预测: x_pred = F * x
        double x_pred[4];
        x_pred[0] = x_[0] + x_[2] * dt;  // x + vx*dt
        x_pred[1] = x_[1] + x_[3] * dt;  // y + vy*dt
        x_pred[2] = x_[2];               // vx
        x_pred[3] = x_[3];               // vy
        
        // 状态转移矩阵 F
        double F[4][4] = {
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };
        
        // P_pred = F * P * F^T + Q
        double P_pred[4][4];
        double temp[4][4];
        
        // temp = F * P
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                temp[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    temp[i][j] += F[i][k] * P_[k][j];
                }
            }
        }
        
        // P_pred = temp * F^T + Q
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                P_pred[i][j] = process_noise_ * ((i == j) ? 1.0 : 0.0);
                // 速度分量噪声更大
                if (i >= 2 && j >= 2 && i == j) {
                    P_pred[i][j] *= 2.0;
                }
                for (int k = 0; k < 4; k++) {
                    P_pred[i][j] += temp[i][k] * F[j][k];  // F^T
                }
            }
        }
        
        // === 更新步骤 ===
        // 测量矩阵 H = [1,0,0,0; 0,1,0,0]
        // 残差 y = z - H * x_pred
        double z[2] = {x, y};
        double y_residual[2];
        y_residual[0] = z[0] - x_pred[0];
        y_residual[1] = z[1] - x_pred[1];
        
        // S = H * P_pred * H^T + R
        double S[2][2];
        S[0][0] = P_pred[0][0] + measurement_noise_;
        S[0][1] = P_pred[0][1];
        S[1][0] = P_pred[1][0];
        S[1][1] = P_pred[1][1] + measurement_noise_;
        
        // K = P_pred * H^T * S^(-1)
        // 2x2矩阵求逆
        double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        if (std::abs(det) < 1e-10) det = 1e-10;
        
        double S_inv[2][2];
        S_inv[0][0] = S[1][1] / det;
        S_inv[0][1] = -S[0][1] / det;
        S_inv[1][0] = -S[1][0] / det;
        S_inv[1][1] = S[0][0] / det;
        
        // K = P_pred * H^T * S_inv (H^T的前两行是单位矩阵，其余为0)
        double K[4][2];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                K[i][j] = 0;
                for (int k = 0; k < 2; k++) {
                    K[i][j] += P_pred[i][k] * S_inv[k][j];
                }
            }
        }
        
        // 状态更新: x = x_pred + K * y
        for (int i = 0; i < 4; i++) {
            x_[i] = x_pred[i] + K[i][0] * y_residual[0] + K[i][1] * y_residual[1];
        }
        
        // 协方差更新: P = (I - K*H) * P_pred
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
        
        // 输出结果
        out_x = x_[0];
        out_y = x_[1];
        out_vx = x_[2];
        out_vy = x_[3];
    }
    
    void reset() {
        initialized_ = false;
    }
    
private:
    uint64_t getCurrentTimeMs() {
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch());
        return ms.count();
    }
    
    double process_noise_;
    double measurement_noise_;
    bool initialized_;
    uint64_t last_time_ms_;
    double x_[4];       // 状态向量 [x, y, vx, vy]
    double P_[4][4];    // 协方差矩阵
};

// ============================================================================
// 物理约束检查器
// ============================================================================

class PhysicalConstraintChecker {
public:
    PhysicalConstraintChecker(double max_velocity = MAX_VELOCITY)
        : max_velocity_(max_velocity), initialized_(false), 
          last_x_(0), last_y_(0), last_time_ms_(0) {}
    
    /**
     * 检查数据是否满足物理约束
     * @return true = 数据有效, false = 数据异常
     */
    bool check(double x, double y, double distance, double azimuth) {
        // 检查距离范围
        if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
            return false;
        }
        
        // 检查角度范围
        if (azimuth < MIN_AZIMUTH || azimuth > MAX_AZIMUTH) {
            return false;
        }
        
        // 检查速度约束
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
        
        // 更新上一次位置
        last_x_ = x;
        last_y_ = y;
        last_time_ms_ = getCurrentTimeMs();
        initialized_ = true;
        
        return true;
    }
    
    void reset() {
        initialized_ = false;
    }
    
private:
    uint64_t getCurrentTimeMs() {
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch());
        return ms.count();
    }
    
    double max_velocity_;
    bool initialized_;
    double last_x_;
    double last_y_;
    uint64_t last_time_ms_;
};

// ============================================================================
// UWB数据处理器（整合所有滤波器）
// ============================================================================

class UWBDataProcessor {
public:
    UWBDataProcessor() : total_count_(0), filtered_count_(0) {}
    
    /**
     * 处理UWB原始数据，输出滤波后的2D坐标
     * @param raw 原始数据
     * @return 处理后的2D数据
     */
    UWB2DData process(const UWBRawData& raw) {
        UWB2DData result;
        result.is_valid = false;
        
        if (!raw.is_valid) {
            return result;
        }
        
        total_count_++;
        
        double distance = static_cast<double>(raw.distance_cm);
        double azimuth = static_cast<double>(raw.azimuth_deg);
        
        // Step 1: 物理约束检查（初步检查）
        if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
            return result;
        }
        if (azimuth < MIN_AZIMUTH || azimuth > MAX_AZIMUTH) {
            return result;
        }
        
        // Step 2: 角度突变抑制
        double smoothed_azimuth, smoothed_distance;
        angle_smoother_.process(azimuth, distance, smoothed_azimuth, smoothed_distance);
        
        // 坐标转换（忽略仰角，直线距离=平地距离）
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
        
        // 填充结果
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
    
    void reset() {
        angle_smoother_.reset();
        constraint_checker_.reset();
        median_filter_.reset();
        ekf_.reset();
        total_count_ = 0;
        filtered_count_ = 0;
    }
    
    int getTotalCount() const { return total_count_; }
    int getFilteredCount() const { return filtered_count_; }
    int getSuppressedCount() const { return angle_smoother_.getSuppressedCount(); }
    
private:
    AngleSmoother angle_smoother_;
    PhysicalConstraintChecker constraint_checker_;
    MedianFilter2D median_filter_;
    SimpleEKF2D ekf_;
    int total_count_;
    int filtered_count_;
};

// ============================================================================
// UWB数据帧解析器（参考STM32代码）
// ============================================================================

class UWBFrameParser {
public:
    static const uint16_t CMD_LOCATION = 0x2001;
    
    UWBFrameParser() : buffer_index_(0) {}
    
    /**
     * 喂入一个字节数据
     * @param byte 接收到的字节
     * @param out_data 解析成功时输出原始数据
     * @return true = 解析成功，out_data有效
     */
    bool feedByte(uint8_t byte, UWBRawData& out_data) {
        buffer_[buffer_index_++] = byte;
        
        // 缓冲区溢出保护
        if (buffer_index_ >= sizeof(buffer_)) {
            buffer_index_ = 0;
            return false;
        }
        
        // 检查帧头（4个连续0xFF）
        if (buffer_index_ >= 4) {
            bool header_found = true;
            for (int i = 0; i < 4; i++) {
                if (buffer_[buffer_index_ - 4 + i] != 0xFF) {
                    header_found = false;
                    break;
                }
            }
            
            if (header_found && buffer_index_ == 4) {
                // 帧头找到，继续接收
            } else if (!header_found && buffer_index_ < 6) {
                // 还没收到帧头，重新查找
                // 保留最后3字节，可能是帧头的一部分
                if (buffer_index_ > 3) {
                    memmove(buffer_, buffer_ + buffer_index_ - 3, 3);
                    buffer_index_ = 3;
                }
            }
        }
        
        // 检查是否收到足够的数据（37字节）
        if (buffer_index_ >= FRAME_SIZE) {
            // 验证帧头
            if (buffer_[0] == 0xFF && buffer_[1] == 0xFF && 
                buffer_[2] == 0xFF && buffer_[3] == 0xFF) {
                
                // 验证校验和
                uint8_t xor_check = 0;
                for (int i = 0; i < FRAME_SIZE - 1; i++) {
                    xor_check ^= buffer_[i];
                }
                
                if (xor_check == buffer_[FRAME_SIZE - 1]) {
                    // 解析命令码
                    uint16_t cmd = (buffer_[8] << 8) | buffer_[9];
                    
                    if (cmd == CMD_LOCATION) {
                        // 解析数据字段（参考STM32代码）
                        out_data.anchor_id = U32HighLowByteSwap(
                            (buffer_[12] << 24) | (buffer_[13] << 16) |
                            (buffer_[14] << 8) | buffer_[15]);
                        
                        out_data.tag_id = U32HighLowByteSwap(
                            (buffer_[16] << 24) | (buffer_[17] << 16) |
                            (buffer_[18] << 8) | buffer_[19]);
                        
                        out_data.distance_cm = U32HighLowByteSwap(
                            (buffer_[20] << 24) | (buffer_[21] << 16) |
                            (buffer_[22] << 8) | buffer_[23]);
                        
                        out_data.azimuth_deg = static_cast<int16_t>(
                            U16HighLowByteSwap((buffer_[24] << 8) | buffer_[25]));
                        
                        out_data.elevation_deg = static_cast<int16_t>(
                            U16HighLowByteSwap((buffer_[26] << 8) | buffer_[27]));
                        
                        out_data.is_valid = true;
                        
                        buffer_index_ = 0;
                        return true;
                    }
                }
            }
            
            // 解析失败，移除第一个字节继续查找
            memmove(buffer_, buffer_ + 1, buffer_index_ - 1);
            buffer_index_--;
        }
        
        out_data.is_valid = false;
        return false;
    }
    
    void reset() {
        buffer_index_ = 0;
    }
    
private:
    uint8_t buffer_[128];
    int buffer_index_;
};

// ============================================================================
// Linux串口类
// ============================================================================

#ifdef __linux__
class LinuxSerial {
public:
    LinuxSerial() : fd_(-1) {}
    
    ~LinuxSerial() {
        close();
    }
    
    /**
     * 打开串口
     * @param port 串口设备，如 "/dev/ttyUSB0"
     * @param baudrate 波特率
     * @return true = 成功
     */
    bool open(const char* port, int baudrate = BAUDRATE) {
        fd_ = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            std::cerr << "错误：无法打开串口 " << port 
                      << " (" << strerror(errno) << ")" << std::endl;
            return false;
        }
        
        // 配置串口参数
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(fd_, &tty) != 0) {
            std::cerr << "错误：无法获取串口属性" << std::endl;
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        // 设置波特率
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
        
        // 8N1: 8数据位，无校验，1停止位
        tty.c_cflag &= ~PARENB;   // 无校验
        tty.c_cflag &= ~CSTOPB;   // 1停止位
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;       // 8数据位
        tty.c_cflag &= ~CRTSCTS;  // 无硬件流控
        tty.c_cflag |= CREAD | CLOCAL;  // 启用接收
        
        // 原始模式
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | 
                         INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        
        // 非阻塞读取
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            std::cerr << "错误：无法设置串口属性" << std::endl;
            ::close(fd_);
            fd_ = -1;
            return false;
        }
        
        std::cout << "✓ 串口 " << port << " 已打开" << std::endl;
        std::cout << "  波特率: " << baudrate << std::endl;
        return true;
    }
    
    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
    
    bool isOpen() const { return fd_ >= 0; }
    
    /**
     * 读取数据
     * @return 读取到的字节数，-1表示错误
     */
    int read(uint8_t* buffer, int max_len) {
        if (fd_ < 0) return -1;
        return ::read(fd_, buffer, max_len);
    }
    
private:
    int fd_;
};
#endif

// ============================================================================
// 主程序
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "============================================" << std::endl;
    std::cout << "  UWB机器狗跟随程序 - 数据处理模块" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << std::endl;
    std::cout << "滤波流程：" << std::endl;
    std::cout << "  1. 物理约束检查 (距离:" << MIN_DISTANCE << "-" 
              << MAX_DISTANCE << "cm, 速度<" << MAX_VELOCITY << "cm/s)" << std::endl;
    std::cout << "  2. 角度突变抑制 (阈值:" << ANGLE_THRESHOLD << "°)" << std::endl;
    std::cout << "  3. 中位数滤波 (窗口:" << MEDIAN_WINDOW_SIZE << ")" << std::endl;
    std::cout << "  4. EKF状态估计" << std::endl;
    std::cout << std::endl;
    
#ifdef __linux__
    // Linux平台 - 使用串口
    if (argc < 2) {
        std::cout << "用法: " << argv[0] << " <串口设备>" << std::endl;
        std::cout << "示例: " << argv[0] << " /dev/ttyUSB0" << std::endl;
        return 1;
    }
    
    const char* port = argv[1];
    
    LinuxSerial serial;
    if (!serial.open(port)) {
        return 1;
    }
    
    UWBFrameParser parser;
    UWBDataProcessor processor;
    
    std::cout << std::endl;
    std::cout << "开始接收数据..." << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
    std::cout << std::endl;
    
    uint8_t buffer[128];
    UWBRawData raw_data;
    
    while (true) {
        int n = serial.read(buffer, sizeof(buffer));
        
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                if (parser.feedByte(buffer[i], raw_data)) {
                    // 解析成功，处理数据
                    UWB2DData result = processor.process(raw_data);
                    
                    if (result.is_valid) {
                        // 输出处理后的X、Y值
                        std::cout << "【滤波后数据】"
                                  << " X=" << std::fixed << std::setprecision(1) << result.x
                                  << "cm, Y=" << result.y
                                  << "cm | Vx=" << std::setprecision(1) << result.vx
                                  << ", Vy=" << result.vy
                                  << "cm/s | 距离=" << (int)result.distance_cm
                                  << "cm, 角度=" << (int)result.azimuth_deg << "°"
                                  << std::endl;
                    }
                }
            }
        }
        
        // 小延时，减少CPU占用
        usleep(5000);  // 5ms
    }
    
#else
    // 非Linux平台（演示模式）
    std::cout << "演示模式（非Linux平台）" << std::endl;
    std::cout << std::endl;
    
    UWBDataProcessor processor;
    UWBRawData raw_data;
    
    // 模拟数据测试
    double test_angles[] = {0, 5, 10, 15, 50, 16, 17, 18, -30, 19, 20};
    double test_distances[] = {100, 102, 105, 108, 150, 110, 112, 115, 200, 118, 120};
    
    for (int i = 0; i < 11; i++) {
        raw_data.anchor_id = 0xAAA2;
        raw_data.tag_id = 0xAAA1;
        raw_data.distance_cm = static_cast<uint32_t>(test_distances[i]);
        raw_data.azimuth_deg = static_cast<int16_t>(test_angles[i]);
        raw_data.elevation_deg = 0;
        raw_data.is_valid = true;
        
        UWB2DData result = processor.process(raw_data);
        
        std::cout << "输入: 距离=" << raw_data.distance_cm 
                  << "cm, 角度=" << raw_data.azimuth_deg << "° -> ";
        
        if (result.is_valid) {
            std::cout << "输出: X=" << std::fixed << std::setprecision(1) << result.x
                      << "cm, Y=" << result.y << "cm" << std::endl;
        } else {
            std::cout << "数据被过滤" << std::endl;
        }
    }
    
    std::cout << std::endl;
    std::cout << "统计: 总数=" << processor.getTotalCount()
              << ", 有效=" << processor.getFilteredCount()
              << ", 角度突变抑制=" << processor.getSuppressedCount() << std::endl;
#endif
    
    return 0;
}
