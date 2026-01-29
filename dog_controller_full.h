/**
 * 功能说明：
 *     整合 UWB数据接收、协议解析、滤波处理 和 跟随控制 的一体化库
 *     只需包含此头文件和对应的cpp文件，即可获取机器狗的控制速度
 *     无需其他依赖
 * 
 * 使用方法：
 *     #include "dog_controller_full.h"
 *     
 *     DogController dog;
 *     dog.init("/dev/ttyUSB0");
 *     
 *     while (running) {
 *         DogVelocity vel = dog.getVelocity();
 *         if (vel.is_valid) {
 *             your_robot.move(vel.vx, vel.vy, vel.omega);
 *         }
 *     }
 * 
 * 编译方法：
 *     g++ your_main.cpp dog_controller_full.cpp -o your_program -std=c++11 -lm
 */

#ifndef DOG_CONTROLLER_FULL_H
#define DOG_CONTROLLER_FULL_H

#include <cstdint>
#include <string>
#include <deque>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <iostream>

// ============================================================================
// UWB配置参数
// ============================================================================

namespace UWBConfig {
    // 物理约束参数
    constexpr double MIN_DISTANCE = 5.0;       // 最小距离 (cm)
    constexpr double MAX_DISTANCE = 5000.0;    // 最大距离 (cm)
    constexpr double MAX_VELOCITY = 600.0;     // 最大速度 (cm/s)
    constexpr double MIN_AZIMUTH = 20.0;       // 最小方位角 (度)
    constexpr double MAX_AZIMUTH = 160.0;      // 最大方位角 (度)
    
    // 中位数滤波参数
    constexpr int MEDIAN_WINDOW_SIZE = 5;      // 中位数滤波窗口大小
    
    // 角度突变抑制参数
    constexpr double ANGLE_THRESHOLD = 10.0;   // 角度突变阈值（度）
    
    // EKF参数
    constexpr double EKF_PROCESS_NOISE = 0.5;      // EKF过程噪声
    constexpr double EKF_MEASUREMENT_NOISE = 1.0;  // EKF测量噪声
    
    // 串口参数
    constexpr int BAUDRATE = 115200;
    constexpr int FRAME_SIZE = 37;             // 数据帧长度
}

// ============================================================================
// 跟随控制参数
// ============================================================================

namespace FollowConfig {
    // 跟随距离参数
    constexpr double MIN_DISTANCE = 250.0;         // 最小跟随距离 (cm) = 2.5米
    constexpr double DISTANCE_DECEL_FACTOR = 1.8;  // 减速距离系数
    
    // 角度参数
    constexpr double MIN_ANGLE = 1.0;              // 最小启动角度 (度)
    constexpr double TARGET_ANGLE = 90.0;          // 目标角度 (度)
    constexpr double ANGLE_DECEL_FACTOR = 2.0;     // 角度减速系数
    
    // 速度限制
    constexpr double MAX_LINEAR_SPEED = 5.0;       // 最大线速度 (m/s)
    constexpr double MAX_ANGULAR_SPEED = 1.0;      // 最大角速度 (rad/s)
    
    // 人标准速度
    constexpr double HUMAN_STANDARD_SPEED = 1.0;   // 人的标准行走速度 (m/s)
    constexpr double HUMAN_ANGULAR_SPEED = 0.4;    // 人的标准角速度 (rad/s)

    // 二次函数系数
    constexpr double RADIAL_COEFF_A = 0.000016;    // 径向二次系数
    constexpr double RADIAL_COEFF_C = 1.0;         // 径向常数项
    
    constexpr double ANGULAR_COEFF_A = 0.0025;     // 角向二次系数
    constexpr double ANGULAR_COEFF_C = 1.0;        // 角向常数项
    
    // 安全参数
    constexpr double STOP_DISTANCE = 100.0;        // 停止距离 (cm)
    constexpr double LOST_DISTANCE = 1000.0;       // 丢失距离 (cm)
    constexpr int MAX_LOST_FRAMES = 30;            // 最大丢失帧数
    
    // 速度平滑参数
    constexpr double RADIAL_SMOOTH_ALPHA = 0.3;    // 径向速度平滑系数 (0.0-1.0)
    constexpr double ANGULAR_SMOOTH_ALPHA = 0.4;   // 角向速度平滑系数 (0.0-1.0)
}

// ============================================================================
// UWB数据结构
// ============================================================================

/**
 * UWB原始数据结构
 */
struct UWBRawData {
    uint32_t anchor_id;     // 基站ID
    uint32_t tag_id;        // 标签ID
    uint32_t distance_cm;   // 距离 (cm)
    int16_t azimuth_deg;    // 方位角 (度)
    int16_t elevation_deg;  // 仰角 (度)，本程序忽略
    bool is_valid;          // 数据有效标志
    
    UWBRawData() : anchor_id(0), tag_id(0), distance_cm(0), 
                  azimuth_deg(0), elevation_deg(0), is_valid(false) {}
};

/**
 * 处理后的2D坐标数据
 */
struct UWB2DData {
    double x;               // X坐标 (cm) - 左右方向
    double y;               // Y坐标 (cm) - 前后方向
    double vx;              // X方向速度 (cm/s)
    double vy;              // Y方向速度 (cm/s)
    double distance_cm;     // 距离 (cm)
    double azimuth_deg;     // 方位角 (度)
    bool is_valid;          // 数据有效标志
    
    UWB2DData() : x(0), y(0), vx(0), vy(0), 
                 distance_cm(0), azimuth_deg(0), is_valid(false) {}
};

/**
 * 机器狗速度结构体 - 跟随程序的主要输出
 */
struct DogVelocity {
    double vx;          // X方向速度 (m/s) - 前进方向
    double vy;          // Y方向速度 (m/s) - 侧向（通常为0）
    double omega;       // 角速度 (rad/s) - 旋转
    bool is_valid;      // 数据有效标志
    bool is_following;  // 是否正在跟随
    
    // 额外信息（调试用）
    double distance_cm;     // 当前距离
    double azimuth_deg;     // 当前角度
    double human_vx;        // 人的X速度
    double human_vy;        // 人的Y速度
    
    DogVelocity() : vx(0), vy(0), omega(0), is_valid(false), is_following(false),
                   distance_cm(0), azimuth_deg(0), human_vx(0), human_vy(0) {}
};

// ============================================================================
// 滤波器类
// ============================================================================

/**
 * 角度突变抑制器
 */
class AngleSmoother {
public:
    AngleSmoother(double threshold = UWBConfig::ANGLE_THRESHOLD);
    void process(double angle, double distance, double& out_angle, double& out_distance);
    void reset();
    int getSuppressedCount() const { return suppressed_count_; }

private:
    double threshold_;
    double prev_angle_;
    double prev_distance_;
    bool initialized_;
    int suppressed_count_;
};

/**
 * 中位数滤波器
 */
class MedianFilter {
public:
    MedianFilter(int window_size = UWBConfig::MEDIAN_WINDOW_SIZE);
    double update(double value);
    void reset();

private:
    int window_size_;
    std::deque<double> buffer_;
};

/**
 * 2D中位数滤波器
 */
class MedianFilter2D {
public:
    MedianFilter2D(int window_size = UWBConfig::MEDIAN_WINDOW_SIZE);
    void update(double x, double y, double& out_x, double& out_y);
    void reset();

private:
    MedianFilter filter_x_;
    MedianFilter filter_y_;
};

/**
 * 简化版2D扩展卡尔曼滤波器
 */
class SimpleEKF2D {
public:
    SimpleEKF2D(double process_noise = UWBConfig::EKF_PROCESS_NOISE,
                double measurement_noise = UWBConfig::EKF_MEASUREMENT_NOISE);
    void update(double x, double y, double& out_x, double& out_y, double& out_vx, double& out_vy);
    void reset();

private:
    uint64_t getCurrentTimeMs();
    
    double process_noise_;
    double measurement_noise_;
    bool initialized_;
    uint64_t last_time_ms_;
    double x_[4];       // 状态向量 [x, y, vx, vy]
    double P_[4][4];    // 协方差矩阵
};

/**
 * 物理约束检查器
 */
class PhysicalConstraintChecker {
public:
    PhysicalConstraintChecker(double max_velocity = UWBConfig::MAX_VELOCITY);
    bool check(double x, double y, double distance, double azimuth);
    void reset();

private:
    uint64_t getCurrentTimeMs();
    
    double max_velocity_;
    bool initialized_;
    double last_x_;
    double last_y_;
    uint64_t last_time_ms_;
};

/**
 * UWB数据帧解析器
 */
class UWBFrameParser {
public:
    static const uint16_t CMD_LOCATION = 0x2001;
    
    UWBFrameParser();
    bool feedByte(uint8_t byte, UWBRawData& out_data);
    void reset();

private:
    uint8_t buffer_[128];
    int buffer_index_;
};

/**
 * UWB数据处理器（整合所有滤波器）
 */
class UWBDataProcessor {
public:
    UWBDataProcessor();
    UWB2DData process(const UWBRawData& raw);
    void reset();
    
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

class DogController {
public:
    DogController();
    ~DogController();
    
    /**
     * 初始化控制器
     * @param port 串口设备路径，如 "/dev/ttyUSB0"
     * @param baudrate 波特率，默认115200
     * @return true = 初始化成功
     */
    bool init(const std::string& port, int baudrate = UWBConfig::BAUDRATE);
    
    /**
     * 关闭控制器
     */
    void close();
    
    /**
     * 检查是否已连接
     */
    bool isConnected() const;
    
    /**
     * 获取机器狗速度指令（主要接口）
     * 
     * 此函数会：
     * 1. 从UWB获取并滤波处理位置数据
     * 2. 计算跟随控制指令
     * 3. 返回平滑后的速度指令
     * 
     * @return 机器狗速度指令，如果无有效数据则is_valid为false
     */
    DogVelocity getVelocity();
    
    /**
     * 获取原始UWB数据
     * @return 处理后的2D数据
     */
    UWB2DData getUWBData();
    
    /**
     * 使用外部UWB数据计算速度
     * @param uwb_data UWB位置数据
     * @return 机器狗速度指令
     */
    DogVelocity computeVelocity(const UWB2DData& uwb_data);
    
    /**
     * 重置控制器状态
     */
    void reset();
    
    /**
     * 是否正在跟随
     */
    bool isFollowing() const;
    
    /**
     * 设置跟随距离
     * @param distance_cm 目标跟随距离 (cm)
     */
    void setFollowDistance(double distance_cm);
    
    /**
     * 设置最大速度
     * @param max_linear 最大线速度 (m/s)
     * @param max_angular 最大角速度 (rad/s)
     */
    void setMaxSpeed(double max_linear, double max_angular);
    
    /**
     * 开启/关闭调试输出
     */
    void setDebugMode(bool enable);
    
    // 获取统计信息
    int getTotalFrames() const;
    int getValidFrames() const;
    int getFilteredFrames() const;

private:
    // UWB接收相关
    int serial_fd_;
    UWBFrameParser parser_;
    UWBDataProcessor processor_;
    UWBRawData last_raw_data_;
    bool has_raw_data_;
    
    // 从串口读取并处理数据
    UWB2DData readUWBData();
    
    // 控制参数
    double min_distance_;
    double max_linear_speed_;
    double max_angular_speed_;
    
    // 状态变量
    int lost_frame_count_;
    bool is_following_;
    bool radial_at_max_speed_;
    bool angular_at_max_speed_;
    
    // 速度平滑变量
    double prev_radial_speed_;
    double prev_angular_speed_;
    bool first_frame_;
    
    // 统计变量
    int total_frames_;
    int valid_frames_;
    
    // 调试模式
    bool debug_mode_;
    
    // 工具函数
    static double clamp(double value, double min_val, double max_val);
};

#endif // DOG_CONTROLLER_FULL_H
