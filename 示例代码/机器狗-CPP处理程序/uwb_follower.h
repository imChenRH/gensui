/**
 * ============================================================================
 * UWB单基站跟随套件 - 机器狗跟随库（头文件）
 * ============================================================================
 * 
 * 文件名：uwb_follower.h
 * 
 * 功能说明：
 *     提供UWB数据接收、解析和滤波处理的封装库
 *     方便机器狗跟随程序调用
 * 
 * 使用方法：
 *     1. 包含头文件: #include "uwb_follower.h"
 *     2. 创建实例: UWBFollower follower;
 *     3. 初始化: follower.init("/dev/ttyUSB0");
 *     4. 循环获取数据: auto data = follower.getData();
 *     5. 使用坐标: data.x, data.y, data.vx, data.vy
 * 
 * 作者：Copilot
 * 日期：2026-01-27
 * ============================================================================
 */

#ifndef UWB_FOLLOWER_H
#define UWB_FOLLOWER_H

#include <cstdint>
#include <string>
#include <deque>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

// ============================================================================
// 配置参数（可根据需要修改）
// ============================================================================

namespace UWBConfig {
    // 物理约束参数
    constexpr double MIN_DISTANCE = 5.0;       // 最小距离 (cm)
    constexpr double MAX_DISTANCE = 5000.0;    // 最大距离 (cm)
    constexpr double MAX_VELOCITY = 300.0;     // 最大速度 (cm/s)
    constexpr double MIN_AZIMUTH = -180.0;     // 最小方位角 (度)
    constexpr double MAX_AZIMUTH = 180.0;      // 最大方位角 (度)
    
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
// 数据结构
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
 * 处理后的2D坐标数据（这是跟随程序主要使用的数据结构）
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

// ============================================================================
// 内部滤波器类（供UWBFollower使用）
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

// ============================================================================
// 主接口类：UWBFollower
// ============================================================================

/**
 * UWB跟随器主类
 * 
 * 使用示例：
 * ```cpp
 * UWBFollower follower;
 * 
 * if (!follower.init("/dev/ttyUSB0")) {
 *     return -1;
 * }
 * 
 * while (true) {
 *     UWB2DData data = follower.getData();
 *     if (data.is_valid) {
 *         // 使用 data.x, data.y 控制机器狗
 *         // 使用 data.vx, data.vy 进行速度前馈
 *     }
 * }
 * ```
 */
class UWBFollower {
public:
    UWBFollower();
    ~UWBFollower();
    
    /**
     * 初始化串口连接
     * @param port 串口设备路径，如 "/dev/ttyUSB0"
     * @param baudrate 波特率，默认115200
     * @return true = 初始化成功
     */
    bool init(const std::string& port, int baudrate = UWBConfig::BAUDRATE);
    
    /**
     * 关闭串口连接
     */
    void close();
    
    /**
     * 检查是否已连接
     */
    bool isConnected() const;
    
    /**
     * 获取处理后的2D坐标数据（主要接口）
     * 
     * 此函数会：
     * 1. 从串口读取数据
     * 2. 解析UWB数据帧
     * 3. 进行四步滤波处理
     * 4. 返回滤波后的X、Y坐标和速度估计
     * 
     * @return 处理后的2D数据，如果无有效数据则is_valid为false
     */
    UWB2DData getData();
    
    /**
     * 获取原始数据（不经过滤波处理）
     * @return 原始UWB数据
     */
    UWBRawData getRawData();
    
    /**
     * 处理外部提供的原始数据
     * 适用于从其他来源获取数据的场景
     * @param raw 原始数据
     * @return 处理后的2D数据
     */
    UWB2DData processRawData(const UWBRawData& raw);
    
    /**
     * 重置所有滤波器状态
     */
    void reset();
    
    // 统计信息
    int getTotalCount() const;
    int getFilteredCount() const;
    int getSuppressedCount() const;

private:
    int serial_fd_;
    UWBFrameParser parser_;
    UWBDataProcessor processor_;
    UWBRawData last_raw_data_;
    bool has_raw_data_;
};

// ============================================================================
// 工具函数
// ============================================================================

/**
 * 字节序转换函数
 */
uint16_t U16HighLowByteSwap(uint16_t value);
uint32_t U32HighLowByteSwap(uint32_t value);

#endif // UWB_FOLLOWER_H
