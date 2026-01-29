/**
 * ============================================================================
 * UWB单基站跟随套件 - 机器狗跟随控制库（整合版）
 * ============================================================================
 * 
 * 文件名：dog_controller.h
 * 
 * 功能说明：
 *     整合 UWB数据接收、滤波处理 和 跟随控制 的一体化库
 *     只需包含此头文件，即可获取机器狗的控制速度
 * 
 * 使用方法：
 *     #include "dog_controller.h"
 *     
 *     DogController dog;
 *     dog.init("/dev/ttyUSB0");
 *     
 *     while (running) {
 *         DogVelocity vel = dog.getVelocity();
 *         if (vel.is_valid) {
 *             // 使用 vel.vx, vel.vy, vel.omega 控制机器狗
 *             your_robot.move(vel.vx, vel.vy, vel.omega);
 *         }
 *     }
 * 
 * ============================================================================
 */

#ifndef DOG_CONTROLLER_H
#define DOG_CONTROLLER_H

#include "uwb_follower.h"
#include <iostream>

// ============================================================================
// 跟随控制参数（可根据实际情况调整）
// ============================================================================

namespace FollowConfig {
    // 跟随距离参数
    constexpr double MIN_DISTANCE = 250.0;         // 最小跟随距离 (cm) = 2.5米
    constexpr double DISTANCE_DECEL_FACTOR = 1.8;  // 减速距离系数
    
    // 角度参数
    constexpr double MIN_ANGLE = 3.5;              // 最小启动角度 (度)
    constexpr double TARGET_ANGLE = 90.0;          // 目标角度 (度)
    constexpr double ANGLE_DECEL_FACTOR = 1.1;     // 角度减速系数
    
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
    constexpr double RADIAL_SMOOTH_ALPHA = 0.3;    // 径向速度平滑系数
    constexpr double ANGULAR_SMOOTH_ALPHA = 0.4;   // 角向速度平滑系数
}

// ============================================================================
// 数据结构
// ============================================================================

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
// 机器狗跟随控制器类
// ============================================================================

/**
 * 机器狗跟随控制器
 * 
 * 整合了：
 * - UWB数据接收
 * - 四步滤波处理（物理约束→角度突变抑制→中位数滤波→EKF）
 * - 跟随控制算法
 * 
 * 输出机器狗的速度指令：vx, vy, omega
 */
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
     * 使用外部UWB数据计算速度
     * 适用于从其他来源获取UWB数据的场景
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
    
    // 获取UWB原始数据（调试用）
    UWB2DData getUWBData();

private:
    /**
     * 数值限幅
     */
    static double clamp(double value, double min_val, double max_val);
    
    UWBFollower uwb_follower_;
    
    // 控制参数（可运行时修改）
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
};

#endif // DOG_CONTROLLER_H
