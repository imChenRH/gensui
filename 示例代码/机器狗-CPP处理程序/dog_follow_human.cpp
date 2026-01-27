/**
 * ============================================================================
 * UWB单基站跟随套件 - 机器狗跟随人程序
 * ============================================================================
 * 
 * 文件名：dog_follow_human.cpp
 * 
 * 功能说明：
 *     使用UWB定位系统实现机器狗跟随人的功能
 *     - 保持约2.5米的跟随距离
 *     - 始终朝向目标人
 *     - 平滑的速度控制
 * 
 * 使用方法：
 *     1. 将此文件与 uwb_follower.cpp 一起编译
 *     2. 运行程序，指定UWB基站串口
 * 
 * 编译命令：
 *     g++ dog_follow_human.cpp uwb_follower.cpp -o dog_follow_human -std=c++11 -lm
 * 
 * 运行命令：
 *     ./dog_follow_human /dev/ttyUSB0
 * 
 * 作者：Copilot
 * 日期：2026-01-27
 * ============================================================================
 */

#include <iostream>
#include <cmath>
#include <csignal>
#include <unistd.h>
#include "uwb_follower.h"

// ============================================================================
// 跟随控制参数（可根据实际情况调整）
// ============================================================================

namespace FollowConfig {
    // 跟随距离参数
    constexpr double TARGET_DISTANCE = 250.0;      // 目标跟随距离 (cm) = 2.5米
    constexpr double DISTANCE_TOLERANCE = 30.0;    // 距离容差 (cm)，在此范围内不调整距离
    
    // 角度参数
    constexpr double ANGLE_TOLERANCE = 5.0;        // 角度容差 (度)，在此范围内不旋转
    
    // 速度限制
    constexpr double MAX_LINEAR_SPEED = 0.8;       // 最大线速度 (m/s)
    constexpr double MAX_ANGULAR_SPEED = 1.2;      // 最大角速度 (rad/s)
    constexpr double MIN_LINEAR_SPEED = 0.1;       // 最小线速度 (m/s)
    constexpr double MIN_ANGULAR_SPEED = 0.1;      // 最小角速度 (rad/s)
    
    // PID控制参数 - 距离控制
    constexpr double KP_DISTANCE = 0.005;          // 距离比例系数
    constexpr double KD_DISTANCE = 0.001;          // 距离微分系数
    
    // PID控制参数 - 角度控制
    constexpr double KP_ANGLE = 0.03;              // 角度比例系数
    constexpr double KD_ANGLE = 0.005;             // 角度微分系数
    
    // 安全参数
    constexpr double STOP_DISTANCE = 100.0;        // 停止距离 (cm)，人太近时停止
    constexpr double LOST_DISTANCE = 1000.0;       // 丢失距离 (cm)，人太远时停止
    constexpr int MAX_LOST_FRAMES = 30;            // 最大丢失帧数，超过则停止
    
    // 控制频率
    constexpr int CONTROL_PERIOD_MS = 50;          // 控制周期 (ms)
}

// ============================================================================
// 全局变量
// ============================================================================

volatile bool g_running = true;

// ============================================================================
// 信号处理函数
// ============================================================================

void signalHandler(int signum) {
    std::cout << "\n[信息] 收到退出信号，正在停止..." << std::endl;
    g_running = false;
}

// ============================================================================
// 机器狗控制接口（需要根据实际SDK替换）
// ============================================================================

/**
 * 机器狗控制类
 * 注意：这是一个示例接口，需要根据您的机器狗SDK进行替换
 */
class RobotDog {
public:
    /**
     * 初始化机器狗
     * @return true = 初始化成功
     */
    bool init() {
        // TODO: 替换为实际的机器狗SDK初始化代码
        std::cout << "[机器狗] 初始化完成" << std::endl;
        return true;
    }
    
    /**
     * 控制机器狗移动
     * @param x_speed X方向速度 (m/s)，正值=向前，负值=向后
     * @param y_speed Y方向速度 (m/s)，正值=向左，负值=向右
     * @param angular_speed 角速度 (rad/s)，正值=逆时针，负值=顺时针
     */
    void move(double x_speed, double y_speed, double angular_speed) {
        // TODO: 替换为实际的机器狗SDK控制代码
        // 例如：robot_sdk_move(x_speed, y_speed, angular_speed);
        
        // 调试输出（实际使用时可以注释掉）
        // printf("[机器狗] 速度指令: X=%.3f m/s, Y=%.3f m/s, W=%.3f rad/s\n", 
        //        x_speed, y_speed, angular_speed);
    }
    
    /**
     * 停止机器狗
     */
    void stop() {
        move(0.0, 0.0, 0.0);
        std::cout << "[机器狗] 已停止" << std::endl;
    }
    
    /**
     * 关闭机器狗连接
     */
    void close() {
        stop();
        // TODO: 替换为实际的机器狗SDK关闭代码
        std::cout << "[机器狗] 已关闭" << std::endl;
    }
};

// ============================================================================
// 跟随控制器类
// ============================================================================

/**
 * 跟随控制器
 * 实现基于UWB定位的人员跟随算法
 */
class FollowController {
public:
    FollowController() 
        : prev_distance_error_(0.0)
        , prev_angle_error_(0.0)
        , lost_frame_count_(0)
        , is_following_(false) {
    }
    
    /**
     * 计算控制指令
     * @param uwb_data UWB位置数据
     * @param x_speed 输出：X方向速度
     * @param y_speed 输出：Y方向速度  
     * @param angular_speed 输出：角速度
     * @return true = 数据有效，输出控制指令；false = 数据无效，应停止
     */
    bool computeControl(const UWB2DData& uwb_data, 
                        double& x_speed, double& y_speed, double& angular_speed) {
        // 初始化输出
        x_speed = 0.0;
        y_speed = 0.0;
        angular_speed = 0.0;
        
        // 检查数据有效性
        if (!uwb_data.is_valid) {
            lost_frame_count_++;
            if (lost_frame_count_ > FollowConfig::MAX_LOST_FRAMES) {
                if (is_following_) {
                    std::cout << "[跟随] 目标丢失，停止跟随" << std::endl;
                    is_following_ = false;
                }
                return false;
            }
            // 短暂丢失，保持上一次的控制（可以改为减速）
            return true;
        }
        
        // 数据有效，重置丢失计数
        lost_frame_count_ = 0;
        
        // 获取当前距离和角度
        double current_distance = uwb_data.distance_cm;
        double current_angle = uwb_data.azimuth_deg;
        
        // ============== 安全检查 ==============
        
        // 人太近，停止
        if (current_distance < FollowConfig::STOP_DISTANCE) {
            if (is_following_) {
                std::cout << "[跟随] 人太近 (" << current_distance 
                          << "cm)，停止并等待" << std::endl;
            }
            is_following_ = false;
            return true;  // 返回true但速度为0
        }
        
        // 人太远，停止
        if (current_distance > FollowConfig::LOST_DISTANCE) {
            if (is_following_) {
                std::cout << "[跟随] 人太远 (" << current_distance 
                          << "cm)，停止跟随" << std::endl;
            }
            is_following_ = false;
            return false;
        }
        
        // 开始跟随
        if (!is_following_) {
            std::cout << "[跟随] 检测到目标，开始跟随 (距离=" 
                      << current_distance << "cm, 角度=" 
                      << current_angle << "°)" << std::endl;
            is_following_ = true;
        }
        
        // ============== 计算控制量 ==============
        
        // 距离误差（正值=需要前进，负值=需要后退）
        double distance_error = current_distance - FollowConfig::TARGET_DISTANCE;
        
        // 角度误差（正值=人在左边需要左转，负值=人在右边需要右转）
        // 方位角定义：正值=左边，负值=右边
        double angle_error = current_angle;  // 目标是让角度归零（正对人）
        
        // ============== 角度控制（旋转） ==============
        
        // 如果角度偏差超过容差，则旋转
        if (std::abs(angle_error) > FollowConfig::ANGLE_TOLERANCE) {
            // PD控制计算角速度
            double angle_deriv = angle_error - prev_angle_error_;
            angular_speed = FollowConfig::KP_ANGLE * angle_error 
                          + FollowConfig::KD_ANGLE * angle_deriv;
            
            // 限幅
            angular_speed = clamp(angular_speed, 
                                 -FollowConfig::MAX_ANGULAR_SPEED, 
                                  FollowConfig::MAX_ANGULAR_SPEED);
            
            // 添加死区
            if (std::abs(angular_speed) < FollowConfig::MIN_ANGULAR_SPEED) {
                angular_speed = 0.0;
            }
        }
        prev_angle_error_ = angle_error;
        
        // ============== 距离控制（前进/后退） ==============
        
        // 如果距离偏差超过容差，则移动
        if (std::abs(distance_error) > FollowConfig::DISTANCE_TOLERANCE) {
            // PD控制计算线速度
            double distance_deriv = distance_error - prev_distance_error_;
            double forward_speed = FollowConfig::KP_DISTANCE * distance_error 
                                 + FollowConfig::KD_DISTANCE * distance_deriv;
            
            // 限幅
            forward_speed = clamp(forward_speed, 
                                 -FollowConfig::MAX_LINEAR_SPEED, 
                                  FollowConfig::MAX_LINEAR_SPEED);
            
            // 添加死区
            if (std::abs(forward_speed) < FollowConfig::MIN_LINEAR_SPEED) {
                forward_speed = 0.0;
            }
            
            // 如果角度偏差较大，减小前进速度（先转向再前进）
            if (std::abs(angle_error) > 30.0) {
                forward_speed *= 0.3;  // 角度偏差大时，大幅减速
            } else if (std::abs(angle_error) > 15.0) {
                forward_speed *= 0.6;  // 角度偏差中等时，适度减速
            }
            
            // 将前进速度分解为X和Y分量
            // 这里假设机器狗始终面向前方（Y正方向）
            // X速度用于侧移（根据人的位置进行横向调整）
            // Y速度用于前进/后退
            
            // 计算人相对于机器狗的位置（cm转换为m）
            double person_x_m = uwb_data.x / 100.0;  // 左右位置 (m)
            double person_y_m = uwb_data.y / 100.0;  // 前后位置 (m)
            
            // Y方向速度：向人所在的方向移动
            x_speed = forward_speed;
            
            // 如果人在侧面，可以添加侧移以更快接近
            // 侧移速度与人的横向位置成正比
            double lateral_factor = 0.002;  // 侧移比例系数
            y_speed = -lateral_factor * person_x_m;  // 向人的方向侧移
            y_speed = clamp(y_speed, -0.3, 0.3);  // 限制侧移速度
        }
        prev_distance_error_ = distance_error;
        
        return true;
    }
    
    /**
     * 重置控制器状态
     */
    void reset() {
        prev_distance_error_ = 0.0;
        prev_angle_error_ = 0.0;
        lost_frame_count_ = 0;
        is_following_ = false;
    }
    
    /**
     * 是否正在跟随
     */
    bool isFollowing() const {
        return is_following_;
    }

private:
    /**
     * 数值限幅
     */
    static double clamp(double value, double min_val, double max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }
    
    double prev_distance_error_;
    double prev_angle_error_;
    int lost_frame_count_;
    bool is_following_;
};

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char* argv[]) {
    // 检查命令行参数
    if (argc < 2) {
        std::cout << "============================================" << std::endl;
        std::cout << "UWB机器狗跟随程序" << std::endl;
        std::cout << "============================================" << std::endl;
        std::cout << "\n用法: " << argv[0] << " <串口设备>" << std::endl;
        std::cout << "\n示例:" << std::endl;
        std::cout << "  " << argv[0] << " /dev/ttyUSB0" << std::endl;
        std::cout << "\n参数说明:" << std::endl;
        std::cout << "  目标跟随距离: " << FollowConfig::TARGET_DISTANCE << " cm (2.5米)" << std::endl;
        std::cout << "  距离容差: ±" << FollowConfig::DISTANCE_TOLERANCE << " cm" << std::endl;
        std::cout << "  角度容差: ±" << FollowConfig::ANGLE_TOLERANCE << "°" << std::endl;
        std::cout << "  最大线速度: " << FollowConfig::MAX_LINEAR_SPEED << " m/s" << std::endl;
        std::cout << "  最大角速度: " << FollowConfig::MAX_ANGULAR_SPEED << " rad/s" << std::endl;
        return 1;
    }
    
    std::string port = argv[1];
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "============================================" << std::endl;
    std::cout << "UWB机器狗跟随程序 启动" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "串口设备: " << port << std::endl;
    std::cout << "目标距离: " << FollowConfig::TARGET_DISTANCE << " cm" << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
    std::cout << "============================================\n" << std::endl;
    
    // 初始化UWB跟随器
    UWBFollower uwb_follower;
    if (!uwb_follower.init(port)) {
        std::cerr << "[错误] 无法打开串口: " << port << std::endl;
        return 1;
    }
    std::cout << "[信息] UWB串口已连接" << std::endl;
    
    // 初始化机器狗
    RobotDog dog;
    if (!dog.init()) {
        std::cerr << "[错误] 机器狗初始化失败" << std::endl;
        uwb_follower.close();
        return 1;
    }
    
    // 创建跟随控制器
    FollowController controller;
    
    // 控制变量
    double x_speed = 0.0;
    double y_speed = 0.0;
    double angular_speed = 0.0;
    
    int frame_count = 0;
    
    // ============== 主控制循环 ==============
    
    std::cout << "[信息] 开始跟随控制循环..." << std::endl;
    
    while (g_running) {
        // 获取UWB数据
        UWB2DData uwb_data = uwb_follower.getData();
        
        // 计算控制指令
        bool should_move = controller.computeControl(uwb_data, x_speed, y_speed, angular_speed);
        
        // 发送控制指令
        if (should_move) {
            dog.move(x_speed, y_speed, angular_speed);
        } else {
            dog.stop();
        }
        
        // 定期输出状态信息
        frame_count++;
        if (frame_count % 20 == 0) {  // 每20帧输出一次（约1秒）
            if (uwb_data.is_valid) {
                printf("[状态] 距离=%.0fcm 角度=%.1f° | 速度: X=%.2f Y=%.2f W=%.2f | %s\n",
                       uwb_data.distance_cm, uwb_data.azimuth_deg,
                       x_speed, y_speed, angular_speed,
                       controller.isFollowing() ? "跟随中" : "等待中");
            } else {
                printf("[状态] 等待UWB数据...\n");
            }
        }
        
        // 控制周期
        usleep(FollowConfig::CONTROL_PERIOD_MS * 1000);
    }
    
    // ============== 清理 ==============
    
    std::cout << "\n[信息] 正在关闭..." << std::endl;
    dog.stop();
    dog.close();
    uwb_follower.close();
    
    std::cout << "[信息] 程序已退出" << std::endl;
    return 0;
}
