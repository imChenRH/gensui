/**
 * ============================================================================
 * UWB单基站跟随套件 - 机器狗跟随人程序
 * ============================================================================
 * 
 * 文件名：dog_follow_human.cpp
 * 
 * 功能说明：
 *     使用UWB定位系统实现机器狗跟随人的功能
 *     
 *     控制逻辑（径向+角向分离控制）：
 *     - 径向运动：当距离 > min_distance 时启动
 *       * 跟随速度 = 人的径向速度 × 系数
 *       * 系数是 (now_distance - min_distance) 的二次函数
 *       * 超过最大速度时保持最大速度，直到距离 < 1.1 * min_distance
 *     - 角向运动：同理
 * 
 * 使用方法：
 *     1. 将此文件与 uwb_follower.cpp 一起编译
 *     2. 运行程序，指定UWB基站串口
 * 
 * 编译命令：
 *     g++ dog_follow_human.cpp uwb_follower.cpp -o dog_follow_human -std=c++11 -lm
 * 
 * 运行命令：
 *     ./dog_follow_human /dev/ttyUSB0           # 正常模式
 *     ./dog_follow_human /dev/ttyUSB0 --debug   # 调试模式（显示详细信息）
 * 
 * 作者：Copilot
 * 日期：2026-01-28
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
    constexpr double MIN_DISTANCE = 250.0;         // 最小跟随距离 (cm) = 2.5米，低于此距离机器狗不启动径向运动
    constexpr double DISTANCE_DECEL_FACTOR = 1.1;  // 减速距离系数，当距离<1.1*MIN_DISTANCE时从最大速度减速
    
    // 角度参数
    constexpr double MIN_ANGLE = 3.5;              // 最小启动角度 (度)，低于此角度不启动角向运动
    constexpr double TARGET_ANGLE = 90.0;          // 目标角度 (度)
    constexpr double ANGLE_DECEL_FACTOR = 1.1;     // 角度减速系数，当角度<1.1*MIN_ANGLE时从最大角速度减速
    
    // 速度限制
    constexpr double MAX_LINEAR_SPEED = 5.0;       // 最大线速度 (m/s)
    constexpr double MAX_ANGULAR_SPEED = 1.0;      // 最大角速度 (rad/s)
    
    //人标准速度
    constexpr double HUMAN_STANDARD_SPEED = 1.2;   // 人的标准行走速度 (m/s)
    constexpr double HUMAN_ANGULAR_SPEED = 0.4;    // 人的标准角速度 (rad/s)

    // 二次函数系数 - 用于计算跟随系数
    // 当 distance_diff 增大时，系数增大
    constexpr double RADIAL_COEFF_A = 0.0001;      // 径向二次系数
    constexpr double RADIAL_COEFF_B = 0.02;        // 径向一次系数
    constexpr double RADIAL_COEFF_C = 1.0;         // 径向常数项
    
    constexpr double ANGULAR_COEFF_A = 0.0001;     // 角向二次系数
    constexpr double ANGULAR_COEFF_B = 0.04;       // 角向一次系数
    constexpr double ANGULAR_COEFF_C = 1.0;        // 角向常数项
    
    // 安全参数
    constexpr double STOP_DISTANCE = 100.0;        // 停止距离 (cm)，人太近时停止
    constexpr double LOST_DISTANCE = 1000.0;       // 丢失距离 (cm)，人太远时停止
    constexpr int MAX_LOST_FRAMES = 30;            // 最大丢失帧数，超过则停止
    
    // 控制频率
    constexpr int CONTROL_PERIOD_MS = 50;          // 控制周期 (ms)
    
    // 速度平滑参数（指数移动平均）
    // alpha越小，平滑效果越强，响应越慢；alpha越大，响应越快，平滑效果越弱
    constexpr double RADIAL_SMOOTH_ALPHA = 0.3;    // 径向速度平滑系数 (0.0-1.0)
    constexpr double ANGULAR_SMOOTH_ALPHA = 0.4;   // 角向速度平滑系数 (0.0-1.0)
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
 * 
 * 控制逻辑：
 * 1. 径向运动：当距离 > min_distance 时启动
 *    - 跟随速度 = 人的径向速度 × 系数
 *    - 系数是 (now_distance - min_distance) 的二次函数
 *    - 超过最大速度时保持最大速度，直到距离 < 1.1 * min_distance
 * 2. 角向运动：同理
 */
class FollowController {
public:
    FollowController() 
        : lost_frame_count_(0)
        , is_following_(false)
        , radial_at_max_speed_(false)
        , angular_at_max_speed_(false)
        , prev_radial_speed_(0.0)
        , prev_angular_speed_(0.0)
        , first_frame_(true) {
    }
    
    /**
     * 计算控制指令
     * @param uwb_data UWB位置数据（包含EKF估计的速度vx, vy）
     * @param x_speed 输出：X方向速度 (前进方向)
     * @param y_speed 输出：Y方向速度 (侧向)
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
            // 短暂丢失，保持上一次的控制
            return true;
        }
        
        // 数据有效，重置丢失计数
        lost_frame_count_ = 0;
        
        // 获取当前距离和角度
        double now_distance = uwb_data.distance_cm;
        double now_angle = uwb_data.azimuth_deg;
        
        // 计算距离和角度差
        double distance_diff = now_distance - FollowConfig::MIN_DISTANCE;
        
        double angle_diff = now_angle - FollowConfig::TARGET_ANGLE;
        double angle_sign = (angle_diff >= 0) ? 1.0 : -1.0;  // 角度方向
        angle_diff = std::abs(angle_diff);

        // 获取EKF估计的人的速度
        double human_vx = uwb_data.vx;  // X方向速度 (cm/s)
        double human_vy = uwb_data.vy;  // Y方向速度 (cm/s)
        
        // 计算人的径向速度（沿着机器狗到人的方向）
        double human_radial_speed = 0.0;
        if(human_vy>0) {//前进运动使用实际速度
            human_radial_speed = std::sqrt(human_vx * human_vx + human_vy * human_vy);
        }
        else {//后退或静止运动使用标准速度
            human_radial_speed = FollowConfig::HUMAN_STANDARD_SPEED * 100;  // 转换为cm/s
        }
        
        // 计算人的角向速度（需要从xy速度分解）
        // 使用arctan2计算人移动方向的角度变化率
        double human_angular_speed = 0.0;
        if (now_distance > 10.0) {  // 避免除以零
            // 角速度近似 = (垂直于径向的速度分量) / 距离
            // 假设x是左右方向，y是前后方向
            // 角速度 = vx / distance (简化计算)
            if((human_vx>0)^(angle_sign>0))//靠近运动使用标准角速度
                human_angular_speed = FollowConfig::HUMAN_ANGULAR_SPEED * 57.3;  // 转换为度/秒
            else//远离运动使用实际角速度
                human_angular_speed = std::abs(human_vx) / now_distance * 57.3;  // 转换为度/秒
        }
        
        // ============== 安全检查 ==============
        
        // 人太近，停止
        if (now_distance < FollowConfig::STOP_DISTANCE) {
            if (is_following_) {
                std::cout << "[跟随] 人太近 (" << now_distance 
                          << "cm)，停止并等待" << std::endl;
            }
            is_following_ = false;
            radial_at_max_speed_ = false;
            angular_at_max_speed_ = false;
            return true;  // 返回true但速度为0
        }
        
        // 人太远，停止
        if (now_distance > FollowConfig::LOST_DISTANCE) {
            if (is_following_) {
                std::cout << "[跟随] 人太远 (" << now_distance 
                          << "cm)，停止跟随" << std::endl;
            }
            is_following_ = false;
            radial_at_max_speed_ = false;
            angular_at_max_speed_ = false;
            return false;
        }
        
        // 开始跟随
        if (!is_following_) {
            std::cout << "[跟随] 检测到目标，开始跟随 (距离=" 
                      << now_distance << "cm, 角度=" 
                      << uwb_data.azimuth_deg << "°)" << std::endl;
            is_following_ = true;
        }
        
        // ============== 径向运动控制 ==============
                
        if (distance_diff > 0) {
            // 距离大于最小距离，启动径向运动
            
            // 计算二次函数系数
            double radial_coeff = FollowConfig::RADIAL_COEFF_A * distance_diff * distance_diff
                                + FollowConfig::RADIAL_COEFF_B * distance_diff
                                + FollowConfig::RADIAL_COEFF_C;
            
            // 机器狗的径向速度 = 人的径向速度 × 系数
            // 单位转换: cm/s -> m/s
            double radial_speed = (human_radial_speed * radial_coeff) / 100.0;
            
            // 如果人在远离机器狗，也要加上距离差带来的基础速度
            // 这样即使人静止，机器狗也会慢慢靠近
            // radial_speed += (distance_diff / 1000.0);  // 每100cm差距增加0.1m/s
            
            // 检查是否超过最大速度
            if (radial_speed > FollowConfig::MAX_LINEAR_SPEED) {
                radial_speed = FollowConfig::MAX_LINEAR_SPEED;
                radial_at_max_speed_ = true;
            } else if (radial_at_max_speed_) {
                // 之前在最大速度，检查是否应该减速
                if (now_distance < FollowConfig::DISTANCE_DECEL_FACTOR * FollowConfig::MIN_DISTANCE) {
                    radial_at_max_speed_ = false;
                } else {
                    // 保持最大速度
                    radial_speed = FollowConfig::MAX_LINEAR_SPEED;
                }
            }
            
            x_speed = radial_speed;
        } else {
            // 距离小于最小距离，不需要前进（可能需要后退）
            radial_at_max_speed_ = false;
            
            // 如果太近，可以考虑后退
            // if (now_distance < FollowConfig::MIN_DISTANCE * 0.8) {
            // x_speed = -0.1;  // 缓慢后退
            // }
        }
        
        // ============== 角向运动控制 ==============

        if (angle_diff > FollowConfig::MIN_ANGLE) {
            // 角度大于最小角度，启动角向运动
            
            // 计算二次函数系数
            double angular_coeff = FollowConfig::ANGULAR_COEFF_A * angle_diff * angle_diff
                                 + FollowConfig::ANGULAR_COEFF_B * angle_diff
                                 + FollowConfig::ANGULAR_COEFF_C;
            
            // 机器狗的角速度 = 人的角速度 × 系数
            // 加上基础的角度偏差纠正
            double angular_speed_cmd = human_angular_speed * angular_coeff;
            
            // 加上基础的角度纠正速度
            // angular_speed_cmd += (angle_diff / 30.0);  // 每30度增加1 rad/s
            
            // 转换为rad/s并应用方向
            angular_speed_cmd = angular_speed_cmd / 57.3;  // 度/秒 转 rad/s
            
            // 检查是否超过最大角速度
            if (angular_speed_cmd > FollowConfig::MAX_ANGULAR_SPEED) {
                angular_speed_cmd = FollowConfig::MAX_ANGULAR_SPEED;
                angular_at_max_speed_ = true;
            } else if (angular_at_max_speed_) {
                // 之前在最大角速度，检查是否应该减速
                if (angle_diff < FollowConfig::ANGLE_DECEL_FACTOR * FollowConfig::MIN_ANGLE) {
                    angular_at_max_speed_ = false;
                } else {
                    // 保持最大角速度
                    angular_speed_cmd = FollowConfig::MAX_ANGULAR_SPEED;
                }
            }
            
            angular_speed = angular_speed_cmd * angle_sign;
        } else {
            // 角度小于最小角度，不需要旋转
            angular_at_max_speed_ = false;
        }
        
        // ============== 速度平滑处理 ==============
        
        if (first_frame_) {
            // 第一帧不平滑，直接使用计算值
            prev_radial_speed_ = x_speed;
            prev_angular_speed_ = angular_speed;
            first_frame_ = false;
        } else {
            // 使用指数移动平均进行平滑
            // smoothed = alpha * current + (1 - alpha) * previous
            x_speed = FollowConfig::RADIAL_SMOOTH_ALPHA * x_speed 
                    + (1.0 - FollowConfig::RADIAL_SMOOTH_ALPHA) * prev_radial_speed_;
            angular_speed = FollowConfig::ANGULAR_SMOOTH_ALPHA * angular_speed 
                          + (1.0 - FollowConfig::ANGULAR_SMOOTH_ALPHA) * prev_angular_speed_;
            
            // 更新上一次速度
            prev_radial_speed_ = x_speed;
            prev_angular_speed_ = angular_speed;
        }
        
        // ============== 侧向运动（可选） ==============
        
        // 如果人在侧面，可以添加侧移以更快接近
        // y_speed 可以用于侧向移动
        // double lateral_factor = 0.001;  // 侧移比例系数
        // y_speed = -lateral_factor * uwb_data.x;  // 向人的方向侧移
        // y_speed = clamp(y_speed, -0.2, 0.2);  // 限制侧移速度
        
        return true;
    }
    
    /**
     * 重置控制器状态
     */
    void reset() {
        lost_frame_count_ = 0;
        is_following_ = false;
        radial_at_max_speed_ = false;
        angular_at_max_speed_ = false;
        prev_radial_speed_ = 0.0;
        prev_angular_speed_ = 0.0;
        first_frame_ = true;
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
    
    int lost_frame_count_;
    bool is_following_;
    bool radial_at_max_speed_;    // 径向是否在最大速度
    bool angular_at_max_speed_;   // 角向是否在最大角速度
    
    // 速度平滑变量
    double prev_radial_speed_;    // 上一次径向速度
    double prev_angular_speed_;   // 上一次角向速度
    bool first_frame_;            // 是否是第一帧
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
        std::cout << "\n用法: " << argv[0] << " <串口设备> [--debug]" << std::endl;
        std::cout << "\n示例:" << std::endl;
        std::cout << "  " << argv[0] << " /dev/ttyUSB0         # 正常模式" << std::endl;
        std::cout << "  " << argv[0] << " /dev/ttyUSB0 --debug # 调试模式" << std::endl;
        std::cout << "\n控制逻辑:" << std::endl;
        std::cout << "  - 径向运动: 速度 = 人的径向速度 × f(distance_diff)" << std::endl;
        std::cout << "  - 角向运动: 角速度 = 人的角速度 × f(angle_diff)" << std::endl;
        std::cout << "  - f(x) = a*x² + b*x + c (二次函数)" << std::endl;
        std::cout << "\n参数说明:" << std::endl;
        std::cout << "  最小启动距离: " << FollowConfig::MIN_DISTANCE << " cm (2.5米)" << std::endl;
        std::cout << "  最小启动角度: " << FollowConfig::MIN_ANGLE << "°" << std::endl;
        std::cout << "  最大线速度: " << FollowConfig::MAX_LINEAR_SPEED << " m/s" << std::endl;
        std::cout << "  最大角速度: " << FollowConfig::MAX_ANGULAR_SPEED << " rad/s" << std::endl;
        std::cout << "  减速距离: " << FollowConfig::DISTANCE_DECEL_FACTOR << " × 最小距离" << std::endl;
        return 1;
    }
    
    std::string port = argv[1];
    
    // 检查调试模式
    bool debug_mode = false;
    if (argc >= 3 && std::string(argv[2]) == "--debug") {
        debug_mode = true;
    }
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "============================================" << std::endl;
    std::cout << "UWB机器狗跟随程序 启动" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << "串口设备: " << port << std::endl;
    std::cout << "调试模式: " << (debug_mode ? "开启" : "关闭") << std::endl;
    std::cout << "最小启动距离: " << FollowConfig::MIN_DISTANCE << " cm" << std::endl;
    std::cout << "控制逻辑: 速度 = 人的速度 × 二次函数系数" << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
    std::cout << "============================================\n" << std::endl;
    
    // 初始化UWB跟随器
    UWBFollower uwb_follower;
    if (!uwb_follower.init(port)) {
        std::cerr << "[错误] 无法打开串口: " << port << std::endl;
        std::cerr << "[提示] 请检查:" << std::endl;
        std::cerr << "  1. 串口设备是否存在: ls " << port << std::endl;
        std::cerr << "  2. 串口权限: sudo chmod 666 " << port << std::endl;
        std::cerr << "  3. 或者添加用户到dialout组: sudo usermod -aG dialout $USER" << std::endl;
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
    int valid_count = 0;     // 有效数据计数
    int invalid_count = 0;   // 无效数据计数
    
    // ============== 主控制循环 ==============
    
    std::cout << "[信息] 开始跟随控制循环..." << std::endl;
    if (debug_mode) {
        std::cout << "[调试] 等待UWB数据..." << std::endl;
    }
    
    while (g_running) {
        // 获取UWB数据
        UWB2DData uwb_data = uwb_follower.getData();
        
        // 调试模式：显示详细信息
        if (debug_mode) {
            if (uwb_data.is_valid) {
                valid_count++;
                printf("[调试] 有效数据 #%d: 距离=%.0fcm 角度=%.1f° X=%.1f Y=%.1f Vx=%.1f Vy=%.1f\n",
                       valid_count, uwb_data.distance_cm, uwb_data.azimuth_deg,
                       uwb_data.x, uwb_data.y, uwb_data.vx, uwb_data.vy);
            } else {
                invalid_count++;
                if (invalid_count % 100 == 0) {  // 每100次无效数据报告一次
                    printf("[调试] 无效数据累计: %d (有效: %d)\n", invalid_count, valid_count);
                    // 检查原始统计
                    printf("[调试] UWB统计: 总帧=%d 滤波后=%d\n",
                           uwb_follower.getTotalCount(), uwb_follower.getFilteredCount());
                }
            }
        }
        
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
                printf("[状态] 等待UWB数据... (有效:%d 无效:%d)\n", valid_count, invalid_count);
            }
        }
        
        // 控制周期
        usleep(FollowConfig::CONTROL_PERIOD_MS * 1000);
    }
    
    // ============== 清理 ==============
    
    std::cout << "\n[信息] 正在关闭..." << std::endl;
    std::cout << "[统计] 有效数据: " << valid_count << " 无效数据: " << invalid_count << std::endl;
    dog.stop();
    dog.close();
    uwb_follower.close();
    
    std::cout << "[信息] 程序已退出" << std::endl;
    return 0;
}
