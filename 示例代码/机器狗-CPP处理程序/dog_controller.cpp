/**
 * ============================================================================
 * UWB单基站跟随套件 - 机器狗跟随控制库（整合版）
 * ============================================================================
 * 
 * 文件名：dog_controller.cpp
 * 
 * 功能说明：
 *     整合 UWB数据接收、滤波处理 和 跟随控制 的一体化库实现
 * 
 * 编译方法：
 *     g++ your_main.cpp dog_controller.cpp uwb_follower.cpp -o your_program -std=c++11 -lm
 * 
 * ============================================================================
 */

#include "dog_controller.h"
#include <cmath>
#include <cstdio>

// ============================================================================
// DogController 实现
// ============================================================================

DogController::DogController()
    : min_distance_(FollowConfig::MIN_DISTANCE)
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
    return uwb_follower_.init(port, baudrate);
}

void DogController::close() {
    uwb_follower_.close();
}

bool DogController::isConnected() const {
    return uwb_follower_.isConnected();
}

DogVelocity DogController::getVelocity() {
    // 获取UWB数据
    UWB2DData uwb_data = uwb_follower_.getData();
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
    uwb_follower_.reset();
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
    return uwb_follower_.getFilteredCount();
}

UWB2DData DogController::getUWBData() {
    return uwb_follower_.getData();
}

double DogController::clamp(double value, double min_val, double max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}
