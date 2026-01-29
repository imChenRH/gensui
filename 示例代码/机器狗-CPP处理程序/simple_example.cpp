/**
 * ============================================================================
 * UWB机器狗跟随库 - 简单使用示例
 * ============================================================================
 * 
 * 展示如何使用 DogController 库获取机器狗的速度指令
 * 
 * 编译命令：
 *     g++ simple_example.cpp dog_controller.cpp uwb_follower.cpp -o simple_example -std=c++11 -lm
 * 
 * 运行命令：
 *     ./simple_example /dev/ttyUSB0
 * 
 * ============================================================================
 */

#include <iostream>
#include <csignal>
#include <unistd.h>
#include "dog_controller.h"

volatile bool g_running = true;

void signalHandler(int signum) {
    std::cout << "\n收到退出信号..." << std::endl;
    g_running = false;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cout << "用法: " << argv[0] << " <串口设备>" << std::endl;
        std::cout << "示例: " << argv[0] << " /dev/ttyUSB0" << std::endl;
        return 1;
    }
    
    std::string port = argv[1];
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    
    std::cout << "============================================" << std::endl;
    std::cout << "UWB机器狗跟随库 - 简单示例" << std::endl;
    std::cout << "============================================" << std::endl;
    
    // 创建控制器
    DogController dog;
    
    // 可选：设置调试模式
    // dog.setDebugMode(true);
    
    // 可选：设置跟随距离（默认250cm = 2.5米）
    // dog.setFollowDistance(200.0);  // 2米
    
    // 可选：设置最大速度
    // dog.setMaxSpeed(1.0, 0.5);  // 最大线速度1m/s, 最大角速度0.5rad/s
    
    // 初始化
    if (!dog.init(port)) {
        std::cerr << "无法打开串口: " << port << std::endl;
        return 1;
    }
    std::cout << "串口已连接: " << port << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
    std::cout << "============================================\n" << std::endl;
    
    int frame_count = 0;
    
    // 主循环
    while (g_running) {
        // ========================================
        // 获取机器狗速度指令（核心代码就这一行！）
        // ========================================
        DogVelocity vel = dog.getVelocity();
        
        // 使用速度指令
        if (vel.is_valid) {
            // ========================================
            // 在这里调用你的机器狗SDK
            // ========================================
            // your_robot.move(vel.vx, vel.vy, vel.omega);
            
            // 打印状态（每20帧输出一次）
            frame_count++;
            if (frame_count % 20 == 0) {
                printf("距离=%.0fcm 角度=%.1f° | 速度: vx=%.2f vy=%.2f omega=%.2f | %s\n",
                       vel.distance_cm, vel.azimuth_deg,
                       vel.vx, vel.vy, vel.omega,
                       vel.is_following ? "跟随中" : "等待中");
            }
        }
        
        // 控制周期50ms
        usleep(50000);
    }
    
    // 清理
    std::cout << "\n正在关闭..." << std::endl;
    std::cout << "总帧数: " << dog.getTotalFrames() 
              << " 有效帧: " << dog.getValidFrames() << std::endl;
    dog.close();
    
    return 0;
}
