/**
 * ============================================================================
 * UWB机器狗跟随程序 - 使用示例
 * ============================================================================
 * 
 * 文件名：example_usage.cpp
 * 
 * 说明：
 *     演示如何使用 uwb_follower 库进行机器狗跟随
 * 
 * 编译方法：
 *     g++ example_usage.cpp uwb_follower.cpp -o dog_follow -std=c++11 -lm
 * 
 * 使用方法：
 *     ./dog_follow /dev/ttyUSB0
 * 
 * 作者：Copilot
 * 日期：2026-01-27
 * ============================================================================
 */

#include <iostream>
#include <iomanip>
#include <csignal>
#include "uwb_follower.h"

#ifdef __linux__
#include <unistd.h>
#endif

// 全局变量用于信号处理
volatile bool g_running = true;

void signalHandler(int signum) {
    std::cout << "\n正在退出..." << std::endl;
    g_running = false;
}

// ============================================================================
// 示例1：基本用法
// ============================================================================

void example_basic_usage(const char* port) {
    std::cout << "=== 示例1：基本用法 ===" << std::endl;
    std::cout << std::endl;
    
    // 1. 创建UWBFollower实例
    UWBFollower follower;
    
    // 2. 初始化串口连接
    if (!follower.init(port)) {
        std::cerr << "初始化失败！" << std::endl;
        return;
    }
    
    std::cout << "已连接到 " << port << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
    std::cout << std::endl;
    
    // 3. 循环获取数据
    while (g_running) {
        // 获取处理后的坐标数据
        UWB2DData data = follower.getData();
        
        if (data.is_valid) {
            // 使用坐标数据
            std::cout << "位置: X=" << std::fixed << std::setprecision(1) << data.x
                      << "cm, Y=" << data.y << "cm"
                      << " | 速度: Vx=" << data.vx << ", Vy=" << data.vy << "cm/s"
                      << std::endl;
            
            // 这里可以添加机器狗控制逻辑
            // control_robot(data.x, data.y, data.vx, data.vy);
        }
        
#ifdef __linux__
        usleep(10000);  // 10ms延时
#endif
    }
    
    // 4. 关闭连接
    follower.close();
    
    // 5. 输出统计信息
    std::cout << std::endl;
    std::cout << "统计: 总数=" << follower.getTotalCount()
              << ", 有效=" << follower.getFilteredCount()
              << ", 角度突变抑制=" << follower.getSuppressedCount() << std::endl;
}

// ============================================================================
// 示例2：在跟随循环中使用
// ============================================================================

void example_follow_loop(const char* port) {
    std::cout << "=== 示例2：跟随循环 ===" << std::endl;
    std::cout << std::endl;
    
    UWBFollower follower;
    
    if (!follower.init(port)) {
        std::cerr << "初始化失败！" << std::endl;
        return;
    }
    
    std::cout << "开始跟随模式..." << std::endl;
    std::cout << std::endl;
    
    // 跟随参数
    const double FOLLOW_DISTANCE = 100.0;  // 目标跟随距离 (cm)
    const double DEAD_ZONE = 10.0;         // 死区半径 (cm)
    
    while (g_running) {
        UWB2DData data = follower.getData();
        
        if (data.is_valid) {
            // 计算当前距离
            double current_distance = std::sqrt(data.x * data.x + data.y * data.y);
            
            // 计算距离误差
            double distance_error = current_distance - FOLLOW_DISTANCE;
            
            // 判断是否需要移动
            if (std::abs(distance_error) > DEAD_ZONE) {
                // 计算移动方向
                double direction_x = data.x / current_distance;
                double direction_y = data.y / current_distance;
                
                std::cout << "跟随: 距离误差=" << std::fixed << std::setprecision(1) 
                          << distance_error << "cm"
                          << " | 方向=(" << direction_x << ", " << direction_y << ")"
                          << std::endl;
                
                // 这里添加机器狗移动控制
                // move_robot(direction_x * speed, direction_y * speed);
            } else {
                std::cout << "停止: 在跟随距离内" << std::endl;
                // stop_robot();
            }
        }
        
#ifdef __linux__
        usleep(50000);  // 50ms控制周期
#endif
    }
    
    follower.close();
}

// ============================================================================
// 示例3：处理外部数据源
// ============================================================================

void example_external_data() {
    std::cout << "=== 示例3：处理外部数据 ===" << std::endl;
    std::cout << std::endl;
    
    UWBFollower follower;
    // 注意：不需要调用init()，因为我们使用外部数据
    
    // 模拟外部数据
    UWBRawData raw;
    raw.anchor_id = 0xAAA2;
    raw.tag_id = 0xAAA1;
    raw.distance_cm = 150;
    raw.azimuth_deg = 30;
    raw.elevation_deg = 0;
    raw.is_valid = true;
    
    // 处理数据
    UWB2DData result = follower.processRawData(raw);
    
    if (result.is_valid) {
        std::cout << "输入: 距离=" << raw.distance_cm << "cm, 角度=" << raw.azimuth_deg << "°" << std::endl;
        std::cout << "输出: X=" << std::fixed << std::setprecision(1) << result.x
                  << "cm, Y=" << result.y << "cm" << std::endl;
    }
    
    std::cout << std::endl;
}

// ============================================================================
// 示例4：简洁API（推荐）
// ============================================================================

void example_simple_api(const char* port) {
    std::cout << "=== 示例4：简洁API ===" << std::endl;
    std::cout << std::endl;
    std::cout << "这是推荐的使用方式：" << std::endl;
    std::cout << std::endl;
    std::cout << "```cpp" << std::endl;
    std::cout << "UWBFollower follower;" << std::endl;
    std::cout << "follower.init(\"/dev/ttyUSB0\");" << std::endl;
    std::cout << std::endl;
    std::cout << "while (running) {" << std::endl;
    std::cout << "    UWB2DData data = follower.getData();" << std::endl;
    std::cout << "    if (data.is_valid) {" << std::endl;
    std::cout << "        // 使用 data.x, data.y 控制机器狗" << std::endl;
    std::cout << "        // 使用 data.vx, data.vy 进行速度前馈" << std::endl;
    std::cout << "    }" << std::endl;
    std::cout << "}" << std::endl;
    std::cout << "```" << std::endl;
    std::cout << std::endl;
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char* argv[]) {
    // 注册信号处理
    signal(SIGINT, signalHandler);
    
    std::cout << "============================================" << std::endl;
    std::cout << "  UWB机器狗跟随库 - 使用示例" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << std::endl;
    
    // 先展示简洁API
    example_simple_api(nullptr);
    
    // 展示外部数据处理
    example_external_data();
    
    // 如果提供了串口参数，运行实际示例
    if (argc >= 2) {
        const char* port = argv[1];
        
        std::cout << "选择运行模式:" << std::endl;
        std::cout << "  1 - 基本用法（显示坐标）" << std::endl;
        std::cout << "  2 - 跟随循环（显示跟随状态）" << std::endl;
        std::cout << "请输入选择 (1/2): ";
        
        int choice;
        std::cin >> choice;
        
        if (choice == 2) {
            example_follow_loop(port);
        } else {
            example_basic_usage(port);
        }
    } else {
        std::cout << "提示：运行带串口参数可测试实际数据" << std::endl;
        std::cout << "用法: " << argv[0] << " /dev/ttyUSB0" << std::endl;
    }
    
    return 0;
}
