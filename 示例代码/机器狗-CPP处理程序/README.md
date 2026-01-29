# UWB机器狗跟随程序 - C++版

## 概述

本程序用于机器狗接收UWB基站数据，进行数据滤波处理，实现跟随人的功能。

**🎉 单头文件版本：只需 `#include "dog_controller_full.h"` 即可使用！**

参考资料：
- STM32F1-Usart文件夹（数据帧解析）
- uwb_filtered_visualizer_2d.py（滤波算法）

## 📁 文件列表

| 文件 | 说明 |
|------|------|
| **dog_controller_full.h** | **单头文件版本** - 只需要这一个头文件！ |
| **dog_controller_full.cpp** | **单头文件版本实现** - 编译时链接 |
| **simple_example.cpp** | **简单示例** - 展示如何使用 |
| `dog_follow_human.cpp` | 完整跟随程序（旧版） |
| `dog_controller.h/cpp` | 旧版整合库（需要uwb_follower） |
| `uwb_follower.h/cpp` | 旧版UWB数据库 |

## 🚀 快速使用（单头文件版 - 推荐！）

**一行代码获取机器狗速度指令，只需include一个头文件！**

```cpp
#include "dog_controller_full.h"  // 只需要这一个！

int main() {
    DogController dog;
    
    // 初始化
    if (!dog.init("/dev/ttyUSB0")) {
        return -1;
    }
    
    // 可选配置
    // dog.setFollowDistance(200.0);  // 设置跟随距离2米
    // dog.setMaxSpeed(1.0, 0.5);     // 设置最大速度
    // dog.setDebugMode(true);        // 开启调试输出
    
    while (true) {
        // ========== 核心代码就这一行！ ==========
        DogVelocity vel = dog.getVelocity();
        
        if (vel.is_valid) {
            // 调用您的机器狗SDK
            your_robot.move(vel.vx, vel.vy, vel.omega);
            
            // vx    : 前进速度 (m/s)
            // vy    : 侧向速度 (m/s)，通常为0
            // omega : 角速度 (rad/s)
        }
        
        usleep(50000);  // 50ms控制周期
    }
    
    return 0;
}
```

## 编译方法

### 单头文件版（推荐）
```bash
g++ simple_example.cpp dog_controller_full.cpp -o simple_example -std=c++11 -lm
```

### 交叉编译（ARM平台）
```bash
arm-linux-gnueabihf-g++ simple_example.cpp dog_controller_full.cpp -o simple_example -std=c++11 -lm
```

## DogVelocity 结构体

```cpp
struct DogVelocity {
    double vx;          // 前进速度 (m/s)
    double vy;          // 侧向速度 (m/s)
    double omega;       // 角速度 (rad/s)
    bool is_valid;      // 数据有效
    bool is_following;  // 正在跟随
    
    // 调试信息
    double distance_cm;  // 当前距离
    double azimuth_deg;  // 当前角度
    double human_vx;     // 人的X速度
    double human_vy;     // 人的Y速度
};
```

## 功能特点

### 坐标转换（忽略仰角）
直线距离直接视作平地距离：
```
X = Distance × sin(Azimuth)
Y = Distance × cos(Azimuth)
```

### 滤波流程（四步）

| 步骤 | 方法 | 说明 |
|------|------|------|
| Step 1 | 物理约束检查 | 距离范围(5-5000cm) + 速度限制(<300cm/s) |
| Step 2 | 角度突变抑制 | 阈值10°，抑制人员遮挡导致的角度突变 |
| Step 3 | 中位数滤波 | 窗口大小5，消除尖峰异常值 |
| Step 4 | EKF状态估计 | 2D位置平滑 + 速度估计 |

### 跟随控制逻辑

- 径向运动：基于EKF估计的人的速度 × 二次函数系数
- 角向运动：同样的逻辑应用于角度控制
- 速度平滑：指数移动平均(EMA)
- 安全保护：太近停止、太远丢失检测

## 配置参数

可在 `FollowConfig` 命名空间中调整：

```cpp
namespace FollowConfig {
    // 跟随距离
    constexpr double MIN_DISTANCE = 250.0;     // 2.5米
    
    // 速度限制
    constexpr double MAX_LINEAR_SPEED = 5.0;   // m/s
    constexpr double MAX_ANGULAR_SPEED = 1.0;  // rad/s
    
    // 安全参数
    constexpr double STOP_DISTANCE = 100.0;    // 太近停止
    constexpr double LOST_DISTANCE = 1000.0;   // 太远丢失
}
```

## 使用方法

```bash
./simple_example /dev/ttyUSB0
```

## 输出示例

```
============================================
UWB机器狗跟随库 - 简单示例
============================================
串口 /dev/ttyUSB0 初始化成功 (波特率=115200)
串口已连接: /dev/ttyUSB0
按 Ctrl+C 退出
============================================

距离=320cm 角度=15.0° | 速度: vx=0.35 vy=0.00 omega=0.45 | 跟随中
距离=280cm 角度=8.2° | 速度: vx=0.15 vy=0.00 omega=0.25 | 跟随中
距离=252cm 角度=2.1° | 速度: vx=0.00 vy=0.00 omega=0.00 | 跟随中
```

## 如何集成到您的机器狗SDK

```cpp
#include "dog_controller_full.h"

// 创建您的机器狗对象
YourRobotSDK robot;

// 创建UWB控制器
DogController dog;
dog.init("/dev/ttyUSB0");

while (running) {
    DogVelocity vel = dog.getVelocity();
    
    if (vel.is_valid) {
        // 将速度指令发送给机器狗
        robot.setVelocity(vel.vx, vel.vy, vel.omega);
    }
}
```

## 作者

Copilot @ 2026-01-29
