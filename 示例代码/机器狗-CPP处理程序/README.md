# UWB机器狗跟随程序 - C++版

## 概述

本程序用于机器狗接收UWB基站数据，进行数据滤波处理，实现跟随人的功能。

**已封装为库**，方便跟随程序调用！

参考资料：
- STM32F1-Usart文件夹（数据帧解析）
- uwb_filtered_visualizer_2d.py（滤波算法）

## 📁 文件列表

| 文件 | 说明 |
|------|------|
| `uwb_follower.h` | **库头文件** - 包含此文件即可使用 |
| `uwb_follower.cpp` | **库实现文件** - 编译时需链接 |
| `dog_follow_human.cpp` | **跟随控制程序** - 完整的2.5米跟随实现 |
| `example_usage.cpp` | 使用示例代码 |
| `uwb_dog_follower.cpp` | 独立版完整程序 |

## 🚀 快速使用（推荐）

```cpp
#include "uwb_follower.h"

int main() {
    // 1. 创建实例
    UWBFollower follower;
    
    // 2. 初始化串口
    if (!follower.init("/dev/ttyUSB0")) {
        return -1;
    }
    
    // 3. 循环获取数据
    while (true) {
        UWB2DData data = follower.getData();
        
        if (data.is_valid) {
            // 使用坐标控制机器狗
            double x = data.x;      // X坐标 (cm)
            double y = data.y;      // Y坐标 (cm)
            double vx = data.vx;    // X速度 (cm/s)
            double vy = data.vy;    // Y速度 (cm/s)
            
            // robot_control(x, y, vx, vy);
        }
    }
    
    return 0;
}
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

### 角度突变抑制原理
```
规则：
- 当 αk - αk-1 > 10°:  αexport = αk-1, dexport = dk-1 + 2
- 当 αk - αk-1 < -10°: αexport = αk-1, dexport = dk-1 - 2
- 当 |αk - αk-1| ≤ 10°: αexport = αk, dexport = dk
```

## 编译方法

### 方式一：编译为库（推荐）
```bash
# 编译库
g++ -c uwb_follower.cpp -o uwb_follower.o -std=c++11

# 编译主程序并链接库
g++ your_main.cpp uwb_follower.o -o your_program -std=c++11 -lm
```

### 方式二：编译示例程序
```bash
g++ example_usage.cpp uwb_follower.cpp -o dog_follow -std=c++11 -lm
```

### 方式三：编译独立版
```bash
g++ -o uwb_dog_follower uwb_dog_follower.cpp -std=c++11 -lm
```

### 交叉编译（ARM平台）
```bash
arm-linux-gnueabihf-g++ -c uwb_follower.cpp -o uwb_follower.o -std=c++11
arm-linux-gnueabihf-g++ your_main.cpp uwb_follower.o -o your_program -std=c++11 -lm
```

## 使用方法

```bash
./uwb_dog_follower /dev/ttyUSB0
```

## 输出示例

```
============================================
  UWB机器狗跟随程序 - 数据处理模块
============================================

滤波流程：
  1. 物理约束检查 (距离:5-5000cm, 速度<300cm/s)
  2. 角度突变抑制 (阈值:10°)
  3. 中位数滤波 (窗口:5)
  4. EKF状态估计

✓ 串口 /dev/ttyUSB0 已打开
  波特率: 115200

开始接收数据...
按 Ctrl+C 退出

【滤波后数据】 X=25.9cm, Y=96.7cm | Vx=0.0, Vy=0.0cm/s | 距离=100cm, 角度=15°
【滤波后数据】 X=27.1cm, Y=99.2cm | Vx=1.2, Vy=2.5cm/s | 距离=103cm, 角度=15°
...
```

## 数据结构

### 输出数据 (UWB2DData)
```cpp
struct UWB2DData {
    double x;           // X坐标 (cm)
    double y;           // Y坐标 (cm)
    double vx;          // X方向速度 (cm/s)
    double vy;          // Y方向速度 (cm/s)
    double distance_cm; // 距离 (cm)
    double azimuth_deg; // 方位角 (度)
    bool is_valid;      // 数据有效标志
};
```

## 如何集成到机器狗控制程序

### 方式一：使用封装库（推荐）
```cpp
#include "uwb_follower.h"

UWBFollower follower;
follower.init("/dev/ttyUSB0");

while (running) {
    UWB2DData data = follower.getData();
    if (data.is_valid) {
        // 跟随逻辑
        double distance = sqrt(data.x * data.x + data.y * data.y);
        double error = distance - TARGET_DISTANCE;
        
        if (fabs(error) > DEAD_ZONE) {
            robot_move(data.x, data.y);
        }
    }
}
```

### 方式二：只使用数据处理器
```cpp
#include "uwb_follower.h"

UWBDataProcessor processor;

// 当收到UWB原始数据时
UWBRawData raw;
raw.distance_cm = 100;
raw.azimuth_deg = 15;
raw.is_valid = true;

UWB2DData result = processor.process(raw);

if (result.is_valid) {
    robot_follow(result.x, result.y, result.vx, result.vy);
}
```

### 方式三：使用独立版程序
直接运行 `uwb_dog_follower`，读取输出结果。

## 配置参数

在代码开头可以调整以下参数：

```cpp
// 物理约束参数
const double MIN_DISTANCE = 5.0;       // 最小距离 (cm)
const double MAX_DISTANCE = 5000.0;    // 最大距离 (cm)
const double MAX_VELOCITY = 300.0;     // 最大速度 (cm/s)

// 中位数滤波参数
const int MEDIAN_WINDOW_SIZE = 5;      // 窗口大小

// 角度突变抑制参数
const double ANGLE_THRESHOLD = 10.0;   // 阈值（度）

// EKF参数
const double EKF_PROCESS_NOISE = 0.5;      // 过程噪声
const double EKF_MEASUREMENT_NOISE = 1.0;  // 测量噪声
```

## 作者

Copilot @ 2026-01-27

---

## 🎯 跟随控制程序 (dog_follow_human.cpp)

### 功能

实现机器狗以约**2.5米**距离跟随人，并始终朝向人。

### 控制逻辑

1. **获取UWB位置** - 使用`UWBFollower`库获取人的位置(x, y)
2. **距离控制** - 保持与目标人2.5米（250cm）的距离
3. **角度控制** - 始终转向面对目标人
4. **速度输出** - 通过`dog.move(x_speed, y_speed, angular_speed)`控制

### 控制参数

```cpp
// 跟随距离
TARGET_DISTANCE = 250.0 cm     // 目标距离：2.5米
DISTANCE_TOLERANCE = 30.0 cm   // 距离容差

// 速度限制
MAX_LINEAR_SPEED = 0.8 m/s     // 最大线速度
MAX_ANGULAR_SPEED = 1.2 rad/s  // 最大角速度

// 安全参数
STOP_DISTANCE = 100.0 cm       // 太近时停止
LOST_DISTANCE = 1000.0 cm      // 太远时停止跟随
```

### 编译运行

```bash
# 编译
g++ dog_follow_human.cpp uwb_follower.cpp -o dog_follow_human -std=c++11 -lm

# 运行
./dog_follow_human /dev/ttyUSB0
```

### 输出示例

```
============================================
UWB机器狗跟随程序 启动
============================================
串口设备: /dev/ttyUSB0
目标距离: 250 cm
按 Ctrl+C 退出
============================================

[信息] UWB串口已连接
[机器狗] 初始化完成
[信息] 开始跟随控制循环...
[跟随] 检测到目标，开始跟随 (距离=320cm, 角度=15°)
[状态] 距离=320cm 角度=15.0° | 速度: X=0.35 Y=0.00 W=0.45 | 跟随中
[状态] 距离=280cm 角度=8.2° | 速度: X=0.15 Y=0.00 W=0.25 | 跟随中
[状态] 距离=252cm 角度=2.1° | 速度: X=0.00 Y=0.00 W=0.00 | 跟随中
```

### 如何集成到您的机器狗SDK

在`dog_follow_human.cpp`中找到`RobotDog`类，将`move()`函数替换为您的机器狗SDK：

```cpp
void move(double x_speed, double y_speed, double angular_speed) {
    // TODO: 替换为您的机器狗SDK
    // 例如：your_robot_sdk.setVelocity(x_speed, y_speed, angular_speed);
}
