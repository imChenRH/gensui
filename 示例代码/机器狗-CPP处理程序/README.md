# UWB机器狗跟随程序 - C++版

## 概述

本程序用于机器狗接收UWB基站数据，进行数据滤波处理，输出用于跟随控制的X、Y坐标值。

参考资料：
- STM32F1-Usart文件夹（数据帧解析）
- uwb_filtered_visualizer_2d.py（滤波算法）

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

### Linux/机器狗平台
```bash
g++ -o uwb_dog_follower uwb_dog_follower.cpp -std=c++11 -lm
```

### 交叉编译（ARM平台）
```bash
arm-linux-gnueabihf-g++ -o uwb_dog_follower uwb_dog_follower.cpp -std=c++11 -lm
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

### 方式一：直接调用处理函数
```cpp
#include "uwb_dog_follower.cpp"  // 或者提取为头文件

UWBDataProcessor processor;

// 当收到UWB原始数据时
UWBRawData raw;
raw.distance_cm = 100;
raw.azimuth_deg = 15;
raw.is_valid = true;

UWB2DData result = processor.process(raw);

if (result.is_valid) {
    // 使用result.x, result.y控制机器狗
    robot_follow(result.x, result.y, result.vx, result.vy);
}
```

### 方式二：将滤波器封装为独立模块
可以将 `UWBDataProcessor` 类提取到单独的头文件中，方便在其他项目中复用。

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
