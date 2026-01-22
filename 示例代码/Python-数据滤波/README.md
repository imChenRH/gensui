# UWB单基站跟随套件 - 数据滤波与异常值去除模块

## 问题背景

在机器狗跟随场景中，UWB定位数据可能因为以下原因出现异常值：
- **人员经过**：其他人经过基站和标签之间，造成信号遮挡
- **多径效应**：信号反射导致距离测量偏大
- **环境干扰**：金属物体、电磁干扰等
- **信号丢失**：临时失去测距连接

这些异常值如果不处理，会导致机器狗出现突然的位置跳变，影响跟随效果。

## 解决方案

本模块提供多种数据滤波和异常值去除方法：

### 1. 卡尔曼滤波 (Kalman Filter)

**原理**：基于状态估计的最优滤波器，结合预测值和测量值，给出最优估计。

**优点**：
- 数学最优估计
- 能处理动态变化
- 对小幅噪声效果好

**参数调节**：
- `Q`（过程噪声）：越大越相信测量值
- `R`（测量噪声）：越大越相信预测值

```python
from uwb_data_filter import KalmanFilter1D

kf = KalmanFilter1D(q=0.1, r=0.5)
filtered_value = kf.update(raw_value)
```

### 2. 中位数滤波 (Median Filter)

**原理**：取滑动窗口内数据的中位数作为输出。

**优点**：
- 对脉冲噪声（突变）非常有效
- 不会被单个异常值拉偏

**适用场景**：人员遮挡导致的突然跳变

```python
from uwb_data_filter import MedianFilter

mf = MedianFilter(window_size=5)
filtered_value = mf.update(raw_value)
```

### 3. 指数加权移动平均 (EWMA)

**原理**：新值 = α × 测量值 + (1-α) × 上一个估计值

**优点**：
- 计算简单
- 可调节响应速度

```python
from uwb_data_filter import EWMAFilter

ewma = EWMAFilter(alpha=0.3)
filtered_value = ewma.update(raw_value)
```

### 4. 异常值检测

提供三种检测方法：

#### 4.1 基于速度的检测
```python
# 如果位置变化速度超过200cm/s，判定为异常
detector.is_outlier_by_velocity(x, y, z)
```

#### 4.2 基于距离跳变的检测
```python
# 如果距离突然变化超过50cm，判定为异常
detector.is_outlier_by_distance_jump(distance)
```

#### 4.3 基于Z-Score的检测
```python
# 如果数据偏离历史均值超过3个标准差，判定为异常
detector.is_outlier_by_zscore(x, y, z)
```

### 5. 综合滤波器 (推荐)

结合异常值检测和卡尔曼滤波的完整解决方案：

```python
from uwb_data_filter import UWBDataFilter

# 创建滤波器
filter = UWBDataFilter(
    max_velocity=200.0,      # 最大速度 200 cm/s
    max_distance_jump=50.0,  # 最大距离跳变 50 cm
    z_score_threshold=3.0,   # Z-Score阈值
    kalman_q=0.1,            # 卡尔曼过程噪声
    kalman_r=0.5,            # 卡尔曼测量噪声
    use_median_prefilter=True,  # 使用中位数预滤波
    median_window=3
)

# 滤波
result = filter.filter(distance, x, y, z)
if result:
    print(f"滤波后: X={result['x']}, Y={result['y']}, Z={result['z']}")
    if result['is_outlier']:
        print("(此数据点被检测为异常，已使用插值)")
```

## 参数调节建议

### 针对机器狗跟随场景

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| max_velocity | 150-250 cm/s | 人正常行走速度约120cm/s，跑步约300cm/s |
| max_distance_jump | 30-60 cm | 根据数据更新频率调整 |
| z_score_threshold | 2.5-3.5 | 越小越严格 |
| kalman_q | 0.05-0.2 | 越大响应越快，但平滑效果弱 |
| kalman_r | 0.3-1.0 | 越大平滑效果越强，但响应越慢 |
| median_window | 3-7 | 窗口越大，去异常值能力越强，但延迟越大 |

### 根据场景调整

**场景1：跟随速度较慢的人**
```python
filter = UWBDataFilter(
    max_velocity=150.0,
    kalman_r=0.8  # 更平滑
)
```

**场景2：跟随速度较快的人**
```python
filter = UWBDataFilter(
    max_velocity=300.0,
    kalman_q=0.2,  # 更快响应
    kalman_r=0.3
)
```

**场景3：环境干扰较多**
```python
filter = UWBDataFilter(
    max_distance_jump=30.0,  # 更严格的跳变检测
    z_score_threshold=2.5,   # 更严格的统计检测
    use_median_prefilter=True,
    median_window=5
)
```

## 使用示例

### 完整示例

```python
import math
from uwb_data_filter import UWBDataFilter

# 创建滤波器
filter = UWBDataFilter()

# 模拟接收数据
while True:
    # 获取原始UWB数据
    raw_distance = get_distance_from_uwb()
    raw_azimuth = get_azimuth_from_uwb()
    raw_elevation = get_elevation_from_uwb()
    
    # 球坐标转三维坐标
    azimuth_rad = math.radians(raw_azimuth)
    elevation_rad = math.radians(raw_elevation)
    horizontal = raw_distance * math.cos(elevation_rad)
    
    raw_x = horizontal * math.sin(azimuth_rad)
    raw_y = horizontal * math.cos(azimuth_rad)
    raw_z = raw_distance * math.sin(elevation_rad)
    
    # 滤波
    result = filter.filter(raw_distance, raw_x, raw_y, raw_z)
    
    if result and not result['is_outlier']:
        # 使用滤波后的数据控制机器狗
        control_robot(result['x'], result['y'])
    elif result and result['is_outlier']:
        # 异常值被检测到，使用插值数据
        print("检测到异常值，使用上一个有效位置")
        control_robot(result['x'], result['y'])
```

## 运行演示

```bash
python uwb_data_filter.py
```

输出示例：
```
============================================================
    UWB数据滤波器演示
============================================================

模拟数据测试（带异常值）：
------------------------------------------------------------
序号 |  原始距离 |    原始X |    原始Y |    滤波X |    滤波Y |   状态
------------------------------------------------------------
   1 |    141.4 |    100.0 |    100.0 |    100.0 |    100.0 |   正常
   2 |    142.1 |    101.5 |    100.2 |    100.5 |    100.1 |   正常
  ...
  11 |    500.2 |    302.5 |    250.2 |      -- |      -- |   丢弃
  12 |    143.8 |    102.3 |    101.5 |    101.2 |    100.6 |   正常
  ...

滤波统计：
  总数据点: 30
  异常值数: 2
  有效数据: 28
  异常率:   6.7%
```

## 常见问题

### Q: 滤波后数据有延迟怎么办？
A: 减小中位数窗口大小，或增大卡尔曼滤波的Q值。

### Q: 异常值检测太严格/太宽松？
A: 调整 `max_velocity`, `max_distance_jump`, `z_score_threshold` 参数。

### Q: 数据仍然不够平滑？
A: 增大卡尔曼滤波的R值，或启用中位数预滤波。
