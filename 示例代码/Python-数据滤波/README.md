# UWB单基站跟随套件 - 数据滤波与异常值去除模块

## 问题背景

在机器狗跟随场景中，UWB定位数据可能因为以下原因出现异常值：
- **人员经过**：其他人经过基站和标签之间，造成信号遮挡
- **多径效应**：信号反射导致距离测量偏大
- **环境干扰**：金属物体、电磁干扰等
- **信号丢失**：临时失去测距连接

这些异常值如果不处理，会导致机器狗出现突然的位置跳变，影响跟随效果。

## 滤波流程

本模块提供完整的三步滤波流程：

| 步骤 | 方法 | 说明 |
|------|------|------|
| Step 1 | 物理约束检查 | 范围限制 + 速度限制 |
| Step 2 | 中位数滤波 | 消除尖峰异常值 |
| Step 3 | EKF状态估计 | 位置平滑 + 速度估计 |

## 使用方法

### 基本用法

```python
from uwb_data_filter import UWBDataFilter

# 创建滤波器
filter = UWBDataFilter()

# 滤波处理
result = filter.filter(
    distance=100,      # 距离 (cm)
    azimuth=30,        # 方位角 (度)
    elevation=5,       # 仰角 (度)
    x=50,              # X坐标 (cm)
    y=86,              # Y坐标 (cm)
    z=8                # Z坐标 (cm)
)

if result and not result['is_outlier']:
    # 使用滤波后的数据
    print(f"位置: ({result['x']:.1f}, {result['y']:.1f}, {result['z']:.1f})")
    print(f"速度: ({result['vx']:.1f}, {result['vy']:.1f}, {result['vz']:.1f})")
```

### 自定义参数

```python
from uwb_data_filter import UWBDataFilter

filter = UWBDataFilter(
    max_velocity=300.0,         # 最大速度 (cm/s)
    median_window=5,            # 中位数窗口大小
    ekf_process_noise=0.5,      # EKF过程噪声
    ekf_measurement_noise=1.0   # EKF测量噪声
)
```

### 获取统计信息

```python
stats = filter.get_statistics()
print(f"总数据: {stats['total']}")
print(f"异常值: {stats['outlier']}")
print(f"有效数据: {stats['valid']}")
print(f"异常率: {stats['outlier_rate']}")
```

## 模块说明

### UWBDataFilter

综合滤波器，整合了物理约束检查、中位数滤波和EKF。

### ExtendedKalmanFilter

扩展卡尔曼滤波器，用于目标跟踪。

状态向量：`[x, y, z, vx, vy, vz]`
- `x, y, z`：位置
- `vx, vy, vz`：速度

使用恒速运动模型。

```python
from uwb_data_filter import ExtendedKalmanFilter

ekf = ExtendedKalmanFilter(process_noise=0.5, measurement_noise=1.0)
result = ekf.update(x, y, z)
# result = {'x', 'y', 'z', 'vx', 'vy', 'vz'}
```

### MedianFilter3D

三维中位数滤波器，对尖峰异常值非常有效。

```python
from uwb_data_filter import MedianFilter3D

mf = MedianFilter3D(window_size=5)
filtered_x, filtered_y, filtered_z = mf.update(x, y, z)
```

### PhysicalConstraintChecker

物理约束检查器，检查数据是否在合理范围内。

```python
from uwb_data_filter import PhysicalConstraintChecker

checker = PhysicalConstraintChecker(max_velocity=300.0)
is_valid = checker.check(x, y, z, distance, azimuth, elevation)
```

## 参数说明

### 物理约束参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| MIN_DISTANCE | 5 cm | 最小有效距离 |
| MAX_DISTANCE | 5000 cm | 最大有效距离 |
| MAX_VELOCITY | 300 cm/s | 最大移动速度（人小跑速度） |

### EKF参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| process_noise | 0.5 | 过程噪声（越大响应越快） |
| measurement_noise | 1.0 | 测量噪声（越大越平滑） |

## 依赖安装

```bash
pip install numpy
```

## 参考资料

- 《Probabilistic Robotics》 (Thrun, Burgard, Fox)
- scipy.signal
