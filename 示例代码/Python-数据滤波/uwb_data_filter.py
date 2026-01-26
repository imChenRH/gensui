#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
UWB单基站跟随套件 - 数据滤波与异常值去除模块
============================================================================

功能说明：
    本模块提供数据滤波和异常值去除功能，用于处理UWB定位数据中的噪声和异常值。
    适用于机器狗跟随场景中因人员经过等原因导致的异常测量值。

滤波流程：
    1. 物理约束检查 - 检查数据是否在合理范围内
    2. 中位数滤波 - 消除尖峰异常值
    3. EKF状态估计 - 位置平滑 + 速度估计

使用方法：
    from uwb_data_filter import UWBDataFilter
    
    # 创建滤波器
    filter = UWBDataFilter()
    
    # 滤波处理
    result = filter.filter(distance, azimuth, elevation, x, y, z)
    if result and not result['is_outlier']:
        # 使用滤波后的数据
        print(f"位置: ({result['x']}, {result['y']}, {result['z']})")
        print(f"速度: ({result['vx']}, {result['vy']}, {result['vz']})")

作者：Copilot
日期：2026-01-26
============================================================================
"""

import math
import time
from collections import deque
from typing import Optional, Dict, Tuple

try:
    import numpy as np
except ImportError:
    print("错误：请安装 numpy")
    print("  pip install numpy")
    raise


# ============================================================================
# 配置参数
# ============================================================================

# 物理约束参数
MIN_DISTANCE = 5.0          # 最小距离 (cm)
MAX_DISTANCE = 5000.0       # 最大距离 (cm)
MAX_VELOCITY = 300.0        # 最大速度 (cm/s)
MIN_AZIMUTH = -180.0        # 最小方位角 (度)
MAX_AZIMUTH = 180.0         # 最大方位角 (度)
MIN_ELEVATION = -90.0       # 最小仰角 (度)
MAX_ELEVATION = 90.0        # 最大仰角 (度)

# 中位数滤波参数
MEDIAN_WINDOW_SIZE = 5      # 中位数滤波窗口大小

# EKF参数
EKF_PROCESS_NOISE = 0.5     # EKF过程噪声
EKF_MEASUREMENT_NOISE = 1.0 # EKF测量噪声


# ============================================================================
# 中位数滤波器
# ============================================================================

class MedianFilter:
    """一维中位数滤波器"""
    
    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
    
    def update(self, value: float) -> float:
        """更新并返回滤波值"""
        self.buffer.append(value)
        if len(self.buffer) < self.window_size:
            return value
        sorted_values = sorted(self.buffer)
        return sorted_values[len(sorted_values) // 2]
    
    def reset(self):
        """重置"""
        self.buffer.clear()


class MedianFilter3D:
    """三维中位数滤波器"""
    
    def __init__(self, window_size: int = 5):
        self.filter_x = MedianFilter(window_size)
        self.filter_y = MedianFilter(window_size)
        self.filter_z = MedianFilter(window_size)
    
    def update(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """更新并返回滤波值"""
        return (
            self.filter_x.update(x),
            self.filter_y.update(y),
            self.filter_z.update(z)
        )
    
    def reset(self):
        """重置"""
        self.filter_x.reset()
        self.filter_y.reset()
        self.filter_z.reset()


# ============================================================================
# 扩展卡尔曼滤波器 (EKF)
# ============================================================================

class ExtendedKalmanFilter:
    """
    扩展卡尔曼滤波器 (EKF) - 用于目标跟踪
    
    状态向量: [x, y, z, vx, vy, vz]
    - x, y, z: 位置
    - vx, vy, vz: 速度
    
    使用恒速运动模型
    """
    
    def __init__(self, process_noise: float = 0.5, measurement_noise: float = 1.0):
        # 状态向量 [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)
        
        # 状态协方差矩阵
        self.P = np.eye(6) * 100.0
        
        # 过程噪声协方差
        self.Q = np.eye(6) * process_noise
        self.Q[3:6, 3:6] *= 2.0  # 速度噪声更大
        
        # 测量噪声协方差
        self.R = np.eye(3) * measurement_noise
        
        # 测量矩阵 (只测量位置)
        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0
        
        self.last_time = None
        self.initialized = False
    
    def update(self, x: float, y: float, z: float, 
               current_time: float = None) -> Dict:
        """
        更新EKF状态
        
        返回：
            {
                'x', 'y', 'z': 滤波后位置,
                'vx', 'vy', 'vz': 估计速度
            }
        """
        if current_time is None:
            current_time = time.time()
        
        measurement = np.array([x, y, z])
        
        # 初始化
        if not self.initialized:
            self.x[:3] = measurement
            self.x[3:6] = 0.0
            self.last_time = current_time
            self.initialized = True
            return self._get_result()
        
        # 计算时间间隔
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.1
        dt = min(dt, 1.0)  # 限制最大时间间隔
        self.last_time = current_time
        
        # 状态转移矩阵
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        
        # 预测步骤
        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + self.Q
        
        # 更新步骤
        y_residual = measurement - self.H @ x_pred
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)
        
        self.x = x_pred + K @ y_residual
        self.P = (np.eye(6) - K @ self.H) @ P_pred
        
        return self._get_result()
    
    def _get_result(self) -> Dict:
        """获取结果字典"""
        return {
            'x': float(self.x[0]),
            'y': float(self.x[1]),
            'z': float(self.x[2]),
            'vx': float(self.x[3]),
            'vy': float(self.x[4]),
            'vz': float(self.x[5])
        }
    
    def predict(self, dt: float) -> Dict:
        """预测dt秒后的位置"""
        x_pred = self.x[0] + self.x[3] * dt
        y_pred = self.x[1] + self.x[4] * dt
        z_pred = self.x[2] + self.x[5] * dt
        return {'x': x_pred, 'y': y_pred, 'z': z_pred}
    
    def reset(self):
        """重置"""
        self.x = np.zeros(6)
        self.P = np.eye(6) * 100.0
        self.last_time = None
        self.initialized = False


# ============================================================================
# 物理约束检查器
# ============================================================================

class PhysicalConstraintChecker:
    """物理约束检查器"""
    
    def __init__(self, max_velocity: float = MAX_VELOCITY):
        self.max_velocity = max_velocity
        self.last_pos = None
        self.last_time = None
    
    def check(self, x: float, y: float, z: float, 
              distance: float, azimuth: float, elevation: float,
              current_time: float = None) -> bool:
        """
        检查数据是否满足物理约束
        
        返回：
            True = 数据有效
            False = 数据异常
        """
        if current_time is None:
            current_time = time.time()
        
        # 检查距离范围
        if distance < MIN_DISTANCE or distance > MAX_DISTANCE:
            return False
        
        # 检查角度范围
        if azimuth < MIN_AZIMUTH or azimuth > MAX_AZIMUTH:
            return False
        if elevation < MIN_ELEVATION or elevation > MAX_ELEVATION:
            return False
        
        # 检查速度约束
        if self.last_pos is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                dx = x - self.last_pos[0]
                dy = y - self.last_pos[1]
                dz = z - self.last_pos[2]
                velocity = math.sqrt(dx*dx + dy*dy + dz*dz) / dt
                
                if velocity > self.max_velocity:
                    return False
        
        # 更新上一次位置
        self.last_pos = (x, y, z)
        self.last_time = current_time
        
        return True
    
    def reset(self):
        """重置"""
        self.last_pos = None
        self.last_time = None


# ============================================================================
# 综合滤波器
# ============================================================================

class UWBDataFilter:
    """
    UWB数据综合滤波器
    
    滤波流程：
        1. 物理约束检查
        2. 中位数滤波
        3. EKF状态估计
    """
    
    def __init__(self, 
                 max_velocity: float = MAX_VELOCITY,
                 median_window: int = MEDIAN_WINDOW_SIZE,
                 ekf_process_noise: float = EKF_PROCESS_NOISE,
                 ekf_measurement_noise: float = EKF_MEASUREMENT_NOISE):
        """
        初始化滤波器
        
        参数：
            max_velocity: 最大允许速度 (cm/s)
            median_window: 中位数滤波窗口大小
            ekf_process_noise: EKF过程噪声
            ekf_measurement_noise: EKF测量噪声
        """
        # 物理约束检查器
        self.constraint_checker = PhysicalConstraintChecker(max_velocity)
        
        # 中位数滤波器
        self.median_filter = MedianFilter3D(median_window)
        
        # EKF滤波器
        self.ekf = ExtendedKalmanFilter(ekf_process_noise, ekf_measurement_noise)
        
        # 统计
        self.total_count = 0
        self.outlier_count = 0
        self.last_valid_result = None
    
    def filter(self, distance: float, azimuth: float, elevation: float,
               x: float, y: float, z: float,
               current_time: float = None) -> Optional[Dict]:
        """
        滤波处理
        
        参数：
            distance: 距离 (cm)
            azimuth: 方位角 (度)
            elevation: 仰角 (度)
            x, y, z: 笛卡尔坐标 (cm)
            current_time: 时间戳 (秒)
        
        返回：
            {
                'x', 'y', 'z': 滤波后位置,
                'vx', 'vy', 'vz': 速度估计,
                'is_outlier': 是否为异常值
            }
            如果数据被丢弃且无历史数据，返回None
        """
        if current_time is None:
            current_time = time.time()
        
        self.total_count += 1
        
        # Step 1: 物理约束检查
        is_valid = self.constraint_checker.check(
            x, y, z, distance, azimuth, elevation, current_time
        )
        
        if not is_valid:
            self.outlier_count += 1
            if self.last_valid_result:
                return {**self.last_valid_result, 'is_outlier': True}
            return None
        
        # Step 2: 中位数滤波
        med_x, med_y, med_z = self.median_filter.update(x, y, z)
        
        # Step 3: EKF状态估计
        ekf_result = self.ekf.update(med_x, med_y, med_z, current_time)
        
        result = {
            'x': ekf_result['x'],
            'y': ekf_result['y'],
            'z': ekf_result['z'],
            'vx': ekf_result['vx'],
            'vy': ekf_result['vy'],
            'vz': ekf_result['vz'],
            'is_outlier': False
        }
        
        self.last_valid_result = result
        return result
    
    def get_statistics(self) -> Dict:
        """获取统计信息"""
        rate = self.outlier_count / self.total_count if self.total_count > 0 else 0
        return {
            'total': self.total_count,
            'outlier': self.outlier_count,
            'valid': self.total_count - self.outlier_count,
            'outlier_rate': f"{rate * 100:.1f}%"
        }
    
    def reset(self):
        """重置滤波器"""
        self.constraint_checker.reset()
        self.median_filter.reset()
        self.ekf.reset()
        self.total_count = 0
        self.outlier_count = 0
        self.last_valid_result = None


# ============================================================================
# 导出
# ============================================================================

__all__ = [
    'UWBDataFilter',
    'ExtendedKalmanFilter',
    'MedianFilter',
    'MedianFilter3D',
    'PhysicalConstraintChecker',
]


# ============================================================================
# 测试代码
# ============================================================================

if __name__ == '__main__':
    print("="*60)
    print("    UWB数据滤波模块测试")
    print("="*60)
    print("\n本模块提供以下类：")
    print("  - UWBDataFilter: 综合滤波器（物理约束+中位数+EKF）")
    print("  - ExtendedKalmanFilter: EKF扩展卡尔曼滤波器")
    print("  - MedianFilter3D: 三维中位数滤波器")
    print("  - PhysicalConstraintChecker: 物理约束检查器")
    print("\n使用示例：")
    print("  from uwb_data_filter import UWBDataFilter")
    print("  filter = UWBDataFilter()")
    print("  result = filter.filter(distance, azimuth, elevation, x, y, z)")
    print("\n滤波流程：")
    print("  1. 物理约束检查（范围+速度限制）")
    print("  2. 中位数滤波（窗口=5）")
    print("  3. EKF状态估计（位置+速度）")
