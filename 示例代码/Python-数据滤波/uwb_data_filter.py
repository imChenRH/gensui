#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
UWB单基站跟随套件 - 数据滤波与异常值去除模块
============================================================================

功能说明：
    本模块提供多种数据滤波和异常值去除方法，用于处理UWB定位数据中的噪声和异常值。
    适用于机器狗跟随场景中因人员经过等原因导致的异常测量值。

主要方法：
    1. 卡尔曼滤波 (Kalman Filter) - 单点测量平滑
    2. 扩展卡尔曼滤波 (EKF) - 位置+速度状态估计（新增）
    3. 滑动窗口中位数滤波 (Median Filter)
    4. 滑动窗口均值滤波 (Moving Average)
    5. 指数加权移动平均 (EWMA)
    6. 基于速度/加速度的异常值检测
    7. 基于统计的异常值检测 (Z-Score / IQR)

EKF扩展卡尔曼滤波说明：
    为什么传感器已有KF还能用EKF？
    - 传感器KF：测量级滤波，仅平滑单个测量值
    - EKF：状态估计滤波，结合运动模型预测位置和速度
    
    EKF优势：
    - 预测目标运动趋势
    - 估计移动速度（用于机器狗控制）
    - 数据丢失时可预测位置

使用方法：
    from uwb_data_filter import UWBDataFilter, ExtendedKalmanFilter
    
    # 使用综合滤波器
    filter = UWBDataFilter()
    filtered_data = filter.filter(distance, x, y, z)
    
    # 使用EKF
    ekf = ExtendedKalmanFilter()
    result = ekf.update(x, y, z, current_time)
    # result包含: x, y, z (位置), vx, vy, vz (速度)
    
作者：Copilot
日期：2026-01-26
============================================================================
"""

import math
import time
from collections import deque
from typing import Tuple, Optional, List, Dict
import statistics


# ============================================================================
# 卡尔曼滤波器
# ============================================================================

class KalmanFilter1D:
    """
    一维卡尔曼滤波器
    
    适用场景：
        - 单个数值的平滑（如距离、角度）
        - 能够预测下一时刻的值
        - 对突变有一定的抑制作用
    
    优点：
        - 数学最优估计
        - 能处理过程噪声和测量噪声
        - 适合动态系统
    
    缺点：
        - 参数调节需要经验
        - 对非线性系统效果有限
    """
    
    def __init__(self, q=0.1, r=0.5, initial_value=0.0):
        """
        初始化卡尔曼滤波器
        
        参数：
            q: 过程噪声协方差（越大，滤波器越相信测量值）
            r: 测量噪声协方差（越大，滤波器越相信预测值）
            initial_value: 初始状态值
        """
        self.q = q  # 过程噪声
        self.r = r  # 测量噪声
        self.x = initial_value  # 状态估计值
        self.p = 1.0  # 估计误差协方差
        self.k = 0.0  # 卡尔曼增益
        self.initialized = False
    
    def update(self, measurement: float) -> float:
        """
        更新滤波器状态
        
        参数：
            measurement: 测量值
            
        返回：
            滤波后的估计值
        """
        if not self.initialized:
            self.x = measurement
            self.initialized = True
            return self.x
        
        # 预测步骤
        # x_pred = x (假设系统状态不变)
        p_pred = self.p + self.q
        
        # 更新步骤
        self.k = p_pred / (p_pred + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (1 - self.k) * p_pred
        
        return self.x
    
    def reset(self):
        """重置滤波器"""
        self.x = 0.0
        self.p = 1.0
        self.k = 0.0
        self.initialized = False


class KalmanFilter3D:
    """
    三维卡尔曼滤波器（用于X, Y, Z坐标同时滤波）
    """
    
    def __init__(self, q=0.1, r=0.5):
        """
        初始化三维卡尔曼滤波器
        
        参数：
            q: 过程噪声协方差
            r: 测量噪声协方差
        """
        self.filter_x = KalmanFilter1D(q, r)
        self.filter_y = KalmanFilter1D(q, r)
        self.filter_z = KalmanFilter1D(q, r)
    
    def update(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """
        更新三维坐标
        
        参数：
            x, y, z: 测量的坐标值
            
        返回：
            滤波后的 (x, y, z) 坐标
        """
        return (
            self.filter_x.update(x),
            self.filter_y.update(y),
            self.filter_z.update(z)
        )
    
    def reset(self):
        """重置滤波器"""
        self.filter_x.reset()
        self.filter_y.reset()
        self.filter_z.reset()


# ============================================================================
# 扩展卡尔曼滤波器 (EKF) - 用于目标跟踪的状态估计
# ============================================================================

class ExtendedKalmanFilter:
    """
    扩展卡尔曼滤波器 (EKF) - 用于机器狗跟随场景的目标跟踪
    
    为什么传感器已有KF还能用EKF？
    ================================
    - 传感器自带的KF：是**测量级滤波**，仅平滑单个测量值
    - EKF：是**状态估计滤波**，结合**运动模型**预测目标的位置和速度
    
    EKF的优势：
    1. 预测目标的运动趋势（即使短暂丢失数据也能预测位置）
    2. 融合速度状态，实现更精确的跟踪
    3. 对突变数据具有更强的抑制能力
    4. 能够输出目标速度信息，用于机器狗控制
    
    状态向量（6维）：
        x = [x, y, z, vx, vy, vz]^T
        其中 (x,y,z) 是位置，(vx,vy,vz) 是速度
    
    运动模型：恒速模型 (Constant Velocity Model)
        x_k = x_{k-1} + vx * dt
        vx_k = vx_{k-1}  （假设速度缓慢变化）
    
    参考：《Probabilistic Robotics》 (Thrun, Burgard, Fox)
    """
    
    def __init__(self, 
                 process_noise_pos: float = 0.5,     # 位置过程噪声
                 process_noise_vel: float = 2.0,     # 速度过程噪声
                 measurement_noise: float = 5.0):    # 测量噪声
        """
        初始化EKF
        
        参数：
            process_noise_pos: 位置过程噪声（Q矩阵对角元素）
            process_noise_vel: 速度过程噪声
            measurement_noise: 测量噪声（R矩阵对角元素）
        """
        self.n_states = 6
        self.n_measurements = 3
        
        self.x = None
        self.P = None
        
        self.Q = self._create_diagonal_matrix(
            [process_noise_pos, process_noise_pos, process_noise_pos,
             process_noise_vel, process_noise_vel, process_noise_vel]
        )
        
        self.R = self._create_diagonal_matrix(
            [measurement_noise, measurement_noise, measurement_noise]
        )
        
        self.H = [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ]
        
        self.last_time = None
        self.initialized = False
    
    def _create_diagonal_matrix(self, diagonal_values):
        n = len(diagonal_values)
        matrix = [[0.0] * n for _ in range(n)]
        for i in range(n):
            matrix[i][i] = diagonal_values[i]
        return matrix
    
    def _matrix_multiply(self, A, B):
        rows_A, cols_A = len(A), len(A[0])
        rows_B, cols_B = len(B), len(B[0])
        result = [[0.0] * cols_B for _ in range(rows_A)]
        for i in range(rows_A):
            for j in range(cols_B):
                for k in range(cols_A):
                    result[i][j] += A[i][k] * B[k][j]
        return result
    
    def _matrix_add(self, A, B):
        return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    
    def _matrix_subtract(self, A, B):
        return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    
    def _matrix_transpose(self, A):
        return [[A[j][i] for j in range(len(A))] for i in range(len(A[0]))]
    
    def _matrix_inverse_3x3(self, A):
        det = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
               A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
               A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))
        
        if abs(det) < 1e-10:
            for i in range(3):
                A[i][i] += 1e-6
            det = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                   A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                   A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))
        
        inv_det = 1.0 / det
        inv = [[0.0] * 3 for _ in range(3)]
        inv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det
        inv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det
        inv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det
        inv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det
        inv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det
        inv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det
        inv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det
        inv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det
        inv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det
        return inv
    
    def _create_state_transition_matrix(self, dt):
        F = [[0.0] * 6 for _ in range(6)]
        for i in range(6):
            F[i][i] = 1.0
        F[0][3] = dt
        F[1][4] = dt
        F[2][5] = dt
        return F
    
    def predict(self, dt: float):
        if not self.initialized:
            return
        
        F = self._create_state_transition_matrix(dt)
        x_pred = [[0.0] for _ in range(6)]
        for i in range(6):
            for j in range(6):
                x_pred[i][0] += F[i][j] * self.x[j][0]
        self.x = x_pred
        
        FP = self._matrix_multiply(F, self.P)
        F_T = self._matrix_transpose(F)
        FP_FT = self._matrix_multiply(FP, F_T)
        self.P = self._matrix_add(FP_FT, self.Q)
    
    def update(self, z_x: float, z_y: float, z_z: float, current_time: float = None) -> Dict:
        """
        更新EKF状态
        
        参数：
            z_x, z_y, z_z: 测量的位置坐标
            current_time: 当前时间戳
            
        返回：
            dict: 滤波后的位置和速度
        """
        if current_time is None:
            current_time = time.time()
        
        if not self.initialized:
            self.x = [[z_x], [z_y], [z_z], [0.0], [0.0], [0.0]]
            self.P = self._create_diagonal_matrix([10, 10, 10, 100, 100, 100])
            self.last_time = current_time
            self.initialized = True
            return {'x': z_x, 'y': z_y, 'z': z_z, 'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
        
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.01
        self.last_time = current_time
        
        self.predict(dt)
        
        z = [[z_x], [z_y], [z_z]]
        H_T = self._matrix_transpose(self.H)
        PH_T = self._matrix_multiply(self.P, H_T)
        HPH_T = self._matrix_multiply(self.H, PH_T)
        S = self._matrix_add(HPH_T, self.R)
        S_inv = self._matrix_inverse_3x3(S)
        K = self._matrix_multiply(PH_T, S_inv)
        
        Hx = [[0.0] for _ in range(3)]
        for i in range(3):
            for j in range(6):
                Hx[i][0] += self.H[i][j] * self.x[j][0]
        y = self._matrix_subtract(z, Hx)
        
        Ky = [[0.0] for _ in range(6)]
        for i in range(6):
            for j in range(3):
                Ky[i][0] += K[i][j] * y[j][0]
        self.x = self._matrix_add(self.x, Ky)
        
        KH = self._matrix_multiply(K, self.H)
        I = self._create_diagonal_matrix([1, 1, 1, 1, 1, 1])
        I_KH = self._matrix_subtract(I, KH)
        self.P = self._matrix_multiply(I_KH, self.P)
        
        return {
            'x': self.x[0][0], 'y': self.x[1][0], 'z': self.x[2][0],
            'vx': self.x[3][0], 'vy': self.x[4][0], 'vz': self.x[5][0]
        }
    
    def get_predicted_position(self, dt_ahead: float = 0.1) -> Optional[Dict]:
        """获取未来位置预测"""
        if not self.initialized:
            return None
        pred_x = self.x[0][0] + self.x[3][0] * dt_ahead
        pred_y = self.x[1][0] + self.x[4][0] * dt_ahead
        pred_z = self.x[2][0] + self.x[5][0] * dt_ahead
        return {'x': pred_x, 'y': pred_y, 'z': pred_z}
    
    def reset(self):
        """重置滤波器"""
        self.x = None
        self.P = None
        self.last_time = None
        self.initialized = False


# ============================================================================
# 滑动窗口中位数滤波器
# ============================================================================

class MedianFilter:
    """
    滑动窗口中位数滤波器
    
    适用场景：
        - 去除脉冲噪声（如人员遮挡导致的突变）
        - 对异常值具有很强的鲁棒性
    
    优点：
        - 能有效去除脉冲噪声和异常值
        - 不会像均值滤波那样被异常值拉偏
        - 保持边缘特性
    
    缺点：
        - 引入一定延迟
        - 窗口大小需要权衡
    """
    
    def __init__(self, window_size: int = 5):
        """
        初始化中位数滤波器
        
        参数：
            window_size: 滑动窗口大小（推荐奇数）
        """
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
    
    def update(self, value: float) -> float:
        """
        更新滤波器
        
        参数：
            value: 新的测量值
            
        返回：
            滤波后的值（窗口内的中位数）
        """
        self.buffer.append(value)
        return statistics.median(self.buffer)
    
    def reset(self):
        """重置滤波器"""
        self.buffer.clear()


class MedianFilter3D:
    """三维中位数滤波器"""
    
    def __init__(self, window_size: int = 5):
        self.filter_x = MedianFilter(window_size)
        self.filter_y = MedianFilter(window_size)
        self.filter_z = MedianFilter(window_size)
    
    def update(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        return (
            self.filter_x.update(x),
            self.filter_y.update(y),
            self.filter_z.update(z)
        )
    
    def reset(self):
        self.filter_x.reset()
        self.filter_y.reset()
        self.filter_z.reset()


# ============================================================================
# 指数加权移动平均滤波器 (EWMA)
# ============================================================================

class EWMAFilter:
    """
    指数加权移动平均滤波器
    
    适用场景：
        - 需要快速响应但也需要平滑的场景
        - 跟踪缓慢变化的信号
    
    优点：
        - 计算简单
        - 可调节的平滑程度
        - 对新数据有即时响应
    
    缺点：
        - 对异常值仍然敏感
        - 需要配合异常值检测使用
    """
    
    def __init__(self, alpha: float = 0.3):
        """
        初始化EWMA滤波器
        
        参数：
            alpha: 平滑因子，范围 (0, 1)
                   - 越大，越相信新测量值，响应越快
                   - 越小，平滑效果越强，但响应越慢
                   - 推荐值：0.1-0.5
        """
        self.alpha = alpha
        self.value = None
    
    def update(self, measurement: float) -> float:
        """
        更新滤波器
        
        参数：
            measurement: 新的测量值
            
        返回：
            滤波后的值
        """
        if self.value is None:
            self.value = measurement
        else:
            self.value = self.alpha * measurement + (1 - self.alpha) * self.value
        return self.value
    
    def reset(self):
        """重置滤波器"""
        self.value = None


class EWMAFilter3D:
    """三维EWMA滤波器"""
    
    def __init__(self, alpha: float = 0.3):
        self.filter_x = EWMAFilter(alpha)
        self.filter_y = EWMAFilter(alpha)
        self.filter_z = EWMAFilter(alpha)
    
    def update(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        return (
            self.filter_x.update(x),
            self.filter_y.update(y),
            self.filter_z.update(z)
        )
    
    def reset(self):
        self.filter_x.reset()
        self.filter_y.reset()
        self.filter_z.reset()


# ============================================================================
# 异常值检测器
# ============================================================================

class OutlierDetector:
    """
    异常值检测器
    
    提供多种异常值检测方法：
    1. 基于速度的检测：如果位置变化速度超过阈值，判定为异常
    2. 基于统计的检测：使用Z-Score或IQR方法
    3. 基于距离跳变的检测：如果距离突然变化过大，判定为异常
    """
    
    def __init__(self, 
                 max_velocity: float = 200.0,  # 最大允许速度 (cm/s)
                 max_distance_jump: float = 50.0,  # 最大允许距离跳变 (cm)
                 z_score_threshold: float = 3.0,  # Z-Score阈值
                 window_size: int = 10):  # 统计窗口大小
        """
        初始化异常值检测器
        
        参数：
            max_velocity: 最大允许移动速度 (cm/s)，超过此速度的数据点被视为异常
            max_distance_jump: 单次最大允许距离变化 (cm)
            z_score_threshold: Z-Score阈值，超过此值的数据点被视为异常
            window_size: 用于统计的滑动窗口大小
        """
        self.max_velocity = max_velocity
        self.max_distance_jump = max_distance_jump
        self.z_score_threshold = z_score_threshold
        self.window_size = window_size
        
        # 历史数据
        self.last_position = None
        self.last_time = None
        self.history_x = deque(maxlen=window_size)
        self.history_y = deque(maxlen=window_size)
        self.history_z = deque(maxlen=window_size)
        self.history_distance = deque(maxlen=window_size)
    
    def is_outlier_by_velocity(self, x: float, y: float, z: float, 
                                current_time: float = None) -> bool:
        """
        基于速度判断是否为异常值
        
        原理：人的正常移动速度有限，如果测量的位置变化速度过快，则可能是异常
        
        参数：
            x, y, z: 当前测量的坐标
            current_time: 当前时间戳（秒）
            
        返回：
            True 表示是异常值
        """
        if current_time is None:
            current_time = time.time()
        
        if self.last_position is None or self.last_time is None:
            self.last_position = (x, y, z)
            self.last_time = current_time
            return False
        
        # 计算位移
        dx = x - self.last_position[0]
        dy = y - self.last_position[1]
        dz = z - self.last_position[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # 计算时间间隔
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.01  # 防止除零
        
        # 计算速度
        velocity = distance / dt
        
        is_outlier = velocity > self.max_velocity
        
        if not is_outlier:
            # 只有非异常值才更新历史位置
            self.last_position = (x, y, z)
            self.last_time = current_time
        
        return is_outlier
    
    def is_outlier_by_distance_jump(self, distance: float) -> bool:
        """
        基于距离跳变判断是否为异常值
        
        原理：UWB测距通常是连续变化的，如果距离突然大幅变化，可能是被遮挡
        
        参数：
            distance: 当前测量的距离
            
        返回：
            True 表示是异常值
        """
        if len(self.history_distance) == 0:
            self.history_distance.append(distance)
            return False
        
        last_distance = self.history_distance[-1]
        jump = abs(distance - last_distance)
        
        is_outlier = jump > self.max_distance_jump
        
        if not is_outlier:
            self.history_distance.append(distance)
        
        return is_outlier
    
    def is_outlier_by_zscore(self, x: float, y: float, z: float) -> bool:
        """
        基于Z-Score判断是否为异常值
        
        原理：假设数据服从正态分布，Z-Score表示数据点偏离均值的标准差倍数
        
        参数：
            x, y, z: 当前测量的坐标
            
        返回：
            True 表示是异常值
        """
        if len(self.history_x) < 3:
            self.history_x.append(x)
            self.history_y.append(y)
            self.history_z.append(z)
            return False
        
        # 计算各维度的Z-Score
        z_scores = []
        for value, history in [(x, self.history_x), 
                                (y, self.history_y), 
                                (z, self.history_z)]:
            mean = statistics.mean(history)
            std = statistics.stdev(history) if len(history) > 1 else 1.0
            if std < 0.001:
                std = 0.001  # 防止除零
            z_score = abs(value - mean) / std
            z_scores.append(z_score)
        
        # 如果任何一个维度的Z-Score超过阈值，则判定为异常
        max_z_score = max(z_scores)
        is_outlier = max_z_score > self.z_score_threshold
        
        if not is_outlier:
            self.history_x.append(x)
            self.history_y.append(y)
            self.history_z.append(z)
        
        return is_outlier
    
    def reset(self):
        """重置检测器"""
        self.last_position = None
        self.last_time = None
        self.history_x.clear()
        self.history_y.clear()
        self.history_z.clear()
        self.history_distance.clear()


# ============================================================================
# 综合数据滤波器
# ============================================================================

class UWBDataFilter:
    """
    UWB数据综合滤波器
    
    结合异常值检测和卡尔曼滤波，提供完整的数据处理流程：
    1. 首先进行异常值检测，丢弃明显异常的数据
    2. 然后使用卡尔曼滤波进行平滑
    
    适用于机器狗跟随场景中的UWB定位数据处理
    """
    
    def __init__(self,
                 # 异常值检测参数
                 max_velocity: float = 200.0,  # 最大速度 cm/s
                 max_distance_jump: float = 50.0,  # 最大距离跳变 cm
                 z_score_threshold: float = 3.0,
                 use_zscore_detection: bool = True,  # 是否启用Z-Score检测
                 # 卡尔曼滤波参数
                 kalman_q: float = 0.1,
                 kalman_r: float = 0.5,
                 # 其他参数
                 use_median_prefilter: bool = True,
                 median_window: int = 3):
        """
        初始化综合滤波器
        
        参数：
            max_velocity: 最大允许移动速度 (cm/s)
            max_distance_jump: 最大允许距离跳变 (cm)
            z_score_threshold: Z-Score异常值阈值
            use_zscore_detection: 是否启用Z-Score检测
            kalman_q: 卡尔曼滤波过程噪声
            kalman_r: 卡尔曼滤波测量噪声
            use_median_prefilter: 是否使用中位数预滤波
            median_window: 中位数滤波窗口大小
        """
        # 异常值检测器
        self.outlier_detector = OutlierDetector(
            max_velocity=max_velocity,
            max_distance_jump=max_distance_jump,
            z_score_threshold=z_score_threshold
        )
        self.use_zscore_detection = use_zscore_detection
        
        # 中位数预滤波器
        self.use_median_prefilter = use_median_prefilter
        self.median_filter = MedianFilter3D(median_window) if use_median_prefilter else None
        
        # 卡尔曼滤波器
        self.kalman_filter = KalmanFilter3D(q=kalman_q, r=kalman_r)
        
        # 距离滤波器
        self.distance_kalman = KalmanFilter1D(q=kalman_q, r=kalman_r)
        
        # 统计信息
        self.total_count = 0
        self.outlier_count = 0
        self.last_valid_data = None
    
    def filter(self, distance: float, x: float, y: float, z: float,
               current_time: float = None) -> Optional[Dict]:
        """
        滤波处理一个数据点
        
        参数：
            distance: 原始距离值 (cm)
            x, y, z: 原始坐标值 (cm)
            current_time: 当前时间戳（秒），可选
            
        返回：
            滤波后的数据字典，如果数据被判定为异常则返回None
            {
                'distance': 滤波后的距离,
                'x': 滤波后的X坐标,
                'y': 滤波后的Y坐标,
                'z': 滤波后的Z坐标,
                'is_outlier': 是否被检测为异常值
            }
        """
        if current_time is None:
            current_time = time.time()
        
        self.total_count += 1
        
        # 步骤1: 异常值检测
        is_outlier = False
        
        # 检测距离跳变
        if self.outlier_detector.is_outlier_by_distance_jump(distance):
            is_outlier = True
        
        # 检测速度异常
        if not is_outlier and self.outlier_detector.is_outlier_by_velocity(x, y, z, current_time):
            is_outlier = True
        
        # Z-Score检测（可选）
        if not is_outlier and self.use_zscore_detection:
            if self.outlier_detector.is_outlier_by_zscore(x, y, z):
                is_outlier = True
        
        if is_outlier:
            self.outlier_count += 1
            # 返回上一个有效数据（如果有的话）
            if self.last_valid_data:
                return {**self.last_valid_data, 'is_outlier': True, 'is_interpolated': True}
            return None
        
        # 步骤2: 中位数预滤波（可选）
        if self.use_median_prefilter:
            x, y, z = self.median_filter.update(x, y, z)
        
        # 步骤3: 卡尔曼滤波
        filtered_x, filtered_y, filtered_z = self.kalman_filter.update(x, y, z)
        filtered_distance = self.distance_kalman.update(distance)
        
        result = {
            'distance': round(filtered_distance, 2),
            'x': round(filtered_x, 2),
            'y': round(filtered_y, 2),
            'z': round(filtered_z, 2),
            'is_outlier': False,
            'is_interpolated': False
        }
        
        self.last_valid_data = result.copy()
        return result
    
    def get_statistics(self) -> Dict:
        """获取滤波统计信息"""
        outlier_rate = self.outlier_count / self.total_count if self.total_count > 0 else 0
        return {
            'total_count': self.total_count,
            'outlier_count': self.outlier_count,
            'valid_count': self.total_count - self.outlier_count,
            'outlier_rate': f"{outlier_rate * 100:.1f}%"
        }
    
    def reset(self):
        """重置滤波器"""
        self.outlier_detector.reset()
        if self.median_filter:
            self.median_filter.reset()
        self.kalman_filter.reset()
        self.distance_kalman.reset()
        self.total_count = 0
        self.outlier_count = 0
        self.last_valid_data = None


# ============================================================================
# 使用示例
# ============================================================================

def demo():
    """演示滤波器的使用"""
    import random
    
    print("="*60)
    print("    UWB数据滤波器演示")
    print("="*60)
    
    # 创建滤波器（禁用Z-Score，主要使用距离跳变检测）
    filter = UWBDataFilter(
        max_velocity=500.0,        # 最大速度 500 cm/s
        max_distance_jump=120.0,   # 最大距离跳变 120 cm（检测突然跳变）
        use_zscore_detection=False, # 禁用Z-Score（对渐变数据不适用）
        kalman_q=0.5,              # 卡尔曼过程噪声（快速响应）
        kalman_r=0.3,              # 卡尔曼测量噪声
        use_median_prefilter=False, # 关闭中位数预滤波
        median_window=3
    )
    
    print("\n模拟数据测试（第11和21个数据点模拟人员遮挡导致的异常跳变）：")
    print("-"*80)
    print(f"{'序号':>4} | {'原始距离':>8} | {'原始X':>8} | {'原始Y':>8} | {'滤波X':>8} | {'滤波Y':>8} | {'状态':>8}")
    print("-"*80)
    
    # 模拟正常移动 + 一些异常值
    base_x, base_y, base_z = 100.0, 100.0, 0.0
    
    for i in range(30):
        # 正常移动（缓慢变化）
        base_x += random.uniform(-2, 3)
        base_y += random.uniform(-2, 3)
        base_z += random.uniform(-0.5, 0.5)
        
        # 添加测量噪声
        noise_x = random.uniform(-3, 3)
        noise_y = random.uniform(-3, 3)
        noise_z = random.uniform(-1, 1)
        
        raw_x = base_x + noise_x
        raw_y = base_y + noise_y
        raw_z = base_z + noise_z
        
        # 每隔一段时间添加异常值（模拟人员遮挡）
        if i == 10 or i == 20:
            raw_x += 200  # 突然跳变
            raw_y += 150
        
        raw_distance = math.sqrt(raw_x**2 + raw_y**2 + raw_z**2)
        
        # 滤波
        result = filter.filter(raw_distance, raw_x, raw_y, raw_z)
        
        if result:
            status = "异常→插值" if result['is_outlier'] else "正常"
            print(f"{i+1:>4} | {raw_distance:>8.1f} | {raw_x:>8.1f} | {raw_y:>8.1f} | "
                  f"{result['x']:>8.1f} | {result['y']:>8.1f} | {status:>8}")
        else:
            print(f"{i+1:>4} | {raw_distance:>8.1f} | {raw_x:>8.1f} | {raw_y:>8.1f} | "
                  f"{'--':>8} | {'--':>8} | {'丢弃':>8}")
        
        time.sleep(0.05)
    
    # 打印统计信息
    stats = filter.get_statistics()
    print("-"*80)
    print(f"\n滤波统计：")
    print(f"  总数据点: {stats['total_count']}")
    print(f"  异常值数: {stats['outlier_count']}")
    print(f"  有效数据: {stats['valid_count']}")
    print(f"  异常率:   {stats['outlier_rate']}")


if __name__ == '__main__':
    demo()
