#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
UWB单基站跟随套件 - 原始数据与滤波数据对比可视化程序（支持EKF）
============================================================================

功能说明：
    本程序从UWB基站接收串口数据，同时显示原始数据和滤波后数据的移动路径。
    使用六个二维平面视图进行对比：
    - 上排：原始数据的俯视图(X-Y)、前视图(X-Z)、侧视图(Y-Z)
    - 下排：滤波后数据的俯视图(X-Y)、前视图(X-Z)、侧视图(Y-Z)

滤波算法（两种模式）：

【标准模式】两步滤波流程：
    Step 1 (物理约束检查): 检测数据是否在物理可能的范围内
    Step 2 (中位数滤波): 使用窗口为5的中位数滤波消除尖峰异常
    注意：传感器数据已自带卡尔曼滤波，因此默认不再额外添加

【EKF模式】三步滤波流程（使用 --ekf 参数启用）：
    Step 1 (物理约束检查): 检测数据是否在物理可能的范围内
    Step 2 (中位数滤波): 使用窗口为5的中位数滤波消除尖峰异常
    Step 3 (EKF状态估计): 扩展卡尔曼滤波（位置+速度状态估计）
    
    为什么传感器已有KF还能用EKF？
    - 传感器KF：测量级滤波，仅平滑单个测量值
    - EKF：状态估计滤波，结合运动模型预测位置和速度
    - EKF优势：预测目标位置、估计移动速度、数据丢失时可预测

硬件连接：
    - UWB基站通过USB转TTL模块连接到电脑
    - 串口参数：115200波特率，8数据位，1停止位，无校验

依赖安装：
    pip install pyserial matplotlib numpy

使用方法：
    python uwb_filtered_visualizer.py [串口号]           # 标准模式
    python uwb_filtered_visualizer.py COM3 --ekf        # EKF模式
    python uwb_filtered_visualizer.py --simulate        # 模拟模式
    python uwb_filtered_visualizer.py --simulate --ekf  # 模拟模式+EKF
    
参考资料：
    - "Probabilistic Robotics" (Thrun, Burgard, Fox)
    - scipy.signal (medfilt, lfilter)
    
作者：Copilot
日期：2026-01-26
============================================================================
"""

import sys
import math
import struct
import threading
import time
from collections import deque
from typing import Optional, Dict
import statistics

# 尝试导入依赖库
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误：请先安装 pyserial 库")
    print("运行命令：pip install pyserial")
    sys.exit(1)

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import numpy as np
except ImportError:
    print("错误：请先安装 matplotlib 和 numpy 库")
    print("运行命令：pip install matplotlib numpy")
    sys.exit(1)

# ============================================================================
# 全局配置
# ============================================================================

# 协议相关常量
FRAME_HEADER = b'\xff\xff\xff\xff'  # 帧头
FRAME_LENGTH = 37                    # 数据帧长度（字节）
CMD_POSITION = 0x2001                # 位置数据命令字

# 可视化配置
MAX_PATH_POINTS = 100       # 最大保留路径点数
UPDATE_INTERVAL = 50        # 图形更新间隔（毫秒）
DISPLAY_RANGE = 200         # 默认显示范围（厘米）
PATH_FADE_TIME = 5.0        # 路径渐隐时间（秒）

# ============================================================================
# 物理约束参数（用于机器狗跟随场景）
# ============================================================================
# 距离约束（单位：厘米）
MIN_DISTANCE = 5            # 最小有效距离
MAX_DISTANCE = 5000         # 最大有效距离（50米）

# 角度约束（单位：度）
MIN_AZIMUTH = -180          # 方位角最小值
MAX_AZIMUTH = 180           # 方位角最大值
MIN_ELEVATION = -90         # 仰角最小值（向下看）
MAX_ELEVATION = 90          # 仰角最大值（向上看）

# 速度约束（单位：厘米/秒）
# 人正常行走约 120cm/s，快走约 200cm/s，小跑约 300cm/s
MAX_VELOCITY = 300          # 最大允许速度

# 中位数滤波窗口大小（必须为奇数）
MEDIAN_WINDOW_SIZE = 5


# ============================================================================
# 中位数滤波器（消除尖峰异常值的"银弹"）
# ============================================================================

class MedianFilter:
    """
    中位数滤波器 - 最有效的尖峰消除方法
    
    原理：保持最近N个读数的缓冲区，排序后取中间值。
    优点：完全忽略异常尖峰，不会像平均值那样被异常值拉偏。
    """
    
    def __init__(self, window_size: int = 5):
        """
        初始化中位数滤波器
        
        Args:
            window_size: 窗口大小（必须为奇数，如3或5）
        """
        # 确保窗口大小为奇数
        if window_size % 2 == 0:
            window_size += 1
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
    
    def filter(self, value: float) -> float:
        """
        对新值进行中位数滤波
        
        Args:
            value: 新的测量值
            
        Returns:
            中位数滤波后的值
        """
        self.buffer.append(value)
        
        # 数据不足时直接返回当前值
        if len(self.buffer) < self.window_size:
            return value
        
        # 返回中位数（中间值）
        return statistics.median(self.buffer)
    
    def reset(self):
        """重置滤波器状态"""
        self.buffer.clear()


class AngleMedianFilter:
    """
    角度中位数滤波器 - 处理0°/360°边界问题
    
    警告：处理方位角（航向）时需要特别注意环绕问题。
    1°和359°的平均值应该是0°，而不是180°。
    必须在滤波前对角度进行归一化处理。
    """
    
    def __init__(self, window_size: int = 5):
        """
        初始化角度中位数滤波器
        
        Args:
            window_size: 窗口大小（必须为奇数）
        """
        if window_size % 2 == 0:
            window_size += 1
        self.window_size = window_size
        self.buffer = deque(maxlen=window_size)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        将角度归一化到 [-180, 180) 范围
        
        Args:
            angle: 输入角度（度）
            
        Returns:
            归一化后的角度
        """
        while angle >= 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    @staticmethod
    def angle_difference(a1: float, a2: float) -> float:
        """
        计算两个角度之间的最短差值
        
        Args:
            a1: 第一个角度
            a2: 第二个角度
            
        Returns:
            角度差（-180到180之间）
        """
        diff = a1 - a2
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff
    
    def filter(self, angle: float) -> float:
        """
        对角度进行中位数滤波（处理环绕问题）
        
        Args:
            angle: 新的角度测量值（度）
            
        Returns:
            中位数滤波后的角度
        """
        normalized_angle = self.normalize_angle(angle)
        self.buffer.append(normalized_angle)
        
        if len(self.buffer) < self.window_size:
            return normalized_angle
        
        # 使用参考角度方法处理环绕问题
        # 以第一个值为参考，计算其他值相对于它的差值
        ref_angle = self.buffer[0]
        differences = []
        
        for a in self.buffer:
            diff = self.angle_difference(a, ref_angle)
            differences.append(diff)
        
        # 对差值取中位数，然后加回参考角度
        median_diff = statistics.median(differences)
        result = self.normalize_angle(ref_angle + median_diff)
        
        return result
    
    def reset(self):
        """重置滤波器状态"""
        self.buffer.clear()


# ============================================================================
# 物理约束滤波器（Range Gating）
# ============================================================================

class PhysicalConstraintFilter:
    """
    物理约束滤波器 - 基于物理规则拒绝不可能的数据
    
    由于机器狗是物理机器人，关节和传感器有物理极限。
    可以硬编码这些物理规则来拒绝不可能的数据。
    
    检查项目：
    1. 绝对限制：如果数据超出物理可能的范围，则为无效
    2. 速度限制：如果位置变化暗示的速度超过物理可能，则为无效
    """
    
    def __init__(self,
                 min_distance: float = MIN_DISTANCE,
                 max_distance: float = MAX_DISTANCE,
                 min_azimuth: float = MIN_AZIMUTH,
                 max_azimuth: float = MAX_AZIMUTH,
                 min_elevation: float = MIN_ELEVATION,
                 max_elevation: float = MAX_ELEVATION,
                 max_velocity: float = MAX_VELOCITY):
        """
        初始化物理约束滤波器
        
        Args:
            min_distance: 最小有效距离（厘米）
            max_distance: 最大有效距离（厘米）
            min_azimuth: 最小方位角（度）
            max_azimuth: 最大方位角（度）
            min_elevation: 最小仰角（度）
            max_elevation: 最大仰角（度）
            max_velocity: 最大允许速度（厘米/秒）
        """
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.min_azimuth = min_azimuth
        self.max_azimuth = max_azimuth
        self.min_elevation = min_elevation
        self.max_elevation = max_elevation
        self.max_velocity = max_velocity
        
        # 上一次有效数据
        self.last_valid_position = None
        self.last_valid_time = None
        self.last_valid_data = None
        
        # 统计
        self.reject_count_range = 0      # 超出范围拒绝
        self.reject_count_velocity = 0   # 超出速度拒绝
    
    def check_range_constraints(self, distance: float, azimuth: float, elevation: float) -> bool:
        """
        检查数据是否在物理可能的范围内
        
        Args:
            distance: 距离（厘米）
            azimuth: 方位角（度）
            elevation: 仰角（度）
            
        Returns:
            True 如果数据在有效范围内，False 如果超出范围
        """
        # 检查距离范围
        if distance < self.min_distance or distance > self.max_distance:
            return False
        
        # 检查方位角范围（归一化后检查）
        normalized_azimuth = AngleMedianFilter.normalize_angle(azimuth)
        if normalized_azimuth < self.min_azimuth or normalized_azimuth > self.max_azimuth:
            return False
        
        # 检查仰角范围
        if elevation < self.min_elevation or elevation > self.max_elevation:
            return False
        
        return True
    
    def check_velocity_constraint(self, x: float, y: float, z: float, 
                                   current_time: float = None) -> bool:
        """
        检查位置变化是否符合速度约束
        
        如果 angle_t 和 angle_t-1 之间的变化暗示速度为 5000°/秒，
        那么新数据点是无效的。
        
        Args:
            x, y, z: 当前位置（厘米）
            current_time: 当前时间戳
            
        Returns:
            True 如果速度在允许范围内，False 如果超出速度限制
        """
        if current_time is None:
            current_time = time.time()
        
        # 第一个数据点总是有效的
        if self.last_valid_position is None or self.last_valid_time is None:
            return True
        
        # 计算位移
        dx = x - self.last_valid_position[0]
        dy = y - self.last_valid_position[1]
        dz = z - self.last_valid_position[2]
        displacement = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # 计算时间差
        dt = current_time - self.last_valid_time
        if dt <= 0:
            dt = 0.01  # 避免除以零
        
        # 计算速度
        velocity = displacement / dt
        
        # 检查速度是否在允许范围内
        return velocity <= self.max_velocity
    
    def filter(self, distance: float, azimuth: float, elevation: float,
               x: float, y: float, z: float,
               current_time: float = None) -> dict:
        """
        对数据进行物理约束检查
        
        Args:
            distance: 距离（厘米）
            azimuth: 方位角（度）
            elevation: 仰角（度）
            x, y, z: 笛卡尔坐标（厘米）
            current_time: 时间戳
            
        Returns:
            字典包含:
            - 'valid': 数据是否有效
            - 'reject_reason': 拒绝原因（如果无效）
            - 原始数据字段
        """
        if current_time is None:
            current_time = time.time()
        
        result = {
            'distance': distance,
            'azimuth': azimuth,
            'elevation': elevation,
            'x': x, 'y': y, 'z': z,
            'valid': True,
            'reject_reason': None
        }
        
        # Step 1: 检查范围约束
        if not self.check_range_constraints(distance, azimuth, elevation):
            result['valid'] = False
            result['reject_reason'] = 'range'
            self.reject_count_range += 1
            
            # 如果有上一次有效数据，使用它
            if self.last_valid_data:
                return {**self.last_valid_data, 'valid': False, 'reject_reason': 'range'}
            return result
        
        # Step 2: 检查速度约束
        if not self.check_velocity_constraint(x, y, z, current_time):
            result['valid'] = False
            result['reject_reason'] = 'velocity'
            self.reject_count_velocity += 1
            
            # 如果有上一次有效数据，使用它
            if self.last_valid_data:
                return {**self.last_valid_data, 'valid': False, 'reject_reason': 'velocity'}
            return result
        
        # 数据有效，更新上一次有效数据
        self.last_valid_position = (x, y, z)
        self.last_valid_time = current_time
        self.last_valid_data = result.copy()
        
        return result
    
    def get_statistics(self) -> dict:
        """获取统计信息"""
        return {
            'reject_count_range': self.reject_count_range,
            'reject_count_velocity': self.reject_count_velocity,
            'total_rejected': self.reject_count_range + self.reject_count_velocity
        }
    
    def reset(self):
        """重置滤波器状态"""
        self.last_valid_position = None
        self.last_valid_time = None
        self.last_valid_data = None
        self.reject_count_range = 0
        self.reject_count_velocity = 0


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
        
        Args:
            process_noise_pos: 位置过程噪声（Q矩阵对角元素）
                - 越大：更相信测量，响应更快
                - 越小：更相信预测，更平滑
            process_noise_vel: 速度过程噪声
            measurement_noise: 测量噪声（R矩阵对角元素）
        """
        # 状态维度
        self.n_states = 6      # [x, y, z, vx, vy, vz]
        self.n_measurements = 3  # [x, y, z]
        
        # 状态向量 x = [x, y, z, vx, vy, vz]^T
        self.x = None  # 延迟初始化
        
        # 状态协方差矩阵 P (6x6)
        self.P = None
        
        # 过程噪声协方差 Q (6x6)
        self.Q = self._create_diagonal_matrix(
            [process_noise_pos, process_noise_pos, process_noise_pos,
             process_noise_vel, process_noise_vel, process_noise_vel]
        )
        
        # 测量噪声协方差 R (3x3)
        self.R = self._create_diagonal_matrix(
            [measurement_noise, measurement_noise, measurement_noise]
        )
        
        # 观测矩阵 H (3x6) - 我们只能观测位置，不能直接观测速度
        # z = H * x = [x, y, z]
        self.H = [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ]
        
        # 上一次更新时间
        self.last_time = None
        self.initialized = False
    
    def _create_diagonal_matrix(self, diagonal_values):
        """创建对角矩阵"""
        n = len(diagonal_values)
        matrix = [[0.0] * n for _ in range(n)]
        for i in range(n):
            matrix[i][i] = diagonal_values[i]
        return matrix
    
    def _matrix_multiply(self, A, B):
        """矩阵乘法 A * B"""
        rows_A, cols_A = len(A), len(A[0])
        rows_B, cols_B = len(B), len(B[0])
        
        if cols_A != rows_B:
            raise ValueError("矩阵维度不匹配")
        
        result = [[0.0] * cols_B for _ in range(rows_A)]
        for i in range(rows_A):
            for j in range(cols_B):
                for k in range(cols_A):
                    result[i][j] += A[i][k] * B[k][j]
        return result
    
    def _matrix_add(self, A, B):
        """矩阵加法 A + B"""
        return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    
    def _matrix_subtract(self, A, B):
        """矩阵减法 A - B"""
        return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]
    
    def _matrix_transpose(self, A):
        """矩阵转置"""
        return [[A[j][i] for j in range(len(A))] for i in range(len(A[0]))]
    
    def _matrix_inverse_3x3(self, A):
        """3x3矩阵求逆（用于卡尔曼增益计算）"""
        # 计算行列式
        det = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
               A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
               A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))
        
        if abs(det) < 1e-10:
            # 如果矩阵接近奇异，添加正则化
            for i in range(3):
                A[i][i] += 1e-6
            det = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                   A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                   A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]))
        
        inv_det = 1.0 / det
        
        # 计算伴随矩阵并除以行列式
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
        """
        创建状态转移矩阵 F（恒速模型）
        
        F = | I   dt*I |
            | 0    I   |
        
        其中 I 是 3x3 单位矩阵
        """
        F = [[0.0] * 6 for _ in range(6)]
        
        # 对角线为1
        for i in range(6):
            F[i][i] = 1.0
        
        # 位置受速度影响：x_new = x_old + vx * dt
        F[0][3] = dt  # x 受 vx 影响
        F[1][4] = dt  # y 受 vy 影响
        F[2][5] = dt  # z 受 vz 影响
        
        return F
    
    def predict(self, dt: float):
        """
        预测步骤：基于运动模型预测下一状态
        
        Args:
            dt: 时间间隔（秒）
        """
        if not self.initialized:
            return
        
        # 状态转移矩阵
        F = self._create_state_transition_matrix(dt)
        
        # 预测状态: x_pred = F * x
        x_pred = [[0.0] for _ in range(6)]
        for i in range(6):
            for j in range(6):
                x_pred[i][0] += F[i][j] * self.x[j][0]
        self.x = x_pred
        
        # 预测协方差: P_pred = F * P * F^T + Q
        FP = self._matrix_multiply(F, self.P)
        F_T = self._matrix_transpose(F)
        FP_FT = self._matrix_multiply(FP, F_T)
        self.P = self._matrix_add(FP_FT, self.Q)
    
    def update(self, z_x: float, z_y: float, z_z: float, current_time: float = None):
        """
        更新步骤：用测量值修正预测
        
        Args:
            z_x, z_y, z_z: 测量的位置坐标
            current_time: 当前时间戳
            
        Returns:
            dict: 滤波后的位置和速度
        """
        if current_time is None:
            current_time = time.time()
        
        # 第一次测量，初始化状态
        if not self.initialized:
            self.x = [[z_x], [z_y], [z_z], [0.0], [0.0], [0.0]]  # 初始速度为0
            self.P = self._create_diagonal_matrix([10, 10, 10, 100, 100, 100])  # 初始不确定性
            self.last_time = current_time
            self.initialized = True
            return {
                'x': z_x, 'y': z_y, 'z': z_z,
                'vx': 0.0, 'vy': 0.0, 'vz': 0.0
            }
        
        # 计算时间间隔
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.01
        self.last_time = current_time
        
        # 1. 预测步骤
        self.predict(dt)
        
        # 2. 更新步骤
        # 测量向量 z (3x1)
        z = [[z_x], [z_y], [z_z]]
        
        # 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^(-1)
        H_T = self._matrix_transpose(self.H)
        PH_T = self._matrix_multiply(self.P, H_T)
        HPH_T = self._matrix_multiply(self.H, PH_T)
        S = self._matrix_add(HPH_T, self.R)  # 创新协方差
        S_inv = self._matrix_inverse_3x3(S)
        K = self._matrix_multiply(PH_T, S_inv)  # 卡尔曼增益 (6x3)
        
        # 计算创新（测量残差）: y = z - H * x
        Hx = [[0.0] for _ in range(3)]
        for i in range(3):
            for j in range(6):
                Hx[i][0] += self.H[i][j] * self.x[j][0]
        y = self._matrix_subtract(z, Hx)
        
        # 更新状态: x = x + K * y
        Ky = [[0.0] for _ in range(6)]
        for i in range(6):
            for j in range(3):
                Ky[i][0] += K[i][j] * y[j][0]
        self.x = self._matrix_add(self.x, Ky)
        
        # 更新协方差: P = (I - K * H) * P
        KH = self._matrix_multiply(K, self.H)
        I = self._create_diagonal_matrix([1, 1, 1, 1, 1, 1])
        I_KH = self._matrix_subtract(I, KH)
        self.P = self._matrix_multiply(I_KH, self.P)
        
        return {
            'x': self.x[0][0],
            'y': self.x[1][0],
            'z': self.x[2][0],
            'vx': self.x[3][0],
            'vy': self.x[4][0],
            'vz': self.x[5][0]
        }
    
    def get_predicted_position(self, dt_ahead: float = 0.1):
        """
        获取未来位置预测（用于机器狗提前规划）
        
        Args:
            dt_ahead: 预测多少秒后的位置
            
        Returns:
            dict: 预测的位置
        """
        if not self.initialized:
            return None
        
        # 使用当前速度预测未来位置
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
# 综合数据滤波器（三步滤波流程：物理约束 + 中位数 + EKF）
# ============================================================================

class UWBDataFilter:
    """
    UWB数据综合滤波器 - 实现机器狗传感器数据滤波的完整流程
    
    滤波流程（支持两种模式）：
    
    【模式1】两步滤波（use_ekf=False，默认）：
        Step 1 (物理约束检查): 应用物理约束滤波
        Step 2 (去尖峰): 中位数滤波器（窗口大小3或5）
        注意：传感器数据已自带卡尔曼滤波，因此默认不再额外添加
    
    【模式2】三步滤波（use_ekf=True）：
        Step 1 (物理约束检查): 应用物理约束滤波
        Step 2 (去尖峰): 中位数滤波器
        Step 3 (状态估计): EKF扩展卡尔曼滤波（位置+速度状态估计）
        
        为什么传感器已有KF还能用EKF？
        - 传感器KF：测量级滤波，仅平滑单个测量值
        - EKF：状态估计滤波，结合运动模型预测位置和速度
        - EKF可以在数据丢失时预测位置，并输出速度信息用于机器狗控制
    
    参考：《Probabilistic Robotics》 (Thrun, Burgard, Fox)
    """
    
    def __init__(self,
                 # 物理约束参数
                 min_distance: float = MIN_DISTANCE,
                 max_distance: float = MAX_DISTANCE,
                 max_velocity: float = MAX_VELOCITY,
                 # 中位数滤波参数
                 median_window: int = MEDIAN_WINDOW_SIZE,
                 # EKF参数
                 use_ekf: bool = False,
                 ekf_process_noise_pos: float = 0.5,
                 ekf_process_noise_vel: float = 2.0,
                 ekf_measurement_noise: float = 5.0):
        """
        初始化综合滤波器
        
        Args:
            min_distance: 最小有效距离
            max_distance: 最大有效距离
            max_velocity: 最大允许速度
            median_window: 中位数滤波窗口大小
            use_ekf: 是否启用EKF扩展卡尔曼滤波
            ekf_process_noise_pos: EKF位置过程噪声
            ekf_process_noise_vel: EKF速度过程噪声
            ekf_measurement_noise: EKF测量噪声
        """
        # ===== Step 1: 物理约束滤波器 =====
        self.physical_filter = PhysicalConstraintFilter(
            min_distance=min_distance,
            max_distance=max_distance,
            max_velocity=max_velocity
        )
        
        # ===== Step 2: 中位数滤波器（对各个参数分别滤波） =====
        self.median_distance = MedianFilter(median_window)
        self.median_azimuth = AngleMedianFilter(median_window)  # 角度特殊处理
        self.median_elevation = MedianFilter(median_window)
        self.median_x = MedianFilter(median_window)
        self.median_y = MedianFilter(median_window)
        self.median_z = MedianFilter(median_window)
        
        # ===== Step 3: EKF扩展卡尔曼滤波器（可选） =====
        self.use_ekf = use_ekf
        if use_ekf:
            self.ekf = ExtendedKalmanFilter(
                process_noise_pos=ekf_process_noise_pos,
                process_noise_vel=ekf_process_noise_vel,
                measurement_noise=ekf_measurement_noise
            )
        else:
            self.ekf = None
        
        # 统计信息
        self.total_count = 0
        self.outlier_count = 0
        self.reject_count_range = 0
        self.reject_count_velocity = 0
        
        # 最后有效数据
        self.last_valid_data = None
        
        # 最后的速度信息（仅EKF模式可用）
        self.last_velocity = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
    
    def filter(self, distance: float, azimuth: float, elevation: float,
               x: float, y: float, z: float,
               current_time: float = None) -> dict:
        """
        执行滤波流程（两步或三步，取决于是否启用EKF）
        
        Args:
            distance: 原始距离（厘米）
            azimuth: 原始方位角（度）
            elevation: 原始仰角（度）
            x, y, z: 原始笛卡尔坐标（厘米）
            current_time: 时间戳
            
        Returns:
            滤波后的数据字典，包含:
            - 位置: x, y, z
            - 球坐标: distance, azimuth, elevation
            - 速度（仅EKF模式）: vx, vy, vz
            - 标志: is_outlier, reject_reason
        """
        if current_time is None:
            current_time = time.time()
        
        self.total_count += 1
        
        # ========== Step 1: 物理约束检查 ==========
        physical_result = self.physical_filter.filter(
            distance, azimuth, elevation, x, y, z, current_time
        )
        
        if not physical_result['valid']:
            self.outlier_count += 1
            if physical_result['reject_reason'] == 'range':
                self.reject_count_range += 1
            elif physical_result['reject_reason'] == 'velocity':
                self.reject_count_velocity += 1
            
            # 返回上一次有效数据（如果有）
            if self.last_valid_data:
                return {**self.last_valid_data, 'is_outlier': True, 
                        'reject_reason': physical_result['reject_reason']}
            return None
        
        # ========== Step 2: 中位数滤波（去尖峰） ==========
        median_distance = self.median_distance.filter(distance)
        median_azimuth = self.median_azimuth.filter(azimuth)
        median_elevation = self.median_elevation.filter(elevation)
        median_x = self.median_x.filter(x)
        median_y = self.median_y.filter(y)
        median_z = self.median_z.filter(z)
        
        # ========== Step 3: EKF扩展卡尔曼滤波（可选） ==========
        if self.use_ekf and self.ekf:
            ekf_result = self.ekf.update(median_x, median_y, median_z, current_time)
            
            # 使用EKF输出
            result = {
                'distance': round(median_distance, 2),
                'azimuth': round(median_azimuth, 2),
                'elevation': round(median_elevation, 2),
                'x': round(ekf_result['x'], 2),
                'y': round(ekf_result['y'], 2),
                'z': round(ekf_result['z'], 2),
                'vx': round(ekf_result['vx'], 2),
                'vy': round(ekf_result['vy'], 2),
                'vz': round(ekf_result['vz'], 2),
                'is_outlier': False,
                'reject_reason': None
            }
            self.last_velocity = {
                'vx': ekf_result['vx'],
                'vy': ekf_result['vy'],
                'vz': ekf_result['vz']
            }
        else:
            # 不使用EKF，直接返回中位数滤波结果
            result = {
                'distance': round(median_distance, 2),
                'azimuth': round(median_azimuth, 2),
                'elevation': round(median_elevation, 2),
                'x': round(median_x, 2),
                'y': round(median_y, 2),
                'z': round(median_z, 2),
                'vx': 0.0,
                'vy': 0.0,
                'vz': 0.0,
                'is_outlier': False,
                'reject_reason': None
            }
        
        self.last_valid_data = result.copy()
        return result
    
    def get_predicted_position(self, dt_ahead: float = 0.1):
        """
        获取未来位置预测（仅EKF模式可用）
        
        Args:
            dt_ahead: 预测多少秒后的位置
            
        Returns:
            dict: 预测的位置，如果EKF未启用则返回None
        """
        if self.use_ekf and self.ekf:
            return self.ekf.get_predicted_position(dt_ahead)
        return None
    
    def get_velocity(self):
        """
        获取当前速度估计（仅EKF模式有意义的值）
        
        Returns:
            dict: {'vx': float, 'vy': float, 'vz': float}
        """
        return self.last_velocity
    
    def get_statistics(self) -> dict:
        """获取滤波统计信息"""
        outlier_rate = self.outlier_count / self.total_count if self.total_count > 0 else 0
        stats = {
            'total_count': self.total_count,
            'outlier_count': self.outlier_count,
            'valid_count': self.total_count - self.outlier_count,
            'outlier_rate': f"{outlier_rate * 100:.1f}%",
            'reject_range': self.reject_count_range,
            'reject_velocity': self.reject_count_velocity,
            'use_ekf': self.use_ekf
        }
        return stats
    
    def reset(self):
        """重置所有滤波器状态"""
        self.physical_filter.reset()
        self.median_distance.reset()
        self.median_azimuth.reset()
        self.median_elevation.reset()
        self.median_x.reset()
        self.median_y.reset()
        self.median_z.reset()
        if self.use_ekf and self.ekf:
            self.ekf.reset()
        self.total_count = 0
        self.outlier_count = 0
        self.reject_count_range = 0
        self.reject_count_velocity = 0
        self.last_valid_data = None
        self.last_velocity = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}


# ============================================================================
# 数据解析类
# ============================================================================

class UWBDataParser:
    """UWB数据帧解析器"""
    
    def __init__(self):
        self.buffer = b''
    
    def calculate_xor(self, data):
        xor_value = 0
        for byte in data:
            xor_value ^= byte
        return xor_value
    
    def parse(self, new_data):
        self.buffer += new_data
        results = []
        
        while True:
            header_idx = self.buffer.find(FRAME_HEADER)
            
            if header_idx < 0:
                if len(self.buffer) > 3:
                    self.buffer = self.buffer[-3:]
                break
            
            if header_idx > 0:
                self.buffer = self.buffer[header_idx:]
            
            if len(self.buffer) < FRAME_LENGTH:
                break
            
            frame = self.buffer[:FRAME_LENGTH]
            
            calculated_xor = self.calculate_xor(frame[:-1])
            received_xor = frame[-1]
            
            if calculated_xor != received_xor:
                self.buffer = self.buffer[4:]
                continue
            
            try:
                command = struct.unpack('>H', frame[8:10])[0]
                
                if command == CMD_POSITION:
                    anchor_id = struct.unpack('>I', frame[12:16])[0]
                    tag_id = struct.unpack('>I', frame[16:20])[0]
                    distance_cm = struct.unpack('>I', frame[20:24])[0]
                    azimuth_deg = struct.unpack('>h', frame[24:26])[0]
                    elevation_deg = struct.unpack('>h', frame[26:28])[0]
                    
                    x, y, z = self.spherical_to_cartesian(
                        distance_cm, azimuth_deg, elevation_deg
                    )
                    
                    result = {
                        'anchor_id': anchor_id,
                        'tag_id': tag_id,
                        'distance_cm': distance_cm,
                        'azimuth_deg': azimuth_deg,
                        'elevation_deg': elevation_deg,
                        'x': x,
                        'y': y,
                        'z': z,
                        'timestamp': time.time()
                    }
                    results.append(result)
            except Exception as e:
                print(f"解析错误: {e}")
            
            self.buffer = self.buffer[FRAME_LENGTH:]
        
        return results if results else None
    
    def spherical_to_cartesian(self, distance_cm, azimuth_deg, elevation_deg):
        azimuth_rad = math.radians(azimuth_deg)
        elevation_rad = math.radians(elevation_deg)
        
        horizontal_distance = distance_cm * math.cos(elevation_rad)
        
        x = horizontal_distance * math.sin(azimuth_rad)
        y = horizontal_distance * math.cos(azimuth_rad)
        z = distance_cm * math.sin(elevation_rad)
        
        return x, y, z


# ============================================================================
# 串口通信类
# ============================================================================

class SerialReceiver:
    """串口数据接收器"""
    
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.parser = UWBDataParser()
        self.data_queue = deque(maxlen=100)
        self.thread = None
    
    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                timeout=0.1
            )
            print(f"✓ 串口 {self.port} 已连接")
            print(f"  波特率: {self.baudrate}")
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("串口已断开")
    
    def start_receiving(self):
        if not self.serial or not self.serial.is_open:
            print("错误：串口未连接")
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        print("✓ 开始接收数据...")
        return True
    
    def _receive_loop(self):
        while self.running:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    results = self.parser.parse(data)
                    if results:
                        for result in results:
                            self.data_queue.append(result)
            except Exception as e:
                print(f"接收错误: {e}")
                time.sleep(0.1)
    
    def get_all_data(self):
        data_list = list(self.data_queue)
        self.data_queue.clear()
        return data_list


# ============================================================================
# 模拟数据生成器（含异常值）
# ============================================================================

class SimulatedReceiver:
    """模拟数据接收器（含异常值，用于测试滤波效果）"""
    
    def __init__(self):
        self.data_queue = deque(maxlen=100)
        self.running = False
        self.thread = None
        self.angle = 0
        self.radius = 50
        self.z_angle = 0
        self.data_count = 0
    
    def connect(self):
        print("✓ 模拟模式已启动（含异常值，用于测试滤波效果）")
        return True
    
    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
    
    def start_receiving(self):
        self.running = True
        self.thread = threading.Thread(target=self._generate_loop, daemon=True)
        self.thread.start()
        print("✓ 开始生成模拟数据（含异常值测试）...")
        return True
    
    def _generate_loop(self):
        import random
        
        while self.running:
            self.data_count += 1
            self.angle += 6
            self.radius += 0.15
            self.z_angle += 4
            
            # 基础轨迹
            noise_x = random.gauss(0, 2)
            noise_y = random.gauss(0, 2)
            noise_z = random.gauss(0, 1)
            
            x = self.radius * math.sin(math.radians(self.angle)) + noise_x
            y = self.radius * math.cos(math.radians(self.angle)) + noise_y
            z = 10 * math.sin(math.radians(self.z_angle)) + noise_z
            
            # 每隔一定间隔添加异常值（模拟人员遮挡）
            if self.data_count % 30 == 0:  # 每30个数据点添加一个异常
                x += random.choice([-1, 1]) * random.uniform(80, 150)
                y += random.choice([-1, 1]) * random.uniform(80, 150)
            
            distance = math.sqrt(x*x + y*y + z*z)
            azimuth = math.degrees(math.atan2(x, y))
            horizontal_dist = math.sqrt(x*x + y*y)
            elevation = math.degrees(math.atan2(z, horizontal_dist)) if horizontal_dist > 0 else 0
            
            data = {
                'anchor_id': 0xAAA2,
                'tag_id': 0xAAA1,
                'distance_cm': int(distance),
                'azimuth_deg': int(azimuth),
                'elevation_deg': int(elevation),
                'x': x,
                'y': y,
                'z': z,
                'timestamp': time.time()
            }
            self.data_queue.append(data)
            
            time.sleep(0.1)
    
    def get_all_data(self):
        data_list = list(self.data_queue)
        self.data_queue.clear()
        return data_list


# ============================================================================
# 对比可视化类（原始数据 vs 滤波数据）
# ============================================================================

class FilteredPathVisualizer:
    """原始数据与滤波数据对比可视化器（支持EKF扩展卡尔曼滤波）"""
    
    def __init__(self, receiver, use_ekf=False):
        """
        初始化可视化器
        
        Args:
            receiver: 数据接收器（串口或模拟）
            use_ekf: 是否启用EKF扩展卡尔曼滤波
        """
        self.receiver = receiver
        self.use_ekf = use_ekf
        
        # 滤波器（支持两步或三步滤波流程）
        self.data_filter = UWBDataFilter(
            # Step 1: 物理约束参数
            min_distance=MIN_DISTANCE,
            max_distance=MAX_DISTANCE,
            max_velocity=MAX_VELOCITY,
            # Step 2: 中位数滤波参数
            median_window=MEDIAN_WINDOW_SIZE,
            # Step 3: EKF参数（可选）
            use_ekf=use_ekf,
            ekf_process_noise_pos=0.5,
            ekf_process_noise_vel=2.0,
            ekf_measurement_noise=5.0
        )
        
        # 原始数据路径
        self.raw_path = deque(maxlen=MAX_PATH_POINTS)
        self.raw_current_pos = None
        
        # 滤波后数据路径
        self.filtered_path = deque(maxlen=MAX_PATH_POINTS)
        self.filtered_current_pos = None
        
        # 统计信息
        self.data_count = 0
        self.outlier_count = 0
        self.last_raw_data = {'distance': 0, 'azimuth': 0, 'elevation': 0, 'x': 0, 'y': 0, 'z': 0}
        self.last_filtered_data = {'distance': 0, 'x': 0, 'y': 0, 'z': 0, 'vx': 0, 'vy': 0, 'vz': 0}
        
        self.setup_plot()
    
    def setup_plot(self):
        """设置六个二维平面图形界面"""
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False
        
        # 创建2行4列布局（3个原始图 + 3个滤波图 + 信息面板）
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.canvas.manager.set_window_title('UWB 原始数据 vs 滤波数据 对比可视化')
        
        # ==================== 上排：原始数据 ====================
        # 俯视图 (X-Y) - 原始
        self.ax_raw_xy = self.fig.add_subplot(241)
        self.ax_raw_xy.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_raw_xy.set_ylim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_raw_xy.set_aspect('equal')
        self.ax_raw_xy.grid(True, linestyle='--', alpha=0.5)
        self.ax_raw_xy.set_xlabel('X (cm)', fontsize=9)
        self.ax_raw_xy.set_ylabel('Y (cm)', fontsize=9)
        self.ax_raw_xy.set_title('原始数据 - 俯视图(X-Y)', fontsize=10, fontweight='bold', color='red')
        self.ax_raw_xy.plot(0, 0, 'rs', markersize=10)
        self.raw_path_xy, = self.ax_raw_xy.plot([], [], 'r-', linewidth=1.5, alpha=0.7, label='原始轨迹')
        self.raw_point_xy, = self.ax_raw_xy.plot([], [], 'ro', markersize=8)
        
        # 前视图 (X-Z) - 原始
        self.ax_raw_xz = self.fig.add_subplot(242)
        self.ax_raw_xz.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_raw_xz.set_ylim(-DISPLAY_RANGE/2, DISPLAY_RANGE/2)
        self.ax_raw_xz.grid(True, linestyle='--', alpha=0.5)
        self.ax_raw_xz.set_xlabel('X (cm)', fontsize=9)
        self.ax_raw_xz.set_ylabel('Z (cm)', fontsize=9)
        self.ax_raw_xz.set_title('原始数据 - 前视图(X-Z)', fontsize=10, fontweight='bold', color='red')
        self.ax_raw_xz.plot(0, 0, 'rs', markersize=10)
        self.raw_path_xz, = self.ax_raw_xz.plot([], [], 'r-', linewidth=1.5, alpha=0.7)
        self.raw_point_xz, = self.ax_raw_xz.plot([], [], 'ro', markersize=8)
        
        # 侧视图 (Y-Z) - 原始
        self.ax_raw_yz = self.fig.add_subplot(243)
        self.ax_raw_yz.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_raw_yz.set_ylim(-DISPLAY_RANGE/2, DISPLAY_RANGE/2)
        self.ax_raw_yz.grid(True, linestyle='--', alpha=0.5)
        self.ax_raw_yz.set_xlabel('Y (cm)', fontsize=9)
        self.ax_raw_yz.set_ylabel('Z (cm)', fontsize=9)
        self.ax_raw_yz.set_title('原始数据 - 侧视图(Y-Z)', fontsize=10, fontweight='bold', color='red')
        self.ax_raw_yz.plot(0, 0, 'rs', markersize=10)
        self.raw_path_yz, = self.ax_raw_yz.plot([], [], 'r-', linewidth=1.5, alpha=0.7)
        self.raw_point_yz, = self.ax_raw_yz.plot([], [], 'ro', markersize=8)
        
        # ==================== 下排：滤波后数据 ====================
        # 俯视图 (X-Y) - 滤波
        self.ax_filt_xy = self.fig.add_subplot(245)
        self.ax_filt_xy.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_filt_xy.set_ylim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_filt_xy.set_aspect('equal')
        self.ax_filt_xy.grid(True, linestyle='--', alpha=0.5)
        self.ax_filt_xy.set_xlabel('X (cm)', fontsize=9)
        self.ax_filt_xy.set_ylabel('Y (cm)', fontsize=9)
        self.ax_filt_xy.set_title('滤波数据 - 俯视图(X-Y)', fontsize=10, fontweight='bold', color='blue')
        self.ax_filt_xy.plot(0, 0, 'bs', markersize=10)
        self.filt_path_xy, = self.ax_filt_xy.plot([], [], 'b-', linewidth=2, alpha=0.8, label='滤波轨迹')
        self.filt_point_xy, = self.ax_filt_xy.plot([], [], 'go', markersize=10)
        
        # 前视图 (X-Z) - 滤波
        self.ax_filt_xz = self.fig.add_subplot(246)
        self.ax_filt_xz.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_filt_xz.set_ylim(-DISPLAY_RANGE/2, DISPLAY_RANGE/2)
        self.ax_filt_xz.grid(True, linestyle='--', alpha=0.5)
        self.ax_filt_xz.set_xlabel('X (cm)', fontsize=9)
        self.ax_filt_xz.set_ylabel('Z (cm)', fontsize=9)
        self.ax_filt_xz.set_title('滤波数据 - 前视图(X-Z)', fontsize=10, fontweight='bold', color='blue')
        self.ax_filt_xz.plot(0, 0, 'bs', markersize=10)
        self.filt_path_xz, = self.ax_filt_xz.plot([], [], 'b-', linewidth=2, alpha=0.8)
        self.filt_point_xz, = self.ax_filt_xz.plot([], [], 'go', markersize=10)
        
        # 侧视图 (Y-Z) - 滤波
        self.ax_filt_yz = self.fig.add_subplot(247)
        self.ax_filt_yz.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_filt_yz.set_ylim(-DISPLAY_RANGE/2, DISPLAY_RANGE/2)
        self.ax_filt_yz.grid(True, linestyle='--', alpha=0.5)
        self.ax_filt_yz.set_xlabel('Y (cm)', fontsize=9)
        self.ax_filt_yz.set_ylabel('Z (cm)', fontsize=9)
        self.ax_filt_yz.set_title('滤波数据 - 侧视图(Y-Z)', fontsize=10, fontweight='bold', color='blue')
        self.ax_filt_yz.plot(0, 0, 'bs', markersize=10)
        self.filt_path_yz, = self.ax_filt_yz.plot([], [], 'b-', linewidth=2, alpha=0.8)
        self.filt_point_yz, = self.ax_filt_yz.plot([], [], 'go', markersize=10)
        
        # ==================== 信息面板 ====================
        self.ax_info = self.fig.add_subplot(244)
        self.ax_info.axis('off')
        self.info_box = self.ax_info.text(
            0.05, 0.95, "等待数据...",
            fontsize=9,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9),
            family='monospace',
            transform=self.ax_info.transAxes
        )
        
        # 统计信息面板
        self.ax_stats = self.fig.add_subplot(248)
        self.ax_stats.axis('off')
        self.stats_box = self.ax_stats.text(
            0.05, 0.95, "滤波统计...",
            fontsize=9,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightcyan', alpha=0.9),
            family='monospace',
            transform=self.ax_stats.transAxes
        )
        
        plt.tight_layout()
    
    def update(self, frame):
        """更新图形"""
        current_time = time.time()
        
        data_list = self.receiver.get_all_data()
        
        for data in data_list:
            self.data_count += 1
            
            # 原始数据
            raw_x, raw_y, raw_z = data['x'], data['y'], data['z']
            self.raw_path.append((raw_x, raw_y, raw_z, current_time))
            self.raw_current_pos = (raw_x, raw_y, raw_z)
            
            self.last_raw_data = {
                'distance': data['distance_cm'],
                'azimuth': data['azimuth_deg'],
                'elevation': data['elevation_deg'],
                'x': raw_x,
                'y': raw_y,
                'z': raw_z
            }
            
            # 滤波处理（支持两步或三步滤波流程）
            filtered = self.data_filter.filter(
                data['distance_cm'], 
                data['azimuth_deg'],
                data['elevation_deg'],
                raw_x, raw_y, raw_z, 
                current_time
            )
            
            if filtered:
                if filtered['is_outlier']:
                    self.outlier_count += 1
                
                filt_x, filt_y, filt_z = filtered['x'], filtered['y'], filtered['z']
                self.filtered_path.append((filt_x, filt_y, filt_z, current_time))
                self.filtered_current_pos = (filt_x, filt_y, filt_z)
                
                self.last_filtered_data = {
                    'distance': filtered['distance'],
                    'x': filt_x,
                    'y': filt_y,
                    'z': filt_z,
                    'vx': filtered.get('vx', 0),
                    'vy': filtered.get('vy', 0),
                    'vz': filtered.get('vz', 0)
                }
        
        # 移除过期路径点
        while self.raw_path and (current_time - self.raw_path[0][3]) > PATH_FADE_TIME:
            self.raw_path.popleft()
        while self.filtered_path and (current_time - self.filtered_path[0][3]) > PATH_FADE_TIME:
            self.filtered_path.popleft()
        
        # 更新原始数据图
        if len(self.raw_path) > 0:
            raw_x_list = [p[0] for p in self.raw_path]
            raw_y_list = [p[1] for p in self.raw_path]
            raw_z_list = [p[2] for p in self.raw_path]
            
            self.raw_path_xy.set_data(raw_x_list, raw_y_list)
            self.raw_path_xz.set_data(raw_x_list, raw_z_list)
            self.raw_path_yz.set_data(raw_y_list, raw_z_list)
        
        if self.raw_current_pos:
            rx, ry, rz = self.raw_current_pos
            self.raw_point_xy.set_data([rx], [ry])
            self.raw_point_xz.set_data([rx], [rz])
            self.raw_point_yz.set_data([ry], [rz])
        
        # 更新滤波后数据图
        if len(self.filtered_path) > 0:
            filt_x_list = [p[0] for p in self.filtered_path]
            filt_y_list = [p[1] for p in self.filtered_path]
            filt_z_list = [p[2] for p in self.filtered_path]
            
            self.filt_path_xy.set_data(filt_x_list, filt_y_list)
            self.filt_path_xz.set_data(filt_x_list, filt_z_list)
            self.filt_path_yz.set_data(filt_y_list, filt_z_list)
        
        if self.filtered_current_pos:
            fx, fy, fz = self.filtered_current_pos
            self.filt_point_xy.set_data([fx], [fy])
            self.filt_point_xz.set_data([fx], [fz])
            self.filt_point_yz.set_data([fy], [fz])
        
        # 更新信息文本
        if self.raw_current_pos:
            info_text = (
                f"━━━ 原始数据 ━━━\n\n"
                f"距离:    {self.last_raw_data['distance']:>6d} cm\n"
                f"方位角:  {self.last_raw_data['azimuth']:>6d}°\n"
                f"仰角:    {self.last_raw_data['elevation']:>6d}°\n\n"
                f"X坐标:   {self.last_raw_data['x']:>8.1f} cm\n"
                f"Y坐标:   {self.last_raw_data['y']:>8.1f} cm\n"
                f"Z坐标:   {self.last_raw_data['z']:>8.1f} cm\n\n"
                f"━━━ 滤波数据 ━━━\n\n"
                f"距离:    {self.last_filtered_data['distance']:>8.1f} cm\n"
                f"X坐标:   {self.last_filtered_data['x']:>8.1f} cm\n"
                f"Y坐标:   {self.last_filtered_data['y']:>8.1f} cm\n"
                f"Z坐标:   {self.last_filtered_data['z']:>8.1f} cm"
            )
            # 如果启用了EKF，显示速度信息
            if self.use_ekf:
                speed = math.sqrt(
                    self.last_filtered_data['vx']**2 + 
                    self.last_filtered_data['vy']**2 + 
                    self.last_filtered_data['vz']**2
                )
                info_text += (
                    f"\n\n━━━ EKF速度估计 ━━━\n\n"
                    f"Vx:      {self.last_filtered_data['vx']:>8.1f} cm/s\n"
                    f"Vy:      {self.last_filtered_data['vy']:>8.1f} cm/s\n"
                    f"Vz:      {self.last_filtered_data['vz']:>8.1f} cm/s\n"
                    f"速度:    {speed:>8.1f} cm/s"
                )
        else:
            info_text = "等待数据...\n\n请确保：\n1. 串口已连接\n2. 基站已上电\n3. 信标在范围内"
        
        self.info_box.set_text(info_text)
        
        # 更新统计信息
        stats = self.data_filter.get_statistics()
        
        # 根据是否启用EKF显示不同的滤波流程
        if self.use_ekf:
            filter_steps = (
                f"━━━ 滤波流程（EKF模式）━━━\n\n"
                f"Step1: 物理约束检查\n"
                f"  ├ 距离: {MIN_DISTANCE}-{MAX_DISTANCE}cm\n"
                f"  └ 速度: <{MAX_VELOCITY}cm/s\n\n"
                f"Step2: 中位数滤波\n"
                f"  └ 窗口: {MEDIAN_WINDOW_SIZE}\n\n"
                f"Step3: EKF状态估计\n"
                f"  ├ 位置噪声: 0.5\n"
                f"  ├ 速度噪声: 2.0\n"
                f"  └ 测量噪声: 5.0\n\n"
                f"🎯 EKF功能:\n"
                f"  ├ 预测目标位置\n"
                f"  ├ 估计移动速度\n"
                f"  └ 数据丢失时预测"
            )
        else:
            filter_steps = (
                f"━━━ 滤波流程（标准模式）━━━\n\n"
                f"Step1: 物理约束检查\n"
                f"  ├ 距离: {MIN_DISTANCE}-{MAX_DISTANCE}cm\n"
                f"  └ 速度: <{MAX_VELOCITY}cm/s\n\n"
                f"Step2: 中位数滤波\n"
                f"  └ 窗口: {MEDIAN_WINDOW_SIZE}\n\n"
                f"💡 提示:\n"
                f"  使用 --ekf 参数\n"
                f"  启用EKF扩展卡尔曼\n"
                f"  滤波获取速度估计"
            )
        
        stats_text = (
            f"━━━ 滤波统计 ━━━\n\n"
            f"总数据点:  {stats['total_count']:>6d}\n"
            f"异常值:    {stats['outlier_count']:>6d}\n"
            f"├ 范围异常: {stats['reject_range']:>5d}\n"
            f"├ 速度异常: {stats['reject_velocity']:>5d}\n"
            f"有效数据:  {stats['valid_count']:>6d}\n"
            f"异常率:    {stats['outlier_rate']:>6s}\n"
            f"EKF状态:   {'✓ 启用' if self.use_ekf else '✗ 禁用'}\n\n"
            f"{filter_steps}\n\n"
            f"━━━━━━━━━━━━━━━━\n\n"
            f"🔴 红色：原始数据\n"
            f"🔵 蓝色：滤波数据\n"
            f"🟢 绿点：当前位置"
        )
        self.stats_box.set_text(stats_text)
        
        # 自动调整显示范围
        all_points = list(self.raw_path) + list(self.filtered_path)
        if len(all_points) > 0:
            all_x = [p[0] for p in all_points]
            all_y = [p[1] for p in all_points]
            all_z = [p[2] for p in all_points]
            
            x_range = max(abs(min(all_x)), abs(max(all_x)), 50) * 1.3
            y_range = max(abs(min(all_y)), abs(max(all_y)), 50) * 1.3
            z_range = max(abs(min(all_z)), abs(max(all_z)), 20) * 1.5
            
            max_xy_range = max(x_range, y_range, DISPLAY_RANGE)
            max_z_range = max(z_range, DISPLAY_RANGE/4)
            
            for ax in [self.ax_raw_xy, self.ax_filt_xy]:
                ax.set_xlim(-max_xy_range, max_xy_range)
                ax.set_ylim(-max_xy_range, max_xy_range)
            
            for ax in [self.ax_raw_xz, self.ax_filt_xz]:
                ax.set_xlim(-max_xy_range, max_xy_range)
                ax.set_ylim(-max_z_range, max_z_range)
            
            for ax in [self.ax_raw_yz, self.ax_filt_yz]:
                ax.set_xlim(-max_xy_range, max_xy_range)
                ax.set_ylim(-max_z_range, max_z_range)
        
        return (self.raw_path_xy, self.filt_path_xy)
    
    def run(self):
        """运行可视化"""
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update,
            interval=UPDATE_INTERVAL,
            blit=False,
            cache_frame_data=False
        )
        
        print("\n✓ 对比可视化窗口已打开")
        print("  上排（红色）：原始数据的三个平面视图")
        print("  下排（蓝色）：滤波后数据的三个平面视图")
        print("  绿色圆点：当前位置")
        print(f"  路径残留时间：{PATH_FADE_TIME}秒")
        print("\n关闭窗口或按 Ctrl+C 退出程序")
        
        plt.show()


# ============================================================================
# 辅助函数
# ============================================================================

def list_serial_ports():
    """列出所有可用的串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未检测到可用串口")
        return []
    
    print("\n可用串口列表：")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")
    
    return [port.device for port in ports]


def print_usage():
    """打印使用说明"""
    print("""
╔════════════════════════════════════════════════════════════════════╗
║     UWB单基站跟随套件 - 原始数据与滤波数据对比可视化程序           ║
╠════════════════════════════════════════════════════════════════════╣
║                                                                    ║
║  用法：python uwb_filtered_visualizer.py [选项] [串口号]           ║
║                                                                    ║
║  选项：                                                            ║
║    <串口号>      指定串口，如 COM3 或 /dev/ttyUSB0                 ║
║    --list        列出所有可用串口                                  ║
║    --simulate    使用模拟数据（含异常值，测试滤波效果）            ║
║    --ekf         启用EKF扩展卡尔曼滤波（位置+速度状态估计）        ║
║    --help        显示此帮助信息                                    ║
║                                                                    ║
║  示例：                                                            ║
║    python uwb_filtered_visualizer.py COM3                          ║
║    python uwb_filtered_visualizer.py --simulate                    ║
║    python uwb_filtered_visualizer.py --simulate --ekf              ║
║    python uwb_filtered_visualizer.py COM3 --ekf                    ║
║                                                                    ║
║  功能：                                                            ║
║    同时显示原始数据和滤波后数据的三个二维平面视图                  ║
║    - 上排（红色）：原始数据的俯视图、前视图、侧视图                ║
║    - 下排（蓝色）：滤波后数据的俯视图、前视图、侧视图              ║
║                                                                    ║
║  EKF扩展卡尔曼滤波（--ekf 参数）：                                 ║
║    - 传感器数据已自带KF，但EKF可以额外提供：                       ║
║    - 状态估计：结合运动模型预测目标位置                            ║
║    - 速度估计：输出目标移动速度（用于机器狗控制）                  ║
║    - 预测功能：数据丢失时可预测目标位置                            ║
║                                                                    ║
║  依赖安装：                                                        ║
║    pip install pyserial matplotlib numpy                           ║
║                                                                    ║
╚════════════════════════════════════════════════════════════════════╝
""")


# ============================================================================
# 主函数
# ============================================================================

def main():
    """主函数"""
    if len(sys.argv) < 2:
        print_usage()
        ports = list_serial_ports()
        if ports:
            print(f"\n提示：请指定串口号或使用模拟模式：")
            print(f"  python {sys.argv[0]} {ports[0]}")
            print(f"  python {sys.argv[0]} --simulate")
            print(f"  python {sys.argv[0]} --simulate --ekf  # 启用EKF")
        return
    
    # 解析命令行参数
    use_ekf = '--ekf' in sys.argv
    use_simulate = '--simulate' in sys.argv
    
    # 过滤掉选项，获取串口号
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    
    if '--help' in sys.argv or '-h' in sys.argv:
        print_usage()
        return
    
    if '--list' in sys.argv:
        list_serial_ports()
        return
    
    if use_simulate:
        print("\n" + "="*60)
        print("    模拟模式 - 含异常值，测试滤波效果")
        if use_ekf:
            print("    ✓ EKF扩展卡尔曼滤波已启用（位置+速度状态估计）")
        print("="*60)
        receiver = SimulatedReceiver()
    elif args:
        port = args[0]
        print("\n" + "="*60)
        print(f"    UWB 原始数据 vs 滤波数据 对比 - 串口 {port}")
        if use_ekf:
            print("    ✓ EKF扩展卡尔曼滤波已启用（位置+速度状态估计）")
        print("="*60)
        receiver = SerialReceiver(port)
    else:
        print_usage()
        return
    
    if not receiver.connect():
        return
    
    if not receiver.start_receiving():
        receiver.disconnect()
        return
    
    visualizer = FilteredPathVisualizer(receiver, use_ekf=use_ekf)
    
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        receiver.disconnect()
        print("程序已退出")


if __name__ == '__main__':
    main()
