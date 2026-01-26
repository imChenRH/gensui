#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
UWB单基站跟随套件 - 数据滤波对比可视化程序（低延迟优化版）
============================================================================

功能说明：
    同时显示原始数据和滤波后数据的三个二维平面视图，用于对比滤波效果。
    整合多种滤波技术，用于目标跟踪和速度估计。

优化特性（参考UWB code文件夹）：
    • 非阻塞串口读取 (timeout=0)
    • 批量数据处理
    • 角度突变抑制器 - 抑制人员遮挡导致的角度突变
    • 心跳包支持 - 检测信标离线状态

滤波流程：
    Step 1: 物理约束检查 - 范围限制 + 速度限制
    Step 2: 角度突变抑制 - 抑制角度突变（阈值10°）
    Step 3: 中位数滤波 - 消除尖峰异常值
    Step 4: EKF状态估计 - 位置平滑 + 速度估计

使用方法：
    python uwb_filtered_visualizer.py <串口号>
    python uwb_filtered_visualizer.py COM3
    python uwb_filtered_visualizer.py /dev/ttyUSB0

依赖安装：
    pip install pyserial matplotlib numpy

作者：Copilot
日期：2026-01-26
============================================================================
"""

import sys
import math
import time
import threading
import struct
from collections import deque
from typing import Optional, Dict, Tuple

# 第三方库
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误：请安装 pyserial")
    print("  pip install pyserial")
    sys.exit(1)

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
except ImportError:
    print("错误：请安装 matplotlib")
    print("  pip install matplotlib")
    sys.exit(1)

try:
    import numpy as np
except ImportError:
    print("错误：请安装 numpy")
    print("  pip install numpy")
    sys.exit(1)


# ============================================================================
# 配置参数
# ============================================================================

# 显示配置
DISPLAY_RANGE = 500         # 显示范围 (cm)
MAX_PATH_POINTS = 100       # 最大路径点数
PATH_FADE_TIME = 5.0        # 路径残留时间 (秒)
UPDATE_INTERVAL = 50        # 更新间隔 (ms)

# 滤波参数 - 物理约束
MIN_DISTANCE = 5.0          # 最小距离 (cm)
MAX_DISTANCE = 5000.0       # 最大距离 (cm)
MAX_VELOCITY = 300.0        # 最大速度 (cm/s)，人快走约150cm/s
MIN_AZIMUTH = -180.0        # 最小方位角 (度)
MAX_AZIMUTH = 180.0         # 最大方位角 (度)
MIN_ELEVATION = -90.0       # 最小仰角 (度)
MAX_ELEVATION = 90.0        # 最大仰角 (度)

# 滤波参数 - 中位数滤波
MEDIAN_WINDOW_SIZE = 5      # 中位数滤波窗口大小

# 滤波参数 - 角度突变抑制
ANGLE_THRESHOLD = 10.0      # 角度突变阈值（度），超过此阈值视为异常

# 滤波参数 - EKF
EKF_PROCESS_NOISE = 0.5     # EKF过程噪声
EKF_MEASUREMENT_NOISE = 1.0 # EKF测量噪声


# ============================================================================
# 数据解析器（优化版：支持心跳包检测）
# ============================================================================

class UWBDataParser:
    """UWB数据帧解析器（低延迟优化版）"""
    
    FRAME_HEADER = b'\xff\xff\xff\xff'
    CMD_LOCATION = 0x2001   # 定位数据包
    CMD_HEARTBEAT = 0x2002  # 心跳包
    
    def __init__(self):
        self.buffer = bytearray()
        self.heartbeat_count = 0
        self.location_count = 0
        self.last_heartbeat_time = None
        self.last_location_time = None
        self.offline_warning_shown = False
    
    def parse(self, data: bytes) -> list:
        """解析数据，返回解析结果列表（批量处理优化）"""
        self.buffer.extend(data)
        results = []
        
        while len(self.buffer) >= 6:
            # 查找帧头
            header_idx = self.buffer.find(self.FRAME_HEADER)
            
            if header_idx < 0:
                self.buffer.clear()
                break
            
            # 丢弃帧头之前的无效数据
            if header_idx > 0:
                self.buffer = self.buffer[header_idx:]
            
            # 检查是否有足够数据读取长度字段
            if len(self.buffer) < 6:
                break
            
            # 读取包长度
            packet_length = struct.unpack('>H', self.buffer[4:6])[0]
            
            # 检查缓冲区是否有完整的包
            if len(self.buffer) < packet_length:
                break
            
            # 提取完整数据包
            frame = bytes(self.buffer[:packet_length])
            self.buffer = self.buffer[packet_length:]
            
            # 根据长度判断数据包类型
            if packet_length == 16:
                # 心跳包
                self._parse_heartbeat(frame)
            elif packet_length == 37:
                # 定位数据包
                result = self._parse_location(frame)
                if result:
                    results.append(result)
        
        return results
    
    def _parse_heartbeat(self, frame: bytes):
        """解析心跳包"""
        try:
            cmd = struct.unpack('>H', frame[8:10])[0]
            if cmd != self.CMD_HEARTBEAT:
                return
            
            self.heartbeat_count += 1
            self.last_heartbeat_time = time.time()
            
            # 检测信标是否离线（有心跳但超过5秒无定位数据）
            if self.last_location_time is not None:
                time_since_location = time.time() - self.last_location_time
                if time_since_location > 5 and not self.offline_warning_shown:
                    print(f"\n⚠️ 警告：已收到 {self.heartbeat_count} 个心跳包，"
                          f"但 {time_since_location:.1f} 秒内无定位数据，信标可能已关闭！")
                    self.offline_warning_shown = True
        except Exception:
            pass
    
    def _parse_location(self, frame: bytes) -> Optional[Dict]:
        """解析定位数据包"""
        try:
            # 验证校验和
            calculated_xor = 0
            for b in frame[:-1]:
                calculated_xor ^= b
            if calculated_xor != frame[-1]:
                return None
            
            # 解析命令码
            cmd = struct.unpack('>H', frame[8:10])[0]
            if cmd != self.CMD_LOCATION:
                return None
            
            # 解析数据字段
            anchor_id = struct.unpack('>I', frame[12:16])[0]
            tag_id = struct.unpack('>I', frame[16:20])[0]
            distance_cm = struct.unpack('>I', frame[20:24])[0]
            azimuth_deg = struct.unpack('>h', frame[24:26])[0]
            elevation_deg = struct.unpack('>h', frame[26:28])[0]
            
            # 球坐标转笛卡尔坐标
            az_rad = math.radians(azimuth_deg)
            el_rad = math.radians(elevation_deg)
            
            #忽略仰角
            x = distance_cm * math.sin(az_rad)
            y = distance_cm * math.cos(az_rad)
            z = 0.0
            
            self.location_count += 1
            self.last_location_time = time.time()
            
            # 信标恢复在线，重置警告标志
            if self.offline_warning_shown:
                print(f"\n✅ 信标已恢复在线！")
                self.offline_warning_shown = False
            
            return {
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
        except Exception:
            return None


# ============================================================================
# 串口接收器（低延迟优化版）
# ============================================================================

class SerialReceiver:
    """串口数据接收器（非阻塞低延迟版）"""
    
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.parser = UWBDataParser()
        self.data_queue = deque(maxlen=100)
        self.thread = None
    
    def connect(self) -> bool:
        """连接串口（非阻塞模式）"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE,
                timeout=0  # 非阻塞模式，降低延迟
            )
            print(f"✓ 串口 {self.port} 已连接（低延迟模式）")
            print(f"  波特率: {self.baudrate}")
            print("  优化特性:")
            print("    • 非阻塞串口读取")
            print("    • 批量数据处理")
            print("    • 心跳包检测")
            print(f"    • 角度突变抑制 (阈值: {ANGLE_THRESHOLD}°)")
            return True
        except Exception as e:
            print(f"✗ 串口连接失败: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("串口已断开")
    
    def start_receiving(self) -> bool:
        """开始接收数据"""
        if not self.serial or not self.serial.is_open:
            print("错误：串口未连接")
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        print("✓ 开始接收数据...")
        return True
    
    def _receive_loop(self):
        """数据接收循环（优化版：一次性读取所有可用数据）"""
        while self.running:
            try:
                # 一次性读取所有可用数据，减少延迟
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    results = self.parser.parse(data)
                    for result in results:
                        self.data_queue.append(result)
            except Exception:
                pass
            time.sleep(0.005)  # 5ms检查间隔，平衡延迟和CPU占用
    
    def get_latest_data(self) -> Optional[Dict]:
        """获取最新数据"""
        if self.data_queue:
            return self.data_queue.popleft()
        return None
    
    def get_all_data(self) -> list:
        """获取所有数据"""
        data_list = list(self.data_queue)
        self.data_queue.clear()
        return data_list


# ============================================================================
# 角度突变抑制器（来自UWB code文件夹的优化技术）
# ============================================================================

class AngleSmoother:
    """
    角度突变抑制器
    
    抑制规则（当角度变化超过阈值时）：
    - αk - αk-1 > threshold:  αexport = αk-1, dexport = dk-1 + 2
    - αk - αk-1 < -threshold: αexport = αk-1, dexport = dk-1 - 2
    - |αk - αk-1| ≤ threshold: αexport = αk, dexport = dk
    
    这种方法可以有效抑制人员遮挡导致的角度突变。
    """
    
    def __init__(self, threshold: float = ANGLE_THRESHOLD):
        """
        初始化角度平滑器
        :param threshold: 角度突变阈值（度），默认10°
        """
        self.threshold = threshold
        self.prev_angle = None
        self.prev_distance = None
        self.suppressed_count = 0
    
    def process(self, angle: float, distance: float) -> Tuple[float, float]:
        """
        处理角度和距离，抑制角度突变
        
        :param angle: 当前角度 αk
        :param distance: 当前距离 dk
        :return: (αexport, dexport) 输出角度和距离
        """
        if angle is None or distance is None:
            return angle, distance
        
        # 首次数据，直接输出
        if self.prev_angle is None or self.prev_distance is None:
            self.prev_angle = angle
            self.prev_distance = distance
            return angle, distance
        
        # 计算角度变化
        delta_angle = angle - self.prev_angle
        
        if delta_angle > self.threshold:
            # 角度正向突变超过阈值
            angle_export = self.prev_angle
            distance_export = self.prev_distance + 2
            self.suppressed_count += 1
        elif delta_angle < -self.threshold:
            # 角度负向突变超过阈值
            angle_export = self.prev_angle
            distance_export = self.prev_distance - 2
            self.suppressed_count += 1
        else:
            # 角度变化在阈值范围内，正常输出
            angle_export = angle
            distance_export = distance
        
        # 更新上一时刻的值（使用原始UWB数据）
        self.prev_angle = angle
        self.prev_distance = distance
        
        return angle_export, distance_export
    
    def reset(self):
        """重置"""
        self.prev_angle = None
        self.prev_distance = None
        self.suppressed_count = 0


# ============================================================================
# 中位数滤波器
# ============================================================================

class MedianFilter:
    """中位数滤波器"""
    
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
# 综合滤波器（整合角度突变抑制）
# ============================================================================

class UWBDataFilter:
    """
    UWB数据综合滤波器
    
    滤波流程：
        1. 物理约束检查
        2. 角度突变抑制（新增）
        3. 中位数滤波
        4. EKF状态估计
    """
    
    def __init__(self):
        # 物理约束检查器
        self.constraint_checker = PhysicalConstraintChecker(MAX_VELOCITY)
        
        # 角度突变抑制器（新增）
        self.angle_smoother = AngleSmoother(ANGLE_THRESHOLD)
        
        # 中位数滤波器
        self.median_filter = MedianFilter3D(MEDIAN_WINDOW_SIZE)
        
        # EKF滤波器
        self.ekf = ExtendedKalmanFilter(EKF_PROCESS_NOISE, EKF_MEASUREMENT_NOISE)
        
        # 统计
        self.total_count = 0
        self.outlier_count = 0
        self.angle_suppressed_count = 0
        self.last_valid_result = None
    
    def filter(self, distance: float, azimuth: float, elevation: float,
               x: float, y: float, z: float,
               current_time: float = None) -> Optional[Dict]:
        """
        滤波处理
        
        返回：
            {
                'x', 'y', 'z': 滤波后位置,
                'vx', 'vy', 'vz': 速度估计,
                'is_outlier': 是否为异常值,
                'angle_smoothed': 平滑后角度,
                'distance_smoothed': 平滑后距离
            }
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
        
        # Step 2: 角度突变抑制（新增）
        smoothed_azimuth, smoothed_distance = self.angle_smoother.process(azimuth, distance)
        self.angle_suppressed_count = self.angle_smoother.suppressed_count
        
        # 使用平滑后的数据重新计算坐标
        az_rad = math.radians(smoothed_azimuth)
        el_rad = math.radians(elevation)
        smooth_x = smoothed_distance * math.cos(el_rad) * math.sin(az_rad)
        smooth_y = smoothed_distance * math.cos(el_rad) * math.cos(az_rad)
        smooth_z = smoothed_distance * math.sin(el_rad)
        
        # Step 3: 中位数滤波
        med_x, med_y, med_z = self.median_filter.update(smooth_x, smooth_y, smooth_z)
        
        # Step 4: EKF状态估计
        ekf_result = self.ekf.update(med_x, med_y, med_z, current_time)
        
        result = {
            'x': ekf_result['x'],
            'y': ekf_result['y'],
            'z': ekf_result['z'],
            'vx': ekf_result['vx'],
            'vy': ekf_result['vy'],
            'vz': ekf_result['vz'],
            'is_outlier': False,
            'angle_smoothed': smoothed_azimuth,
            'distance_smoothed': smoothed_distance
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
            'outlier_rate': f"{rate * 100:.1f}%",
            'angle_suppressed': self.angle_suppressed_count
        }
    
    def reset(self):
        """重置"""
        self.constraint_checker.reset()
        self.angle_smoother.reset()
        self.median_filter.reset()
        self.ekf.reset()
        self.total_count = 0
        self.outlier_count = 0
        self.angle_suppressed_count = 0
        self.last_valid_result = None


# ============================================================================
# 可视化器
# ============================================================================

class FilteredPathVisualizer:
    """原始数据与滤波数据对比可视化器"""
    
    def __init__(self, receiver: SerialReceiver):
        self.receiver = receiver
        self.data_filter = UWBDataFilter()
        
        # 路径数据
        self.raw_path = deque(maxlen=MAX_PATH_POINTS)
        self.filtered_path = deque(maxlen=MAX_PATH_POINTS)
        
        # 当前位置
        self.raw_current = None
        self.filtered_current = None
        
        # 数据统计
        self.last_raw = {'distance': 0, 'azimuth': 0, 'elevation': 0, 'x': 0, 'y': 0, 'z': 0}
        self.last_filtered = {'x': 0, 'y': 0, 'z': 0, 'vx': 0, 'vy': 0, 'vz': 0}
        
        self._setup_plot()
    
    def _setup_plot(self):
        """设置图形界面"""
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False
        
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.canvas.manager.set_window_title('UWB 原始数据 vs 滤波数据 对比可视化')
        
        # 上排：原始数据
        self.ax_raw_xy = self.fig.add_subplot(241)
        self._setup_axis(self.ax_raw_xy, '原始 - 俯视图(X-Y)', 'X (cm)', 'Y (cm)', 'red')
        self.raw_line_xy, = self.ax_raw_xy.plot([], [], 'r-', linewidth=1.5, alpha=0.7)
        self.raw_point_xy, = self.ax_raw_xy.plot([], [], 'ro', markersize=8)
        
        self.ax_raw_xz = self.fig.add_subplot(242)
        self._setup_axis(self.ax_raw_xz, '原始 - 前视图(X-Z)', 'X (cm)', 'Z (cm)', 'red', half_z=True)
        self.raw_line_xz, = self.ax_raw_xz.plot([], [], 'r-', linewidth=1.5, alpha=0.7)
        self.raw_point_xz, = self.ax_raw_xz.plot([], [], 'ro', markersize=8)
        
        self.ax_raw_yz = self.fig.add_subplot(243)
        self._setup_axis(self.ax_raw_yz, '原始 - 侧视图(Y-Z)', 'Y (cm)', 'Z (cm)', 'red', half_z=True)
        self.raw_line_yz, = self.ax_raw_yz.plot([], [], 'r-', linewidth=1.5, alpha=0.7)
        self.raw_point_yz, = self.ax_raw_yz.plot([], [], 'ro', markersize=8)
        
        # 下排：滤波后数据
        self.ax_filt_xy = self.fig.add_subplot(245)
        self._setup_axis(self.ax_filt_xy, '滤波 - 俯视图(X-Y)', 'X (cm)', 'Y (cm)', 'blue')
        self.filt_line_xy, = self.ax_filt_xy.plot([], [], 'b-', linewidth=2, alpha=0.8)
        self.filt_point_xy, = self.ax_filt_xy.plot([], [], 'go', markersize=10)
        
        self.ax_filt_xz = self.fig.add_subplot(246)
        self._setup_axis(self.ax_filt_xz, '滤波 - 前视图(X-Z)', 'X (cm)', 'Z (cm)', 'blue', half_z=True)
        self.filt_line_xz, = self.ax_filt_xz.plot([], [], 'b-', linewidth=2, alpha=0.8)
        self.filt_point_xz, = self.ax_filt_xz.plot([], [], 'go', markersize=10)
        
        self.ax_filt_yz = self.fig.add_subplot(247)
        self._setup_axis(self.ax_filt_yz, '滤波 - 侧视图(Y-Z)', 'Y (cm)', 'Z (cm)', 'blue', half_z=True)
        self.filt_line_yz, = self.ax_filt_yz.plot([], [], 'b-', linewidth=2, alpha=0.8)
        self.filt_point_yz, = self.ax_filt_yz.plot([], [], 'go', markersize=10)
        
        # 信息面板
        self.ax_info = self.fig.add_subplot(244)
        self.ax_info.axis('off')
        self.info_text = self.ax_info.text(0.05, 0.95, "等待数据...",
            fontsize=9, family='monospace', verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # 统计面板
        self.ax_stats = self.fig.add_subplot(248)
        self.ax_stats.axis('off')
        self.stats_text = self.ax_stats.text(0.05, 0.95, "",
            fontsize=9, family='monospace', verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
        
        plt.tight_layout()
    
    def _setup_axis(self, ax, title: str, xlabel: str, ylabel: str, 
                    color: str, half_z: bool = False):
        """设置坐标轴"""
        ax.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        if half_z:
            ax.set_ylim(-DISPLAY_RANGE/2, DISPLAY_RANGE/2)
        else:
            ax.set_ylim(-DISPLAY_RANGE, DISPLAY_RANGE)
            ax.set_aspect('equal')
        ax.grid(True, linestyle='--', alpha=0.5)
        ax.set_xlabel(xlabel, fontsize=9)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.set_title(title, fontsize=10, fontweight='bold', color=color)
        ax.plot(0, 0, 's', markersize=10, color=color)
    
    def _update(self, frame):
        """动画更新函数"""
        # 获取所有新数据
        data_list = self.receiver.get_all_data()
        
        for data in data_list:
            # 保存原始数据
            raw_x, raw_y, raw_z = data['x'], data['y'], data['z']
            self.raw_path.append((raw_x, raw_y, raw_z, time.time()))
            self.raw_current = (raw_x, raw_y, raw_z)
            
            self.last_raw = {
                'distance': data['distance_cm'],
                'azimuth': data['azimuth_deg'],
                'elevation': data['elevation_deg'],
                'x': raw_x, 'y': raw_y, 'z': raw_z
            }
            
            # 滤波处理
            result = self.data_filter.filter(
                data['distance_cm'],
                data['azimuth_deg'],
                data['elevation_deg'],
                raw_x, raw_y, raw_z,
                data['timestamp']
            )
            
            if result:
                self.filtered_path.append((result['x'], result['y'], result['z'], time.time()))
                self.filtered_current = (result['x'], result['y'], result['z'])
                self.last_filtered = result
        
        # 移除过期的路径点
        current_time = time.time()
        while self.raw_path and current_time - self.raw_path[0][3] > PATH_FADE_TIME:
            self.raw_path.popleft()
        while self.filtered_path and current_time - self.filtered_path[0][3] > PATH_FADE_TIME:
            self.filtered_path.popleft()
        
        # 更新原始数据图形
        if self.raw_path:
            xs = [p[0] for p in self.raw_path]
            ys = [p[1] for p in self.raw_path]
            zs = [p[2] for p in self.raw_path]
            self.raw_line_xy.set_data(xs, ys)
            self.raw_line_xz.set_data(xs, zs)
            self.raw_line_yz.set_data(ys, zs)
        
        if self.raw_current:
            self.raw_point_xy.set_data([self.raw_current[0]], [self.raw_current[1]])
            self.raw_point_xz.set_data([self.raw_current[0]], [self.raw_current[2]])
            self.raw_point_yz.set_data([self.raw_current[1]], [self.raw_current[2]])
        
        # 更新滤波数据图形
        if self.filtered_path:
            xs = [p[0] for p in self.filtered_path]
            ys = [p[1] for p in self.filtered_path]
            zs = [p[2] for p in self.filtered_path]
            self.filt_line_xy.set_data(xs, ys)
            self.filt_line_xz.set_data(xs, zs)
            self.filt_line_yz.set_data(ys, zs)
        
        if self.filtered_current:
            self.filt_point_xy.set_data([self.filtered_current[0]], [self.filtered_current[1]])
            self.filt_point_xz.set_data([self.filtered_current[0]], [self.filtered_current[2]])
            self.filt_point_yz.set_data([self.filtered_current[1]], [self.filtered_current[2]])
        
        # 更新信息面板
        info = f"""【原始数据】
距离: {self.last_raw['distance']:.1f} cm
方位: {self.last_raw['azimuth']:.1f} deg
仰角: {self.last_raw['elevation']:.1f} deg
X: {self.last_raw['x']:.1f} cm
Y: {self.last_raw['y']:.1f} cm
Z: {self.last_raw['z']:.1f} cm

【滤波数据】
X: {self.last_filtered['x']:.1f} cm
Y: {self.last_filtered['y']:.1f} cm
Z: {self.last_filtered['z']:.1f} cm

【速度估计】
Vx: {self.last_filtered.get('vx', 0):.1f} cm/s
Vy: {self.last_filtered.get('vy', 0):.1f} cm/s
Vz: {self.last_filtered.get('vz', 0):.1f} cm/s"""
        self.info_text.set_text(info)
        
        # 更新统计面板
        stats = self.data_filter.get_statistics()
        stats_text = f"""【滤波统计】

滤波流程:
  1. 物理约束检查
  2. 角度突变抑制(阈值={ANGLE_THRESHOLD}°)
  3. 中位数滤波(窗口={MEDIAN_WINDOW_SIZE})
  4. EKF状态估计

数据统计:
  总数据:     {stats['total']}
  物理约束异常: {stats['outlier']}
  角度突变抑制: {stats['angle_suppressed']}
  有效数据:   {stats['valid']}
  异常率:     {stats['outlier_rate']}

参数设置:
  最大速度: {MAX_VELOCITY} cm/s
  距离范围: {MIN_DISTANCE}-{MAX_DISTANCE} cm"""
        self.stats_text.set_text(stats_text)
        
        return (self.raw_line_xy, self.raw_line_xz, self.raw_line_yz,
                self.raw_point_xy, self.raw_point_xz, self.raw_point_yz,
                self.filt_line_xy, self.filt_line_xz, self.filt_line_yz,
                self.filt_point_xy, self.filt_point_xz, self.filt_point_yz,
                self.info_text, self.stats_text)
    
    def run(self):
        """运行可视化"""
        print("\n可视化说明：")
        print("  上排（红色）：原始数据")
        print("  下排（蓝色）：滤波后数据（含EKF速度估计）")
        print("  绿色圆点：当前位置")
        print("\n关闭窗口或按 Ctrl+C 退出程序")
        
        ani = animation.FuncAnimation(
            self.fig, self._update,
            interval=UPDATE_INTERVAL,
            blit=True,
            cache_frame_data=False
        )
        plt.show()


# ============================================================================
# 辅助函数
# ============================================================================

def list_serial_ports() -> list:
    """列出所有可用串口"""
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
============================================================================
     UWB单基站跟随套件 - 数据滤波对比可视化程序（低延迟优化版）
============================================================================

用法：python uwb_filtered_visualizer.py <串口号>

选项：
  <串口号>      指定串口，如 COM3 或 /dev/ttyUSB0
  --list        列出所有可用串口
  --help        显示此帮助信息

示例：
  python uwb_filtered_visualizer.py COM3
  python uwb_filtered_visualizer.py /dev/ttyUSB0

功能：
  - 上排（红色）：原始数据的三个二维平面视图
  - 下排（蓝色）：滤波后数据的三个二维平面视图
  - 整合EKF扩展卡尔曼滤波，输出速度估计

优化特性（参考UWB code文件夹）：
  • 非阻塞串口读取 - 降低延迟
  • 批量数据处理 - 提高效率
  • 心跳包检测 - 监测信标状态
  • 角度突变抑制 - 抑制人员遮挡导致的角度跳变

滤波流程：
  1. 物理约束检查（范围+速度限制）
  2. 角度突变抑制（阈值=10°）
  3. 中位数滤波（窗口=5）
  4. EKF状态估计（位置+速度）

依赖安装：
  pip install pyserial matplotlib numpy

============================================================================
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
            print(f"\n提示：请指定串口号：")
            print(f"  python {sys.argv[0]} {ports[0]}")
        return
    
    arg = sys.argv[1]
    
    if arg in ['--help', '-h']:
        print_usage()
        return
    
    if arg == '--list':
        list_serial_ports()
        return
    
    # 连接串口
    port = arg
    print("\n" + "="*70)
    print(f"    UWB 数据滤波对比可视化（低延迟优化版）- 串口 {port}")
    print("="*70)
    
    receiver = SerialReceiver(port)
    
    if not receiver.connect():
        return
    
    if not receiver.start_receiving():
        receiver.disconnect()
        return
    
    visualizer = FilteredPathVisualizer(receiver)
    
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        receiver.disconnect()
        print("程序已退出")


if __name__ == '__main__':
    main()
