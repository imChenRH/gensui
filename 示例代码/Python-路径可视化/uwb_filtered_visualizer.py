#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
UWB单基站跟随套件 - 原始数据与滤波数据对比可视化程序
============================================================================

功能说明：
    本程序从UWB基站接收串口数据，同时显示原始数据和滤波后数据的移动路径。
    使用六个二维平面视图进行对比：
    - 上排：原始数据的俯视图(X-Y)、前视图(X-Z)、侧视图(Y-Z)
    - 下排：滤波后数据的俯视图(X-Y)、前视图(X-Z)、侧视图(Y-Z)

硬件连接：
    - UWB基站通过USB转TTL模块连接到电脑
    - 串口参数：115200波特率，8数据位，1停止位，无校验

依赖安装：
    pip install pyserial matplotlib numpy

使用方法：
    python uwb_filtered_visualizer.py [串口号]
    例如：python uwb_filtered_visualizer.py COM3
    
    模拟模式（含异常值）：
    python uwb_filtered_visualizer.py --simulate
    
作者：Copilot
日期：2026-01-22
============================================================================
"""

import sys
import math
import struct
import threading
import time
from collections import deque
from typing import Tuple, Optional, Dict
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
# 卡尔曼滤波器（从uwb_data_filter.py提取）
# ============================================================================

class KalmanFilter1D:
    """一维卡尔曼滤波器"""
    
    def __init__(self, q=0.1, r=0.5, initial_value=0.0):
        self.q = q  # 过程噪声
        self.r = r  # 测量噪声
        self.x = initial_value  # 状态估计值
        self.p = 1.0  # 估计误差协方差
        self.k = 0.0  # 卡尔曼增益
        self.initialized = False
    
    def update(self, measurement: float) -> float:
        if not self.initialized:
            self.x = measurement
            self.initialized = True
            return self.x
        
        p_pred = self.p + self.q
        self.k = p_pred / (p_pred + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (1 - self.k) * p_pred
        
        return self.x
    
    def reset(self):
        self.x = 0.0
        self.p = 1.0
        self.k = 0.0
        self.initialized = False


class KalmanFilter3D:
    """三维卡尔曼滤波器"""
    
    def __init__(self, q=0.1, r=0.5):
        self.filter_x = KalmanFilter1D(q, r)
        self.filter_y = KalmanFilter1D(q, r)
        self.filter_z = KalmanFilter1D(q, r)
    
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
    """异常值检测器"""
    
    def __init__(self, 
                 max_velocity: float = 200.0,
                 max_distance_jump: float = 50.0):
        self.max_velocity = max_velocity
        self.max_distance_jump = max_distance_jump
        
        self.last_position = None
        self.last_time = None
        self.last_distance = None
    
    def is_outlier_by_velocity(self, x: float, y: float, z: float, 
                                current_time: float = None) -> bool:
        if current_time is None:
            current_time = time.time()
        
        if self.last_position is None or self.last_time is None:
            self.last_position = (x, y, z)
            self.last_time = current_time
            return False
        
        dx = x - self.last_position[0]
        dy = y - self.last_position[1]
        dz = z - self.last_position[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.01
        
        velocity = distance / dt
        is_outlier = velocity > self.max_velocity
        
        if not is_outlier:
            self.last_position = (x, y, z)
            self.last_time = current_time
        
        return is_outlier
    
    def is_outlier_by_distance_jump(self, distance: float) -> bool:
        if self.last_distance is None:
            self.last_distance = distance
            return False
        
        jump = abs(distance - self.last_distance)
        is_outlier = jump > self.max_distance_jump
        
        if not is_outlier:
            self.last_distance = distance
        
        return is_outlier
    
    def reset(self):
        self.last_position = None
        self.last_time = None
        self.last_distance = None


# ============================================================================
# 综合数据滤波器
# ============================================================================

class UWBDataFilter:
    """UWB数据综合滤波器"""
    
    def __init__(self,
                 max_velocity: float = 200.0,
                 max_distance_jump: float = 50.0,
                 kalman_q: float = 0.1,
                 kalman_r: float = 0.5):
        
        self.outlier_detector = OutlierDetector(
            max_velocity=max_velocity,
            max_distance_jump=max_distance_jump
        )
        
        self.kalman_filter = KalmanFilter3D(q=kalman_q, r=kalman_r)
        self.distance_kalman = KalmanFilter1D(q=kalman_q, r=kalman_r)
        
        self.total_count = 0
        self.outlier_count = 0
        self.last_valid_data = None
    
    def filter(self, distance: float, x: float, y: float, z: float,
               current_time: float = None) -> Optional[Dict]:
        
        if current_time is None:
            current_time = time.time()
        
        self.total_count += 1
        
        is_outlier = False
        
        if self.outlier_detector.is_outlier_by_distance_jump(distance):
            is_outlier = True
        
        if not is_outlier and self.outlier_detector.is_outlier_by_velocity(x, y, z, current_time):
            is_outlier = True
        
        if is_outlier:
            self.outlier_count += 1
            if self.last_valid_data:
                return {**self.last_valid_data, 'is_outlier': True}
            return None
        
        filtered_x, filtered_y, filtered_z = self.kalman_filter.update(x, y, z)
        filtered_distance = self.distance_kalman.update(distance)
        
        result = {
            'distance': round(filtered_distance, 2),
            'x': round(filtered_x, 2),
            'y': round(filtered_y, 2),
            'z': round(filtered_z, 2),
            'is_outlier': False
        }
        
        self.last_valid_data = result.copy()
        return result
    
    def get_statistics(self) -> Dict:
        outlier_rate = self.outlier_count / self.total_count if self.total_count > 0 else 0
        return {
            'total_count': self.total_count,
            'outlier_count': self.outlier_count,
            'valid_count': self.total_count - self.outlier_count,
            'outlier_rate': f"{outlier_rate * 100:.1f}%"
        }
    
    def reset(self):
        self.outlier_detector.reset()
        self.kalman_filter.reset()
        self.distance_kalman.reset()
        self.total_count = 0
        self.outlier_count = 0
        self.last_valid_data = None


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
    """原始数据与滤波数据对比可视化器"""
    
    def __init__(self, receiver):
        self.receiver = receiver
        
        # 滤波器
        self.data_filter = UWBDataFilter(
            max_velocity=250.0,        # 最大速度
            max_distance_jump=60.0,    # 最大距离跳变
            kalman_q=0.5,              # 卡尔曼过程噪声（较大，响应快）
            kalman_r=0.3               # 卡尔曼测量噪声（较小，跟踪紧密）
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
        self.last_filtered_data = {'distance': 0, 'x': 0, 'y': 0, 'z': 0}
        
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
            
            # 滤波处理
            filtered = self.data_filter.filter(
                data['distance_cm'], raw_x, raw_y, raw_z, current_time
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
                    'z': filt_z
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
        else:
            info_text = "等待数据...\n\n请确保：\n1. 串口已连接\n2. 基站已上电\n3. 信标在范围内"
        
        self.info_box.set_text(info_text)
        
        # 更新统计信息
        stats = self.data_filter.get_statistics()
        stats_text = (
            f"━━━ 滤波统计 ━━━\n\n"
            f"总数据点:  {stats['total_count']:>6d}\n"
            f"异常值:    {stats['outlier_count']:>6d}\n"
            f"有效数据:  {stats['valid_count']:>6d}\n"
            f"异常率:    {stats['outlier_rate']:>6s}\n\n"
            f"━━━ 滤波参数 ━━━\n\n"
            f"最大速度:   250 cm/s\n"
            f"最大跳变:   60 cm\n"
            f"卡尔曼Q:    0.5\n"
            f"卡尔曼R:    0.3\n\n"
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
║  用法：python uwb_filtered_visualizer.py [选项]                    ║
║                                                                    ║
║  选项：                                                            ║
║    <串口号>      指定串口，如 COM3 或 /dev/ttyUSB0                 ║
║    --list        列出所有可用串口                                  ║
║    --simulate    使用模拟数据（含异常值，测试滤波效果）            ║
║    --help        显示此帮助信息                                    ║
║                                                                    ║
║  示例：                                                            ║
║    python uwb_filtered_visualizer.py COM3                          ║
║    python uwb_filtered_visualizer.py --simulate                    ║
║                                                                    ║
║  功能：                                                            ║
║    同时显示原始数据和滤波后数据的三个二维平面视图                  ║
║    - 上排（红色）：原始数据的俯视图、前视图、侧视图                ║
║    - 下排（蓝色）：滤波后数据的俯视图、前视图、侧视图              ║
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
        return
    
    arg = sys.argv[1]
    
    if arg in ['--help', '-h']:
        print_usage()
        return
    
    if arg == '--list':
        list_serial_ports()
        return
    
    if arg == '--simulate':
        print("\n" + "="*60)
        print("    模拟模式 - 含异常值，测试滤波效果")
        print("="*60)
        receiver = SimulatedReceiver()
    else:
        port = arg
        print("\n" + "="*60)
        print(f"    UWB 原始数据 vs 滤波数据 对比 - 串口 {port}")
        print("="*60)
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
