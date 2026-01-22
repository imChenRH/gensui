#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
UWB单基站跟随套件 - 二维平面移动路径可视化程序
============================================================================

功能说明：
    本程序从UWB基站接收串口数据，实时可视化显示信标的移动路径。
    使用三个二维平面视图：俯视图(X-Y)、前视图(X-Z)、侧视图(Y-Z)

硬件连接：
    - UWB基站通过USB转TTL模块连接到电脑
    - 串口参数：115200波特率，8数据位，1停止位，无校验

依赖安装：
    pip install pyserial matplotlib numpy

使用方法：
    python uwb_path_visualizer.py [串口号]
    例如：python uwb_path_visualizer.py COM3
    
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
# 数据解析类
# ============================================================================

class UWBDataParser:
    """
    UWB数据帧解析器
    
    协议格式（共37字节）：
    - MessageHeader:   4字节  帧头 0xFFFFFFFF
    - PacketLength:    2字节  消息体长度
    - SequenceID:      2字节  消息流水号
    - RequestCommand:  2字节  命令码 0x2001
    - VersionID:       2字节  协议版本 0x0100
    - AnchorID:        4字节  基站ID
    - TagID:           4字节  标签ID
    - Distance:        4字节  距离，单位：cm
    - Azimuth:         2字节  方位角，单位：度（有符号）
    - Elevation:       2字节  仰角，单位：度（有符号）
    - TagStatus:       2字节  标签状态
    - BatchSn:         2字节  测距序号
    - Reserve:         4字节  预留
    - XorByte:         1字节  异或校验
    """
    
    def __init__(self):
        self.buffer = b''
    
    def calculate_xor(self, data):
        """计算异或校验值"""
        xor_value = 0
        for byte in data:
            xor_value ^= byte
        return xor_value
    
    def parse(self, new_data):
        """
        解析接收到的数据
        
        参数：
            new_data: 新接收的字节数据
            
        返回：
            解析成功返回字典，包含距离、角度等信息
            解析失败返回 None
        """
        self.buffer += new_data
        results = []
        
        while True:
            # 查找帧头
            header_idx = self.buffer.find(FRAME_HEADER)
            
            if header_idx < 0:
                # 未找到帧头，保留最后3字节（可能是不完整的帧头）
                if len(self.buffer) > 3:
                    self.buffer = self.buffer[-3:]
                break
            
            # 移除帧头之前的无效数据
            if header_idx > 0:
                self.buffer = self.buffer[header_idx:]
            
            # 检查数据长度是否足够
            if len(self.buffer) < FRAME_LENGTH:
                break
            
            # 提取一帧数据
            frame = self.buffer[:FRAME_LENGTH]
            
            # 验证异或校验
            calculated_xor = self.calculate_xor(frame[:-1])
            received_xor = frame[-1]
            
            if calculated_xor != received_xor:
                # 校验失败，跳过这个帧头，继续搜索
                self.buffer = self.buffer[4:]
                continue
            
            # 解析数据（大端序）
            try:
                # 协议字段偏移量：
                # MessageHeader:   0-3   (4字节)
                # PacketLength:    4-5   (2字节)
                # SequenceID:      6-7   (2字节)
                # RequestCommand:  8-9   (2字节)
                # VersionID:       10-11 (2字节)
                # AnchorID:        12-15 (4字节)
                # TagID:           16-19 (4字节)
                # Distance:        20-23 (4字节)
                # Azimuth:         24-25 (2字节)
                # Elevation:       26-27 (2字节)
                
                command = struct.unpack('>H', frame[8:10])[0]
                
                if command == CMD_POSITION:
                    # 解析各字段（修正字节偏移）
                    anchor_id = struct.unpack('>I', frame[12:16])[0]
                    tag_id = struct.unpack('>I', frame[16:20])[0]
                    distance_cm = struct.unpack('>I', frame[20:24])[0]
                    azimuth_deg = struct.unpack('>h', frame[24:26])[0]  # 有符号
                    elevation_deg = struct.unpack('>h', frame[26:28])[0]  # 有符号
                    
                    # 将球坐标转换为笛卡尔坐标
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
            
            # 移除已处理的数据
            self.buffer = self.buffer[FRAME_LENGTH:]
        
        return results if results else None
    
    def spherical_to_cartesian(self, distance_cm, azimuth_deg, elevation_deg):
        """
        将球坐标转换为笛卡尔坐标
        
        参数：
            distance_cm:   距离（厘米）
            azimuth_deg:   方位角（度），正前方为0度，顺时针为正
            elevation_deg: 俯仰角（度），水平为0度，向上为正
            
        返回：
            (x, y, z) 坐标，单位：厘米
            
        坐标系定义：
            - Y轴：正前方
            - X轴：右侧为正
            - Z轴：向上为正
        """
        # 转换为弧度
        azimuth_rad = math.radians(azimuth_deg)
        elevation_rad = math.radians(elevation_deg)
        
        # 计算水平投影距离
        horizontal_distance = distance_cm * math.cos(elevation_rad)
        
        # 计算XYZ坐标
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
        """
        初始化串口接收器
        
        参数：
            port: 串口号，如 'COM3' 或 '/dev/ttyUSB0'
            baudrate: 波特率，默认115200
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.running = False
        self.parser = UWBDataParser()
        self.data_queue = deque(maxlen=100)  # 数据队列
        self.thread = None
    
    def connect(self):
        """连接串口"""
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
        """断开串口连接"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("串口已断开")
    
    def start_receiving(self):
        """开始接收数据（在新线程中）"""
        if not self.serial or not self.serial.is_open:
            print("错误：串口未连接")
            return False
        
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        print("✓ 开始接收数据...")
        return True
    
    def _receive_loop(self):
        """数据接收循环（在后台线程中运行）"""
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
    
    def get_latest_data(self):
        """获取最新的数据"""
        if self.data_queue:
            return self.data_queue.popleft()
        return None
    
    def get_all_data(self):
        """获取所有待处理的数据"""
        data_list = list(self.data_queue)
        self.data_queue.clear()
        return data_list


# ============================================================================
# 可视化类（三个二维平面版本）
# ============================================================================

class PathVisualizer:
    """二维平面路径可视化器（三个视图）"""
    
    def __init__(self, receiver):
        """
        初始化可视化器
        
        参数：
            receiver: SerialReceiver 实例
        """
        self.receiver = receiver
        
        # 路径数据存储（包含时间戳用于渐隐效果）
        self.path_data = deque(maxlen=MAX_PATH_POINTS)  # 存储 (x, y, z, timestamp)
        self.current_pos = None
        
        # 统计信息
        self.data_count = 0
        self.last_distance = 0
        self.last_azimuth = 0
        self.last_elevation = 0
        self.last_x = 0
        self.last_y = 0
        self.last_z = 0
        
        # 创建图形
        self.setup_plot()
    
    def setup_plot(self):
        """设置三个二维平面图形界面"""
        # 设置中文字体支持
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False
        
        # 创建2x2布局的图形
        self.fig = plt.figure(figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('UWB 二维平面移动路径可视化')
        
        # 创建俯视图 (X-Y平面) - 左上
        self.ax_xy = self.fig.add_subplot(221)
        
        # 创建前视图 (X-Z平面) - 右上
        self.ax_xz = self.fig.add_subplot(222)
        
        # 创建侧视图 (Y-Z平面) - 左下
        self.ax_yz = self.fig.add_subplot(223)
        
        # ==================== 俯视图 (X-Y平面) ====================
        self.ax_xy.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_xy.set_ylim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_xy.set_aspect('equal')
        self.ax_xy.grid(True, linestyle='--', alpha=0.5)
        self.ax_xy.set_xlabel('X (cm) - 左右', fontsize=10)
        self.ax_xy.set_ylabel('Y (cm) - 前后', fontsize=10)
        self.ax_xy.set_title('俯视图 (X-Y平面)', fontsize=12, fontweight='bold')
        self.ax_xy.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
        self.ax_xy.axvline(x=0, color='gray', linestyle='-', alpha=0.3)
        self.ax_xy.plot(0, 0, 'rs', markersize=12, label='基站')
        self.path_line_xy, = self.ax_xy.plot([], [], 'b-', linewidth=2, alpha=0.7, label='轨迹')
        self.current_point_xy, = self.ax_xy.plot([], [], 'go', markersize=10, label='当前')
        self.ax_xy.legend(loc='upper right', fontsize=9)
        
        # ==================== 前视图 (X-Z平面) ====================
        self.ax_xz.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_xz.set_ylim(-DISPLAY_RANGE/2, DISPLAY_RANGE/2)
        self.ax_xz.grid(True, linestyle='--', alpha=0.5)
        self.ax_xz.set_xlabel('X (cm) - 左右', fontsize=10)
        self.ax_xz.set_ylabel('Z (cm) - 上下', fontsize=10)
        self.ax_xz.set_title('前视图 (X-Z平面)', fontsize=12, fontweight='bold')
        self.ax_xz.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
        self.ax_xz.axvline(x=0, color='gray', linestyle='-', alpha=0.3)
        self.ax_xz.plot(0, 0, 'rs', markersize=12)
        self.path_line_xz, = self.ax_xz.plot([], [], 'b-', linewidth=2, alpha=0.7)
        self.current_point_xz, = self.ax_xz.plot([], [], 'go', markersize=10)
        
        # ==================== 侧视图 (Y-Z平面) ====================
        self.ax_yz.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax_yz.set_ylim(-DISPLAY_RANGE/2, DISPLAY_RANGE/2)
        self.ax_yz.grid(True, linestyle='--', alpha=0.5)
        self.ax_yz.set_xlabel('Y (cm) - 前后', fontsize=10)
        self.ax_yz.set_ylabel('Z (cm) - 上下', fontsize=10)
        self.ax_yz.set_title('侧视图 (Y-Z平面)', fontsize=12, fontweight='bold')
        self.ax_yz.axhline(y=0, color='gray', linestyle='-', alpha=0.3)
        self.ax_yz.axvline(x=0, color='gray', linestyle='-', alpha=0.3)
        self.ax_yz.plot(0, 0, 'rs', markersize=12)
        self.path_line_yz, = self.ax_yz.plot([], [], 'b-', linewidth=2, alpha=0.7)
        self.current_point_yz, = self.ax_yz.plot([], [], 'go', markersize=10)
        
        # 创建信息文本框 - 右下
        self.ax_info = self.fig.add_subplot(224)
        self.ax_info.axis('off')
        info_text = "等待数据..."
        self.info_box = self.ax_info.text(
            0.1, 0.9, info_text,
            fontsize=11,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9),
            family='monospace',
            transform=self.ax_info.transAxes
        )
        
        # 紧凑布局
        plt.tight_layout()
    
    def update(self, frame):
        """
        更新图形（由动画函数调用）
        
        参数：
            frame: 帧编号（由matplotlib自动传入）
        """
        current_time = time.time()
        
        # 获取所有新数据
        data_list = self.receiver.get_all_data()
        
        for data in data_list:
            self.data_count += 1
            x, y, z = data['x'], data['y'], data['z']
            
            # 添加路径点（包含时间戳）
            self.path_data.append((x, y, z, current_time))
            self.current_pos = (x, y, z)
            
            # 更新统计信息
            self.last_distance = data['distance_cm']
            self.last_azimuth = data['azimuth_deg']
            self.last_elevation = data['elevation_deg']
            self.last_x = x
            self.last_y = y
            self.last_z = z
        
        # 移除过期的路径点（渐隐效果）
        while self.path_data and (current_time - self.path_data[0][3]) > PATH_FADE_TIME:
            self.path_data.popleft()
        
        # 提取路径坐标
        if len(self.path_data) > 0:
            path_x = [p[0] for p in self.path_data]
            path_y = [p[1] for p in self.path_data]
            path_z = [p[2] for p in self.path_data]
            
            # 更新俯视图路径 (X-Y)
            self.path_line_xy.set_data(path_x, path_y)
            
            # 更新前视图路径 (X-Z)
            self.path_line_xz.set_data(path_x, path_z)
            
            # 更新侧视图路径 (Y-Z)
            self.path_line_yz.set_data(path_y, path_z)
        
        # 更新当前位置点
        if self.current_pos:
            x, y, z = self.current_pos
            
            # 更新俯视图当前位置
            self.current_point_xy.set_data([x], [y])
            
            # 更新前视图当前位置
            self.current_point_xz.set_data([x], [z])
            
            # 更新侧视图当前位置
            self.current_point_yz.set_data([y], [z])
        
        # 更新信息文本
        if self.current_pos:
            info_text = (
                f"━━━━━━ 实时数据 ━━━━━━\n\n"
                f"  数据点数:  {self.data_count}\n\n"
                f"━━━━━ 原始数据 ━━━━━\n\n"
                f"  距离:      {self.last_distance:6d} cm\n"
                f"  方位角:    {self.last_azimuth:6d}°\n"
                f"  仰角:      {self.last_elevation:6d}°\n\n"
                f"━━━━━ 转换坐标 ━━━━━\n\n"
                f"  X坐标:     {self.last_x:8.1f} cm\n"
                f"  Y坐标:     {self.last_y:8.1f} cm\n"
                f"  Z坐标:     {self.last_z:8.1f} cm\n\n"
                f"━━━━━━━━━━━━━━━━━━━━\n\n"
                f"  路径点:    {len(self.path_data):3d}/{MAX_PATH_POINTS}\n"
                f"  残留时间:  {PATH_FADE_TIME:.1f}秒"
            )
        else:
            info_text = (
                "等待数据...\n\n"
                "请确保：\n"
                "1. 串口已正确连接\n"
                "2. 基站已上电\n"
                "3. 信标在范围内"
            )
        
        self.info_box.set_text(info_text)
        
        # 自动调整显示范围
        if len(self.path_data) > 0:
            path_x = [p[0] for p in self.path_data]
            path_y = [p[1] for p in self.path_data]
            path_z = [p[2] for p in self.path_data]
            
            x_range = max(abs(min(path_x)), abs(max(path_x)), 50) * 1.3
            y_range = max(abs(min(path_y)), abs(max(path_y)), 50) * 1.3
            z_range = max(abs(min(path_z)), abs(max(path_z)), 20) * 1.5
            
            max_xy_range = max(x_range, y_range, DISPLAY_RANGE)
            max_z_range = max(z_range, DISPLAY_RANGE/4)
            
            # 更新各视图范围
            self.ax_xy.set_xlim(-max_xy_range, max_xy_range)
            self.ax_xy.set_ylim(-max_xy_range, max_xy_range)
            
            self.ax_xz.set_xlim(-max_xy_range, max_xy_range)
            self.ax_xz.set_ylim(-max_z_range, max_z_range)
            
            self.ax_yz.set_xlim(-max_xy_range, max_xy_range)
            self.ax_yz.set_ylim(-max_z_range, max_z_range)
        
        return self.path_line_xy, self.current_point_xy, self.info_box
    
    def run(self):
        """运行可视化"""
        # 创建动画
        self.ani = animation.FuncAnimation(
            self.fig,
            self.update,
            interval=UPDATE_INTERVAL,
            blit=False,
            cache_frame_data=False
        )
        
        # 显示窗口
        print("\n✓ 二维平面可视化窗口已打开")
        print("  - 左上：俯视图 (X-Y平面)")
        print("  - 右上：前视图 (X-Z平面)")
        print("  - 左下：侧视图 (Y-Z平面)")
        print("  - 右下：实时数据信息")
        print("  - 绿色圆点：当前位置")
        print("  - 蓝色线条：移动路径")
        print("  - 红色方块：基站位置")
        print(f"  - 路径残留时间：{PATH_FADE_TIME}秒")
        print("\n关闭窗口或按 Ctrl+C 退出程序")
        
        plt.show()


# ============================================================================
# 模拟数据生成器（用于测试）
# ============================================================================

class SimulatedReceiver:
    """模拟数据接收器（用于无硬件时测试）"""
    
    def __init__(self):
        self.data_queue = deque(maxlen=100)
        self.running = False
        self.thread = None
        self.angle = 0
        self.radius = 30  # 起始半径（厘米）
        self.z_angle = 0
    
    def connect(self):
        print("✓ 模拟模式已启动（无需真实硬件）")
        return True
    
    def disconnect(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
    
    def start_receiving(self):
        self.running = True
        self.thread = threading.Thread(target=self._generate_loop, daemon=True)
        self.thread.start()
        print("✓ 开始生成模拟数据（螺旋轨迹）...")
        return True
    
    def _generate_loop(self):
        """生成模拟数据"""
        while self.running:
            # 生成螺旋线轨迹
            self.angle += 8
            self.radius += 0.2
            self.z_angle += 3  # Z轴周期性变化
            
            # 添加一些随机噪声
            noise_x = np.random.normal(0, 1)
            noise_y = np.random.normal(0, 1)
            noise_z = np.random.normal(0, 0.5)
            
            # 计算XY坐标（螺旋线）
            x = self.radius * math.sin(math.radians(self.angle)) + noise_x
            y = self.radius * math.cos(math.radians(self.angle)) + noise_y
            
            # 计算Z坐标（正弦波动，范围约±15cm）
            z = 15 * math.sin(math.radians(self.z_angle)) + noise_z
            
            # 计算距离和角度
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
            
            time.sleep(0.1)  # 10Hz 数据率
    
    def get_latest_data(self):
        if self.data_queue:
            return self.data_queue.popleft()
        return None
    
    def get_all_data(self):
        data_list = list(self.data_queue)
        self.data_queue.clear()
        return data_list


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
    print("-" * 50)
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")
    print("-" * 50)
    
    return [port.device for port in ports]


def print_usage():
    """打印使用说明"""
    print("""
╔══════════════════════════════════════════════════════════════╗
║         UWB单基站跟随套件 - 移动路径可视化程序               ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  用法：python uwb_path_visualizer.py [选项]                  ║
║                                                              ║
║  选项：                                                      ║
║    <串口号>      指定串口，如 COM3 或 /dev/ttyUSB0           ║
║    --list        列出所有可用串口                            ║
║    --simulate    使用模拟数据（无需硬件）                    ║
║    --help        显示此帮助信息                              ║
║                                                              ║
║  示例：                                                      ║
║    python uwb_path_visualizer.py COM3                        ║
║    python uwb_path_visualizer.py --simulate                  ║
║    python uwb_path_visualizer.py --list                      ║
║                                                              ║
║  依赖安装：                                                  ║
║    pip install pyserial matplotlib numpy                     ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
""")


# ============================================================================
# 主函数
# ============================================================================

def main():
    """主函数"""
    # 解析命令行参数
    if len(sys.argv) < 2:
        print_usage()
        ports = list_serial_ports()
        if ports:
            print("\n提示：请指定串口号，例如：")
            print(f"  python {sys.argv[0]} {ports[0]}")
            print(f"\n或使用模拟模式测试：")
            print(f"  python {sys.argv[0]} --simulate")
        return
    
    arg = sys.argv[1]
    
    # 帮助信息
    if arg in ['--help', '-h']:
        print_usage()
        return
    
    # 列出串口
    if arg == '--list':
        list_serial_ports()
        return
    
    # 模拟模式
    if arg == '--simulate':
        print("\n" + "="*50)
        print("       模拟模式 - 无需真实硬件")
        print("="*50)
        receiver = SimulatedReceiver()
    else:
        # 真实串口模式
        port = arg
        print("\n" + "="*50)
        print(f"    UWB 路径可视化 - 串口 {port}")
        print("="*50)
        receiver = SerialReceiver(port)
    
    # 连接
    if not receiver.connect():
        return
    
    # 开始接收数据
    if not receiver.start_receiving():
        receiver.disconnect()
        return
    
    # 创建可视化器
    visualizer = PathVisualizer(receiver)
    
    try:
        # 运行可视化
        visualizer.run()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        # 清理
        receiver.disconnect()
        print("程序已退出")


if __name__ == '__main__':
    main()
