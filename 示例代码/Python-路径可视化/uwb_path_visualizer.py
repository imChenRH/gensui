#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
============================================================================
UWB单基站跟随套件 - 移动路径可视化程序
============================================================================

功能说明：
    本程序从UWB基站接收串口数据，实时可视化显示信标的移动路径。
    支持实时轨迹绘制、历史路径显示、坐标网格等功能。

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
    from matplotlib.patches import Circle, Arrow
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
MAX_PATH_POINTS = 500       # 最大保留路径点数
UPDATE_INTERVAL = 50        # 图形更新间隔（毫秒）
GRID_SIZE = 50              # 网格大小（厘米）
DISPLAY_RANGE = 500         # 默认显示范围（厘米）

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
                # 命令字在第8-9字节
                command = struct.unpack('>H', frame[8:10])[0]
                
                if command == CMD_POSITION:
                    # 解析各字段
                    anchor_id = struct.unpack('>I', frame[14:18])[0]
                    tag_id = struct.unpack('>I', frame[18:22])[0]
                    distance_cm = struct.unpack('>I', frame[22:26])[0]
                    azimuth_deg = struct.unpack('>h', frame[26:28])[0]  # 有符号
                    elevation_deg = struct.unpack('>h', frame[28:30])[0]  # 有符号
                    
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
# 可视化类
# ============================================================================

class PathVisualizer:
    """路径可视化器"""
    
    def __init__(self, receiver):
        """
        初始化可视化器
        
        参数：
            receiver: SerialReceiver 实例
        """
        self.receiver = receiver
        
        # 路径数据存储
        self.path_x = deque(maxlen=MAX_PATH_POINTS)
        self.path_y = deque(maxlen=MAX_PATH_POINTS)
        self.current_pos = None
        
        # 统计信息
        self.data_count = 0
        self.last_distance = 0
        self.last_azimuth = 0
        
        # 创建图形
        self.setup_plot()
    
    def setup_plot(self):
        """设置图形界面"""
        # 设置中文字体支持
        plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
        plt.rcParams['axes.unicode_minus'] = False
        
        # 创建图形和坐标轴
        self.fig, self.ax = plt.subplots(1, 1, figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('UWB 移动路径可视化')
        
        # 设置坐标轴
        self.ax.set_xlim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax.set_ylim(-DISPLAY_RANGE, DISPLAY_RANGE)
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.set_xlabel('X 坐标 (cm) - 左右方向', fontsize=12)
        self.ax.set_ylabel('Y 坐标 (cm) - 前后方向', fontsize=12)
        self.ax.set_title('UWB 信标移动路径实时显示', fontsize=14, fontweight='bold')
        
        # 绘制基站位置（原点）
        self.ax.plot(0, 0, 'rs', markersize=15, label='基站位置')
        self.ax.annotate('基站', (0, 0), textcoords="offset points", 
                        xytext=(10, 10), ha='left', fontsize=10, color='red')
        
        # 绘制方向指示
        self.ax.annotate('', xy=(0, DISPLAY_RANGE*0.9), xytext=(0, DISPLAY_RANGE*0.7),
                        arrowprops=dict(arrowstyle='->', color='green', lw=2))
        self.ax.text(0, DISPLAY_RANGE*0.95, '前方(Y+)', ha='center', fontsize=10, color='green')
        
        # 创建路径线和当前位置点
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=1.5, alpha=0.7, label='移动路径')
        self.current_point, = self.ax.plot([], [], 'go', markersize=12, label='当前位置')
        self.start_point, = self.ax.plot([], [], 'g^', markersize=10, label='起始位置')
        
        # 添加图例
        self.ax.legend(loc='upper right', fontsize=10)
        
        # 创建信息文本框
        info_text = "等待数据..."
        self.info_box = self.ax.text(
            0.02, 0.98, info_text,
            transform=self.ax.transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
            family='monospace'
        )
        
        # 紧凑布局
        plt.tight_layout()
    
    def update(self, frame):
        """
        更新图形（由动画函数调用）
        
        参数：
            frame: 帧编号（由matplotlib自动传入）
        """
        # 获取所有新数据
        data_list = self.receiver.get_all_data()
        
        for data in data_list:
            self.data_count += 1
            x, y = data['x'], data['y']
            
            # 记录第一个点作为起始位置
            if len(self.path_x) == 0:
                self.start_point.set_data([x], [y])
            
            # 添加路径点
            self.path_x.append(x)
            self.path_y.append(y)
            self.current_pos = (x, y)
            
            # 更新统计信息
            self.last_distance = data['distance_cm']
            self.last_azimuth = data['azimuth_deg']
        
        # 更新路径线
        if len(self.path_x) > 0:
            self.path_line.set_data(list(self.path_x), list(self.path_y))
        
        # 更新当前位置点
        if self.current_pos:
            self.current_point.set_data([self.current_pos[0]], [self.current_pos[1]])
        
        # 更新信息文本
        if self.current_pos:
            info_text = (
                f"━━━━ 实时数据 ━━━━\n"
                f"数据点数: {self.data_count}\n"
                f"━━━━━━━━━━━━━━━━\n"
                f"距离: {self.last_distance} cm\n"
                f"方位角: {self.last_azimuth}°\n"
                f"━━━━━━━━━━━━━━━━\n"
                f"X坐标: {self.current_pos[0]:.1f} cm\n"
                f"Y坐标: {self.current_pos[1]:.1f} cm\n"
                f"━━━━━━━━━━━━━━━━\n"
                f"路径点: {len(self.path_x)}/{MAX_PATH_POINTS}"
            )
        else:
            info_text = "等待数据...\n\n请确保：\n1. 串口已正确连接\n2. 基站已上电\n3. 信标在范围内"
        
        self.info_box.set_text(info_text)
        
        # 自动调整显示范围
        if len(self.path_x) > 0:
            x_range = max(abs(min(self.path_x)), abs(max(self.path_x)), 100) * 1.2
            y_range = max(abs(min(self.path_y)), abs(max(self.path_y)), 100) * 1.2
            max_range = max(x_range, y_range, DISPLAY_RANGE)
            self.ax.set_xlim(-max_range, max_range)
            self.ax.set_ylim(-max_range, max_range)
        
        return self.path_line, self.current_point, self.start_point, self.info_box
    
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
        print("\n✓ 可视化窗口已打开")
        print("  - 绿色圆点：当前位置")
        print("  - 蓝色线条：移动路径")
        print("  - 红色方块：基站位置")
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
        self.radius = 100
    
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
        print("✓ 开始生成模拟数据...")
        return True
    
    def _generate_loop(self):
        """生成模拟数据"""
        while self.running:
            # 生成螺旋线轨迹
            self.angle += 5
            self.radius += 0.5
            
            # 添加一些随机噪声
            noise_x = np.random.normal(0, 2)
            noise_y = np.random.normal(0, 2)
            
            x = self.radius * math.sin(math.radians(self.angle)) + noise_x
            y = self.radius * math.cos(math.radians(self.angle)) + noise_y
            
            distance = math.sqrt(x*x + y*y)
            azimuth = math.degrees(math.atan2(x, y))
            
            data = {
                'anchor_id': 0xAAA2,
                'tag_id': 0xAAA1,
                'distance_cm': int(distance),
                'azimuth_deg': int(azimuth),
                'elevation_deg': 0,
                'x': x,
                'y': y,
                'z': 0,
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
