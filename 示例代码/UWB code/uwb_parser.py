import serial
import struct
from collections import deque
import time


class UWBDataParser:
    """
    UWB定位数据解析器

    数据更新频率: 50Hz
    支持两种数据包:
    - 定位数据包 (0x2001): 37字节
    - 心跳包 (0x2002): 16字节
    """

    def __init__(self, port='COM3', baudrate=115200):
        """
        初始化串口连接
        :param port: 串口号 (Windows: 'COM3', Linux: '/dev/ttyUSB0')
        :param baudrate: 波特率
        """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0)
            print(f"串口 {port} 已成功打开，波特率: {baudrate}")
        except serial.SerialException as e:
            print(f"串口打开失败: {e}")
            raise

        self.angles = deque(maxlen=50)  # 存储角度
        self.distances = deque(maxlen=50)  # 存储距离
        self.elevations = deque(maxlen=50)  # 存储仰角
        self.packet_count = 0  # 定位数据包计数
        self.heartbeat_count = 0  # 心跳包计数
        self.last_heartbeat_time = None  # 最后心跳时间
        self.last_location_time = None  # 最后定位数据时间
        self.offline_warning_shown = False  # 是否已显示离线警告

        # 数据帧头
        self.FRAME_HEADER = b'\xFF\xFF\xFF\xFF'

        # 命令码
        self.CMD_LOCATION = 0x2001  # 定位数据
        self.CMD_HEARTBEAT = 0x2002  # 心跳包

        # 帧同步缓冲区
        self.buffer = bytearray()



    def parse_heartbeat(self, data):
        """
        解析心跳包
        数据格式 (16字节):
        0-3:   帧头 (4字节) FF FF FF FF
        4-5:   长度 (2字节) 00 10 (16字节)
        6-7:   流号 (2字节)
        8-9:   命令 (2字节) 20 02
        10-11: 版本 (2字节) 01 01
        12-15: 基站ID (4字节)
        """
        try:
            # 验证帧头
            if data[0:4] != self.FRAME_HEADER:
                return None

            # 验证长度
            length = struct.unpack('>H', data[4:6])[0]
            if length != 16:
                return None

            # 验证命令码
            command = struct.unpack('>H', data[8:10])[0]
            if command != self.CMD_HEARTBEAT:
                return None

            # 提取信息
            sequence = struct.unpack('>H', data[6:8])[0]
            version = struct.unpack('>H', data[10:12])[0]
            anchor_id = struct.unpack('>I', data[12:16])[0]

            return {
                'sequence': sequence,
                'version': version,
                'anchor_id': anchor_id
            }

        except Exception as e:
            print(f"心跳包解析错误: {e}")
            return None

    def parse_packet(self, data):
        """
        解析定位数据包
        数据格式 (37字节):
        0-3:   帧头 (4字节) FF FF FF FF
        4-5:   长度 (2字节) 00 25 (37字节)
        6-7:   流号 (2字节)
        8-9:   命令 (2字节) 20 01
        10-11: 版本 (2字节)
        12-15: 基站ID (4字节)
        16-19: 信标ID (4字节)
        20-23: 距离 (4字节) - 单位cm
        24-25: 角度 (2字节) - 有符号，单位度
        26-27: 仰角 (2字节) - 有符号，单位度
        28-29: 状态 (2字节)
        30-31: 序号 (2字节)
        32-35: 预留 (4字节)
        36:    校验 (1字节)
        """
        try:
            # 验证帧头
            if data[0:4] != self.FRAME_HEADER:
                return None, None, None

            # 验证长度
            length = struct.unpack('>H', data[4:6])[0]
            if length != 37:
                return None, None, None

            # 验证命令码
            command = struct.unpack('>H', data[8:10])[0]
            if command != self.CMD_LOCATION:
                return None, None, None

            # 提取距离 (字节20-23, 4字节无符号整数)
            distance = struct.unpack('>I', data[20:24])[0]  # cm

            # 提取角度 (字节24-25, 2字节有符号整数)
            angle = struct.unpack('>h', data[24:26])[0]  # 度

            # 提取仰角 (字节26-27, 2字节有符号整数)
            elevation = struct.unpack('>h', data[26:28])[0]  # 度

            return distance, angle, elevation

        except Exception as e:
            print(f"定位数据解析错误: {e}")
            return None, None, None

    def read_serial_data(self):
        """
        优化的串口读取 - 一次性处理所有可用数据
        """
        try:      
            # 一次性读取所有可用数据
            if self.ser.in_waiting > 0:
                new_data = self.ser.read(self.ser.in_waiting)
                self.buffer.extend(new_data)
            
            # 在缓冲区中查找并处理所有完整数据包
            while len(self.buffer) >= 6:  # 至少需要帧头+长度
                # 查找帧头
                header_idx = self.buffer.find(self.FRAME_HEADER)
                
                if header_idx == -1:
                    # 没有找到帧头，清空缓冲区
                    self.buffer.clear()
                    break
                
                # 丢弃帧头之前的无效数据
                if header_idx > 0:
                    self.buffer = self.buffer[header_idx:]
                
                # 检查是否有足够的数据读取长度字段
                if len(self.buffer) < 6:
                    break
                
                # 读取包长度
                packet_length = struct.unpack('>H', self.buffer[4:6])[0]
                #packet_length = 37
                
                
                # 检查缓冲区是否有完整的包
                if len(self.buffer) < packet_length:
                    break  # 等待更多数据
                
                # 提取完整数据包
                full_packet = bytes(self.buffer[:packet_length])
                self.buffer = self.buffer[packet_length:]  # 移除已处理的包
                
                # 解析数据包
                if packet_length == 16:
                    # 心跳包
                    heartbeat_info = self.parse_heartbeat(full_packet)
                    if heartbeat_info:
                        self.heartbeat_count += 1
                        self.last_heartbeat_time = time.time()
                        
                        # 检测信标是否离线（有心跳但超过5秒无定位数据）
                        if self.last_location_time is not None:
                            time_since_location = time.time() - self.last_location_time
                            if time_since_location > 5 and not self.offline_warning_shown:
                                print(f"\n⚠️ 警告：已收到 {self.heartbeat_count} 个心跳包，"
                                      f"但 {time_since_location:.1f} 秒内无定位数据，信标可能已关闭！\n")
                                self.offline_warning_shown = True
                        
                        if self.heartbeat_count % 10 == 0:
                            print(f"[心跳] #{self.heartbeat_count} | "
                                  f"基站: 0x{heartbeat_info['anchor_id']:08X}")
                
                elif packet_length == 37:
                    # 定位数据包
                    distance, angle, elevation = self.parse_packet(full_packet)
                    
                    if distance is not None:
                        self.distances.append(distance)
                        self.angles.append(angle)
                        self.elevations.append(elevation)
                        self.packet_count += 1
                        self.last_location_time = time.time()
                        
                        # 信标恢复在线，重置警告标志
                        if self.offline_warning_shown:
                            print(f"\n✅ 信标已恢复在线！\n")
                            self.offline_warning_shown = False
                        
                        # 减少打印频率，降低延迟
                        # if self.packet_count % 20 == 0:
                        #     print(f"[定位] 数据包 #{self.packet_count}: "
                        #           f"距离 = {distance}cm, 角度 = {angle}°,仰角 = {elevation}°")
                        
                        return distance, angle, elevation
            
        except Exception as e:
            print(f"读取错误: {e}")
        
        return None, None, None

    def close(self):
        """关闭串口"""
        if self.ser.is_open:
            self.ser.close()
            print("串口已关闭")