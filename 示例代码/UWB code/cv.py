import serial
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque

plt.rcParams['font.sans-serif'] = ['SimHei']   # 全局使用黑体
plt.rcParams['axes.unicode_minus'] = False     # 解决负号显示问题

class UWBDataParser:
    """
    UWB定位数据解析器
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
            self.ser = serial.Serial(port, baudrate, timeout=1)
            print(f"串口 {port} 已成功打开，波特率: {baudrate}")
        except serial.SerialException as e:
            print(f"串口打开失败: {e}")
            raise

        self.angles = deque(maxlen=500)  # 存储角度
        self.distances = deque(maxlen=500)  # 存储距离
        self.elevations = deque(maxlen=500)  # 存储仰角
        self.packet_count = 0  # 定位数据包计数
        self.heartbeat_count = 0  # 心跳包计数
        self.last_heartbeat_time = None  # 最后心跳时间

        # 数据帧头
        self.FRAME_HEADER = b'\xFF\xFF\xFF\xFF'

        # 命令码
        self.CMD_LOCATION = 0x2001  # 定位数据
        self.CMD_HEARTBEAT = 0x2002  # 心跳包

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
        读取串口数据并解析
        支持定位数据包和心跳包
        """
        try:
            # 查找帧头
            while self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if byte == b'\xFF':
                    # 可能是帧头，继续读取3字节验证
                    next_bytes = self.ser.read(3)
                    if next_bytes == b'\xFF\xFF\xFF':
                        # 找到帧头，读取长度字段
                        length_bytes = self.ser.read(2)
                        if len(length_bytes) == 2:
                            packet_length = struct.unpack('>H', length_bytes)[0]

                            # 读取剩余数据
                            remaining = self.ser.read(packet_length - 6)  # 减去已读的帧头和长度

                            if len(remaining) == packet_length - 6:
                                # 完整数据包
                                full_packet = self.FRAME_HEADER + length_bytes + remaining

                                # 根据长度判断数据包类型
                                if packet_length == 16:
                                    # 心跳包
                                    heartbeat_info = self.parse_heartbeat(full_packet)
                                    if heartbeat_info:
                                        self.heartbeat_count += 1
                                        self.last_heartbeat_time = __import__('time').time()

                                        if self.heartbeat_count % 10 == 0:
                                            print(f"[心跳] 收到第 {self.heartbeat_count} 个心跳包 | "
                                                  f"基站ID: 0x{heartbeat_info['anchor_id']:08X} | "
                                                  f"流号: {heartbeat_info['sequence']}")

                                elif packet_length == 37:
                                    # 定位数据包
                                    distance, angle, elevation = self.parse_packet(full_packet)

                                    if distance is not None:
                                        self.distances.append(distance)
                                        self.angles.append(angle)
                                        self.elevations.append(elevation)
                                        self.packet_count += 1

                                        # 每10个包打印一次
                                        if self.packet_count % 10 == 0:
                                            print(f"[定位] 数据包 #{self.packet_count}: "
                                                  f"距离={distance}cm, 角度={angle}°, 仰角={elevation}°")

                                        return distance, angle, elevation
        except Exception as e:
            print(f"读取串口错误: {e}")

        return None, None, None

    def close(self):
        """关闭串口"""
        if self.ser.is_open:
            self.ser.close()
            print("串口已关闭")


class UWBVisualizer:
    """
    UWB定位数据可视化
    """

    def __init__(self, parser):
        self.parser = parser
        self.fig = plt.figure(figsize=(15, 5))

        # 子图1: 极坐标图 (俯视图)
        self.ax1 = self.fig.add_subplot(131, projection='polar')
        self.ax1.set_title('俯视图 (极坐标)', fontproperties='SimHei')
        self.ax1.set_theta_zero_location('N')  # 0度在上方
        self.ax1.set_theta_direction(-1)  # 顺时针
        self.scatter1 = self.ax1.scatter([], [], c='red', s=20, alpha=0.6)

        # 子图2: 角度-距离时序图
        self.ax2 = self.fig.add_subplot(132)
        self.ax2.set_title('角度-距离时序图', fontproperties='SimHei')
        self.ax2.set_xlabel('数据点序号')
        self.ax2.set_ylabel('距离 (cm)', color='b')
        self.ax2.tick_params(axis='y', labelcolor='b')
        self.ax2.grid(True, alpha=0.3)
        self.line_dist, = self.ax2.plot([], [], 'b-', linewidth=1.5, label='距离')

        # 共享x轴的第二个y轴 (角度)
        self.ax2_twin = self.ax2.twinx()
        self.ax2_twin.set_ylabel('角度 (度)', color='r')
        self.ax2_twin.tick_params(axis='y', labelcolor='r')
        self.line_angle, = self.ax2_twin.plot([], [], 'r-', linewidth=1.5, label='角度')

        # 子图3: XY笛卡尔坐标图
        self.ax3 = self.fig.add_subplot(133)
        self.ax3.set_title('XY平面图 (笛卡尔坐标)', fontproperties='SimHei')
        self.ax3.set_xlabel('X (cm)')
        self.ax3.set_ylabel('Y (cm)')
        self.ax3.grid(True, alpha=0.3)
        self.ax3.set_aspect('equal')
        self.scatter3 = self.ax3.scatter([], [], c='green', s=20, alpha=0.6)

        # 在原点绘制基站
        self.ax3.plot(0, 0, 'k^', markersize=15, label='基站')
        self.ax3.legend()

    def update(self, frame):
        """更新动画帧"""
        # 读取新数据
        self.parser.read_serial_data()

        if len(self.parser.angles) > 0:
            angles = np.array(list(self.parser.angles))
            distances = np.array(list(self.parser.distances))

            # 转换角度为弧度 (极坐标图)
            angles_rad = np.deg2rad(angles)

            # 更新极坐标图
            self.scatter1.set_offsets(np.c_[angles_rad, distances])
            max_dist = max(distances) * 1.2 if len(distances) > 0 else 100
            self.ax1.set_ylim(0, max_dist)

            # 更新时序图
            indices = list(range(len(distances)))
            self.line_dist.set_data(indices, distances)
            self.line_angle.set_data(indices, angles)
            self.ax2.set_xlim(0, max(len(distances), 50))
            self.ax2.set_ylim(0, max_dist)
            self.ax2_twin.set_ylim(min(angles) - 10, max(angles) + 10)

            # 更新XY笛卡尔坐标图
            # 极坐标转笛卡尔坐标: x = r*sin(θ), y = r*cos(θ)
            x = distances * np.sin(angles_rad)
            y = distances * np.cos(angles_rad)
            self.scatter3.set_offsets(np.c_[x, y])

            # 动态调整XY图范围
            if len(x) > 0:
                margin = max_dist * 0.1
                self.ax3.set_xlim(min(x) - margin, max(x) + margin)
                self.ax3.set_ylim(min(y) - margin, max(y) + margin)

        return self.scatter1, self.line_dist, self.line_angle, self.scatter3

    def start(self):
        """启动可视化（实时动画）"""
        self.anim = animation.FuncAnimation(
            self.fig,
            self.update,
            interval=50,
            cache_frame_data=False  # ← 关键：关闭帧缓存
        )
        plt.tight_layout()
        plt.show()

def main():
    """主函数 - 直接连接串口解析实际数据"""

    # ========== 配置区域 - 根据实际情况修改 ==========
    PORT = 'COM4'  # 串口号: Windows用'COM3', Linux用'/dev/ttyUSB0'
    BAUDRATE = 115200  # 波特率: 根据设备设置
    # ================================================

    print("=" * 80)
    print("                    UWB定位数据采集与可视化系统")
    print("=" * 80)
    print(f"串口配置: {PORT} @ {BAUDRATE} bps")
    print("支持数据包:")
    print("  • 定位数据包 (0x2001) - 37字节 - 距离、角度、仰角")
    print("  • 心跳包 (0x2002)     - 16字节 - 基站状态")
    print("=" * 80)
    print("\n正在连接串口...")

    try:
        # 创建解析器并连接串口
        parser = UWBDataParser(port=PORT, baudrate=BAUDRATE)

        print("✓ 串口连接成功!")
        print("\n启动实时可视化...")
        print("提示:")
        print("  - 数据将实时显示在图表中")
        print("  - 控制台会打印数据包统计")
        print("  - 按 Ctrl+C 或关闭图形窗口退出\n")
        print("-" * 80)

        # 创建可视化器并启动
        visualizer = UWBVisualizer(parser)
        visualizer.start()

    except serial.SerialException as e:
        print(f"\n✗ 串口错误: {e}")
        print("\n排查步骤:")
        print("  1. 检查串口号是否正确")
        print("     Windows: 在'设备管理器'查看端口")
        print("     Linux:   运行 'ls /dev/ttyUSB*' 或 'ls /dev/ttyACM*'")
        print("  2. 确认UWB设备已连接并上电")
        print("  3. 确认串口未被其他程序占用")
        print("  4. Linux用户需要串口权限:")
        print("     sudo usermod -a -G dialout $USER")
        print("     (需要重新登录生效)")

    except KeyboardInterrupt:
        print("\n\n程序已停止 (用户中断)")

    except Exception as e:
        print(f"\n✗ 发生错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        try:
            parser.close()
            print("\n串口已关闭")
        except:
            pass

if __name__ == "__main__":
    PORT = "COM6"        # 改成你设备管理器里的真实串口号
    BAUDRATE = 115200

    print("=" * 80)
    print("UWB 定位数据实时解析系统（仅实战模式）")
    print("=" * 80)

    try:
        parser = UWBDataParser(port=PORT, baudrate=BAUDRATE)
        visualizer = UWBVisualizer(parser)
        visualizer.start()

    except serial.SerialException as e:
        print("串口打开失败:", e)

    except KeyboardInterrupt:
        print("\n程序已终止")

    finally:
        try:
            parser.close()
        except:
            pass


