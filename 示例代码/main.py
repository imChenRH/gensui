"""
UWB定位数据采集与可视化系统 - 主程序（低延迟优化版）
"""

import serial
from uwb_parser import UWBDataParser
from uwb_visualizer import UWBVisualizer


class AngleSmoother:
    """角度突变抑制器"""
    
    def __init__(self, threshold=10):
        """
        初始化角度平滑器
        :param threshold: 角度突变阈值（度），默认10°
        """
        self.threshold = threshold
        self.prev_angle = None  # 上一时刻角度 αk-1
        self.prev_distance = None  # 上一时刻距离 dk-1
    
    def process(self, angle, distance):
        """
        处理角度和距离，抑制角度突变
        
        规则：
        - αk - αk-1 > 10°:  αexport = αk-1, dexport = dk-1 + 2
        - αk - αk-1 < -10°: αexport = αk-1, dexport = dk-1 - 2
        - |αk - αk-1| ≤ 10°: αexport = αk, dexport = dk
        
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
        elif delta_angle < -self.threshold:
            # 角度负向突变超过阈值
            angle_export = self.prev_angle
            distance_export = self.prev_distance - 2
        else:
            # 角度变化在阈值范围内，正常输出
            angle_export = angle
            distance_export = distance
        
        # 更新上一时刻的值（使用原始UWB数据）
        self.prev_angle = angle
        self.prev_distance = distance
        
        return angle_export, distance_export


def main():
    """主函数"""
    
    # ========== 配置 ==========
    PORT = 'COM6'
    BAUDRATE = 115200
    ANGLE_THRESHOLD = 10  # 角度突变抑制阈值（度）
    # =========================

    print("=" * 70)
    print("          UWB定位数据采集系统 (低延迟优化版)")
    print("=" * 70)
    print(f"串口: {PORT} @ {BAUDRATE} bps")
    print("优化特性:")
    print("  • 非阻塞串口读取")
    print("  • 批量数据处理")
    print("  • 高速图形刷新 (50Hz)")
    print(f"  • 角度突变抑制 (阈值: {ANGLE_THRESHOLD}°)")
    print("=" * 70)

    try:
        # 创建解析器
        parser = UWBDataParser(port=PORT, baudrate=BAUDRATE)
        
        # 创建角度平滑器
        angle_smoother = AngleSmoother(threshold=ANGLE_THRESHOLD)
        
        print("\n✓ 串口连接成功")
        print("启动实时可视化...\n")
        print("提示: 按 Ctrl+C 或关闭窗口退出")
        print("-" * 70)

        # 创建并启动可视化（传入角度平滑器）
        visualizer = UWBVisualizer(parser, angle_smoother=angle_smoother)
        visualizer.start()

    except serial.SerialException as e:
        print(f"\n✗ 串口错误: {e}")
        print("\n快速解决:")
        print("  1. 重新插拔USB")
        print("  2. 关闭其他串口程序")
        print("  3. 检查设备管理器中的COM口号")

    except KeyboardInterrupt:
        print("\n\n程序已停止")

    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        try:
            parser.close()
        except:
            pass


if __name__ == "__main__":
    main()