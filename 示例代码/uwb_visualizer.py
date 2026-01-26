"""
UWB数据可视化模块 - 简化版
只显示距离和角度的实时曲线，配合单UWB使用
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False


class UWBVisualizer:
    """
    UWB定位数据可视化 - 简化版
    只显示距离和角度曲线
    """

    def __init__(self, parser, angle_smoother=None, history_window=200):
        """
        初始化可视化界面
        :param parser: UWBDataParser实例
        :param angle_smoother: AngleSmoother实例（角度突变抑制器，可选）
        :param history_window: 历史数据窗口大小
        """
        self.parser = parser
        self.angle_smoother = angle_smoother
        self.history_window = history_window
        
        # 存储平滑后的输出数据
        self.export_angles = deque(maxlen=500)
        self.export_distances = deque(maxlen=500)
        
        # 统计计数器
        self.total_count = 0
        self.print_interval = 20  # 每20个数据包打印一次
        
        self.fig = plt.figure(figsize=(16, 6))
        plt.ion()
        
        # ========== 距离曲线 ==========
        self.ax_dist = self.fig.add_subplot(1, 2, 1)
        self.ax_dist.set_title('距离变化', fontsize=14, fontweight='bold')
        self.ax_dist.set_xlabel('数据点序号', fontsize=11)
        self.ax_dist.set_ylabel('距离 (cm)', fontsize=11, fontweight='bold')
        self.ax_dist.grid(True, alpha=0.4, linestyle='--')
        self.line_dist, = self.ax_dist.plot([], [], 'b-', linewidth=2.5, label='距离')
        
        # 当前值显示
        self.text_dist = self.ax_dist.text(0.02, 0.98, '', transform=self.ax_dist.transAxes,
                                          fontsize=12, verticalalignment='top',
                                          bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        self.ax_dist.legend(loc='upper right', fontsize=10)
        
        # ========== 角度曲线 ==========
        self.ax_angle = self.fig.add_subplot(1, 2, 2)
        self.ax_angle.set_title('角度变化', fontsize=14, fontweight='bold')
        self.ax_angle.set_xlabel('数据点序号', fontsize=11)
        self.ax_angle.set_ylabel('角度 (度)', fontsize=11, fontweight='bold')
        self.ax_angle.grid(True, alpha=0.4, linestyle='--')
        self.line_angle, = self.ax_angle.plot([], [], 'r-', linewidth=2.5, label='角度')
        
        # 标记有效范围和参考线（单UWB视野边界 ±60°）
        self.ax_angle.axhline(y=-60, color='orange', linestyle='--', linewidth=1.5, alpha=0.6, label='视野边界 (±60°)')
        self.ax_angle.axhline(y=60, color='orange', linestyle='--', linewidth=1.5, alpha=0.6)
        self.ax_angle.axhline(y=0, color='green', linestyle='-', linewidth=2, alpha=0.7, label='正前方 (0°)')
        
        # 当前值显示
        self.text_angle = self.ax_angle.text(0.02, 0.98, '', transform=self.ax_angle.transAxes,
                                            fontsize=12, verticalalignment='top',
                                            bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
        self.ax_angle.legend(loc='upper right', fontsize=10)

    def update(self, frame):
        """更新动画帧"""
        # 批量处理数据
        update_count = 0
        max_updates = 10
        
        while update_count < max_updates:
            result = self.parser.read_serial_data()
            if result[0] is None:
                break
            
            distance, angle, elevation = result
            
            # 应用角度突变抑制
            if self.angle_smoother is not None:
                angle_export, distance_export = self.angle_smoother.process(angle, distance)
            else:
                angle_export, distance_export = angle, distance
            
            # 存储平滑后的数据
            self.export_angles.append(angle_export)
            self.export_distances.append(distance_export)
            
            # 统计计数并打印
            self.total_count += 1
            if self.total_count % self.print_interval == 0:
                print(f"[数据] #{self.total_count:5d} | "
                      f"原始: 距离={distance:6.1f}cm, 角度={angle:6.1f}° | "
                      f"输出: 距离={distance_export:6.1f}cm, 角度={angle_export:6.1f}°")
            
            update_count += 1

        if len(self.export_angles) == 0:
            return self.line_dist, self.line_angle
        
        # 获取所有历史数据
        angles = np.array(list(self.export_angles))
        distances = np.array(list(self.export_distances))
        total_points = len(distances)
        
        # X轴索引（从0开始累计）
        indices = np.arange(total_points)
        
        # ========== 距离曲线 ==========
        self.line_dist.set_data(indices, distances)
        
        # 滚动显示：只显示最近 history_window 个点
        start_idx = max(0, total_points - self.history_window)
        self.ax_dist.set_xlim(start_idx - 5, total_points + 5)
        
        # 固定Y轴范围：0 ~ 400 cm
        self.ax_dist.set_ylim(0, 400)
        
        if total_points > 0:
            current_dist = distances[-1]
            avg_dist = np.mean(distances[-min(10, len(distances)):])
            self.text_dist.set_text(f'当前: {current_dist:.1f} cm\n'
                                   f'平均: {avg_dist:.1f} cm\n'
                                   f'总数: {total_points}')
        
        # ========== 角度曲线 ==========
        self.line_angle.set_data(indices, angles)
        self.ax_angle.set_xlim(start_idx - 5, total_points + 5)
        
        # 固定Y轴范围：-70° ~ +70°（覆盖 ±60° 视野边界，留裕量）
        self.ax_angle.set_ylim(-70, 70)
        
        if total_points > 0:
            current_angle = angles[-1]
            avg_angle = np.mean(angles[-min(10, len(angles)):])
            self.text_angle.set_text(f'当前: {current_angle:.1f}°\n'
                                    f'平均: {avg_angle:.1f}°\n'
                                    f'总数: {total_points}')
        
        return self.line_dist, self.line_angle

    def start(self):
        """启动可视化"""
        self.anim = animation.FuncAnimation(
            self.fig,
            self.update,
            interval=20,
            blit=False,
            cache_frame_data=False
        )
        
        plt.tight_layout(pad=3.0)
        plt.show(block=True)