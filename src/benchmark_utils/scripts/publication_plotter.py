#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
publication_plotter.py - 出版级可视化模块

生成符合 IEEE / T-RO / IROS / ICRA 等顶级期刊会议标准的图表

标准规范：
=========
- 字体: Times New Roman (serif), 数学: STIX
- 字号: 标题 12pt, 轴标签 10pt, 刻度 9pt, 图例 9pt
- 线宽: 主线 1.5pt, 辅助线 1.0pt, 网格 0.5pt
- 颜色: 使用色盲友好配色方案
- 尺寸: 单栏 3.5in, 双栏 7.16in (IEEE标准)
- 格式: PDF (矢量) + PNG (300dpi 栅格)
- 边距: tight, 无多余空白

图表类型：
=========
1. 3D轨迹对比图 (fig_trajectory_3d)
2. XY平面投影图 (fig_trajectory_xy)
3. 位置误差时序图 (fig_error_timeseries)
4. 误差分布直方图 (fig_error_distribution)
5. 速度/加速度分析图 (fig_dynamics)
6. 多维雷达图 (fig_radar)
7. 控制器对比箱线图 (fig_comparison_boxplot)
8. 指标汇总表 (table_metrics)
"""

import os
import sys
import numpy as np
import json
from datetime import datetime

# Matplotlib 配置
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.ticker as ticker

# 尝试导入 seaborn
try:
    import seaborn as sns
    SEABORN_AVAILABLE = True
except ImportError:
    SEABORN_AVAILABLE = False


class PublicationStyle:
    """IEEE 期刊出版样式配置"""

    # 页面尺寸 (inches)
    SINGLE_COLUMN_WIDTH = 3.5      # IEEE 单栏宽度
    DOUBLE_COLUMN_WIDTH = 7.16     # IEEE 双栏宽度
    GOLDEN_RATIO = 1.618           # 黄金比例

    # 字体配置
    FONT_FAMILY = 'serif'
    FONT_SERIF = ['Times New Roman', 'DejaVu Serif', 'serif']
    MATH_FONTSET = 'stix'

    # 字号 (points)
    FONT_SIZE_TITLE = 12
    FONT_SIZE_LABEL = 10
    FONT_SIZE_TICK = 9
    FONT_SIZE_LEGEND = 9
    FONT_SIZE_ANNOTATION = 8

    # 线宽 (points)
    LINE_WIDTH_MAJOR = 1.5
    LINE_WIDTH_MINOR = 1.0
    LINE_WIDTH_GRID = 0.5
    LINE_WIDTH_AXIS = 0.8

    # 色盲友好配色 (IBM Design / Color Universal Design)
    COLORS = {
        'blue': '#0072B2',
        'orange': '#E69F00',
        'green': '#009E73',
        'red': '#D55E00',
        'purple': '#CC79A7',
        'cyan': '#56B4E9',
        'yellow': '#F0E442',
        'black': '#000000',
        'gray': '#999999',
    }

    # 配色方案
    COLOR_REFERENCE = '#0072B2'    # 参考轨迹 (蓝)
    COLOR_ACTUAL = '#D55E00'       # 实际轨迹 (橙红)
    COLOR_ERROR = '#CC79A7'        # 误差 (紫粉)
    COLOR_FILL = '#56B4E9'         # 填充 (浅蓝)

    # 标记样式
    MARKER_START = 'o'
    MARKER_END = 's'
    MARKER_SIZE = 6

    @classmethod
    def apply(cls):
        """应用出版样式到 matplotlib"""
        plt.rcParams.update({
            # 字体
            'font.family': cls.FONT_FAMILY,
            'font.serif': cls.FONT_SERIF,
            'mathtext.fontset': cls.MATH_FONTSET,

            # 字号
            'font.size': cls.FONT_SIZE_TICK,
            'axes.titlesize': cls.FONT_SIZE_TITLE,
            'axes.labelsize': cls.FONT_SIZE_LABEL,
            'xtick.labelsize': cls.FONT_SIZE_TICK,
            'ytick.labelsize': cls.FONT_SIZE_TICK,
            'legend.fontsize': cls.FONT_SIZE_LEGEND,

            # 线宽
            'lines.linewidth': cls.LINE_WIDTH_MAJOR,
            'axes.linewidth': cls.LINE_WIDTH_AXIS,
            'grid.linewidth': cls.LINE_WIDTH_GRID,
            'patch.linewidth': cls.LINE_WIDTH_MINOR,

            # 网格
            'axes.grid': True,
            'grid.alpha': 0.3,
            'grid.linestyle': '--',

            # 图例
            'legend.frameon': True,
            'legend.framealpha': 0.9,
            'legend.edgecolor': '0.8',
            'legend.fancybox': False,

            # 刻度
            'xtick.direction': 'in',
            'ytick.direction': 'in',
            'xtick.major.size': 4,
            'ytick.major.size': 4,
            'xtick.minor.size': 2,
            'ytick.minor.size': 2,

            # 保存
            'savefig.dpi': 300,
            'savefig.bbox': 'tight',
            'savefig.pad_inches': 0.02,

            # 其他
            'axes.unicode_minus': False,
            'figure.autolayout': False,
        })


class PublicationPlotter:
    """出版级图表生成器"""

    def __init__(self, output_dir='./publication_plots'):
        """
        初始化

        Args:
            output_dir: 输出目录
        """
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        # 应用出版样式
        PublicationStyle.apply()

        # 样式快捷引用
        self.style = PublicationStyle

    def _save_figure(self, fig, name, formats=['pdf', 'png']):
        """保存图表（多格式）"""
        for fmt in formats:
            path = os.path.join(self.output_dir, f"{name}.{fmt}")
            fig.savefig(path, format=fmt, dpi=300, bbox_inches='tight',
                       pad_inches=0.02, facecolor='white', edgecolor='none')
            print(f"  已保存: {path}")

    def plot_trajectory_3d(self, data, metrics, name='fig_trajectory_3d'):
        """
        3D轨迹对比图

        IEEE双栏宽度，展示参考轨迹和实际轨迹的三维对比
        """
        fig = plt.figure(figsize=(self.style.DOUBLE_COLUMN_WIDTH,
                                  self.style.DOUBLE_COLUMN_WIDTH * 0.7))
        ax = fig.add_subplot(111, projection='3d')

        target = data['target_position']
        actual = data['actual_position']

        # 参考轨迹
        ax.plot(target[:, 0], target[:, 1], target[:, 2],
                color=self.style.COLOR_REFERENCE,
                linewidth=self.style.LINE_WIDTH_MAJOR,
                linestyle='--', label='Reference', zorder=5)

        # 实际轨迹（根据误差着色）
        pos_errors = np.linalg.norm(target - actual, axis=1)
        scatter = ax.scatter(actual[:, 0], actual[:, 1], actual[:, 2],
                           c=pos_errors, cmap='RdYlGn_r',
                           s=4, alpha=0.6, label='Actual', zorder=3)

        # 颜色条
        cbar = plt.colorbar(scatter, ax=ax, shrink=0.6, pad=0.1)
        cbar.set_label('Position Error (m)', fontsize=self.style.FONT_SIZE_LABEL)
        cbar.ax.tick_params(labelsize=self.style.FONT_SIZE_TICK)

        # 起点/终点标记
        ax.scatter(*target[0], c='green', s=80, marker='o',
                  edgecolors='black', linewidth=1, label='Start', zorder=10)
        ax.scatter(*target[-1], c='red', s=80, marker='s',
                  edgecolors='black', linewidth=1, label='End', zorder=10)

        # 坐标轴设置
        ax.set_xlabel('X (m)', fontsize=self.style.FONT_SIZE_LABEL, labelpad=8)
        ax.set_ylabel('Y (m)', fontsize=self.style.FONT_SIZE_LABEL, labelpad=8)
        ax.set_zlabel('Z (m)', fontsize=self.style.FONT_SIZE_LABEL, labelpad=8)

        # 统一比例尺
        max_range = max(
            target[:, 0].ptp(), target[:, 1].ptp(), target[:, 2].ptp()
        ) * 0.6
        mid = target.mean(axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

        try:
            ax.set_box_aspect((1, 1, 1))
        except:
            pass

        # 视角
        ax.view_init(elev=25, azim=-45)
        ax.grid(True, alpha=0.2)

        # 图例
        ax.legend(loc='upper left', fontsize=self.style.FONT_SIZE_LEGEND,
                 framealpha=0.9)

        # 标题（含性能指标）
        title = f'RMS Error: {metrics.pos_rms_error:.3f} m | Max Error: {metrics.pos_max_error:.3f} m'
        ax.set_title(title, fontsize=self.style.FONT_SIZE_TITLE, pad=15)

        plt.tight_layout()
        self._save_figure(fig, name)
        plt.close()

    def plot_trajectory_xy(self, data, metrics, name='fig_trajectory_xy'):
        """
        XY平面投影图

        IEEE单栏宽度，俯视图对比
        """
        fig, ax = plt.subplots(figsize=(self.style.SINGLE_COLUMN_WIDTH,
                                        self.style.SINGLE_COLUMN_WIDTH))

        target = data['target_position']
        actual = data['actual_position']

        # 时间缩放软启动后位置始终在曲线上，从 t=0 开始绘制完整轨迹
        # 参考轨迹
        ax.plot(target[:, 0], target[:, 1],
                color=self.style.COLOR_REFERENCE,
                linewidth=self.style.LINE_WIDTH_MAJOR,
                linestyle='--', label='Reference', zorder=5)

        # 实际轨迹
        ax.plot(actual[:, 0], actual[:, 1],
                color=self.style.COLOR_ACTUAL,
                linewidth=self.style.LINE_WIDTH_MINOR,
                alpha=0.8, label='Actual', zorder=3)

        # 起点/终点
        ax.scatter(target[0, 0], target[0, 1], c='#009E73', s=60, marker='o',
                  edgecolors='black', linewidth=1, label='Start', zorder=10)
        ax.scatter(target[-1, 0], target[-1, 1], c='red', s=60, marker='s',
                  edgecolors='black', linewidth=1, label='End', zorder=10)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_aspect('equal')
        ax.margins(0.18)   # 适当缩小轨迹，右上角留空给图例
        ax.legend(loc='upper right', fontsize=self.style.FONT_SIZE_LEGEND,
                 framealpha=0.92, edgecolor='0.8', fancybox=False)
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        self._save_figure(fig, name)
        plt.close()

    def plot_error_timeseries(self, data, metrics, name='fig_error_timeseries'):
        """
        误差时序图

        IEEE双栏宽度，展示位置误差随时间的变化
        """
        fig, axes = plt.subplots(2, 1, figsize=(self.style.DOUBLE_COLUMN_WIDTH,
                                                 self.style.DOUBLE_COLUMN_WIDTH * 0.5),
                                 sharex=True)

        time = data['time']
        pos_err = data['target_position'] - data['actual_position']
        pos_err_norm = np.linalg.norm(pos_err, axis=1)

        # 上图：各轴误差
        ax1 = axes[0]
        ax1.plot(time, pos_err[:, 0], color=self.style.COLORS['red'],
                linewidth=self.style.LINE_WIDTH_MINOR, label='$e_x$', alpha=0.9)
        ax1.plot(time, pos_err[:, 1], color=self.style.COLORS['green'],
                linewidth=self.style.LINE_WIDTH_MINOR, label='$e_y$', alpha=0.9)
        ax1.plot(time, pos_err[:, 2], color=self.style.COLORS['blue'],
                linewidth=self.style.LINE_WIDTH_MINOR, label='$e_z$', alpha=0.9)
        ax1.axhline(0, color='black', linewidth=0.5, linestyle='-')
        ax1.set_ylabel('Position Error (m)')
        ax1.legend(loc='upper right', ncol=3, fontsize=self.style.FONT_SIZE_LEGEND)
        ax1.grid(True, alpha=0.3)

        # 下图：误差范数
        ax2 = axes[1]
        ax2.fill_between(time, 0, pos_err_norm,
                        color=self.style.COLOR_FILL, alpha=0.3)
        ax2.plot(time, pos_err_norm, color=self.style.COLOR_ERROR,
                linewidth=self.style.LINE_WIDTH_MAJOR, label='$\|e\|$')

        # 添加统计线
        ax2.axhline(metrics.pos_rms_error, color=self.style.COLORS['orange'],
                   linewidth=1, linestyle='--',
                   label=f'RMS = {metrics.pos_rms_error:.3f} m')
        ax2.axhline(metrics.pos_p95_error, color=self.style.COLORS['purple'],
                   linewidth=1, linestyle=':',
                   label=f'P95 = {metrics.pos_p95_error:.3f} m')

        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error Norm (m)')
        ax2.legend(loc='upper right', fontsize=self.style.FONT_SIZE_LEGEND)
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim(bottom=0)

        plt.tight_layout()
        self._save_figure(fig, name)
        plt.close()

    def plot_error_distribution(self, data, metrics, name='fig_error_distribution'):
        """
        误差分布直方图

        IEEE单栏宽度，展示误差的统计分布
        """
        fig, ax = plt.subplots(figsize=(self.style.SINGLE_COLUMN_WIDTH,
                                        self.style.SINGLE_COLUMN_WIDTH * 0.75))

        pos_errors = np.linalg.norm(
            data['target_position'] - data['actual_position'], axis=1
        )

        # 确保是 numpy array
        pos_errors = np.asarray(pos_errors).flatten()

        # 直方图
        n, bins, patches = ax.hist(pos_errors, bins=50, density=True,
                                   color=self.style.COLOR_FILL, alpha=0.7,
                                   edgecolor='white', linewidth=0.5)

        # KDE 曲线（使用 scipy 替代 seaborn 避免兼容性问题）
        try:
            from scipy import stats
            kde = stats.gaussian_kde(pos_errors)
            x_range = np.linspace(pos_errors.min(), pos_errors.max(), 200)
            ax.plot(x_range, kde(x_range), color=self.style.COLOR_ERROR,
                   linewidth=self.style.LINE_WIDTH_MAJOR, label='KDE')
        except ImportError:
            pass  # scipy 不可用则跳过 KDE

        # 统计标记
        ax.axvline(metrics.pos_rms_error, color=self.style.COLORS['orange'],
                  linewidth=1.5, linestyle='--',
                  label=f'RMS = {metrics.pos_rms_error:.3f} m')
        ax.axvline(metrics.pos_mean_error, color=self.style.COLORS['blue'],
                  linewidth=1.5, linestyle=':',
                  label=f'Mean = {metrics.pos_mean_error:.3f} m')
        ax.axvline(metrics.pos_p95_error, color=self.style.COLORS['purple'],
                  linewidth=1.5, linestyle='-.',
                  label=f'P95 = {metrics.pos_p95_error:.3f} m')

        ax.set_xlabel('Position Error (m)')
        ax.set_ylabel('Probability Density')
        ax.legend(loc='upper right', fontsize=self.style.FONT_SIZE_LEGEND)
        ax.grid(True, alpha=0.3, axis='y')
        ax.set_xlim(left=0)

        plt.tight_layout()
        self._save_figure(fig, name)
        plt.close()

    def plot_dynamics(self, data, metrics, name='fig_dynamics'):
        """
        速度/加速度分析图

        IEEE双栏宽度，展示轨迹动态特性
        """
        fig, axes = plt.subplots(3, 1, figsize=(self.style.DOUBLE_COLUMN_WIDTH,
                                                 self.style.DOUBLE_COLUMN_WIDTH * 0.65),
                                 sharex=True)

        time = data['time']

        # 速度
        ax1 = axes[0]
        target_vel = np.linalg.norm(data['target_velocity'], axis=1)
        actual_vel = np.linalg.norm(data['actual_velocity'], axis=1)
        ax1.plot(time, target_vel, color=self.style.COLOR_REFERENCE,
                linewidth=self.style.LINE_WIDTH_MAJOR, linestyle='--', label='Reference')
        ax1.plot(time, actual_vel, color=self.style.COLOR_ACTUAL,
                linewidth=self.style.LINE_WIDTH_MINOR, alpha=0.8, label='Actual')
        ax1.set_ylabel('Velocity (m/s)')
        ax1.legend(loc='upper right', ncol=2, fontsize=self.style.FONT_SIZE_LEGEND)
        ax1.grid(True, alpha=0.3)
        ax1.text(0.02, 0.95, f'$v_{{max}}$ = {metrics.traj_max_velocity:.2f} m/s',
                transform=ax1.transAxes, fontsize=self.style.FONT_SIZE_ANNOTATION,
                verticalalignment='top')

        # 加速度
        ax2 = axes[1]
        target_acc = np.linalg.norm(data['target_acceleration'], axis=1)
        ax2.plot(time, target_acc, color=self.style.COLORS['green'],
                linewidth=self.style.LINE_WIDTH_MAJOR, label='Target Acceleration')
        ax2.set_ylabel('Acceleration (m/s$^2$)')
        ax2.legend(loc='upper right', fontsize=self.style.FONT_SIZE_LEGEND)
        ax2.grid(True, alpha=0.3)
        ax2.text(0.02, 0.95, f'$a_{{max}}$ = {metrics.traj_max_acceleration:.2f} m/s$^2$',
                transform=ax2.transAxes, fontsize=self.style.FONT_SIZE_ANNOTATION,
                verticalalignment='top')

        # Jerk
        ax3 = axes[2]
        target_jerk = np.linalg.norm(data['target_jerk'], axis=1)
        ax3.plot(time, target_jerk, color=self.style.COLORS['purple'],
                linewidth=self.style.LINE_WIDTH_MAJOR, label='Target Jerk')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Jerk (m/s$^3$)')
        ax3.legend(loc='upper right', fontsize=self.style.FONT_SIZE_LEGEND)
        ax3.grid(True, alpha=0.3)
        ax3.text(0.02, 0.95, f'$j_{{max}}$ = {metrics.traj_max_jerk:.2f} m/s$^3$',
                transform=ax3.transAxes, fontsize=self.style.FONT_SIZE_ANNOTATION,
                verticalalignment='top')

        plt.tight_layout()
        self._save_figure(fig, name)
        plt.close()

    def plot_radar(self, metrics, name='fig_radar'):
        """
        多维雷达图

        IEEE单栏宽度，展示多维性能评估
        """
        fig, ax = plt.subplots(figsize=(self.style.SINGLE_COLUMN_WIDTH,
                                        self.style.SINGLE_COLUMN_WIDTH),
                               subplot_kw=dict(polar=True))

        # 评估维度
        categories = [
            'Track.\nAccuracy',
            'Velocity\nTracking',
            'Control\nSmooth.',
            'Response\nSpeed',
            'Energy\nEff.'
        ]

        def soft_inverse_score(val, best, worst, floor=28.0, gamma=0.60):
            if val <= best:
                return 100
            if val >= worst:
                return floor
            ratio = (val - best) / (worst - best)
            return floor + (100 - floor) * ((1 - ratio) ** gamma)

        def soft_log_inverse_score(val, best, worst, floor=24.0, gamma=0.85):
            if val <= best:
                return 100
            val = min(max(val, best), worst)
            ratio = (np.log(val) - np.log(best)) / (np.log(worst) - np.log(best))
            return floor + (100 - floor) * ((1 - ratio) ** gamma)

        def soft_energy_score(val, target=0.8, floor=35.0):
            if val <= 0:
                return floor
            ratio = min(val / target, 1.0)
            return floor + (100 - floor) * np.sqrt(ratio)

        values = [
            soft_inverse_score(metrics.pos_rms_error, 0.01, 0.25),
            soft_inverse_score(metrics.vel_rms_error, 0.02, 0.50),
            soft_log_inverse_score(metrics.actual_jerk_rms, 30.0, 5000.0),
            soft_inverse_score(abs(metrics.phase_delay), 0.005, 0.30),
            soft_energy_score(metrics.energy_efficiency_index),
        ]

        # 闭合多边形
        angles = np.linspace(0, 2*np.pi, len(categories), endpoint=False).tolist()
        values_plot = values + values[:1]
        angles += angles[:1]

        # 绘制
        ax.fill(angles, values_plot, color=self.style.COLOR_FILL, alpha=0.3)
        ax.plot(angles, values_plot, 'o-', color=self.style.COLOR_REFERENCE,
               linewidth=self.style.LINE_WIDTH_MAJOR, markersize=6)

        # 设置
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(categories, fontsize=self.style.FONT_SIZE_TICK)
        ax.tick_params(axis='x', pad=10)
        ax.set_ylim(0, 100)
        ax.set_yticks([20, 40, 60, 80, 100])
        ax.set_yticklabels(['20', '40', '60', '80', '100'],
                          fontsize=self.style.FONT_SIZE_TICK - 1)
        ax.set_rlabel_position(90)
        ax.grid(True, alpha=0.3)

        # 标题
        ax.set_title(f'Overall Score: {metrics.overall_score:.1f}/100 ({metrics.grade})',
                    fontsize=self.style.FONT_SIZE_TITLE, pad=18)

        plt.tight_layout()
        self._save_figure(fig, name)
        plt.close()

    def plot_comparison_boxplot(self, metrics_list, labels, name='fig_comparison'):
        """
        控制器对比箱线图

        IEEE双栏宽度，对比多个控制器的性能

        Args:
            metrics_list: PerformanceMetrics 对象列表
            labels: 控制器标签列表
        """
        fig, axes = plt.subplots(1, 3, figsize=(self.style.DOUBLE_COLUMN_WIDTH,
                                                 self.style.DOUBLE_COLUMN_WIDTH * 0.35))

        # 提取数据
        pos_errors = [[m.pos_rms_error] for m in metrics_list]
        vel_errors = [[m.vel_rms_error] for m in metrics_list]
        delays = [[m.phase_delay] for m in metrics_list]

        # 位置误差
        ax1 = axes[0]
        bp1 = ax1.boxplot(pos_errors, labels=labels, patch_artist=True)
        for patch in bp1['boxes']:
            patch.set_facecolor(self.style.COLOR_FILL)
            patch.set_alpha(0.7)
        ax1.set_ylabel('Position RMS Error (m)')
        ax1.set_title('Tracking Accuracy')
        ax1.grid(True, alpha=0.3, axis='y')

        # 速度误差
        ax2 = axes[1]
        bp2 = ax2.boxplot(vel_errors, labels=labels, patch_artist=True)
        for patch in bp2['boxes']:
            patch.set_facecolor(self.style.COLOR_FILL)
            patch.set_alpha(0.7)
        ax2.set_ylabel('Velocity RMS Error (m/s)')
        ax2.set_title('Velocity Tracking')
        ax2.grid(True, alpha=0.3, axis='y')

        # 相位延迟
        ax3 = axes[2]
        bp3 = ax3.boxplot(delays, labels=labels, patch_artist=True)
        for patch in bp3['boxes']:
            patch.set_facecolor(self.style.COLOR_FILL)
            patch.set_alpha(0.7)
        ax3.set_ylabel('Phase Delay (s)')
        ax3.set_title('Response Speed')
        ax3.grid(True, alpha=0.3, axis='y')

        plt.tight_layout()
        self._save_figure(fig, name)
        plt.close()

    def plot_comprehensive_figure(self, data, metrics, name='fig_comprehensive'):
        """
        综合分析图（6子图）

        IEEE双栏宽度，适合论文主图
        """
        fig = plt.figure(figsize=(self.style.DOUBLE_COLUMN_WIDTH * 1.1,
                                  self.style.DOUBLE_COLUMN_WIDTH * 0.90))

        gs = GridSpec(3, 2, figure=fig, hspace=0.50, wspace=0.38,
                      left=0.09, right=0.97, top=0.91, bottom=0.08)

        S = self.style   # shorthand
        LEG_KW = dict(loc='upper right', fontsize=7, framealpha=0.9,
                      edgecolor='0.8', fancybox=False)
        TITLE_KW = dict(fontsize=9, fontweight='bold')

        time = data['time']
        target = data['target_position']
        actual = data['actual_position']
        pos_err = target - actual
        pos_err_norm = np.linalg.norm(pos_err, axis=1)

        # ── (a) XY 轨迹 ────────────────────────────────────────────────────────
        ax1 = fig.add_subplot(gs[0, 0])
        # 时间缩放软启动后位置始终在曲线上，无需跳过任何数据
        ax1.plot(target[:, 0], target[:, 1], '--',
                 color=S.COLOR_REFERENCE, linewidth=1.5, label='Ref.', zorder=5)
        ax1.plot(actual[:, 0], actual[:, 1], '-',
                 color=S.COLOR_ACTUAL, linewidth=1.0, alpha=0.85, label='Act.', zorder=4)
        ax1.scatter(target[0, 0], target[0, 1],
                    c='#009E73', s=48, marker='o', edgecolors='k', linewidth=0.8,
                    label='Start', zorder=10)
        ax1.set_xlabel('X (m)', fontsize=S.FONT_SIZE_LABEL)
        ax1.set_ylabel('Y (m)', fontsize=S.FONT_SIZE_LABEL)
        ax1.set_aspect('equal')
        # 加边距使轨迹缩小，右上角留空间绘制图例
        ax1.margins(0.18)
        ax1.legend(loc='upper right', fontsize=7, framealpha=0.92,
                   edgecolor='0.8', fancybox=False)
        ax1.set_title('(a) XY Trajectory', **TITLE_KW)
        ax1.grid(True, alpha=0.25, linestyle='--')

        # ── (b) XZ 轨迹 ────────────────────────────────────────────────────────
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(target[:, 0], target[:, 2], '--',
                 color=S.COLOR_REFERENCE, linewidth=1.5, label='Ref.', zorder=5)
        ax2.plot(actual[:, 0], actual[:, 2], '-',
                 color=S.COLOR_ACTUAL, linewidth=1.0, alpha=0.85, label='Act.', zorder=4)
        ax2.set_xlabel('X (m)', fontsize=S.FONT_SIZE_LABEL)
        ax2.set_ylabel('Z (m)', fontsize=S.FONT_SIZE_LABEL)
        ax2.margins(0.18)
        ax2.legend(loc='upper right', fontsize=7, framealpha=0.92,
                   edgecolor='0.8', fancybox=False)
        ax2.set_title('(b) XZ Trajectory', **TITLE_KW)
        ax2.grid(True, alpha=0.25, linestyle='--')

        # ── (c) 位置误差时序 ────────────────────────────────────────────────────
        ax3 = fig.add_subplot(gs[1, 0])
        ax3.plot(time, pos_err[:, 0], color=S.COLORS['red'],
                 linewidth=0.9, label='$e_x$', alpha=0.9)
        ax3.plot(time, pos_err[:, 1], color=S.COLORS['green'],
                 linewidth=0.9, label='$e_y$', alpha=0.9)
        ax3.plot(time, pos_err[:, 2], color=S.COLORS['blue'],
                 linewidth=0.9, label='$e_z$', alpha=0.9)
        ax3.axhline(0, color='black', linewidth=0.4, linestyle='-')
        ax3.set_xlabel('Time (s)', fontsize=S.FONT_SIZE_LABEL)
        ax3.set_ylabel('Error (m)', fontsize=S.FONT_SIZE_LABEL)
        ax3.legend(ncol=3, **LEG_KW)
        ax3.set_title('(c) Position Error', **TITLE_KW)
        ax3.grid(True, alpha=0.25, linestyle='--')

        # ── (d) 误差范数 ─────────────────────────────────────────────────────────
        ax4 = fig.add_subplot(gs[1, 1])
        ax4.fill_between(time, 0, pos_err_norm,
                         color=S.COLOR_FILL, alpha=0.35)
        ax4.plot(time, pos_err_norm, color=S.COLOR_ERROR, linewidth=1.0,
                 label='$\|e\|$')
        ax4.axhline(metrics.pos_rms_error, color=S.COLORS['orange'],
                    linewidth=1.2, linestyle='--',
                    label=f'RMS={metrics.pos_rms_error:.3f} m')
        ax4.axhline(metrics.pos_p95_error, color=S.COLORS['purple'],
                    linewidth=1.0, linestyle=':',
                    label=f'P95={metrics.pos_p95_error:.3f} m')
        ax4.set_xlabel('Time (s)', fontsize=S.FONT_SIZE_LABEL)
        ax4.set_ylabel('$\|e\|$ (m)', fontsize=S.FONT_SIZE_LABEL)
        ax4.legend(**LEG_KW)
        ax4.set_title('(d) Error Magnitude', **TITLE_KW)
        ax4.grid(True, alpha=0.25, linestyle='--')
        ax4.set_ylim(bottom=0)

        # ── (e) 速度跟踪 ─────────────────────────────────────────────────────────
        ax5 = fig.add_subplot(gs[2, 0])
        target_vel_plot = np.linalg.norm(data['target_velocity'], axis=1)
        actual_vel_plot = np.linalg.norm(data['actual_velocity'], axis=1)
        ax5.plot(time, target_vel_plot, '--', color=S.COLOR_REFERENCE,
                 linewidth=1.5, label='Ref.', zorder=5)
        ax5.plot(time, actual_vel_plot, '-', color=S.COLOR_ACTUAL,
                 linewidth=1.0, alpha=0.85, label='Act.', zorder=4)
        ax5.set_xlabel('Time (s)', fontsize=S.FONT_SIZE_LABEL)
        ax5.set_ylabel('Velocity (m/s)', fontsize=S.FONT_SIZE_LABEL)
        ax5.legend(**LEG_KW)
        ax5.set_title(f'(e) Velocity  |  $v_{{max}}$={metrics.traj_max_velocity:.2f} m/s', **TITLE_KW)
        ax5.grid(True, alpha=0.25, linestyle='--')

        # ── (f) 误差分布 ─────────────────────────────────────────────────────────
        ax6 = fig.add_subplot(gs[2, 1])
        pos_err_flat = np.asarray(pos_err_norm).flatten()
        ax6.hist(pos_err_flat, bins=40, density=True,
                 color=S.COLOR_FILL, alpha=0.75, edgecolor='white', linewidth=0.4)
        ax6.axvline(metrics.pos_rms_error, color=S.COLORS['orange'],
                    linewidth=1.5, linestyle='--',
                    label=f'RMS={metrics.pos_rms_error:.3f} m')
        ax6.axvline(metrics.pos_p95_error, color=S.COLORS['purple'],
                    linewidth=1.5, linestyle=':',
                    label=f'P95={metrics.pos_p95_error:.3f} m')
        ax6.set_xlabel('Error (m)', fontsize=S.FONT_SIZE_LABEL)
        ax6.set_ylabel('Density', fontsize=S.FONT_SIZE_LABEL)
        ax6.legend(**LEG_KW)
        ax6.set_title('(f) Error Distribution', **TITLE_KW)
        ax6.grid(True, alpha=0.25, linestyle='--', axis='y')
        ax6.set_xlim(left=0)

        # ── suptitle ──────────────────────────────────────────────────────────
        fig.suptitle(
            f'Controller Performance  |  RMS: {metrics.pos_rms_error:.4f} m'
            f'  |  Score: {metrics.overall_score:.1f}/100 ({metrics.grade})',
            fontsize=11, fontweight='bold', y=0.97
        )

        self._save_figure(fig, name)
        plt.close()



    def generate_metrics_table(self, metrics, name='table_metrics'):
        """
        生成指标汇总表（LaTeX格式）

        Args:
            metrics: PerformanceMetrics 对象
            name: 输出文件名
        """
        latex_content = r"""\begin{table}[htbp]
\centering
\caption{Controller Performance Metrics}
\label{tab:metrics}
\begin{tabular}{lcc}
\hline
\textbf{Metric} & \textbf{Value} & \textbf{Unit} \\
\hline
\multicolumn{3}{l}{\textit{Tracking Accuracy}} \\
Position RMS Error & %.4f & m \\
Position Max Error & %.4f & m \\
Position P95 Error & %.4f & m \\
Cross-Track RMS & %.4f & m \\
\hline
\multicolumn{3}{l}{\textit{Velocity Tracking}} \\
Velocity RMS Error & %.4f & m/s \\
Velocity Max Error & %.4f & m/s \\
\hline
\multicolumn{3}{l}{\textit{Yaw Control}} \\
Yaw RMS Error & %.4f & rad \\
Yaw Delay & %.4f & s \\
\hline
\multicolumn{3}{l}{\textit{Time Response}} \\
Phase Delay & %.4f & s \\
\hline
\multicolumn{3}{l}{\textit{Trajectory Characteristics}} \\
Max Velocity & %.3f & m/s \\
Max Acceleration & %.3f & m/s$^2$ \\
Max Jerk & %.3f & m/s$^3$ \\
Difficulty Level & %s & - \\
\hline
\multicolumn{3}{l}{\textit{Control Smoothness}} \\
Actual Jerk RMS & %.4f & m/s$^3$ \\
Smoothness Index & %.4f & - \\
\hline
\multicolumn{3}{l}{\textit{Energy Efficiency}} \\
Energy Efficiency Index & %.4f & - \\
\hline
\textbf{Overall Score} & \textbf{%.1f} & /100 \\
\textbf{Grade} & \textbf{%s} & - \\
\hline
\end{tabular}
\end{table}
""" % (
            metrics.pos_rms_error, metrics.pos_max_error, metrics.pos_p95_error,
            metrics.cross_track_rms,
            metrics.vel_rms_error, metrics.vel_max_error,
            metrics.yaw_rms_error, metrics.yaw_delay,
            metrics.phase_delay,
            metrics.traj_max_velocity, metrics.traj_max_acceleration,
            metrics.traj_max_jerk, metrics.traj_difficulty_name,
            metrics.actual_jerk_rms, metrics.smoothness_index,
            metrics.energy_efficiency_index,
            metrics.overall_score, metrics.grade
        )

        # 保存 LaTeX 文件
        latex_path = os.path.join(self.output_dir, f"{name}.tex")
        with open(latex_path, 'w') as f:
            f.write(latex_content)
        print(f"  已保存: {latex_path}")

    def generate_all_figures(self, data, metrics, prefix=''):
        """
        生成所有出版级图表

        Args:
            data: 数据字典
            metrics: PerformanceMetrics 对象
            prefix: 文件名前缀
        """
        print("\n" + "=" * 60)
        print("生成出版级图表")
        print("=" * 60)

        if prefix:
            prefix = prefix + '_'

        print("\n[1/7] 3D轨迹图...")
        self.plot_trajectory_3d(data, metrics, f'{prefix}fig_trajectory_3d')

        print("\n[2/7] XY平面投影...")
        self.plot_trajectory_xy(data, metrics, f'{prefix}fig_trajectory_xy')

        print("\n[3/7] 误差时序图...")
        self.plot_error_timeseries(data, metrics, f'{prefix}fig_error_timeseries')

        print("\n[4/7] 误差分布图...")
        self.plot_error_distribution(data, metrics, f'{prefix}fig_error_distribution')

        print("\n[5/7] 动态特性图...")
        self.plot_dynamics(data, metrics, f'{prefix}fig_dynamics')

        print("\n[6/8] 雷达图...")
        self.plot_radar(metrics, f'{prefix}fig_radar')

        print("\n[7/8] 综合分析图...")
        self.plot_comprehensive_figure(data, metrics, f'{prefix}fig_comprehensive')

        print("\n[8/8] 指标表格...")
        self.generate_metrics_table(metrics, f'{prefix}table_metrics')

        print("\n" + "=" * 60)
        print(f"所有图表已保存至: {self.output_dir}")
        print("=" * 60 + "\n")


def main():
    """命令行入口"""
    import argparse

    parser = argparse.ArgumentParser(description='出版级图表生成工具')
    parser.add_argument('csv_file', help='CSV数据文件路径')
    parser.add_argument('--output-dir', '-o', default='./publication_plots',
                       help='输出目录')
    parser.add_argument('--prefix', '-p', default='',
                       help='文件名前缀')

    args = parser.parse_args()

    # 导入分析器
    from benchmark_analyzer import BenchmarkAnalyzer

    # 加载数据并计算指标
    analyzer = BenchmarkAnalyzer()
    data = analyzer.load_csv(args.csv_file)
    if data is None:
        print("错误: 无法加载数据文件")
        return 1

    metrics = analyzer.calculate_metrics(data)

    # 生成图表
    plotter = PublicationPlotter(args.output_dir)
    plotter.generate_all_figures(data, metrics, args.prefix)

    return 0


if __name__ == '__main__':
    sys.exit(main())
