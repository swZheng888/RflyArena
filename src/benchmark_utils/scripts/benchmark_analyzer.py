#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
benchmark_analyzer.py - 控制器综合性能评估系统 v2.0

核心评估指标体系：
================

1. 跟踪精度 (Tracking Accuracy)
   - 位置 RMS 误差 [m]
   - 位置最大误差 [m]
   - 横向误差 (Cross-Track Error) [m]
   - 沿轨误差 (Along-Track Error) [m]
   - 各轴位置误差 (X/Y/Z) [m]

2. 航向控制 (Yaw Control)
   - 航向 RMS 误差 [rad]
   - 航向最大误差 [rad]
   - 航向跟踪延迟 [s]

3. 速度跟踪 (Velocity Tracking)
   - 速度 RMS 误差 [m/s]
   - 速度最大误差 [m/s]

4. 时间响应 (Temporal Response)
   - 相位延迟 [s]
   - 调节时间 (Settling Time) [s]
   - 上升时间 (Rise Time) [s]

5. 轨迹特性 (Trajectory Characteristics)
   - 轨迹最大速度 [m/s]
   - 轨迹最大加速度 [m/s²]
   - 轨迹最大 Jerk [m/s³]
   - 轨迹总长度 [m]
   - 轨迹难度等级

6. 控制平滑度 (Control Smoothness)
   - 实际速度变化率 (加速度) RMS [m/s²]
   - 实际加速度变化率 (Jerk) RMS [m/s³]
   - 速度标准差 [m/s]

7. 能效估计 (Energy Efficiency)
   - 累积加速度积分 (推力代理) [m/s]
   - 平均加速度范数 [m/s²]
   - 能效指数 (距离/累积加速度)

8. 综合评分 (Overall Score)
   - 多维度加权评分 [0-100]
   - 性能等级 [A+/A/B/C/D/F]
"""

import os
import sys
import csv
import json
import numpy as np
import glob
import argparse
from datetime import datetime
from collections import defaultdict


try:
    import matplotlib
    matplotlib.use('Agg')  # 无GUI后端
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # seaborn可选
    try:
        import seaborn as sns
        sns.set_style("whitegrid")
        sns.set_palette("husl")
        SEABORN_AVAILABLE = True
    except:
        SEABORN_AVAILABLE = False

    # IROS/Science Paper 风格配置
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']
    plt.rcParams['mathtext.fontset'] = 'stix'
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.labelsize'] = 14
    plt.rcParams['axes.titlesize'] = 14
    plt.rcParams['xtick.labelsize'] = 12
    plt.rcParams['ytick.labelsize'] = 12
    plt.rcParams['legend.fontsize'] = 11
    plt.rcParams['figure.titlesize'] = 16
    plt.rcParams['lines.linewidth'] = 2.0
    plt.rcParams['lines.markersize'] = 6
    plt.rcParams['axes.grid'] = True
    plt.rcParams['grid.alpha'] = 0.3
    plt.rcParams['grid.linestyle'] = '--'
    plt.rcParams['savefig.dpi'] = 300
    plt.rcParams['figure.figsize'] = (10, 6)
    plt.rcParams['axes.unicode_minus'] = False

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("警告: matplotlib/seaborn 未安装，图表生成功能将不可用")


class PerformanceMetrics:
    """性能指标数据类"""

    def __init__(self):
        # 基本信息
        self.sample_count = 0
        self.duration = 0.0
        self.sampling_rate = 0.0
        self.is_hover = False

        # 1. 跟踪精度
        self.pos_rms_error = 0.0
        self.pos_max_error = 0.0
        self.pos_mean_error = 0.0
        self.pos_std_error = 0.0
        self.pos_median_error = 0.0
        self.pos_p95_error = 0.0
        self.pos_p99_error = 0.0

        self.cross_track_rms = 0.0
        self.cross_track_max = 0.0
        self.cross_track_mean = 0.0
        self.cross_track_p95 = 0.0

        self.along_track_rms = 0.0
        self.along_track_mean = 0.0

        self.pos_error_x_max = 0.0
        self.pos_error_y_max = 0.0
        self.pos_error_z_max = 0.0
        self.pos_error_x_mean = 0.0
        self.pos_error_y_mean = 0.0
        self.pos_error_z_mean = 0.0

        # 2. 航向控制
        self.yaw_rms_error = 0.0
        self.yaw_max_error = 0.0
        self.yaw_mean_error = 0.0
        self.yaw_delay = 0.0

        # 3. 速度跟踪
        self.vel_rms_error = 0.0
        self.vel_max_error = 0.0
        self.vel_mean_error = 0.0
        self.vel_std_error = 0.0

        # 4. 时间响应
        self.phase_delay = 0.0
        self.settling_time = 0.0
        self.rise_time = 0.0

        # 时间对齐后的指标（用于区分纯相位滞后与形状跟踪误差）
        self.aligned_pos_rms_error = 0.0
        self.aligned_pos_max_error = 0.0
        self.aligned_pos_mean_error = 0.0
        self.aligned_vel_rms_error = 0.0
        self.aligned_cross_track_rms = 0.0
        self.aligned_along_track_rms = 0.0
        self.aligned_yaw_rms_error = 0.0

        # 5. 轨迹特性
        self.traj_max_velocity = 0.0
        self.traj_mean_velocity = 0.0
        self.traj_max_acceleration = 0.0
        self.traj_mean_acceleration = 0.0
        self.traj_max_jerk = 0.0
        self.traj_mean_jerk = 0.0
        self.traj_total_length = 0.0
        self.traj_difficulty_level = 1
        self.traj_difficulty_name = "EASY"

        # 6. 控制平滑度
        self.actual_acc_rms = 0.0
        self.actual_jerk_rms = 0.0
        self.actual_vel_std = 0.0
        self.smoothness_index = 0.0

        # 7. 能效估计
        self.cumulative_acceleration = 0.0
        self.mean_acceleration_norm = 0.0
        self.energy_efficiency_index = 0.0

        # 8. 综合评分
        self.overall_score = 0.0
        self.grade = "F"

        # 悬停专用
        self.steady_state_error = 0.0
        self.peak_deviation = 0.0
        self.peak_time = 0.0
        self.settling_threshold = 0.0

    def to_dict(self):
        """转换为字典（确保所有值都是 JSON 可序列化的）"""
        result = {}
        for key, value in self.__dict__.items():
            # 转换 numpy 类型为 Python 原生类型
            if isinstance(value, (np.bool_, np.generic)):
                result[key] = value.item()
            elif isinstance(value, np.ndarray):
                result[key] = value.tolist()
            else:
                result[key] = value
        return result

    def to_json(self, filepath):
        """保存为JSON"""
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=2, ensure_ascii=False)


class BenchmarkAnalyzer:
    """控制器综合性能评估分析器"""

    # 物理常数
    GRAVITY = 9.81  # m/s²
    MASS = 0.319    # kg (默认无人机质量)

    def __init__(self, log_dir='~/.benchmark_logs'):
        """初始化分析器"""
        self.log_dir = os.path.expanduser(log_dir)

    def load_csv(self, csv_file):
        """
        加载CSV日志文件（支持新旧格式）

        Returns:
            dict: 包含所有数据的字典
        """
        data = {
            'time': [],
            'target_position': [],
            'actual_position': [],
            'target_velocity': [],
            'actual_velocity': [],
            'target_acceleration': [],
            'target_jerk': [],
            'target_yaw': [],
            'target_yaw_dot': [],
        }

        try:
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                fieldnames = reader.fieldnames

                for row in reader:
                    data['time'].append(float(row['time']))

                    data['target_position'].append([
                        float(row['target_x']),
                        float(row['target_y']),
                        float(row['target_z'])
                    ])

                    data['actual_position'].append([
                        float(row['actual_x']),
                        float(row['actual_y']),
                        float(row['actual_z'])
                    ])

                    data['target_velocity'].append([
                        float(row['target_vx']),
                        float(row['target_vy']),
                        float(row['target_vz'])
                    ])

                    data['actual_velocity'].append([
                        float(row['actual_vx']),
                        float(row['actual_vy']),
                        float(row['actual_vz'])
                    ])

                    # 新格式字段（可选）
                    if 'target_ax' in fieldnames:
                        data['target_acceleration'].append([
                            float(row.get('target_ax', 0)),
                            float(row.get('target_ay', 0)),
                            float(row.get('target_az', 0))
                        ])
                    else:
                        data['target_acceleration'].append([0.0, 0.0, 0.0])

                    if 'target_jerk_x' in fieldnames:
                        data['target_jerk'].append([
                            float(row.get('target_jerk_x', 0)),
                            float(row.get('target_jerk_y', 0)),
                            float(row.get('target_jerk_z', 0))
                        ])
                    else:
                        data['target_jerk'].append([0.0, 0.0, 0.0])

                    if 'target_yaw' in fieldnames:
                        data['target_yaw'].append(float(row.get('target_yaw', 0)))
                        data['target_yaw_dot'].append(float(row.get('target_yaw_dot', 0)))
                    else:
                        data['target_yaw'].append(0.0)
                        data['target_yaw_dot'].append(0.0)

        except Exception as e:
            print(f"加载CSV文件失败: {e}")
            return None

        # 转换为numpy数组
        for key in data:
            data[key] = np.array(data[key])

        return data

    def calculate_metrics(self, data):
        """
        计算完整的性能指标

        Returns:
            PerformanceMetrics: 性能指标对象
        """
        if data is None or len(data['time']) == 0:
            return PerformanceMetrics()

        metrics = PerformanceMetrics()

        # 基本信息
        metrics.sample_count = len(data['time'])
        metrics.duration = float(data['time'][-1] - data['time'][0])
        metrics.sampling_rate = metrics.sample_count / metrics.duration if metrics.duration > 0 else 0

        # 判断是否为悬停任务
        target_diff = np.diff(data['target_position'], axis=0)
        traj_len = np.sum(np.linalg.norm(target_diff, axis=1))
        metrics.is_hover = traj_len < 0.5
        metrics.traj_total_length = traj_len

        # =====================================================================
        # 1. 跟踪精度
        # =====================================================================
        pos_errors = np.linalg.norm(
            data['target_position'] - data['actual_position'], axis=1
        )
        pos_err_vec = data['target_position'] - data['actual_position']

        metrics.pos_rms_error = float(np.sqrt(np.mean(pos_errors**2)))
        metrics.pos_max_error = float(np.max(pos_errors))
        metrics.pos_mean_error = float(np.mean(pos_errors))
        metrics.pos_std_error = float(np.std(pos_errors))
        metrics.pos_median_error = float(np.median(pos_errors))
        metrics.pos_p95_error = float(np.percentile(pos_errors, 95))
        metrics.pos_p99_error = float(np.percentile(pos_errors, 99))

        # 各轴误差
        metrics.pos_error_x_max = float(np.max(np.abs(pos_err_vec[:, 0])))
        metrics.pos_error_y_max = float(np.max(np.abs(pos_err_vec[:, 1])))
        metrics.pos_error_z_max = float(np.max(np.abs(pos_err_vec[:, 2])))
        metrics.pos_error_x_mean = float(np.mean(np.abs(pos_err_vec[:, 0])))
        metrics.pos_error_y_mean = float(np.mean(np.abs(pos_err_vec[:, 1])))
        metrics.pos_error_z_mean = float(np.mean(np.abs(pos_err_vec[:, 2])))

        # 横向/沿轨误差（非悬停）
        if not metrics.is_hover:
            cross_track, along_track = self._calculate_cross_track_error(data)
            metrics.cross_track_rms = float(np.sqrt(np.mean(cross_track**2)))
            metrics.cross_track_max = float(np.max(cross_track))
            metrics.cross_track_mean = float(np.mean(cross_track))
            metrics.cross_track_p95 = float(np.percentile(cross_track, 95))
            metrics.along_track_rms = float(np.sqrt(np.mean(along_track**2)))
            metrics.along_track_mean = float(np.mean(along_track))

        # =====================================================================
        # 2. 航向控制
        # =====================================================================
        if len(data['target_yaw']) > 0 and np.any(data['target_yaw'] != 0):
            # 从实际速度计算实际航向
            actual_vel = data['actual_velocity']
            actual_yaw = np.arctan2(actual_vel[:, 1], actual_vel[:, 0])

            # 处理航向角差异（考虑周期性）
            yaw_errors = self._wrap_angle(data['target_yaw'] - actual_yaw)

            metrics.yaw_rms_error = float(np.sqrt(np.mean(yaw_errors**2)))
            metrics.yaw_max_error = float(np.max(np.abs(yaw_errors)))
            metrics.yaw_mean_error = float(np.mean(np.abs(yaw_errors)))

            # 航向延迟
            metrics.yaw_delay = self._estimate_delay(data['target_yaw'], actual_yaw, data['time'])

        # =====================================================================
        # 3. 速度跟踪
        # =====================================================================
        vel_errors = np.linalg.norm(
            data['target_velocity'] - data['actual_velocity'], axis=1
        )

        metrics.vel_rms_error = float(np.sqrt(np.mean(vel_errors**2)))
        metrics.vel_max_error = float(np.max(vel_errors))
        metrics.vel_mean_error = float(np.mean(vel_errors))
        metrics.vel_std_error = float(np.std(vel_errors))

        # =====================================================================
        # 4. 时间响应
        # =====================================================================
        if not metrics.is_hover:
            metrics.phase_delay = self._estimate_phase_delay(data)
            aligned = self._calculate_time_aligned_metrics(data, metrics.phase_delay)
            metrics.aligned_pos_rms_error = aligned['pos_rms_error']
            metrics.aligned_pos_max_error = aligned['pos_max_error']
            metrics.aligned_pos_mean_error = aligned['pos_mean_error']
            metrics.aligned_vel_rms_error = aligned['vel_rms_error']
            metrics.aligned_cross_track_rms = aligned['cross_track_rms']
            metrics.aligned_along_track_rms = aligned['along_track_rms']
            metrics.aligned_yaw_rms_error = aligned['yaw_rms_error']
        else:
            hover_metrics = self._analyze_hover_transient(data, pos_errors)
            metrics.steady_state_error = hover_metrics['steady_state_error']
            metrics.peak_deviation = hover_metrics['peak_deviation']
            metrics.peak_time = hover_metrics['peak_time']
            metrics.settling_time = hover_metrics['settling_time']
            metrics.settling_threshold = hover_metrics['settling_threshold']

        # =====================================================================
        # 5. 轨迹特性
        # =====================================================================
        target_vel_norms = np.linalg.norm(data['target_velocity'], axis=1)
        target_acc_norms = np.linalg.norm(data['target_acceleration'], axis=1)
        target_jerk_norms = np.linalg.norm(data['target_jerk'], axis=1)

        metrics.traj_max_velocity = float(np.max(target_vel_norms))
        metrics.traj_mean_velocity = float(np.mean(target_vel_norms))
        metrics.traj_max_acceleration = float(np.max(target_acc_norms))
        metrics.traj_mean_acceleration = float(np.mean(target_acc_norms))
        metrics.traj_max_jerk = float(np.max(target_jerk_norms))
        metrics.traj_mean_jerk = float(np.mean(target_jerk_norms))

        # 轨迹难度评估
        difficulty = self._estimate_trajectory_difficulty(
            metrics.traj_max_velocity,
            metrics.traj_max_acceleration,
            metrics.traj_max_jerk
        )
        metrics.traj_difficulty_level = difficulty['level']
        metrics.traj_difficulty_name = difficulty['name']

        # =====================================================================
        # 6. 控制平滑度
        # =====================================================================
        dt = np.mean(np.diff(data['time']))
        if dt > 0:
            # 实际加速度（从速度差分）
            actual_acc = np.diff(data['actual_velocity'], axis=0) / dt
            actual_acc_norms = np.linalg.norm(actual_acc, axis=1)
            metrics.actual_acc_rms = float(np.sqrt(np.mean(actual_acc_norms**2)))

            # 实际 Jerk（从加速度差分）
            if len(actual_acc) > 1:
                actual_jerk = np.diff(actual_acc, axis=0) / dt
                actual_jerk_norms = np.linalg.norm(actual_jerk, axis=1)
                metrics.actual_jerk_rms = float(np.sqrt(np.mean(actual_jerk_norms**2)))

            # 速度标准差
            actual_vel_norms = np.linalg.norm(data['actual_velocity'], axis=1)
            metrics.actual_vel_std = float(np.std(actual_vel_norms))

            # 平滑度指数（Jerk RMS 的倒数，越大越平滑）
            if metrics.actual_jerk_rms > 0:
                metrics.smoothness_index = 1.0 / metrics.actual_jerk_rms

        # =====================================================================
        # 7. 能效估计
        # =====================================================================
        # 累积加速度（推力代理）
        if len(data['target_acceleration']) > 0:
            # 总加速度 = 控制加速度 + 重力补偿
            total_acc = data['target_acceleration'].copy()
            total_acc[:, 2] += self.GRAVITY  # Z轴需要补偿重力

            acc_norms = np.linalg.norm(total_acc, axis=1)
            metrics.cumulative_acceleration = float(np.sum(acc_norms) * dt)
            metrics.mean_acceleration_norm = float(np.mean(acc_norms))

            # 能效指数 = 轨迹长度 / 累积加速度
            if metrics.cumulative_acceleration > 0:
                metrics.energy_efficiency_index = metrics.traj_total_length / metrics.cumulative_acceleration

        # =====================================================================
        # 8. 综合评分
        # =====================================================================
        score, grade = self._calculate_comprehensive_score(metrics)
        metrics.overall_score = score
        metrics.grade = grade

        return metrics

    def _wrap_angle(self, angle):
        """将角度归一化到 [-π, π]"""
        return np.arctan2(np.sin(angle), np.cos(angle))

    def _estimate_delay(self, target, actual, time):
        """估算信号延迟"""
        try:
            dt = np.mean(np.diff(time))
            if dt <= 0:
                return 0.0

            n = len(target)
            max_lag = min(int(1.0 / dt), n // 4)  # 最多1秒延迟

            best_lag = 0
            best_corr = -1

            for lag in range(max_lag):
                if lag == 0:
                    corr = np.corrcoef(target, actual)[0, 1]
                else:
                    corr = np.corrcoef(target[:-lag], actual[lag:])[0, 1]

                if not np.isnan(corr) and corr > best_corr:
                    best_corr = corr
                    best_lag = lag

            return float(best_lag * dt)
        except:
            return 0.0

    def _estimate_phase_delay(self, data):
        """估算位置跟踪的相位延迟"""
        try:
            target_x = data['target_position'][:, 0]
            actual_x = data['actual_position'][:, 0]
            return self._estimate_delay(target_x, actual_x, data['time'])
        except:
            return 0.0

    def _interp_series(self, time_src, values_src, time_query):
        """对 1D/2D 序列按时间插值。"""
        values_src = np.asarray(values_src)
        if values_src.ndim == 1:
            return np.interp(time_query, time_src, values_src)
        out = np.zeros((len(time_query), values_src.shape[1]))
        for i in range(values_src.shape[1]):
            out[:, i] = np.interp(time_query, time_src, values_src[:, i])
        return out

    def _calculate_time_aligned_metrics(self, data, phase_delay):
        """计算将目标按相位延迟平移后的对齐指标。"""
        result = {
            'pos_rms_error': 0.0,
            'pos_max_error': 0.0,
            'pos_mean_error': 0.0,
            'vel_rms_error': 0.0,
            'cross_track_rms': 0.0,
            'along_track_rms': 0.0,
            'yaw_rms_error': 0.0,
        }
        try:
            time = data['time']
            shifted_time = time - phase_delay

            target_pos_aligned = self._interp_series(time, data['target_position'], shifted_time)
            target_vel_aligned = self._interp_series(time, data['target_velocity'], shifted_time)
            target_yaw_aligned = self._interp_series(time, data['target_yaw'], shifted_time)

            pos_err_vec = target_pos_aligned - data['actual_position']
            pos_errors = np.linalg.norm(pos_err_vec, axis=1)
            vel_errors = np.linalg.norm(target_vel_aligned - data['actual_velocity'], axis=1)

            result['pos_rms_error'] = float(np.sqrt(np.mean(pos_errors**2)))
            result['pos_max_error'] = float(np.max(pos_errors))
            result['pos_mean_error'] = float(np.mean(pos_errors))
            result['vel_rms_error'] = float(np.sqrt(np.mean(vel_errors**2)))

            aligned_data = dict(data)
            aligned_data['target_position'] = target_pos_aligned
            cross_track, along_track = self._calculate_cross_track_error(aligned_data)
            result['cross_track_rms'] = float(np.sqrt(np.mean(cross_track**2)))
            result['along_track_rms'] = float(np.sqrt(np.mean(along_track**2)))

            actual_vel = data['actual_velocity']
            actual_yaw = np.arctan2(actual_vel[:, 1], actual_vel[:, 0])
            yaw_errors = self._wrap_angle(target_yaw_aligned - actual_yaw)
            result['yaw_rms_error'] = float(np.sqrt(np.mean(yaw_errors**2)))
        except Exception:
            pass

        return result

    def _calculate_cross_track_error(self, data):
        """基于同一时刻附近的参考线段投影计算横向误差和沿轨误差。"""
        target = data['target_position']
        actual = data['actual_position']
        n = len(target)

        cross_track = np.zeros(n)
        along_track = np.zeros(n)

        if n == 0:
            return cross_track, along_track

        if n == 1:
            diff = actual - target[0]
            cross_track[:] = np.linalg.norm(diff, axis=1)
            return cross_track, along_track

        for i in range(n):
            point = actual[i]

            candidates = []
            if i > 0:
                candidates.append((target[i - 1], target[i]))
            if i < n - 1:
                candidates.append((target[i], target[i + 1]))

            best_cross = float('inf')
            best_along = 0.0

            for seg_start, seg_end in candidates:
                seg_vec = seg_end - seg_start
                seg_len = np.linalg.norm(seg_vec)

                if seg_len < 1e-9:
                    err_vec = point - seg_start
                    along = 0.0
                    cross = np.linalg.norm(err_vec)
                else:
                    t_hat = seg_vec / seg_len
                    err_vec = point - target[i]
                    along = np.dot(err_vec, t_hat)
                    cross_vec = err_vec - along * t_hat
                    cross = np.linalg.norm(cross_vec)

                if cross < best_cross:
                    best_cross = cross
                    best_along = abs(along)

            cross_track[i] = best_cross
            along_track[i] = best_along

        return cross_track, along_track

    def _analyze_hover_transient(self, data, pos_errors):
        """分析悬停瞬态特性"""
        time = data['time']
        start_time = time[0]

        # 稳态误差（最后20%数据）
        steady_state_idx = int(len(pos_errors) * 0.8)
        steady_state_error = np.mean(pos_errors[steady_state_idx:])

        # 最大偏差
        max_dev = np.max(pos_errors)
        max_dev_idx = np.argmax(pos_errors)
        peak_time = time[max_dev_idx] - start_time

        # 调节时间
        settling_threshold = max(0.10, steady_state_error * 1.2)
        settling_time = 0.0

        for i in range(len(pos_errors) - 1, -1, -1):
            if pos_errors[i] > settling_threshold:
                settling_time = time[i] - start_time
                break

        if max_dev < settling_threshold:
            settling_time = 0.0

        return {
            'steady_state_error': float(steady_state_error),
            'peak_deviation': float(max_dev),
            'peak_time': float(peak_time),
            'settling_time': float(settling_time),
            'settling_threshold': float(settling_threshold)
        }

    def _estimate_trajectory_difficulty(self, max_vel, max_acc, max_jerk):
        """估算轨迹难度"""
        # 基于速度、加速度、Jerk 的综合评估
        score = 0

        # 速度评分
        if max_vel > 3.0:
            score += 2
        elif max_vel > 1.5:
            score += 1

        # 加速度评分
        if max_acc > 5.0:
            score += 2
        elif max_acc > 2.0:
            score += 1

        # Jerk 评分
        if max_jerk > 10.0:
            score += 2
        elif max_jerk > 3.0:
            score += 1

        # 映射到难度等级
        if score >= 5:
            return {'level': 4, 'name': 'EXTREME'}
        elif score >= 3:
            return {'level': 3, 'name': 'HARD'}
        elif score >= 1:
            return {'level': 2, 'name': 'MEDIUM'}
        else:
            return {'level': 1, 'name': 'EASY'}

    def _calculate_comprehensive_score(self, metrics):
        """
        计算综合评分

        评分维度与权重：
        - 位置精度 (40%): 基于 RMS 误差
        - 速度跟踪 (20%): 基于速度 RMS 误差
        - 控制平滑度 (20%): 基于 Jerk RMS
        - 响应速度 (10%): 基于相位延迟
        - 能效 (10%): 基于能效指数
        """
        def score_metric(val, best, worst):
            """计算单项得分 [0-100]"""
            if val <= best:
                return 100
            if val >= worst:
                return 0
            return 100 * (1 - (val - best) / (worst - best))

        # 选择合适的误差指标
        if metrics.is_hover:
            pos_score = score_metric(metrics.pos_rms_error, 0.03, 0.30)
        else:
            pos_score = score_metric(metrics.cross_track_rms if metrics.cross_track_rms > 0 else metrics.pos_rms_error, 0.03, 0.25)

        vel_score = score_metric(metrics.vel_rms_error, 0.05, 0.50)
        smooth_score = score_metric(metrics.actual_jerk_rms, 1.0, 20.0) if metrics.actual_jerk_rms > 0 else 50
        delay_score = score_metric(abs(metrics.phase_delay), 0.01, 0.30)
        energy_score = min(100, metrics.energy_efficiency_index * 100) if metrics.energy_efficiency_index > 0 else 50

        # 加权平均
        total_score = (
            pos_score * 0.40 +
            vel_score * 0.20 +
            smooth_score * 0.20 +
            delay_score * 0.10 +
            energy_score * 0.10
        )

        # 难度调整（高难度轨迹适当加分）
        difficulty_bonus = (metrics.traj_difficulty_level - 1) * 2
        total_score = min(100, total_score + difficulty_bonus)

        # 等级映射
        if total_score >= 95:
            grade = "A+"
        elif total_score >= 85:
            grade = "A"
        elif total_score >= 75:
            grade = "B"
        elif total_score >= 65:
            grade = "C"
        elif total_score >= 50:
            grade = "D"
        else:
            grade = "F"

        return total_score, grade

    # =========================================================================
    # 可视化方法
    # =========================================================================

    def plot_trajectory_3d(self, data, save_path, metrics=None):
        """绘制3D轨迹对比图"""
        if not MATPLOTLIB_AVAILABLE:
            return

        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')

        target_pos = data['target_position']
        actual_pos = data['actual_position']

        # 实际轨迹（根据速度着色）
        ax.grid(False)
        actual_vel = data['actual_velocity']
        vel_norm = np.linalg.norm(actual_vel, axis=1)
        scatter = ax.scatter(actual_pos[:, 0], actual_pos[:, 1], actual_pos[:, 2],
                           c=vel_norm, cmap='jet', s=8, alpha=0.35,
                           label='Actual', zorder=3)

        # 期望轨迹
        ax.plot(target_pos[:, 0], target_pos[:, 1], target_pos[:, 2],
                'b--', linewidth=2.0, label='Reference', alpha=0.9, zorder=5)

        cbar = plt.colorbar(scatter, ax=ax, pad=0.1, shrink=0.7)
        cbar.set_label('Velocity (m/s)', fontsize=12)

        # 起点和终点
        ax.scatter(target_pos[0, 0], target_pos[0, 1], target_pos[0, 2],
                  c='g', s=150, marker='o', label='Start', edgecolors='black', linewidth=1.5)
        ax.scatter(target_pos[-1, 0], target_pos[-1, 1], target_pos[-1, 2],
                  c='r', s=150, marker='s', label='End', edgecolors='black', linewidth=1.5)

        ax.set_xlabel('X (m)', fontsize=14)
        ax.set_ylabel('Y (m)', fontsize=14)
        ax.set_zlabel('Z (m)', fontsize=14)

        # 添加性能指标到标题
        if metrics:
            title = f"3D Trajectory | RMS: {metrics.pos_rms_error:.3f}m | Grade: {metrics.grade}"
        else:
            title = "3D Trajectory Tracking"
        ax.set_title(title, fontsize=16, fontweight='bold', pad=20)
        ax.legend(fontsize=12, loc='upper right')

        # 统一坐标轴比例
        max_range = max(
            target_pos[:, 0].max() - target_pos[:, 0].min(),
            target_pos[:, 1].max() - target_pos[:, 1].min(),
            target_pos[:, 2].max() - target_pos[:, 2].min()
        )
        mid_x = (target_pos[:, 0].max() + target_pos[:, 0].min()) * 0.5
        mid_y = (target_pos[:, 1].max() + target_pos[:, 1].min()) * 0.5
        mid_z = (target_pos[:, 2].max() + target_pos[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range*0.6, mid_x + max_range*0.6)
        ax.set_ylim(mid_y - max_range*0.6, mid_y + max_range*0.6)
        ax.set_zlim(mid_z - max_range*0.6, mid_z + max_range*0.6)

        try:
            ax.set_box_aspect((1, 1, 1))
        except:
            pass

        ax.view_init(elev=20, azim=-60)

        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"已生成: {save_path}")

    def plot_comprehensive_analysis(self, data, save_path, metrics):
        """绘制综合分析图（多子图）"""
        if not MATPLOTLIB_AVAILABLE:
            return

        fig, axes = plt.subplots(3, 2, figsize=(16, 14))

        time = data['time']

        # 1. 位置误差时序
        ax1 = axes[0, 0]
        pos_err = data['target_position'] - data['actual_position']
        ax1.plot(time, pos_err[:, 0], 'r-', linewidth=1.2, label='X Error', alpha=0.8)
        ax1.plot(time, pos_err[:, 1], 'g-', linewidth=1.2, label='Y Error', alpha=0.8)
        ax1.plot(time, pos_err[:, 2], 'b-', linewidth=1.2, label='Z Error', alpha=0.8)
        ax1.axhline(0, color='k', linestyle='-', linewidth=0.5)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position Error (m)')
        ax1.set_title(f'Position Error | RMS: {metrics.pos_rms_error:.4f}m')
        ax1.legend(loc='upper right')
        ax1.grid(True, alpha=0.3)

        # 2. 位置误差范数
        ax2 = axes[0, 1]
        pos_err_norm = np.linalg.norm(pos_err, axis=1)
        ax2.plot(time, pos_err_norm, 'purple', linewidth=1.5)
        ax2.axhline(metrics.pos_rms_error, color='r', linestyle='--', linewidth=1.5,
                   label=f'RMS: {metrics.pos_rms_error:.4f}m')
        ax2.axhline(metrics.pos_max_error, color='orange', linestyle=':', linewidth=1.5,
                   label=f'Max: {metrics.pos_max_error:.4f}m')
        ax2.fill_between(time, 0, pos_err_norm, alpha=0.3, color='purple')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error Norm (m)')
        ax2.set_title('Position Error Magnitude')
        ax2.legend(loc='upper right')
        ax2.grid(True, alpha=0.3)

        # 3. 速度跟踪
        ax3 = axes[1, 0]
        target_vel_norm = np.linalg.norm(data['target_velocity'], axis=1)
        actual_vel_norm = np.linalg.norm(data['actual_velocity'], axis=1)
        ax3.plot(time, target_vel_norm, 'b--', linewidth=2, label='Reference', alpha=0.8)
        ax3.plot(time, actual_vel_norm, 'r-', linewidth=1.5, label='Actual', alpha=0.8)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Velocity (m/s)')
        ax3.set_title(f'Velocity Tracking | Max: {metrics.traj_max_velocity:.2f} m/s')
        ax3.legend(loc='upper right')
        ax3.grid(True, alpha=0.3)

        # 4. 加速度/Jerk 分析
        ax4 = axes[1, 1]
        target_acc_norm = np.linalg.norm(data['target_acceleration'], axis=1)
        target_jerk_norm = np.linalg.norm(data['target_jerk'], axis=1)
        ax4.plot(time, target_acc_norm, 'b-', linewidth=1.5, label='Acceleration', alpha=0.8)
        ax4.plot(time, target_jerk_norm, 'orange', linewidth=1.2, label='Jerk', alpha=0.7)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Magnitude')
        ax4.set_title(f'Trajectory Dynamics | Max Acc: {metrics.traj_max_acceleration:.2f} m/s²')
        ax4.legend(loc='upper right')
        ax4.grid(True, alpha=0.3)

        # 5. XY 平面轨迹
        ax5 = axes[2, 0]
        target_pos = data['target_position']
        actual_pos = data['actual_position']
        ax5.plot(target_pos[:, 0], target_pos[:, 1], 'b--', linewidth=2, label='Reference')
        ax5.plot(actual_pos[:, 0], actual_pos[:, 1], 'r-', linewidth=1.2, label='Actual', alpha=0.7)
        ax5.scatter(target_pos[0, 0], target_pos[0, 1], c='g', s=100, marker='o', label='Start', zorder=5)
        ax5.scatter(target_pos[-1, 0], target_pos[-1, 1], c='r', s=100, marker='s', label='End', zorder=5)
        ax5.set_xlabel('X (m)')
        ax5.set_ylabel('Y (m)')
        ax5.set_title('XY Plane Projection')
        ax5.legend(loc='upper right')
        ax5.axis('equal')
        ax5.grid(True, alpha=0.3)

        # 6. 性能雷达图
        ax6 = axes[2, 1]
        self._plot_radar_on_axis(ax6, metrics)

        plt.suptitle(f'Controller Performance Analysis | Overall: {metrics.overall_score:.1f}/100 ({metrics.grade})',
                    fontsize=18, fontweight='bold', y=1.02)
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"已生成: {save_path}")

    def _plot_radar_on_axis(self, ax, metrics):
        """在指定轴上绘制雷达图"""
        categories = ['Tracking\nPrecision', 'Velocity\nTracking', 'Smoothness',
                     'Response', 'Energy\nEfficiency']

        def score(val, best, worst):
            if val <= best:
                return 100
            if val >= worst:
                return 0
            return 100 * (1 - (val - best) / (worst - best))

        values = [
            score(metrics.pos_rms_error, 0.03, 0.25),
            score(metrics.vel_rms_error, 0.05, 0.50),
            score(metrics.actual_jerk_rms, 1.0, 20.0) if metrics.actual_jerk_rms > 0 else 50,
            score(abs(metrics.phase_delay), 0.01, 0.30),
            min(100, metrics.energy_efficiency_index * 100) if metrics.energy_efficiency_index > 0 else 50
        ]

        angles = np.linspace(0, 2*np.pi, len(categories), endpoint=False).tolist()
        values_plot = values + values[:1]
        angles += angles[:1]

        ax.clear()
        ax = plt.subplot(3, 2, 6, polar=True)

        ax.fill(angles, values_plot, color='#3498db', alpha=0.25)
        ax.plot(angles, values_plot, 'o-', color='#2980b9', linewidth=2, markersize=8)

        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(categories, fontsize=10)
        ax.set_ylim(0, 100)
        ax.set_yticks([20, 40, 60, 80, 100])

        ax.set_title(f'Performance Radar\nScore: {metrics.overall_score:.1f}', fontsize=12, fontweight='bold')

    def generate_report(self, task_name, metrics, csv_file, output_dir):
        """生成完整的分析报告（文本 + JSON）"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # 文本报告
        txt_file = os.path.join(output_dir, f"report_{task_name}.txt")
        with open(txt_file, 'w', encoding='utf-8') as f:
            f.write("=" * 80 + "\n")
            f.write(f"控制器性能评估报告 - {task_name.upper()}\n")
            f.write("=" * 80 + "\n\n")

            f.write("【基本信息】\n")
            f.write("-" * 80 + "\n")
            f.write(f"任务名称: {task_name}\n")
            f.write(f"分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"数据文件: {csv_file}\n")
            f.write(f"采样点数: {metrics.sample_count}\n")
            f.write(f"执行时长: {metrics.duration:.2f} s\n")
            f.write(f"采样频率: {metrics.sampling_rate:.1f} Hz\n")
            f.write(f"任务类型: {'悬停' if metrics.is_hover else '轨迹跟踪'}\n\n")

            f.write("【综合评分】\n")
            f.write("-" * 80 + "\n")
            f.write(f"综合得分: {metrics.overall_score:.1f} / 100\n")
            f.write(f"性能等级: {metrics.grade}\n\n")

            f.write("【轨迹特性】\n")
            f.write("-" * 80 + "\n")
            f.write(f"轨迹总长度: {metrics.traj_total_length:.2f} m\n")
            f.write(f"最大速度: {metrics.traj_max_velocity:.3f} m/s\n")
            f.write(f"平均速度: {metrics.traj_mean_velocity:.3f} m/s\n")
            f.write(f"最大加速度: {metrics.traj_max_acceleration:.3f} m/s²\n")
            f.write(f"平均加速度: {metrics.traj_mean_acceleration:.3f} m/s²\n")
            f.write(f"最大Jerk: {metrics.traj_max_jerk:.3f} m/s³\n")
            f.write(f"轨迹难度: {metrics.traj_difficulty_name} (Level {metrics.traj_difficulty_level})\n\n")

            f.write("【位置跟踪精度】\n")
            f.write("-" * 80 + "\n")
            f.write(f"位置 RMS 误差: {metrics.pos_rms_error:.4f} m\n")
            f.write(f"位置最大误差: {metrics.pos_max_error:.4f} m\n")
            f.write(f"位置平均误差: {metrics.pos_mean_error:.4f} m\n")
            f.write(f"位置误差标准差: {metrics.pos_std_error:.4f} m\n")
            f.write(f"位置 P95 误差: {metrics.pos_p95_error:.4f} m\n")
            f.write(f"位置 P99 误差: {metrics.pos_p99_error:.4f} m\n")
            f.write(f"X轴最大误差: {metrics.pos_error_x_max:.4f} m\n")
            f.write(f"Y轴最大误差: {metrics.pos_error_y_max:.4f} m\n")
            f.write(f"Z轴最大误差: {metrics.pos_error_z_max:.4f} m\n\n")

            if not metrics.is_hover:
                f.write("【横向/沿轨误差】\n")
                f.write("-" * 80 + "\n")
                f.write(f"横向 RMS 误差: {metrics.cross_track_rms:.4f} m\n")
                f.write(f"横向最大误差: {metrics.cross_track_max:.4f} m\n")
                f.write(f"横向 P95 误差: {metrics.cross_track_p95:.4f} m\n")
                f.write(f"沿轨 RMS 误差: {metrics.along_track_rms:.4f} m\n\n")

            f.write("【速度跟踪】\n")
            f.write("-" * 80 + "\n")
            f.write(f"速度 RMS 误差: {metrics.vel_rms_error:.4f} m/s\n")
            f.write(f"速度最大误差: {metrics.vel_max_error:.4f} m/s\n")
            f.write(f"速度平均误差: {metrics.vel_mean_error:.4f} m/s\n\n")

            f.write("【航向控制】\n")
            f.write("-" * 80 + "\n")
            f.write(f"航向 RMS 误差: {metrics.yaw_rms_error:.4f} rad ({np.degrees(metrics.yaw_rms_error):.2f}°)\n")
            f.write(f"航向最大误差: {metrics.yaw_max_error:.4f} rad ({np.degrees(metrics.yaw_max_error):.2f}°)\n")
            f.write(f"航向跟踪延迟: {metrics.yaw_delay:.4f} s\n\n")

            f.write("【时间响应】\n")
            f.write("-" * 80 + "\n")
            f.write(f"相位延迟: {metrics.phase_delay:.4f} s\n")
            if metrics.is_hover:
                f.write(f"调节时间: {metrics.settling_time:.4f} s\n")
                f.write(f"峰值偏差: {metrics.peak_deviation:.4f} m\n")
                f.write(f"峰值时刻: {metrics.peak_time:.4f} s\n")
                f.write(f"稳态误差: {metrics.steady_state_error:.4f} m\n")
            f.write("\n")

            if not metrics.is_hover:
                f.write("【时间对齐后指标】\n")
                f.write("-" * 80 + "\n")
                f.write(f"对齐位置 RMS 误差: {metrics.aligned_pos_rms_error:.4f} m\n")
                f.write(f"对齐位置最大误差: {metrics.aligned_pos_max_error:.4f} m\n")
                f.write(f"对齐位置平均误差: {metrics.aligned_pos_mean_error:.4f} m\n")
                f.write(f"对齐速度 RMS 误差: {metrics.aligned_vel_rms_error:.4f} m/s\n")
                f.write(f"对齐横向 RMS 误差: {metrics.aligned_cross_track_rms:.4f} m\n")
                f.write(f"对齐沿轨 RMS 误差: {metrics.aligned_along_track_rms:.4f} m\n")
                f.write(f"对齐航向 RMS 误差: {metrics.aligned_yaw_rms_error:.4f} rad ({np.degrees(metrics.aligned_yaw_rms_error):.2f}°)\n\n")

            f.write("【控制平滑度】\n")
            f.write("-" * 80 + "\n")
            f.write(f"实际加速度 RMS: {metrics.actual_acc_rms:.4f} m/s²\n")
            f.write(f"实际 Jerk RMS: {metrics.actual_jerk_rms:.4f} m/s³\n")
            f.write(f"速度标准差: {metrics.actual_vel_std:.4f} m/s\n")
            f.write(f"平滑度指数: {metrics.smoothness_index:.4f}\n\n")

            f.write("【能效估计】\n")
            f.write("-" * 80 + "\n")
            f.write(f"累积加速度: {metrics.cumulative_acceleration:.4f} m/s\n")
            f.write(f"平均加速度范数: {metrics.mean_acceleration_norm:.4f} m/s²\n")
            f.write(f"能效指数: {metrics.energy_efficiency_index:.4f}\n\n")

            f.write("=" * 80 + "\n")
            f.write("报告生成完成\n")
            f.write("=" * 80 + "\n")

        print(f"已生成报告: {txt_file}")

        # JSON 报告
        json_file = os.path.join(output_dir, f"metrics_{task_name}.json")
        metrics.to_json(json_file)
        print(f"已生成指标: {json_file}")

    def analyze(self, csv_file, task_name=None, output_dir=None, skip_plot=False):
        """
        完整分析流程

        Args:
            csv_file: CSV数据文件路径
            task_name: 任务名称
            output_dir: 输出目录
            skip_plot: 跳过图表生成（加速调参）
        """
        if output_dir is None:
            output_dir = os.path.dirname(csv_file)

        if task_name is None:
            basename = os.path.basename(csv_file)
            task_name = basename.replace('.csv', '')

        print("\n" + "=" * 70)
        print(f"控制器性能评估: {task_name}")
        print(f"数据文件: {csv_file}")
        print("=" * 70 + "\n")

        # 1. 加载数据
        print("正在加载数据...")
        data = self.load_csv(csv_file)
        if data is None:
            print("数据加载失败！")
            return None
        print(f"已加载 {len(data['time'])} 个数据点\n")

        # 2. 计算指标
        print("正在计算性能指标...")
        metrics = self.calculate_metrics(data)
        print(f"综合评分: {metrics.overall_score:.1f}/100 ({metrics.grade})\n")

        # 3. 生成图表
        if MATPLOTLIB_AVAILABLE and not skip_plot:
            print("正在生成出版级图表...")

            # 使用出版级绘图器
            try:
                from publication_plotter import PublicationPlotter
                plotter = PublicationPlotter(output_dir)
                plotter.generate_all_figures(data, metrics, task_name)
            except ImportError:
                # 回退到基础图表
                print("  (使用基础绘图模式)")
                plot_3d = os.path.join(output_dir, f"{task_name}_3d.png")
                self.plot_trajectory_3d(data, plot_3d, metrics)

                plot_analysis = os.path.join(output_dir, f"{task_name}_analysis.png")
                self.plot_comprehensive_analysis(data, plot_analysis, metrics)

            print()

        # 4. 生成报告
        print("正在生成分析报告...")
        self.generate_report(task_name, metrics, csv_file, output_dir)

        print("\n" + "=" * 70)
        print("分析完成！")
        print("=" * 70 + "\n")

        return metrics


def main():
    """命令行入口"""
    parser = argparse.ArgumentParser(description='控制器综合性能评估工具')
    parser.add_argument('--csv', required=True, help='CSV数据文件路径')
    parser.add_argument('--output', help='输出目录（默认为CSV文件所在目录）')
    parser.add_argument('--name', help='任务名称（默认从文件名提取）')
    parser.add_argument('--no-plot', action='store_true', help='跳过图表生成（加速调参）')

    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"错误: 文件不存在 - {args.csv}")
        return 1

    analyzer = BenchmarkAnalyzer()
    analyzer.analyze(args.csv, task_name=args.name, output_dir=args.output, skip_plot=getattr(args, 'no_plot', False))

    return 0


if __name__ == '__main__':
    sys.exit(main())
