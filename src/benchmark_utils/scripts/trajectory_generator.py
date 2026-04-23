#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
trajectory_generator.py - 重构版 v2.0

微分平坦轨迹生成器 - 完整版

核心改进:
1. 所有轨迹类型都提供完整的微分平坦信息 (pos, vel, acc, jerk, yaw, yaw_dot)
2. 软启动机制正确应用到所有轨迹
3. 统一速度定义：base_speed * speed_factor
4. 准确的周期计算
5. 轨迹难度评级系统

支持的轨迹类型 (12种):
1. circle - 圆形（水平）
2. ellipse - 椭圆
3. long_ellipse - 长椭圆
4. tilted_ellipse - 倾斜椭圆
5. figure8 - 8字形
6. spiral_circle - 螺旋圆（3D）
7. cone3d - 3D锥形
8. sine_wave - 正弦波
9. zigzag - 锯齿形 (Lissajous)
10. polynomial - 多项式轨迹
11. square - 方形
12. star - 五角星

测试空间：5m x 5m x 5m
"""

import numpy as np
import rospy


class TrajectoryDifficulty:
    """轨迹难度评级系统"""

    # 难度等级定义
    EASY = 1        # 简单：低速、平滑曲率
    MEDIUM = 2      # 中等：中速、适度曲率变化
    HARD = 3        # 困难：高速、急剧曲率变化
    EXTREME = 4     # 极限：极高速、不可微点

    @staticmethod
    def get_difficulty(traj_type, speed_factor=1.0):
        """
        获取轨迹难度等级

        Returns:
            dict: {
                'level': 难度等级 (1-4),
                'name': 等级名称,
                'description': 描述,
                'max_curvature': 最大曲率估计,
                'is_differentiable': 是否处处可微
            }
        """
        # 基础难度定义
        difficulty_map = {
            'circle': {
                'base_level': 1,
                'max_curvature': 0.5,  # 1/R
                'is_differentiable': True,
                'description': '恒定曲率，适合基准测试'
            },
            'ellipse': {
                'base_level': 1,
                'max_curvature': 0.6,
                'is_differentiable': True,
                'description': '变曲率，轻微难度提升'
            },
            'long_ellipse': {
                'base_level': 2,
                'max_curvature': 0.8,
                'is_differentiable': True,
                'description': '大离心率椭圆，曲率变化显著'
            },
            'tilted_ellipse': {
                'base_level': 2,
                'max_curvature': 0.7,
                'is_differentiable': True,
                'description': '3D倾斜椭圆，需要姿态协调'
            },
            'figure8': {
                'base_level': 2,
                'max_curvature': 1.0,
                'is_differentiable': True,
                'description': '8字形，需频繁变向'
            },
            'spiral_circle': {
                'base_level': 2,
                'max_curvature': 0.6,
                'is_differentiable': True,
                'description': '3D螺旋，测试高度控制'
            },
            'cone3d': {
                'base_level': 3,
                'max_curvature': 1.2,
                'is_differentiable': True,
                'description': '锥形螺旋，半径持续变化'
            },
            'sine_wave': {
                'base_level': 2,
                'max_curvature': 0.8,
                'is_differentiable': True,
                'description': '正弦波，周期性加减速'
            },
            'zigzag': {
                'base_level': 4,
                'max_curvature': float('inf'),
                'is_differentiable': False,
                'description': '锯齿形，拐点处不可微'
            },
            'polynomial': {
                'base_level': 2,
                'max_curvature': 0.9,
                'is_differentiable': True,
                'description': '多项式平滑曲线'
            },
            'square': {
                'base_level': 4,
                'max_curvature': float('inf'),
                'is_differentiable': False,
                'description': '方形，直角拐点不可微'
            },
            'star': {
                'base_level': 4,
                'max_curvature': float('inf'),
                'is_differentiable': False,
                'description': '五角星，尖角不可微'
            },
        }

        if traj_type not in difficulty_map:
            return {
                'level': 2,
                'name': 'MEDIUM',
                'description': '未知轨迹类型',
                'max_curvature': 1.0,
                'is_differentiable': True
            }

        info = difficulty_map[traj_type]

        # 速度因子影响难度
        speed_bonus = 0
        if speed_factor >= 2.0:
            speed_bonus = 1
        elif speed_factor >= 1.5:
            speed_bonus = 0.5

        final_level = min(4, info['base_level'] + speed_bonus)

        level_names = {1: 'EASY', 2: 'MEDIUM', 3: 'HARD', 4: 'EXTREME'}

        return {
            'level': final_level,
            'name': level_names[int(final_level)],
            'description': info['description'],
            'max_curvature': info['max_curvature'] * speed_factor,
            'is_differentiable': info['is_differentiable'],
            'aggressiveness': final_level * speed_factor  # 综合激进度指标
        }


class DifferentialFlatTrajectory:
    """微分平坦轨迹生成器 - 完整版"""

    def __init__(self, frame_id='world'):
        self.frame_id = frame_id
        self.trajectory_start_time = None

        # 核心参数
        self.speed_factor = 1.0      # 速度倍率 (1x, 1.5x, 2x, ...)
        self.amplitude = 2.0         # 轨迹幅度 (m)
        self.base_speed = 1.0        # 基础速度 (m/s)
        self.num_loops = 3           # 执行圈数
        self.z_base = 2.5            # 基础飞行高度 (m)

        # 软启动参数
        self.soft_start_duration = 3.0  # 软启动持续时间 (s)
        self.soft_start_enabled = True  # 是否启用软启动

        # 世界坐标偏移量（使轨迹与无人机实际起点对齐）
        self._world_offset = np.zeros(3)
        self._current_traj_type = None  # 跟踪当前轨迹类型，供 set_world_offset 使用

    def set_trajectory_start_time(self, start_time):
        """设置轨迹开始时间"""
        self.trajectory_start_time = start_time

    def set_params(self, speed_factor=1.0, amplitude=2.0, num_loops=3,
                   base_speed=1.0, soft_start_duration=3.0, world_offset=None):
        """设置轨迹参数"""
        self.speed_factor = speed_factor
        self.amplitude = amplitude
        self.num_loops = num_loops
        self.base_speed = base_speed
        self.soft_start_duration = soft_start_duration
        if world_offset is not None:
            self._world_offset = np.asarray(world_offset, dtype=float)

    def set_world_offset(self, drone_start_pos):
        """
        设置世界坐标偏移量，使轨迹以无人机实际起飞位置为参考。

        轨迹原始坐标是以世界原点为中心，但无人机的实际位置
        可能不在原点。调用此方法后，所有轨迹位置都会偏移:
            pos_world = pos_trajectory + world_offset

        其中 world_offset = drone_start_pos - pos_trajectory(t=0)，
        确保轨迹在 t=0 时正好从无人机实际位置出发。

        Args:
            drone_start_pos (np.ndarray): 无人机起飞时的 ENU 世界坐标 (3,)
        """
        if self._current_traj_type is None:
            rospy.logwarn("Trajectory type not set. Cannot calculate world_offset accurately. "
                          "Please call get_trajectory once to set the type.")
            # Fallback to a default t=0 position if type is unknown
            traj_pos_t0 = np.array([self.amplitude, 0.0, self.z_base])
        else:
            traj_pos_t0 = self._get_traj_pos_t0(self._current_traj_type)

        self._world_offset = np.asarray(drone_start_pos, dtype=float) - traj_pos_t0
        rospy.loginfo("[trajectory] world_offset set to [%.2f, %.2f, %.2f]",
                      self._world_offset[0], self._world_offset[1], self._world_offset[2])

    def _get_traj_pos_t0(self, traj_type):
        """
        获取轨迹在 t=0 时的原始位置（用于计算 world_offset）
        为了获取原始位置，需要临时禁用软启动和世界偏移。
        """
        saved_soft_start_enabled = self.soft_start_enabled
        saved_world_offset = self._world_offset.copy()

        self.soft_start_enabled = False
        self._world_offset = np.zeros(3) # 临时设置为0，确保获取的是原始轨迹的t=0位置

        try:
            # 调用 get_trajectory 获取 t=0 的原始位置
            pos_t0, _, _, _, _, _ = self.get_trajectory(traj_type, 0.0)
            return pos_t0
        finally:
            # 恢复之前的设置
            self.soft_start_enabled = saved_soft_start_enabled
            self._world_offset = saved_world_offset

    def enable_soft_start(self, enabled=True, duration=3.0):
        """启用/禁用软启动"""
        self.soft_start_enabled = enabled
        self.soft_start_duration = duration

    # =========================================================================
    # 软启动机制
    # =========================================================================

    def _get_soft_start_factor(self, t):
        """
        计算软启动因子 (0~1)
        使用 Smootherstep (S曲线) 平滑过渡

        公式: 6t^5 - 15t^4 + 10t^3
        """
        if not self.soft_start_enabled or self.soft_start_duration <= 0:
            return 1.0
        if t >= self.soft_start_duration:
            return 1.0
        if t <= 0:
            return 0.0

        x = t / self.soft_start_duration
        return x * x * x * (x * (x * 6 - 15) + 10)

    def _get_soft_start_derivatives(self, t):
        """
        计算软启动因子及其导数 (用于链式法则)

        Returns:
            (s, s_dot, s_ddot, s_dddot): 软启动因子及其1/2/3阶导数
        """
        if not self.soft_start_enabled or self.soft_start_duration <= 0:
            return 1.0, 0.0, 0.0, 0.0
        if t >= self.soft_start_duration:
            return 1.0, 0.0, 0.0, 0.0
        if t <= 0:
            return 0.0, 0.0, 0.0, 0.0

        T = self.soft_start_duration
        x = t / T

        # s = 6x^5 - 15x^4 + 10x^3
        s = x**3 * (x * (x * 6 - 15) + 10)

        # s' = (30x^4 - 60x^3 + 30x^2) / T = 30x^2(x-1)^2 / T
        s_dot = 30 * x**2 * (1 - x)**2 / T

        # s'' = (120x^3 - 180x^2 + 60x) / T^2 = 60x(2x-1)(x-1) / T^2
        s_ddot = 60 * x * (2*x - 1) * (x - 1) / (T**2)

        # s''' = (360x^2 - 360x + 60) / T^3 = 60(6x^2 - 6x + 1) / T^3
        s_dddot = 60 * (6*x**2 - 6*x + 1) / (T**3)

        return s, s_dot, s_ddot, s_dddot

    def _apply_soft_start(self, t, pos_raw, vel_raw, acc_raw, jerk_raw):
        """
        应用软启动到轨迹状态（空间混合法的严格求导）

        混合公式: p_ss(t) = p_start + s(t) * (p_raw(t) - p_start)
        严格求导如下:
        v_ss(t) = s(t) * v_raw(t) + s_dot(t) * (p_raw(t) - p_start)
        a_ss(t) = s(t) * a_raw(t) + 2*s_dot(t)*v_raw(t) + s_ddot(t)*(p_raw(t) - p_start)
        j_ss(t) = s(t) * j_raw(t) + 3*s_dot(t)*a_raw(t) + 3*s_ddot(t)*v_raw(t) + s_dddot(t)*(p_raw(t) - p_start)
        """
        s, s_dot, s_ddot, s_dddot = self._get_soft_start_derivatives(t)

        # 起始位置（轨迹在 t=0 的位置）
        if not hasattr(self, '_start_pos'):
            self._start_pos = pos_raw.copy()

        # 计算位置差
        delta_p = pos_raw - self._start_pos

        # 位置插值
        pos = self._start_pos + s * delta_p

        # 速度
        vel = s * vel_raw + s_dot * delta_p

        # 加速度
        acc = s * acc_raw + 2.0 * s_dot * vel_raw + s_ddot * delta_p

        # Jerk
        jerk = s * jerk_raw + 3.0 * s_dot * acc_raw + 3.0 * s_ddot * vel_raw + s_dddot * delta_p

        return pos, vel, acc, jerk

    # =========================================================================
    # 周期计算（统一方法）
    # =========================================================================

    def get_trajectory_period(self, traj_type):
        """
        获取轨迹周期（精确计算）

        统一公式：周期 = 轨迹长度 / 实际速度
        实际速度 = base_speed * speed_factor
        """
        v = self.base_speed * self.speed_factor  # 实际速度
        R = self.amplitude

        periods = {
            # 圆形：周长 = 2πR
            'circle': 2 * np.pi * R / v,

            # 椭圆：近似周长 = π * (3(a+b) - sqrt((3a+b)(a+3b)))
            'ellipse': self._ellipse_period(1.2, 0.8),
            'long_ellipse': self._ellipse_period(1.8, 0.6),
            'tilted_ellipse': self._ellipse_period(1.0, 0.8),

            # Figure8 (Lemniscate of Gerono): omega = v/(2R), period = 2π/omega = 4πR/v
            'figure8': 4 * np.pi * R / v,

            # 螺旋圆：一圈水平 + 上升
            'spiral_circle': 2 * np.pi * (R * 0.8) / v,

            # 锥形：平均半径
            'cone3d': 2 * np.pi * (R * 0.5) / v,

            # 正弦波：水平往返
            'sine_wave': 4 * R / v,

            # 锯齿形（Lissajous 3:4）
            'zigzag': 2 * np.pi * R / v,

            # 多项式（5航点）
            'polynomial': 25.0 / self.speed_factor,

            # 方形：周长 = 4 * side
            'square': 4 * (R * 1.5) / v,

            # 五角星：10条边
            'star': 10 * R * 0.8 / v,
        }

        return periods.get(traj_type, 2 * np.pi * R / v)

    def _ellipse_period(self, a_ratio, b_ratio):
        """计算椭圆周期（Ramanujan 近似）"""
        a = self.amplitude * a_ratio
        b = self.amplitude * b_ratio
        v = self.base_speed * self.speed_factor

        # Ramanujan 椭圆周长近似公式
        h = ((a - b) / (a + b)) ** 2
        circumference = np.pi * (a + b) * (1 + 3*h / (10 + np.sqrt(4 - 3*h)))

        return circumference / v

    def get_trajectory_info(self, traj_type):
        """获取轨迹完整信息"""
        difficulty = TrajectoryDifficulty.get_difficulty(traj_type, self.speed_factor)
        period = self.get_trajectory_period(traj_type)

        return {
            'type': traj_type,
            'period': period,
            'speed': self.base_speed * self.speed_factor,
            'amplitude': self.amplitude,
            'difficulty': difficulty,
            'total_duration': period * self.num_loops,
            'loops': self.num_loops
        }

    # =========================================================================
    # 主接口
    # =========================================================================

    def _compute_soft_start_tau(self, t):
        """
        计算软启动的等效轨迹时间 tau(t) = integral_0^t s(t') dt'

        使用 Smootherstep s(x) = 6x^5 - 15x^4 + 10x^3 (x = t/T),
        其不定积分为 T * (x^6 - 3x^5 + 2.5x^4)。

        在 t=T 时, tau(T) = T * 0.5 (只跑了半圈)，
        之后时间连续接入: tau(t) = tau(T) + (t - T) for t > T
        """
        if not self.soft_start_enabled or self.soft_start_duration <= 0:
            return t
        T = self.soft_start_duration
        if t <= 0:
            return 0.0
        if t < T:
            x = t / T
            return T * (x**6 - 3.0*x**5 + 2.5*x**4)
        else:
            # 软启动结束后，tau 连续接入
            # tau(T) = T * 0.5
            return 0.5 * T + (t - T)

    def get_trajectory(self, traj_type, t):
        """
        获取 t 时刻的轨迹状态

        软启动策略（时间缩放）：
          - 位置始终在轨迹曲线上（不做空间混合），沿弧线慢慢启动
          - 速度/加速度/jerk 通过链式法则从时间缩放因子 s(t) 推导
          - 这样 Reference 和 Actual 在起始时就在同一圆上

        Args:
            traj_type: 轨迹类型
            t: 时间 (s)

        Returns:
            (pos, vel, acc, jerk, yaw, yaw_dot): 完整微分平坦信息
        """
        # 获取原始轨迹函数
        trajectory_functions = {
            'circle': self._circle_trajectory,
            'ellipse': self._ellipse_trajectory,
            'long_ellipse': self._long_ellipse_trajectory,
            'tilted_ellipse': self._tilted_ellipse_trajectory,
            'figure8': self._figure8_trajectory,
            'spiral_circle': self._spiral_circle_trajectory,
            'cone3d': self._cone3d_trajectory,
            'sine_wave': self._sine_wave_trajectory,
            'zigzag': self._zigzag_trajectory,
            'polynomial': self._polynomial_trajectory,
            'square': self._square_trajectory,
            'star': self._star_trajectory,
        }

        func = trajectory_functions.get(traj_type, self._circle_trajectory)

        # 记录当前轨迹类型（供 set_world_offset 使用）
        self._current_traj_type = traj_type

        # =====================================================================
        # 时间缩放软启动
        # =====================================================================
        # 计算等效轨迹时间 tau(t)，使位置始终在曲线上
        if self.soft_start_enabled and t < self.soft_start_duration:
            s, s_dot, s_ddot, s_dddot = self._get_soft_start_derivatives(t)
            tau = self._compute_soft_start_tau(t)

            # 在 tau 时刻求轨迹原始值，确保位置在曲线上
            pos, vel_raw, acc_raw, jerk_raw, yaw, yaw_dot_raw = func(tau)

            # 链式法则推导各阶导数（pos 对 t 的导数 = pos 对 tau 的导数 * dtau/dt = vel_raw * s）
            vel   = vel_raw * s
            acc   = acc_raw * s**2 + vel_raw * s_dot
            jerk  = jerk_raw * s**3 + 3.0*acc_raw*s*s_dot + vel_raw*s_ddot
            yaw_dot = yaw_dot_raw * s
        else:
            # 软启动结束后，平移时间轴使轨迹连续
            if self.soft_start_enabled:
                tau = self._compute_soft_start_tau(t)
            else:
                tau = t
            pos, vel, acc, jerk, yaw, yaw_dot = func(tau)

        # 应用世界坐标偏移（使轨迹与无人机实际起点对齐）
        pos = pos + self._world_offset

        # 统一归一化 yaw 到 [-π, π]
        yaw = self._normalize_angle(yaw)

        # 限制 yaw_dot 防止过大的角速率（PID/RL 可能无法跟踪）
        MAX_YAW_RATE = 1.5  # rad/s (~86 deg/s)
        yaw_dot = np.clip(yaw_dot, -MAX_YAW_RATE, MAX_YAW_RATE)

        return pos, vel, acc, jerk, yaw, yaw_dot

    @staticmethod
    def _normalize_angle(angle):
        """将角度归一化到 [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    # =========================================================================
    # 轨迹实现 - 全部提供完整微分平坦信息
    # =========================================================================

    def _circle_trajectory(self, t):
        """
        圆形轨迹（水平）

        x = R * cos(ωt)
        y = R * sin(ωt)
        z = z_base

        完整解析导数
        """
        R = self.amplitude
        omega = (self.base_speed * self.speed_factor) / R

        c, s = np.cos(omega * t), np.sin(omega * t)

        pos = np.array([R * c, R * s, self.z_base])
        vel = np.array([-R * omega * s, R * omega * c, 0.0])
        acc = np.array([-R * omega**2 * c, -R * omega**2 * s, 0.0])
        jerk = np.array([R * omega**3 * s, -R * omega**3 * c, 0.0])

        # Yaw 切向 (归一化到 [-π, π])
        yaw = (omega * t + np.pi/2 + np.pi) % (2 * np.pi) - np.pi
        yaw_dot = omega

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _ellipse_trajectory(self, t):
        """
        椭圆轨迹

        x = a * cos(ωt)
        y = b * sin(ωt)
        z = z_base + δz * sin(2ωt)
        """
        a = self.amplitude * 1.2
        b = self.amplitude * 0.8
        avg_r = (a + b) / 2
        omega = (self.base_speed * self.speed_factor) / avg_r
        dz = 0.3  # 高度变化幅度

        c, s = np.cos(omega * t), np.sin(omega * t)
        c2, s2 = np.cos(2 * omega * t), np.sin(2 * omega * t)

        pos = np.array([a * c, b * s, self.z_base + dz * s2])
        vel = np.array([-a * omega * s, b * omega * c, 2 * dz * omega * c2])
        acc = np.array([-a * omega**2 * c, -b * omega**2 * s, -4 * dz * omega**2 * s2])
        jerk = np.array([a * omega**3 * s, -b * omega**3 * c, -8 * dz * omega**3 * c2])

        # Yaw 切向
        yaw = np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel[:2]) > 0.01 else 0.0
        v2 = vel[0]**2 + vel[1]**2
        yaw_dot = (vel[0]*acc[1] - vel[1]*acc[0]) / v2 if v2 > 0.01 else 0.0

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _long_ellipse_trajectory(self, t):
        """
        长椭圆轨迹（大离心率）

        x = a * cos(ωt), a = 1.8 * amplitude
        y = b * sin(ωt), b = 0.6 * amplitude
        """
        a = self.amplitude * 1.8
        b = self.amplitude * 0.6
        avg_r = (a + b) / 2
        omega = (self.base_speed * self.speed_factor) / avg_r

        c, s = np.cos(omega * t), np.sin(omega * t)

        pos = np.array([a * c, b * s, self.z_base])
        vel = np.array([-a * omega * s, b * omega * c, 0.0])
        acc = np.array([-a * omega**2 * c, -b * omega**2 * s, 0.0])
        jerk = np.array([a * omega**3 * s, -b * omega**3 * c, 0.0])

        yaw = np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel[:2]) > 0.01 else 0.0
        v2 = vel[0]**2 + vel[1]**2
        yaw_dot = (vel[0]*acc[1] - vel[1]*acc[0]) / v2 if v2 > 0.01 else 0.0

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _tilted_ellipse_trajectory(self, t):
        """
        3D倾斜椭圆 - XY平面椭圆绕X轴倾斜45度
        """
        a = self.amplitude * 1.0
        b = self.amplitude * 0.8
        avg_r = (a + b) / 2
        omega = (self.base_speed * self.speed_factor) / avg_r

        tilt = np.pi / 4  # 45度倾斜
        cos_t, sin_t = np.cos(tilt), np.sin(tilt)

        c, s = np.cos(omega * t), np.sin(omega * t)

        # 原始 XY 平面椭圆
        x0, y0 = a * c, b * s
        vx0, vy0 = -a * omega * s, b * omega * c
        ax0, ay0 = -a * omega**2 * c, -b * omega**2 * s
        jx0, jy0 = a * omega**3 * s, -b * omega**3 * c

        # 绕 X 轴旋转
        pos = np.array([x0, y0 * cos_t, self.z_base + y0 * sin_t])
        vel = np.array([vx0, vy0 * cos_t, vy0 * sin_t])
        acc = np.array([ax0, ay0 * cos_t, ay0 * sin_t])
        jerk = np.array([jx0, jy0 * cos_t, jy0 * sin_t])

        yaw = np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel[:2]) > 0.01 else 0.0
        v2 = vel[0]**2 + vel[1]**2
        yaw_dot = (vel[0]*acc[1] - vel[1]*acc[0]) / v2 if v2 > 0.01 else 0.0

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _figure8_trajectory(self, t):
        """
        8字形轨迹（Lemniscate of Gerono）

        x = a * sin(ωt)
        y = a/2 * sin(2ωt)
        z = z_base + h * sin(2ωt)
        
        Yaw 策略：使用 atan2 计算切线方向，但在交叉点
        (ωt≈0, π) 附近平滑过渡以避免 ±π 跳变。
        """
        scale = self.amplitude
        omega = (self.base_speed * self.speed_factor) / scale * 0.5  # 调整为合理速度
        h = 0.5  # 高度变化

        c, s = np.cos(omega * t), np.sin(omega * t)
        c2, s2 = np.cos(2 * omega * t), np.sin(2 * omega * t)

        pos = np.array([
            scale * s,
            scale * 0.5 * s2,
            self.z_base + h * s2
        ])

        vel = np.array([
            scale * omega * c,
            scale * omega * c2,
            h * 2 * omega * c2
        ])

        acc = np.array([
            -scale * omega**2 * s,
            -2 * scale * omega**2 * s2,
            -4 * h * omega**2 * s2
        ])

        jerk = np.array([
            -scale * omega**3 * c,
            -4 * scale * omega**3 * c2,
            -8 * h * omega**3 * c2
        ])

        # 使用解析 yaw：基于 ωt 的相位直接计算，避免 arctan2 跳变
        # vx = scale*ω*cos(ωt), vy = scale*ω*cos(2ωt)
        # 用 atan2(cos(2θ), cos(θ)) 会在 θ=π/2, 3π/2 时因 cos(θ)=0 发生跳变
        # 改用解析公式：yaw = atan2(cos(2θ), cos(θ))
        # 但确保连续性：当 cos(θ) 过零时平滑处理
        v2 = vel[0]**2 + vel[1]**2
        if v2 > 0.001:
            yaw = np.arctan2(vel[1], vel[0])
        else:
            yaw = 0.0
        
        # yaw_dot 解析计算
        yaw_dot = (vel[0]*acc[1] - vel[1]*acc[0]) / v2 if v2 > 0.001 else 0.0
        
        # 限制 yaw_dot 防止过大的角速率（PID/RL 无法跟踪）
        max_yaw_rate = 1.5  # rad/s (~86 deg/s)
        yaw_dot = np.clip(yaw_dot, -max_yaw_rate, max_yaw_rate)

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _spiral_circle_trajectory(self, t):
        """
        3D螺旋圆 - 圆周运动 + 连续上升下降

        使用正弦波控制高度，保证平滑
        """
        R = self.amplitude * 0.8
        omega = (self.base_speed * self.speed_factor) / R

        z_min, z_max = 1.0, 4.0
        z_center = (z_min + z_max) / 2
        z_amp = (z_max - z_min) / 2

        # 高度变化周期 = 5个水平圈
        omega_z = omega / 5

        c, s = np.cos(omega * t), np.sin(omega * t)
        cz, sz = np.cos(omega_z * t), np.sin(omega_z * t)

        pos = np.array([R * c, R * s, z_center + z_amp * sz])

        vel = np.array([
            -R * omega * s,
            R * omega * c,
            z_amp * omega_z * cz
        ])

        acc = np.array([
            -R * omega**2 * c,
            -R * omega**2 * s,
            -z_amp * omega_z**2 * sz
        ])

        jerk = np.array([
            R * omega**3 * s,
            -R * omega**3 * c,
            -z_amp * omega_z**3 * cz
        ])

        yaw = (omega * t + np.pi/2 + np.pi) % (2 * np.pi) - np.pi
        yaw_dot = omega

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _cone3d_trajectory(self, t):
        """
        3D锥形轨迹 - 半径随高度变化的螺旋

        R(t) = R_max * (1 - z_frac)
        z = z_base + z_height * phase
        """
        R_max = self.amplitude
        omega = (self.base_speed * self.speed_factor) / R_max * 0.7
        z_base = 1.0
        z_height = 3.0

        # 相位 (0-1 循环)
        period = 2 * np.pi / omega
        phase = (t % period) / period

        # 当前半径和高度
        R = R_max * (1 - phase)
        z = z_base + z_height * phase

        # 半径变化率
        R_dot = -R_max / period
        z_dot = z_height / period

        c, s = np.cos(omega * t), np.sin(omega * t)

        pos = np.array([R * c, R * s, z])

        vel = np.array([
            R_dot * c - R * omega * s,
            R_dot * s + R * omega * c,
            z_dot
        ])

        acc = np.array([
            -2 * R_dot * omega * s - R * omega**2 * c,
            2 * R_dot * omega * c - R * omega**2 * s,
            0.0
        ])

        # Jerk (简化)
        jerk = np.array([
            -3 * R_dot * omega**2 * c + R * omega**3 * s,
            -3 * R_dot * omega**2 * s - R * omega**3 * c,
            0.0
        ])

        yaw = np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel[:2]) > 0.01 else 0.0
        v2 = vel[0]**2 + vel[1]**2
        yaw_dot = (vel[0]*acc[1] - vel[1]*acc[0]) / v2 if v2 > 0.01 else 0.0

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _sine_wave_trajectory(self, t):
        """
        正弦波轨迹 - X方向往复 + Z方向波动

        x = A * sin(ωt)
        y = 0
        z = z_base + δz * sin(ω_z * t)
        """
        A = self.amplitude
        omega = (self.base_speed * self.speed_factor) / A * 0.5
        omega_z = omega * 0.5
        dz = 0.5

        c, s = np.cos(omega * t), np.sin(omega * t)
        cz, sz = np.cos(omega_z * t), np.sin(omega_z * t)

        pos = np.array([A * s, 0.0, self.z_base + dz * sz])
        vel = np.array([A * omega * c, 0.0, dz * omega_z * cz])
        acc = np.array([-A * omega**2 * s, 0.0, -dz * omega_z**2 * sz])
        jerk = np.array([-A * omega**3 * c, 0.0, -dz * omega_z**3 * cz])

        # Yaw 保持 0 或根据速度方向
        yaw = 0.0 if vel[0] >= 0 else np.pi
        yaw_dot = 0.0

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _zigzag_trajectory(self, t):
        """
        锯齿形轨迹 (Lissajous 3:4)

        使用三角波近似，拐点处不可微
        为了安全，提供平滑的近似导数
        """
        f_x, f_y = 3.0, 4.0
        omega = (self.base_speed * self.speed_factor) / self.amplitude

        # 使用高阶傅里叶近似三角波（更平滑）
        def smooth_triangle(phi, n_terms=5):
            """平滑三角波（傅里叶级数）"""
            result = 0.0
            for k in range(n_terms):
                n = 2 * k + 1
                result += ((-1)**k / n**2) * np.sin(n * phi)
            return result * (8 / np.pi**2)

        def smooth_triangle_deriv(phi, freq, n_terms=5):
            """平滑三角波导数"""
            result = 0.0
            for k in range(n_terms):
                n = 2 * k + 1
                result += ((-1)**k / n) * np.cos(n * phi)
            return result * (8 / np.pi**2) * freq

        phi_x = f_x * omega * t
        phi_y = f_y * omega * t

        pos = np.array([
            self.amplitude * smooth_triangle(phi_x),
            self.amplitude * smooth_triangle(phi_y),
            self.z_base
        ])

        vel = np.array([
            self.amplitude * smooth_triangle_deriv(phi_x, f_x * omega),
            self.amplitude * smooth_triangle_deriv(phi_y, f_y * omega),
            0.0
        ])

        # 加速度和 Jerk 近似为 0（不可微轨迹）
        acc = np.zeros(3)
        jerk = np.zeros(3)

        yaw = np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel[:2]) > 0.01 else 0.0
        yaw_dot = 0.0

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _polynomial_trajectory(self, t):
        """
        多项式轨迹 (5阶 Minimum Jerk)

        连接预设航点的平滑曲线
        """
        scale = self.amplitude / 2.0
        waypoints = np.array([
            [0, 0, 2.5],
            [2, 2, 3.0],
            [-2, 2, 2.0],
            [-2, -2, 3.0],
            [2, -2, 2.0],
            [0, 0, 2.5]
        ]) * np.array([scale, scale, 1.0])

        n_seg = len(waypoints) - 1
        total_t = 25.0 / self.speed_factor
        seg_t = total_t / n_seg

        # 循环轨迹
        t_mod = t % total_t
        seg_idx = min(int(t_mod / seg_t), n_seg - 1)
        local_t = t_mod - seg_idx * seg_t
        u = min(local_t / seg_t, 1.0)

        p0, p1 = waypoints[seg_idx], waypoints[(seg_idx + 1) % len(waypoints)]
        diff = p1 - p0

        # Minimum Jerk: s = 10u³ - 15u⁴ + 6u⁵
        s = 10*u**3 - 15*u**4 + 6*u**5
        s_dot = (30*u**2 - 60*u**3 + 30*u**4) / seg_t
        s_ddot = (60*u - 180*u**2 + 120*u**3) / seg_t**2
        s_dddot = (60 - 360*u + 360*u**2) / seg_t**3

        pos = p0 + diff * s
        vel = diff * s_dot
        acc = diff * s_ddot
        jerk = diff * s_dddot

        yaw = np.arctan2(vel[1], vel[0]) if np.linalg.norm(vel[:2]) > 0.01 else 0.0
        v2 = vel[0]**2 + vel[1]**2
        yaw_dot = (vel[0]*acc[1] - vel[1]*acc[0]) / v2 if v2 > 0.01 else 0.0

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _square_trajectory(self, t):
        """
        方形轨迹

        不可微轨迹，在拐角处提供近似导数
        """
        side = self.amplitude * 1.5
        speed = self.base_speed * self.speed_factor
        perimeter = 4 * side

        dist = (t * speed) % perimeter

        # 确定当前边和位置
        edge = int(dist // side) % 4
        edge_pos = dist % side

        # 四条边的方向向量
        directions = [
            np.array([1, 0, 0]),   # 右
            np.array([0, -1, 0]),  # 下
            np.array([-1, 0, 0]),  # 左
            np.array([0, 1, 0])    # 上
        ]

        # 四个角点
        corners = [
            np.array([-side/2, side/2, self.z_base]),
            np.array([side/2, side/2, self.z_base]),
            np.array([side/2, -side/2, self.z_base]),
            np.array([-side/2, -side/2, self.z_base])
        ]

        pos = corners[edge] + directions[edge] * edge_pos
        vel = directions[edge] * speed
        acc = np.zeros(3)  # 直线段加速度为 0
        jerk = np.zeros(3)

        yaw = np.arctan2(vel[1], vel[0])
        yaw_dot = 0.0

        return pos, vel, acc, jerk, yaw, yaw_dot

    def _star_trajectory(self, t):
        """
        五角星轨迹

        10个顶点（5外+5内），不可微轨迹
        """
        R_out = self.amplitude
        R_in = self.amplitude * 0.4
        speed = self.base_speed * self.speed_factor

        # 计算所有顶点
        vertices = []
        for i in range(5):
            # 外顶点
            angle_out = np.pi/2 + i * 2*np.pi/5
            vertices.append(np.array([R_out * np.cos(angle_out), R_out * np.sin(angle_out), self.z_base]))
            # 内顶点
            angle_in = np.pi/2 + np.pi/5 + i * 2*np.pi/5
            vertices.append(np.array([R_in * np.cos(angle_in), R_in * np.sin(angle_in), self.z_base]))

        # 计算总周长
        total_len = 0
        edge_lengths = []
        for i in range(10):
            edge_len = np.linalg.norm(vertices[(i+1) % 10] - vertices[i])
            edge_lengths.append(edge_len)
            total_len += edge_len

        # 当前距离
        dist = (t * speed) % total_len

        # 找到当前边
        cumsum = 0
        for i, edge_len in enumerate(edge_lengths):
            if cumsum + edge_len > dist:
                # 在第 i 条边上
                edge_frac = (dist - cumsum) / edge_len
                p0, p1 = vertices[i], vertices[(i+1) % 10]

                pos = p0 + (p1 - p0) * edge_frac
                direction = (p1 - p0) / edge_len
                vel = direction * speed
                acc = np.zeros(3)
                jerk = np.zeros(3)

                yaw = np.arctan2(vel[1], vel[0])
                yaw_dot = 0.0

                return pos, vel, acc, jerk, yaw, yaw_dot
            cumsum += edge_len

        # 默认返回起点
        return vertices[0], np.zeros(3), np.zeros(3), np.zeros(3), 0.0, 0.0


class WaypointTrajectory:
    """航点轨迹生成器"""

    def __init__(self, waypoints, frame_id='world'):
        self.waypoints = [np.array(w) for w in waypoints]
        self.frame_id = frame_id
        self.current_waypoint = 0
        self.hold_time = 5.0
        self.transition_speed = 0.5

    def set_hold_time(self, hold_time):
        self.hold_time = hold_time

    def set_transition_speed(self, speed):
        self.transition_speed = speed

    def get_waypoint(self, t):
        if not self.waypoints:
            return np.array([0.0, 0.0, 2.5]), np.zeros(3), np.zeros(3)

        total_cycle_time = len(self.waypoints) * self.hold_time
        cycle_t = t % total_cycle_time
        waypoint_idx = int(cycle_t / self.hold_time) % len(self.waypoints)

        position = self.waypoints[waypoint_idx].copy()
        velocity = np.zeros(3)
        acceleration = np.zeros(3)

        return position, velocity, acceleration


if __name__ == '__main__':
    """测试所有轨迹类型"""
    import rospy

    try:
        rospy.init_node('trajectory_test', anonymous=True)
    except:
        pass

    traj = DifferentialFlatTrajectory()
    traj.set_params(speed_factor=1.0, amplitude=2.0)

    print("=" * 70)
    print("轨迹生成器测试 v2.0")
    print("=" * 70)

    all_types = [
        'circle', 'ellipse', 'long_ellipse', 'tilted_ellipse',
        'figure8', 'spiral_circle', 'cone3d', 'sine_wave',
        'zigzag', 'polynomial', 'square', 'star'
    ]

    for traj_type in all_types:
        info = traj.get_trajectory_info(traj_type)
        pos, vel, acc, jerk, yaw, yaw_dot = traj.get_trajectory(traj_type, 0.0)

        print(f"\n{traj_type.upper()}:")
        print(f"  周期: {info['period']:.2f}s")
        print(f"  难度: {info['difficulty']['name']} (Level {info['difficulty']['level']})")
        print(f"  可微: {'是' if info['difficulty']['is_differentiable'] else '否'}")
        print(f"  初始位置: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")
        print(f"  Jerk范数: {np.linalg.norm(jerk):.4f}")

    print("\n" + "=" * 70)
    print("测试完成")
    print("=" * 70)
