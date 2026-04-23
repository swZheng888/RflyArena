# -*- coding: utf-8 -*-
"""
trajectory.py
轨迹生成器：输出 ENU 坐标系下的期望位置、速度、加速度和姿态四元数。
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


class TrajectoryGenerator:
    """
    简单轨迹生成器，输出 ENU 坐标系下的期望位置、速度、加速度和姿态。

    支持的轨迹类型：
        - circle  : 圆形轨迹（水平面）
        - line    : 直线往返
        - figure8 : 8 字形轨迹
        - spiral  : 螺旋上升

    坐标系约定：
        - 位置 / 速度 / 加速度：ENU（x 正东，y 正北，z 正上）
        - 姿态四元数：ENU / FLU，顺序 [w, x, y, z]
          其中欧拉角采用 ZYX 旋转顺序（roll about X, pitch about Y, yaw about Z）。
    """
    
    def __init__(self) -> None:
        self.trajectory_start_time: float | None = None
        self.current_trajectory_type: str = "circle"  # "circle", "line", "figure8", "spiral"
        self.trajectory_speed: float = 2.0            # 轨迹线速度 (m/s)

        # 用于在速度很小的时候保持上一帧航向角，避免抖动
        self._current_yaw: float = 0.0
    def get_start_point(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        获取当前轨迹在 t=0 时的起始点（位置/速度/加速度/姿态四元数）。

        这里的位置按各轨迹公式 t=0 计算；
        速度、加速度用于确定“朝向”，但对外返回时人为设置为 0，
        表示在起点悬停等待起飞指令。
        """
        if self.current_trajectory_type == "circle":
            position, v_dir, _ = self._circle_trajectory(0.0)
        elif self.current_trajectory_type == "line":
            position, v_dir, _ = self._line_trajectory(0.0)
        elif self.current_trajectory_type == "figure8":
            position, v_dir, _ = self._figure8_trajectory(0.0)
        elif self.current_trajectory_type == "spiral":
            position, v_dir, _ = self._spiral_trajectory(0.0)
        else:
            position, v_dir, _ = self._circle_trajectory(0.0)

        # 用理论速度方向来确定 yaw，然后让速度/加速度为 0（在起点悬停）
        yaw = self._compute_yaw_from_velocity(v_dir)
        quat = self._euler_to_quaternion(0.0, 0.0, yaw)

        velocity = np.zeros(3)
        acceleration = np.zeros(3)

        return position, velocity, acceleration, quat
    # ------------------------------------------------------------------
    # 外部接口
    # ------------------------------------------------------------------

    def set_trajectory_start_time(self, start_time: float) -> None:
        """设置轨迹起始时间（秒）。"""
        self.trajectory_start_time = start_time

    def get_trajectory_point(
        self, current_time: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        根据当前时间获取轨迹点。

        Parameters
        ----------
        current_time : float
            当前时间（秒），建议用 ROS 的 wall time / sim time。

        Returns
        -------
        position : np.ndarray, shape (3,)
            期望位置 [x, y, z] (ENU)。
        velocity : np.ndarray, shape (3,)
            期望速度 [vx, vy, vz] (ENU)。
        acceleration : np.ndarray, shape (3,)
            期望加速度 [ax, ay, az] (ENU)。
        quat : np.ndarray, shape (4,)
            期望姿态四元数 [w, x, y, z]，ENU / FLU。
        """
        if self.trajectory_start_time is None:
            # 还没开始计时：在该轨迹 t=0 的起点悬停
            position, velocity, acceleration, quat = self.get_start_point()
            return position, velocity, acceleration, quat

        t = current_time - self.trajectory_start_time

        if self.current_trajectory_type == "circle":
            position, velocity, acceleration = self._circle_trajectory(t)
        elif self.current_trajectory_type == "line":
            position, velocity, acceleration = self._line_trajectory(t)
        elif self.current_trajectory_type == "figure8":
            position, velocity, acceleration = self._figure8_trajectory(t)
        elif self.current_trajectory_type == "spiral":
            position, velocity, acceleration = self._spiral_trajectory(t)
        else:
            # 默认回退到圆形轨迹
            position, velocity, acceleration = self._circle_trajectory(t)

        # 根据速度方向更新 yaw，并生成姿态四元数
        yaw = self._compute_yaw_from_velocity(velocity)
        quat = self._euler_to_quaternion(0.0, 0.0, yaw)

        return position, velocity, acceleration, quat

    # ------------------------------------------------------------------
    # 各类轨迹（位置 / 速度 / 加速度）
    # ------------------------------------------------------------------

    def _circle_trajectory(
        self, t: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        水平圆形轨迹：
            x = r cos(ωt)
            y = r sin(ωt)
            z = constant
        """
        radius = 1.0
        center = np.array([0.0, 0.0, radius])
        omega = self.trajectory_speed / radius  # 角速度

        # 位置
        x = radius * math.cos(omega * t)
        y = radius * math.sin(omega * t)
        z = radius
        position = center + np.array([x, y, z - center[2]])

        # 速度
        vx = -radius * omega * math.sin(omega * t)
        vy = radius * omega * math.cos(omega * t)
        vz = 0.0
        velocity = np.array([vx, vy, vz])

        # 加速度
        ax = -radius * (omega ** 2) * math.cos(omega * t)
        ay = -radius * (omega ** 2) * math.sin(omega * t)
        az = 0.0
        acceleration = np.array([ax, ay, az])

        return position, velocity, acceleration

    def _line_trajectory(
        self, t: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        直线往返轨迹：从 start 到 end，之后保持在 end 不动（加速度为 0）。
        """
        start_point = np.array([-2.0, 0.0, 2.5])
        end_point = np.array([2.0, 0.0, 2.5])
        length = np.linalg.norm(end_point - start_point)

        if t * self.trajectory_speed <= length:
            # 匀速运动阶段
            s = t * self.trajectory_speed / length
            position = start_point + s * (end_point - start_point)
            velocity = (end_point - start_point) / length * self.trajectory_speed
            acceleration = np.zeros(3)
        else:
            # 到达终点后停留
            position = end_point
            velocity = np.zeros(3)
            acceleration = np.zeros(3)

        return position, velocity, acceleration

    def _figure8_trajectory(
        self, t: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        8 字形轨迹：
            x = a sin(ωt)
            y = b sin(2ωt)
        """
        center = np.array([0.0, 0.0, 5])
        a, b = 6.0, 3.0
        omega = self.trajectory_speed

        # 位置
        x = a * math.sin(omega * t)
        y = b * math.sin(2.0 * omega * t)
        z = 5
        position = center + np.array([x, y, z - center[2]])

        # 速度
        vx = a * omega * math.cos(omega * t)
        vy = 2.0 * b * omega * math.cos(2.0 * omega * t)
        vz = 0.0
        velocity = np.array([vx, vy, vz])

        # 加速度
        ax = -a * (omega ** 2) * math.sin(omega * t)
        ay = -4.0 * b * (omega ** 2) * math.sin(2.0 * omega * t)
        az = 0.0
        acceleration = np.array([ax, ay, az])

        return position, velocity, acceleration

    def _spiral_trajectory(
        self, t: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        螺旋上升轨迹：
            x = r cos(ωt)
            y = r sin(ωt)
            z = z0 + v_ascent * t
        """
        center = np.array([0.0, 0.0, 2.5])
        radius = 1.5
        ascent_speed = 0.2  # m/s
        omega = self.trajectory_speed

        # 位置
        x = radius * math.cos(omega * t)
        y = radius * math.sin(omega * t)
        z = center[2] + ascent_speed * t
        position = center + np.array([x, y, z - center[2]])

        # 速度
        vx = -radius * omega * math.sin(omega * t)
        vy = radius * omega * math.cos(omega * t)
        vz = ascent_speed
        velocity = np.array([vx, vy, vz])

        # 加速度（z 方向为匀速上升，az = 0）
        ax = -radius * (omega ** 2) * math.cos(omega * t)
        ay = -radius * (omega ** 2) * math.sin(omega * t)
        az = 0.0
        acceleration = np.array([ax, ay, az])

        return position, velocity, acceleration

    # ------------------------------------------------------------------
    # 姿态相关工具函数
    # ------------------------------------------------------------------

    def _compute_yaw_from_velocity(self, velocity: np.ndarray) -> float:
        """
        根据水平速度方向计算 yaw（rad），并在速度过小时保持上一帧 yaw。

        仅使用 vx, vy 来决定航向角，vz 不参与航向计算。
        """
        vx, vy = float(velocity[0]), float(velocity[1])
        speed_xy = math.hypot(vx, vy)

        if speed_xy < 1e-3:
            # 如果水平速度太小，则保持当前 yaw，避免抖动
            return self._current_yaw

        yaw = math.atan2(vy, vx)  # ENU: x 前，y 左，z 上
        self._current_yaw = yaw
        return yaw

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        将欧拉角 (roll, pitch, yaw) 转为四元数 [w, x, y, z]。

        按 ZYX 顺序（yaw-pitch-roll）：
            R = Rz(yaw) * Ry(pitch) * Rx(roll)
        """
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return np.array([w, x, y, z], dtype=float)
