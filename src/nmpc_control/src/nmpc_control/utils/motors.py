# -*- coding: utf-8 -*-
"""
motors.py
推力 / 油门相关工具函数。
"""

from __future__ import annotations

import numpy as np

from utils.config import GRAVITY, MASS, CT, CD, CR, WB, MAX_SPEED, MAX_THRUST


def calc_motor_force(krpm: float) -> float:
    """
    根据电机转速（krpm）计算单个电机推力（N）。
    """
    return CT * krpm ** 2


def calc_motor_input(krpm: float) -> float:
    """
    将电机转速（krpm）转换为归一化油门 [0, 1]。
    使用 推力 / MAX_THRUST 的比例进行归一化。
    """
    if krpm > MAX_SPEED:
        krpm = MAX_SPEED
    elif krpm < 0.0:
        krpm = 0.0

    force = calc_motor_force(krpm)
    throttle = force / MAX_THRUST

    if throttle > 1.0:
        throttle = 1.0
    elif throttle < 0.0:
        throttle = 0.0

    return float(throttle)


def calc_thrust(rad: float) -> float:
    """
    将电机角速度 rad 映射为归一化油门 [0, 1]。

    使用线性映射：
        w = CR * throttle + WB
    =>  throttle = (w - WB) / CR
    """
    throttle = (rad - WB) / CR
    throttle = float(np.clip(throttle, 0.0, 1.0))
    return throttle
