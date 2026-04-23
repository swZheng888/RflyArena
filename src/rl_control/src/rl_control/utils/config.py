# -*- coding: utf-8 -*-
"""
rl_control 物理参数配置
与 IsaacLab 训练环境 QuadcopterTrajectoryEnvCfg 完全对齐
"""

# 物理参数
GRAVITY = 9.8     # [m/s^2]  (对齐训练 env)
MASS    = 0.319   # [kg]

# 推力模型: 单电机推力 T = At * throttle + Bt
# 总推力 F = 4 * T
At = 3.8619   # [N/throttle]
Bt = -0.7873  # [N]

# 转动惯量 [kg.m^2] (实测值，对齐训练 env)
Ixx = 4.603658832003199e-04  # around x-axis (roll)
Iyy = 5.356539710941770e-04  # around y-axis (pitch)
Izz = 8.290048007402470e-04  # around z-axis (yaw)

# 角速度上限 (对齐训练 env 和 QGC 参数) [rad/s]
MAX_ROLL_RATE  = 3.84  # 220 deg/s (MC_ROLLRATE_MAX)
MAX_PITCH_RATE = 3.84  # 220 deg/s (MC_PITCHRATE_MAX)
MAX_YAW_RATE   = 3.49  # 200 deg/s (MC_YAWRATE_MAX)

# RL 动作: throttle = hover_throttle ± THRUST_RANGE
THRUST_RANGE = 0.15

# 计算悬停油门 hover_throttle = (mass*g/4 - Bt) / At
HOVER_THROTTLE = (MASS * GRAVITY / 4.0 - Bt) / At
