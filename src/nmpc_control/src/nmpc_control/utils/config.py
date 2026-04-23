# -*- coding: utf-8 -*-
"""
config.py
全局参数与配置。
"""
# 物理参数
GRAVITY = 9.8        # 重力加速度 m/s^2
MASS = 0.319          # 飞行器质量 kg
#简易模型ATBT为油门与拉力关系 AMBM为油门与转矩
At=3.8619
Bt=-0.7873

Am=0.0387
Bm=-0.0073
# 电机 / 螺旋桨参数
CT = 1.105e-05       # 推力系数 (N/rad/s^2)
CD = 1.779e-07       # 反扭矩系数 (N/rad/s^2)
# 电机角速度与油门线性映射：w = CR * throttle + WB
CR = 1148.0 #rad/s
WB = -141.4 #rad/s
# 最大电机转速（与 CT / CD 的单位要匹配）
MAX_SPEED = 1006.6
MAX_THRUST = 1.0
MAX_WX = 4
MAX_WY = 4
MAX_WZ = 4
#转动惯量
Ixx = 4.603658832003199e-04  # around x-axis (roll)
Iyy = 5.356539710941770e-04  # around y-axis (pitch)
Izz = 8.290048007402470e-04  # around z-axis (yaw)

#轴距
dq  = 0.15      # [m] distance between motors' center
l   = dq/2       # [m] distance between motors' center and the axis of rotation
# 推导出的最大推力与最大反扭矩（单个电机）
# HOV_THRUST =0.38
# MAX_THRUST = MAX_SPEED ** 2 * CT
# MAX_TORQUE = MAX_SPEED ** 2 * CD
# 是否使用 PX4MavCtrlV4 的真值状态（而不是 PX4 VehicleOdometry）
USE_TRUE_STATE_FROM_MAVCTRL = True
# CopterSim 相关配置
START_COPTER_ID = 1
