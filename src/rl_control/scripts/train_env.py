# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Quadcopter Trajectory Tracking Environment.

This environment trains a policy to track differentially flat trajectories
using a PX4-compatible control interface (Throttle + Angular Velocity).

Supported trajectory types: circle, figure8, spiral, line
"""

from __future__ import annotations

import math
import torch
import gymnasium as gym

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.envs.ui import BaseEnvWindow
from isaaclab.markers import VisualizationMarkers
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import subtract_frame_transforms, quat_mul, quat_conjugate, quat_rotate_inverse

# Pre-defined configs
from isaaclab_assets import CRAZYFLIE_CFG
from isaaclab.markers import CUBOID_MARKER_CFG, FRAME_MARKER_CFG
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


# ============================================================================
# Torch-based Trajectory Generator (Batched)
# ============================================================================

@torch.jit.script
def generate_circle_trajectory(
    t: torch.Tensor,
    radius: torch.Tensor,
    omega: torch.Tensor,
    height: torch.Tensor,
    center_xy: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Generate circle trajectory for multiple environments.
    When omega=0, generates a hover trajectory at the center point.

    Args:
        t: Time for each environment. Shape: (num_envs,)
        radius: Radius for each environment. Shape: (num_envs,)
        omega: Angular velocity for each environment. Shape: (num_envs,)
        height: Height for each environment. Shape: (num_envs,)
        center_xy: Center position xy for each environment. Shape: (num_envs, 2)

    Returns:
        pos: Position (num_envs, 3)
        vel: Velocity (num_envs, 3)
        acc: Acceleration (num_envs, 3)
        yaw: Yaw angle (num_envs,)
        yaw_rate: Yaw rate (num_envs,)
    """
    num_envs = t.shape[0]
    device = t.device

    # 判断是否为悬停模式 (omega ≈ 0)
    is_hover = torch.abs(omega) < 1e-6

    theta = omega * t

    # Position: 悬停时在圆心，否则在圆上
    pos = torch.zeros(num_envs, 3, device=device)
    pos[:, 0] = torch.where(is_hover, center_xy[:, 0], center_xy[:, 0] + radius * torch.cos(theta))
    pos[:, 1] = torch.where(is_hover, center_xy[:, 1], center_xy[:, 1] + radius * torch.sin(theta))
    pos[:, 2] = height

    # Velocity: 悬停时为零
    vel = torch.zeros(num_envs, 3, device=device)
    vel[:, 0] = torch.where(is_hover, torch.zeros_like(omega), -radius * omega * torch.sin(theta))
    vel[:, 1] = torch.where(is_hover, torch.zeros_like(omega), radius * omega * torch.cos(theta))
    vel[:, 2] = 0.0

    # Acceleration: 悬停时为零
    acc = torch.zeros(num_envs, 3, device=device)
    acc[:, 0] = torch.where(is_hover, torch.zeros_like(omega), -radius * omega * omega * torch.cos(theta))
    acc[:, 1] = torch.where(is_hover, torch.zeros_like(omega), -radius * omega * omega * torch.sin(theta))
    acc[:, 2] = 0.0

    # Yaw: 悬停时为0，否则跟随速度方向
    yaw = torch.where(is_hover, torch.zeros_like(omega), torch.atan2(vel[:, 1], vel[:, 0]))
    yaw_rate = torch.where(is_hover, torch.zeros_like(omega), omega)

    return pos, vel, acc, yaw, yaw_rate


@torch.jit.script
def generate_figure8_trajectory(
    t: torch.Tensor,
    a: torch.Tensor,
    b: torch.Tensor,
    omega: torch.Tensor,
    height: torch.Tensor,
    center_xy: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Generate figure-8 trajectory: x = a*sin(wt), y = b*sin(2wt)
    """
    num_envs = t.shape[0]
    device = t.device

    wt = omega * t
    wt2 = 2.0 * omega * t

    # Position
    pos = torch.zeros(num_envs, 3, device=device)
    pos[:, 0] = center_xy[:, 0] + a * torch.sin(wt)
    pos[:, 1] = center_xy[:, 1] + b * torch.sin(wt2)
    pos[:, 2] = height

    # Velocity
    vel = torch.zeros(num_envs, 3, device=device)
    vel[:, 0] = a * omega * torch.cos(wt)
    vel[:, 1] = 2.0 * b * omega * torch.cos(wt2)
    vel[:, 2] = 0.0

    # Acceleration
    acc = torch.zeros(num_envs, 3, device=device)
    acc[:, 0] = -a * omega * omega * torch.sin(wt)
    acc[:, 1] = -4.0 * b * omega * omega * torch.sin(wt2)
    acc[:, 2] = 0.0

    # Yaw follows velocity direction
    speed_xy = torch.hypot(vel[:, 0], vel[:, 1])
    yaw = torch.where(
        speed_xy > 1e-3,
        torch.atan2(vel[:, 1], vel[:, 0]),
        torch.zeros_like(speed_xy)
    )

    # Yaw rate (numerical derivative approximation)
    yaw_rate = omega  # Simplified

    return pos, vel, acc, yaw, yaw_rate


@torch.jit.script
def generate_spiral_trajectory(
    t: torch.Tensor,
    radius: torch.Tensor,
    omega: torch.Tensor,
    ascent_rate: torch.Tensor,
    base_height: torch.Tensor,
    center_xy: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Generate spiral ascending trajectory.
    """
    num_envs = t.shape[0]
    device = t.device

    theta = omega * t

    # Position
    pos = torch.zeros(num_envs, 3, device=device)
    pos[:, 0] = center_xy[:, 0] + radius * torch.cos(theta)
    pos[:, 1] = center_xy[:, 1] + radius * torch.sin(theta)
    pos[:, 2] = base_height + ascent_rate * t

    # Velocity
    vel = torch.zeros(num_envs, 3, device=device)
    vel[:, 0] = -radius * omega * torch.sin(theta)
    vel[:, 1] = radius * omega * torch.cos(theta)
    vel[:, 2] = ascent_rate

    # Acceleration
    acc = torch.zeros(num_envs, 3, device=device)
    acc[:, 0] = -radius * omega * omega * torch.cos(theta)
    acc[:, 1] = -radius * omega * omega * torch.sin(theta)
    acc[:, 2] = 0.0

    # Yaw follows horizontal velocity direction
    yaw = torch.atan2(vel[:, 1], vel[:, 0])
    yaw_rate = omega

    return pos, vel, acc, yaw, yaw_rate


@torch.jit.script
def generate_line_trajectory(
    t: torch.Tensor,
    start: torch.Tensor,
    end: torch.Tensor,
    speed: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Generate straight line trajectory from start to end.
    """
    num_envs = t.shape[0]
    device = t.device

    direction = end - start  # (num_envs, 3)
    length = torch.linalg.norm(direction, dim=1, keepdim=True)  # (num_envs, 1)
    unit_dir = direction / (length + 1e-6)  # (num_envs, 3)

    # Distance traveled
    dist = speed.unsqueeze(1) * t.unsqueeze(1)  # (num_envs, 1)

    # Check if reached end
    reached = dist >= length

    # Position
    pos = torch.where(
        reached,
        end,
        start + unit_dir * dist
    )

    # Velocity
    vel = torch.where(
        reached,
        torch.zeros(num_envs, 3, device=device),
        unit_dir * speed.unsqueeze(1)
    )

    # Acceleration (zero for constant velocity)
    acc = torch.zeros(num_envs, 3, device=device)

    # Yaw follows direction
    yaw = torch.atan2(unit_dir[:, 1], unit_dir[:, 0])
    yaw_rate = torch.zeros(num_envs, device=device)

    return pos, vel, acc, yaw, yaw_rate


@torch.jit.script
def generate_poly_waypoint_trajectory(
    t: torch.Tensor,
    waypoints: torch.Tensor,
    segment_duration: torch.Tensor,
    num_waypoints: int,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """
    Generate smooth polynomial trajectory through random waypoints.
    Uses cubic Hermite interpolation for C1 continuity (pos + vel连续).
    速度在航点处渐变以保证动力学可行性。

    Args:
        t: Time for each environment. Shape: (num_envs,)
        waypoints: (num_envs, num_waypoints, 3) xyz positions
        segment_duration: (num_envs,) time per segment
        num_waypoints: number of waypoints

    Returns: pos, vel, acc, yaw, yaw_rate  (all world frame)
    """
    num_envs = t.shape[0]
    device = t.device

    # 总时长 = (num_waypoints - 1) * segment_duration
    total_time = (num_waypoints - 1) * segment_duration
    # 循环时间（到终点后重新开始）
    t_wrapped = torch.fmod(t, total_time + 1e-6)

    # 当前在哪个 segment
    seg_float = t_wrapped / (segment_duration + 1e-6)
    seg_idx = seg_float.long().clamp(0, num_waypoints - 2)  # (num_envs,)
    s = seg_float - seg_idx.float()  # [0, 1) 段内归一化时间

    # 取当前和下一航点
    batch_idx = torch.arange(num_envs, device=device)
    p0 = waypoints[batch_idx, seg_idx]          # (num_envs, 3)
    p1 = waypoints[batch_idx, (seg_idx + 1).clamp(max=num_waypoints-1)]  # (num_envs, 3)

    # 估算切线方向（用相邻航点差分）
    seg_prev = (seg_idx - 1).clamp(min=0)
    seg_next2 = (seg_idx + 2).clamp(max=num_waypoints-1)
    p_prev = waypoints[batch_idx, seg_prev]
    p_next2 = waypoints[batch_idx, seg_next2]

    # Catmull-Rom 切线
    m0 = 0.5 * (p1 - p_prev)   # 起点切线
    m1 = 0.5 * (p_next2 - p0)  # 终点切线

    # Hermite 插值: h(s) = (2s³-3s²+1)p0 + (s³-2s²+s)m0 + (-2s³+3s²)p1 + (s³-s²)m1
    s2 = s * s
    s3 = s2 * s
    s_u = s.unsqueeze(1)   # (num_envs, 1)
    s2_u = s2.unsqueeze(1)
    s3_u = s3.unsqueeze(1)

    h00 = 2*s3_u - 3*s2_u + 1
    h10 = s3_u - 2*s2_u + s_u
    h01 = -2*s3_u + 3*s2_u
    h11 = s3_u - s2_u

    pos = h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1

    # 速度: dh/dt = (dh/ds) / segment_duration
    dh00 = 6*s2_u - 6*s_u
    dh10 = 3*s2_u - 4*s_u + 1
    dh01 = -6*s2_u + 6*s_u
    dh11 = 3*s2_u - 2*s_u

    inv_T = 1.0 / (segment_duration.unsqueeze(1) + 1e-6)
    vel = (dh00 * p0 + dh10 * m0 + dh01 * p1 + dh11 * m1) * inv_T

    # 加速度: d²h/dt²
    ddh00 = 12*s_u - 6
    ddh10 = 6*s_u - 4
    ddh01 = -12*s_u + 6
    ddh11 = 6*s_u - 2

    inv_T2 = inv_T * inv_T
    acc = (ddh00 * p0 + ddh10 * m0 + ddh01 * p1 + ddh11 * m1) * inv_T2

    # Yaw follows velocity
    speed_xy = torch.hypot(vel[:, 0], vel[:, 1])
    yaw = torch.where(speed_xy > 1e-3,
                      torch.atan2(vel[:, 1], vel[:, 0]),
                      torch.zeros_like(speed_xy))

    # Yaw rate (simplified)
    yaw_rate = torch.zeros(num_envs, device=device)

    return pos, vel, acc, yaw, yaw_rate


# ============================================================================
# Environment Window
# ============================================================================

class QuadcopterTrajectoryEnvWindow(BaseEnvWindow):
    """Window manager for the Quadcopter Trajectory Tracking environment."""

    def __init__(self, env: QuadcopterTrajectoryEnv, window_name: str = "IsaacLab"):
        super().__init__(env, window_name)
        with self.ui_window_elements["main_vstack"]:
            with self.ui_window_elements["debug_frame"]:
                with self.ui_window_elements["debug_vstack"]:
                    self._create_debug_vis_ui_element("targets", self.env)


# ============================================================================
# Environment Configuration
# ============================================================================

@configclass
class QuadcopterTrajectoryEnvCfg(DirectRLEnvCfg):
    """Configuration for the Quadcopter Trajectory Tracking environment.

    Action space (4D):
        - action[0]: Throttle offset [-1, 1] -> hover ± thrust_range
        - action[1]: Roll rate command [-1, 1] -> scaled to max_roll_rate
        - action[2]: Pitch rate command [-1, 1] -> scaled to max_pitch_rate
        - action[3]: Yaw rate command [-1, 1] -> scaled to max_yaw_rate

    Observation space (22D):
        - vel_b (3): Body frame velocity
        - ang_vel_b (3): Body frame angular velocity
        - g_b (3): Projected gravity in body frame
        - pos_err_b (3): Position error in body frame
        - vel_err_b (3): Velocity error in body frame
        - acc_ref_b (3): Reference acceleration in body frame (feedforward)
        - yaw_err (1): Yaw error
        - yaw_rate_ref (1): Reference yaw rate
        - throttle_ref (1): Reference throttle (hover-normalized)
        - last_throttle (1): Last throttle action
    """

    # Environment
    episode_length_s = 60.0  # 上限: 用于 max_episode_length 计算
    randomize_episode_length = True
    episode_length_range = (15.0, 60.0)  # [短, 长] 秒
    decimation = 2  # 100Hz control frequency (sim.dt=1/200, policy runs every 2 physics steps)
    action_space = 4
    observation_space = 25  # +3: pos_integral_b (I-term 防漂移)
    state_space = 0
    debug_vis = True

    ui_window_class_type = QuadcopterTrajectoryEnvWindow

    # Simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 200,  # 200Hz physics — 更快响应，减小控制延迟
        render_interval=2,  # 仍以 100Hz 渲染，不影响可视化帧率
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
    )
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # Scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096, env_spacing=2.5, replicate_physics=True
    )

    # Robot
    robot: ArticulationCfg = CRAZYFLIE_CFG.replace(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Bitcraze/Crazyflie/cf2x.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=10.0,
                enable_gyroscopic_forces=True,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.319),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
            ),
            copy_from_source=False,
        ),
    )

    # Physical parameters (from real drone)
    drone_mass = 0.319  # kg
    gravity = 9.8  # m/s^2
    num_motors = 4

    # ========== 实测单电机线性模型 ==========
    # T_single = max(0, At * throttle + Bt)  [N]
    # M_single = max(0, Am * throttle + Bm)  [N·m]
    motor_thrust_slope = 3.8619       # At
    motor_thrust_intercept = -0.7873  # Bt
    motor_torque_slope = 0.0387       # Am (备用，yaw仍用PD)
    motor_torque_intercept = -0.0073  # Bm (备用)

    # Inertia moments [kg.m^2] - real drone (from RFlySim ModelParam_uavJ**)
    Ixx = 4.603658832003199e-04  # around x-axis (roll)
    Iyy = 5.356539710941770e-04  # around y-axis (pitch)
    Izz = 8.290048007402470e-04  # around z-axis (yaw)

    # Angular velocity limits — 对齐 QGC 参数 (MC_ROLLRATE_MAX / MC_YAWRATE_MAX)
    max_roll_rate  = 3.84  # rad/s  = 220 deg/s (MC_ROLLRATE_MAX)
    max_pitch_rate = 3.84  # rad/s  = 220 deg/s (MC_PITCHRATE_MAX)
    max_yaw_rate   = 3.49  # rad/s  = 200 deg/s (MC_YAWRATE_MAX)

    # PID gains for angular velocity control
    # 对齐 PX4 有效响应速度：
    #   PX4 PID 输出归一化力矩，实际角加速度取决于电机力矩能力
    #   训练 PID 直接输出 Nm，角加速 = P/Ixx
    #   原值 P=0.05 → 109 rad/s²/error，响应极快，与 PX4 实际响应不匹配
    #   新值 P=0.025 → 54 rad/s²/error，更接近 PX4 真实响应
    #   PX4 Pitch P(0.080) > Roll P(0.063)，比例 1.27，训练中 pitch 略大
    ang_vel_p_gain  = [0.025,  0.032,  0.120 ]  # Nm/(rad/s) — Yaw P 提高对齐 PX4 (Yaw_P/Roll_P=4.8x)
    ang_vel_i_gain  = [0.050,  0.050,  0.016 ]  # 对齐 PX4: yaw_I/yaw_P 比 roll_I/roll_P
    ang_vel_d_gain  = [1.5e-4, 1.5e-4, 0.0   ]  # Nm/(rad/s²)
    ang_vel_i_limit = [0.05,   0.05,   0.06  ]  # 积分限幅
    max_torque      = [0.10,   0.10,   0.03  ]  # N·m  (降低以匹配更小的 PID 增益)

    # 油门范围：与定点env一致的 ±15% 小范围映射
    # action[0]∈[-1,1] → throttle = hover_throttle ± thrust_range
    # 原全范围映射：随机动作能达最大推力，导致无人机灬射上升
    thrust_range = 0.3   # 对齐 RflySim 实测最佳映射 (0.3 时 Err 低至 0.003m)

    # ========== Trajectory Configuration ==========
    trajectory_type: str = "circle"  # circle, figure8, spiral, line, poly
    trajectory_speed: float = 1.0    # 默认单环境速度
    trajectory_radius: float = 1.5   # m
    trajectory_height: float = 1.0   # m

    # Figure-8 specific
    figure8_a: float = 2.0  # x amplitude
    figure8_b: float = 1.0  # y amplitude

    # Spiral specific
    spiral_ascent_rate: float = 0.2  # m/s vertical speed

    # Line specific
    line_length: float = 4.0  # m

    # Polynomial waypoint specific
    poly_num_waypoints: int = 6       # 航点数量
    poly_waypoint_spread: float = 3.0 # 航点在中心周围的散布半径 (m)，扩大配合大半径轨迹
    poly_segment_time_range: tuple = (1.0, 3.0)  # 每段时间范围 (s), 越短=越快

    # 轨迹随机化：每个 env 得到不同速度/半径，提升泛化能力
    randomize_trajectory: bool = True
    trajectory_radius_range: tuple = (0.8, 5.0)   # m  (扩大: 允许大半径配合高速)
    trajectory_height_range: tuple = (0.5, 2.5)   # m
    trajectory_speed_range:  tuple = (0.5, 5.0)   # m/s (提高上限到 5m/s)
    # 动力学可行性: v ≤ sqrt(max_accel * R)
    # v=5m/s, R=3m → accel=8.3m/s², R=5m → accel=5m/s² ← 均可行
    # v=5m/s, R=2m → accel=12.5m/s² ← 被限制住
    max_centripetal_accel: float = 12.0  # m/s² ≈ 1.2g

    # 轨迹类型随机化：每个 env 每个 episode 随机选一种轨迹类型
    # 类型编号: 0=circle, 1=figure8, 2=spiral, 3=poly, 4=hover
    randomize_trajectory_type: bool = True
    # 采样权重 [circle, figure8, spiral, poly, hover]
    # hover 从 0.10 提高到 0.20：保证策略充分学习静态悬停
    trajectory_type_weights: list = [0.28, 0.22, 0.13, 0.17, 0.20]

    # ========== Reward Scales ==========
    pos_tracking_reward_scale  = 25.0
    vel_tracking_reward_scale  = 8.0
    yaw_tracking_reward_scale  = 6.0
    ang_vel_reward_scale       = -0.3
    action_rate_reward_scale   = -2.0    # 全维度动作跳变惩罚（负值不能被懒策略利用）
    throttle_rate_reward_scale = -3.0    # 专项抑制油门跳变

    # 持续悬停奖励
    sustained_hover_threshold = 0.2
    sustained_hover_steps     = 100
    sustained_hover_reward_scale = 5.0

    # 奖励形状 sigma
    pos_sigma  = 0.3    # 适中: 30cm时71%奖励
    vel_sigma  = 0.3    # 适中: 静止奖励最高
    yaw_sigma  = 0.3

    # ========== Domain Randomization (参考 rl-vs-gc 项目) ==========
    # dr_dict: 值 > 0 表示启用，值为 uniform 随机化半宽
    # 例如 'mass': 0.1 → mass * uniform(0.9, 1.1)
    # 注意：初始值全部为 0，由 curriculum 逐步开启！
    dr_dict = {
        'mass':           0.0,
        'inertia':        0.0,
        'thrust':         0.0,
        'motor_lag':      0.3,   # ±30% 电机时间常数随机化 → 抗真实电机响应差异
        'pid_gains':      0.5,   # ±50%，策略对 PID 增益变化更鲁棒
    }

    # 传统 flag 保留兼容（初始关闭，curriculum 会开启）
    randomize_mass = False
    randomize_thrust = False
    mass_scale_range = [0.8, 1.3]
    thrust_scale_range = [0.8, 1.3]
    add_action_delay = True
    action_delay_range = [1, 3]       # 10-30ms @ 100Hz

    # Observation delay randomization
    add_observation_delay = True
    observation_delay_range = [1, 3]  # 10-30ms @ 100Hz

    # ========== Observation Noise ==========
    add_observation_noise = True
    vel_noise_std          = 0.10   # m/s   (↑ 加大)
    ang_vel_noise_std      = 0.05   # rad/s (↑ 加大)
    gravity_proj_noise_std = 0.02            # (↑ 加大)

    # ========== Wind Disturbance ==========
    add_wind_disturbance = False
    wind_force_max  = 0.15   # N (原 0.3，减半，约占重量 ~5%)
    wind_change_prob = 0.01

    # ========== Curriculum Learning (压缩到 5000 轮内完成) ==========
    enable_curriculum = True
    # 每轮步数 = 4096 envs × 24 steps = 98304 steps/iter
    # Stage 1:  500 iters →  ~49M steps
    curriculum_stage1_steps =  49_000_000
    # Stage 2: 1500 iters → ~148M steps
    curriculum_stage2_steps = 148_000_000
    # Stage 3: 2500 iters → ~246M steps
    curriculum_stage3_steps = 246_000_000
    # Stage 4: 4000 iters → ~393M steps
    curriculum_stage4_steps = 393_000_000

    # ========== Motor Dynamics (First-order lag) ==========
    # 真实电机不可能瞬时响应，典型无刷电机响应时间 30-60ms
    add_motor_lag = True
    motor_time_constant = 0.04            # 40ms 基准
    motor_time_constant_range = [0.04, 0.15]  # 随机化范围: 20-80ms 加大
    randomize_motor_time_constant = True   # 开启: 每次 reset 随机抽取时间常数

    # 角速度指令平滑：防止角速度指令跳变导致力矩抖动
    # 100Hz 下 dt=0.01s
    ang_vel_cmd_time_constant = 0.02     # 20ms 角速度指令滤波（对齐 200Hz 的 alpha≈0.20）


# ============================================================================
# Environment Class
# ============================================================================

class QuadcopterTrajectoryEnv(DirectRLEnv):
    """Quadcopter environment for trajectory tracking.

    This environment trains a policy to track differentially flat trajectories
    using throttle + angular velocity commands (PX4 RATE mode compatible).
    """

    cfg: QuadcopterTrajectoryEnvCfg

    def __init__(self, cfg: QuadcopterTrajectoryEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        # Actions and control
        self._actions = torch.zeros(self.num_envs, 4, device=self.device)
        self._prev_actions = torch.zeros(self.num_envs, 4, device=self.device)
        self._thrust = torch.zeros(self.num_envs, 1, 3, device=self.device)
        self._moment = torch.zeros(self.num_envs, 1, 3, device=self.device)
        self._prev_ang_vel_error = torch.zeros(self.num_envs, 3, device=self.device)

        # 位置积分项：累计位置误差（I-term），用于抵消常値漂移
        self._pos_integral = torch.zeros(self.num_envs, 3, device=self.device)

        # PID gains
        self._ang_vel_p_gain = torch.tensor(self.cfg.ang_vel_p_gain, device=self.device)
        self._ang_vel_i_gain = torch.tensor(self.cfg.ang_vel_i_gain, device=self.device)
        self._ang_vel_d_gain = torch.tensor(self.cfg.ang_vel_d_gain, device=self.device)
        self._ang_vel_i_limit = torch.tensor(self.cfg.ang_vel_i_limit, device=self.device)
        self._max_torque = torch.tensor(self.cfg.max_torque, device=self.device)
        self._ang_vel_integral = torch.zeros(self.num_envs, 3, device=self.device)
        self._max_ang_vel = torch.tensor(
            [self.cfg.max_roll_rate, self.cfg.max_pitch_rate, self.cfg.max_yaw_rate],
            device=self.device,
        )

        # Drone inertia tensor (for gyroscopic compensation, same as PX4 env)
        self._inertia = torch.tensor([self.cfg.Ixx, self.cfg.Iyy, self.cfg.Izz], device=self.device)

        # Trajectory state
        self._trajectory_time = torch.zeros(self.num_envs, device=self.device)
        self._desired_pos = torch.zeros(self.num_envs, 3, device=self.device)
        self._desired_vel = torch.zeros(self.num_envs, 3, device=self.device)
        self._desired_acc = torch.zeros(self.num_envs, 3, device=self.device)
        self._desired_yaw = torch.zeros(self.num_envs, device=self.device)
        self._desired_yaw_rate = torch.zeros(self.num_envs, device=self.device)

        # Per-environment trajectory parameters
        self._traj_radius = torch.ones(self.num_envs, device=self.device) * self.cfg.trajectory_radius
        self._traj_height = torch.ones(self.num_envs, device=self.device) * self.cfg.trajectory_height
        self._traj_speed = torch.ones(self.num_envs, device=self.device) * self.cfg.trajectory_speed
        self._traj_omega = self._traj_speed / self._traj_radius
        self._traj_center_xy = torch.zeros(self.num_envs, 2, device=self.device)

        # Per-env trajectory type: 0=circle, 1=figure8, 2=spiral, 3=poly, 4=hover
        self._traj_type_idx = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        # Figure-8 per-env amplitudes
        self._fig8_a = torch.ones(self.num_envs, device=self.device) * self.cfg.figure8_a
        self._fig8_b = torch.ones(self.num_envs, device=self.device) * self.cfg.figure8_b

        # Spiral per-env ascent rate
        self._spiral_ascent = torch.ones(self.num_envs, device=self.device) * self.cfg.spiral_ascent_rate

        # Polynomial waypoint trajectory buffers
        nwp = self.cfg.poly_num_waypoints
        self._poly_waypoints = torch.zeros(self.num_envs, nwp, 3, device=self.device)
        self._poly_seg_dur = torch.ones(self.num_envs, device=self.device) * 2.0

        # Line trajectory endpoints
        self._line_start = torch.zeros(self.num_envs, 3, device=self.device)
        self._line_end = torch.zeros(self.num_envs, 3, device=self.device)

        # Domain randomization
        self._mass_scale = torch.ones(self.num_envs, device=self.device)
        self._thrust_scale = torch.ones(self.num_envs, device=self.device)

        # Per-env PID gain scale (对齐 rl-vs-gc kp_att/kd_att 随机化)
        self._pid_gain_scale = torch.ones(self.num_envs, device=self.device)

        # Wind disturbance (世界系随机力)
        self._wind_force = torch.zeros(self.num_envs, 1, 3, device=self.device)

        # Action delay buffer
        max_delay = self.cfg.action_delay_range[1] + 1
        self._action_buffer = torch.zeros(self.num_envs, max_delay, 4, device=self.device)
        self._action_delay = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._action_buffer_idx = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        # Observation delay buffer
        max_obs_delay = self.cfg.observation_delay_range[1] + 1
        self._obs_buffer = torch.zeros(self.num_envs, max_obs_delay, self.cfg.observation_space, device=self.device)
        self._obs_delay = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self._obs_buffer_idx = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        # Motor dynamics (first-order lag)
        self._filtered_thrust = torch.zeros(self.num_envs, device=self.device)
        self._motor_time_constant = torch.ones(self.num_envs, device=self.device) * self.cfg.motor_time_constant
        # 角速度指令滤波状态
        self._filtered_ang_vel_cmd = torch.zeros(self.num_envs, 3, device=self.device)

        # Robot properties
        self._body_id = self._robot.find_bodies("body")[0]
        self._robot_mass = self._robot.root_physx_view.get_masses()[0].sum()
        self._gravity_magnitude = torch.tensor(self.sim.cfg.gravity, device=self.device).norm()
        self._robot_weight = (self._robot_mass * self._gravity_magnitude).item()

        # 修正质量和转动悯量（比例缩放，防止碰撞冲量爆炸）
        self._fix_drone_mass()
        # 重新读取修正后质量
        self._robot_mass = self._robot.root_physx_view.get_masses()[0].sum()
        self._robot_weight = (self._robot_mass * self._gravity_magnitude).item()

        # 悬停油门: num_motors * (At * hover_throttle + Bt) = mass * g
        # hover_throttle = (mass * g / num_motors - Bt) / At
        self._hover_throttle = (
            (self._robot_weight / self.cfg.num_motors - self.cfg.motor_thrust_intercept)
            / self.cfg.motor_thrust_slope
        )

        # Logging
        self._episode_sums = {
            key: torch.zeros(self.num_envs, dtype=torch.float, device=self.device)
            for key in ["pos_tracking", "vel_tracking", "yaw_tracking",
                        "ang_vel", "action_rate", "throttle_rate", "sustained_hover"]
        }

        # 持续悬停计数器：连续多少步 pos_err < threshold
        self._sustained_hover_count = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)

        # Visualization
        self.set_debug_vis(self.cfg.debug_vis)

        # ========== Curriculum Learning State ==========
        self._total_steps = 0
        self._current_curriculum_stage = 0

    def _update_curriculum(self):
        """按总步数自动逐步开启域随机化（4 阶段，更平滑）。

        Stage 0: 无随机化 — 纯学习轨迹跟踪
        Stage 1 (~1500 iters): 质量 ±5% + 推力 ±5% (小幅度开始)
        Stage 2 (~4000 iters): 质量/推力 加到 ±10% + 观测噪声
        Stage 3 (~7000 iters): + 动作延迟 + 观测延迟 + PID ±10%
        Stage 4 (~10000 iters): 全开：+ 风 + 惯量 ±10% + 电机延迟 ±30% + PID ±15%
        """
        if not self.cfg.enable_curriculum:
            return

        old_stage = self._current_curriculum_stage

        if self._total_steps >= getattr(self.cfg, 'curriculum_stage4_steps', float('inf')):
            self._current_curriculum_stage = 4
        elif self._total_steps >= self.cfg.curriculum_stage3_steps:
            self._current_curriculum_stage = 3
        elif self._total_steps >= self.cfg.curriculum_stage2_steps:
            self._current_curriculum_stage = 2
        elif self._total_steps >= self.cfg.curriculum_stage1_steps:
            self._current_curriculum_stage = 1
        else:
            self._current_curriculum_stage = 0

        if self._current_curriculum_stage == old_stage:
            return

        stage = self._current_curriculum_stage

        # Stage 1: 小幅质量 + 推力
        if stage >= 1:
            self.cfg.dr_dict['mass'] = 0.05
            self.cfg.dr_dict['thrust'] = 0.05
            self.cfg.randomize_mass = True
            self.cfg.randomize_thrust = True

        # Stage 2: 加大质量/推力 + 观测噪声 + 收紧sigma + 加大油门惩罚
        if stage >= 2:
            self.cfg.dr_dict['mass'] = 0.1
            self.cfg.dr_dict['thrust'] = 0.1
            self.cfg.add_observation_noise = True
            self.cfg.pos_sigma = 0.2
            self.cfg.vel_sigma = 0.3   # 放宽
            self.cfg.yaw_sigma = 0.3   # 放宽
            self.cfg.throttle_rate_reward_scale = -4.0   # 加大油门平滑惩罚

        # Stage 3: 延迟 + PID + 精确模式 + 更强油门惩罚
        if stage >= 3:
            self.cfg.add_action_delay = True
            self.cfg.add_observation_delay = True
            self.cfg.dr_dict['pid_gains'] = max(self.cfg.dr_dict['pid_gains'], 0.1)
            self.cfg.pos_sigma = 0.12
            self.cfg.vel_sigma = 0.3   # 放宽
            self.cfg.yaw_sigma = 0.3   # 放宽
            self.cfg.throttle_rate_reward_scale = -5.0   # 极致平滑
            self.cfg.action_rate_reward_scale   = -3.0

        # Stage 4: 全开 + 超精确
        if stage >= 4:
            self.cfg.add_wind_disturbance = True
            self.cfg.dr_dict['pid_gains'] = max(self.cfg.dr_dict['pid_gains'], 0.15)
            self.cfg.dr_dict['inertia'] = max(self.cfg.dr_dict['inertia'], 0.1)
            self.cfg.dr_dict['motor_lag'] = max(self.cfg.dr_dict['motor_lag'], 0.3)
            self.cfg.randomize_motor_time_constant = True
            self.cfg.pos_sigma = 0.08   # 8cm→45%: 近零误差压力
            self.cfg.vel_sigma = 0.3    # 放宽
            self.cfg.yaw_sigma = 0.3    # 放宽

        print(f"\n{'='*60}")
        print(f"[Curriculum] Stage {old_stage} → {stage}  (step {self._total_steps:,})")
        print(f"  pos/vel/yaw sigma: {self.cfg.pos_sigma}/{self.cfg.vel_sigma}/{self.cfg.yaw_sigma}")
        print(f"  mass:      ±{self.cfg.dr_dict['mass']*100:.0f}%")
        print(f"  thrust:    ±{self.cfg.dr_dict['thrust']*100:.0f}%")
        print(f"  inertia:   ±{self.cfg.dr_dict['inertia']*100:.0f}%")
        print(f"  motor_lag: ±{self.cfg.dr_dict['motor_lag']*100:.0f}%")
        print(f"  pid_gains: ±{self.cfg.dr_dict['pid_gains']*100:.0f}%")
        print(f"  obs_noise:     {self.cfg.add_observation_noise}")
        print(f"  action_delay:  {self.cfg.add_action_delay}")
        print(f"  obs_delay:     {self.cfg.add_observation_delay}")
        print(f"  wind:          {self.cfg.add_wind_disturbance}")
        print(f"{'='*60}\n")


    def _setup_scene(self):
        self._robot = Articulation(self.cfg.robot)
        self.scene.articulations["robot"] = self._robot

        self.cfg.terrain.num_envs = self.scene.cfg.num_envs
        self.cfg.terrain.env_spacing = self.scene.cfg.env_spacing
        self._terrain = self.cfg.terrain.class_type(self.cfg.terrain)

        # 对齐 PX4 env：clone 前䆌修 SDK USD 质量，子 link 设为0
        # 这样 clone 出的所有 env 都继承修正后质量，避免内昷8（prop质量大）
        self._patch_link_masses_usd()

        self.scene.clone_environments(copy_from_source=False)
        if self.device == "cpu":
            self.scene.filter_collisions(global_prim_paths=[self.cfg.terrain.prim_path])

        light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
        light_cfg.func("/World/Light", light_cfg)

    def _patch_link_masses_usd(self):
        """对齐 PX4 env: clone 前在 USD Stage 上把 body→drone_mass, 其余子 link→0。"""
        try:
            from pxr import UsdPhysics, Usd
            import omni.usd
        except ImportError:
            print("[WARN] pxr/omni.usd 不可用，跳过质量修补")
            return

        stage = omni.usd.get_context().get_stage()
        source_path = "/World/envs/env_0/Robot"
        source_prim = stage.GetPrimAtPath(source_path)
        if not source_prim.IsValid():
            print(f"[WARN] 找不到源 prim: {source_path}")
            return

        info = []
        for prim in Usd.PrimRange(source_prim):
            if prim.HasAPI(UsdPhysics.MassAPI):
                mass_api = UsdPhysics.MassAPI(prim)
                name = prim.GetName()
                if name == "body":
                    mass_api.GetMassAttr().Set(float(self.cfg.drone_mass))
                    info.append(f"{name}={self.cfg.drone_mass:.4f}kg")
                else:
                    mass_api.GetMassAttr().Set(0.0)
                    info.append(f"{name}=0")
        if info:
            print(f"[INFO] USD 质量修補: {', '.join(info)}")

    def _update_trajectory(self):
        """Update desired trajectory state for current time.
        
        支持 per-env 不同轨迹类型的混合训练。
        类型编号: 0=circle, 1=figure8, 2=spiral, 3=poly, 4=hover
        """
        t = self._trajectory_time

        # 初始化输出
        pos = torch.zeros(self.num_envs, 3, device=self.device)
        vel = torch.zeros(self.num_envs, 3, device=self.device)
        acc = torch.zeros(self.num_envs, 3, device=self.device)
        yaw = torch.zeros(self.num_envs, device=self.device)
        yaw_rate = torch.zeros(self.num_envs, device=self.device)

        if not self.cfg.randomize_trajectory_type:
            # 固定轨迹类型（向后兼容）
            if self.cfg.trajectory_type == "circle":
                pos, vel, acc, yaw, yaw_rate = generate_circle_trajectory(
                    t, self._traj_radius, self._traj_omega, self._traj_height, self._traj_center_xy)
            elif self.cfg.trajectory_type == "figure8":
                pos, vel, acc, yaw, yaw_rate = generate_figure8_trajectory(
                    t, self._fig8_a, self._fig8_b, self._traj_omega, self._traj_height, self._traj_center_xy)
            elif self.cfg.trajectory_type == "spiral":
                pos, vel, acc, yaw, yaw_rate = generate_spiral_trajectory(
                    t, self._traj_radius, self._traj_omega, self._spiral_ascent, self._traj_height, self._traj_center_xy)
            elif self.cfg.trajectory_type == "poly":
                pos, vel, acc, yaw, yaw_rate = generate_poly_waypoint_trajectory(
                    t, self._poly_waypoints, self._poly_seg_dur, self.cfg.poly_num_waypoints)
            elif self.cfg.trajectory_type == "line":
                pos, vel, acc, yaw, yaw_rate = generate_line_trajectory(
                    t, self._line_start, self._line_end, self._traj_speed)
            else:
                pos, vel, acc, yaw, yaw_rate = generate_circle_trajectory(
                    t, self._traj_radius, self._traj_omega, self._traj_height, self._traj_center_xy)
        else:
            # ── Per-env 混合轨迹 ────────────────────────────────────────
            # 为每种类型生成完整结果（对所有 env），然后用 mask 选取
            # 这样避免了 scatter 和 for 循环，保持 GPU 并行效率

            # 0: circle
            p0, v0, a0, y0, yr0 = generate_circle_trajectory(
                t, self._traj_radius, self._traj_omega, self._traj_height, self._traj_center_xy)

            # 1: figure8
            p1, v1, a1, y1, yr1 = generate_figure8_trajectory(
                t, self._fig8_a, self._fig8_b, self._traj_omega, self._traj_height, self._traj_center_xy)

            # 2: spiral
            p2, v2, a2, y2, yr2 = generate_spiral_trajectory(
                t, self._traj_radius, self._traj_omega, self._spiral_ascent, self._traj_height, self._traj_center_xy)

            # 3: poly waypoint
            p3, v3, a3, y3, yr3 = generate_poly_waypoint_trajectory(
                t, self._poly_waypoints, self._poly_seg_dur, self.cfg.poly_num_waypoints)

            # 4: hover (omega=0 circle = hover in place)
            zero_omega = torch.zeros_like(self._traj_omega)
            zero_radius = torch.zeros_like(self._traj_radius)
            p4, v4, a4, y4, yr4 = generate_circle_trajectory(
                t, zero_radius, zero_omega, self._traj_height, self._traj_center_xy)

            # 按类型 mask 赋值
            for type_idx, (pi, vi, ai, yi, yri) in enumerate([
                (p0,v0,a0,y0,yr0), (p1,v1,a1,y1,yr1), (p2,v2,a2,y2,yr2),
                (p3,v3,a3,y3,yr3), (p4,v4,a4,y4,yr4),
            ]):
                mask = (self._traj_type_idx == type_idx)
                if mask.any():
                    mask_3d = mask.unsqueeze(1).expand_as(pos)
                    pos = torch.where(mask_3d, pi, pos)
                    vel = torch.where(mask_3d, vi, vel)
                    acc = torch.where(mask_3d, ai, acc)
                    yaw = torch.where(mask, yi, yaw)
                    yaw_rate = torch.where(mask, yri, yaw_rate)


        self._desired_pos = pos
        self._desired_vel = vel
        self._desired_acc = acc
        self._desired_yaw = yaw
        self._desired_yaw_rate = yaw_rate

    def _pre_physics_step(self, actions: torch.Tensor):
        """Process actions and compute thrust and moments."""
        raw_actions = actions.clone().clamp(-1.0, 1.0)

        # Action delay
        if self.cfg.add_action_delay:
            max_delay = self.cfg.action_delay_range[1] + 1
            self._action_buffer[
                torch.arange(self.num_envs, device=self.device),
                self._action_buffer_idx,
            ] = raw_actions
            delayed_idx = (self._action_buffer_idx - self._action_delay) % max_delay
            self._actions = self._action_buffer[
                torch.arange(self.num_envs, device=self.device),
                delayed_idx,
            ]
            self._action_buffer_idx = (self._action_buffer_idx + 1) % max_delay
        else:
            self._actions = raw_actions

        # Thrust computation
        # action[0] ∈ [-1, 1] → throttle = hover ± thrust_range（与定点 env 一致）
        throttle = (self._hover_throttle
                    + self._actions[:, 0] * self.cfg.thrust_range)
        throttle = torch.clamp(throttle, 0.0, 1.0)

        # 单电机推力: T = max(0, At * throttle + Bt)
        single_thrust = torch.clamp(
            self.cfg.motor_thrust_slope * throttle + self.cfg.motor_thrust_intercept,
            min=0.0,
        )

        # 总推力 = num_motors × 单电机推力 × 域随机化缩放
        thrust_cmd = self.cfg.num_motors * single_thrust * self._thrust_scale

        # ========== Wind Disturbance ==========
        if self.cfg.add_wind_disturbance:
            # 每步有 wind_change_prob 概率改变风向
            change_mask = torch.rand(self.num_envs, device=self.device) < self.cfg.wind_change_prob
            if change_mask.any():
                n_change = change_mask.sum().item()
                self._wind_force[change_mask, 0, :] = (
                    torch.rand(n_change, 3, device=self.device) * 2.0 - 1.0
                ) * self.cfg.wind_force_max

        # ========== Motor Dynamics (First-order lag) ==========
        # 一阶惯性环节: thrust[k+1] = alpha * thrust_cmd + (1-alpha) * thrust[k]
        # alpha = dt / (tau + dt)
        if self.cfg.add_motor_lag:
            alpha = self.step_dt / (self._motor_time_constant + self.step_dt)
            self._filtered_thrust = alpha * thrust_cmd + (1.0 - alpha) * self._filtered_thrust
            self._thrust[:, 0, 2] = self._filtered_thrust
        else:
            self._thrust[:, 0, 2] = thrust_cmd

        # Angular velocity PID control (对齐 PX4 Rate Controller)
        desired_ang_vel_raw = self._actions[:, 1:] * self._max_ang_vel

        # 角速度指令平滑（一阶滤波，防止瞬时跳变 → 真实世界电机无法响应）
        alpha_ang = self.step_dt / (self.cfg.ang_vel_cmd_time_constant + self.step_dt)
        self._filtered_ang_vel_cmd = alpha_ang * desired_ang_vel_raw + (1.0 - alpha_ang) * self._filtered_ang_vel_cmd
        desired_ang_vel = self._filtered_ang_vel_cmd

        current_ang_vel = self._robot.data.root_ang_vel_b
        ang_vel_error = desired_ang_vel - current_ang_vel
        # NaN 防护（对齐 PX4 env）
        ang_vel_error = torch.nan_to_num(ang_vel_error, nan=0.0, posinf=10.0, neginf=-10.0)

        # P 项 × per-env PID 增益缩放（域随机化，对齐 rl-vs-gc kp_att DR）
        p_term = ang_vel_error * self._ang_vel_p_gain * self._pid_gain_scale.unsqueeze(1)

        # I 项 (带 anti-windup)
        self._ang_vel_integral += ang_vel_error * self.step_dt
        self._ang_vel_integral = torch.clamp(
            self._ang_vel_integral, -self._ang_vel_i_limit, self._ang_vel_i_limit
        )
        i_term = self._ang_vel_integral * self._ang_vel_i_gain

        # D 项（增益为0，保留结构）
        d_term = (ang_vel_error - self._prev_ang_vel_error) / self.step_dt * self._ang_vel_d_gain
        self._prev_ang_vel_error = ang_vel_error.clone()

        # 陀螺力矩补偿: τ_gyro = ω × (I·ω)  ← 对齐 PX4 env，低角速度下也能改善稳定性
        I_omega = current_ang_vel * self._inertia  # (N,3) element-wise
        gyro = torch.cross(current_ang_vel, I_omega, dim=1)
        gyro = torch.nan_to_num(gyro, nan=0.0, posinf=0.5, neginf=-0.5)

        # 合力矩 + 限幅
        torque = p_term + i_term + d_term + gyro
        torque = torch.clamp(torque, -self._max_torque, self._max_torque)
        self._moment[:, 0, :] = torque

    def _apply_action(self):
        """Apply thrust and moment to robot."""
        # 合并推力 + 风力
        total_force = self._thrust.clone()
        if self.cfg.add_wind_disturbance:
            total_force = total_force + self._wind_force

        self._robot.permanent_wrench_composer.set_forces_and_torques(
            body_ids=self._body_id, forces=total_force, torques=self._moment
        )

    def _get_observations(self) -> dict:
        """Get observations for policy.

        Returns 22-dim observation:
            [0:3]   vel_b          - Body frame velocity
            [3:6]   ang_vel_b      - Body frame angular velocity
            [6:9]   g_b            - Projected gravity
            [9:12]  pos_err_b      - Position error in body frame
            [12:15] vel_err_b      - Velocity error in body frame
            [15:18] acc_ref_b      - Reference acceleration in body frame
            [18]    yaw_err        - Yaw error
            [19]    yaw_rate_ref   - Reference yaw rate
            [20]    throttle_ref   - Reference throttle (normalized)
            [21]    last_throttle  - Last throttle action
        """
        # Update trajectory time and reference
        self._trajectory_time += self.step_dt
        self._update_trajectory()

        # ========== Curriculum 自动开启 DR ==========
        self._total_steps += self.num_envs
        self._update_curriculum()

        # Body frame quantities
        vel_b = self._robot.data.root_lin_vel_b
        ang_vel_b = self._robot.data.root_ang_vel_b
        g_b = self._robot.data.projected_gravity_b

        # ========== Observation Noise (模拟传感器噪声) ==========
        if self.cfg.add_observation_noise:
            vel_b = vel_b + torch.randn_like(vel_b) * self.cfg.vel_noise_std
            ang_vel_b = ang_vel_b + torch.randn_like(ang_vel_b) * self.cfg.ang_vel_noise_std
            g_b = g_b + torch.randn_like(g_b) * self.cfg.gravity_proj_noise_std

        # Position error in body frame
        pos_err_w = self._desired_pos - self._robot.data.root_pos_w
        pos_err_b = quat_rotate_inverse(self._robot.data.root_quat_w, pos_err_w)


        # Velocity error in body frame
        vel_err_w = self._desired_vel - self._robot.data.root_lin_vel_w
        vel_err_b = quat_rotate_inverse(self._robot.data.root_quat_w, vel_err_w)

        # Reference acceleration in body frame
        acc_ref_b = quat_rotate_inverse(self._robot.data.root_quat_w, self._desired_acc)

        # Yaw error (wrapped to [-pi, pi])
        current_quat = self._robot.data.root_quat_w
        w, x, y, z = current_quat[:, 0], current_quat[:, 1], current_quat[:, 2], current_quat[:, 3]
        current_yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        yaw_err = self._desired_yaw - current_yaw
        yaw_err = torch.atan2(torch.sin(yaw_err), torch.cos(yaw_err))

        # Reference yaw rate
        yaw_rate_ref = self._desired_yaw_rate

        # Reference throttle (从参考加速度反推油门，基于实测电机模型)
        # total_thrust = mass * (acc_z + g)
        # single_thrust = total_thrust / num_motors
        # throttle_raw = (single_thrust - Bt) / At
        # 再反映射回 action 空间 [-1, 1]
        acc_z_ref = self._desired_acc[:, 2]
        total_thrust_ref = self._robot_mass * (acc_z_ref + self._gravity_magnitude)
        single_thrust_ref = total_thrust_ref / self.cfg.num_motors
        throttle_raw = (single_thrust_ref - self.cfg.motor_thrust_intercept) / self.cfg.motor_thrust_slope
        throttle_ref = torch.where(
            throttle_raw >= self._hover_throttle,
            (throttle_raw - self._hover_throttle) / (1.0 - self._hover_throttle),
            (throttle_raw - self._hover_throttle) / self._hover_throttle,
        )
        throttle_ref = torch.clamp(throttle_ref, -1.0, 1.0)

        # Last throttle action
        last_throttle = self._prev_actions[:, 0]

        # 位置积分项更新 (world frame, 限幅防止积分飱转)
        # 能支消常値漂移/分气漯移，类似 PID I项
        pos_err_w = self._desired_pos - self._robot.data.root_pos_w
        self._pos_integral = torch.clamp(
            self._pos_integral + pos_err_w * self.step_dt,
            -3.0, 3.0  # 限幅 ±3m，防止极大漂移导致积分饱和
        )
        # 转换到机体帧
        pos_integral_b = quat_rotate_inverse(self._robot.data.root_quat_w, self._pos_integral)

        obs = torch.cat([
            vel_b,                          # [0:3]
            ang_vel_b,                      # [3:6]
            g_b,                            # [6:9]
            pos_err_b,                      # [9:12]
            vel_err_b,                      # [12:15]
            acc_ref_b,                      # [15:18]
            yaw_err.unsqueeze(1),           # [18]
            yaw_rate_ref.unsqueeze(1),      # [19]
            throttle_ref.unsqueeze(1),      # [20]
            last_throttle.unsqueeze(1),     # [21]
            pos_integral_b,                 # [22:25] I-term
        ], dim=-1)

        # 防止观测值爆炸导致网络发散
        obs = torch.clamp(obs, -20.0, 20.0)
        obs = torch.nan_to_num(obs, nan=0.0, posinf=20.0, neginf=-20.0)

        # Store current actions for next step
        self._prev_actions = self._actions.clone()

        # ========== Observation Delay ==========
        if self.cfg.add_observation_delay:
            max_obs_delay = self.cfg.observation_delay_range[1] + 1
            # Store current observation in circular buffer
            self._obs_buffer[
                torch.arange(self.num_envs, device=self.device),
                self._obs_buffer_idx,
            ] = obs
            # Get delayed observation
            delayed_idx = (self._obs_buffer_idx - self._obs_delay) % max_obs_delay
            obs_delayed = self._obs_buffer[
                torch.arange(self.num_envs, device=self.device),
                delayed_idx,
            ]
            # Advance buffer index
            self._obs_buffer_idx = (self._obs_buffer_idx + 1) % max_obs_delay
            return {"policy": obs_delayed}

        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        """Compute trajectory tracking rewards."""
        pos_err = torch.linalg.norm(self._desired_pos - self._robot.data.root_pos_w, dim=1)

        # exp(-err²/sigma) —— sigma 越小梯度越尖锐，迫使策略追求更小误差
        pos_reward = torch.exp(-pos_err.pow(2) / self.cfg.pos_sigma)

        # Velocity tracking
        vel_err = torch.linalg.norm(self._desired_vel - self._robot.data.root_lin_vel_w, dim=1)
        vel_reward = torch.exp(-vel_err.pow(2) / self.cfg.vel_sigma)

        # Yaw tracking
        current_quat = self._robot.data.root_quat_w
        w, x, y, z = current_quat[:, 0], current_quat[:, 1], current_quat[:, 2], current_quat[:, 3]
        current_yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        yaw_err = self._desired_yaw - current_yaw
        yaw_err = torch.atan2(torch.sin(yaw_err), torch.cos(yaw_err))
        yaw_reward = torch.exp(-yaw_err.pow(2) / self.cfg.yaw_sigma)

        # Angular velocity penalty
        ang_vel_penalty = torch.sum(self._robot.data.root_ang_vel_b.pow(2), dim=1)

        # 动作变化率惩罚 (二次型，覆盖全部 4 维)
        action_rate = torch.sum((self._actions - self._prev_actions).pow(2), dim=1)
        # 油门跳变专项惩罚
        throttle_rate = (self._actions[:, 0] - self._prev_actions[:, 0]).pow(2)

        # Sustained hover bonus
        within_threshold = pos_err < self.cfg.sustained_hover_threshold
        self._sustained_hover_count = torch.where(
            within_threshold,
            self._sustained_hover_count + 1,
            torch.zeros_like(self._sustained_hover_count)
        )
        sustained_hover_bonus = (self._sustained_hover_count >= self.cfg.sustained_hover_steps).float()

        # Combine rewards
        rewards = {
            "pos_tracking":    pos_reward   * self.cfg.pos_tracking_reward_scale  * self.step_dt,
            "vel_tracking":    vel_reward   * self.cfg.vel_tracking_reward_scale  * self.step_dt,
            "yaw_tracking":    yaw_reward   * self.cfg.yaw_tracking_reward_scale  * self.step_dt,
            "ang_vel":         ang_vel_penalty * self.cfg.ang_vel_reward_scale    * self.step_dt,
            "action_rate":     action_rate  * self.cfg.action_rate_reward_scale   * self.step_dt,
            "throttle_rate":   throttle_rate * self.cfg.throttle_rate_reward_scale * self.step_dt,
            "sustained_hover": sustained_hover_bonus * self.cfg.sustained_hover_reward_scale * self.step_dt,
        }

        reward = torch.sum(torch.stack(list(rewards.values())), dim=0)
        # 防止 NaN 奖励污染 PPO rollout buffer
        reward = torch.nan_to_num(reward, nan=0.0, posinf=0.0, neginf=0.0)

        # Logging
        for key, value in rewards.items():
            self._episode_sums[key] += torch.nan_to_num(value, nan=0.0)

        return reward

    def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Check termination conditions."""
        time_out = self.episode_length_buf >= self.max_episode_length - 1

        z = self._robot.data.root_pos_w[:, 2]
        out_of_bounds = torch.logical_or(z < 0.1, z > 5.0)  # 提高上限: spiral 会持续爬升
        nan_state = torch.isnan(z) | torch.isnan(self._robot.data.root_lin_vel_b).any(dim=1)
        died = out_of_bounds | nan_state

        return died, time_out

    def _reset_idx(self, env_ids: torch.Tensor | None):
        if env_ids is None or len(env_ids) == self.num_envs:
            env_ids = self._robot._ALL_INDICES

        # Logging
        extras = dict()
        for key in self._episode_sums.keys():
            episodic_sum_avg = torch.mean(self._episode_sums[key][env_ids])
            extras["Episode_Reward/" + key] = episodic_sum_avg / self.max_episode_length_s
            self._episode_sums[key][env_ids] = 0.0
        self.extras["log"] = dict()
        self.extras["log"].update(extras)

        self._robot.reset(env_ids)
        super()._reset_idx(env_ids)

        # Reset sustained hover counter
        self._sustained_hover_count[env_ids] = 0

        if len(env_ids) == self.num_envs:
            self.episode_length_buf = torch.randint_like(
                self.episode_length_buf, high=int(self.max_episode_length)
            )

        num_reset = len(env_ids)

        # Reset trajectory time
        self._trajectory_time[env_ids] = 0.0

        # ── 轨迹类型随机化 ────────────────────────────────────────────────
        if self.cfg.randomize_trajectory_type:
            weights = torch.tensor(self.cfg.trajectory_type_weights, device=self.device)
            weights = weights / weights.sum()  # 归一化
            self._traj_type_idx[env_ids] = torch.multinomial(
                weights.expand(num_reset, -1), 1
            ).squeeze(1)
        else:
            # 固定类型映射到编号
            type_map = {"circle": 0, "figure8": 1, "spiral": 2, "poly": 3, "hover": 4, "line": 0}
            self._traj_type_idx[env_ids] = type_map.get(self.cfg.trajectory_type, 0)

        # ── 基础参数随机化（所有类型共用）─────────────────────────────────
        if self.cfg.randomize_trajectory:
            self._traj_radius[env_ids] = torch.empty(num_reset, device=self.device).uniform_(
                self.cfg.trajectory_radius_range[0], self.cfg.trajectory_radius_range[1])
            self._traj_height[env_ids] = torch.empty(num_reset, device=self.device).uniform_(
                self.cfg.trajectory_height_range[0], self.cfg.trajectory_height_range[1])
            self._traj_speed[env_ids] = torch.empty(num_reset, device=self.device).uniform_(
                self.cfg.trajectory_speed_range[0], self.cfg.trajectory_speed_range[1])
            # 向心加速度限制: v ≤ sqrt(max_accel * R)
            max_speed = torch.sqrt(self.cfg.max_centripetal_accel * self._traj_radius[env_ids])
            self._traj_speed[env_ids] = torch.min(self._traj_speed[env_ids], max_speed)
        else:
            self._traj_radius[env_ids] = self.cfg.trajectory_radius
            self._traj_height[env_ids] = self.cfg.trajectory_height
            self._traj_speed[env_ids] = self.cfg.trajectory_speed

        self._traj_omega[env_ids] = self._traj_speed[env_ids] / self._traj_radius[env_ids]
        self._traj_center_xy[env_ids] = self._terrain.env_origins[env_ids, :2]

        # ── Figure-8 参数 (type=1) ──────────────────────────────────────
        # a, b 随机化，同样受 max_centripetal_accel 约束
        # figure8 最大加速度 ≈ max(a*ω², 4*b*ω²), 限制 ω = speed/(avg_dim)
        fig8_mask = (self._traj_type_idx[env_ids] == 1)
        if fig8_mask.any():
            n_f8 = fig8_mask.sum().item()
            f8_ids = env_ids[fig8_mask]
            self._fig8_a[f8_ids] = torch.empty(n_f8, device=self.device).uniform_(0.8, 2.5)
            self._fig8_b[f8_ids] = torch.empty(n_f8, device=self.device).uniform_(0.5, 1.5)
            # figure8 的 omega 用 speed / avg_amplitude
            avg_amp = (self._fig8_a[f8_ids] + self._fig8_b[f8_ids]) / 2.0
            self._traj_omega[f8_ids] = self._traj_speed[f8_ids] / (avg_amp + 1e-6)
            # 限制加速度: max_acc ≈ 4*b*ω² ≤ max_centripetal_accel
            max_omega_f8 = torch.sqrt(self.cfg.max_centripetal_accel / (4.0 * self._fig8_b[f8_ids] + 1e-6))
            self._traj_omega[f8_ids] = torch.min(self._traj_omega[f8_ids], max_omega_f8)

        # ── Spiral 参数 (type=2) ────────────────────────────────────────
        spiral_mask = (self._traj_type_idx[env_ids] == 2)
        if spiral_mask.any():
            n_sp = spiral_mask.sum().item()
            sp_ids = env_ids[spiral_mask]
            self._spiral_ascent[sp_ids] = torch.empty(n_sp, device=self.device).uniform_(0.1, 0.4)

        # ── Poly waypoint 参数 (type=3) ─────────────────────────────────
        poly_mask = (self._traj_type_idx[env_ids] == 3)
        if poly_mask.any():
            n_poly = poly_mask.sum().item()
            poly_ids = env_ids[poly_mask]
            nwp = self.cfg.poly_num_waypoints
            spread = self.cfg.poly_waypoint_spread
            center_xy = self._traj_center_xy[poly_ids]  # (n_poly, 2)
            height = self._traj_height[poly_ids]         # (n_poly,)

            # 生成随机航点: center ± spread 的 xy, height ± 0.3m 的 z
            wp_xy = center_xy.unsqueeze(1).expand(-1, nwp, -1) + \
                    (torch.rand(n_poly, nwp, 2, device=self.device) * 2 - 1) * spread
            wp_z = height.unsqueeze(1).expand(-1, nwp) + \
                   (torch.rand(n_poly, nwp, device=self.device) * 2 - 1) * 0.3
            # 限制高度范围
            wp_z = wp_z.clamp(self.cfg.trajectory_height_range[0], self.cfg.trajectory_height_range[1])

            self._poly_waypoints[poly_ids, :, 0] = wp_xy[:, :, 0]
            self._poly_waypoints[poly_ids, :, 1] = wp_xy[:, :, 1]
            self._poly_waypoints[poly_ids, :, 2] = wp_z

            # 每段时间: 根据航点间距和 max_accel 计算安全段时间
            # T_seg ≥ sqrt(2 * max_dist / max_accel) 以保证加速度可行
            seg_time = torch.empty(n_poly, device=self.device).uniform_(
                self.cfg.poly_segment_time_range[0], self.cfg.poly_segment_time_range[1])
            self._poly_seg_dur[poly_ids] = seg_time

        # ── Hover (type=4): omega=0 即可，无需额外设置 ──────────────────
        hover_mask = (self._traj_type_idx[env_ids] == 4)
        if hover_mask.any():
            self._traj_omega[env_ids[hover_mask]] = 0.0

        # Line trajectory endpoints (向后兼容)
        line_mask = (self._traj_type_idx[env_ids] == 5) if not self.cfg.randomize_trajectory_type else torch.zeros(num_reset, dtype=torch.bool, device=self.device)
        if (not self.cfg.randomize_trajectory_type) and self.cfg.trajectory_type == "line":
            half_len = self.cfg.line_length / 2.0
            self._line_start[env_ids, 0] = self._traj_center_xy[env_ids, 0] - half_len
            self._line_start[env_ids, 1] = self._traj_center_xy[env_ids, 1]
            self._line_start[env_ids, 2] = self._traj_height[env_ids]
            self._line_end[env_ids, 0] = self._traj_center_xy[env_ids, 0] + half_len
            self._line_end[env_ids, 1] = self._traj_center_xy[env_ids, 1]
            self._line_end[env_ids, 2] = self._traj_height[env_ids]

        # ========== Domain Randomization (对齐 rl-vs-gc 风格) ==========
        self._domain_randomization(env_ids, num_reset)

        # Reset actions
        self._actions[env_ids] = 0.0
        self._prev_actions[env_ids] = 0.0
        self._prev_ang_vel_error[env_ids] = 0.0
        self._ang_vel_integral[env_ids] = 0.0
        self._pos_integral[env_ids] = 0.0  # 重置位置积分项

        # Get initial trajectory point
        self._update_trajectory()

        # Initialize robot at trajectory start
        # ========== 对齐 PX4 env 的 reset 方式 ==========
        # PX4 env: 用 USD default_root_state（z≈0.5m）+ terrain_origin，
        #          目标在 z=[0.8,1.2]m → 初始 pos_err≈0.5m → 学习信号充足！
        # 旧做法: 直接把无人机放在轨迹目标点 → pos_err=0 → 无学习信号 → 训练不收敛
        joint_pos = self._robot.data.default_joint_pos[env_ids]
        joint_vel = self._robot.data.default_joint_vel[env_ids]
        default_root_state = self._robot.data.default_root_state[env_ids].clone()

        # 使用 terrain origin 作为基准（与 PX4 env 一致）
        default_root_state[:, :3] += self._terrain.env_origins[env_ids]
        # 在目标附近随机扰动：xy±0.8m, z±0.5m（加大初始误差，提升策略鲁棒性）
        tgt = self._desired_pos[env_ids]
        xy_perturb = torch.zeros(num_reset, 2, device=self.device).uniform_(-3.0, 3.0)
        z_perturb  = torch.zeros(num_reset, device=self.device).uniform_(-1.5, 1.5)
        default_root_state[:, 0] = tgt[:, 0] + xy_perturb[:, 0]
        default_root_state[:, 1] = tgt[:, 1] + xy_perturb[:, 1]
        default_root_state[:, 2] = torch.clamp(tgt[:, 2] + z_perturb, 0.3, 3.0)

        # 姿态：小随机 yaw（无大翻滚）
        half_yaw = self._desired_yaw[env_ids] * 0.5
        default_root_state[:, 3] = torch.cos(half_yaw)  # w
        default_root_state[:, 4] = 0.0  # x
        default_root_state[:, 5] = 0.0  # y
        default_root_state[:, 6] = torch.sin(half_yaw)  # z

        # 初始速度清零（不继承轨迹速度，让无人机从静止开始学）
        default_root_state[:, 7:] = 0.0

        self._robot.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
        self._robot.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
        self._robot.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)

    def _domain_randomization(self, env_ids: torch.Tensor, num_reset: int):
        """Per-episode domain randomization (不修改 PhysX 质量/惯量！).

        关键设计：悬停油门 _hover_throttle 和观测中 throttle_ref 都是按
        原始质量计算的。如果直接改 PhysX 质量，这些参考值就全错了。
        所以用推力缩放来**等效**模拟质量变化：
            thrust_scale < 1 → 等效更重（推力不足）
            thrust_scale > 1 → 等效更轻（推力过剩）

        dr_dict 控制：
            'mass':      通过 thrust_scale 等效模拟
            'inertia':   通过 pid_gain_scale 等效模拟（惯量变化 → 角响应变化）
            'thrust':    直接乘法缩放
            'motor_lag': 缩放电机时间常数
            'pid_gains': 缩放 PID P 增益
        """
        dr = self.cfg.dr_dict

        # ── 推力随机化（含等效质量随机化） ─────────────────────────
        # 合并 mass 和 thrust 的效果到 _thrust_scale
        mass_factor = 1.0
        if dr.get('mass', 0.0) > 0:
            r = dr['mass']
            # mass 增大 → 推力等效减小，所以取倒数
            mass_rand = torch.empty(num_reset, device=self.device).uniform_(1 - r, 1 + r)
            mass_factor = 1.0 / mass_rand  # mass+5% → thrust_scale=0.952

        thrust_factor = 1.0
        if dr.get('thrust', 0.0) > 0:
            r = dr['thrust']
            thrust_factor = torch.empty(num_reset, device=self.device).uniform_(1 - r, 1 + r)
        elif self.cfg.randomize_thrust:
            thrust_factor = torch.empty(num_reset, device=self.device).uniform_(
                self.cfg.thrust_scale_range[0], self.cfg.thrust_scale_range[1]
            )

        self._thrust_scale[env_ids] = mass_factor * thrust_factor

        # ── 惯量随机化 → 等效为 PID 增益缩放 ──────────────────────
        # 惯量增大 → 角加速度变小 → 等效 PID P 增益变小
        inertia_factor = 1.0
        if dr.get('inertia', 0.0) > 0:
            r = dr['inertia']
            inertia_rand = torch.empty(num_reset, device=self.device).uniform_(1 - r, 1 + r)
            inertia_factor = 1.0 / inertia_rand  # inertia+10% → gain_scale=0.909

        pid_factor = 1.0
        if dr.get('pid_gains', 0.0) > 0:
            r = dr['pid_gains']
            pid_factor = torch.empty(num_reset, device=self.device).uniform_(1 - r, 1 + r)

        self._pid_gain_scale[env_ids] = inertia_factor * pid_factor

        # ── 电机时间常数随机化（对齐 rl-vs-gc tau_m DR） ────────────
        if self.cfg.add_motor_lag:
            self._filtered_thrust[env_ids] = 0.0
            self._filtered_ang_vel_cmd[env_ids] = 0.0
            if dr.get('motor_lag', 0.0) > 0:
                r = dr['motor_lag']
                self._motor_time_constant[env_ids] = torch.empty(num_reset, device=self.device).uniform_(
                    1 - r, 1 + r
                ) * self.cfg.motor_time_constant
            elif self.cfg.randomize_motor_time_constant:
                self._motor_time_constant[env_ids] = torch.empty(num_reset, device=self.device).uniform_(
                    self.cfg.motor_time_constant_range[0], self.cfg.motor_time_constant_range[1]
                )
            else:
                self._motor_time_constant[env_ids] = self.cfg.motor_time_constant


        # ── Action delay ────────────────────────────────────────────
        if self.cfg.add_action_delay:
            self._action_delay[env_ids] = torch.randint(
                self.cfg.action_delay_range[0],
                self.cfg.action_delay_range[1] + 1,
                (num_reset,), device=self.device,
            )
            self._action_buffer[env_ids] = 0.0
            self._action_buffer_idx[env_ids] = 0

        # ── Observation delay ───────────────────────────────────────
        if self.cfg.add_observation_delay:
            self._obs_delay[env_ids] = torch.randint(
                self.cfg.observation_delay_range[0],
                self.cfg.observation_delay_range[1] + 1,
                (num_reset,), device=self.device,
            )
            self._obs_buffer[env_ids] = 0.0
            self._obs_buffer_idx[env_ids] = 0

        # ── Wind force reset ───────────────────────────────────────
        if self.cfg.add_wind_disturbance:
            self._wind_force[env_ids, 0, :] = (
                torch.rand(num_reset, 3, device=self.device) * 2.0 - 1.0
            ) * self.cfg.wind_force_max

    def _fix_drone_mass(self):
        """等比例缩放所有链接质量，总质量 → cfg.drone_mass（0.319 kg）。

        不能把子链接质量设为 1e-6 kg：
          - 碎地时冲量 = 速度 / 质量 → 1e-6 kg 时产生天文级冲量力 → PhysX NaN 角速度
        正确方式：等比例缩放（保留碰撞物理的相对分布）。
        """
        target_mass = float(self.cfg.drone_mass)
        env_ids = torch.arange(self.num_envs, dtype=torch.int32)
        body_idx = int(self._body_id[0]) if hasattr(self._body_id, '__len__') else int(self._body_id)

        # 等比例缩放质量
        masses = self._robot.root_physx_view.get_masses().clone()
        total_original = masses[0].sum().item()
        if total_original < 1e-6:
            print("[WARN] _fix_drone_mass: 原始总质量为零，跳过")
            return
        scale = target_mass / total_original
        masses = masses * scale
        self._robot.root_physx_view.set_masses(masses.cpu(), env_ids.cpu())

        # 等比例缩放惯量，body link 覆盖为真实对角惯量（从 cfg 读取保持同步）
        Ixx, Iyy, Izz = self.cfg.Ixx, self.cfg.Iyy, self.cfg.Izz  # kg·m²
        inertias = self._robot.root_physx_view.get_inertias().clone()
        inertias = inertias * scale
        body_inertia = torch.tensor(
            [Ixx, 0., 0., 0., Iyy, 0., 0., 0., Izz], dtype=torch.float32
        )
        inertias[:, body_idx, :] = body_inertia.unsqueeze(0)
        self._robot.root_physx_view.set_inertias(inertias.cpu(), env_ids.cpu())

        new_masses = self._robot.root_physx_view.get_masses()
        total = new_masses[0].sum().item()
        print(f"[INFO] 轨迹跟踪 env: _fix_drone_mass 完成 总质量={total:.4f}kg (缩放={scale:.4f})")

    def _set_debug_vis_impl(self, debug_vis: bool):
        if debug_vis:
            # Goal position marker (green cube)
            if not hasattr(self, "goal_pos_visualizer"):
                marker_cfg = CUBOID_MARKER_CFG.copy()
                marker_cfg.markers["cuboid"].size = (0.05, 0.05, 0.05)
                marker_cfg.prim_path = "/Visuals/Command/goal_position"
                self.goal_pos_visualizer = VisualizationMarkers(marker_cfg)

            # Goal yaw orientation frame
            if not hasattr(self, "yaw_visualizer"):
                frame_cfg = FRAME_MARKER_CFG.copy()
                frame_cfg.prim_path = "/Visuals/Command/goal_yaw"
                frame_cfg.markers["frame"].scale = (0.2, 0.2, 0.2)
                self.yaw_visualizer = VisualizationMarkers(frame_cfg)

            # Robot current orientation frame
            if not hasattr(self, "robot_frame_visualizer"):
                robot_frame_cfg = FRAME_MARKER_CFG.copy()
                robot_frame_cfg.prim_path = "/Visuals/Robot/current_orientation"
                robot_frame_cfg.markers["frame"].scale = (0.15, 0.15, 0.15)
                self.robot_frame_visualizer = VisualizationMarkers(robot_frame_cfg)

            # Trajectory preview points (small spheres showing future path)
            if not hasattr(self, "traj_preview_visualizer"):
                from isaaclab.markers import VisualizationMarkersCfg
                traj_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/Trajectory/preview",
                    markers={
                        "sphere": sim_utils.SphereCfg(
                            radius=0.02,
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.8, 0.0)),
                        ),
                    },
                )
                self.traj_preview_visualizer = VisualizationMarkers(traj_cfg)
                self._traj_preview_points = 50  # Number of preview points
                self._traj_preview_dt = 0.1  # Time step between preview points

            # Robot history trail (small red spheres showing actual path)
            if not hasattr(self, "robot_history_visualizer"):
                from isaaclab.markers import VisualizationMarkersCfg
                history_cfg = VisualizationMarkersCfg(
                    prim_path="/Visuals/Robot/history",
                    markers={
                        "sphere": sim_utils.SphereCfg(
                            radius=0.015,
                            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.8, 0.2, 0.2)),
                        ),
                    },
                )
                self.robot_history_visualizer = VisualizationMarkers(history_cfg)
                self._history_length = 100  # Number of history points
                self._robot_pos_history = torch.zeros(self.num_envs, self._history_length, 3, device=self.device)
                self._history_idx = 0

            self.goal_pos_visualizer.set_visibility(True)
            self.yaw_visualizer.set_visibility(True)
            self.robot_frame_visualizer.set_visibility(True)
            self.traj_preview_visualizer.set_visibility(True)
            self.robot_history_visualizer.set_visibility(True)
        else:
            if hasattr(self, "goal_pos_visualizer"):
                self.goal_pos_visualizer.set_visibility(False)
            if hasattr(self, "yaw_visualizer"):
                self.yaw_visualizer.set_visibility(False)
            if hasattr(self, "robot_frame_visualizer"):
                self.robot_frame_visualizer.set_visibility(False)
            if hasattr(self, "traj_preview_visualizer"):
                self.traj_preview_visualizer.set_visibility(False)
            if hasattr(self, "robot_history_visualizer"):
                self.robot_history_visualizer.set_visibility(False)

    def _debug_vis_callback(self, event):
        # Visualize goal position (green cube)
        self.goal_pos_visualizer.visualize(self._desired_pos)

        # Visualize goal yaw orientation
        half_yaw = self._desired_yaw * 0.5
        goal_quat = torch.zeros(self.num_envs, 4, device=self.device)
        goal_quat[:, 0] = torch.cos(half_yaw)
        goal_quat[:, 3] = torch.sin(half_yaw)
        self.yaw_visualizer.visualize(self._desired_pos, goal_quat)

        # Visualize robot current orientation
        self.robot_frame_visualizer.visualize(
            self._robot.data.root_pos_w, self._robot.data.root_quat_w
        )

        # Update robot position history
        self._robot_pos_history[:, self._history_idx, :] = self._robot.data.root_pos_w
        self._history_idx = (self._history_idx + 1) % self._history_length

        # Visualize robot history trail (red spheres)
        history_positions = self._robot_pos_history.reshape(-1, 3)  # (num_envs * history_length, 3)
        self.robot_history_visualizer.visualize(history_positions)

        # Generate and visualize trajectory preview (green spheres)
        preview_positions = self._generate_trajectory_preview()
        self.traj_preview_visualizer.visualize(preview_positions)

    def _generate_trajectory_preview(self) -> torch.Tensor:
        """Generate future trajectory points for visualization.
        支持 per-env 不同轨迹类型。
        """
        num_points = self._traj_preview_points
        dt = self._traj_preview_dt

        future_times = self._trajectory_time.unsqueeze(1) + torch.arange(
            0, num_points, device=self.device
        ).unsqueeze(0) * dt  # (num_envs, num_points)

        all_positions = []

        for i in range(num_points):
            t = future_times[:, i]

            if not self.cfg.randomize_trajectory_type:
                # 固定类型（向后兼容）
                if self.cfg.trajectory_type == "circle":
                    pos, _, _, _, _ = generate_circle_trajectory(
                        t, self._traj_radius, self._traj_omega, self._traj_height, self._traj_center_xy)
                elif self.cfg.trajectory_type == "figure8":
                    pos, _, _, _, _ = generate_figure8_trajectory(
                        t, self._fig8_a, self._fig8_b, self._traj_omega, self._traj_height, self._traj_center_xy)
                elif self.cfg.trajectory_type == "spiral":
                    pos, _, _, _, _ = generate_spiral_trajectory(
                        t, self._traj_radius, self._traj_omega, self._spiral_ascent, self._traj_height, self._traj_center_xy)
                elif self.cfg.trajectory_type == "poly":
                    pos, _, _, _, _ = generate_poly_waypoint_trajectory(
                        t, self._poly_waypoints, self._poly_seg_dur, self.cfg.poly_num_waypoints)
                elif self.cfg.trajectory_type == "line":
                    pos, _, _, _, _ = generate_line_trajectory(
                        t, self._line_start, self._line_end, self._traj_speed)
                else:
                    pos, _, _, _, _ = generate_circle_trajectory(
                        t, self._traj_radius, self._traj_omega, self._traj_height, self._traj_center_xy)
            else:
                # Per-env 混合类型预览
                pos = torch.zeros(self.num_envs, 3, device=self.device)

                p0, _, _, _, _ = generate_circle_trajectory(
                    t, self._traj_radius, self._traj_omega, self._traj_height, self._traj_center_xy)
                p1, _, _, _, _ = generate_figure8_trajectory(
                    t, self._fig8_a, self._fig8_b, self._traj_omega, self._traj_height, self._traj_center_xy)
                p2, _, _, _, _ = generate_spiral_trajectory(
                    t, self._traj_radius, self._traj_omega, self._spiral_ascent, self._traj_height, self._traj_center_xy)
                p3, _, _, _, _ = generate_poly_waypoint_trajectory(
                    t, self._poly_waypoints, self._poly_seg_dur, self.cfg.poly_num_waypoints)
                zero_omega = torch.zeros_like(self._traj_omega)
                zero_radius = torch.zeros_like(self._traj_radius)
                p4, _, _, _, _ = generate_circle_trajectory(
                    t, zero_radius, zero_omega, self._traj_height, self._traj_center_xy)

                for type_idx, pi in enumerate([p0, p1, p2, p3, p4]):
                    mask = (self._traj_type_idx == type_idx).unsqueeze(1).expand_as(pos)
                    pos = torch.where(mask, pi, pos)

            all_positions.append(pos)

        preview_positions = torch.stack(all_positions, dim=1).reshape(-1, 3)
        return preview_positions
