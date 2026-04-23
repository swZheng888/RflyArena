#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
RL 轨迹跟踪控制节点 (独立功能包)

仿照 nmpc_control/scripts/control_nmpc_node.py 的输入/输出接口及起飞逻辑，
使用深度强化学习策略（25D 观测 ONNX 模型）替代 NMPC 优化器。

订阅话题（与 nmpc_control 一致）:
  /vio/odometry              — nav_msgs/Odometry  (ENU, FLU)
  /position_cmd              — quadrotor_msgs/PositionCommand  (平坦轨迹命令)
  /mavros/rc/in              — mavros_msgs/RCIn   (通道5 低位:轨迹 / 高位:位置)
  /mavros/state              — mavros_msgs/State  (Offboard + Armed 检测)

发布话题（与 nmpc_control 一致）:
  /mavros/setpoint_raw/attitude — mavros_msgs/AttitudeTarget  (throttle + body_rate)
  /control_node/mode            — std_msgs/String  (TAKEOFF / HOVER / RL_TRACKING)
  /rl_control/solve_time_ms     — std_msgs/Float32 (推理耗时 ms, 与 NMPC 对齐)

状态机（与 nmpc_control._run_trajectory_mode 对齐）:
  IDLE → TAKEOFF → HOVER → [GOTO_START] → TRACKING

launch 参数:
  model_path          (必须)  policy.onnx 路径
  takeoff_height      [1.0]   起飞高度 (m)
  takeoff_vz_max      [0.5]   最大起飞速度 (m/s)
  takeoff_dist_thr    [0.15]  高度判定阈值 (m)
  start_threshold     [0.3]   到达轨迹起点的距离阈值 (m)
  skip_takeoff        [False] 跳过起飞（已手动起飞时）
  control_rate        [100]   控制频率 (Hz)
  odom_topic          [/vio/odometry]
  position_cmd_topic  [/position_cmd]
  attitude_cmd_topic  [/mavros/setpoint_raw/attitude]
  debug_flag          [0]     1=开启调试输出
"""

import rospy
import numpy as np
import time
import sys
import os

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry, Path
from mavros_msgs.msg import AttitudeTarget, State, RCIn
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

import tf.transformations as tf_trans

from rl_control.utils.config import (
    GRAVITY, MASS, At, Bt,
    Ixx, Iyy, Izz,
    MAX_ROLL_RATE, MAX_PITCH_RATE, MAX_YAW_RATE,
    THRUST_RANGE, HOVER_THROTTLE
)


# ===========================================================================
# 控制模式枚举（与 nmpc_control 完全一致）
# ===========================================================================
class ControlMode:
    TRAJECTORY = 0
    POSITION   = 1


class NodeState:
    IDLE        = "IDLE"
    TAKEOFF     = "TAKEOFF"
    HOVER       = "HOVER"
    GOTO_START  = "GOTO_START"
    TRACKING    = "TRACKING"


# ===========================================================================
# ONNX 策略包装器
# ===========================================================================
class RLPolicy:
    """加载并运行 ONNX 格式的 RL 策略（rsl_rl 自动导出，含 empirical normalizer）"""

    def __init__(self, model_path: str):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"找不到模型文件: {model_path}")
        try:
            import onnxruntime as ort
            self._sess = ort.InferenceSession(
                model_path, providers=["CPUExecutionProvider"]
            )
            self._in  = self._sess.get_inputs()[0].name
            self._out = self._sess.get_outputs()[0].name
            rospy.loginfo("[rl_control] ✅ ONNX 策略已加载: %s", model_path)
            rospy.loginfo("[rl_control]    输入 shape: %s", self._sess.get_inputs()[0].shape)
        except ImportError:
            rospy.logerr("[rl_control] onnxruntime 未安装，请执行: pip3 install onnxruntime")
            raise
        except Exception as e:
            rospy.logerr("[rl_control] 加载模型失败: %s", e)
            raise

    def predict(self, obs: np.ndarray) -> np.ndarray:
        """obs (25,) -> action (4,) ∈ [-1, 1]"""
        x = obs.astype(np.float32).reshape(1, -1)
        out = self._sess.run([self._out], {self._in: x})[0][0]
        return np.clip(out, -1.0, 1.0)


# ===========================================================================
# 主控制节点
# ===========================================================================
class ControlRLNode:

    def __init__(self):
        rospy.init_node("control_rl_node", anonymous=False)

        self._init_params()
        self._init_state()
        self._init_policy()
        self._init_subscribers()
        self._init_publishers()
        self._init_timers()

        rospy.loginfo("=" * 60)
        rospy.loginfo("[rl_control] 节点启动完成")
        rospy.loginfo("[rl_control] 悬停油门 (计算值): %.4f", HOVER_THROTTLE)
        rospy.loginfo("[rl_control] 起飞高度: %.2f m | 最大 Vz: %.2f m/s", 
                      self.takeoff_height, self.takeoff_vz_max)
        rospy.loginfo("[rl_control] 控制频率: %d Hz", self.ctrl_rate)
        rospy.loginfo("[rl_control] 角速率上限: roll=%.1f pitch=%.1f yaw=%.1f rad/s",
                      self.max_roll_rate, self.max_pitch_rate, self.max_yaw_rate)
        rospy.loginfo("=" * 60)

    # ------------------------------------------------------------------
    # 初始化
    # ------------------------------------------------------------------
    def _init_params(self):
        # 必须参数
        self.model_path = rospy.get_param("~model_path", "")
        if not self.model_path:
            rospy.logerr("[rl_control] 未指定 ~model_path，退出")
            sys.exit(1)

        # 控制参数（与训练 cfg 对齐）
        self.ctrl_rate       = rospy.get_param("~control_rate",    100)  # 100Hz = decimation=2@200Hz物理
        self.thrust_range    = rospy.get_param("~thrust_range",    THRUST_RANGE)
        self.max_roll_rate   = rospy.get_param("~max_roll_rate",   MAX_ROLL_RATE)
        self.max_pitch_rate  = rospy.get_param("~max_pitch_rate",  MAX_PITCH_RATE)
        self.max_yaw_rate    = rospy.get_param("~max_yaw_rate",    MAX_YAW_RATE)
        self.hover_throttle  = rospy.get_param("~hover_throttle",  HOVER_THROTTLE)

        # 起飞参数（与 nmpc_control 一致）
        self.skip_takeoff        = rospy.get_param("~skip_takeoff",       False)
        self.takeoff_height      = rospy.get_param("~takeoff_height",     1.0)
        self.takeoff_vz_max      = rospy.get_param("~takeoff_vz_max",     0.5)
        self.takeoff_dist_thr    = rospy.get_param("~takeoff_dist_thr",   0.15)
        self.start_threshold     = rospy.get_param("~start_threshold",    0.3)

        # 话题
        self.odom_topic     = rospy.get_param("~odom_topic",     "/vio/odometry")
        self.pos_cmd_topic  = rospy.get_param("~position_cmd_topic", "/position_cmd")
        self.att_cmd_topic  = rospy.get_param("~attitude_cmd_topic",
                                              "/mavros/setpoint_raw/attitude")
        self.debug_flag     = rospy.get_param("~debug_flag", 0)
        self.max_path_pts   = rospy.get_param("~max_path_points", 2000)

        # 物理常量
        self.mass    = MASS
        self.gravity = GRAVITY

    def _init_state(self):
        # Odometry (ENU / FLU)
        self.pos_w      = np.zeros(3)
        self.vel_w      = np.zeros(3)
        self.quat       = np.array([1., 0., 0., 0.])  # w,x,y,z
        self.ang_vel_b  = np.zeros(3)                  # body FLU

        # 轨迹指令
        self.tgt_pos      = np.zeros(3)
        self.tgt_vel      = np.zeros(3)
        self.tgt_acc      = np.zeros(3)
        self.tgt_jerk     = np.zeros(3)
        self.tgt_yaw      = 0.0
        self.tgt_yaw_dot  = 0.0
        self.yaw_dir      = np.array([1., 0., 0.])

        # 飞控状态
        self.fcu_mode    = ""
        self.armed       = False
        self.ctrl_active = False
        self.rc_mode     = ControlMode.TRAJECTORY

        # 收到标志
        self.odom_ok = False
        self.traj_ok = False

        # 状态机
        if self.skip_takeoff:
            self.node_state = NodeState.HOVER
            rospy.loginfo("[rl_control] skip_takeoff=True，初始状态 → HOVER")
        else:
            self.node_state = NodeState.IDLE

        # 起飞缓存
        self.takeoff_pos       = None  # 起飞时的 xy 位置
        self.takeoff_start_z   = None
        self.takeoff_start_t   = None
        self.traj_started      = False
        self.traj_start_pos    = None
        self.hover_pos         = None

        # 上一步动作（25D obs 最后一维）
        self.last_thr_action = 0.0

        # 位置积分项（对齐训练 env _pos_integral）
        self._pos_integral  = np.zeros(3)   # world 坐标系，限幅 ·3m
        self._last_obs_time = None          # 用于积分 dt 计算

    def _init_policy(self):
        try:
            self.policy = RLPolicy(self.model_path)
        except Exception as e:
            rospy.logerr("[rl_control] 初始化策略失败: %s", e)
            sys.exit(1)

    def _init_subscribers(self):
        rospy.Subscriber(self.odom_topic,    Odometry,         self._cb_odom,    queue_size=10)
        rospy.Subscriber(self.pos_cmd_topic, PositionCommand,  self._cb_pos_cmd, queue_size=10)
        rospy.Subscriber("/mavros/state",    State,            self._cb_state,   queue_size=10)
        rospy.Subscriber("/mavros/rc/in",    RCIn,             self._cb_rc,      queue_size=10)

    def _init_publishers(self):
        self.pub_att   = rospy.Publisher(self.att_cmd_topic, AttitudeTarget, queue_size=10)
        self.pub_mode  = rospy.Publisher("/control_node/mode", String, queue_size=10)
        self.pub_time  = rospy.Publisher("/rl_control/solve_time_ms", Float32, queue_size=10)

        if self.debug_flag:
            self.pub_traj_pose = rospy.Publisher("/vis/rl/traj_pose", PoseStamped, queue_size=100)
            self.pub_traj_path = rospy.Publisher("/vis/rl/traj_path", Path, queue_size=10)
            self.pub_odom_pose = rospy.Publisher("/vis/rl/odom_pose", PoseStamped, queue_size=10)
            self.pub_odom_path = rospy.Publisher("/vis/rl/odom_path", Path, queue_size=10)
            self._traj_path_msg = Path(); self._traj_path_msg.header.frame_id = "world"
            self._odom_path_msg = Path(); self._odom_path_msg.header.frame_id = "world"

    def _init_timers(self):
        rospy.Timer(rospy.Duration(1.0 / self.ctrl_rate), self._control_loop)

    # ------------------------------------------------------------------
    # ROS 回调
    # ------------------------------------------------------------------
    def _cb_odom(self, msg: Odometry):
        self.pos_w     = np.array([msg.pose.pose.position.x,
                                   msg.pose.pose.position.y,
                                   msg.pose.pose.position.z])
        self.vel_w     = np.array([msg.twist.twist.linear.x,
                                   msg.twist.twist.linear.y,
                                   msg.twist.twist.linear.z])
        self.quat      = np.array([msg.pose.pose.orientation.w,
                                   msg.pose.pose.orientation.x,
                                   msg.pose.pose.orientation.y,
                                   msg.pose.pose.orientation.z])
        self.ang_vel_b = np.array([msg.twist.twist.angular.x,
                                   msg.twist.twist.angular.y,
                                   msg.twist.twist.angular.z])

        if not self.odom_ok:
            self.odom_ok = True
            if self.debug_flag:
                rospy.loginfo("[rl_control] 首次收到 Odom: [%.2f, %.2f, %.2f]",
                              self.pos_w[0], self.pos_w[1], self.pos_w[2])

        if self.debug_flag:
            self._pub_debug_odom()

    def _cb_pos_cmd(self, msg: PositionCommand):
        self.tgt_pos     = np.array([msg.position.x,     msg.position.y,     msg.position.z])
        self.tgt_vel     = np.array([msg.velocity.x,     msg.velocity.y,     msg.velocity.z])
        self.tgt_acc     = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z])
        self.tgt_jerk    = np.array([msg.jerk.x,         msg.jerk.y,         msg.jerk.z])
        self.yaw_dir     = np.array([msg.yaw_dir.x,      msg.yaw_dir.y,      msg.yaw_dir.z])
        self.tgt_yaw     = float(msg.yaw)
        self.tgt_yaw_dot = float(msg.yaw_dot)

        if not self.traj_ok:
            self.traj_ok        = True
            self.traj_start_pos = self.tgt_pos.copy()
            rospy.loginfo("[rl_control] 首次收到轨迹命令，起点: [%.2f, %.2f, %.2f]",
                          self.traj_start_pos[0], self.traj_start_pos[1], self.traj_start_pos[2])

    def _cb_state(self, msg: State):
        self.fcu_mode = msg.mode
        self.armed    = msg.armed
        is_offboard   = (self.fcu_mode == "OFFBOARD") and self.armed

        if is_offboard and not self.ctrl_active:
            rospy.loginfo("[rl_control] >>> 飞控进入 Offboard + Armed <<<")
            self.ctrl_active = True
            # 在首次激活时初始化起飞位置
            if self.node_state == NodeState.IDLE:
                self.node_state    = NodeState.TAKEOFF
                self.takeoff_pos   = self.pos_w.copy()
                self.takeoff_start_z = self.pos_w[2]
                self.takeoff_start_t = rospy.Time.now().to_sec()
                rospy.loginfo("[rl_control] 开始起飞: 目标高度=%.2f m", self.takeoff_height)

        if not is_offboard and self.ctrl_active:
            rospy.loginfo("[rl_control] 离开 Offboard 模式: %s", self.fcu_mode)
            self.ctrl_active = False
            if not self.skip_takeoff:
                self.node_state  = NodeState.IDLE
                self.takeoff_pos = None

    def _cb_rc(self, msg: RCIn):
        if len(msg.channels) < 5:
            return
        sw = msg.channels[4]  # 通道5，与 nmpc_control 对齐
        old = self.rc_mode
        if sw < 1300:
            self.rc_mode = ControlMode.TRAJECTORY
        elif sw > 1700:
            self.rc_mode = ControlMode.POSITION
        if old != self.rc_mode:
            mode_str = "TRAJECTORY" if self.rc_mode == ControlMode.TRAJECTORY else "POSITION"
            rospy.loginfo("[rl_control] RC 模式切换 → %s (ch5=%d)", mode_str, sw)

    # ------------------------------------------------------------------
    # 主控制循环
    # ------------------------------------------------------------------
    def _control_loop(self, event):
        if not self.odom_ok:
            return
        if not self.ctrl_active or self.rc_mode == ControlMode.POSITION:
            self._send_zero()
            return

        t0 = time.perf_counter()
        try:
            self._run_state_machine()
        except Exception as e:
            rospy.logerr_throttle(1.0, "[rl_control] 控制异常: %s", e)
            self._send_hover()

        dt_ms = (time.perf_counter() - t0) * 1000.0
        self.pub_time.publish(Float32(dt_ms))

    def _run_state_machine(self):
        # ── TAKEOFF ──────────────────────────────────────────────────
        if self.node_state == NodeState.TAKEOFF:
            self.pub_mode.publish("TAKEOFF")
            done = self._do_takeoff()
            if done:
                rospy.loginfo("[rl_control] ✅ 起飞完成 → HOVER")
                self.node_state = NodeState.HOVER
                self.hover_pos  = self.pos_w.copy()

        # ── HOVER ────────────────────────────────────────────────────
        elif self.node_state == NodeState.HOVER:
            self.pub_mode.publish("HOVER")
            if self.hover_pos is None:
                self.hover_pos = self.pos_w.copy()
            hp = self.hover_pos.copy()
            hp[2] = (self.takeoff_pos[2] if self.takeoff_pos is not None else 0.0) + self.takeoff_height
            self._rl_step(hp, np.zeros(3), np.zeros(3), 0.0, 0.0)

            if self.traj_ok:
                dist = np.linalg.norm(self.pos_w - self.traj_start_pos)
                if dist > self.start_threshold:
                    rospy.loginfo("[rl_control] 轨迹起点在 %.2f m 处，先飞过去 → GOTO_START", dist)
                    self.node_state   = NodeState.GOTO_START
                    self.traj_started = False
                    self._pos_integral = np.zeros(3)  # 切换目标时重置 I-term
                else:
                    rospy.loginfo("[rl_control] 已在轨迹起点附近 → TRACKING")
                    self.node_state   = NodeState.TRACKING
                    self.traj_started = True
                    self._pos_integral = np.zeros(3)  # 进入 TRACKING 时重置 I-term

        # ── GOTO_START ───────────────────────────────────────────────
        elif self.node_state == NodeState.GOTO_START:
            self.pub_mode.publish("GOTO_START")
            if self.traj_start_pos is None:
                return
            # 飞向轨迹起点（零速零加速度目标）
            yaw_ref = np.arctan2(
                self.traj_start_pos[1] - self.pos_w[1],
                self.traj_start_pos[0] - self.pos_w[0]
            )
            self._rl_step(self.traj_start_pos, np.zeros(3), np.zeros(3), yaw_ref, 0.0)
            dist = np.linalg.norm(self.pos_w[:2] - self.traj_start_pos[:2])
            rospy.loginfo_throttle(1.0, "[rl_control] 前往轨迹起点... 距离=%.2f m", dist)
            if dist < self.start_threshold:
                rospy.loginfo("[rl_control] 到达轨迹起点 → TRACKING")
                self.node_state   = NodeState.TRACKING
                self.traj_started = True

        # ── TRACKING ─────────────────────────────────────────────────
        elif self.node_state == NodeState.TRACKING:
            self.pub_mode.publish("RL_TRACKING")
            if not self.traj_ok:
                self.node_state = NodeState.HOVER
                return
            self._rl_step(self.tgt_pos, self.tgt_vel, self.tgt_acc,
                          self.tgt_yaw, self.tgt_yaw_dot)
            if self.debug_flag:
                self._pub_debug_traj(self.tgt_pos, self.tgt_yaw)

        # ── IDLE ─────────────────────────────────────────────────────
        else:
            self.pub_mode.publish("IDLE")
            self._send_zero()

    # ------------------------------------------------------------------
    # 起飞控制（复用 nmpc_control 三段式速度规划）
    # ------------------------------------------------------------------
    def _do_takeoff(self) -> bool:
        """
        三段式起飞：加速 → 匀速 → 减速
        使用 RL 策略飞到起飞高度（目标给 RL 静态悬停点）。
        返回 True = 起飞完成。
        """
        if self.takeoff_pos is None:
            self.takeoff_pos     = self.pos_w.copy()
            self.takeoff_start_z = self.pos_w[2]
            self.takeoff_start_t = rospy.Time.now().to_sec()

        cur_z   = self.pos_w[2]
        cur_vz  = self.vel_w[2]
        tgt_z   = (self.takeoff_pos[2] if self.takeoff_pos is not None else 0.0) + self.takeoff_height
        dist    = tgt_z - cur_z
        slow_d  = 0.3  # 减速区域

        if dist > slow_d:
            tgt_vz = self.takeoff_vz_max
        elif dist > 0.0:
            tgt_vz = max(self.takeoff_vz_max * (dist / slow_d), 0.05)
        else:
            tgt_vz = 0.0

        # 构建起飞目标（xy 保持初始位置，z = 目标高度，速度前馈 vz）
        tgt = np.array([self.takeoff_pos[0], self.takeoff_pos[1], tgt_z])
        tgt_vel_ref = np.array([0., 0., tgt_vz])
        cur_yaw = self._quat_to_yaw(self.quat)
        self._rl_step(tgt, tgt_vel_ref, np.zeros(3), cur_yaw, 0.0)

        rospy.loginfo_throttle(1.0,
            "[rl_control] 起飞中... z=%.2f/%.2f dist=%.2f vz=%.2f/%.2f",
            cur_z, tgt_z, dist, cur_vz, tgt_vz)

        return abs(dist) < self.takeoff_dist_thr and abs(cur_vz) < 0.1

    # ------------------------------------------------------------------
    # RL 推理步（核心）
    # ------------------------------------------------------------------
    def _rl_step(self, tgt_pos, tgt_vel, tgt_acc, tgt_yaw, tgt_yaw_rate):
        obs    = self._build_obs(tgt_pos, tgt_vel, tgt_acc, tgt_yaw, tgt_yaw_rate)
        action = self.policy.predict(obs)
        self.last_thr_action = action[0]
        self._publish_action(action)

    def _build_obs(self, tgt_pos, tgt_vel, tgt_acc, tgt_yaw, tgt_yaw_rate) -> np.ndarray:
        """
        25D 观测，与训练 quadcopter_trajectory_env._get_observations 严格对齐:
          [0:3]  vel_b        [3:6]  ang_vel_b   [6:9]  g_b
          [9:12] pos_err_b    [12:15] vel_err_b  [15:18] acc_ref_b
          [18]   yaw_err      [19]   yaw_rate_ref [20] throttle_ref [21] last_thr
          [22:25] pos_integral_b   (I-term 防漂移)
        """
        R_wb  = self._quat_to_R(self.quat)   # world ← body (FLU)
        R_bw  = R_wb.T                        # body ← world

        vel_b     = R_bw @ self.vel_w
        g_b       = R_bw @ np.array([0., 0., -1.])
        pos_err_w = tgt_pos - self.pos_w
        pos_err_b = R_bw @ pos_err_w
        vel_err_b = R_bw @ (tgt_vel - self.vel_w)
        acc_ref_b = R_bw @ tgt_acc

        cur_yaw   = self._quat_to_yaw(self.quat)
        yaw_err   = np.arctan2(np.sin(tgt_yaw - cur_yaw), np.cos(tgt_yaw - cur_yaw))

        # 参考油门前馈（与训练 obs 计算逻辑一致）
        total_thr_ref = self.mass * (tgt_acc[2] + self.gravity)
        single_thr_ref = total_thr_ref / 4.0
        thr_raw = (single_thr_ref - Bt) / At
        # 归一化到 action 空间 [-1, 1]
        if thr_raw >= self.hover_throttle:
            thr_ref_norm = (thr_raw - self.hover_throttle) / max(1.0 - self.hover_throttle, 1e-6)
        else:
            thr_ref_norm = (thr_raw - self.hover_throttle) / max(self.hover_throttle, 1e-6)
        thr_ref_norm = float(np.clip(thr_ref_norm, -1., 1.))

        # 位置积分项更新 (world frame 累计位置误差)
        now = rospy.Time.now().to_sec()
        if self._last_obs_time is not None:
            dt = min(now - self._last_obs_time, 0.02)  # 限制 dt 最大2个控制周期，防突变
            self._pos_integral += pos_err_w * dt
            self._pos_integral  = np.clip(self._pos_integral, -3.0, 3.0)  # 限幅 ·3m
        self._last_obs_time = now
        pos_integral_b = R_bw @ self._pos_integral

        obs = np.concatenate([
            vel_b, self.ang_vel_b, g_b,
            pos_err_b, vel_err_b, acc_ref_b,
            [yaw_err, tgt_yaw_rate, thr_ref_norm, self.last_thr_action],
            pos_integral_b,                    # [22:25] I-term
        ])
        obs = np.clip(np.nan_to_num(obs, nan=0., posinf=20., neginf=-20.), -20., 20.)
        return obs.astype(np.float32)

    # ------------------------------------------------------------------
    # 发布控制指令
    # ------------------------------------------------------------------
    def _publish_action(self, action: np.ndarray):
        """
        action[0] ∈ [-1,1] → throttle = hover_throttle ± thrust_range
        action[1:4] ∈ [-1,1] → body angular rate FLU

        注意: MAVROS body_rate 使用 FLU 坐标系（MAVROS 内部自动做 FLU→FRD 输出给 PX4）
        与 RflySim 不同! RflySim 的 PX4MavCtrlV4 直接用 FRD，所以那边需要翻转 pitch/yaw
        但 MAVROS 不需要翻转 —— 与 NMPC 节点 _publish_control 一致
        """
        throttle = float(np.clip(
            self.hover_throttle + action[0] * self.thrust_range, 0., 1.
        ))
        # MAVROS body_rate = FLU, 不做翻转（对齐 NMPC 节点行为）
        wx = action[1] * self.max_roll_rate
        wy = action[2] * self.max_pitch_rate
        wz = action[3] * self.max_yaw_rate

        msg = AttitudeTarget()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = "FCU"
        msg.type_mask       = AttitudeTarget.IGNORE_ATTITUDE
        msg.thrust          = throttle
        msg.body_rate.x     = float(wx)
        msg.body_rate.y     = float(wy)
        msg.body_rate.z     = float(wz)
        self.pub_att.publish(msg)

    def _send_zero(self):
        msg = AttitudeTarget()
        msg.header.stamp    = rospy.Time.now()
        msg.type_mask       = AttitudeTarget.IGNORE_ATTITUDE
        msg.thrust          = 0.0
        msg.body_rate.x = msg.body_rate.y = msg.body_rate.z = 0.0
        self.pub_att.publish(msg)

    def _send_hover(self):
        msg = AttitudeTarget()
        msg.header.stamp    = rospy.Time.now()
        msg.type_mask       = AttitudeTarget.IGNORE_ATTITUDE
        msg.thrust          = float(np.clip(self.hover_throttle, 0., 1.))
        msg.body_rate.x = msg.body_rate.y = msg.body_rate.z = 0.0
        self.pub_att.publish(msg)

    # ------------------------------------------------------------------
    # 数学工具
    # ------------------------------------------------------------------
    def _quat_to_R(self, q: np.ndarray) -> np.ndarray:
        """四元数 [w,x,y,z] → 旋转矩阵 R_wb（world ← body）"""
        w, x, y, z = q
        return np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
            [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]
        ])

    def _quat_to_yaw(self, q: np.ndarray) -> float:
        w, x, y, z = q
        return float(np.arctan2(2.*(w*z + x*y), 1. - 2.*(y*y + z*z)))

    # ------------------------------------------------------------------
    # 调试可视化
    # ------------------------------------------------------------------
    def _pub_debug_odom(self):
        stamp = rospy.Time.now()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.position.x = self.pos_w[0]
        pose.pose.position.y = self.pos_w[1]
        pose.pose.position.z = self.pos_w[2]
        pose.pose.orientation.w = self.quat[0]
        pose.pose.orientation.x = self.quat[1]
        pose.pose.orientation.y = self.quat[2]
        pose.pose.orientation.z = self.quat[3]
        self.pub_odom_pose.publish(pose)
        self._odom_path_msg.header.stamp = stamp
        self._odom_path_msg.poses.append(pose)
        if len(self._odom_path_msg.poses) > self.max_path_pts:
            self._odom_path_msg.poses.pop(0)
        self.pub_odom_path.publish(self._odom_path_msg)

    def _pub_debug_traj(self, pos, yaw):
        stamp = rospy.Time.now()
        quat_arr = tf_trans.quaternion_from_euler(0, 0, yaw)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.x = quat_arr[0]
        pose.pose.orientation.y = quat_arr[1]
        pose.pose.orientation.z = quat_arr[2]
        pose.pose.orientation.w = quat_arr[3]
        self.pub_traj_pose.publish(pose)
        self._traj_path_msg.header.stamp = stamp
        self._traj_path_msg.poses.append(pose)
        if len(self._traj_path_msg.poses) > self.max_path_pts:
            self._traj_path_msg.poses.pop(0)
        self.pub_traj_path.publish(self._traj_path_msg)


# ===========================================================================
if __name__ == "__main__":
    try:
        node = ControlRLNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[rl_control] 节点被中断")
    except Exception as e:
        rospy.logerr("[rl_control] 程序异常: %s", e)
        raise
    finally:
        rospy.loginfo("[rl_control] 程序退出")
