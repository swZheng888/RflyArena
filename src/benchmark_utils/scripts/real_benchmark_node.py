#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
real_benchmark_node.py - 真机安全 Benchmark 节点

单进程多任务设计：一次启动，顺序执行所有测试，任务之间安全悬停过渡。
不会 kill/restart 进程，避免真机飞行时 setpoint 中断。

用法:
  rosrun benchmark_utils real_benchmark_node.py \
      _tasks_file:=/path/to/real_benchmark_tasks.yaml \
      _log_dir:=/path/to/results \
      _odom_topic:=/vrpn_client_node/droneyee08/0/odometry

状态机 (每个任务):
  HOVER_TRANSIT → MOVE_TO_START → STABILIZE → EXECUTING → TASK_DONE
  → (下一个任务 or ALL_DONE)

安全特性:
  - /real_benchmark/abort (std_msgs/Empty) 随时中断 → 悬停
  - 任务间 5s 悬停过渡
  - 移动到起点超时保护
  - 持续发布 /position_cmd，控制器永不断流
"""

import rospy
import numpy as np
import yaml
import os
import sys
import csv
from enum import Enum
from datetime import datetime

from std_msgs.msg import String, Float32, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand

# 导入轨迹生成器
try:
    from trajectory_generator import DifferentialFlatTrajectory
except ImportError:
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "trajectory_generator",
        os.path.join(os.path.dirname(__file__), "trajectory_generator.py")
    )
    trajectory_generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(trajectory_generator)
    DifferentialFlatTrajectory = trajectory_generator.DifferentialFlatTrajectory


class BenchmarkState(Enum):
    WAITING       = "waiting"        # 等待开始信号
    HOVER_TRANSIT = "hover_transit"   # 任务间安全悬停过渡
    MOVE_TO_START = "move_to_start"  # 移动到起始点
    STABILIZE     = "stabilize"      # 在起点稳定
    EXECUTING     = "executing"      # 执行测试
    TASK_DONE     = "task_done"      # 单个任务完成
    ALL_DONE      = "all_done"       # 所有任务完成 → 触发降落
    LANDING       = "landing"        # 自动降落
    ABORTED       = "aborted"        # 中止


class RealBenchmarkNode:
    def __init__(self):
        rospy.init_node("real_benchmark_node", anonymous=False)

        self._load_params()
        self._load_tasks()
        self._init_state()
        self._init_trajectory_generator()
        self._setup_ros()

        rospy.loginfo("=" * 60)
        rospy.loginfo("[RealBench] 真机安全评估节点启动")
        rospy.loginfo("[RealBench] 任务数: %d", len(self.tasks))
        rospy.loginfo("[RealBench] 控制器: %s, publish_dynamic_yaw=%s",
                      self.controller_name, self.publish_dynamic_yaw)
        rospy.loginfo("[RealBench] 悬停点: %s", self.hover_point)
        rospy.loginfo("[RealBench] 日志: %s", self.log_dir)
        rospy.loginfo("[RealBench] 发布 /real_benchmark/start (Empty) 开始")
        rospy.loginfo("[RealBench] 发布 /real_benchmark/abort (Empty) 中止")
        rospy.loginfo("=" * 60)

    # ------------------------------------------------------------------
    # 参数加载
    # ------------------------------------------------------------------
    def _load_params(self):
        self.tasks_file = rospy.get_param("~tasks_file", "")
        if not self.tasks_file:
            rospy.logfatal("[RealBench] 必须指定 _tasks_file 参数!")
            sys.exit(1)

        self.log_dir_base = rospy.get_param("~log_dir", "")
        if not self.log_dir_base:
            self.log_dir_base = os.path.expanduser(
                f"~/benchmark_results"
            )
        self.use_timestamp_subdir = rospy.get_param("~use_timestamp_subdir", True)
        # 默认每次运行创建带时间戳的子目录；批量脚本可关闭，直接写入指定目录
        if self.use_timestamp_subdir:
            run_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_dir = os.path.join(self.log_dir_base, run_timestamp)
        else:
            self.log_dir = self.log_dir_base
        os.makedirs(self.log_dir, exist_ok=True)

        self.odom_topic = rospy.get_param("~odom_topic", "/vio/odometry")
        self.cmd_topic = rospy.get_param("~position_cmd_topic", "/position_cmd")
        self.controller_name = rospy.get_param("~controller_name", "unknown").lower()
        self.rate_hz = rospy.get_param("~rate", 100.0)
        self.frame_id = rospy.get_param("~frame_id", "world")
        # 真机默认不做在线分析，节省资源；仿真/离线可开启
        self.run_analysis = rospy.get_param("~run_analysis", False)
        # 仿真批量测试时可自动开始、结束后自动退出，便于外部脚本重复拉起
        self.auto_start = rospy.get_param("~auto_start", False)
        self.auto_start_delay = rospy.get_param("~auto_start_delay", 2.0)
        self.shutdown_on_finish = rospy.get_param("~shutdown_on_finish", False)
        self.publish_dynamic_yaw = rospy.get_param(
            "~publish_dynamic_yaw",
            self.controller_name != "rl",
        )

    def _load_tasks(self):
        with open(self.tasks_file, "r") as f:
            cfg = yaml.safe_load(f)

        self.hover_point = np.array(cfg.get("hover_point", [0.0, 0.0, 1.0]))
        self.hover_transit_time = cfg.get("hover_transit_time", 5.0)
        self.stabilize_time = cfg.get("stabilize_time", 3.0)
        self.start_pos_tolerance = cfg.get("start_pos_tolerance", 0.3)
        self.move_timeout = cfg.get("move_timeout", 30.0)
        self.trajectory_amplitude = cfg.get("trajectory_radius", 1.0)
        self.trajectory_height = cfg.get("trajectory_height", 1.0)
        self.num_loops = cfg.get("num_loops", 2)             # 飞行圈数
        self.max_height = cfg.get("max_height", 1.5)         # 硬限高
        self.land_vz = cfg.get("land_vz", 0.3)              # 降落速度 m/s
        self.land_threshold = cfg.get("land_threshold", 0.1) # 判定落地高度
        self.tasks = cfg.get("tasks", [])

        if not self.tasks:
            rospy.logfatal("[RealBench] 任务清单为空!")
            sys.exit(1)

    def _init_state(self):
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.current_yaw = 0.0  # 当前航向角 (rad)
        self.odom_ok = False

        self.state = BenchmarkState.WAITING
        self.task_idx = 0
        self.aborted = False

        # 时间戳
        self.state_entry_time = None
        self.traj_start_time = None
        self.trajectory_duration = None

        # 降落状态
        self.land_tgt_z = 0.0            # 当前降落目标高度

        # 数据记录
        self.recording = False
        self.log_file = None
        self.log_writer = None
        self.log_file_path = None
        self.record_start_time = None
        self.sample_count = 0
        self.node_start_time = rospy.Time.now()

    def _init_trajectory_generator(self):
        self.traj_gen = DifferentialFlatTrajectory(self.frame_id)
        # Yaw 平滑状态（防止交叉点处跳变）
        self._prev_smooth_yaw = None
        self._yaw_alpha = 0.3  # 低通滤波系数 (0~1, 越小越平滑)

    def _setup_ros(self):
        # Subscribers
        rospy.Subscriber(self.odom_topic, Odometry, self._cb_odom, queue_size=10)
        rospy.Subscriber("/real_benchmark/start", Empty, self._cb_start, queue_size=1)
        rospy.Subscriber("/real_benchmark/abort", Empty, self._cb_abort, queue_size=1)

        # Publishers
        self.cmd_pub = rospy.Publisher(self.cmd_topic, PositionCommand, queue_size=10)
        self.status_pub = rospy.Publisher("/real_benchmark/status", String, queue_size=10)

        # Timer
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._control_loop)
        rospy.Timer(rospy.Duration(1.0), self._status_loop)

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def _cb_odom(self, msg):
        self.current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.current_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        # 从四元数提取 yaw 角
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)
        if not self.odom_ok:
            self.odom_ok = True
            rospy.loginfo("[RealBench] 首次收到 odom: [%.2f, %.2f, %.2f]",
                          *self.current_position)

    def _cb_start(self, msg):
        if self.state == BenchmarkState.WAITING:
            rospy.loginfo("[RealBench] >>> 收到开始信号 <<<")
            self._transition_to(BenchmarkState.HOVER_TRANSIT)
        else:
            rospy.logwarn("[RealBench] 已在运行中，忽略 start")

    def _cb_abort(self, msg):
        rospy.logwarn("[RealBench] >>> 收到中止信号! 立即悬停 <<<")
        self.aborted = True
        self._stop_recording()
        self._transition_to(BenchmarkState.ABORTED)

    # ------------------------------------------------------------------
    # 状态机
    # ------------------------------------------------------------------
    def _transition_to(self, new_state):
        old = self.state
        self.state = new_state
        self.state_entry_time = rospy.Time.now()
        rospy.loginfo("[RealBench] 状态: %s → %s", old.value, new_state.value)

    def _elapsed(self):
        if self.state_entry_time is None:
            return 0.0
        return (rospy.Time.now() - self.state_entry_time).to_sec()

    @property
    def current_task(self):
        if 0 <= self.task_idx < len(self.tasks):
            return self.tasks[self.task_idx]
        return None

    def _control_loop(self, event):
        # 状态机始终运行，不受 odom 影响
        # 仅位置相关判断（MOVE_TO_START 距离检测）需要 odom

        # ── WAITING ──
        if self.state == BenchmarkState.WAITING:
            # 如有 odom 才发布悬停指令
            if self.odom_ok:
                self._publish_hover()
                if self.auto_start:
                    elapsed_from_boot = (rospy.Time.now() - self.node_start_time).to_sec()
                    if elapsed_from_boot >= self.auto_start_delay:
                        rospy.loginfo("[RealBench] 自动开始测试")
                        self._transition_to(BenchmarkState.HOVER_TRANSIT)

        # ── ABORTED ──
        elif self.state == BenchmarkState.ABORTED:
            if self.odom_ok:
                self._publish_hover()

        # ── ALL_DONE → 进入降落 ──
        elif self.state == BenchmarkState.ALL_DONE:
            if self.odom_ok:
                self._do_landing()

        # ── LANDING ──
        elif self.state == BenchmarkState.LANDING:
            if self.odom_ok:
                self._do_landing()

        # ── HOVER_TRANSIT ──
        elif self.state == BenchmarkState.HOVER_TRANSIT:
            if self.odom_ok:
                self._publish_hover()
            if self._elapsed() >= self.hover_transit_time:
                task = self.current_task
                if task is None:
                    self._transition_to(BenchmarkState.ALL_DONE)
                    self._on_all_done()
                else:
                    rospy.loginfo("=" * 60)
                    rospy.loginfo("[RealBench] 任务 [%d/%d]: %s",
                                 self.task_idx + 1, len(self.tasks),
                                 task["name"])
                    rospy.loginfo("=" * 60)
                    self._prepare_task(task)
                    self._transition_to(BenchmarkState.MOVE_TO_START)

        # ── MOVE_TO_START ──
        elif self.state == BenchmarkState.MOVE_TO_START:
            if self.odom_ok:
                self._publish_position(self.start_point, np.zeros(3),
                                       np.zeros(3), 0.0)
                dist = np.linalg.norm(self.current_position - self.start_point)
                if dist < self.start_pos_tolerance:
                    self._transition_to(BenchmarkState.STABILIZE)
            else:
                rospy.logwarn_throttle(5.0, "[RealBench] 等待 odom (%s)",
                                       self.odom_topic)
            if self._elapsed() > self.move_timeout:
                rospy.logwarn("[RealBench] 移动超时 (%.0fs), 强制稳定",
                             self.move_timeout)
                self._transition_to(BenchmarkState.STABILIZE)

        # ── STABILIZE ──
        elif self.state == BenchmarkState.STABILIZE:
            if self.odom_ok:
                self._publish_position(self.start_point, np.zeros(3),
                                       np.zeros(3), self.start_yaw)
            if self._elapsed() >= self.stabilize_time:
                self._start_recording()
                self._transition_to(BenchmarkState.EXECUTING)
                self.traj_start_time = rospy.Time.now()
                self._prev_smooth_yaw = None  # 重置yaw平滑状态

        # ── EXECUTING ──
        elif self.state == BenchmarkState.EXECUTING:
            task = self.current_task
            if self.odom_ok:
                if task["type"] == "hover":
                    self._execute_hover()
                else:
                    self._execute_dynamic()

            elapsed = self._elapsed()
            # 悬停: 用 YAML 的 duration; 动态轨迹: 用 period × num_loops
            if task["type"] == "hover":
                stop_time = task.get("duration", 10)
            else:
                stop_time = self.trajectory_duration or 120  # 安全超时
            if elapsed >= stop_time:
                self._stop_recording()
                self._transition_to(BenchmarkState.TASK_DONE)

        # ── TASK_DONE ──
        elif self.state == BenchmarkState.TASK_DONE:
            if self.odom_ok:
                self._publish_hover()
            if self._elapsed() >= 1.0:
                if self.run_analysis:
                    self._run_analysis()
                else:
                    rospy.loginfo("[RealBench] 跳过分析 (run_analysis=false)")
                self.task_idx += 1
                self._transition_to(BenchmarkState.HOVER_TRANSIT)

    # ------------------------------------------------------------------
    # 任务准备
    # ------------------------------------------------------------------
    def _prepare_task(self, task):
        task_type = task.get("type", "hover")

        if task_type == "hover":
            self.start_point = self.hover_point.copy()
            self.start_yaw = 0.0
        else:
            traj_type = task.get("trajectory", "circle")
            speed = task.get("speed", 1.0)

            # 配置轨迹生成器
            actual_speed = 0.6 * speed  # 基础速度 × 倍率
            self.traj_gen.set_params(actual_speed, self.trajectory_amplitude)
            self.traj_gen.num_loops = self.num_loops

            # 计算 N 圈所需时长
            period = self.traj_gen.get_trajectory_period(traj_type)
            soft_comp = 0.5 * self.traj_gen.soft_start_duration
            self.trajectory_duration = period * self.num_loops + soft_comp
            rospy.loginfo("[RealBench] %s: 周期=%.1fs × %d圈 = %.1fs",
                          traj_type, period, self.num_loops, self.trajectory_duration)

            # ① 设置飞行高度 (覆盖硬编码的 z_base=2.5m)
            self.traj_gen.z_base = self.trajectory_height

            # ② 先用原点临时查 t=0 位置（无偏移），确定 world_offset
            self.traj_gen._world_offset = np.zeros(3)
            self.traj_gen._current_traj_type = traj_type
            try:
                pos0_raw, _, _, _, _, _ = self.traj_gen.get_trajectory(traj_type, 0.0)
            except Exception as e:
                rospy.logwarn("[RealBench] 获取轨迹原点失败: %s", e)
                pos0_raw = np.array([0.0, 0.0, self.trajectory_height])

            # ③ 将轨迹 XY 中心对齐到无人机当前 XY，Z 保持 trajectory_height
            center_pos = np.array([
                self.current_position[0],
                self.current_position[1],
                self.trajectory_height
            ])
            self.traj_gen._world_offset = center_pos - pos0_raw

            # ④ 获取偏移后的起始点
            try:
                pos, _, _, _, yaw, _ = self.traj_gen.get_trajectory(traj_type, 0.0)
                self.start_point = pos.copy()
                self.start_yaw = yaw
            except Exception as e:
                rospy.logwarn("[RealBench] 获取起始轨迹点失败: %s, 使用悬停点", e)
                self.start_point = self.hover_point.copy()
                self.start_yaw = 0.0

        rospy.loginfo("[RealBench] 起始点: [%.2f, %.2f, %.2f] yaw=%.1f°",
                     self.start_point[0], self.start_point[1],
                     self.start_point[2], np.degrees(self.start_yaw))

    # ------------------------------------------------------------------
    # 执行任务
    # ------------------------------------------------------------------
    def _execute_hover(self):
        self._publish_position(self.hover_point, np.zeros(3),
                               np.zeros(3), 0.0)
        self._record_data(self.hover_point, np.zeros(3),
                          np.zeros(3), np.zeros(3), 0.0, 0.0)

    def _execute_dynamic(self):
        task = self.current_task
        traj_type = task.get("trajectory", "circle")

        if self.traj_start_time is None:
            self.traj_start_time = rospy.Time.now()

        traj_t = (rospy.Time.now() - self.traj_start_time).to_sec()

        try:
            pos, vel, acc, jerk, yaw, yaw_dot = self.traj_gen.get_trajectory(
                traj_type, traj_t)
        except Exception as e:
            rospy.logerr_throttle(1.0, "[RealBench] 轨迹生成错误: %s", e)
            self._publish_hover()
            return

        # figure8 / zigzag / star / square 在交叉点或尖点附近速度方向可能跳变。
        # 这里保留“机头沿速度方向”的语义，只做角度展开平滑，不再强制置零。
        if traj_type in ("figure8", "zigzag", "star", "square", "circle","ellipse"):
            yaw, yaw_dot = self._smooth_yaw(yaw, yaw_dot)

        if not self.publish_dynamic_yaw:
            yaw = 0.0
            yaw_dot = 0.0

        self._publish_position(pos, vel, acc, yaw, yaw_dot, jerk)
        self._record_data(pos, vel, acc, jerk, yaw, yaw_dot)

    # ------------------------------------------------------------------
    # Yaw 平滑处理
    # ------------------------------------------------------------------
    def _smooth_yaw(self, raw_yaw, raw_yaw_dot):
        """
        角度展开：防止 atan2 在交叉点处产生 ±180° 跳变
        yaw 本身连续，只需要展开 atan2 的计算结果
        """
        if self._prev_smooth_yaw is None:
            self._prev_smooth_yaw = raw_yaw
            return raw_yaw, raw_yaw_dot

        # 展开：保证与上次差值在 [-π, π] 内
        delta = raw_yaw - self._prev_smooth_yaw
        while delta > np.pi:
            delta -= 2 * np.pi
        while delta < -np.pi:
            delta += 2 * np.pi

        smooth_yaw = self._prev_smooth_yaw + delta

        # 归一化到 [-π, π]
        while smooth_yaw > np.pi:
            smooth_yaw -= 2 * np.pi
        while smooth_yaw < -np.pi:
            smooth_yaw += 2 * np.pi

        self._prev_smooth_yaw = smooth_yaw
        return smooth_yaw, raw_yaw_dot

    # ------------------------------------------------------------------
    # 发布指令
    # ------------------------------------------------------------------
    def _publish_hover(self):
        self._publish_position(self.hover_point, np.zeros(3),
                               np.zeros(3), 0.0)

    def _publish_position(self, position, velocity, acceleration,
                          yaw, yaw_dot=0.0, jerk=None):
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        msg.position.x = float(position[0])
        msg.position.y = float(position[1])
        # 高度硬限: 不超过 max_height
        z_clamped = float(min(position[2], self.max_height))
        msg.position.z = z_clamped

        msg.velocity.x = float(velocity[0])
        msg.velocity.y = float(velocity[1])
        msg.velocity.z = float(velocity[2])

        msg.acceleration.x = float(acceleration[0])
        msg.acceleration.y = float(acceleration[1])
        msg.acceleration.z = float(acceleration[2])

        if jerk is not None:
            msg.jerk.x = float(jerk[0])
            msg.jerk.y = float(jerk[1])
            msg.jerk.z = float(jerk[2])

        msg.yaw_dir.x = float(np.cos(yaw))
        msg.yaw_dir.y = float(np.sin(yaw))
        msg.yaw_dir.z = 0.0
        msg.yaw_dir_dot.x = float(-np.sin(yaw) * yaw_dot)
        msg.yaw_dir_dot.y = float(np.cos(yaw) * yaw_dot)
        msg.yaw_dir_dot.z = 0.0
        msg.yaw = float(yaw)
        msg.yaw_dot = float(yaw_dot)
        msg.trajectory_id = 1
        msg.trajectory_flag = 0

        self.cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # 数据记录
    # ------------------------------------------------------------------
    def _start_recording(self):
        task = self.current_task
        task_name = task["name"] if task else "unknown"
        task_dir = os.path.join(self.log_dir, task_name)
        os.makedirs(task_dir, exist_ok=True)

        filename = f"{task_name}.csv"
        self.log_file_path = os.path.join(task_dir, filename)

        try:
            self.log_file = open(self.log_file_path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow([
                "time",
                "target_x", "target_y", "target_z",
                "actual_x", "actual_y", "actual_z",
                "target_vx", "target_vy", "target_vz",
                "actual_vx", "actual_vy", "actual_vz",
                "target_ax", "target_ay", "target_az",
                "target_jerk_x", "target_jerk_y", "target_jerk_z",
                "target_yaw", "actual_yaw", "target_yaw_dot"
            ])
            self.recording = True
            self.record_start_time = rospy.Time.now()
            self.sample_count = 0
            rospy.loginfo("[RealBench] 开始记录 → %s", self.log_file_path)
        except Exception as e:
            rospy.logerr("[RealBench] 创建日志失败: %s", e)
            self.recording = False

    def _stop_recording(self):
        self.recording = False
        if self.log_file:
            try:
                self.log_file.close()
                self.log_file = None
                rospy.loginfo("[RealBench] 记录停止 (%d 样本): %s",
                             self.sample_count, self.log_file_path)
            except Exception as e:
                rospy.logerr("[RealBench] 关闭日志失败: %s", e)

    def _record_data(self, target_pos, target_vel, target_acc=None,
                     target_jerk=None, target_yaw=0.0, target_yaw_dot=0.0):
        if not self.recording or self.log_writer is None:
            return
        try:
            elapsed = (rospy.Time.now() - self.record_start_time).to_sec()
            if target_acc is None:
                target_acc = np.zeros(3)
            if target_jerk is None:
                target_jerk = np.zeros(3)

            self.log_writer.writerow([
                elapsed,
                target_pos[0], target_pos[1], target_pos[2],
                self.current_position[0], self.current_position[1],
                self.current_position[2],
                target_vel[0], target_vel[1], target_vel[2],
                self.current_velocity[0], self.current_velocity[1],
                self.current_velocity[2],
                target_acc[0], target_acc[1], target_acc[2],
                target_jerk[0], target_jerk[1], target_jerk[2],
                target_yaw, self.current_yaw, target_yaw_dot
            ])
            self.sample_count += 1
            if self.sample_count % 100 == 0:
                self.log_file.flush()
        except Exception as e:
            rospy.logwarn_throttle(1.0, "[RealBench] 记录失败: %s", e)

    # ------------------------------------------------------------------
    # 分析
    # ------------------------------------------------------------------
    def _run_analysis(self):
        """调用 benchmark_analyzer 分析当前任务"""
        task = self.current_task
        if task is None or self.log_file_path is None:
            return

        task_name = task["name"]
        task_dir = os.path.join(self.log_dir, task_name)

        analyzer_path = os.path.join(
            os.path.dirname(__file__), "benchmark_analyzer.py"
        )
        if os.path.exists(analyzer_path) and os.path.exists(self.log_file_path):
            rospy.loginfo("[RealBench] 分析: %s", task_name)
            try:
                import subprocess
                subprocess.Popen([
                    sys.executable, analyzer_path,
                    "--csv", self.log_file_path,
                    "--output", task_dir,
                    "--name", task_name
                ])
            except Exception as e:
                rospy.logwarn("[RealBench] 分析失败: %s", e)

    def _do_landing(self):
        """缓慢降落：每步降低 land_vz/rate_hz m，直到高度 < land_threshold"""
        if self.state != BenchmarkState.LANDING:
            # 初始化降落目标为当前高度
            self.land_tgt_z = self.current_position[2]
            self._transition_to(BenchmarkState.LANDING)

        dt = 1.0 / self.rate_hz
        self.land_tgt_z = max(self.land_tgt_z - self.land_vz * dt, 0.0)
        land_pos = np.array([self.current_position[0],
                             self.current_position[1],
                             self.land_tgt_z])
        land_vel = np.array([0., 0., -self.land_vz])
        self._publish_position(land_pos, land_vel, np.zeros(3), 0.0)

        rospy.loginfo_throttle(2.0, "[RealBench] 降落中... z=%.2f m",
                              self.current_position[2])

        if self.current_position[2] < self.land_threshold:
            rospy.loginfo("[RealBench] ✅ 降落完成")
            # 停止发布指令，飞控会自动 disarm
            self._transition_to(BenchmarkState.ABORTED)  # 进入静止状态
            if self.shutdown_on_finish:
                rospy.loginfo("[RealBench] 任务完成，自动退出节点")
                rospy.signal_shutdown("benchmark finished")

    def _on_all_done(self):
        rospy.loginfo("=" * 60)
        rospy.loginfo("[RealBench] 🎉 所有 %d 个任务完成! 开始降落...", len(self.tasks))
        rospy.loginfo("[RealBench] 结果: %s", self.log_dir)
        rospy.loginfo("=" * 60)
        # 批量仿真时直接退出当前轮次，避免卡在降落状态导致下一次起不来。
        if self.shutdown_on_finish:
            rospy.loginfo("[RealBench] 批量模式：当前轮次完成，直接退出")
            rospy.signal_shutdown("benchmark run finished")

    # ------------------------------------------------------------------
    # 状态发布
    # ------------------------------------------------------------------
    def _status_loop(self, event):
        task = self.current_task
        task_name = task["name"] if task else "N/A"
        msg = f"[{self.task_idx+1}/{len(self.tasks)}] {task_name} | {self.state.value}"

        if self.state == BenchmarkState.EXECUTING:
            task_dur = task.get("duration", 0) if task else 0
            msg += f" | {self._elapsed():.0f}/{task_dur}s"
        elif self.state == BenchmarkState.MOVE_TO_START:
            dist = np.linalg.norm(self.current_position - self.start_point)
            msg += f" | dist={dist:.2f}m"

        self.status_pub.publish(msg)

    def run(self):
        rospy.spin()


# ===========================================================================
if __name__ == "__main__":
    try:
        node = RealBenchmarkNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[RealBench] 节点中断")
    except Exception as e:
        rospy.logerr("[RealBench] 异常: %s", e)
        raise
