#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
NMPC 控制节点

该节点实现了基于 NMPC（非线性模型预测控制）的无人机控制框架，支持两种控制模式：
1. 轨迹跟踪模式 - 跟踪预设轨迹
2. 位置控制模式 - 手动位置控制

遥控器通道7用于模式切换：
- 低位 (<1300): 轨迹跟踪模式
- 高位 (>1300): 位置控制模式

订阅话题：
- /vio/odometry 或 /mavros/local_position/odom: 视觉/里程计位姿
- /trajectory_publisher/target: 原始轨迹点
- /position_cmd: 平坦轨迹命令
- /mavros/rc/in: 遥控器输入
- /mavros/state: 飞控状态
- /control_node/debug_flag: 调试标志

发布话题：
- /mavros/setpoint_raw/attitude: 姿态目标
- /control_node/mode: 当前控制模式
- /vis/*: 调试可视化话题（调试模式）
"""

import rospy
import numpy as np
import signal
import sys

from std_msgs.msg import String, Int32, Float32
from nav_msgs.msg import Odometry, Path
from mavros_msgs.msg import AttitudeTarget, State, RCIn
from geometry_msgs.msg import PoseStamped, TransformStamped

import tf2_ros
import tf.transformations as tf_trans

from trajectory_publisher.msg import TrajectoryPoint
from quadrotor_msgs.msg import PositionCommand

try:
    from nmpc_control.nmpc_controller import NMPC_Controller
    from nmpc_control.utils.transforms import FrameTransformer
    from nmpc_control.utils.config import CT, CR, WB, Bt, At
except ImportError as e:
    rospy.logerr("导入 nmpc_control 模块失败: %s", e)
    sys.exit(1)


class ControlMode:
    """控制模式枚举"""
    TRAJECTORY = 0
    POSITION = 1


class ControlNode(object):
    """
    NMPC 控制节点类

    实现了基于 NMPC 的无人机控制框架，支持轨迹跟踪和位置控制两种模式。
    通过遥控器通道7进行模式切换。
    """
    def __init__(self):
        """初始化控制节点"""
        rospy.init_node("control_node", anonymous=True)

        self._init_parameters()
        self._init_system_state()
        self._init_controller()
        self._init_takeoff_params()
        self._init_hover_params()
        self._init_trajectory_params()
        self._init_rc_params()
        self._init_position_control_params()
        self._init_subscribers()
        self._init_publishers()
        self._init_debug_tools()
        self._init_timers()
        self._init_display()

        rospy.loginfo("控制节点初始化完成")

    def _init_parameters(self):
        """初始化 ROS 参数"""
        self.debug_flag = rospy.get_param("~debug_flag", 0)
        self.coordinate_frame = rospy.get_param("~coordinate_frame", 0)
        self.use_flat_traj = rospy.get_param("~use_flat_traj", False)
        self.start_threshold = rospy.get_param("~start_threshold", 0.3)
        self.max_path_points = rospy.get_param("~max_path_points", 2000)
        self.takeoff_distance_threshold = rospy.get_param("~takeoff_distance_threshold", 0.2)
        
        # 轨迹预览控制参数 (方便A/B测试)
        self.use_trajectory_preview = rospy.get_param("~use_trajectory_preview", True)  # True=新方法, False=旧方法
        self.acc_threshold = rospy.get_param("~acc_threshold", 3.0)  # 加速度阈值 m/s²

        self.odom_topic = rospy.get_param("~odom_topic", "/vio/odometry")
        self.position_cmd_topic = rospy.get_param("~position_cmd_topic", "/position_cmd")
        self.attitude_cmd_topic = rospy.get_param("~attitude_cmd_topic", "/mavros/setpoint_raw/attitude")
        self.rc_topic = rospy.get_param("~rc_topic", "/mavros/rc/in")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.mode_topic = rospy.get_param("~mode_topic", "/control_node/mode")
        self.debug_flag_topic = rospy.get_param("~debug_flag_topic", "/control_node/debug_flag")

        rospy.loginfo("调试模式: %s", "开启" if self.debug_flag == 1 else "关闭")
        traj_mode = "平坦轨迹 (PositionCommand)" if self.use_flat_traj else "原始 pvq 轨迹 (TrajectoryPoint)"
        rospy.loginfo("轨迹模式: %s", traj_mode)

    def _init_system_state(self):
        """初始化系统状态变量"""
        self.current_mode = None
        self.armed = False
        self.current_state = None
        self.control_active = False
        self._last_solve_time_ms = 0.0

    def _init_controller(self):
        """初始化 NMPC 控制器"""
        self.controller = NMPC_Controller()

    def _init_takeoff_params(self):
        """初始化起飞相关参数"""
        self.takeoff_height = rospy.get_param("~takeoff_height", 1.0)
        self.takeoff_vz_max = rospy.get_param("~takeoff_vz_max", 0.5)  # m/s 最大垂直速度
        self.takeoff_position = None
        self.takeoff_quat = None
        # skip_takeoff: 跳过起飞阶段，假设已手动起飞（Docker 模式）
        self.skip_takeoff = rospy.get_param("~skip_takeoff", False)
        self.takeoff_done = self.skip_takeoff  # 如果跳过起飞，直接标记完成
        self.takeoff_start_time = None
        self.takeoff_start_z = None
        if self.skip_takeoff:
            rospy.loginfo("跳过起飞阶段 (skip_takeoff=True)")

    def _init_hover_params(self):
        """初始化悬停参考参数"""
        self.hover_position = None
        self.hover_quat = None

    def _init_trajectory_params(self):
        """初始化轨迹跟踪参数"""
        self.trajectory_started = False
        self.trajectory_received = False
        self.target_position = np.zeros(3, dtype=float)
        self.target_velocity = np.zeros(3, dtype=float)
        self.target_acceleration = np.zeros(3, dtype=float)
        self.target_angular_velocity = np.zeros(3, dtype=float)
        self.target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.target_jerk = np.zeros(3, dtype=float)
        self.yaw_dir = np.array([1.0, 0.0, 0.0], dtype=float)
        self.yaw_dir_dot = np.zeros(3, dtype=float)
        self.target_yaw = 0.0
        self.target_yaw_dot = 0.0
        self.kx = np.zeros(3, dtype=float)
        self.kv = np.zeros(3, dtype=float)
        self.trajectory_id = 0
        self.trajectory_flag = 0
        self.start_position = None
        self.start_quat = None
        
        # 轨迹预览参数 (从NMPC控制器自动获取，确保同步)
        self.preview_N = self.controller.N
        self.preview_Tf = self.controller.Tf
        self.preview_dt = self.preview_Tf / self.preview_N

    def _init_rc_params(self):
        """初始化遥控器相关参数"""
        self.RC_CHANNELS = {
            "ROLL": 0,
            "PITCH": 1,
            "THROTTLE": 2,
            "YAW": 3,
            "SWITCH_MODE": 4,
        }
        self.rc_channels = [1500] * 18
        self.rc_mode = ControlMode.TRAJECTORY
        self.RC_LOW_THRESHOLD = 1300
        self.RC_HIGH_THRESHOLD = 1700

    def _init_position_control_params(self):
        """初始化位置控制参数"""
        self.pos_ctrl_position = None
        self.pos_ctrl_yaw = 0.0
        self.pos_ctrl_max_vel_xy = rospy.get_param("~pos_ctrl_max_vel_xy", 2.0)
        self.pos_ctrl_max_vel_z = rospy.get_param("~pos_ctrl_max_vel_z", 1.0)
        self.pos_ctrl_max_yaw_rate = rospy.get_param("~pos_ctrl_max_yaw_rate", 1.0)

    def _init_subscribers(self):
        """初始化订阅者"""
        if self.coordinate_frame == 0:
            self.vio_sub = rospy.Subscriber(
                self.odom_topic, Odometry, self.odom_callback, queue_size=10
            )
        elif self.coordinate_frame == 1:
            self.vio_sub = rospy.Subscriber(
                self.odom_topic, Odometry, self.odom_callback, queue_size=10
            )
        else:
            self.vio_sub = rospy.Subscriber(
                self.odom_topic,
                Odometry,
                self.odom_callback,
                queue_size=10,
            )

        if self.use_flat_traj:
            self.traj_sub = rospy.Subscriber(
                self.position_cmd_topic, PositionCommand, self.position_cmd_callback, queue_size=10)
        else:
            self.traj_sub = rospy.Subscriber(
                "/trajectory_publisher/target", TrajectoryPoint, self.trajectory_callback, queue_size=10)

        self.rc_sub = rospy.Subscriber(self.rc_topic, RCIn, self.rc_callback, queue_size=10)
        self.state_sub = rospy.Subscriber(self.state_topic, State, self.state_callback, queue_size=10)
        self.debug_sub = rospy.Subscriber(self.debug_flag_topic, Int32, self.debug_callback, queue_size=10)
        
        # 动态参数更新
        rospy.Subscriber("/nmpc_control/update_weights", String, self.weights_update_callback, queue_size=1)
        
    def weights_update_callback(self, msg):
        """处理权重更新消息 (JSON格式)"""
        try:
            import json
            data = json.loads(msg.data)
            
            Q_pos = data.get('Q_pos')
            Q_att = data.get('Q_att')
            Q_vel = data.get('Q_vel')
            
            if self.controller.update_weights(Q_pos, Q_att, Q_vel):
                rospy.loginfo(f"NMPC weights updated: {data}")
            else:
                rospy.logwarn("Failed to update NMPC weights")
                
        except Exception as e:
            rospy.logerr(f"Invalid weight update message: {e}")

    def _init_publishers(self):
        """初始化发布者"""
        self.ctrl_FCU_pub = rospy.Publisher(
            self.attitude_cmd_topic, AttitudeTarget, queue_size=10)
        self.mode_pub = rospy.Publisher(self.mode_topic, String, queue_size=10)

        # 性能指标发布者
        self.solve_time_pub = rospy.Publisher(
            "/nmpc_control/solve_time_ms", Float32, queue_size=10)
        self.control_freq_pub = rospy.Publisher(
            "/nmpc_control/control_freq_hz", Float32, queue_size=10)

        # 控制频率统计
        self.last_control_time = None
        self.control_freq_window = []  # 滑动窗口计算频率

    def _init_debug_tools(self):
        """初始化调试可视化工具"""
        if self.debug_flag == 1:
            self.traj_pose_pub = rospy.Publisher("/vis/traj_pose", PoseStamped, queue_size=100)
            self.traj_path_pub = rospy.Publisher("/vis/traj_path", Path, queue_size=100)
            self.odom_path_pub = rospy.Publisher("/vis/odom_path", Path, queue_size=10)
            self.odom_pose_pub = rospy.Publisher("/vis/odom_pose", PoseStamped, queue_size=10)
            self.nmpc_pred_path_pub = rospy.Publisher("/vis/nmpc_pred_path", Path, queue_size=10)
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        else:
            self.traj_pose_pub = None
            self.traj_path_pub = None
            self.odom_path_pub = None
            self.odom_pose_pub = None
            self.nmpc_pred_path_pub = None
            self.tf_broadcaster = None

        self.traj_path_msg = Path()
        self.traj_path_msg.header.frame_id = "world"
        self.odom_path_msg = Path()
        self.odom_path_msg.header.frame_id = "world"
        self.nmpc_pred_path_msg = Path()
        self.nmpc_pred_path_msg.header.frame_id = "world"
        self._last_traj_pub_time = rospy.Time(0)

        rospy.loginfo("=" * 60)
        rospy.loginfo("控制节点已启动")
        rospy.loginfo("=" * 60)
        rospy.loginfo("起飞高度: %.2f m", self.takeoff_height)
        rospy.loginfo("距离判断阈值: %.2f m", self.takeoff_distance_threshold)
        rospy.loginfo("=== 遥控器模式切换（通道7）===")
        rospy.loginfo("  低位 (<1300): 轨迹跟踪模式")
        rospy.loginfo("  高位 (>1300): 位置控制模式")
        rospy.loginfo("=" * 60)

    def _init_timers(self):
        """初始化定时器"""
        control_rate = rospy.get_param("~control_rate", 100)
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / control_rate), self.control_callback)

    # -------------------------------------------------------------------------- #
    # 回调函数
    # -------------------------------------------------------------------------- #

    def debug_callback(self, msg):
        """调试标志回调函数"""
        old_flag = self.debug_flag
        self.debug_flag = msg.data
        if old_flag != self.debug_flag:
            rospy.loginfo("调试模式已%s", "开启" if self.debug_flag == 1 else "关闭")

    def trajectory_callback(self, msg):
        """原始轨迹点回调函数"""
        self.target_position = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float)
        self.target_velocity = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z], dtype=float)
        self.target_acceleration = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z], dtype=float)
        self.target_angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=float)
        self.target_quat = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z], dtype=float)

        quat_norm = np.linalg.norm(self.target_quat)
        if quat_norm > 1e-6:
            self.target_quat /= quat_norm
        else:
            self.target_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        if not self.trajectory_received:
            self.start_position = self.target_position.copy()
            self.start_quat = self.target_quat.copy()
            self.trajectory_received = True
            rospy.loginfo("收到轨迹起点: [%.2f, %.2f, %.2f]",
                          self.start_position[0], self.start_position[1], self.start_position[2])

    def position_cmd_callback(self, msg):
        """平坦轨迹命令回调函数"""
        self.target_position = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=float)
        self.target_velocity = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z], dtype=float)
        self.target_acceleration = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z], dtype=float)
        self.target_jerk = np.array([msg.jerk.x, msg.jerk.y, msg.jerk.z], dtype=float)
        self.yaw_dir = np.array([msg.yaw_dir.x, msg.yaw_dir.y, msg.yaw_dir.z], dtype=float)
        self.yaw_dir_dot = np.array([msg.yaw_dir_dot.x, msg.yaw_dir_dot.y, msg.yaw_dir_dot.z], dtype=float)
        self.target_yaw = float(msg.yaw)
        self.target_yaw_dot = float(msg.yaw_dot)
        self.kx = np.array(msg.kx, dtype=float)
        self.kv = np.array(msg.kv, dtype=float)
        self.trajectory_id = msg.trajectory_id
        self.trajectory_flag = msg.trajectory_flag

        if not self.trajectory_received:
            self.start_position = self.target_position.copy()
            self.start_quat = self.flat_to_quat(self.start_position, np.zeros(3), self.target_yaw)
            self.trajectory_received = True

    def state_callback(self, msg):
        """飞控状态回调函数"""
        self.current_mode = msg.mode
        self.armed = msg.armed

        if self.current_mode == "OFFBOARD" and self.armed:
            if not self.control_active:
                rospy.loginfo(">>> 飞控已进入 Offboard 模式且已解锁 <<<")
                self.control_active = True
        else:
            if self.control_active:
                rospy.loginfo("离开 Offboard 模式: %s", self.current_mode)
                self.control_active = False

    def rc_callback(self, msg):
        """遥控器通道回调函数"""
        if len(msg.channels) < 7:
            return

        self.rc_channels = list(msg.channels)
        mode_switch = msg.channels[self.RC_CHANNELS["SWITCH_MODE"]]

        old_mode = self.rc_mode
        if mode_switch < self.RC_LOW_THRESHOLD:
            self.rc_mode = ControlMode.TRAJECTORY
        elif mode_switch > self.RC_HIGH_THRESHOLD:
            self.rc_mode = ControlMode.POSITION
        else:
            # 中间位置保持当前模式
            pass

        if old_mode != self.rc_mode:
            mode_names = {
                ControlMode.TRAJECTORY: "轨迹跟踪",
                ControlMode.POSITION: "位置控制"
            }
            rospy.loginfo(">>> 模式切换: %s -> %s (通道值=%d) <<<",
                          mode_names.get(old_mode, "未知"),
                          mode_names.get(self.rc_mode, "未知"),
                          mode_switch)

            if self.rc_mode == ControlMode.POSITION:
                self._init_position_control_mode()

    def _init_position_control_mode(self):
        if self.current_state is not None:
            self.pos_ctrl_position = self.current_state[0:3].copy()
            quat = self.current_state[3:7]
            euler = tf_trans.euler_from_quaternion([quat[1], quat[2], quat[3], quat[0]])
            self.pos_ctrl_yaw = euler[2]
            rospy.loginfo("位置控制初始化: [%.2f, %.2f, %.2f], yaw=%.2f",
                          self.pos_ctrl_position[0], self.pos_ctrl_position[1],
                          self.pos_ctrl_position[2], self.pos_ctrl_yaw)

    def signal_handler(self, sig, frame):
        rospy.loginfo("接收到关闭信号...")
        self.send_zero_commands()
        rospy.signal_shutdown("Ctrl+C pressed")

    def send_zero_commands(self):
        """发送单条零命令（非阻塞心跳，保持 PX4 Offboard setpoint 流）"""
        control_msg = AttitudeTarget()
        control_msg.header.frame_id = "FCU"
        control_msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        control_msg.thrust = 0.0
        control_msg.body_rate.x = 0.0
        control_msg.body_rate.y = 0.0
        control_msg.body_rate.z = 0.0
        control_msg.header.stamp = rospy.Time.now()
        self.ctrl_FCU_pub.publish(control_msg)

    def publish_tf(self, position, quat):
        if self.debug_flag != 1 or self.tf_broadcaster is None:
            return
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        t.transform.rotation.w = float(quat[0])
        t.transform.rotation.x = float(quat[1])
        t.transform.rotation.y = float(quat[2])
        t.transform.rotation.z = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

    def publish_odom_path(self, position, quat):
        if self.debug_flag != 1 or self.odom_pose_pub is None:
            return
        stamp = rospy.Time.now()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        pose_msg.pose.orientation.w = float(quat[0])
        pose_msg.pose.orientation.x = float(quat[1])
        pose_msg.pose.orientation.y = float(quat[2])
        pose_msg.pose.orientation.z = float(quat[3])
        self.odom_pose_pub.publish(pose_msg)
        self.odom_path_msg.header.stamp = stamp
        self.odom_path_msg.poses.append(pose_msg)
        if len(self.odom_path_msg.poses) > self.max_path_points:
            self.odom_path_msg.poses.pop(0)
        self.odom_path_pub.publish(self.odom_path_msg)

    def publish_nmpc_prediction_path(self, all_control):
        if self.debug_flag != 1 or self.nmpc_pred_path_pub is None or all_control is None:
            return
        stamp = rospy.Time.now()
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = "world"
        try:
            if all_control.ndim == 1:
                state_dim = 10
                n_steps = len(all_control) // state_dim
                predicted_states = all_control.reshape(n_steps, state_dim)
            elif all_control.ndim == 2:
                predicted_states = all_control
            else:
                return
            for state in predicted_states:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = stamp
                pose_msg.header.frame_id = "world"
                pose_msg.pose.position.x = float(state[0])
                pose_msg.pose.position.y = float(state[1])
                pose_msg.pose.position.z = float(state[2])
                if len(state) >= 7:
                    pose_msg.pose.orientation.w = float(state[3])
                    pose_msg.pose.orientation.x = float(state[4])
                    pose_msg.pose.orientation.y = float(state[5])
                    pose_msg.pose.orientation.z = float(state[6])
                else:
                    pose_msg.pose.orientation.w = 1.0
                path_msg.poses.append(pose_msg)
            self.nmpc_pred_path_pub.publish(path_msg)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "发布预测轨迹失败: %s", str(e))

    def odom_callback(self, msg):
        if self.coordinate_frame == 0:
            position_ned = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ], dtype=float)
            lin_vel_ned = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ], dtype=float)
            quat_ned_frd = np.array([
                -msg.pose.pose.orientation.w,
                -msg.pose.pose.orientation.x,
                -msg.pose.pose.orientation.y,
                -msg.pose.pose.orientation.z
            ], dtype=float)

            position_enu = FrameTransformer.ned_to_enu_position(position_ned)
            lin_vel_enu = FrameTransformer.ned_to_enu_position(lin_vel_ned)
            quat_enu_flu = FrameTransformer.nedfrd_to_enuflu_rotation(quat_ned_frd)
        elif self.coordinate_frame == 1:
            position_enu = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ], dtype=float)
            lin_vel_frd = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ], dtype=float)
            quat_enu_flu = np.array([
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ], dtype=float)
            lin_vel_enu = FrameTransformer.flu_to_enu_velocity(lin_vel_frd, quat_enu_flu)
        else:
            position_enu = np.array(
                [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ],
                dtype=float,
            )
            lin_vel_enu = np.array(
                [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z,
                ],
                dtype=float,
            )
            quat_enu_flu = np.array(
                [
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                ],
                dtype=float,
            )

        self.current_state = np.concatenate([position_enu, quat_enu_flu, lin_vel_enu])

        if self.debug_flag == 1 and self.control_active:
            self.publish_tf(position_enu, quat_enu_flu)
            self.publish_odom_path(position_enu, quat_enu_flu)

    def get_distance_to_start(self):
        if self.current_state is None or self.start_position is None:
            return float("inf")
        return np.linalg.norm(self.current_state[0:3] - self.start_position)

    # -------------------------------------------------------------------------- #
    # 性能指标发布
    # -------------------------------------------------------------------------- #

    def _publish_performance_metrics(self, solve_time):
        """发布 NMPC 性能指标

        Args:
            solve_time: NMPC 求解时间 (秒)
        """
        import time

        # 发布求解时间 (毫秒)
        solve_time_ms = solve_time * 1000.0
        self._last_solve_time_ms = solve_time_ms
        self.solve_time_pub.publish(Float32(solve_time_ms))

        # 计算并发布控制频率
        current_time = time.time()
        if self.last_control_time is not None:
            dt = current_time - self.last_control_time
            if dt > 0:
                freq = 1.0 / dt
                self.control_freq_window.append(freq)
                # 保持最近 50 个样本
                if len(self.control_freq_window) > 50:
                    self.control_freq_window.pop(0)
                # 发布平均频率
                avg_freq = sum(self.control_freq_window) / len(self.control_freq_window)
                self.control_freq_pub.publish(Float32(avg_freq))

        self.last_control_time = current_time

    # -------------------------------------------------------------------------- #
    # 终端状态仪表盘
    # -------------------------------------------------------------------------- #

    def _init_display(self):
        """初始化终端状态显示 (2Hz, 极低开销)"""
        self._status_line_count = 0
        self._display_enabled = rospy.get_param("~status_display", True)
        if self._display_enabled:
            self._display_timer = rospy.Timer(rospy.Duration(0.5), self._display_status)

    def _get_current_stage(self):
        """从现有状态推导当前阶段 (无额外开销)"""
        if self.current_state is None:
            return "NO ODOM"
        if not self.control_active:
            return "STANDBY"
        if self.rc_mode == ControlMode.POSITION:
            return "POS CTRL"
        if not self.takeoff_done:
            return "TAKEOFF"
        if not self.trajectory_received:
            return "HOVER"
        if not self.trajectory_started:
            return "GOTO START"
        return "TRACKING"

    def _display_status(self, event):
        """终端状态仪表盘 (2Hz)

        使用 stdout + ANSI 转义码原地刷新，rospy 日志走 stderr 互不干扰。
        """
        fcu = (self.current_mode or "N/A")[:12]
        armed_s = "Y" if self.armed else "N"
        odom_s  = "Y" if self.current_state is not None else "N"
        rc_s    = "TRAJ" if self.rc_mode == ControlMode.TRAJECTORY else "POS"
        stage   = self._get_current_stage()

        if self.current_state is not None:
            p = self.current_state[0:3]
            v = self.current_state[7:10]
            pos_s = "{:6.2f} {:6.2f} {:6.2f}".format(p[0], p[1], p[2])
            vel_s = "{:5.2f} {:5.2f} {:5.2f}".format(v[0], v[1], v[2])
        else:
            pos_s = "  --     --     --  "
            vel_s = "  --    --    --  "

        freq_w = self.control_freq_window
        freq = sum(freq_w) / len(freq_w) if freq_w else 0.0
        solve = self._last_solve_time_ms

        W = 48
        bar = "-" * (W - 2)
        lines = [
            "+" + bar + "+",
            "|" + "NMPC Control Dashboard".center(W - 2) + "|",
            "+" + bar + "+",
            "| FCU: {:<10s} Armed:{} Odom:{}        |".format(fcu, armed_s, odom_s),
            "| RC: {:<5s} Stage: {:<16s}       |".format(rc_s, stage),
            "| Pos: {:<30s}       |".format(pos_s),
            "| Vel: {:<30s}       |".format(vel_s),
            "| Solve:{:5.1f}ms  Freq:{:5.1f}Hz              |".format(solve, freq),
            "+" + bar + "+",
        ]

        # ANSI: move cursor up to overwrite previous block
        if self._status_line_count > 0:
            sys.stdout.write("\033[{}A".format(self._status_line_count))
        for line in lines:
            sys.stdout.write("\033[K" + line + "\n")
        sys.stdout.flush()
        self._status_line_count = len(lines)

    # -------------------------------------------------------------------------- #
    # 主控制循环
    # -------------------------------------------------------------------------- #

    def control_callback(self, event):
        if self.current_state is None:
            mode_name = "轨迹跟踪" if self.rc_mode == ControlMode.TRAJECTORY else "位置控制"
            rospy.logwarn_throttle(2.0, "⚠️ 未收到里程计数据，禁止切入 Offboard！当前遥控器模式: %s | odom 话题: %s",
                                   mode_name, self.odom_topic)
            return  # 不发送任何 setpoint，PX4 将拒绝 Offboard 切换

        if not self.control_active:
            self.send_zero_commands()
            return

        try:
            if self.rc_mode == ControlMode.POSITION:
                self._run_position_control_mode()
            else:
                self._run_trajectory_mode()
        except Exception as e:
            rospy.logerr_throttle(1.0, "控制计算错误: %s", str(e))
            self.send_zero_commands()

    # -------------------------------------------------------------------------- #
    # 位置控制模式
    # -------------------------------------------------------------------------- #

    def _run_position_control_mode(self):
        self.mode_pub.publish("POSITION_CTRL")

        if self.current_state is None or self.pos_ctrl_position is None:
            self._init_position_control_mode()
            return

        roll_ch = self.rc_channels[self.RC_CHANNELS["ROLL"]]
        pitch_ch = self.rc_channels[self.RC_CHANNELS["PITCH"]]
        throttle_ch = self.rc_channels[self.RC_CHANNELS["THROTTLE"]]
        yaw_ch = self.rc_channels[self.RC_CHANNELS["YAW"]]

        roll_norm = (roll_ch - 1500) / 500.0
        pitch_norm = (pitch_ch - 1500) / 500.0
        throttle_norm = (throttle_ch - 1500) / 500.0
        yaw_norm = (yaw_ch - 1500) / 500.0

        deadzone = 0.1
        if abs(roll_norm) < deadzone: roll_norm = 0.0
        if abs(pitch_norm) < deadzone: pitch_norm = 0.0
        if abs(throttle_norm) < deadzone: throttle_norm = 0.0
        if abs(yaw_norm) < deadzone: yaw_norm = 0.0

        vel_body_x = -pitch_norm * self.pos_ctrl_max_vel_xy
        vel_body_y = roll_norm * self.pos_ctrl_max_vel_xy
        vel_z = throttle_norm * self.pos_ctrl_max_vel_z
        yaw_rate = -yaw_norm * self.pos_ctrl_max_yaw_rate

        cos_yaw = np.cos(self.pos_ctrl_yaw)
        sin_yaw = np.sin(self.pos_ctrl_yaw)
        vel_world_x = vel_body_x * cos_yaw - vel_body_y * sin_yaw
        vel_world_y = vel_body_x * sin_yaw + vel_body_y * cos_yaw

        dt = 0.01
        self.pos_ctrl_position[0] += vel_world_x * dt
        self.pos_ctrl_position[1] += vel_world_y * dt
        self.pos_ctrl_position[2] += vel_z * dt
        self.pos_ctrl_yaw += yaw_rate * dt
        self.pos_ctrl_position[2] = np.clip(self.pos_ctrl_position[2], 0.3, 10.0)

        target_quat = self.flat_to_quat(self.pos_ctrl_position, np.zeros(3), self.pos_ctrl_yaw)
        target_state = np.zeros(10, dtype=float)
        target_state[0:3] = self.pos_ctrl_position
        target_state[3:7] = target_quat
        target_state[7:10] = np.array([vel_world_x, vel_world_y, vel_z], dtype=float)

        _dt, control_input, all_control = self.controller.nmpc_state_control(self.current_state, target_state)

        if self.debug_flag == 1 and all_control is not None:
            self.publish_nmpc_prediction_path(all_control)

        self._publish_control(control_input)

        # 发布性能指标
        self._publish_performance_metrics(_dt)

        if self.debug_flag == 1:
            self._publish_traj_point(self.pos_ctrl_position, target_quat)

    # -------------------------------------------------------------------------- #
    # 轨迹跟踪模式
    # -------------------------------------------------------------------------- #

    def _run_trajectory_mode(self):
        if not self.takeoff_done:
            target_state = self._get_takeoff_target_state()
            self.mode_pub.publish("TAKEOFF")
            # 起飞阶段使用单点控制
            _dt, control_input, all_control = self.controller.nmpc_state_control(self.current_state, target_state)
        else:
            if not self.trajectory_received:
                rospy.logwarn_throttle(2.0, "等待轨迹消息...")
                target_state = self._get_hover_target_state()
                self.mode_pub.publish("HOVER")
                # 悬停阶段使用单点控制
                _dt, control_input, all_control = self.controller.nmpc_state_control(self.current_state, target_state)
            else:
                target_state = self._update_trajectory_point()
                if self.trajectory_started:
                    self.mode_pub.publish("TRACKING")

                    # ★ 可配置的控制模式切换 ★
                    if not self.use_trajectory_preview:
                        # 旧方法：始终使用单点控制
                        _dt, control_input, all_control = self.controller.nmpc_state_control(
                            self.current_state, target_state)
                    else:
                        # ========================================================================
                        # 新方法：使用轨迹预览控制
                        # 优化：添加加速度阈值检查，提醒高加速度情况下的外推误差
                        # ========================================================================
                        acc_mag = np.linalg.norm(self.target_acceleration)

                        # 警告：加速度超过阈值时，外推精度可能降低
                        if acc_mag > self.acc_threshold:
                            rospy.logwarn_throttle(5.0,
                                "⚠️ 高加速度检测: |acc|=%.2f m/s² > %.2f m/s² (阈值) - "
                                "轨迹外推误差可能增大，建议降低速度或检查轨迹平滑性",
                                acc_mag, self.acc_threshold)

                        # 生成未来轨迹并执行预览控制
                        trajectory_refs = self._generate_future_trajectory(target_state)
                        _dt, control_input, all_control = self.controller.nmpc_trajectory_control(
                            self.current_state, trajectory_refs, yaw_dot_ref=self.target_yaw_dot)
                else:
                    self.mode_pub.publish("GOTO_START")
                    # 前往起点时使用单点控制
                    _dt, control_input, all_control = self.controller.nmpc_state_control(self.current_state, target_state)

        if self.debug_flag == 1 and all_control is not None:
            self.publish_nmpc_prediction_path(all_control)

        self._publish_control(control_input)

        # 发布性能指标
        self._publish_performance_metrics(_dt)

    # ========================================================================
    # 轨迹预览生成函数 - 基于当前PVA外推未来N+1步
    # ========================================================================
    def _generate_future_trajectory(self, current_target_state):
        """
        基于当前的微分平坦信息(pos, vel, acc)外推未来轨迹
        
        使用泰勒展开: 
            pos(t+dt) = pos(t) + vel(t)*dt + 0.5*acc(t)*dt^2
            vel(t+dt) = vel(t) + acc(t)*dt
        
        对于圆/椭圆等平滑轨迹，在0.5秒内这个近似是足够准确的
        
        Args:
            current_target_state: 当前目标状态 (10,)
            
        Returns:
            trajectory_refs: 未来N+1步参考轨迹 (N+1, 10)
        """
        N = self.preview_N
        dt = self.preview_dt
        
        # 初始化轨迹数组
        trajectory_refs = np.zeros((N + 1, 10), dtype=float)
        
        # 获取当前目标信息 (从PositionCommand消息)
        pos = current_target_state[0:3].copy()
        quat = current_target_state[3:7].copy()
        vel = self.target_velocity.copy()
        acc = self.target_acceleration.copy()
        jerk = self.target_jerk.copy()  # 使用jerk进行加速度外推
        yaw = self.target_yaw
        yaw_dot = self.target_yaw_dot

        # 外推未来N+1步
        for i in range(N + 1):
            t = i * dt
            
            # 加速度外推 (使用jerk)
            # a(t) = a0 + jerk * t
            future_acc = acc + jerk * t
            
            # 位置外推 (泰勒展开，包含jerk项)
            # p(t) = p0 + v0*t + 0.5*a0*t^2 + (1/6)*jerk*t^3
            future_pos = pos + vel * t + 0.5 * acc * t * t + (1.0/6.0) * jerk * t * t * t
            
            # 速度外推 (包含jerk项)
            # v(t) = v0 + a0*t + 0.5*jerk*t^2
            future_vel = vel + acc * t + 0.5 * jerk * t * t
            
            # yaw外推
            future_yaw = yaw + yaw_dot * t
            
            # yaw_dir外推
            future_yaw_dir = self.yaw_dir  # 简化：使用当前yaw_dir
            
            # 姿态计算 (微分平坦) - 使用外推后的加速度
            if self.use_flat_traj:
                future_quat = self.flat_to_quat(future_pos, future_acc, future_yaw, future_yaw_dir)
            else:
                future_quat = quat
            
            # 填充参考状态
            trajectory_refs[i, 0:3] = future_pos
            trajectory_refs[i, 3:7] = future_quat
            trajectory_refs[i, 7:10] = future_vel
        
        return trajectory_refs

    def _publish_control(self, control_input):
        control_input = np.array(control_input).flatten()
        control_input[0] = (control_input[0] / 4.0 - Bt) / At

        if self.debug_flag == 1:
            rospy.loginfo_throttle(5.0, "控制输入: thrust=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f",
                                   control_input[0], control_input[1], control_input[2], control_input[3])

        control_msg = AttitudeTarget()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.header.frame_id = "FCU"
        control_msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        control_msg.thrust = float(control_input[0])
        control_msg.body_rate.x = float(control_input[1])
        control_msg.body_rate.y = float(control_input[2])
        control_msg.body_rate.z = float(control_input[3])
        self.ctrl_FCU_pub.publish(control_msg)

    def _get_hover_target_state(self):
        target_state = np.zeros(10, dtype=float)
        if self.current_state is None:
            return target_state

        if self.takeoff_done and self.takeoff_position is not None:
            pos_ref = self.takeoff_position
            quat_ref = self.takeoff_quat if self.takeoff_quat is not None else self.current_state[3:7]
        else:
            if self.hover_position is None:
                self.hover_position = self.current_state[0:3].copy()
                self.hover_quat = self.current_state[3:7].copy()
                rospy.loginfo("锁定悬停参考点: [%.2f, %.2f, %.2f]",
                              self.hover_position[0], self.hover_position[1], self.hover_position[2])
            pos_ref = self.hover_position
            quat_ref = self.hover_quat

        target_state[0:3] = pos_ref
        target_state[3:7] = quat_ref
        target_state[7:10] = np.zeros(3, dtype=float)

        if self.debug_flag == 1:
            self._publish_traj_point(pos_ref, quat_ref)

        return target_state

    def _get_takeoff_target_state(self):
        """起飞阶段目标状态（带速度限制的平滑起飞）"""
        target_state = np.zeros(10, dtype=float)
        if self.current_state is None:
            return target_state

        cur_pos = self.current_state[0:3]
        cur_vel = self.current_state[7:10] if len(self.current_state) >= 10 else np.zeros(3)
        cur_time = rospy.Time.now().to_sec()

        # 初始化起飞参数
        if self.takeoff_position is None:
            self.takeoff_position = cur_pos.copy()
            self.takeoff_position[2] = self.takeoff_height  # 最终目标高度
            self.takeoff_quat = np.array([1, 0, 0, 0], dtype=float)
            self.takeoff_start_time = cur_time
            self.takeoff_start_z = cur_pos[2]
            rospy.loginfo("设置起飞目标: [%.2f, %.2f, %.2f] (基于当前位置), 最大速度: %.2f m/s",
                          self.takeoff_position[0], self.takeoff_position[1], self.takeoff_position[2],
                          self.takeoff_vz_max)

        # ========================================================================
        # 优化：平滑起飞减速逻辑（避免速度突变导致的超调和振荡）
        # ========================================================================
        elapsed_time = cur_time - self.takeoff_start_time
        delta_z = self.takeoff_height - self.takeoff_start_z

        # 计算剩余距离
        distance_to_final = self.takeoff_height - cur_pos[2]

        # 定义减速区域（提前开始减速，避免急刹车）
        slow_down_distance = 0.3  # 提前 0.3m 开始减速（可根据 takeoff_vz_max 调整）

        # 三段式速度规划：加速 → 匀速 → 减速
        if distance_to_final > slow_down_distance:
            # 阶段 1+2: 加速/匀速阶段
            target_vz = self.takeoff_vz_max
            # 修复：目标高度应该是最终目标，而不是仅前瞻 20ms
            # 这样 NMPC 才能看到完整的高度误差并产生足够推力
            current_target_z = self.takeoff_height
        elif distance_to_final > 0.0:
            # 阶段 3: 线性减速阶段（从 vz_max 平滑降到 0）
            # target_vz = vz_max * (distance / slow_down_distance)
            decel_ratio = distance_to_final / slow_down_distance
            target_vz = self.takeoff_vz_max * decel_ratio
            # 确保最小速度不低于 0.05 m/s（避免过慢悬停）
            target_vz = max(target_vz, 0.05)
            # 修复：减速阶段也使用最终目标高度
            current_target_z = self.takeoff_height
        else:
            # 已到达或超过目标高度
            target_vz = 0.0
            current_target_z = self.takeoff_height

        # 判断起飞完成（到达目标高度附近且速度接近零）
        velocity_settled = abs(cur_vel[2]) < 0.1  # 速度小于 0.1 m/s
        position_reached = abs(distance_to_final) < self.takeoff_distance_threshold

        if position_reached and velocity_settled:
            if not self.takeoff_done:
                rospy.loginfo(">>> 起飞完成！耗时=%.1f s, 最终高度=%.2f m, Vz=%.3f m/s <<<",
                              elapsed_time, cur_pos[2], cur_vel[2])
            self.takeoff_done = True
            current_target_z = self.takeoff_height
            target_vz = 0.0
        else:
            rospy.loginfo_throttle(1.0, "起飞中... 高度=%.2f/%.2f, 剩余=%.2f m, Vz=%.2f/%.2f m/s",
                                   cur_pos[2], self.takeoff_height, distance_to_final,
                                   cur_vel[2], target_vz)

        # 设置目标状态
        target_state[0:2] = self.takeoff_position[0:2]  # XY保持初始位置
        target_state[2] = current_target_z              # Z逐渐上升
        target_state[3:7] = self.takeoff_quat           # 姿态
        target_state[7:9] = np.zeros(2, dtype=float)    # Vx, Vy = 0
        target_state[9] = target_vz                      # Vz渐变

        if self.debug_flag == 1:
            self._publish_traj_point(np.array([self.takeoff_position[0], self.takeoff_position[1], current_target_z]), 
                                     self.takeoff_quat)

        return target_state

    def _publish_traj_point(self, position, quat):
        if self.debug_flag != 1 or self.traj_pose_pub is None:
            return

        now = rospy.Time.now()
        if (now - self._last_traj_pub_time).to_sec() < 0.1:
            return
        self._last_traj_pub_time = now

        stamp = now
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        pose_msg.pose.orientation.w = float(quat[0])
        pose_msg.pose.orientation.x = float(quat[1])
        pose_msg.pose.orientation.y = float(quat[2])
        pose_msg.pose.orientation.z = float(quat[3])

        self.traj_pose_pub.publish(pose_msg)
        self.traj_path_msg.header.stamp = stamp
        self.traj_path_msg.poses.append(pose_msg)

        if len(self.traj_path_msg.poses) > self.max_path_points:
            self.traj_path_msg.poses.pop(0)
        self.traj_path_pub.publish(self.traj_path_msg)

    def _update_trajectory_point(self):
        if not self.trajectory_started:
            dist_to_start = self.get_distance_to_start()
            if dist_to_start < self.start_threshold:
                self.trajectory_started = True
                rospy.loginfo("到达轨迹起点，开始跟踪!")
            else:
                rospy.loginfo_throttle(1.0, "前往轨迹起点... 距离=%.2f m", dist_to_start)
                target_state = np.zeros(10, dtype=float)
                if self.start_position is not None:
                    target_state[0:3] = self.start_position
                if self.start_quat is not None:
                    target_state[3:7] = self.start_quat
                if self.debug_flag == 1 and self.start_position is not None:
                    self._publish_traj_point(self.start_position, self.start_quat)
                return target_state

        if self.use_flat_traj:
            target_state = self.flat_to_state(
                self.target_position, self.target_velocity, self.target_acceleration, 
                self.target_yaw, self.yaw_dir)
            quat = target_state[3:7]
        else:
            target_state = np.zeros(10, dtype=float)
            target_state[0:3] = self.target_position
            target_state[3:7] = self.target_quat
            target_state[7:10] = self.target_velocity
            quat = self.target_quat

        if self.debug_flag == 1:
            self._publish_traj_point(self.target_position, quat)

        return target_state

    def flat_to_quat(self, position, acc, yaw, yaw_dir=None):
        """
        微分平坦到四元数的转换
        
        参考: Px4Ctrl SO3Control实现
        
        Args:
            position: 位置 (未使用，保留接口)
            acc: 加速度 (3,)
            yaw: yaw角度 (当yaw_dir为None时使用)
            yaw_dir: yaw方向向量 (3,)，优先使用这个
            
        Returns:
            quat: 四元数 [w, x, y, z]
        """
        g = 9.81
        e3 = np.array([0.0, 0.0, 1.0])
        
        # 计算推力方向 (body z轴)
        # zu = acc + g*e3
        a_tot = np.array(acc, dtype=float) + g * e3
        norm_a = np.linalg.norm(a_tot)
        if norm_a < 1e-6:
            a_tot = g * e3
            norm_a = g
        
        # body z轴 = 归一化的推力方向
        b3 = a_tot / norm_a
        
        # 使用yaw_dir或从yaw角计算方向
        if yaw_dir is not None and np.linalg.norm(yaw_dir) > 1e-6:
            # 使用yaw_dir向量 (与Px4Ctrl一致)
            # dir_xb = dir - dir.dot(b3) * b3  (投影到与b3垂直的平面)
            dir_normalized = yaw_dir / np.linalg.norm(yaw_dir)
            dir_dot_zb = np.dot(dir_normalized, b3)
            dir_xb = dir_normalized - dir_dot_zb * b3
            norm_dir_xb = np.linalg.norm(dir_xb)
            
            if norm_dir_xb > 1e-6:
                # 使用yaw_dir投影作为x轴方向
                b1c = dir_xb / norm_dir_xb
            else:
                # 退化情况：使用yaw角
                b1c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        else:
            # 回退到yaw角计算
            b1c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        
        # 计算body y轴 = b3 x b1c
        b2 = np.cross(b3, b1c)
        norm_b2 = np.linalg.norm(b2)
        if norm_b2 < 1e-6:
            # 处理退化情况
            b1c = np.array([1.0, 0.0, 0.0])
            b2 = np.cross(b3, b1c)
            norm_b2 = np.linalg.norm(b2)
        b2 /= norm_b2
        
        # body x轴 = b2 x b3
        b1 = np.cross(b2, b3)

        # 构建旋转矩阵并转换为四元数
        R = np.column_stack((b1, b2, b3))
        T = np.eye(4)
        T[:3, :3] = R
        q_xyzw = tf_trans.quaternion_from_matrix(T)

        # ⚠️ 修复：添加四元数归一化（防止数值误差累积）
        quat = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)
        quat_norm = np.linalg.norm(quat)
        if quat_norm > 1e-6:
            quat /= quat_norm
        else:
            # 退化情况：返回单位四元数
            rospy.logwarn_throttle(5.0, "flat_to_quat: 四元数接近零，使用单位四元数")
            quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

        return quat

    def flat_to_state(self, position, velocity, acc, yaw, yaw_dir=None):
        """
        微分平坦信息转换为目标状态
        
        Args:
            position: 位置 (3,)
            velocity: 速度 (3,)
            acc: 加速度 (3,)
            yaw: yaw角度
            yaw_dir: yaw方向向量 (可选)
            
        Returns:
            target_state: [pos(3), quat(4), vel(3)] (10,)
        """
        quat = self.flat_to_quat(position, acc, yaw, yaw_dir)
        target_state = np.zeros(10, dtype=float)
        target_state[0:3] = np.array(position, dtype=float)
        target_state[3:7] = quat
        target_state[7:10] = np.array(velocity, dtype=float)
        return target_state

    def clear_paths(self):
        self.traj_path_msg.poses[:] = []
        self.odom_path_msg.poses[:] = []
        self.nmpc_pred_path_msg.poses[:] = []
        rospy.loginfo("已清空所有轨迹")


if __name__ == "__main__":
    try:
        node = ControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 节点被中断")
    except Exception as e:
        rospy.logerr("程序异常: %s", str(e))
    finally:
        rospy.loginfo("程序退出")
