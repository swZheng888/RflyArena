#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
odom_repub.py

功能：
1. 订阅 PX4 的 /fmu/out/vehicle_odometry（或者可选使用 RflySim 真值）；
2. 调用 transforms.FrameTransformer 做 NED/FRD -> ENU/FLU 转换；
3. 发布给 RViz：
   - /odom      : nav_msgs/Odometry
   - /uav_pose  : geometry_msgs/PoseStamped
   - /uav_path  : nav_msgs/Path
   - TF: map -> base_link
4. 将最新 ENU 里程计状态写入全局 odom_state，方便同一进程其他代码直接读取。
"""

from __future__ import annotations

import threading
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster

from utils.config import USE_TRUE_STATE_FROM_MAVCTRL, START_COPTER_ID
from utils.transforms import FrameTransformer


if USE_TRUE_STATE_FROM_MAVCTRL:
    import PX4MavCtrlV4
    import VisionCaptureApi
    import ReqCopterSim

    VisionCaptureApi.isEnableRosTrans = True
    mavCtrl = PX4MavCtrlV4.PX4MavCtrler()
    mavCtrl.InitTrueDataLoop()
    req = ReqCopterSim.ReqCopterSim()
    TARGET_IP = req.getSimIpID(START_COPTER_ID)
    print(f"[odom] Use RflySim true state, TARGET_IP={TARGET_IP}")
else:
    TARGET_IP = None


# ======================================================
# 全局里程计状态：OdomState（给同进程其他代码直接 get）
# ======================================================


class OdomState:
    """
    一个线程安全的全局里程计缓冲区。
    任何地方都可以通过 odom_state.get() 拿到最新 ENU 位置 / 姿态 / 速度。
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._position = np.zeros(3)
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w,x,y,z]
        self._velocity = np.zeros(3)
        self._ang_vel_flu = np.zeros(3)
        self._stamp = 0.0  # 秒

    def update(self, position, quaternion, velocity, ang_vel_flu, stamp: float) -> None:
        """
        position: 3 元数组 [x,y,z] (ENU)
        quaternion: 4 元数组 [w,x,y,z] (ENU-FLU)
        velocity: 3 元数组 [vx,vy,vz] (ENU)
        stamp: float 秒
        """
        position = np.asarray(position, dtype=float)
        quaternion = np.asarray(quaternion, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        ang_vel_flu = np.asarray(velocity, dtype=float)
        with self._lock:
            self._position[:] = position
            self._quaternion[:] = quaternion
            self._velocity[:] = velocity
            self._ang_vel_flu[:] = ang_vel_flu
            self._stamp = float(stamp)

    def get(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray,np.asarray,float]:
        """
        返回 (position, quaternion, velocity, stamp)
        """
        with self._lock:
            pos = self._position.copy()
            quat = self._quaternion.copy()
            vel = self._velocity.copy()
            gyro =self._ang_vel_flu.copy()
            t = self._stamp
        return pos, quat, vel,gyro, t


# 模块级单例：别的文件 / 类可以 `from odom_repub import odom_state` 然后 `odom_state.get()`
odom_state = OdomState()


# ======================================================
# 里程计节点：OdomNode
# ======================================================


class OdomNode(Node):
    """
    里程计处理 + RViz 可视化 + 更新全局 odom_state：

    - 订阅 /fmu/out/vehicle_odometry 或 RflySim 真值；
    - 调用 FrameTransformer 做 NED/FRD -> ENU/FLU 转换；
    - 发布 /odom, /uav_pose, /uav_path, TF；
    - 把 ENU 位姿/速度写入 odom_state。
    """

    def __init__(self) -> None:
        super().__init__("odom_node")

        self.tf_util = FrameTransformer()

        # 发布给 RViz 的几个话题
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/uav_pose", 10)
        self.path_pub = self.create_publisher(Path, "/uav_path", 10)

        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 轨迹 Path
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # 订阅 PX4 里程计
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if not USE_TRUE_STATE_FROM_MAVCTRL:
            self.create_subscription(
                VehicleOdometry,
                "/fmu/out/vehicle_odometry",
                self.odom_callback,
                qos_sub,
            )
            self.get_logger().info("[odom] Using PX4 /fmu/out/vehicle_odometry")
        else:
            # 如果用 RflySim 真值，则定时从 mavCtrl 读取
            self.timer = self.create_timer(0.01, self.odom_timer_loop)
            self.get_logger().info("[odom] Using RflySim true state via PX4MavCtrlV4")

    # ---------------- 核心处理 ----------------

    def odom_timer_loop(self) -> None:
        """
        定时从 mavCtrl 读取真值，转成 ENU/FLU 并更新 odom_state。
        仅在 USE_TRUE_STATE_FROM_MAVCTRL=True 时使用。
        """
        position_ned = np.array(
            [mavCtrl.truePosNED[0], mavCtrl.truePosNED[1], mavCtrl.truePosNED[2]]
        )
        velocity_ned = np.array(
            [mavCtrl.trueVelNED[0], mavCtrl.trueVelNED[1], mavCtrl.trueVelNED[2]]
        )
        quat_ned_frd = np.array(
            [
                mavCtrl.trueAngQuatern[0],
                mavCtrl.trueAngQuatern[1],
                mavCtrl.trueAngQuatern[2],
                mavCtrl.trueAngQuatern[3],
            ]
        )
        ang_vel_frd = np.array(
            [mavCtrl.trueAngRate[0], mavCtrl.trueAngRate[1], mavCtrl.trueAngRate[2]]
        )

        self.process_state(position_ned, velocity_ned, quat_ned_frd, ang_vel_frd)

    def odom_callback(self, msg: VehicleOdometry) -> None:
        """
        订阅 PX4 /fmu/out/vehicle_odometry 的回调。
        """
        position_ned = np.array(msg.position, dtype=float)
        velocity_ned = np.array(msg.velocity, dtype=float)
        quat_ned_frd = np.array(msg.q, dtype=float)
        ang_vel_frd = np.array(msg.angular_velocity, dtype=float)

        self.process_state(position_ned, velocity_ned, quat_ned_frd, ang_vel_frd)

    def process_state(
        self,
        position_ned: np.ndarray,
        velocity_ned: np.ndarray,
        quat_ned_frd: np.ndarray,
        ang_vel_frd: np.ndarray,
    ) -> None:
        """
        统一的状态处理接口：
        - 将 NED/FRD 转换到 ENU/FLU；
        - 更新 odom_state；
        - 发布 /odom, /uav_pose, /uav_path, TF。
        """
        # 1) NED/FRD -> ENU/FLU
        pos_enu = self.tf_util.ned_to_enu_position(position_ned)
        vel_enu = self.tf_util.ned_to_enu_position(velocity_ned)
        quat_enu_flu = self.tf_util.nedfrd_to_enuflu_rotation(quat_ned_frd)
        _ang_vel_flu = self.tf_util.frd_to_flu_ang_vel(ang_vel_frd)  # 目前没用到，可调试用

        # 2) 当前时间
        now_ros = self.get_clock().now()
        stamp_msg = now_ros.to_msg()
        stamp_sec = now_ros.nanoseconds * 1e-9

        # 3) 更新全局 odom_state
        odom_state.update(pos_enu, quat_enu_flu, vel_enu,_ang_vel_flu, stamp_sec)

        # 4) 发布 /odom
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp_msg
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = float(pos_enu[0])
        odom_msg.pose.pose.position.y = float(pos_enu[1])
        odom_msg.pose.pose.position.z = float(pos_enu[2])
        odom_msg.pose.pose.orientation.w = float(quat_enu_flu[0])
        odom_msg.pose.pose.orientation.x = float(quat_enu_flu[1])
        odom_msg.pose.pose.orientation.y = float(quat_enu_flu[2])
        odom_msg.pose.pose.orientation.z = float(quat_enu_flu[3])
        odom_msg.twist.twist.linear.x = float(vel_enu[0])
        odom_msg.twist.twist.linear.y = float(vel_enu[1])
        odom_msg.twist.twist.linear.z = float(vel_enu[2])
        self.odom_pub.publish(odom_msg)

        # 5) 发布 /uav_pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp_msg
        pose_msg.header.frame_id = "map"
        pose_msg.pose = odom_msg.pose.pose
        self.pose_pub.publish(pose_msg)

        # 6) 发布 /uav_path
        self.path_msg.header.stamp = stamp_msg
        self.path_msg.poses.append(pose_msg)
        self.path_pub.publish(self.path_msg)

        # 7) 发布 TF: map -> base_link
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp_msg
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = float(pos_enu[0])
        tf_msg.transform.translation.y = float(pos_enu[1])
        tf_msg.transform.translation.z = float(pos_enu[2])
        tf_msg.transform.rotation.w = float(quat_enu_flu[0])
        tf_msg.transform.rotation.x = float(quat_enu_flu[1])
        tf_msg.transform.rotation.y = float(quat_enu_flu[2])
        tf_msg.transform.rotation.z = float(quat_enu_flu[3])
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
