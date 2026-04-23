#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
VRPN PoseStamped → nav_msgs/Odometry 转换节点

将动捕系统 (OptiTrack/Vicon) 通过 vrpn_client_ros 发布的 PoseStamped
转为 RL 控制节点所需的 Odometry（含线速度和角速度）。

速度估计方法: 有限差分 + 一阶低通滤波
"""

import rospy
import numpy as np
import tf.transformations as tf_trans

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class VrpnToOdom:

    def __init__(self):
        rospy.init_node("vrpn_to_odom", anonymous=False)

        # ---------- 参数 ----------
        self.vrpn_topic = rospy.get_param("~vrpn_topic",
                                          "/vrpn_client_node/drone/pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/vio/odometry")
        self.rate       = rospy.get_param("~rate", 200.0)           # Hz
        self.alpha      = rospy.get_param("~velocity_filter_alpha", 0.5)
        self.frame_id       = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")

        # ---------- 状态 ----------
        self.prev_pos  = None   # np.array (3,)
        self.prev_quat = None   # np.array (4,)  x,y,z,w (tf 格式)
        self.prev_time = None   # rospy.Time

        self.vel_filt      = np.zeros(3)
        self.ang_vel_filt  = np.zeros(3)

        self.first_msg = True

        # ---------- 发布 / 订阅 ----------
        self.pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        rospy.Subscriber(self.vrpn_topic, PoseStamped,
                         self._cb_pose, queue_size=10)

        rospy.loginfo("[vrpn_to_odom] 启动: %s → %s (alpha=%.2f, rate=%.0f Hz)",
                      self.vrpn_topic, self.odom_topic, self.alpha, self.rate)

    # ------------------------------------------------------------------
    def _cb_pose(self, msg):
        now = msg.header.stamp
        if now.to_sec() < 1e-3:
            now = rospy.Time.now()

        pos = np.array([msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z])
        # tf 格式: x,y,z,w
        quat = np.array([msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z,
                         msg.pose.orientation.w])

        if self.first_msg:
            self.prev_pos  = pos.copy()
            self.prev_quat = quat.copy()
            self.prev_time = now
            self.first_msg = False
            rospy.loginfo("[vrpn_to_odom] 首次收到 VRPN: [%.2f, %.2f, %.2f]",
                          pos[0], pos[1], pos[2])
            return

        # ---- 时间差 ----
        dt = (now - self.prev_time).to_sec()
        if dt < 1e-6:
            return

        # ---- 线速度（世界系有限差分 + 低通） ----
        vel_raw = (pos - self.prev_pos) / dt
        self.vel_filt = self.alpha * vel_raw + (1.0 - self.alpha) * self.vel_filt

        # ---- 角速度（机体系） ----
        # 四元数差分: dq = q_curr * q_prev^{-1}
        q_prev_inv = tf_trans.quaternion_inverse(self.prev_quat)
        dq = tf_trans.quaternion_multiply(quat, q_prev_inv)
        # 确保 w > 0（短弧旋转）
        if dq[3] < 0:
            dq = -dq
        # 旋转向量 ≈ 2 * [x, y, z] (小角度近似)
        angle = 2.0 * np.arccos(np.clip(dq[3], -1.0, 1.0))
        if abs(angle) < 1e-8:
            omega_world = np.zeros(3)
        else:
            axis = dq[:3] / np.sin(angle / 2.0)
            omega_world = axis * angle / dt

        # 转到机体系
        R_wb = tf_trans.quaternion_matrix(quat)[:3, :3]
        omega_body = R_wb.T @ omega_world

        self.ang_vel_filt = self.alpha * omega_body + (1.0 - self.alpha) * self.ang_vel_filt

        # ---- 发布 Odometry ----
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id  = self.child_frame_id

        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        # Odometry 用 geometry_msgs/Quaternion (x,y,z,w)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        odom.twist.twist.linear.x  = self.vel_filt[0]
        odom.twist.twist.linear.y  = self.vel_filt[1]
        odom.twist.twist.linear.z  = self.vel_filt[2]
        odom.twist.twist.angular.x = self.ang_vel_filt[0]
        odom.twist.twist.angular.y = self.ang_vel_filt[1]
        odom.twist.twist.angular.z = self.ang_vel_filt[2]

        self.pub.publish(odom)

        # ---- 更新缓存 ----
        self.prev_pos  = pos.copy()
        self.prev_quat = quat.copy()
        self.prev_time = now


if __name__ == "__main__":
    try:
        node = VrpnToOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
