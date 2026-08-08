#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rflysim_odom_node.py

RflySim 里程计发布节点（NMPC 版本）

从 RflySim/CopterSim 获取无人机真值状态，直接发布 NED 坐标系数据

发布话题:
- /mavros/local_position/odom: Odometry

坐标系:
- 输入/输出: NED (RflySim 真值数据)
"""

import rospy
import numpy as np
import math
import sys
import VisionCaptureApi
import ReqCopterSim
import RflyRosStart
import PX4MavCtrlV4
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf import TransformBroadcaster


class RflySimOdomNode:
    """RflySim 里程计节点（NED 坐标系，原样发布）"""

    def __init__(self):
        rospy.init_node('rflysim_odom_node', anonymous=True)

        self._init_parameters()
        self._init_rflysim()
        self._init_publishers()
        self._init_tf_broadcaster()

        rospy.loginfo("=" * 60)
        rospy.loginfo("RflySim 里程计节点已启动 (NED + ENU)")
        rospy.loginfo("订阅飞机 ID: %d", self.copter_id)
        rospy.loginfo("目标 IP: %s", self.target_ip)
        rospy.loginfo("发布话题 (NED): %s", self.odom_topic)
        rospy.loginfo("发布话题 (ENU): %s", self.odom_enu_topic)
        rospy.loginfo("发布频率: %.1f Hz", self.rate)
        rospy.loginfo("=" * 60)

    def _init_parameters(self):
        """初始化参数"""
        self.copter_id = rospy.get_param('~copter_id', 1)
        self.rate = rospy.get_param('~rate', 200.0)
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')
        self.odom_topic = rospy.get_param('~odom_topic', '/vio/odometry')
        self.odom_enu_topic = rospy.get_param('~odom_enu_topic', '/vio/odometry_enu')

    def _init_rflysim(self):
        """初始化 RflySim 连接"""
        self.mavCtrl = PX4MavCtrlV4.PX4MavCtrler(self.copter_id)
        self.mavCtrl.InitTrueDataLoop()
        if rospy.get_param('~init_mavlink', True):
            self.mavCtrl.InitMavLoop()  # 同时初始化 MAVLink 接口以获取 uavAngRate
        else:
            rospy.loginfo("MAVLink 由外部 MAVROS 负责，里程计节点不占用 20101")

        listen_sim_timestamp = rospy.get_param('~listen_sim_timestamp', True)
        self.req = ReqCopterSim.ReqCopterSim(listen_sim_timestamp)
        self.target_ip = self.req.getSimIpID(self.copter_id)

        rospy.loginfo("请求 RflySim 数据，飞机 ID: %d", self.copter_id)
        self.req.sendReSimIP(self.copter_id)

        if not (RflyRosStart.isLinux and RflyRosStart.isRosOk):
            rospy.logerr("此节点需要在 Linux + ROS 环境下运行")
            sys.exit(1)

        self.ros_start = RflyRosStart.RflyRosStart(self.copter_id, self.target_ip)

        self.vis = None
        if rospy.get_param('~start_vision_capture', True):
            self.vis = VisionCaptureApi.VisionCaptureApi(self.target_ip)
            self.vis.jsonLoad()
            self.vis.sendReqToUE4(0, self.target_ip)
            self.vis.startImgCap()
            self.vis.sendImuReqCopterSim(self.copter_id, self.target_ip)
        else:
            rospy.loginfo("视觉采集由外部节点负责，里程计节点不占用图像端口")

        rospy.loginfo("RflySim 连接已建立")

    def _init_publishers(self):
        """初始化发布者"""
        self.odom_pub = rospy.Publisher(
            self.odom_topic,
            Odometry,
            queue_size=10
        )
        self.odom_enu_pub = rospy.Publisher(
            self.odom_enu_topic,
            Odometry,
            queue_size=10
        )

    def _init_tf_broadcaster(self):
        """初始化 TF 广播器"""
        self.tf_broadcaster = TransformBroadcaster()

    def _publish_odom(self):
        """发布里程计消息 - NED 坐标系原样发布"""
        now = rospy.Time.now()

        pos_ned = np.array([
            self.mavCtrl.truePosNED[0],
            self.mavCtrl.truePosNED[1],
            self.mavCtrl.truePosNED[2]
        ])

        vel_ned = np.array([
            self.mavCtrl.trueVelNED[0],
            self.mavCtrl.trueVelNED[1],
            self.mavCtrl.trueVelNED[2]
        ])

        q_ned = np.array([
            self.mavCtrl.trueAngQuatern[0],
            self.mavCtrl.trueAngQuatern[1],
            self.mavCtrl.trueAngQuatern[2],
            self.mavCtrl.trueAngQuatern[3]
        ])

        # 获取角速度 (FRD 机体系) - 从 MAVLink 接口获取
        ang_vel_frd = np.array([
            self.mavCtrl.uavAngRate[0],
            self.mavCtrl.uavAngRate[1],
            self.mavCtrl.uavAngRate[2]
        ])

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = pos_ned[0]
        odom_msg.pose.pose.position.y = pos_ned[1]
        odom_msg.pose.pose.position.z = pos_ned[2]

        odom_msg.pose.pose.orientation.x = q_ned[1]
        odom_msg.pose.pose.orientation.y = q_ned[2]
        odom_msg.pose.pose.orientation.z = q_ned[3]
        odom_msg.pose.pose.orientation.w = q_ned[0]

        odom_msg.twist.twist.linear.x = vel_ned[0]
        odom_msg.twist.twist.linear.y = vel_ned[1]
        odom_msg.twist.twist.linear.z = vel_ned[2]

        # NED版本发布 FRD 角速度
        odom_msg.twist.twist.angular.x = ang_vel_frd[0]
        odom_msg.twist.twist.angular.y = ang_vel_frd[1]
        odom_msg.twist.twist.angular.z = ang_vel_frd[2]

        odom_msg.pose.covariance = np.zeros(36).tolist()
        odom_msg.twist.covariance = np.zeros(36).tolist()

        self.odom_pub.publish(odom_msg)

        # 发布 ENU 版本（包含角速度）
        self._publish_odom_enu(pos_ned, vel_ned, q_ned, ang_vel_frd, now)

        self._publish_tf(pos_ned, q_ned)

    def _ned_to_enu_position(self, pos_ned):
        """NED -> ENU 位置转换"""
        return np.array([pos_ned[1], pos_ned[0], -pos_ned[2]])

    def _ned_to_enu_velocity(self, vel_ned):
        """NED -> ENU 速度转换"""
        return np.array([vel_ned[1], vel_ned[0], -vel_ned[2]])

    def _quat_mult(self, q1, q2):
        """Hamilton 四元数乘法 [w, x, y, z]"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ])

    def _ned_to_enu_quaternion(self, q_ned):
        """
        NED-FRD -> ENU-FLU 四元数转换
        q_ned: [w, x, y, z] 表示从NED到FRD机体系的旋转
        返回: [w, x, y, z] 表示从ENU到FLU机体系的旋转
        """
        sqrt2_2 = np.sqrt(2.0) / 2.0
        # NED->ENU 坐标系变换四元数
        Q_ENU_NED = np.array([0.0, sqrt2_2, sqrt2_2, 0.0])
        # FRD->FLU 机体系变换四元数 (绕x轴旋转180度)
        Q_FRD_FLU = np.array([0.0, 1.0, 0.0, 0.0])

        # q_enu_flu = Q_ENU_NED * q_ned * Q_FRD_FLU
        temp = self._quat_mult(q_ned, Q_FRD_FLU)
        q_enu_flu = self._quat_mult(Q_ENU_NED, temp)

        # 归一化
        return q_enu_flu / np.linalg.norm(q_enu_flu)

    def _frd_to_flu_angular_velocity(self, ang_vel_frd):
        """FRD -> FLU 角速度转换: [wx, wy, wz] -> [wx, -wy, -wz]"""
        return np.array([ang_vel_frd[0], -ang_vel_frd[1], -ang_vel_frd[2]])

    def _publish_odom_enu(self, pos_ned, vel_ned, q_ned, ang_vel_frd, stamp):
        """发布 ENU 坐标系里程计（包含角速度）"""
        pos_enu = self._ned_to_enu_position(pos_ned)
        vel_enu = self._ned_to_enu_velocity(vel_ned)
        q_enu = self._ned_to_enu_quaternion(q_ned)
        ang_vel_flu = self._frd_to_flu_angular_velocity(ang_vel_frd)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = pos_enu[0]
        odom_msg.pose.pose.position.y = pos_enu[1]
        odom_msg.pose.pose.position.z = pos_enu[2]

        odom_msg.pose.pose.orientation.x = q_enu[1]
        odom_msg.pose.pose.orientation.y = q_enu[2]
        odom_msg.pose.pose.orientation.z = q_enu[3]
        odom_msg.pose.pose.orientation.w = q_enu[0]

        odom_msg.twist.twist.linear.x = vel_enu[0]
        odom_msg.twist.twist.linear.y = vel_enu[1]
        odom_msg.twist.twist.linear.z = vel_enu[2]

        # FLU 机体系角速度
        odom_msg.twist.twist.angular.x = ang_vel_flu[0]
        odom_msg.twist.twist.angular.y = ang_vel_flu[1]
        odom_msg.twist.twist.angular.z = ang_vel_flu[2]

        odom_msg.pose.covariance = np.zeros(36).tolist()
        odom_msg.twist.covariance = np.zeros(36).tolist()

        self.odom_enu_pub.publish(odom_msg)

    def _publish_tf(self, position, orientation):
        """发布 TF 变换"""
        now = rospy.Time.now()

        translation = (position[0], position[1], position[2])
        rotation = (orientation[1], orientation[2], orientation[3], orientation[0])

        self.tf_broadcaster.sendTransform(translation, rotation, now, self.child_frame_id, self.frame_id)

    def run(self):
        """主循环"""
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            try:
                self._publish_odom()
            except Exception as e:
                rospy.logwarn_throttle(1.0, "发布里程计失败: %s", str(e))

            rate.sleep()


if __name__ == '__main__':
    try:
        node = RflySimOdomNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
