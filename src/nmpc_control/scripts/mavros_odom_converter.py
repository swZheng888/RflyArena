#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mavros_odom_converter.py

MAVROS 里程计坐标转换节点（Docker 版本）

订阅 MAVROS 里程计（NED），转换并发布为 ENU 坐标系
用于 Docker 容器内运行，不依赖 RFlySim 专用模块

订阅话题:
- /mavros/local_position/odom: 原始里程计 (NED)

发布话题:
- /vio/odometry_enu: 转换后里程计 (ENU)
"""

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class MavrosOdomConverter:
    """MAVROS 里程计 NED → ENU 转换器"""

    def __init__(self):
        rospy.init_node('mavros_odom_converter', anonymous=True)

        # 参数
        self.input_topic = rospy.get_param('~input_topic', '/mavros/local_position/odom')
        self.output_topic = rospy.get_param('~output_topic', '/vio/odometry_enu')
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_link')

        # 发布者
        self.odom_pub = rospy.Publisher(
            self.output_topic,
            Odometry,
            queue_size=10
        )

        # TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 订阅者
        rospy.Subscriber(
            self.input_topic,
            Odometry,
            self.odom_callback,
            queue_size=10
        )

        # 统计
        self.msg_count = 0

        rospy.loginfo("=" * 60)
        rospy.loginfo("MAVROS Odometry Converter (NED → ENU)")
        rospy.loginfo("订阅话题: %s", self.input_topic)
        rospy.loginfo("发布话题: %s", self.output_topic)
        rospy.loginfo("Frame: %s → %s", self.frame_id, self.child_frame_id)
        rospy.loginfo("=" * 60)

    def ned_to_enu_position(self, pos_ned):
        """NED 位置转换为 ENU"""
        # NED [x, y, z] → ENU [y, x, -z]
        return np.array([pos_ned[1], pos_ned[0], -pos_ned[2]])

    def ned_to_enu_velocity(self, vel_ned):
        """NED 速度转换为 ENU"""
        return np.array([vel_ned[1], vel_ned[0], -vel_ned[2]])

    def ned_to_enu_quaternion(self, q_ned):
        """
        NED-FRD 四元数转换为 ENU-FLU
        输入: [w, x, y, z] NED-FRD
        输出: [w, x, y, z] ENU-FLU
        """
        w, x, y, z = q_ned
        # NED-FRD → ENU-FLU 转换
        # 交换 x, y 并反转 z
        return np.array([w, y, x, -z])

    def odom_callback(self, msg):
        """里程计回调 - NED → ENU 转换"""
        try:
            # 提取 NED 位置
            pos_ned = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])

            # 提取 NED 速度
            vel_ned = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])

            # 提取 NED 四元数
            q_ned = np.array([
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ])

            # 提取角速度
            ang_vel_ned = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])

            # 坐标转换
            pos_enu = self.ned_to_enu_position(pos_ned)
            vel_enu = self.ned_to_enu_velocity(vel_ned)
            q_enu = self.ned_to_enu_quaternion(q_ned)
            ang_vel_enu = self.ned_to_enu_velocity(ang_vel_ned)

            # 构建 ENU 里程计消息
            odom_enu = Odometry()
            odom_enu.header.stamp = msg.header.stamp
            odom_enu.header.frame_id = self.frame_id
            odom_enu.child_frame_id = self.child_frame_id

            # 位置
            odom_enu.pose.pose.position.x = pos_enu[0]
            odom_enu.pose.pose.position.y = pos_enu[1]
            odom_enu.pose.pose.position.z = pos_enu[2]

            # 姿态
            odom_enu.pose.pose.orientation.w = q_enu[0]
            odom_enu.pose.pose.orientation.x = q_enu[1]
            odom_enu.pose.pose.orientation.y = q_enu[2]
            odom_enu.pose.pose.orientation.z = q_enu[3]

            # 线速度
            odom_enu.twist.twist.linear.x = vel_enu[0]
            odom_enu.twist.twist.linear.y = vel_enu[1]
            odom_enu.twist.twist.linear.z = vel_enu[2]

            # 角速度
            odom_enu.twist.twist.angular.x = ang_vel_enu[0]
            odom_enu.twist.twist.angular.y = ang_vel_enu[1]
            odom_enu.twist.twist.angular.z = ang_vel_enu[2]

            # 协方差 (复制原始)
            odom_enu.pose.covariance = msg.pose.covariance
            odom_enu.twist.covariance = msg.twist.covariance

            # 发布
            self.odom_pub.publish(odom_enu)

            # 发布 TF
            self.publish_tf(odom_enu)

            # 统计
            self.msg_count += 1
            if self.msg_count % 200 == 0:
                rospy.loginfo("已转换 %d 条里程计消息", self.msg_count)

        except Exception as e:
            rospy.logerr("里程计转换错误: %s", str(e))

    def publish_tf(self, odom):
        """发布 TF 变换"""
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id

        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z

        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def run(self):
        """运行节点"""
        rospy.loginfo("等待 MAVROS 里程计消息...")
        rospy.spin()


def main():
    try:
        node = MavrosOdomConverter()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("节点异常: %s", str(e))


if __name__ == '__main__':
    main()
