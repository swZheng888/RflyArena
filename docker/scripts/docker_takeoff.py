#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
docker_takeoff.py

Docker 环境下的统一起飞脚本
处理 arm、offboard 模式切换和起飞命令

用法:
    python3 docker_takeoff.py [--height 1.0] [--timeout 30]
"""

import rospy
import sys
import argparse
from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import TakeoffLand
import time


class DockerTakeoff:
    """Docker 环境起飞管理器"""

    def __init__(self, target_height=1.0, timeout=30.0):
        rospy.init_node('docker_takeoff', anonymous=True)

        self.target_height = target_height
        self.timeout = timeout

        # 状态
        self.current_state = None
        self.current_height = 0.0
        self.odom_received = False
        self.takeoff_done = False

        # 订阅者
        rospy.Subscriber('/mavros/state', State, self._state_cb)
        rospy.Subscriber('/vio/odometry_enu', Odometry, self._odom_cb)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self._mavros_odom_cb)

        # 发布者
        self.setpoint_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.takeoff_land_pub = rospy.Publisher(
            '/px4ctrl/takeoff_land', TakeoffLand, queue_size=10)

        # 服务客户端
        rospy.loginfo("等待 MAVROS 服务...")
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=10.0)
            rospy.wait_for_service('/mavros/set_mode', timeout=10.0)
            self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            rospy.loginfo("MAVROS 服务已连接")
        except rospy.ROSException:
            rospy.logwarn("MAVROS 服务不可用，将使用备用方法")
            self.arming_client = None
            self.set_mode_client = None

    def _state_cb(self, msg):
        self.current_state = msg

    def _odom_cb(self, msg):
        self.current_height = msg.pose.pose.position.z
        self.odom_received = True

    def _mavros_odom_cb(self, msg):
        # 备用：如果 /vio/odometry_enu 不可用，使用 mavros 直接数据
        # 注意：这里是 NED，需要取反
        if not self.odom_received:
            self.current_height = -msg.pose.pose.position.z

    def _send_setpoint(self, x=0.0, y=0.0, z=1.0):
        """发送位置设定点"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self.setpoint_pub.publish(pose)

    def _arm(self):
        """解锁电机"""
        if self.arming_client is None:
            rospy.logwarn("Arming 服务不可用")
            return False

        try:
            req = CommandBoolRequest()
            req.value = True
            resp = self.arming_client(req)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Arming 失败: {e}")
            return False

    def _set_offboard(self):
        """切换到 Offboard 模式"""
        if self.set_mode_client is None:
            rospy.logwarn("SetMode 服务不可用")
            return False

        try:
            req = SetModeRequest()
            req.custom_mode = "OFFBOARD"
            resp = self.set_mode_client(req)
            return resp.mode_sent
        except rospy.ServiceException as e:
            rospy.logerr(f"设置 Offboard 失败: {e}")
            return False

    def _send_px4ctrl_takeoff(self):
        """发送 px4ctrl 起飞命令"""
        msg = TakeoffLand()
        msg.takeoff_land_cmd = 1  # TAKEOFF
        self.takeoff_land_pub.publish(msg)
        rospy.loginfo("已发送 px4ctrl 起飞命令")

    def run(self):
        """执行起飞流程"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("Docker 起飞管理器")
        rospy.loginfo(f"目标高度: {self.target_height}m")
        rospy.loginfo(f"超时时间: {self.timeout}s")
        rospy.loginfo("=" * 60)

        rate = rospy.Rate(20)
        start_time = rospy.Time.now()

        # 阶段 1: 等待连接
        rospy.loginfo("[1/4] 等待 MAVROS 连接...")
        while not rospy.is_shutdown():
            if self.current_state is not None and self.current_state.connected:
                rospy.loginfo("✓ MAVROS 已连接")
                break
            if (rospy.Time.now() - start_time).to_sec() > self.timeout:
                rospy.logerr("等待连接超时")
                return False
            rate.sleep()

        # 阶段 2: 发送预热设定点 (Offboard 模式需要)
        rospy.loginfo("[2/4] 发送预热设定点...")
        for _ in range(100):  # 发送 5 秒
            self._send_setpoint(z=self.target_height)
            rate.sleep()

        # 阶段 3: 切换模式并解锁
        rospy.loginfo("[3/4] 切换到 Offboard 模式并解锁...")

        offboard_ok = False
        armed_ok = False

        for attempt in range(10):
            # 继续发送设定点
            self._send_setpoint(z=self.target_height)

            if self.current_state:
                # 尝试切换 Offboard
                if self.current_state.mode != "OFFBOARD" and not offboard_ok:
                    if self._set_offboard():
                        rospy.loginfo("✓ Offboard 模式已设置")
                        offboard_ok = True
                elif self.current_state.mode == "OFFBOARD":
                    offboard_ok = True

                # 尝试解锁
                if not self.current_state.armed and not armed_ok:
                    if self._arm():
                        rospy.loginfo("✓ 电机已解锁")
                        armed_ok = True
                elif self.current_state.armed:
                    armed_ok = True

            if offboard_ok and armed_ok:
                break

            time.sleep(0.5)

        # 同时发送 px4ctrl 起飞命令（如果使用 px4ctrl）
        self._send_px4ctrl_takeoff()

        # 阶段 4: 等待起飞完成
        rospy.loginfo("[4/4] 等待起飞完成...")

        stable_count = 0
        required_stable = 40  # 2 秒稳定 (20Hz * 2)

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            if elapsed > self.timeout:
                rospy.logerr(f"起飞超时 ({self.timeout}s)")
                return False

            # 继续发送设定点
            self._send_setpoint(z=self.target_height)

            # 检查高度
            height_ok = self.current_height >= (self.target_height * 0.8)

            if height_ok:
                stable_count += 1
                if stable_count >= required_stable:
                    self.takeoff_done = True
                    break
            else:
                stable_count = 0

            rospy.loginfo_throttle(2.0,
                f"起飞中... 高度: {self.current_height:.2f}m / {self.target_height}m")

            rate.sleep()

        if self.takeoff_done:
            rospy.loginfo("=" * 60)
            rospy.loginfo(f"✓ 起飞完成！最终高度: {self.current_height:.2f}m")
            rospy.loginfo("=" * 60)
            return True

        return False


def main():
    parser = argparse.ArgumentParser(description='Docker 环境起飞脚本')
    parser.add_argument('--height', type=float, default=1.0, help='目标起飞高度 (m)')
    parser.add_argument('--timeout', type=float, default=30.0, help='超时时间 (s)')

    # 解析 ROS 参数
    args, unknown = parser.parse_known_args()

    try:
        takeoff = DockerTakeoff(target_height=args.height, timeout=args.timeout)
        success = takeoff.run()
        sys.exit(0 if success else 1)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"起飞异常: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
