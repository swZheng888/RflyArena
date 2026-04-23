#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bag_trajectory_player.py

从 rosbag 中读取预录的 PositionCommand 轨迹并按原始时间间隔回放发布。
支持循环播放、速度缩放、起始偏移。

用法:
  rosrun benchmark_utils bag_trajectory_player.py \
      _bag_path:=/root/nmpc.bag \
      _cmd_topic_in:=/position_cmd \
      _cmd_topic_out:=/position_cmd \
      _speed_scale:=1.0 \
      _loop:=true \
      _offset_x:=0 _offset_y:=0 _offset_z:=0
"""

import rospy
import rosbag
import numpy as np
from quadrotor_msgs.msg import PositionCommand


class BagTrajectoryPlayer:
    def __init__(self):
        rospy.init_node('bag_trajectory_player', anonymous=True)
        self._load_params()
        self._load_bag()
        self._init_pub()

    def _load_params(self):
        self.bag_path = rospy.get_param('~bag_path', '')
        self.cmd_topic_in = rospy.get_param('~cmd_topic_in', '/position_cmd')
        self.cmd_topic_out = rospy.get_param('~cmd_topic_out', '/position_cmd')
        self.speed_scale = rospy.get_param('~speed_scale', 1.0)
        self.loop = rospy.get_param('~loop', False)
        self.start_delay = rospy.get_param('~start_delay', 3.0)

        # 位置偏移 (NED/ENU 取决于 bag 录制时的坐标系)
        self.offset_x = rospy.get_param('~offset_x', 0.0)
        self.offset_y = rospy.get_param('~offset_y', 0.0)
        self.offset_z = rospy.get_param('~offset_z', 0.0)

        if not self.bag_path:
            rospy.logfatal("[BagPlayer] 必须指定 bag_path 参数!")
            rospy.signal_shutdown("No bag_path")
            return

    def _load_bag(self):
        """从 bag 中提取所有 PositionCommand 消息和时间戳"""
        rospy.loginfo(f"[BagPlayer] 加载 bag: {self.bag_path}")
        rospy.loginfo(f"[BagPlayer] 读取话题: {self.cmd_topic_in}")

        self.messages = []
        self.timestamps = []

        try:
            bag = rosbag.Bag(self.bag_path, 'r')
            info = bag.get_type_and_topic_info()

            # 列出 bag 中的所有话题
            rospy.loginfo(f"[BagPlayer] Bag 话题:")
            for topic, info_t in info.topics.items():
                rospy.loginfo(f"  {topic}: {info_t.message_count} msgs ({info_t.msg_type})")

            for topic, msg, t in bag.read_messages(topics=[self.cmd_topic_in]):
                self.messages.append(msg)
                self.timestamps.append(t.to_sec())

            bag.close()
        except Exception as e:
            rospy.logfatal(f"[BagPlayer] 无法加载 bag: {e}")
            rospy.signal_shutdown(str(e))
            return

        if len(self.messages) == 0:
            rospy.logfatal(f"[BagPlayer] 话题 '{self.cmd_topic_in}' 中没有消息!")
            rospy.signal_shutdown("No messages")
            return

        # 计算相对时间
        t0 = self.timestamps[0]
        self.rel_times = [t - t0 for t in self.timestamps]
        self.total_duration = self.rel_times[-1]

        rospy.loginfo(f"[BagPlayer] ✅ 加载完成:")
        rospy.loginfo(f"  消息数: {len(self.messages)}")
        rospy.loginfo(f"  时长:   {self.total_duration:.2f}s")
        rospy.loginfo(f"  频率:   {len(self.messages)/self.total_duration:.1f} Hz")
        rospy.loginfo(f"  速度:   {self.speed_scale}x")
        rospy.loginfo(f"  循环:   {self.loop}")
        rospy.loginfo(f"  偏移:   [{self.offset_x}, {self.offset_y}, {self.offset_z}]")

    def _init_pub(self):
        self.pub = rospy.Publisher(self.cmd_topic_out, PositionCommand, queue_size=10)

    def _apply_offset(self, msg):
        """给消息加上位置偏移"""
        msg.position.x += self.offset_x
        msg.position.y += self.offset_y
        msg.position.z += self.offset_z
        return msg

    def run(self):
        if not self.messages:
            return

        # 等待启动
        rospy.loginfo(f"[BagPlayer] {self.start_delay}s 后开始回放...")
        rospy.sleep(self.start_delay)

        loop_count = 0
        while not rospy.is_shutdown():
            loop_count += 1
            rospy.loginfo(f"[BagPlayer] 开始回放 (第 {loop_count} 次, "
                          f"时长 {self.total_duration/self.speed_scale:.1f}s)")

            play_start = rospy.Time.now()

            for i in range(len(self.messages)):
                if rospy.is_shutdown():
                    break

                # 计算应该发布的时间点
                target_time = self.rel_times[i] / self.speed_scale
                elapsed = (rospy.Time.now() - play_start).to_sec()
                wait_time = target_time - elapsed

                if wait_time > 0:
                    rospy.sleep(wait_time)

                # 构造并发布消息
                msg = PositionCommand()
                src = self.messages[i]

                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = src.header.frame_id

                msg.position = src.position
                msg.velocity = src.velocity
                msg.acceleration = src.acceleration
                msg.jerk = src.jerk
                msg.yaw = src.yaw
                msg.yaw_dot = src.yaw_dot
                msg.yaw_dir = src.yaw_dir
                msg.yaw_dir_dot = src.yaw_dir_dot
                msg.kx = src.kx
                msg.kv = src.kv
                msg.trajectory_id = src.trajectory_id
                msg.trajectory_flag = src.trajectory_flag

                msg = self._apply_offset(msg)
                self.pub.publish(msg)

            actual_duration = (rospy.Time.now() - play_start).to_sec()
            rospy.loginfo(f"[BagPlayer] 第 {loop_count} 次回放完成 "
                          f"(实际 {actual_duration:.1f}s)")

            if not self.loop:
                rospy.loginfo("[BagPlayer] 单次回放结束")
                break

            rospy.sleep(1.0)  # 循环间隔

        rospy.loginfo("[BagPlayer] 退出")


if __name__ == '__main__':
    try:
        player = BagTrajectoryPlayer()
        player.run()
    except rospy.ROSInterruptException:
        pass
