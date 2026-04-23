#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Trajectory Replayer - 回放录制的轨迹 Bag 包

功能:
- 从 ROS bag 文件中读取位置/速度数据
- 将轨迹作为参考发布给控制器
- 支持时间偏移和起点平移

使用方法:
    rosrun benchmark_utils trajectory_replayer.py _bag_file:=/path/to/bag _topic:=/vio/odometry
"""

import rospy
import rosbag
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


class TrajectoryReplayer:
    """从 Bag 文件回放轨迹"""
    
    def __init__(self):
        rospy.init_node('trajectory_replayer', anonymous=True)
        
        # 参数
        self.bag_file = rospy.get_param('~bag_file', '')
        self.topic = rospy.get_param('~topic', '/vio/odometry')
        self.relocate = rospy.get_param('~relocate', True)  # 平移到当前位置
        self.speed_factor = rospy.get_param('~speed_factor', 1.0)
        self.loop = rospy.get_param('~loop', False)
        
        # 发布器
        self.cmd_pub = rospy.Publisher('/benchmark/trajectory_setpoint', 
                                       PositionCommand, queue_size=10)
        
        # 当前位置订阅
        self.current_pos = np.zeros(3)
        rospy.Subscriber('/vio/odometry_enu', Odometry, self._odom_cb)
        
        # 加载轨迹数据
        self.trajectory = []
        self.duration = 0.0
        
        if self.bag_file:
            self._load_bag()
        else:
            rospy.logerr("未指定 bag 文件! 使用 _bag_file:=/path/to/bag")
            return
            
        # 等待初始位置
        rospy.sleep(1.0)
        rospy.loginfo(f"轨迹已加载: {len(self.trajectory)} 点, 总时长: {self.duration:.2f}s")
        
    def _odom_cb(self, msg):
        """里程计回调"""
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
    def _load_bag(self):
        """从 bag 文件加载轨迹"""
        try:
            bag = rosbag.Bag(self.bag_file, 'r')
            
            start_time = None
            first_pos = None
            
            for topic, msg, t in bag.read_messages(topics=[self.topic]):
                # 提取位置
                if hasattr(msg, 'pose'):
                    # Odometry 或 PoseStamped
                    if hasattr(msg.pose, 'pose'):
                        pos = msg.pose.pose.position
                        vel = msg.twist.twist.linear if hasattr(msg, 'twist') else None
                    else:
                        pos = msg.pose.position
                        vel = None
                else:
                    continue
                    
                time_sec = t.to_sec()
                if start_time is None:
                    start_time = time_sec
                    first_pos = np.array([pos.x, pos.y, pos.z])
                    
                rel_time = time_sec - start_time
                
                point = {
                    'time': rel_time,
                    'pos': np.array([pos.x, pos.y, pos.z]) - first_pos,  # 相对起点
                    'vel': np.array([vel.x, vel.y, vel.z]) if vel else np.zeros(3)
                }
                self.trajectory.append(point)
                
            bag.close()
            
            if self.trajectory:
                self.duration = self.trajectory[-1]['time']
                
        except Exception as e:
            rospy.logerr(f"加载 bag 文件失败: {e}")
            
    def get_state(self, t):
        """获取时间 t 的轨迹状态 (插值)"""
        if not self.trajectory:
            return None, None, None
            
        # 处理循环
        if self.loop and t > self.duration:
            t = t % self.duration
        elif t > self.duration:
            t = self.duration
            
        # 线性搜索 (可优化为二分)
        for i in range(len(self.trajectory) - 1):
            if self.trajectory[i]['time'] <= t < self.trajectory[i+1]['time']:
                # 线性插值
                t0, t1 = self.trajectory[i]['time'], self.trajectory[i+1]['time']
                alpha = (t - t0) / (t1 - t0) if t1 > t0 else 0
                
                pos = self.trajectory[i]['pos'] * (1-alpha) + self.trajectory[i+1]['pos'] * alpha
                vel = self.trajectory[i]['vel'] * (1-alpha) + self.trajectory[i+1]['vel'] * alpha
                acc = np.zeros(3)
                
                return pos, vel, acc
                
        # 返回最后一个点
        return self.trajectory[-1]['pos'], self.trajectory[-1]['vel'], np.zeros(3)
        
    def run(self):
        """运行回放"""
        if not self.trajectory:
            return
            
        rate = rospy.Rate(50)  # 50 Hz
        
        # 起点偏移
        offset = self.current_pos if self.relocate else np.zeros(3)
        
        start_time = rospy.Time.now()
        rospy.loginfo("开始回放轨迹...")
        
        while not rospy.is_shutdown():
            t = (rospy.Time.now() - start_time).to_sec() * self.speed_factor
            
            if t > self.duration and not self.loop:
                rospy.loginfo("轨迹回放完成!")
                break
                
            pos, vel, acc = self.get_state(t)
            if pos is None:
                continue
                
            # 发布指令
            cmd = PositionCommand()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "world"
            
            pos_shifted = pos + offset
            cmd.position.x = pos_shifted[0]
            cmd.position.y = pos_shifted[1]
            cmd.position.z = pos_shifted[2]
            
            cmd.velocity.x = vel[0] * self.speed_factor
            cmd.velocity.y = vel[1] * self.speed_factor
            cmd.velocity.z = vel[2] * self.speed_factor
            
            cmd.acceleration.x = acc[0]
            cmd.acceleration.y = acc[1]
            cmd.acceleration.z = acc[2]
            
            # Yaw 计算
            if np.linalg.norm(vel[:2]) > 0.1:
                yaw = np.arctan2(vel[1], vel[0])
            else:
                yaw = 0.0
            cmd.yaw_dir.x = np.cos(yaw)
            cmd.yaw_dir.y = np.sin(yaw)
            
            self.cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == '__main__':
    try:
        replayer = TrajectoryReplayer()
        replayer.run()
    except rospy.ROSInterruptException:
        pass
