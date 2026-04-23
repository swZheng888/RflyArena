#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
benchmark_logger.py - 重构版本

Benchmark数据记录器

核心改进:
- 独立的ROS节点，通过话题控制开始/停止
- 仅在收到"start"指令后记录数据
- 精简的CSV格式
- 实时误差计算并发布

工作流程:
1. 订阅/benchmark/logging_control接收开始/停止指令
2. 订阅/position_cmd和/odom获取目标和实际数据
3. recording=True时写入CSV
4. 收到"stop"时关闭文件
"""

import rospy
import numpy as np
import csv
import os
from datetime import datetime
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


class BenchmarkLogger:
    """
    Benchmark数据记录器
    
    独立节点，监听控制信号决定何时记录数据
    """
    
    def __init__(self):
        rospy.init_node('benchmark_logger', anonymous=True)
        
        # 参数
        self.log_dir = rospy.get_param('~log_dir', '~/.benchmark_logs')
        self.log_dir = os.path.expanduser(self.log_dir)
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # 坐标系参数（从benchmark_node获取）
        self.coordinate_frame = rospy.get_param('/benchmark_node/coordinate_frame', 0)  # 0=NED, 1=ENU
        self.odom_topic = rospy.get_param('/benchmark_node/odom_topic', '/vio/odometry')
            
        # 记录状态
        self.recording = False
        self.log_file = None
        self.log_writer = None
        self.log_file_path = None
        self.record_start_time = None
        
        # 数据缓存
        self.target_position = np.zeros(3)
        self.target_velocity = np.zeros(3)
        self.actual_position = np.zeros(3)
        self.actual_velocity = np.zeros(3)
        self.has_target = False
        self.has_odom = False
        
        # 统计信息
        self.sample_count = 0
        self.max_pos_error = 0.0
        self.sum_pos_error = 0.0
        self.sum_pos_error_sq = 0.0
        
        # ROS接口
        self._setup_ros_interface()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Benchmark Logger 已启动 (重构版)")
        rospy.loginfo("日志目录: %s", self.log_dir)
        rospy.loginfo("Odom话题: %s", self.odom_topic)
        rospy.loginfo("坐标系: %s", "NED (将转换为ENU)" if self.coordinate_frame == 0 else "ENU")
        rospy.loginfo("等待记录指令...")
        rospy.loginfo("=" * 60)
        
    def _setup_ros_interface(self):
        """设置ROS接口"""
        # 订阅者
        rospy.Subscriber('/benchmark/logging_control', String, 
                        self._control_callback, queue_size=10)
        rospy.Subscriber('/position_cmd', PositionCommand, 
                        self._target_callback, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, 
                        self._odom_callback, queue_size=10)
                        
    def _control_callback(self, msg):
        """控制指令回调"""
        command = msg.data.lower()
        
        if command == "start" and not self.recording:
            self._start_recording()
        elif command == "stop" and self.recording:
            self._stop_recording()
            
    def _target_callback(self, msg):
        """目标轨迹回调"""
        self.target_position = np.array([
            msg.position.x,
            msg.position.y,
            msg.position.z
        ])
        self.target_velocity = np.array([
            msg.velocity.x,
            msg.velocity.y,
            msg.velocity.z
        ])
        self.has_target = True
        
        # 如果正在记录且有odom数据，记录一行
        if self.recording and self.has_odom:
            self._record_data()
            
    def _odom_callback(self, msg):
        """里程计回调（支持NED→ENU转换）"""
        # 读取原始数据
        pos_raw = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        vel_raw = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        # 坐标系转换
        if self.coordinate_frame == 0:  # NED输入，转换为ENU
            # NED -> ENU: [x_n, y_n, z_n] -> [y_e, x_e, -z_e]
            self.actual_position = np.array([
                pos_raw[1],   # ENU_x = NED_y (东)
                pos_raw[0],   # ENU_y = NED_x (北)
                -pos_raw[2]   # ENU_z = -NED_z (上)
            ])
            self.actual_velocity = np.array([
                vel_raw[1],
                vel_raw[0],
                -vel_raw[2]
            ])
        else:  # ENU输入，直接使用
            self.actual_position = pos_raw
            self.actual_velocity = vel_raw
            
        self.has_odom = True
        
    def _start_recording(self):
        """开始记录"""
        # 生成文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        task_name = rospy.get_param('/benchmark_node/task_type', 'unknown')
        filename = f"{task_name}_{timestamp}.csv"
        self.log_file_path = os.path.join(self.log_dir, filename)
        
        # 打开文件
        self.log_file = open(self.log_file_path, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        
        # 写入header
        self.log_writer.writerow([
            'time',
            'target_x', 'target_y', 'target_z',
            'actual_x', 'actual_y', 'actual_z',
            'target_vx', 'target_vy', 'target_vz',
            'actual_vx', 'actual_vy', 'actual_vz'
        ])
        
        # 重置状态
        self.recording = True
        self.record_start_time = rospy.Time.now()
        self.sample_count = 0
        self.max_pos_error = 0.0
        self.sum_pos_error = 0.0
        self.sum_pos_error_sq = 0.0
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("开始记录数据")
        rospy.loginfo("文件: %s", self.log_file_path)
        rospy.loginfo("=" * 60)
        
    def _stop_recording(self):
        """停止记录"""
        self.recording = False
        
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            
        # 计算并显示统计信息
        if self.sample_count > 0:
            mean_error = self.sum_pos_error / self.sample_count
            rms_error = np.sqrt(self.sum_pos_error_sq / self.sample_count)
            
            rospy.loginfo("=" * 60)
            rospy.loginfo("记录完成")
            rospy.loginfo("文件: %s", self.log_file_path)
            rospy.loginfo("采样数: %d", self.sample_count)
            rospy.loginfo("最大位置误差: %.4f m", self.max_pos_error)
            rospy.loginfo("平均位置误差: %.4f m", mean_error)
            rospy.loginfo("RMS位置误差: %.4f m", rms_error)
            rospy.loginfo("=" * 60)
        else:
            rospy.logwarn("没有记录任何数据!")
            
    def _record_data(self):
        """记录一行数据"""
        if not self.recording or not self.has_target or not self.has_odom:
            return
            
        # 计算时间（从开始记录算起）
        elapsed = (rospy.Time.now() - self.record_start_time).to_sec()
        
        # 写入CSV
        self.log_writer.writerow([
            elapsed,
            self.target_position[0], self.target_position[1], self.target_position[2],
            self.actual_position[0], self.actual_position[1], self.actual_position[2],
            self.target_velocity[0], self.target_velocity[1], self.target_velocity[2],
            self.actual_velocity[0], self.actual_velocity[1], self.actual_velocity[2]
        ])
        
        # 更新统计
        pos_error = np.linalg.norm(self.target_position - self.actual_position)
        self.sample_count += 1
        self.max_pos_error = max(self.max_pos_error, pos_error)
        self.sum_pos_error += pos_error
        self.sum_pos_error_sq += pos_error ** 2
        
        # 每100个样本刷新一次文件
        if self.sample_count % 100 == 0:
            self.log_file.flush()
            
    def run(self):
        """运行节点"""
        rospy.spin()


if __name__ == '__main__':
    try:
        logger = BenchmarkLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass
