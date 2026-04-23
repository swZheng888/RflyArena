#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
综合评估器 - 顺序执行多个任务并生成综合报告

支持功能：
1. 悬停稳态测试
2. 多速度动态轨迹测试
3. 综合性能评分
4. 对比分析报告
"""

import rospy
import roslaunch
import sys
import os
import time
import numpy as np
from datetime import datetime
import glob
import re
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand

class ComprehensiveBenchmark:
    """综合评估器"""
    
    def __init__(self):
        rospy.init_node('comprehensive_benchmark', anonymous=True)
        
        # 日志目录 - 工作空间内
        workspace_path = rospy.get_param('~workspace_path', '/root/trajectory_project')
        self.base_log_dir = os.path.join(workspace_path, 'benchmark_results')
        
        # 创建时间戳文件夹
        self.session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = os.path.join(self.base_log_dir, f'session_{self.session_timestamp}')
        os.makedirs(self.log_dir, exist_ok=True)
        
        # UUID for roslaunch
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        
        # 任务配置 - 所有轨迹类型 + 多速度测试
        self.trajectory_types = [
            'circle',          # 圆形
            'ellipse',         # 椭圆
            'long_ellipse',    # 长椭圆
            'figure8',         # 8字形
            'spiral_circle',   # 螺旋圆
            'tilted_ellipse',  # 倾斜椭圆
            'cone3d',          # 3D锥形
            'sine_wave',       # 正弦波
            'square',          # 方形
            'zigzag',          # 锯齿形
            'star',            # 五角星
        ]
        
        self.speed_levels = [1.0, 1.5, 2.0]  # 测试3个速度等级
        
        # 生成任务列表
        self.tasks = []
        
        # 1. 悬停测试
        self.tasks.append({
            'name': 'hover',
            'type': 'hover',
            'duration': 30.0,
            'wait_time': 50.0,
            'description': '悬停稳态测试'
        })
        
        # 2. 所有轨迹类型 x 所有速度
        for traj_type in self.trajectory_types:
            for speed in self.speed_levels:
                task_name = f'{traj_type}_{speed}x'
                # 困难任务降速：只跑1.0x
                if traj_type in ['zigzag', 'cone3d', 'star'] and speed > 1.0:
                    continue
                    
                self.tasks.append({
                    'name': task_name,
                    'type': 'dynamic',
                    'trajectory': traj_type,
                    'speed_level': speed,
                    'duration': 60.0,  # 增加时长以确保慢速任务能跑完2圈 (超时保护)
                    'wait_time': 30.0,
                    'description': f'{traj_type}轨迹-{speed}x速度'
                })
        
        # 结果存储
        self.results = []
        
        # 位置监听
        self.current_position = None
        self.start_point = np.array([0.0, 0.0, 1.0])  # 起点(0,0,1)
        self.odom_sub = rospy.Subscriber('/vio/odometry', Odometry, self._odom_callback)
        
        # 位置指令发布器 - 用于返回起点
        self.position_cmd_pub = rospy.Publisher('/position_cmd', PositionCommand, queue_size=10)
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("综合评估器已启动")
        rospy.loginfo("任务数量: %d", len(self.tasks))
        rospy.loginfo("日志目录: %s", self.log_dir)
        rospy.loginfo("起点位置: [%.2f, %.2f, %.2f]", self.start_point[0], self.start_point[1], self.start_point[2])
        rospy.loginfo("=" * 70)
        
    def _odom_callback(self, msg):
        """里程计回调 - NED转ENU"""
        # NED -> ENU转换
        pos_ned = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        # NED to ENU: [y_n, x_n, -z_n]
        self.current_position = np.array([pos_ned[1], pos_ned[0], -pos_ned[2]])
        
    def run_all_tasks(self):
        """顺序执行所有任务"""
        rospy.loginfo("\n" + "=" * 70)
        rospy.loginfo("开始综合评估 - %d个任务", len(self.tasks))
        rospy.loginfo("=" * 70 + "\n")
        
        start_time = time.time()
        
        for i, task in enumerate(self.tasks, 1):
            rospy.loginfo("\n>>> 任务 %d/%d: %s <<<", i, len(self.tasks), task['description'])
            rospy.loginfo("-" * 70)
            
            # 创建任务专属文件夹
            task_folder = os.path.join(self.log_dir, f"{i:02d}_{task['name']}")
            os.makedirs(task_folder, exist_ok=True)
            rospy.loginfo("任务日志目录: %s", task_folder)
            
            # 启动benchmark任务
            launch_process = self._launch_benchmark_task(task, task_folder)
            
            if launch_process is None:
                rospy.logerr("任务启动失败，跳过")
                continue
            
            # 等待任务完成（检测报告生成）
            max_wait_time = task['duration'] + 45.0  # 增加更长的缓冲时间
            rospy.loginfo("等待任务完成 (超时: %.1f秒)...", max_wait_time)
            
            start_wait = time.time()
            report_found = False
            
            while time.time() - start_wait < max_wait_time:
                # 1. 检查报告文件
                report_files = glob.glob(os.path.join(task_folder, 'report_*.txt'))
                if report_files:
                    # 找到报告，再短暂等待以确保写入完成
                    rospy.sleep(2.0)
                    rospy.loginfo("检测到任务报告，任务完成")
                    report_found = True
                    break
                
                # 2. 检查进程状态
                if launch_process.poll() is not None:
                    rospy.logwarn("Benchmark进程意外退出 (Exit Code: %s)", launch_process.returncode)
                    break
                    
                rospy.sleep(1.0)
            
            if not report_found:
                rospy.logwarn("等待任务超时")
            
            # 停止benchmark
            rospy.loginfo("停止任务...")
            try:
                launch_process.terminate()
                launch_process.wait(timeout=5.0)
            except:
                launch_process.kill()
            rospy.sleep(2.0)
            
            # 收集结果
            result = self._collect_task_result(task, task_folder)
            if result:
                self.results.append(result)
                rospy.loginfo("任务完成 - 评分: %.1f", result['score'])
            else:
                rospy.logwarn("未能收集任务结果")
            
            # 等待返回起点
            if i < len(self.tasks):
                rospy.loginfo("等待UAV返回起点...")
                self._wait_for_start_point(timeout=30.0)
                rospy.loginfo("已到达起点，继续下一任务\n")
        
        total_time = time.time() - start_time
        
        # 生成综合报告
        self._generate_comprehensive_report(total_time)
        
        rospy.loginfo("\n" + "=" * 70)
        rospy.loginfo("综合评估完成！总耗时: %.1f分钟", total_time / 60.0)
        rospy.loginfo("=" * 70)
        
    def _launch_benchmark_task(self, task, task_log_dir):
        """启动benchmark任务"""
        try:
            # 构造命令行参数
            # 使用列表形式直接传给subprocess
            cmd = ['roslaunch', 'benchmark_utils', 'benchmark.launch']
            cmd.append(f'task_type:={task["type"]}')
            cmd.append(f'task_duration:={task["duration"]}')
            cmd.append(f'log_dir:={task_log_dir}')
            cmd.append('coordinate_frame:=0')
            cmd.append('odom_topic:=/vio/odometry')
            cmd.append('position_cmd_topic:=/position_cmd')
            
            if task['type'] == 'dynamic':
                cmd.append(f'trajectory_type:={task["trajectory"]}')
                cmd.append(f'speed_level:={task["speed_level"]}')
            
            rospy.loginfo("启动命令: %s", ' '.join(cmd))
            
            # 使用subprocess启动
            import subprocess
            process = subprocess.Popen(cmd)
            
            rospy.loginfo("Benchmark任务已启动 (PID: %d)", process.pid)
            return process
            
        except Exception as e:
            rospy.logerr("启动任务失败: %s", str(e))
            return None
    
    def _collect_task_result(self, task, task_folder):
        """收集任务结果"""
        # 查找任务文件夹中的报告文件
        report_files = glob.glob(os.path.join(task_folder, 'report_*.txt'))
        if not report_files:
            rospy.logwarn("未找到报告文件: %s", task_folder)
            return None
        
        # 按修改时间排序，获取最新的
        latest_report = max(report_files, key=os.path.getmtime)
        
        # 检查文件是否在最近生成（60秒内）
        file_age = time.time() - os.path.getmtime(latest_report)
        if file_age > 60:
            rospy.logwarn("最新报告文件太旧 (%.1f秒前)", file_age)
            return None
        
        # 解析报告
        try:
            with open(latest_report, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 提取关键指标
            score_match = re.search(r'综合得分:\s*([\d.]+)', content)
            rms_match = re.search(r'RMS误差:\s*([\d.]+)', content)
            max_match = re.search(r'最大误差:\s*([\d.]+)', content)
            grade_match = re.search(r'性能等级:\s*(\w+)', content)
            
            result = {
                'task_name': task['name'],
                'description': task['description'],
                'score': float(score_match.group(1)) if score_match else 0.0,
                'rms_error': float(rms_match.group(1)) if rms_match else 0.0,
                'max_error': float(max_match.group(1)) if max_match else 0.0,
                'grade': grade_match.group(1) if grade_match else 'N/A',
                'report_file': latest_report
            }
            
            rospy.loginfo("收集到结果: 得分=%.1f, RMS=%.4fm, 等级=%s", 
                         result['score'], result['rms_error'], result['grade'])
            
            return result
            
        except Exception as e:
            rospy.logerr("解析报告失败: %s", str(e))
            return None
    
    def _wait_for_start_point(self, timeout=30.0):
        """等待UAV返回起点 - 主动发布返回指令"""
        start_time = time.time()
        check_threshold = 0.2  # 0.2m
        
        rospy.loginfo("开始发布返回起点指令...")
        
        while not rospy.is_shutdown():
            # 发布返回起点的位置指令
            self._publish_return_cmd()
            
            if self.current_position is None:
                rospy.sleep(0.1)
                continue
            
            # 计算距离
            dist = np.linalg.norm(self.current_position - self.start_point)
            
            if dist < check_threshold:
                rospy.loginfo("已到达起点，距离=%.3fm", dist)
                return True
            
            # 超时检查
            elapsed = time.time() - start_time
            if elapsed > timeout:
                rospy.logwarn("等待起点超时，当前距离=%.3fm，强制继续", dist)
                return False
            
            # 定期打印进度
            if int(elapsed) % 3 == 0:
                rospy.loginfo("返回起点中... 当前距离=%.3fm", dist)
            
            rospy.sleep(0.1)  # 10Hz发布频率
        
        return False
    
    def _publish_return_cmd(self):
        """发布返回起点的位置指令"""
        cmd = PositionCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = 'world'
        
        # ENU坐标
        cmd.position.x = self.start_point[0]
        cmd.position.y = self.start_point[1]
        cmd.position.z = self.start_point[2]
        
        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0
        
        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0
        
        cmd.yaw = 0.0
        cmd.yaw_dot = 0.0
        
        self.position_cmd_pub.publish(cmd)
    
    def _generate_comprehensive_report(self, total_time):
        """生成综合评估报告"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_path = os.path.join(self.log_dir, f'comprehensive_report_{timestamp}.txt')
        
        # 计算综合评分
        if self.results:
            avg_score = np.mean([r['score'] for r in self.results])
            avg_rms = np.mean([r['rms_error'] for r in self.results])
            max_rms = max([r['rms_error'] for r in self.results])
            min_rms = min([r['rms_error'] for r in self.results])
        else:
            avg_score = avg_rms = max_rms = min_rms = 0.0
        
        # 生成报告
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("=" * 80 + "\n")
            f.write("NMPC控制器综合性能评估报告\n")
            f.write("=" * 80 + "\n\n")
            
            f.write(f"评估时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"总耗时: {total_time/60.0:.1f} 分钟\n")
            f.write(f"完成任务数: {len(self.results)}/{len(self.tasks)}\n\n")
            
            f.write("【综合评分】\n")
            f.write("-" * 80 + "\n")
            f.write(f"平均得分: {avg_score:.1f} / 100\n")
            f.write(f"平均RMS误差: {avg_rms:.4f} m\n")
            f.write(f"最小RMS误差: {min_rms:.4f} m\n")
            f.write(f"最大RMS误差: {max_rms:.4f} m\n\n")
            
            # 性能评级
            if avg_score >= 97:
                overall_grade = "A+ (优秀)"
            elif avg_score >= 90:
                overall_grade = "A (良好)"
            elif avg_score >= 80:
                overall_grade = "B (及格)"
            else:
                overall_grade = "C (需改进)"
            
            f.write(f"综合评级: {overall_grade}\n\n")
            
            f.write("【各任务详情】\n")
            f.write("-" * 80 + "\n")
            for i, result in enumerate(self.results, 1):
                f.write(f"\n{i}. {result['description']}\n")
                f.write(f"   任务名称: {result['task_name']}\n")
                f.write(f"   评分: {result['score']:.1f}\n")
                f.write(f"   等级: {result['grade']}\n")
                f.write(f"   RMS误差: {result['rms_error']:.4f} m\n")
                f.write(f"   最大误差: {result['max_error']:.4f} m\n")
                f.write(f"   详细报告: {os.path.basename(result['report_file'])}\n")
            
            f.write("\n" + "=" * 80 + "\n")
            f.write("评估完成\n")
            f.write("=" * 80 + "\n")
        
        rospy.loginfo("\n综合报告已生成: %s", report_path)
        rospy.loginfo("平均得分: %.1f, 平均RMS: %.4f m, 综合评级: %s", 
                     avg_score, avg_rms, overall_grade)


if __name__ == '__main__':
    try:
        benchmark = ComprehensiveBenchmark()
        rospy.sleep(2.0)  # 等待ROS初始化
        benchmark.run_all_tasks()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("综合评估失败: %s", str(e))
        import traceback
        traceback.print_exc()
