#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
task_manager.py

Benchmark 任务管理器

支持运行单个任务或多个任务的序列：
- hover: 悬停稳态测试
- disturbance: 抗扰能力测试
- dynamic: 动态轨迹测试
- all: 运行所有任务（hover -> disturbance -> dynamic）

任务完成后自动生成分析报告
"""

import rospy
import subprocess
import signal
import sys
import time
import os
import glob
import re
from datetime import datetime
from std_msgs.msg import String

try:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from benchmark_analyzer import BenchmarkAnalyzer
    ANALYZER_AVAILABLE = True
except ImportError as e:
    ANALYZER_AVAILABLE = False
    rospy.logwarn("benchmark_analyzer 未找到，详细分析功能将不可用: %s", str(e))


class TaskManager:
    """任务管理器"""
    
    def __init__(self):
        rospy.init_node('task_manager', anonymous=True)
        
        self._init_parameters()
        self._init_subscribers()
        self._init_publishers()
        
        self.current_benchmark_process = None
        self.current_task_index = 0
        self.task_results = []
        self.task_completion_received = False
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("任务管理器已启动")
        rospy.loginfo("任务序列: %s", self.task_sequence)
        rospy.loginfo("日志目录: %s", self.log_dir)
        rospy.loginfo("=" * 60)
        
        self._run_task_sequence()
        
        rospy.Timer(rospy.Duration(5.0), self._monitor_process)
    
    def _init_parameters(self):
        """初始化参数"""
        self.task_type = rospy.get_param('~task_type', 'all')
        
        self.hover_duration = rospy.get_param('~hover_duration', 30.0)
        self.disturbance_duration = rospy.get_param('~disturbance_duration', 30.0)
        self.dynamic_duration = rospy.get_param('~dynamic_duration', 60.0)
        
        self.log_dir = rospy.get_param('~log_dir', '~/.benchmark_logs')
        self.log_dir = os.path.expanduser(self.log_dir)
        
        self.initial_speed = rospy.get_param('~initial_speed', 1.0)
        
        self.trajectory_types = rospy.get_param('~trajectory_types', 
            ['figure8', 'spiral', 'sine_wave', 'square', 'fast_orbit'])
        
        self.speed_levels = self._parse_list_param(rospy.get_param('~speed_levels', ['1.0x', '1.5x', '2.0x', '2.5x']))
        
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        self.task_sequence = self._get_task_sequence()
        
        self.task_configs = {
            'hover': {
                'task_type': 'hover',
                'task_duration': self.hover_duration,
                'hold_height': 1.0,
                'switch_threshold': 0.3,
                'hover_points': '[[0, 0, 1.0], [1, 0, 1.0], [0, 1, 1.0], [-1, 0, 1.0], [0, -1, 1.0], [0, 0, 1.0]]',
                'hover_hold_time': 5.0,
                'start_delay': 0.0,
                'takeoff_height_threshold': 0.8,
                'takeoff_stable_time': 2.0,
                'control_delay': 0.2,
                'odom_topic': '/vio/odometry',
                'position_cmd_topic': '/position_cmd',
                'ref_path_topic': '/benchmark_node/ref_path',
                'task_status_topic': '/benchmark_node/task_status',
                'task_phase_topic': '/benchmark_node/task_phase',
                'progress_topic': '/benchmark_node/progress',
                'coordinate_frame': 0
            },
            'disturbance': {
                'task_type': 'disturbance',
                'task_duration': self.disturbance_duration,
                'hold_height': 1.0,
                'disturbance_type': 'step',
                'disturbance_magnitude': 0.5,
                'disturbance_start_time': 5.0,
                'disturbance_duration': 2.0,
                'start_delay': 0.0,
                'takeoff_height_threshold': 0.8,
                'takeoff_stable_time': 2.0,
                'control_delay': 0.2,
                'odom_topic': '/vio/odometry',
                'position_cmd_topic': '/position_cmd',
                'ref_path_topic': '/benchmark_node/ref_path',
                'task_status_topic': '/benchmark_node/task_status',
                'task_phase_topic': '/benchmark_node/task_phase',
                'progress_topic': '/benchmark_node/progress',
                'coordinate_frame': 0
            }
        }
        
        for traj_type in self.trajectory_types:
            for speed_level in self.speed_levels:
                speed_value = float(speed_level.replace('x', ''))
                self.task_configs[f'dynamic_{traj_type}_{speed_level}'] = {
                    'task_type': 'dynamic',
                    'task_duration': self.dynamic_duration,
                    'trajectory_type': traj_type,
                    'trajectory_speed': self.initial_speed,
                    'trajectory_amplitude': 1.5,
                    'speed_level': speed_value,
                    'num_loops': 3,
                    'start_delay': 0.0,
                    'takeoff_height_threshold': 0.8,
                    'takeoff_stable_time': 2.0,
                    'control_delay': 0.2,
                    'odom_topic': '/vio/odometry',
                    'position_cmd_topic': '/position_cmd',
                    'ref_path_topic': '/benchmark_node/ref_path',
                    'task_status_topic': '/benchmark_node/task_status',
                    'task_phase_topic': '/benchmark_node/task_phase',
                    'progress_topic': '/benchmark_node/progress',
                    'coordinate_frame': 0
                }
    
    def _get_task_sequence(self):
        """获取任务序列"""
        if self.task_type == 'all':
            sequence = ['hover', 'disturbance']
            for traj_type in self.trajectory_types:
                for speed_level in self.speed_levels:
                    sequence.append(f'dynamic_{traj_type}_{speed_level}')
            return sequence
        elif self.task_type == 'dynamic_all':
            sequence = []
            for traj_type in self.trajectory_types:
                for speed_level in self.speed_levels:
                    sequence.append(f'dynamic_{traj_type}_{speed_level}')
            return sequence
        else:
            return [self.task_type]
    
    def _init_subscribers(self):
        """初始化订阅者"""
        rospy.Subscriber('/benchmark_node/task_status', String, self._task_status_callback, queue_size=10)
    
    def _init_publishers(self):
        """初始化发布者"""
        self.status_pub = rospy.Publisher('/task_manager/status', String, queue_size=10)
    
    def _parse_list_param(self, param):
        """解析列表参数，支持字符串和列表格式"""
        if isinstance(param, list):
            return param
        if isinstance(param, str):
            try:
                return eval(param)
            except:
                return [param]
        return [param]
    
    def _task_status_callback(self, msg):
        """任务状态回调"""
        status = msg.data
        
        if status == "任务完成" and self.current_benchmark_process is not None:
            if not self.task_completion_received:
                self.task_completion_received = True
                rospy.loginfo("=" * 60)
                rospy.loginfo(">>> [TaskManager] 当前任务完成，准备切换到下一个任务")
                rospy.loginfo("=" * 60)
                self._stop_current_task()
                self._start_next_task()
            else:
                rospy.logwarn(">>> [TaskManager] 已收到任务完成信号，忽略重复消息")
    
    def _run_task_sequence(self):
        """运行任务序列"""
        if not self.task_sequence:
            rospy.logwarn("没有任务需要执行")
            return
        
        rospy.sleep(2.0)
        self._start_next_task()
    
    def _start_next_task(self):
        """开始下一个任务"""
        if self.current_task_index >= len(self.task_sequence):
            rospy.loginfo("=" * 60)
            rospy.loginfo(">>> [TaskManager] 所有任务已完成！")
            rospy.loginfo("=" * 60)
            self._generate_final_report()
            return
        
        task_name = self.task_sequence[self.current_task_index]
        
        task_info = task_name
        if task_name.startswith('dynamic_'):
            parts = task_name.split('_')
            if len(parts) >= 3:
                traj_type = parts[1]
                speed = parts[2]
                task_info = f"{traj_type} @ {speed}"
        
        rospy.loginfo("=" * 60)
        rospy.loginfo(">>> [TaskManager] 开始任务 %d/%d: %s", 
                      self.current_task_index + 1, 
                      len(self.task_sequence), 
                      task_info)
        rospy.loginfo("=" * 60)
        
        self.status_pub.publish(f"开始任务: {task_name}")
        
        config = self.task_configs[task_name]
        self._launch_benchmark_node(config)
    
    def _launch_benchmark_node(self, config):
        """启动 benchmark 节点"""
        self.task_completion_received = False
        
        args = []
        for key, value in config.items():
            args.append(f"{key}:='{value}'")
        
        cmd = f"roslaunch benchmark_utils benchmark.launch {' '.join(args)}"
        
        rospy.loginfo("=" * 60)
        rospy.loginfo(">>> [TaskManager] 启动 benchmark 节点")
        rospy.loginfo(">>> [TaskManager] 启动命令: %s", cmd)
        rospy.loginfo("=" * 60)
        
        try:
            self.current_benchmark_process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            rospy.loginfo(">>> [TaskManager] benchmark 节点已启动，PID: %d", self.current_benchmark_process.pid)
        except Exception as e:
            rospy.logerr(">>> [TaskManager] 启动 benchmark 节点失败: %s", e)
    
    def _stop_current_task(self):
        """停止当前任务"""
        if self.current_benchmark_process is not None:
            rospy.loginfo("=" * 60)
            rospy.loginfo(">>> [TaskManager] 停止当前任务...")
            rospy.loginfo("=" * 60)
            self.current_benchmark_process.terminate()
            try:
                self.current_benchmark_process.wait(timeout=5)
                rospy.loginfo(">>> [TaskManager] 任务已正常停止")
            except subprocess.TimeoutExpired:
                self.current_benchmark_process.kill()
                self.current_benchmark_process.wait()
                rospy.logwarn(">>> [TaskManager] 任务被强制停止")
            
            self.current_benchmark_process = None
        
        self.current_task_index += 1
        rospy.loginfo(">>> [TaskManager] 任务索引更新为: %d/%d", 
                      self.current_task_index, len(self.task_sequence))
    
    def _monitor_process(self, event):
        """监控benchmark_node进程状态"""
        if self.current_benchmark_process is not None:
            return_code = self.current_benchmark_process.poll()
            if return_code is not None:
                rospy.logwarn("benchmark_node进程已退出，返回码: %d", return_code)
                if return_code != 0:
                    stderr = self.current_benchmark_process.stderr.read()
                    if stderr:
                        rospy.logerr("启动错误: %s", stderr)
            else:
                if self.current_task_index < len(self.task_sequence):
                    current_task = self.task_sequence[self.current_task_index]
                    total_tasks = len(self.task_sequence)
                    progress = (self.current_task_index + 1) / total_tasks * 100
                    
                    task_info = current_task
                    if current_task.startswith('dynamic_'):
                        parts = current_task.split('_')
                        if len(parts) >= 3:
                            traj_type = parts[1]
                            speed = parts[2]
                            task_info = f"{traj_type} @ {speed}"
                    
                    rospy.loginfo(">>> [TaskManager] 任务: %s | 进度: %d/%d (%.1f%%)", 
                                 task_info, self.current_task_index + 1, total_tasks, progress)
    
    def _parse_task_report(self, task_name):
        """解析单个任务的报告文件"""
        pattern = os.path.join(self.log_dir, f"report_{task_name}_*.txt")
        report_files = glob.glob(pattern)
        
        if not report_files:
            return None
            
        latest_report = max(report_files, key=os.path.getctime)
        
        metrics = {}
        try:
            with open(latest_report, 'r', encoding='utf-8') as f:
                content = f.read()
                
                max_pos_match = re.search(r'最大位置误差:\s+([\d.]+)\s+m', content)
                mean_pos_match = re.search(r'平均位置误差:\s+([\d.]+)\s+m', content)
                rms_pos_match = re.search(r'RMS位置误差:\s+([\d.]+)\s+m', content)
                std_pos_match = re.search(r'位置误差标准差:\s+([\d.]+)\s+m', content)
                max_vel_match = re.search(r'最大速度误差:\s+([\d.]+)\s+m/s', content)
                mean_vel_match = re.search(r'平均速度误差:\s+([\d.]+)\s+m/s', content)
                sample_match = re.search(r'采样点数:\s+(\d+)', content)
                duration_match = re.search(r'实际执行时间:\s+([\d.]+)\s+秒', content)
                
                if max_pos_match:
                    metrics['max_pos_err'] = float(max_pos_match.group(1))
                if mean_pos_match:
                    metrics['mean_pos_err'] = float(mean_pos_match.group(1))
                if rms_pos_match:
                    metrics['rms_pos_err'] = float(rms_pos_match.group(1))
                if std_pos_match:
                    metrics['std_pos_err'] = float(std_pos_match.group(1))
                if max_vel_match:
                    metrics['max_vel_err'] = float(max_vel_match.group(1))
                if mean_vel_match:
                    metrics['mean_vel_err'] = float(mean_vel_match.group(1))
                if sample_match:
                    metrics['sample_count'] = int(sample_match.group(1))
                if duration_match:
                    metrics['duration'] = float(duration_match.group(1))
                    
                metrics['report_file'] = latest_report
                
        except Exception as e:
            rospy.logwarn("解析任务 %s 报告失败: %s", task_name, str(e))
            
        return metrics if metrics else None
    
    def _generate_final_report(self):
        """生成最终综合报告"""
        rospy.loginfo("=" * 70)
        rospy.loginfo("开始生成综合分析报告...")
        rospy.loginfo("=" * 70)
        
        if ANALYZER_AVAILABLE:
            analyzer = BenchmarkAnalyzer(self.log_dir)
            
            rospy.loginfo("使用 BenchmarkAnalyzer 进行详细分析...")
            results = analyzer.analyze_all_tasks(self.task_sequence)
            
            if results:
                rospy.loginfo("生成综合对比报告...")
                analyzer.generate_summary_report(results)
                
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                summary_file = os.path.join(self.log_dir, f"benchmark_summary_{timestamp}.txt")
                
                with open(summary_file, 'w', encoding='utf-8') as f:
                    f.write("=" * 70 + "\n")
                    f.write("Benchmark 综合测试报告\n")
                    f.write("=" * 70 + "\n\n")
                    
                    f.write("【测试概览】\n")
                    f.write("-" * 70 + "\n")
                    f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                    f.write(f"任务序列: {' -> '.join(self.task_sequence)}\n")
                    f.write(f"日志目录: {self.log_dir}\n\n")
                    
                    f.write("【各任务详细结果】\n")
                    f.write("=" * 70 + "\n\n")
                    
                    for result in results:
                        task_name = result['task_name']
                        metrics = result['metrics']
                        
                        f.write(f"任务: {task_name.upper()}\n")
                        f.write("-" * 70 + "\n")
                        f.write(f"  采样点数: {metrics['sample_count']}\n")
                        f.write(f"  执行时间: {metrics['duration']:.2f} 秒\n\n")
                        
                        f.write(f"  位置跟踪性能:\n")
                        f.write(f"    最大位置误差: {metrics['max_position_error']:.4f} m\n")
                        f.write(f"    平均位置误差: {metrics['mean_position_error']:.4f} m\n")
                        f.write(f"    RMS位置误差: {metrics['rms_position_error']:.4f} m\n")
                        f.write(f"    位置误差标准差: {metrics['std_position_error']:.4f} m\n\n")
                        
                        f.write(f"  速度跟踪性能:\n")
                        f.write(f"    最大速度误差: {metrics['max_velocity_error']:.4f} m/s\n")
                        f.write(f"    平均速度误差: {metrics['mean_velocity_error']:.4f} m/s\n")
                        f.write(f"    RMS速度误差: {metrics['rms_velocity_error']:.4f} m/s\n\n")
                        
                        f.write(f"  任务完成率: {metrics['completion_rate']*100:.2f}%\n\n")
                        
                        f.write(f"  生成的图表:\n")
                        for plot_name, plot_path in result['plots'].items():
                            f.write(f"    {plot_name}: {plot_path}\n")
                        f.write(f"\n  详细分析报告: {result['report_file']}\n\n")
                    
                    if len(results) > 1:
                        f.write("=" * 70 + "\n")
                        f.write("【性能对比】\n")
                        f.write("=" * 70 + "\n\n")
                        
                        f.write("位置误差对比:\n")
                        f.write("-" * 70 + "\n")
                        f.write(f"{'任务':<15} {'最大误差(m)':<15} {'平均误差(m)':<15} {'RMS误差(m)':<15}\n")
                        f.write("-" * 70 + "\n")
                        
                        for result in results:
                            task_name = result['task_name']
                            metrics = result['metrics']
                            f.write(f"{task_name:<15} {metrics['max_position_error']:<15.4f} "
                                   f"{metrics['mean_position_error']:<15.4f} "
                                   f"{metrics['rms_position_error']:<15.4f}\n")
                        
                        f.write("\n")
                        
                        f.write("速度误差对比:\n")
                        f.write("-" * 70 + "\n")
                        f.write(f"{'任务':<15} {'最大误差(m/s)':<20} {'平均误差(m/s)':<20}\n")
                        f.write("-" * 70 + "\n")
                        
                        for result in results:
                            task_name = result['task_name']
                            metrics = result['metrics']
                            f.write(f"{task_name:<15} {metrics['max_velocity_error']:<20.4f} "
                                   f"{metrics['mean_velocity_error']:<20.4f}\n")
                    
                    f.write("\n" + "=" * 70 + "\n")
                    f.write("报告生成完成\n")
                    f.write("=" * 70 + "\n")
                
                rospy.loginfo("=" * 70)
                rospy.loginfo("综合报告已生成: %s", summary_file)
                rospy.loginfo("=" * 70)
            else:
                rospy.logwarn("没有找到可分析的数据")
        else:
            rospy.logwarn("BenchmarkAnalyzer 不可用，使用简化报告生成方式")
            self._generate_simple_report()
    
    def _generate_simple_report(self):
        """生成简化版报告（当 BenchmarkAnalyzer 不可用时）"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_file = os.path.join(self.log_dir, f"benchmark_summary_{timestamp}.txt")
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("=" * 70 + "\n")
            f.write("Benchmark 综合测试报告（简化版）\n")
            f.write("=" * 70 + "\n\n")
            
            f.write("【测试概览】\n")
            f.write("-" * 70 + "\n")
            f.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"任务序列: {' -> '.join(self.task_sequence)}\n")
            f.write(f"日志目录: {self.log_dir}\n\n")
            
            f.write("【各任务详细结果】\n")
            f.write("=" * 70 + "\n\n")
            
            all_metrics = []
            for i, task_name in enumerate(self.task_sequence):
                f.write(f"任务 {i+1}: {task_name.upper()}\n")
                f.write("-" * 70 + "\n")
                
                metrics = self._parse_task_report(task_name)
                
                if metrics:
                    all_metrics.append((task_name, metrics))
                    
                    f.write(f"  状态: 完成\n")
                    f.write(f"  执行时间: {metrics.get('duration', 'N/A')} 秒\n")
                    f.write(f"  采样点数: {metrics.get('sample_count', 'N/A')}\n\n")
                    
                    f.write("  位置跟踪性能:\n")
                    f.write(f"    最大位置误差: {metrics.get('max_pos_err', 'N/A'):.4f} m\n")
                    f.write(f"    平均位置误差: {metrics.get('mean_pos_err', 'N/A'):.4f} m\n")
                    f.write(f"    RMS位置误差: {metrics.get('rms_pos_err', 'N/A'):.4f} m\n")
                    f.write(f"    位置误差标准差: {metrics.get('std_pos_err', 'N/A'):.4f} m\n\n")
                    
                    f.write("  速度跟踪性能:\n")
                    f.write(f"    最大速度误差: {metrics.get('max_vel_err', 'N/A'):.4f} m/s\n")
                    f.write(f"    平均速度误差: {metrics.get('mean_vel_err', 'N/A'):.4f} m/s\n\n")
                    
                    f.write(f"  详细报告: {metrics.get('report_file', 'N/A')}\n")
                else:
                    f.write(f"  状态: 报告未找到或解析失败\n")
                
                f.write("\n")
            
            if all_metrics:
                f.write("=" * 70 + "\n")
                f.write("【综合性能评估】\n")
                f.write("=" * 70 + "\n\n")
                
                f.write("位置误差对比:\n")
                f.write("-" * 70 + "\n")
                f.write(f"{'任务':<15} {'最大误差(m)':<15} {'平均误差(m)':<15} {'RMS误差(m)':<15}\n")
                f.write("-" * 70 + "\n")
                
                for task_name, metrics in all_metrics:
                    f.write(f"{task_name:<15} {metrics.get('max_pos_err', 0):<15.4f} "
                           f"{metrics.get('mean_pos_err', 0):<15.4f} "
                           f"{metrics.get('rms_pos_err', 0):<15.4f}\n")
                
                f.write("\n")
                
                f.write("速度误差对比:\n")
                f.write("-" * 70 + "\n")
                f.write(f"{'任务':<15} {'最大误差(m/s)':<20} {'平均误差(m/s)':<20}\n")
                f.write("-" * 70 + "\n")
                
                for task_name, metrics in all_metrics:
                    f.write(f"{task_name:<15} {metrics.get('max_vel_err', 0):<20.4f} "
                           f"{metrics.get('mean_vel_err', 0):<20.4f}\n")
                
                f.write("\n")
                
                avg_max_pos_err = sum(m.get('max_pos_err', 0) for _, m in all_metrics) / len(all_metrics)
                avg_mean_pos_err = sum(m.get('mean_pos_err', 0) for _, m in all_metrics) / len(all_metrics)
                avg_rms_pos_err = sum(m.get('rms_pos_err', 0) for _, m in all_metrics) / len(all_metrics)
                
                f.write("整体性能指标:\n")
                f.write("-" * 70 + "\n")
                f.write(f"平均最大位置误差: {avg_max_pos_err:.4f} m\n")
                f.write(f"平均平均位置误差: {avg_mean_pos_err:.4f} m\n")
                f.write(f"平均RMS位置误差: {avg_rms_pos_err:.4f} m\n")
            
            f.write("\n" + "=" * 70 + "\n")
            f.write("报告生成完成\n")
            f.write("=" * 70 + "\n")
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("综合报告已生成: %s", report_file)
        rospy.loginfo("=" * 70)
        
        self.status_pub.publish("所有任务完成，综合报告已生成")


def main():
    try:
        task_manager = TaskManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("任务管理器被中断")
    finally:
        rospy.loginfo("任务管理器退出")


if __name__ == '__main__':
    main()
