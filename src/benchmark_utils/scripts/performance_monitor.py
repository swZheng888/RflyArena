#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
performance_monitor.py

性能监控器 - Task 4专用

功能:
- 监控目标进程的CPU占用率
- 监控内存使用
- 计算控制循环频率
- 记录性能数据并生成报告

使用示例:
    rosrun benchmark_utils performance_monitor.py \\
        --target-node nmpc_control_node \\
        --log-dir ~/.benchmark_logs
"""

import rospy
import psutil
import os
import time
import csv
from datetime import datetime
from std_msgs.msg import String, Float32


class PerformanceMonitor:
    """
    性能监控器节点
    
    监控控制器进程的资源使用情况
    """
    
    def __init__(self):
        rospy.init_node('performance_monitor', anonymous=True)
        
        # 参数
        self.target_node_name = rospy.get_param('~target_node_name', 'nmpc_control_node')
        self.log_dir = rospy.get_param('~log_dir', '~/.benchmark_logs')
        self.log_dir = os.path.expanduser(self.log_dir)
        self.monitor_rate = rospy.get_param('~rate', 10.0)  # Hz
        
        # 状态
        self.target_process = None
        self.monitoring = False
        self.log_file = None
        self.log_writer = None
        self.start_time = None
        
        # 统计数据
        self.cpu_samples = []
        self.mem_samples = []
        
        # ROS接口
        self._setup_ros_interface()
        
        # 查找目标进程
        self._find_target_process()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Performance Monitor 已启动")
        rospy.loginfo("目标节点: %s", self.target_node_name)
        if self.target_process:
            rospy.loginfo("进程PID: %d", self.target_process.pid)
        else:
            rospy.logwarn("未找到目标进程，将持续搜索...")
        rospy.loginfo("=" * 60)
        
    def _setup_ros_interface(self):
        """设置ROS接口"""
        # 发布者
        self.cpu_pub = rospy.Publisher('/performance_monitor/cpu_usage', Float32, queue_size=10)
        self.mem_pub = rospy.Publisher('/performance_monitor/memory_mb', Float32, queue_size=10)
        
        # 订阅benchmark控制信号
        rospy.Subscriber('/benchmark/logging_control', String, 
                        self._control_callback, queue_size=10)
                        
        # 定时器
        self.monitor_timer = rospy.Timer(rospy.Duration(1.0 / self.monitor_rate), 
                                        self._monitor_loop)
                                        
    def _find_target_process(self):
        """查找目标进程"""
        for proc in psutil.process_iter(['name', 'cmdline']):
            try:
                cmdline = proc.cmdline()
                # 检查命令行中是否包含目标节点名
                if any(self.target_node_name in arg for arg in cmdline):
                    self.target_process = proc
                    rospy.loginfo("找到目标进程: PID=%d, 名称=%s", 
                                proc.pid, proc.name())
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return False
        
    def _control_callback(self, msg):
        """控制信号回调"""
        command = msg.data.lower()
        
        if command == "start" and not self.monitoring:
            self._start_monitoring()
        elif command == "stop" and self.monitoring:
            self._stop_monitoring()
            
    def _start_monitoring(self):
        """开始监控"""
        # 确保找到进程
        if not self.target_process:
            if not self._find_target_process():
                rospy.logwarn("无法开始监控：未找到目标进程")
                return
                
        # 创建日志文件
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"performance_{timestamp}.csv"
        log_path = os.path.join(self.log_dir, filename)
        
        self.log_file = open(log_path, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['time', 'cpu_percent', 'memory_mb'])
        
        self.monitoring = True
        self.start_time = time.time()
        self.cpu_samples = []
        self.mem_samples = []
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("开始性能监控")
        rospy.loginfo("日志文件: %s", log_path)
        rospy.loginfo("=" * 60)
        
    def _stop_monitoring(self):
        """停止监控"""
        self.monitoring = False
        
        if self.log_file:
            self.log_file.close()
            
        # 计算统计信息
        if self.cpu_samples:
            avg_cpu = sum(self.cpu_samples) / len(self.cpu_samples)
            max_cpu = max(self.cpu_samples)
            avg_mem = sum(self.mem_samples) / len(self.mem_samples)
            max_mem = max(self.mem_samples)
            
            rospy.loginfo("=" * 60)
            rospy.loginfo("性能监控完成")
            rospy.loginfo("CPU占用 - 平均: %.2f%%, 最大: %.2f%%", avg_cpu, max_cpu)
            rospy.loginfo("内存占用 - 平均: %.2f MB, 最大: %.2f MB", avg_mem, max_mem)
            rospy.loginfo("=" * 60)
            
            # 生成报告
            self._generate_report(avg_cpu, max_cpu, avg_mem, max_mem)
            
    def _monitor_loop(self, event):
        """监控循环"""
        if not self.monitoring:
            return
            
        if not self.target_process:
            return
            
        try:
            # 获取CPU使用率（过去1秒的平均）
            cpu_percent = self.target_process.cpu_percent(interval=0)
            
            # 获取内存使用（MB）
            mem_info = self.target_process.memory_info()
            mem_mb = mem_info.rss / 1024 / 1024
            
            # 记录数据
            elapsed = time.time() - self.start_time
            self.log_writer.writerow([elapsed, cpu_percent, mem_mb])
            
            # 发布话题
            self.cpu_pub.publish(Float32(cpu_percent))
            self.mem_pub.publish(Float32(mem_mb))
            
            # 保存样本
            self.cpu_samples.append(cpu_percent)
            self.mem_samples.append(mem_mb)
            
            # 定期刷新文件
            if len(self.cpu_samples) % 100 == 0:
                self.log_file.flush()
                
        except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
            rospy.logwarn_throttle(5.0, "无法访问目标进程: %s", str(e))
            self.target_process = None
            self._find_target_process()
            
    def _generate_report(self, avg_cpu, max_cpu, avg_mem, max_mem):
        """生成性能报告"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_path = os.path.join(self.log_dir, f"performance_report_{timestamp}.txt")
        
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("=" * 70 + "\n")
            f.write("性能监控报告\n")
            f.write("=" * 70 + "\n\n")
            
            f.write("【监控信息】\n")
            f.write("-" * 70 + "\n")
            f.write(f"目标节点: {self.target_node_name}\n")
            f.write(f"进程PID: {self.target_process.pid if self.target_process else 'N/A'}\n")
            f.write(f"监控时长: {time.time() - self.start_time:.2f} 秒\n")
            f.write(f"采样数量: {len(self.cpu_samples)}\n\n")
            
            f.write("【CPU占用】\n")
            f.write("-" * 70 + "\n")
            f.write(f"平均: {avg_cpu:.2f}%\n")
            f.write(f"最大: {max_cpu:.2f}%\n")
            if self.cpu_samples:
                p95_cpu = sorted(self.cpu_samples)[int(len(self.cpu_samples) * 0.95)]
                f.write(f"P95: {p95_cpu:.2f}%\n")
            f.write("\n")
            
            f.write("【内存占用】\n")
            f.write("-" * 70 + "\n")
            f.write(f"平均: {avg_mem:.2f} MB\n")
            f.write(f"最大: {max_mem:.2f} MB\n\n")
            
            # 评价
            f.write("【性能评价】\n")
            f.write("-" * 70 + "\n")
            if avg_cpu < 50:
                f.write("CPU占用: 良好（低于50%）\n")
            elif avg_cpu < 80:
                f.write("CPU占用: 中等（50-80%）\n")
            else:
                f.write("CPU占用: 较高（超过80%）\n")
                
            if max_mem < 500:
                f.write("内存占用: 良好（低于500MB）\n")
            elif max_mem < 1000:
                f.write("内存占用: 中等（500-1000MB）\n")
            else:
                f.write("内存占用: 较高（超过1000MB）\n")
                
            f.write("\n" + "=" * 70 + "\n")
            
        rospy.loginfo("性能报告已生成: %s", report_path)
        
    def run(self):
        """运行节点"""
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = PerformanceMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
