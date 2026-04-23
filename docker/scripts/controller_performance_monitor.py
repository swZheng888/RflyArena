#!/usr/bin/env python3
"""
控制器性能监控器 - 监控 NMPC/PID 控制器的实际性能

关键指标：
1. 控制器进程 CPU 使用率（而非整个系统）
2. 控制器进程内存使用
3. 控制频率（Hz）
4. 单次计算延迟（ms）
5. 话题发布频率

用法：
    python3 controller_performance_monitor.py --controller nmpc --duration 60
"""

import os
import sys
import time
import argparse
import csv
import signal
import subprocess
from datetime import datetime
from collections import deque
from typing import Optional, Dict, List

try:
    import psutil
except ImportError:
    print("Error: psutil not installed. Run: pip3 install psutil")
    sys.exit(1)

try:
    import rospy
    from std_msgs.msg import Header
    from mavros_msgs.msg import AttitudeTarget
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("Warning: ROS not available, using system-level monitoring only")


class ControllerPerformanceMonitor:
    """控制器性能监控器"""

    def __init__(self, controller_type: str, output_dir: str, duration: int = 60):
        self.controller_type = controller_type
        self.output_dir = output_dir
        self.duration = duration
        self.running = True
        self.start_time = time.time()

        # 进程信息
        self.controller_process: Optional[psutil.Process] = None
        self.controller_pid: Optional[int] = None

        # 性能数据
        self.samples: List[Dict] = []
        self.topic_timestamps: deque = deque(maxlen=1000)
        self.last_topic_time: Optional[float] = None

        # 统计
        self.cpu_samples: List[float] = []
        self.mem_samples: List[float] = []
        self.freq_samples: List[float] = []
        self.latency_samples: List[float] = []

        # 资源限制
        self.cpu_limit = self._get_cpu_limit()
        self.mem_limit_mb = self._get_memory_limit() / (1024 * 1024)

        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)

    def _get_cpu_limit(self) -> float:
        """获取 Cgroup CPU 限制"""
        try:
            quota_file = "/sys/fs/cgroup/cpu/cpu.cfs_quota_us"
            period_file = "/sys/fs/cgroup/cpu/cpu.cfs_period_us"

            if os.path.exists(quota_file):
                with open(quota_file, 'r') as f:
                    quota = int(f.read().strip())
                with open(period_file, 'r') as f:
                    period = int(f.read().strip())
                if quota > 0:
                    return quota / period
        except:
            pass
        return float(psutil.cpu_count())

    def _get_memory_limit(self) -> int:
        """获取 Cgroup 内存限制 (bytes)"""
        try:
            limit_file = "/sys/fs/cgroup/memory/memory.limit_in_bytes"
            if os.path.exists(limit_file):
                with open(limit_file, 'r') as f:
                    limit = int(f.read().strip())
                    if limit < 9223372036854771712:
                        return limit
        except:
            pass
        return psutil.virtual_memory().total

    def find_controller_process(self) -> bool:
        """查找控制器进程"""
        search_patterns = {
            'nmpc': ['control_nmpc_node', 'nmpc_control_node', 'python.*control_nmpc'],
            'pid': ['px4ctrl', 'px4ctrl_node'],
        }

        patterns = search_patterns.get(self.controller_type, [])

        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline'] or [])
                name = proc.info['name'] or ''

                for pattern in patterns:
                    if pattern in cmdline or pattern in name:
                        self.controller_process = psutil.Process(proc.info['pid'])
                        self.controller_pid = proc.info['pid']
                        print(f"Found controller process: PID={self.controller_pid}, name={name}")
                        return True
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue

        return False

    def get_topic_frequency(self) -> float:
        """计算话题发布频率"""
        if len(self.topic_timestamps) < 2:
            return 0.0

        # 使用最近的时间戳计算频率
        timestamps = list(self.topic_timestamps)
        if len(timestamps) < 2:
            return 0.0

        # 计算最近 1 秒内的频率
        now = time.time()
        recent = [t for t in timestamps if now - t < 1.0]
        if len(recent) < 2:
            return len(recent)

        duration = recent[-1] - recent[0]
        if duration > 0:
            return (len(recent) - 1) / duration
        return 0.0

    def attitude_callback(self, msg):
        """姿态指令话题回调"""
        now = time.time()
        self.topic_timestamps.append(now)

        if self.last_topic_time is not None:
            latency = (now - self.last_topic_time) * 1000  # ms
            self.latency_samples.append(latency)

        self.last_topic_time = now

    def sample_performance(self) -> Dict:
        """采样一次性能数据"""
        sample = {
            'timestamp': datetime.now().isoformat(),
            'elapsed_s': time.time() - self.start_time,
            'controller_cpu_percent': 0.0,
            'controller_mem_mb': 0.0,
            'controller_mem_percent': 0.0,
            'controller_threads': 0,
            'topic_freq_hz': 0.0,
            'avg_latency_ms': 0.0,
            'system_cpu_percent': 0.0,
            'system_mem_percent': 0.0,
        }

        # 控制器进程性能
        if self.controller_process:
            try:
                # CPU 使用率（相对于单核）
                cpu_percent = self.controller_process.cpu_percent()
                # 转换为相对于 CPU 限制的百分比
                sample['controller_cpu_percent'] = cpu_percent / self.cpu_limit

                mem_info = self.controller_process.memory_info()
                sample['controller_mem_mb'] = mem_info.rss / (1024 * 1024)
                sample['controller_mem_percent'] = (mem_info.rss / (self.mem_limit_mb * 1024 * 1024)) * 100
                sample['controller_threads'] = self.controller_process.num_threads()

                self.cpu_samples.append(sample['controller_cpu_percent'])
                self.mem_samples.append(sample['controller_mem_mb'])

            except (psutil.NoSuchProcess, psutil.AccessDenied):
                # 进程可能已退出，尝试重新查找
                self.find_controller_process()

        # 话题频率
        sample['topic_freq_hz'] = self.get_topic_frequency()
        if sample['topic_freq_hz'] > 0:
            self.freq_samples.append(sample['topic_freq_hz'])

        # 平均延迟
        if len(self.latency_samples) > 0:
            recent_latencies = self.latency_samples[-100:]  # 最近 100 个
            sample['avg_latency_ms'] = sum(recent_latencies) / len(recent_latencies)

        # 系统级别
        sample['system_cpu_percent'] = psutil.cpu_percent()
        sample['system_mem_percent'] = psutil.virtual_memory().percent

        return sample

    def run_with_ros(self):
        """使用 ROS 运行监控"""
        rospy.init_node('controller_performance_monitor', anonymous=True)

        # 订阅姿态控制话题
        rospy.Subscriber('/mavros/setpoint_raw/attitude',
                         AttitudeTarget,
                         self.attitude_callback)

        print("ROS node started, monitoring /mavros/setpoint_raw/attitude")

        rate = rospy.Rate(10)  # 10 Hz 采样

        while not rospy.is_shutdown() and self.running:
            if time.time() - self.start_time > self.duration:
                break

            sample = self.sample_performance()
            self.samples.append(sample)

            # 实时输出
            print(f"\r[{sample['elapsed_s']:6.1f}s] "
                  f"CPU: {sample['controller_cpu_percent']:5.1f}% | "
                  f"MEM: {sample['controller_mem_mb']:6.1f}MB | "
                  f"Freq: {sample['topic_freq_hz']:5.1f}Hz | "
                  f"Lat: {sample['avg_latency_ms']:5.2f}ms",
                  end='', flush=True)

            rate.sleep()

    def run_without_ros(self):
        """不使用 ROS 运行监控（纯进程监控）"""
        print("Running without ROS, monitoring process only")

        # 使用 rostopic hz 获取频率
        hz_process = None
        try:
            hz_process = subprocess.Popen(
                ['rostopic', 'hz', '/mavros/setpoint_raw/attitude'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
        except:
            print("Warning: rostopic hz not available")

        sample_interval = 0.1  # 100ms

        while self.running and (time.time() - self.start_time < self.duration):
            sample = self.sample_performance()
            self.samples.append(sample)

            # 实时输出
            if len(self.samples) % 10 == 0:
                print(f"\r[{sample['elapsed_s']:6.1f}s] "
                      f"CPU: {sample['controller_cpu_percent']:5.1f}% | "
                      f"MEM: {sample['controller_mem_mb']:6.1f}MB | "
                      f"Threads: {sample['controller_threads']}",
                      end='', flush=True)

            time.sleep(sample_interval)

        if hz_process:
            hz_process.terminate()

    def run(self):
        """运行监控"""
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, 'running', False))
        signal.signal(signal.SIGTERM, lambda s, f: setattr(self, 'running', False))

        print("=" * 60)
        print("Controller Performance Monitor")
        print("=" * 60)
        print(f"  Controller: {self.controller_type}")
        print(f"  Duration: {self.duration}s")
        print(f"  CPU Limit: {self.cpu_limit:.2f} cores")
        print(f"  Memory Limit: {self.mem_limit_mb:.0f} MB")
        print(f"  Output: {self.output_dir}")
        print("=" * 60)
        print()

        # 初始化 CPU 计数器
        psutil.cpu_percent(interval=None)

        # 查找控制器进程
        print("Searching for controller process...")
        for _ in range(30):  # 等待最多 30 秒
            if self.find_controller_process():
                break
            time.sleep(1)
            print(".", end='', flush=True)

        if not self.controller_process:
            print("\nWarning: Controller process not found, monitoring system only")

        print("\nStarting monitoring...")

        # 根据 ROS 可用性选择运行模式
        if ROS_AVAILABLE:
            try:
                self.run_with_ros()
            except:
                self.run_without_ros()
        else:
            self.run_without_ros()

        self.save_results()
        self.print_summary()

    def save_results(self):
        """保存结果到 CSV"""
        if not self.samples:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file = os.path.join(self.output_dir,
                                f"performance_{self.controller_type}_{timestamp}.csv")

        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.samples[0].keys())
            writer.writeheader()
            writer.writerows(self.samples)

        print(f"\nResults saved to: {csv_file}")

    def print_summary(self):
        """打印统计摘要"""
        print("\n")
        print("=" * 60)
        print("Performance Summary")
        print("=" * 60)

        if self.cpu_samples:
            print(f"  CPU Usage (controller):")
            print(f"    Average: {sum(self.cpu_samples)/len(self.cpu_samples):.1f}%")
            print(f"    Max:     {max(self.cpu_samples):.1f}%")
            print(f"    Min:     {min(self.cpu_samples):.1f}%")

        if self.mem_samples:
            print(f"  Memory Usage (controller):")
            print(f"    Average: {sum(self.mem_samples)/len(self.mem_samples):.1f} MB")
            print(f"    Max:     {max(self.mem_samples):.1f} MB")

        if self.freq_samples:
            print(f"  Control Frequency:")
            print(f"    Average: {sum(self.freq_samples)/len(self.freq_samples):.1f} Hz")
            print(f"    Max:     {max(self.freq_samples):.1f} Hz")
            print(f"    Min:     {min(self.freq_samples):.1f} Hz")

        if self.latency_samples:
            print(f"  Control Latency:")
            print(f"    Average: {sum(self.latency_samples)/len(self.latency_samples):.2f} ms")
            print(f"    Max:     {max(self.latency_samples):.2f} ms")
            print(f"    P99:     {sorted(self.latency_samples)[int(len(self.latency_samples)*0.99)]:.2f} ms")

        print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description='Controller Performance Monitor')
    parser.add_argument('--controller', '-c', default='nmpc',
                        choices=['nmpc', 'pid'],
                        help='Controller type')
    parser.add_argument('--duration', '-d', type=int, default=60,
                        help='Monitoring duration in seconds')
    parser.add_argument('--output', '-o', default='/tmp/controller_performance',
                        help='Output directory')

    args = parser.parse_args()

    monitor = ControllerPerformanceMonitor(
        controller_type=args.controller,
        output_dir=args.output,
        duration=args.duration
    )
    monitor.run()


if __name__ == '__main__':
    main()
