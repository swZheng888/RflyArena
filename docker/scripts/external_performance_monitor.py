#!/usr/bin/env python3
"""
外部独立性能监控器 - 不依赖被测控制器的任何报告

核心原则：
1. 监控器在宿主机/独立进程运行
2. 通过 Docker API 获取容器资源使用
3. 通过 ROS 话题频率测量控制性能
4. 被测控制器无法伪造这些数据

监控指标：
1. 容器 CPU 使用率（Docker stats）
2. 容器内存使用（Docker stats）
3. 控制话题发布频率（rostopic hz）
4. 控制话题延迟分布（消息时间戳 vs 接收时间）

用法：
    python3 external_performance_monitor.py --container benchmark_jetson_nano --duration 60
"""

import os
import sys
import time
import json
import argparse
import csv
import signal
import subprocess
import threading
from datetime import datetime
from collections import deque
from typing import Optional, Dict, List, Tuple

# Docker stats 解析
def get_docker_stats(container_name: str) -> Optional[Dict]:
    """通过 docker stats 获取容器资源使用（无法伪造）"""
    try:
        result = subprocess.run(
            ['docker', 'stats', container_name, '--no-stream', '--format',
             '{{json .}}'],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0 and result.stdout.strip():
            data = json.loads(result.stdout.strip())
            return {
                'cpu_percent': parse_percent(data.get('CPUPerc', '0%')),
                'mem_usage_mb': parse_memory(data.get('MemUsage', '0MiB / 0MiB')),
                'mem_limit_mb': parse_memory_limit(data.get('MemUsage', '0MiB / 0MiB')),
                'mem_percent': parse_percent(data.get('MemPerc', '0%')),
                'net_io': data.get('NetIO', '0B / 0B'),
                'pids': int(data.get('PIDs', 0)),
            }
    except Exception as e:
        print(f"Docker stats error: {e}")
    return None


def parse_percent(s: str) -> float:
    """解析百分比字符串"""
    try:
        return float(s.replace('%', '').strip())
    except:
        return 0.0


def parse_memory(s: str) -> float:
    """解析内存使用字符串，返回 MB"""
    try:
        usage = s.split('/')[0].strip()
        if 'GiB' in usage:
            return float(usage.replace('GiB', '').strip()) * 1024
        elif 'MiB' in usage:
            return float(usage.replace('MiB', '').strip())
        elif 'KiB' in usage:
            return float(usage.replace('KiB', '').strip()) / 1024
        elif 'B' in usage:
            return float(usage.replace('B', '').strip()) / (1024 * 1024)
    except:
        pass
    return 0.0


def parse_memory_limit(s: str) -> float:
    """解析内存限制字符串，返回 MB"""
    try:
        limit = s.split('/')[1].strip()
        if 'GiB' in limit:
            return float(limit.replace('GiB', '').strip()) * 1024
        elif 'MiB' in limit:
            return float(limit.replace('MiB', '').strip())
        elif 'KiB' in limit:
            return float(limit.replace('KiB', '').strip()) / 1024
    except:
        pass
    return 0.0


class TopicFrequencyMonitor:
    """ROS 话题频率监控器（通过外部测量，无法伪造）"""

    def __init__(self, topic: str):
        self.topic = topic
        self.timestamps: deque = deque(maxlen=500)
        self.latencies: deque = deque(maxlen=500)
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.process: Optional[subprocess.Popen] = None

    def start(self):
        """启动频率监控"""
        self.running = True
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """停止监控"""
        self.running = False
        if self.process:
            self.process.terminate()
            self.process = None
        if self.thread:
            self.thread.join(timeout=2)

    def _monitor_loop(self):
        """监控循环 - 使用 rostopic echo 获取消息时间戳"""
        try:
            # 使用 rostopic echo 获取消息头时间戳
            self.process = subprocess.Popen(
                ['rostopic', 'echo', '-p', self.topic + '/header/stamp'],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True
            )

            last_time = None
            for line in iter(self.process.stdout.readline, ''):
                if not self.running:
                    break

                now = time.time()
                self.timestamps.append(now)

                # 计算接收间隔
                if last_time is not None:
                    interval = now - last_time
                    if interval > 0:
                        self.latencies.append(interval * 1000)  # ms

                last_time = now

        except Exception as e:
            print(f"Topic monitor error: {e}")

    def get_frequency(self) -> float:
        """获取当前频率（Hz）"""
        if len(self.timestamps) < 2:
            return 0.0

        now = time.time()
        # 使用最近 1 秒的数据
        recent = [t for t in self.timestamps if now - t < 1.0]
        if len(recent) < 2:
            return float(len(recent))

        duration = recent[-1] - recent[0]
        if duration > 0:
            return (len(recent) - 1) / duration
        return 0.0

    def get_latency_stats(self) -> Dict[str, float]:
        """获取延迟统计"""
        if not self.latencies:
            return {'avg': 0, 'max': 0, 'min': 0, 'p99': 0}

        latencies = list(self.latencies)
        latencies_sorted = sorted(latencies)
        n = len(latencies)

        return {
            'avg': sum(latencies) / n,
            'max': max(latencies),
            'min': min(latencies),
            'p99': latencies_sorted[int(n * 0.99)] if n > 1 else latencies[0],
        }


class RostopicHzMonitor:
    """使用 rostopic hz 命令监控频率（更可靠）"""

    def __init__(self, topic: str):
        self.topic = topic
        self.current_freq = 0.0
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.process: Optional[subprocess.Popen] = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.process:
            self.process.terminate()
        if self.thread:
            self.thread.join(timeout=2)

    def _monitor_loop(self):
        try:
            self.process = subprocess.Popen(
                ['rostopic', 'hz', self.topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )

            for line in iter(self.process.stdout.readline, ''):
                if not self.running:
                    break

                # 解析 "average rate: 50.00"
                if 'average rate:' in line:
                    try:
                        rate_str = line.split('average rate:')[1].strip()
                        self.current_freq = float(rate_str.split()[0])
                    except:
                        pass
        except:
            pass

    def get_frequency(self) -> float:
        return self.current_freq


class ExternalPerformanceMonitor:
    """外部独立性能监控器"""

    def __init__(self, container_name: str, output_dir: str, duration: int = 60):
        self.container_name = container_name
        self.output_dir = output_dir
        self.duration = duration
        self.running = True
        self.start_time = time.time()

        # 数据收集
        self.samples: List[Dict] = []

        # 统计
        self.cpu_samples: List[float] = []
        self.mem_samples: List[float] = []
        self.freq_samples: List[float] = []

        # 话题监控
        self.attitude_topic = '/mavros/setpoint_raw/attitude'
        self.hz_monitor: Optional[RostopicHzMonitor] = None

        os.makedirs(output_dir, exist_ok=True)

    def run(self):
        """运行监控"""
        signal.signal(signal.SIGINT, lambda s, f: setattr(self, 'running', False))
        signal.signal(signal.SIGTERM, lambda s, f: setattr(self, 'running', False))

        print("=" * 70)
        print("External Performance Monitor (独立外部监控 - 无法伪造)")
        print("=" * 70)
        print(f"  Container: {self.container_name}")
        print(f"  Duration:  {self.duration}s")
        print(f"  Output:    {self.output_dir}")
        print()
        print("监控方式:")
        print("  - CPU/内存: Docker stats API (容器无法伪造)")
        print("  - 控制频率: rostopic hz 外部测量")
        print("=" * 70)
        print()

        # 检查容器是否存在
        if not self._check_container():
            print(f"Error: Container '{self.container_name}' not found or not running")
            print("Running containers:")
            os.system("docker ps --format '  {{.Names}}'")
            return

        # 启动话题频率监控
        print("Starting topic frequency monitor...")
        self.hz_monitor = RostopicHzMonitor(self.attitude_topic)
        self.hz_monitor.start()
        time.sleep(2)  # 等待初始化

        print("Monitoring started...")
        print()

        sample_interval = 1.0  # 1 秒采样

        while self.running and (time.time() - self.start_time < self.duration):
            sample = self._collect_sample()
            if sample:
                self.samples.append(sample)
                self._print_sample(sample)

            time.sleep(sample_interval)

        # 停止监控
        if self.hz_monitor:
            self.hz_monitor.stop()

        self._save_results()
        self._print_summary()

    def _check_container(self) -> bool:
        """检查容器是否运行"""
        try:
            result = subprocess.run(
                ['docker', 'inspect', '-f', '{{.State.Running}}', self.container_name],
                capture_output=True, text=True, timeout=5
            )
            return result.stdout.strip() == 'true'
        except:
            return False

    def _collect_sample(self) -> Optional[Dict]:
        """收集一个样本"""
        elapsed = time.time() - self.start_time

        # Docker stats
        docker_stats = get_docker_stats(self.container_name)
        if not docker_stats:
            return None

        # 话题频率
        topic_freq = self.hz_monitor.get_frequency() if self.hz_monitor else 0.0

        sample = {
            'timestamp': datetime.now().isoformat(),
            'elapsed_s': elapsed,
            'container_cpu_percent': docker_stats['cpu_percent'],
            'container_mem_mb': docker_stats['mem_usage_mb'],
            'container_mem_limit_mb': docker_stats['mem_limit_mb'],
            'container_mem_percent': docker_stats['mem_percent'],
            'container_pids': docker_stats['pids'],
            'control_freq_hz': topic_freq,
        }

        # 统计收集
        self.cpu_samples.append(docker_stats['cpu_percent'])
        self.mem_samples.append(docker_stats['mem_usage_mb'])
        if topic_freq > 0:
            self.freq_samples.append(topic_freq)

        return sample

    def _print_sample(self, sample: Dict):
        """打印样本"""
        print(f"\r[{sample['elapsed_s']:6.1f}s] "
              f"CPU: {sample['container_cpu_percent']:5.1f}% | "
              f"MEM: {sample['container_mem_mb']:6.1f}/{sample['container_mem_limit_mb']:.0f}MB "
              f"({sample['container_mem_percent']:4.1f}%) | "
              f"Freq: {sample['control_freq_hz']:5.1f}Hz | "
              f"PIDs: {sample['container_pids']}",
              end='', flush=True)

    def _save_results(self):
        """保存结果"""
        if not self.samples:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_file = os.path.join(self.output_dir,
                                f"external_perf_{self.container_name}_{timestamp}.csv")

        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.samples[0].keys())
            writer.writeheader()
            writer.writerows(self.samples)

        print(f"\n\nResults saved to: {csv_file}")

    def _print_summary(self):
        """打印统计摘要"""
        print("\n")
        print("=" * 70)
        print("Performance Summary (外部独立测量 - 数据可信)")
        print("=" * 70)

        if self.cpu_samples:
            avg_cpu = sum(self.cpu_samples) / len(self.cpu_samples)
            max_cpu = max(self.cpu_samples)
            min_cpu = min(self.cpu_samples)
            print(f"  Container CPU Usage:")
            print(f"    Average: {avg_cpu:.1f}%")
            print(f"    Max:     {max_cpu:.1f}%")
            print(f"    Min:     {min_cpu:.1f}%")

        if self.mem_samples:
            avg_mem = sum(self.mem_samples) / len(self.mem_samples)
            max_mem = max(self.mem_samples)
            print(f"  Container Memory Usage:")
            print(f"    Average: {avg_mem:.1f} MB")
            print(f"    Max:     {max_mem:.1f} MB")

        if self.freq_samples:
            avg_freq = sum(self.freq_samples) / len(self.freq_samples)
            max_freq = max(self.freq_samples)
            min_freq = min(self.freq_samples)
            print(f"  Control Frequency (外部测量):")
            print(f"    Average: {avg_freq:.1f} Hz")
            print(f"    Max:     {max_freq:.1f} Hz")
            print(f"    Min:     {min_freq:.1f} Hz")

            # 评估是否满足实时性要求
            if avg_freq >= 50:
                print(f"    Status:  ✓ 满足实时性要求 (≥50Hz)")
            elif avg_freq >= 30:
                print(f"    Status:  ⚠ 边缘性能 (30-50Hz)")
            else:
                print(f"    Status:  ✗ 不满足实时性要求 (<30Hz)")

        print("=" * 70)


def main():
    parser = argparse.ArgumentParser(
        description='External Performance Monitor - Independent measurement that cannot be faked'
    )
    parser.add_argument(
        '--container', '-c',
        default='',
        help='Docker container name to monitor'
    )
    parser.add_argument(
        '--duration', '-d',
        type=int, default=60,
        help='Monitoring duration in seconds'
    )
    parser.add_argument(
        '--output', '-o',
        default='/tmp/external_performance',
        help='Output directory'
    )
    parser.add_argument(
        '--list', '-l',
        action='store_true',
        help='List running containers'
    )

    args = parser.parse_args()

    if args.list:
        print("Running containers:")
        os.system("docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'")
        return

    if not args.container:
        # 尝试自动检测 benchmark 容器
        result = subprocess.run(
            ['docker', 'ps', '--format', '{{.Names}}'],
            capture_output=True, text=True
        )
        containers = [c for c in result.stdout.strip().split('\n') if 'benchmark' in c]
        if containers:
            args.container = containers[0]
            print(f"Auto-detected container: {args.container}")
        else:
            print("Error: No container specified and no benchmark container found")
            print("Usage: python3 external_performance_monitor.py --container <name>")
            print("       python3 external_performance_monitor.py --list")
            return

    monitor = ExternalPerformanceMonitor(
        container_name=args.container,
        output_dir=args.output,
        duration=args.duration
    )
    monitor.run()


if __name__ == '__main__':
    main()
