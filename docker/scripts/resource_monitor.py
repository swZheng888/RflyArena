#!/usr/bin/env python3
"""
资源监控器 - 在 Docker 容器内监控 CPU/内存使用
用于记录控制器实际资源消耗
"""

import os
import sys
import time
import argparse
import csv
import signal
from datetime import datetime
from typing import Optional

try:
    import psutil
except ImportError:
    print("Error: psutil not installed. Run: pip3 install psutil")
    sys.exit(1)


class ResourceMonitor:
    """资源监控器类"""

    def __init__(self, output_file: str, interval: float = 0.1):
        """
        初始化监控器

        Args:
            output_file: 输出 CSV 文件路径
            interval: 采样间隔(秒)
        """
        self.output_file = output_file
        self.interval = interval
        self.running = True
        self.start_time = time.time()

        # 获取资源限制
        self.cpu_limit = self._get_cpu_limit()
        self.mem_limit = self._get_memory_limit()

        # 初始化 CSV 写入
        self.csv_file: Optional[object] = None
        self.csv_writer: Optional[csv.writer] = None

        # 统计数据
        self.samples = 0
        self.cpu_sum = 0.0
        self.mem_sum = 0.0
        self.cpu_max = 0.0
        self.mem_max = 0.0

    def _get_cpu_limit(self) -> float:
        """获取 Cgroup CPU 限制"""
        try:
            # Cgroup v1
            quota_file = "/sys/fs/cgroup/cpu/cpu.cfs_quota_us"
            period_file = "/sys/fs/cgroup/cpu/cpu.cfs_period_us"

            if os.path.exists(quota_file):
                with open(quota_file, 'r') as f:
                    quota = int(f.read().strip())
                with open(period_file, 'r') as f:
                    period = int(f.read().strip())

                if quota > 0:
                    return quota / period

            # Cgroup v2
            max_file = "/sys/fs/cgroup/cpu.max"
            if os.path.exists(max_file):
                with open(max_file, 'r') as f:
                    content = f.read().strip().split()
                    if content[0] != 'max':
                        return int(content[0]) / int(content[1])

        except Exception as e:
            print(f"Warning: Could not read CPU limit: {e}")

        return float(psutil.cpu_count())  # 无限制

    def _get_memory_limit(self) -> int:
        """获取 Cgroup 内存限制 (bytes)"""
        try:
            # Cgroup v1
            limit_file = "/sys/fs/cgroup/memory/memory.limit_in_bytes"
            if os.path.exists(limit_file):
                with open(limit_file, 'r') as f:
                    limit = int(f.read().strip())
                    # 检查是否是无限制 (很大的数字)
                    if limit < 9223372036854771712:
                        return limit

            # Cgroup v2
            max_file = "/sys/fs/cgroup/memory.max"
            if os.path.exists(max_file):
                with open(max_file, 'r') as f:
                    content = f.read().strip()
                    if content != 'max':
                        return int(content)

        except Exception as e:
            print(f"Warning: Could not read memory limit: {e}")

        return psutil.virtual_memory().total  # 无限制

    def _init_csv(self):
        """初始化 CSV 文件"""
        self.csv_file = open(self.output_file, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # 写入头部
        self.csv_writer.writerow([
            'timestamp',
            'elapsed_s',
            'cpu_percent',
            'cpu_limit_percent',
            'mem_used_mb',
            'mem_limit_mb',
            'mem_percent',
            'num_threads',
            'num_processes',
        ])

    def _signal_handler(self, signum, frame):
        """信号处理器"""
        print("\nStopping monitor...")
        self.running = False

    def start(self):
        """开始监控"""
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._init_csv()

        cpu_limit_percent = self.cpu_limit * 100
        mem_limit_mb = self.mem_limit / (1024 * 1024)

        print(f"Resource Monitor Started")
        print(f"  CPU Limit: {self.cpu_limit:.2f} cores ({cpu_limit_percent:.0f}%)")
        print(f"  Memory Limit: {mem_limit_mb:.0f} MB")
        print(f"  Output: {self.output_file}")
        print(f"  Interval: {self.interval}s")
        print()

        # 初始化 CPU 计数器
        psutil.cpu_percent(interval=None)

        while self.running:
            try:
                elapsed = time.time() - self.start_time

                # 获取资源使用
                cpu_percent = psutil.cpu_percent(interval=None)
                mem_info = psutil.virtual_memory()
                mem_used_mb = mem_info.used / (1024 * 1024)
                mem_percent = (mem_info.used / self.mem_limit) * 100

                # 进程统计
                num_processes = len(psutil.pids())
                current_process = psutil.Process()
                num_threads = current_process.num_threads()

                # 更新统计
                self.samples += 1
                self.cpu_sum += cpu_percent
                self.mem_sum += mem_used_mb
                self.cpu_max = max(self.cpu_max, cpu_percent)
                self.mem_max = max(self.mem_max, mem_used_mb)

                # 写入 CSV
                self.csv_writer.writerow([
                    datetime.now().isoformat(),
                    f"{elapsed:.3f}",
                    f"{cpu_percent:.1f}",
                    f"{cpu_limit_percent:.1f}",
                    f"{mem_used_mb:.1f}",
                    f"{mem_limit_mb:.1f}",
                    f"{mem_percent:.1f}",
                    num_threads,
                    num_processes,
                ])

                # 实时输出
                if self.samples % 10 == 0:
                    print(f"\r[{elapsed:6.1f}s] CPU: {cpu_percent:5.1f}% | "
                          f"MEM: {mem_used_mb:6.1f}/{mem_limit_mb:.0f} MB ({mem_percent:4.1f}%)",
                          end='', flush=True)

                time.sleep(self.interval)

            except Exception as e:
                print(f"\nError: {e}")
                break

        self._finalize()

    def _finalize(self):
        """结束监控，输出统计"""
        if self.csv_file:
            self.csv_file.close()

        print("\n")
        print("=" * 50)
        print("Resource Usage Summary")
        print("=" * 50)

        if self.samples > 0:
            print(f"  Duration: {time.time() - self.start_time:.1f}s")
            print(f"  Samples: {self.samples}")
            print(f"  CPU Average: {self.cpu_sum / self.samples:.1f}%")
            print(f"  CPU Max: {self.cpu_max:.1f}%")
            print(f"  Memory Average: {self.mem_sum / self.samples:.1f} MB")
            print(f"  Memory Max: {self.mem_max:.1f} MB")
        print("=" * 50)
        print(f"Results saved to: {self.output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Monitor resource usage in Docker container'
    )
    parser.add_argument(
        '--output', '-o',
        default='/tmp/resource_usage.csv',
        help='Output CSV file path'
    )
    parser.add_argument(
        '--interval', '-i',
        type=float,
        default=0.1,
        help='Sampling interval in seconds'
    )

    args = parser.parse_args()

    monitor = ResourceMonitor(
        output_file=args.output,
        interval=args.interval
    )
    monitor.start()


if __name__ == '__main__':
    main()
