#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
benchmark_bag_analyzer.py

Rosbag离线分析工具 - Task 3专用

功能:
- 从rosbag读取/position_cmd（目标）和/odom（实际）数据
- 时间对齐和插值
- 调用benchmark_analyzer生成分析报告
- 边界检测（3×3×2m空间限制）

使用示例:
    rosrun benchmark_utils benchmark_bag_analyzer.py \\
        --bag flight.bag \\
        --output ~/results/ \\
        --name aerobatic \\
        --check-bounds --max-x 1.5 --max-y 1.5 --max-z 1.8
"""

import sys
import os
import argparse
import csv
import numpy as np
from datetime import datetime

try:
    import rosbag
except ImportError:
    print("错误: 需要安装rosbag。请运行: sudo apt-get install python-rosbag")
    sys.exit(1)

# 导入分析器
try:
    from benchmark_analyzer import BenchmarkAnalyzer
except ImportError:
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "benchmark_analyzer",
        os.path.join(os.path.dirname(__file__), "benchmark_analyzer.py")
    )
    benchmark_analyzer = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(benchmark_analyzer)
    BenchmarkAnalyzer = benchmark_analyzer.BenchmarkAnalyzer


class BenchmarkBagAnalyzer:
    """从Rosbag提取数据并分析"""
    
    def __init__(self, bag_path, target_topic='/position_cmd', odom_topic='/mavros/local_position/odom'):
        self.bag_path = bag_path
        self.target_topic = target_topic
        self.odom_topic = odom_topic
        
    def extract_data(self):
        """
        从bag提取数据
        
        Returns:
            tuple: (target_data, odom_data)
        """
        print(f"正在读取bag文件: {self.bag_path}")
        
        target_data = []
        odom_data = []
        
        try:
            bag = rosbag.Bag(self.bag_path, 'r')
            
            # 统计消息数量
            target_count = bag.get_message_count(self.target_topic)
            odom_count = bag.get_message_count(self.odom_topic)
            
            print(f"找到 {target_count} 条目标轨迹消息")
            print(f"找到 {odom_count} 条里程计消息")
            
            # 提取目标轨迹
            for topic, msg, t in bag.read_messages(topics=[self.target_topic]):
                target_data.append({
                    'time': t.to_sec(),
                    'position': [msg.position.x, msg.position.y, msg.position.z],
                    'velocity': [msg.velocity.x, msg.velocity.y, msg.velocity.z]
                })
                
            # 提取里程计
            for topic, msg, t in bag.read_messages(topics=[self.odom_topic]):
                odom_data.append({
                    'time': t.to_sec(),
                    'position': [
                        msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z
                    ],
                    'velocity': [
                        msg.twist.twist.linear.x,
                        msg.twist.twist.linear.y,
                        msg.twist.twist.linear.z
                    ]
                })
                
            bag.close()
            
        except Exception as e:
            print(f"读取bag文件失败: {e}")
            return None, None
            
        print(f"成功提取数据: {len(target_data)} 目标点, {len(odom_data)} odom点\n")
        return target_data, odom_data
        
    def align_and_interpolate(self, target_data, odom_data):
        """
        时间对齐和插值
        
        为每个odom时间戳插值对应的target
        """
        print("正在对齐时间戳...")
        
        # 转换为numpy数组
        target_times = np.array([d['time'] for d in target_data])
        target_pos = np.array([d['position'] for d in target_data])
        target_vel = np.array([d['velocity'] for d in target_data])
        
        odom_times = np.array([d['time'] for d in odom_data])
        odom_pos = np.array([d['position'] for d in odom_data])
        odom_vel = np.array([d['velocity'] for d in odom_data])
        
        # 找到时间重叠区间
        start_time = max(target_times[0], odom_times[0])
        end_time = min(target_times[-1], odom_times[-1])
        
        # 筛选重叠区间的odom数据
        valid_idx = (odom_times >= start_time) & (odom_times <= end_time)
        odom_times = odom_times[valid_idx]
        odom_pos = odom_pos[valid_idx]
        odom_vel = odom_vel[valid_idx]
        
        # 对每个odom时间戳插值target
        aligned_target_pos = np.zeros_like(odom_pos)
        aligned_target_vel = np.zeros_like(odom_vel)
        
        for i in range(3):  # x, y, z
            aligned_target_pos[:, i] = np.interp(odom_times, target_times, target_pos[:, i])
            aligned_target_vel[:, i] = np.interp(odom_times, target_times, target_vel[:, i])
            
        # 时间归一化（从0开始）
        odom_times_norm = odom_times - odom_times[0]
        
        print(f"对齐完成: {len(odom_times)} 个数据点\n")
        
        return odom_times_norm, aligned_target_pos, aligned_target_vel, odom_pos, odom_vel
        
    def check_bounds(self, positions, max_x, max_y, max_z):
        """检查空间边界违规"""
        violations = []
        
        for i, pos in enumerate(positions):
            if abs(pos[0]) > max_x or abs(pos[1]) > max_y or pos[2] > max_z or pos[2] < 0:
                violations.append((i, pos))
                
        if violations:
            print(f"警告: 检测到 {len(violations)} 个边界违规点")
            print(f"违规率: {len(violations)/len(positions)*100:.1f}%")
            print(f"边界限制: X=±{max_x}m, Y=±{max_y}m, Z=0-{max_z}m\n")
        else:
            print(f"边界检查: 通过 (X=±{max_x}m, Y=±{max_y}m, Z=0-{max_z}m)\n")
            
        return violations
        
    def save_to_csv(self, output_path, times, target_pos, target_vel, actual_pos, actual_vel):
        """保存为CSV格式（与benchmark_logger相同格式）"""
        print(f"正在保存CSV: {output_path}")
        
        with open(output_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time',
                'target_x', 'target_y', 'target_z',
                'actual_x', 'actual_y', 'actual_z',
                'target_vx', 'target_vy', 'target_vz',
                'actual_vx', 'actual_vy', 'actual_vz'
            ])
            
            for i in range(len(times)):
                writer.writerow([
                    times[i],
                    target_pos[i, 0], target_pos[i, 1], target_pos[i, 2],
                    actual_pos[i, 0], actual_pos[i, 1], actual_pos[i, 2],
                    target_vel[i, 0], target_vel[i, 1], target_vel[i, 2],
                    actual_vel[i, 0], actual_vel[i, 1], actual_vel[i, 2]
                ])
                
        print(f"CSV已保存\n")
        
    def analyze_bag(self, output_dir, task_name, check_bounds=False, 
                   max_x=1.5, max_y=1.5, max_z=1.8):
        """完整分析流程"""
        # 1. 提取数据
        target_data, odom_data = self.extract_data()
        if target_data is None or odom_data is None:
            return False
            
        # 2. 对齐数据
        times, target_pos, target_vel, actual_pos, actual_vel = \
            self.align_and_interpolate(target_data, odom_data)
            
        # 3. 边界检查
        if check_bounds:
            self.check_bounds(actual_pos, max_x, max_y, max_z)
            
        # 4. 保存CSV
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(output_dir, f"{task_name}_{timestamp}.csv")
        self.save_to_csv(csv_path, times, target_pos, target_vel, actual_pos, actual_vel)
        
        # 5. 调用分析器
        print("正在生成分析报告...")
        analyzer = BenchmarkAnalyzer()
        analyzer.analyze(csv_path, task_name=task_name, output_dir=output_dir)
        
        return True


def main():
    parser = argparse.ArgumentParser(description='Rosbag离线分析工具 (Task 3)')
    parser.add_argument('--bag', required=True, help='Rosbag文件路径')
    parser.add_argument('--output', default='~/aerobatic_results',
                       help='输出目录')
    parser.add_argument('--name', default='aerobatic',
                       help='任务名称')
    parser.add_argument('--target-topic', default='/position_cmd',
                       help='目标轨迹话题')
    parser.add_argument('--odom-topic', default='/mavros/local_position/odom',
                       help='里程计话题')
    parser.add_argument('--check-bounds', action='store_true',
                       help='检查空间边界违规')
    parser.add_argument('--max-x', type=float, default=1.5,
                       help='X轴最大值(m)')
    parser.add_argument('--max-y', type=float, default=1.5,
                       help='Y轴最大值(m)')
    parser.add_argument('--max-z', type=float, default=1.8,
                       help='Z轴最大值(m)')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.bag):
        print(f"错误: bag文件不存在 - {args.bag}")
        return 1
        
    output_dir = os.path.expanduser(args.output)
    
    print("=" * 70)
    print("Rosbag离线分析工具")
    print("=" * 70 + "\n")
    
    analyzer = BenchmarkBagAnalyzer(args.bag, args.target_topic, args.odom_topic)
    success = analyzer.analyze_bag(
        output_dir, args.name,
        check_bounds=args.check_bounds,
        max_x=args.max_x, max_y=args.max_y, max_z=args.max_z
    )
    
    if success:
        print("\n" + "=" * 70)
        print("分析完成！结果保存在: " + output_dir)
        print("=" * 70)
        return 0
    else:
        print("\n分析失败！")
        return 1


if __name__ == '__main__':
    sys.exit(main())
