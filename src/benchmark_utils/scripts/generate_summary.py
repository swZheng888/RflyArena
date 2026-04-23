#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Benchmark Summary Generator - 汇总所有测试结果生成综合评估雷达图

功能:
- 读取所有子目录的性能指标
- 计算平均值和加权得分
- 生成一张综合雷达图

使用方法:
    python generate_summary.py --results_dir /path/to/benchmark_results/controller_timestamp
"""

import os
import sys
import argparse
import glob
import csv
import numpy as np
from datetime import datetime

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
    
    # Science paper style
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.size'] = 12
    plt.rcParams['axes.labelsize'] = 14
    plt.rcParams['savefig.dpi'] = 300
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not available")


def load_metrics_from_csv(csv_file):
    """从CSV文件加载数据并计算指标"""
    try:
        data = {
            'time': [], 'target_position': [], 'actual_position': [],
            'target_velocity': [], 'actual_velocity': []
        }
        
        with open(csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                data['time'].append(float(row['time']))
                data['target_position'].append([
                    float(row['target_x']), float(row['target_y']), float(row['target_z'])
                ])
                data['actual_position'].append([
                    float(row['actual_x']), float(row['actual_y']), float(row['actual_z'])
                ])
                data['target_velocity'].append([
                    float(row['target_vx']), float(row['target_vy']), float(row['target_vz'])
                ])
                data['actual_velocity'].append([
                    float(row['actual_vx']), float(row['actual_vy']), float(row['actual_vz'])
                ])
        
        # 转换为numpy数组
        for key in data:
            data[key] = np.array(data[key])
            
        # 计算指标
        pos_errors = np.linalg.norm(data['target_position'] - data['actual_position'], axis=1)
        vel_errors = np.linalg.norm(data['target_velocity'] - data['actual_velocity'], axis=1)
        
        return {
            'rms_position_error': float(np.sqrt(np.mean(pos_errors**2))),
            'max_position_error': float(np.max(pos_errors)),
            'mean_position_error': float(np.mean(pos_errors)),
            'rms_velocity_error': float(np.sqrt(np.mean(vel_errors**2))),
            'std_velocity_error': float(np.std(vel_errors)),
            'mean_velocity_error': float(np.mean(vel_errors)),
        }
    except Exception as e:
        print(f"Error loading {csv_file}: {e}")
        return None


def aggregate_metrics(all_metrics):
    """汇总所有任务的指标"""
    if not all_metrics:
        return None
        
    # 计算各指标的平均值和最差值
    keys = all_metrics[0].keys()
    summary = {}
    
    for key in keys:
        values = [m[key] for m in all_metrics if m and key in m]
        if values:
            summary[f'{key}_avg'] = np.mean(values)
            summary[f'{key}_max'] = np.max(values)
            summary[f'{key}_min'] = np.min(values)
            summary[key] = np.mean(values)  # 主要使用平均值
    
    return summary


def generate_summary_radar(summary, output_path, controller_name):
    """生成综合评估雷达图"""
    if not MATPLOTLIB_AVAILABLE:
        return
    
    # 3个核心位置跟踪误差指标
    categories = [
        'RMS\nError',
        'Mean\nError', 
        'Max\nError'
    ]
    
    # 获取位置跟踪误差指标
    rms_err = summary.get('rms_position_error', 0.1)
    mean_err = summary.get('mean_position_error', 0.1)
    max_err = summary.get('max_position_error_max', summary.get('max_position_error', 0.3))
    
    # 归一化显示 (误差越小，display值越高)
    # 使用更宽松的参考上限
    s1 = np.clip(100 * (1 - rms_err / 0.5), 0, 100)      # RMS Error
    s2 = np.clip(100 * (1 - mean_err / 0.5), 0, 100)     # Mean Error
    s3 = np.clip(100 * (1 - max_err / 1.5), 0, 100)      # Max Error
    
    values = [s1, s2, s3]
    
    # 绘制雷达图
    angles = np.linspace(0, 2*np.pi, len(categories), endpoint=False).tolist()
    values_plot = values + values[:1]
    angles += angles[:1]
    
    fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(polar=True))
    
    # 填充区域
    ax.fill(angles, values_plot, color='#3498db', alpha=0.25)
    ax.plot(angles, values_plot, 'o-', color='#2980b9', linewidth=2.5, markersize=10)
    
    # 设置类别标签
    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(categories, fontsize=14)
    
    # 设置刻度范围
    ax.set_ylim(0, 100)
    ax.set_yticks([20, 40, 60, 80, 100])
    ax.set_yticklabels(['20', '40', '60', '80', '100'], fontsize=11)
    
    # 添加参考圈
    for val in [20, 40, 60, 80]:
        ax.plot(angles, [val]*len(angles), '--', color='gray', alpha=0.3, linewidth=0.5)
    
    # 计算总分
    avg_score = np.mean(values)
    
    # 标题
    ax.set_title(f'{controller_name.upper()} Controller\nOverall Performance: {avg_score:.1f}/100', 
                fontsize=18, fontweight='bold', pad=30)
    
    # 添加分数标注
    for angle, val, score_val in zip(angles[:-1], values_plot[:-1], values):
        ax.annotate(f'{score_val:.0f}', xy=(angle, val), 
                   xytext=(angle, val + 10), ha='center', fontsize=12, fontweight='bold',
                   color='#2c3e50')
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Summary radar chart saved: {output_path}")
    return avg_score


def generate_text_summary(summary, all_task_names, output_path, controller_name, overall_score):
    """生成文本汇总报告"""
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write("=" * 80 + "\n")
        f.write(f"BENCHMARK SUMMARY REPORT - {controller_name.upper()}\n")
        f.write("=" * 80 + "\n\n")
        
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Total Tasks Evaluated: {len(all_task_names)}\n")
        f.write(f"Overall Performance Score: {overall_score:.1f}/100\n\n")
        
        f.write("=" * 80 + "\n")
        f.write("AGGREGATED METRICS\n")
        f.write("=" * 80 + "\n\n")
        
        f.write("Position Tracking:\n")
        f.write(f"  RMS Error (avg): {summary.get('rms_position_error', 0):.4f} m\n")
        f.write(f"  RMS Error (best): {summary.get('rms_position_error_min', 0):.4f} m\n")
        f.write(f"  RMS Error (worst): {summary.get('rms_position_error_max', 0):.4f} m\n")
        f.write(f"  Max Error (worst): {summary.get('max_position_error_max', 0):.4f} m\n\n")
        
        f.write("Velocity Tracking:\n")
        f.write(f"  RMS Error (avg): {summary.get('rms_velocity_error', 0):.4f} m/s\n")
        f.write(f"  Std (avg): {summary.get('std_velocity_error', 0):.4f} m/s\n\n")
        
        f.write("=" * 80 + "\n")
        f.write("TASKS EVALUATED\n")
        f.write("=" * 80 + "\n\n")
        
        for name in all_task_names:
            f.write(f"  - {name}\n")
        
        f.write("\n" + "=" * 80 + "\n")
    
    print(f"Summary report saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Generate benchmark summary')
    parser.add_argument('--results_dir', required=True, help='Path to benchmark results directory')
    args = parser.parse_args()
    
    results_dir = args.results_dir
    
    if not os.path.isdir(results_dir):
        print(f"Error: {results_dir} is not a valid directory")
        sys.exit(1)
    
    # 获取控制器名称
    controller_name = os.path.basename(results_dir).split('_')[0]
    
    # 查找所有CSV文件
    csv_files = glob.glob(os.path.join(results_dir, '*', '*.csv'))
    
    if not csv_files:
        print(f"No CSV files found in {results_dir}")
        sys.exit(1)
    
    print(f"Found {len(csv_files)} CSV files")
    
    # 加载所有指标 (排除悬停任务)
    all_metrics = []
    all_task_names = []
    
    for csv_file in csv_files:
        task_name = os.path.basename(os.path.dirname(csv_file))
        
        # 排除悬停任务
        if 'hover' in task_name.lower():
            print(f"  Skipped (hover): {task_name}")
            continue
        
        # 排除不可行轨迹
        if 'infeasible' in task_name.lower() or any(x in task_name.lower() for x in ['zigzag', 'star', 'square']):
            print(f"  Skipped (infeasible): {task_name}")
            continue
            
        metrics = load_metrics_from_csv(csv_file)
        if metrics:
            all_metrics.append(metrics)
            all_task_names.append(task_name)
            print(f"  Loaded: {task_name} (RMS={metrics['rms_position_error']:.4f}m)")
    
    # 汇总指标
    summary = aggregate_metrics(all_metrics)
    
    if not summary:
        print("Failed to aggregate metrics")
        sys.exit(1)
    
    # 生成综合雷达图
    radar_path = os.path.join(results_dir, f"{controller_name}_summary_radar.png")
    overall_score = generate_summary_radar(summary, radar_path, controller_name)
    
    # 生成文本汇总
    report_path = os.path.join(results_dir, f"{controller_name}_summary_report.txt")
    generate_text_summary(summary, all_task_names, report_path, controller_name, overall_score)
    
    print(f"\n{'='*60}")
    print(f"SUMMARY COMPLETE - Overall Score: {overall_score:.1f}/100")
    print(f"{'='*60}")


if __name__ == '__main__':
    main()
