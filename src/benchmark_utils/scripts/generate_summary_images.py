#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import argparse
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math
from datetime import datetime

# 配置Matplotlib中文字体
try:
    plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'PingFang SC', 'WenQuanYi Micro Hei', 'Droid Sans Fallback', 'DejaVu Sans', 'sans-serif']
    plt.rcParams['axes.unicode_minus'] = False
except:
    pass

def find_latest_session(base_dir):
    """查找最新的benchmark session目录"""
    sessions = glob.glob(os.path.join(base_dir, 'session_*'))
    if not sessions:
        return None
    return max(sessions, key=os.path.getmtime)

def create_summary_grid(image_paths, output_path, title):
    """创建图片网格汇总图"""
    if not image_paths:
        print(f"没有找到图片用于: {title}")
        return

    n_images = len(image_paths)
    n_cols = 4  # 固定4列，适应更多子图
    n_rows = math.ceil(n_images / n_cols)
    
    # 动态调整图片大小
    fig = plt.figure(figsize=(n_cols * 5, n_rows * 4))
    plt.suptitle(title, fontsize=24, y=0.99)
    
    for i, img_path in enumerate(sorted(image_paths)):
        try:
            img = mpimg.imread(img_path)
            ax = fig.add_subplot(n_rows, n_cols, i + 1)
            ax.imshow(img)
            ax.axis('off')
            
            # 提取任务名作为子图标题
            # 尝试从目录名或文件名获取有意义的名称
            parent_dir = os.path.basename(os.path.dirname(img_path))
            filename = os.path.basename(img_path)
            
            # 简化标题
            if 'circle' in parent_dir or 'ellipse' in parent_dir or '1.0x' in parent_dir:
                label = parent_dir
            else:
                label = filename.replace('.png', '')
                
            ax.set_title(label, fontsize=10)
        except Exception as e:
            print(f"无法读取图片 {img_path}: {e}")

    plt.tight_layout(rect=[0, 0.02, 1, 0.97])
    # 保存
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"✅ 已生成汇总图: {output_path}")

def main():
    parser = argparse.ArgumentParser(description='生成Benchmark汇总图表')
    parser.add_argument('--session', help='指定Session目录 (默认最新)')
    parser.add_argument('--type', help='指定要合并的类型 (如 xy, 3d, err)，不指定则合并所有')
    args = parser.parse_args()

    # 确定基础目录
    base_dir = os.path.expanduser('~/trajectory_project/benchmark_results')
    
    # 确定Session目录
    if args.session:
        session_dir = args.session
    else:
        session_dir = find_latest_session(base_dir)
        
    if not session_dir or not os.path.exists(session_dir):
        print(f"错误: 找不到Session目录: {session_dir}")
        return

    print(f"📂 正在分析Session: {session_dir}")
    
    # 获取所有任务目录
    all_task_dirs = [d for d in glob.glob(os.path.join(session_dir, '*')) if os.path.isdir(d)]
    
    # 提取所有出现的速度等级
    speed_levels = set()
    for d in all_task_dirs:
        dirname = os.path.basename(d)
        if '_' in dirname and dirname.endswith('x'):
            parts = dirname.split('_')
            speed = parts[-1]  # e.g. "1.0x"
            speed_levels.add(speed)
            
    if not speed_levels:
        print("⚠️ 未能识别出带速度后缀的任务目录，尝试作为单组处理...")
        speed_levels.add('all')

    print(f"🔍 识别到速度等级: {sorted(list(speed_levels))}")

    # 定义要汇总的图片匹配模式
    patterns = {
        'xy': ('XY 平面投影汇总', ['_xy.png', 'xy_projection']),
        '3d': ('3D 轨迹汇总', ['_3d.png', 'trajectory_3d']),
        'err': ('误差分析汇总', ['_err.png', 'error_analysis']),
        'pos': ('位置跟踪汇总', ['_pos.png', 'position_tracking'])
    }
    
    # 如果用户指定了类型，只处理该类型
    if args.type:
        target_keys = [k for k in patterns.keys() if args.type in k]
    else:
        target_keys = patterns.keys()

    # 针对每个速度等级分别生成汇总
    for speed in sorted(list(speed_levels)):
        print(f"\n🚀 处理速度等级: {speed}")
        
        # 筛选当前速度的任务目录
        if speed == 'all':
            current_speed_dirs = all_task_dirs
        else:
            current_speed_dirs = [d for d in all_task_dirs if d.endswith(f'_{speed}')]
            
        if not current_speed_dirs:
            continue

        # 执行汇总
        for key in target_keys:
            base_title, keywords = patterns[key]
            title = f"{base_title} ({speed})"
            image_list = []
            
            for task_dir in current_speed_dirs:
                # 在每个任务目录中查找匹配的文件
                for kw in keywords:
                    found = glob.glob(os.path.join(task_dir, f"*{kw}*"))
                    if found:
                        image_list.append(found[0])
                        break 
            
            if image_list:
                print(f"  - 类型 '{key}': 找到 {len(image_list)} 张图片")
                output_filename = f"summary_{speed}_{key}.png"
                output_path = os.path.join(session_dir, output_filename)
                create_summary_grid(image_list, output_path, title)
            else:
                pass # 静默跳过无图类型

if __name__ == '__main__':
    main()
