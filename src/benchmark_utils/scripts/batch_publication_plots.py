#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
batch_publication_plots.py - 批量生成出版级图表

用法:
    python3 batch_publication_plots.py --input-dir ~/.benchmark_logs --output-dir ./publication_plots
    python3 batch_publication_plots.py --input-dir ./benchmark_results --output-dir ./publication_plots --pattern "*.csv"
"""

import os
import sys
import glob
import argparse
from datetime import datetime

# 添加脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from benchmark_analyzer import BenchmarkAnalyzer
from publication_plotter import PublicationPlotter


def extract_info_from_filename(filename):
    """从文件名提取信息"""
    basename = os.path.basename(filename).replace('.csv', '')
    parts = basename.split('_')

    # 尝试解析: task_YYYYMMDD_HHMMSS 格式
    if len(parts) >= 3:
        task_type = parts[0]  # dynamic / hover
        return {
            'task_type': task_type,
            'name': basename
        }

    return {'task_type': 'unknown', 'name': basename}


def process_single_file(csv_file, output_base_dir, analyzer, verbose=True):
    """处理单个CSV文件"""
    info = extract_info_from_filename(csv_file)

    # 创建输出子目录
    output_dir = os.path.join(output_base_dir, info['name'])
    os.makedirs(output_dir, exist_ok=True)

    if verbose:
        print(f"\n{'='*60}")
        print(f"处理: {os.path.basename(csv_file)}")
        print(f"输出: {output_dir}")
        print(f"{'='*60}")

    try:
        # 加载数据
        data = analyzer.load_csv(csv_file)
        if data is None:
            print(f"  [错误] 无法加载数据")
            return None

        # 计算指标
        metrics = analyzer.calculate_metrics(data)

        if verbose:
            print(f"  采样点: {metrics.sample_count}")
            print(f"  时长: {metrics.duration:.1f}s")
            print(f"  位置RMS误差: {metrics.pos_rms_error:.4f}m")
            print(f"  综合评分: {metrics.overall_score:.1f}/100 ({metrics.grade})")

        # 生成出版级图表
        plotter = PublicationPlotter(output_dir)
        plotter.generate_all_figures(data, metrics, '')

        # 生成文本报告
        analyzer.generate_report(info['name'], metrics, csv_file, output_dir)

        return metrics

    except Exception as e:
        print(f"  [错误] 处理失败: {e}")
        import traceback
        traceback.print_exc()
        return None


def generate_summary_table(results, output_dir):
    """生成汇总表格"""
    if not results:
        return

    # Markdown 表格
    md_file = os.path.join(output_dir, 'summary.md')
    with open(md_file, 'w', encoding='utf-8') as f:
        f.write("# Benchmark Results Summary\n\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

        f.write("| File | Duration | Pos RMS | Pos Max | Vel RMS | Delay | Score | Grade |\n")
        f.write("|------|----------|---------|---------|---------|-------|-------|-------|\n")

        for name, m in results.items():
            f.write(f"| {name} | {m.duration:.1f}s | {m.pos_rms_error:.4f}m | ")
            f.write(f"{m.pos_max_error:.4f}m | {m.vel_rms_error:.4f}m/s | ")
            f.write(f"{m.phase_delay:.3f}s | {m.overall_score:.1f} | {m.grade} |\n")

    print(f"\n汇总表格: {md_file}")

    # LaTeX 表格
    tex_file = os.path.join(output_dir, 'summary_table.tex')
    with open(tex_file, 'w', encoding='utf-8') as f:
        f.write(r"""\begin{table}[htbp]
\centering
\caption{Benchmark Results Summary}
\label{tab:benchmark_summary}
\begin{tabular}{lccccccc}
\hline
\textbf{Test} & \textbf{Duration} & \textbf{Pos RMS} & \textbf{Pos Max} & \textbf{Vel RMS} & \textbf{Delay} & \textbf{Score} & \textbf{Grade} \\
 & (s) & (m) & (m) & (m/s) & (s) & (/100) & \\
\hline
""")
        for name, m in results.items():
            short_name = name[:20] + "..." if len(name) > 20 else name
            f.write(f"{short_name} & {m.duration:.1f} & {m.pos_rms_error:.4f} & ")
            f.write(f"{m.pos_max_error:.4f} & {m.vel_rms_error:.4f} & ")
            f.write(f"{m.phase_delay:.3f} & {m.overall_score:.1f} & {m.grade} \\\\\n")

        f.write(r"""\hline
\end{tabular}
\end{table}
""")

    print(f"LaTeX表格: {tex_file}")


def main():
    parser = argparse.ArgumentParser(description='批量生成出版级图表')
    parser.add_argument('--input-dir', '-i', default='~/.benchmark_logs',
                       help='输入目录（包含CSV文件）')
    parser.add_argument('--output-dir', '-o', default='./publication_plots',
                       help='输出目录')
    parser.add_argument('--pattern', '-p', default='*.csv',
                       help='文件匹配模式')
    parser.add_argument('--limit', '-n', type=int, default=0,
                       help='最多处理N个文件（0=不限制）')
    parser.add_argument('--quiet', '-q', action='store_true',
                       help='减少输出')

    args = parser.parse_args()

    # 展开路径
    input_dir = os.path.expanduser(args.input_dir)
    output_dir = os.path.expanduser(args.output_dir)

    # 查找CSV文件
    pattern = os.path.join(input_dir, args.pattern)
    csv_files = sorted(glob.glob(pattern))

    if not csv_files:
        print(f"未找到匹配的文件: {pattern}")
        return 1

    print(f"\n{'='*60}")
    print(f"批量生成出版级图表")
    print(f"{'='*60}")
    print(f"输入目录: {input_dir}")
    print(f"输出目录: {output_dir}")
    print(f"找到文件: {len(csv_files)} 个")

    if args.limit > 0:
        csv_files = csv_files[:args.limit]
        print(f"处理限制: {args.limit} 个")

    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)

    # 初始化分析器
    analyzer = BenchmarkAnalyzer()

    # 处理每个文件
    results = {}
    success = 0
    failed = 0

    for i, csv_file in enumerate(csv_files):
        print(f"\n[{i+1}/{len(csv_files)}] {os.path.basename(csv_file)}")

        metrics = process_single_file(csv_file, output_dir, analyzer,
                                      verbose=not args.quiet)

        if metrics:
            name = os.path.basename(csv_file).replace('.csv', '')
            results[name] = metrics
            success += 1
        else:
            failed += 1

    # 生成汇总
    print(f"\n{'='*60}")
    print(f"处理完成")
    print(f"{'='*60}")
    print(f"成功: {success}")
    print(f"失败: {failed}")

    if results:
        generate_summary_table(results, output_dir)

    print(f"\n所有结果保存在: {output_dir}")

    return 0


if __name__ == '__main__':
    sys.exit(main())
