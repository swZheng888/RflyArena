#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
generate_group_summary_radar.py - 目录级综合雷达图生成器

功能:
- 读取某个 benchmark 结果目录下所有任务的 metrics_*.json
- 聚合为一组目录级别的综合指标
- 生成一张综合雷达图和一份文本汇总报告

使用方法:
    python generate_group_summary_radar.py /path/to/benchmark_results/controller_timestamp
    python generate_group_summary_radar.py /path/to/benchmark_results/controller_timestamp --name my_controller_run
"""

import argparse
import json
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


plt.rcParams["font.family"] = "serif"
plt.rcParams["font.size"] = 12
plt.rcParams["axes.labelsize"] = 13
plt.rcParams["axes.titlesize"] = 14
plt.rcParams["savefig.dpi"] = 300
plt.rcParams["axes.unicode_minus"] = False


def soft_inverse_score(val, best, worst, floor=28.0, gamma=0.60):
    """误差越小越好的软归一化评分。"""
    if val <= best:
        return 100.0
    if val >= worst:
        return floor
    ratio = (val - best) / (worst - best)
    return floor + (100.0 - floor) * ((1.0 - ratio) ** gamma)


def soft_log_inverse_score(val, best, worst, floor=24.0, gamma=0.85):
    """对跨数量级指标更稳的对数软归一化评分。"""
    if val <= best:
        return 100.0
    val = min(max(val, best), worst)
    ratio = (np.log(val) - np.log(best)) / (np.log(worst) - np.log(best))
    return floor + (100.0 - floor) * ((1.0 - ratio) ** gamma)


def soft_energy_score(val, target=0.8, floor=35.0):
    """能效类正向指标的软饱和评分。"""
    if val <= 0:
        return floor
    ratio = min(val / target, 1.0)
    return floor + (100.0 - floor) * np.sqrt(ratio)


def is_feasible_trajectory(metrics):
    """用更宽松的工程标准判断轨迹是否可行。"""
    pos_rms = float(metrics.get("pos_rms_error", 0.0))
    pos_max = float(metrics.get("pos_max_error", 0.0))
    vel_rms = float(metrics.get("vel_rms_error", 0.0))
    overall = float(metrics.get("overall_score", 0.0))
    is_hover = bool(metrics.get("is_hover", False))

    if is_hover:
        return pos_rms <= 0.08 and pos_max <= 0.30

    if overall >= 55.0:
        return True

    return pos_rms <= 0.15 and pos_max <= 0.60 and vel_rms <= 1.20


def load_metrics(results_dir):
    """读取结果目录下各任务的 metrics JSON。"""
    metrics = []
    for json_path in sorted(Path(results_dir).glob("*/metrics_*.json")):
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        data["task_name"] = json_path.parent.name
        metrics.append(data)
    return metrics


def aggregate_metrics(all_metrics):
    """按目录级别聚合关键数值指标。"""
    agg = {}
    numeric_keys = [
        "overall_score",
        "pos_rms_error",
        "pos_max_error",
        "vel_rms_error",
        "actual_jerk_rms",
        "phase_delay",
        "energy_efficiency_index",
    ]

    for key in numeric_keys:
        vals = [float(m.get(key, 0.0)) for m in all_metrics]
        agg["%s_avg" % key] = float(np.mean(vals))
        agg["%s_min" % key] = float(np.min(vals))
        agg["%s_max" % key] = float(np.max(vals))

    agg["task_count"] = len(all_metrics)
    agg["grades"] = dict(Counter(m.get("grade", "?") for m in all_metrics))
    feasible_flags = [is_feasible_trajectory(m) for m in all_metrics]
    agg["feasible_count"] = int(sum(feasible_flags))
    agg["infeasible_count"] = int(len(feasible_flags) - agg["feasible_count"])
    agg["feasible_ratio"] = float(agg["feasible_count"] / len(feasible_flags)) if feasible_flags else 0.0
    return agg


def radar_values(agg):
    """从聚合指标生成雷达图五维显示值。"""
    return [
        soft_inverse_score(agg["pos_rms_error_avg"], 0.01, 0.25),
        soft_inverse_score(agg["vel_rms_error_avg"], 0.02, 0.50),
        soft_log_inverse_score(agg["actual_jerk_rms_avg"], 30.0, 5000.0),
        soft_inverse_score(abs(agg["phase_delay_avg"]), 0.005, 0.30),
        soft_energy_score(agg["energy_efficiency_index_avg"]),
        100.0 * agg["feasible_ratio"],
    ]


def generate_radar(agg, output_path, title):
    """生成目录级综合雷达图。"""
    categories = [
        "Track.\nAccuracy",
        "Velocity\nTracking",
        "Control\nSmooth.",
        "Response\nSpeed",
        "Energy\nEff.",
        "Feasible\nRatio",
    ]
    values = radar_values(agg)
    angles = np.linspace(0, 2 * np.pi, len(categories), endpoint=False).tolist()
    values_plot = values + values[:1]
    angles_plot = angles + angles[:1]

    fig, ax = plt.subplots(figsize=(6.6, 6.2), subplot_kw=dict(polar=True))
    ax.fill(angles_plot, values_plot, color="#5DADE2", alpha=0.30)
    ax.plot(angles_plot, values_plot, "o-", color="#1F77B4", linewidth=2.6, markersize=7)

    ax.set_xticks(angles)
    ax.set_xticklabels(categories, fontsize=11)
    ax.tick_params(axis="x", pad=10)
    ax.set_ylim(0, 100)
    ax.set_yticks([20, 40, 60, 80, 100])
    ax.set_yticklabels(["20", "40", "60", "80", "100"], fontsize=9)
    ax.set_rlabel_position(90)
    ax.grid(True, alpha=0.30)

    quality_score = float(np.mean(values[:-1]))
    adjusted_overall = 0.45 * quality_score + 0.55 * values[-1]
    ax.set_title(
        "%s\nAdjusted Group Score: %.1f/100" % (title, adjusted_overall),
        fontsize=15,
        fontweight="bold",
        pad=22,
    )

    fig.savefig(str(output_path), bbox_inches="tight")
    plt.close(fig)


def write_summary(agg, all_metrics, output_path, title):
    """生成目录级文本汇总报告。"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("=" * 80 + "\n")
        f.write("%s\n" % title)
        f.write("=" * 80 + "\n\n")
        f.write("Tasks included: %d\n" % agg["task_count"])
        f.write("Average raw overall score: %.2f/100\n" % agg["overall_score_avg"])
        f.write("Feasible trajectories: %d\n" % agg["feasible_count"])
        f.write("Infeasible trajectories: %d\n" % agg["infeasible_count"])
        f.write("Feasible ratio: %.1f%%\n" % (agg["feasible_ratio"] * 100.0))
        f.write("Average position RMS error: %.4f m\n" % agg["pos_rms_error_avg"])
        f.write("Average velocity RMS error: %.4f m/s\n" % agg["vel_rms_error_avg"])
        f.write("Average jerk RMS: %.4f m/s^3\n" % agg["actual_jerk_rms_avg"])
        f.write("Average phase delay: %.4f s\n" % agg["phase_delay_avg"])
        f.write("Average energy efficiency index: %.4f\n" % agg["energy_efficiency_index_avg"])
        f.write("Grade distribution: %s\n\n" % agg["grades"])
        f.write("Per-task overall scores:\n")
        for metric in sorted(all_metrics, key=lambda x: x.get("overall_score", 0.0), reverse=True):
            feasible_tag = "feasible" if is_feasible_trajectory(metric) else "infeasible"
            f.write(
                "  - %s: %.2f/100 (%s, %s), pos_rms=%.4f m\n"
                % (
                    metric["task_name"],
                    metric["overall_score"],
                    metric.get("grade", "?"),
                    feasible_tag,
                    metric.get("pos_rms_error", 0.0),
                )
            )


def main():
    parser = argparse.ArgumentParser(description="Generate a group-level summary radar for benchmark results")
    parser.add_argument("results_dir", help="Directory containing per-task subdirectories with metrics_*.json")
    parser.add_argument("--name", default="", help="Optional display name")
    args = parser.parse_args()

    results_dir = Path(args.results_dir).resolve()
    all_metrics = load_metrics(results_dir)
    if not all_metrics:
        raise SystemExit("No metrics JSON files found under %s" % results_dir)

    agg = aggregate_metrics(all_metrics)
    display_name = args.name or results_dir.name

    radar_png = results_dir / ("%s_group_summary_radar.png" % display_name)
    radar_pdf = results_dir / ("%s_group_summary_radar.pdf" % display_name)
    report_txt = results_dir / ("%s_group_summary_report.txt" % display_name)

    generate_radar(agg, radar_png, "%s Group Summary" % display_name)
    generate_radar(agg, radar_pdf, "%s Group Summary" % display_name)
    write_summary(agg, all_metrics, report_txt, "%s Group Summary" % display_name)

    print("Saved %s" % radar_png)
    print("Saved %s" % radar_pdf)
    print("Saved %s" % report_txt)


if __name__ == "__main__":
    main()
