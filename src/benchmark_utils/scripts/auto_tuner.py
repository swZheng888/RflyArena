#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
auto_tuner.py - 基于Optuna的无人机控制器自动调参脚本

功能:
1. 自动启动控制器(NMPC/PID)和Benchmark
2. 动态调整参数(不重启控制器)
3. 采集Benchmark结果(RMS误差)
4. 使用贝叶斯优化寻找最优参数
"""

import os
import sys
import time
import argparse
import subprocess
import signal
import re
import json
import select
import csv
from datetime import datetime

import optuna
from optuna.samplers import TPESampler
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# 添加当前目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

class AutoTuner:
    def __init__(self, controller_type, trajectory, speed, duration, n_trials, workspace, 
                 odom_topic='/vio/odometry_enu', coordinate_frame=1):
        self.controller_type = controller_type
        self.trajectory = trajectory
        self.speed = speed
        self.duration = duration
        self.n_trials = n_trials
        self.workspace = workspace
        self.odom_topic = odom_topic
        self.coordinate_frame = coordinate_frame
        
        self.results_dir = os.path.join(workspace, 'tuning_results')
        os.makedirs(self.results_dir, exist_ok=True)
        
        # Trial history for visualization
        self.trial_history = []
        self.best_value_history = []
        
        # 初始化ROS节点
        self._init_ros()
        
    def _init_ros(self):
        import rospy
        from std_msgs.msg import String
        
        if not rospy.core.is_initialized():
            rospy.init_node('auto_tuner', anonymous=True)
            
        # 参数发布话题
        if self.controller_type == 'nmpc':
            self.param_pub = rospy.Publisher('/nmpc_control/update_weights', String, queue_size=1)
        else:
            # Px4Ctrl PID参数更新话题
            self.param_pub = rospy.Publisher('/px4ctrl/update_gains', String, queue_size=1)
            
        time.sleep(1.0) # 等待连接
        
    def update_params(self, params):
        """发布新参数"""
        from std_msgs.msg import String
        msg = String()
        msg.data = json.dumps(params)
        
        # 多次发送确保接收
        for _ in range(3):
            self.param_pub.publish(msg)
            time.sleep(0.1)
        print(f"  [AutoTuner] 参数已更新: {params}")

    def run_benchmark(self, trial_number, trajectory_type, speed=None, max_retries=3):
        """
        运行Benchmark并等待结果
        speed: 如果提供则覆盖默认速度
        max_retries: 失败时最大重试次数
        """
        current_speed = speed if speed else self.speed
        
        for attempt in range(max_retries):
            if attempt > 0:
                print(f"  [Trial {trial_number}] 重试 {attempt + 1}/{max_retries}...")
                time.sleep(3)  # 等待3秒再重试
            
            # 检查 ROS Master 是否可用
            try:
                import subprocess
                result = subprocess.run(
                    ['rostopic', 'list'], 
                    capture_output=True, 
                    timeout=5,
                    env=self._get_ros_env()
                )
                if result.returncode != 0:
                    print(f"  [Trial {trial_number}] ROS Master 不可用，等待重试...")
                    time.sleep(5)
                    continue
            except Exception as e:
                print(f"  [Trial {trial_number}] ROS 检查失败: {e}")
                time.sleep(5)
                continue
            
            print(f"\n  [Trial {trial_number} - {trajectory_type} @ {current_speed}x] 启动Benchmark...")
            
            # 完整的ROS环境变量加载
            env = self._get_ros_env()
            
            # 构建命令
            if trajectory_type == 'hover':
                # 悬停任务
                cmd = f"""
                source {self.workspace}/devel/setup.bash && \
                rosrun benchmark_utils benchmark_node.py \
                    _task_type:=hover \
                    _duration:={self.duration} \
                    _odom_topic:={self.odom_topic} \
                    _coordinate_frame:={self.coordinate_frame} \
                    _skip_plot:=true
                """
            else:
                # 动态轨迹任务
                cmd = f"""
                source {self.workspace}/devel/setup.bash && \
                rosrun benchmark_utils benchmark_node.py \
                    _task_type:=dynamic \
                    _trajectory_type:={trajectory_type} \
                    _speed_level:={current_speed} \
                    _num_loops:=2 \
                    _odom_topic:={self.odom_topic} \
                    _coordinate_frame:={self.coordinate_frame} \
                    _skip_plot:=true
                """
            
            try:
                # 添加PYTHONUNBUFFERED确保实时输出
                env['PYTHONUNBUFFERED'] = '1'
                env['ROSCONSOLE_STDOUT_LINE_BUFFERED'] = '1'
                
                # 启动子进程 (使用setsid创建新进程组)
                process = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable='/bin/bash',
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                    universal_newlines=True,
                    bufsize=1  # 行缓冲
                )
                
                # 使用select进行非阻塞读取
                start_time = time.time()
                timeout = self.duration + 25 # 缩短缓冲时间加快调参
                out_of_control = False
                timed_out = False
                
                while True:
                    # 检查进程是否已退出
                    if process.poll() is not None:
                        break
                        
                    # 检查超时
                    if time.time() - start_time > timeout:
                        print("  [AutoTuner] Benchmark超时，强制终止...")
                        try:
                            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                            time.sleep(1)
                            if process.poll() is None:
                                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                        except:
                            pass
                        timed_out = True
                        break
                    
                    # 读取输出
                    reads = [process.stdout.fileno()]
                    ret = select.select(reads, [], [], 0.5) # 0.5s超时
                    
                    if ret[0]:
                        line = process.stdout.readline()
                        if line:
                            line_stripped = line.strip()
                            print(f"    [Bench] {line_stripped}")
                            
                            # 检测失控：位置误差过大 / 飞机坠毁
                            if any(kw in line_stripped for kw in [
                                'Failsafe', 'DISARMED', 'Land now',
                                'Emergency', 'crash', 'out of control'
                            ]):
                                print("  ⚠️  [AutoTuner] 检测到失控，跳过此参数组...")
                                out_of_control = True
                                try:
                                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                                    time.sleep(0.5)
                                    if process.poll() is None:
                                        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                                except:
                                    pass
                                break
                
                # 超时或失控 → 直接返回罚分，不重试
                if out_of_control or timed_out:
                    time.sleep(2)  # 等待飞机恢复
                    return float('inf')
                    
                print(f"  [AutoTuner] Benchmark进程已结束")
                
            except Exception as e:
                print(f"  [AutoTuner] 执行出错: {e}")
                return float('inf')  # 出错也不重试，直接跳过
            
            # 解析结果
            result = self._parse_latest_result()
            if result != float('inf'):
                return result
            else:
                print(f"  [Trial {trial_number}] 解析失败，跳过...")
                return float('inf')  # 解析失败也直接跳过
        
        # 所有重试都失败了
        print(f"  [Trial {trial_number}] 所有 {max_retries} 次重试均失败")
        return float('inf')

    def _get_ros_env(self):
        """获取完整的ROS环境变量"""
        # 通过执行 source setup.bash && env 获取
        cmd = f"source {self.workspace}/devel/setup.bash && env"
        result = subprocess.run(cmd, shell=True, executable='/bin/bash', capture_output=True, text=True)
        env = {}
        for line in result.stdout.split('\n'):
            if '=' in line:
                k, _, v = line.partition('=')
                env[k] = v
        return env

    def _parse_latest_result(self):
        """解析最新的Benchmark报告"""
        # 优先查找默认日志目录
        result_dir = os.path.expanduser('~/.benchmark_logs')
        if not os.path.exists(result_dir):
            # 回退到workspace下的目录
            result_dir = os.path.join(self.workspace, 'benchmark_results')
            
        if not os.path.exists(result_dir):
            return float('inf')
            
        # 查找最近修改的报告文件
        report_files = []
        for root, dirs, files in os.walk(result_dir):
            for f in files:
                if f.startswith("report_") and f.endswith(".txt"):
                    path = os.path.join(root, f)
                    # 仅考虑最近2分钟内生成的
                    if time.time() - os.path.getmtime(path) < 120:
                        report_files.append(path)
                        
        if not report_files:
            print("  [AutoTuner] 未找到有效的Benchmark报告")
            return float('inf')
            
        latest_report = max(report_files, key=os.path.getmtime)
        print(f"  [AutoTuner] 解析报告: {os.path.basename(latest_report)}")
        
        try:
            with open(latest_report, 'r') as f:
                content = f.read()
                # 匹配RMS位置误差 (根据报告实际内容: "RMS误差: 0.1386 m")
                # 尝试多种常用的RMS标签
                patterns = [
                    r'位置\s*RMS\s*误差[：:]\s*([\d.]+)',
                    r'RMS\s*误差[：:]\s*([\d.]+)',
                    r'RMS位置误差[：:]\s*([\d.]+)',
                    r'RMS\s*Error[：:]\s*([\d.]+)',
                    r'pos_rms_error[\":\s]+([\d.]+)',
                ]
                
                for pattern in patterns:
                    match = re.search(pattern, content)
                    if match:
                        error = float(match.group(1))
                        print(f"  [AutoTuner] RMS Error: {error:.4f} m")
                        return error
        except:
            pass
            
        print("  [AutoTuner] 解析RMS失败")
        return float('inf')

    def objective(self, trial):
        """Optuna目标函数"""
        params = {}
        
        if self.controller_type == 'nmpc':
            # NMPC参数搜索空间（扩大范围，更客观）
            params['Q_pos_xy'] = trial.suggest_float('Q_pos_xy', 1.0, 100.0)  # XY位置权重
            params['Q_pos_z'] = trial.suggest_float('Q_pos_z', 1.0, 100.0)    # Z位置权重
            params['Q_vel_xy'] = trial.suggest_float('Q_vel_xy', 0.01, 50.0)  # XY速度权重
            params['Q_vel_z'] = trial.suggest_float('Q_vel_z', 0.01, 50.0)    # Z速度权重
            params['Q_att'] = trial.suggest_float('Q_att', 0.1, 30.0)         # 姿态权重
        else:
            # SO(3) PID参数搜索空间（以已知稳定参数为中心）
            # 位置环增益 Kp (稳定值: 3.0, 3.0, 3.0)
            params['Kp0'] = trial.suggest_float('Kp0', 1.5, 8.0)   # X位置
            params['Kp1'] = trial.suggest_float('Kp1', 1.5, 8.0)   # Y位置
            params['Kp2'] = trial.suggest_float('Kp2', 1.5, 10.0)  # Z位置
            # 速度环增益 Kv (稳定值: 2.0, 2.0, 2.0)
            params['Kv0'] = trial.suggest_float('Kv0', 1.0, 6.0)   # X速度
            params['Kv1'] = trial.suggest_float('Kv1', 1.0, 6.0)   # Y速度
            params['Kv2'] = trial.suggest_float('Kv2', 1.0, 8.0)   # Z速度
            # 姿态环增益 KAng (稳定值: 12.0, 12.0, 4.0)
            params['KAngR'] = trial.suggest_float('KAngR', 5.0, 25.0)  # Roll
            params['KAngP'] = trial.suggest_float('KAngP', 5.0, 25.0)  # Pitch
            params['KAngY'] = trial.suggest_float('KAngY', 1.5, 10.0)  # Yaw


        # 更新参数
        self.update_params(params)
        
        # 多任务、多速度评估
        trajectories = self.trajectory.split(',')
        
        # 定义测试速度（如果用户指定了 --speed，也包含用户速度）
        if hasattr(self, 'test_speeds') and self.test_speeds:
            # 使用用户指定的速度列表
            speed_levels = self.test_speeds
        else:
            # 默认：测试多个速度以确保鲁棒性
            speed_levels = [1.0, 1.5] if self.speed == 1.0 else [self.speed]
        
        total_error = 0.0
        test_count = 0
        
        for traj_type in trajectories:
            traj_type = traj_type.strip()
            if not traj_type:
                continue
            
            # 每个轨迹测试多个速度
            for speed in speed_levels:
                # 临时修改速度
                original_speed = self.speed
                self.speed = speed
                
                print(f"  [Trial {trial.number}] {traj_type} @ {speed}x 速度")
                error = self.run_benchmark(trial.number, traj_type)
                
                # 恢复原始速度
                self.speed = original_speed
                
                # 如果任意一个任务失败，则整个Trial失败
                if error == float('inf'):
                    # 记录失败的试验（error = -1 表示失败）
                    failed_trial_data = {
                        'trial_number': trial.number,
                        'error': -1.0,  # 用-1表示失败
                        'params': params.copy()
                    }
                    self.trial_history.append(failed_trial_data)
                    # 立即保存失败记录
                    try:
                        temp_study_name = f"{self.controller_type}_tuning_live"
                        self.save_trial_history(temp_study_name)
                    except:
                        pass
                    return float('inf')
                    
                total_error += error
                test_count += 1
            
        # 返回平均误差
        avg_error = total_error / test_count
        print(f"  [Trial {trial.number}] 完成. 平均RMS Error: {avg_error:.4f} m ({test_count} 个测试)")
        
        # 记录试验历史
        trial_data = {
            'trial_number': trial.number,
            'error': avg_error,
            'params': params.copy()
        }
        self.trial_history.append(trial_data)
        
        # 更新最佳值历史（用于收敛曲线）
        if self.best_value_history:
            current_best = min(self.best_value_history[-1], avg_error)
        else:
            current_best = avg_error
        self.best_value_history.append(current_best)
        
        # 实时保存：每个trial完成后立即保存历史和图表
        try:
            # 使用临时名称，以便实时查看
            temp_study_name = f"{self.controller_type}_tuning_live"
            self.save_trial_history(temp_study_name)
            
            # 每5个trial或最后一个trial才更新图表（避免频繁绘图）
            if trial.number % 5 == 0 or trial.number == self.n_trials - 1:
                self.plot_convergence(temp_study_name)
                print(f"  💾 实时保存: CSV + 收敛图已更新 (Trial {trial.number + 1}/{self.n_trials})")
        except Exception as e:
            print(f"  ⚠️  实时保存警告: {e}")
        
        return avg_error
    
    def save_trial_history(self, study_name):
        """Save trial history to CSV"""
        csv_path = os.path.join(self.results_dir, f'{study_name}_history.csv')
        
        with open(csv_path, 'w', newline='') as f:
            if not self.trial_history:
                return csv_path
                
            # Write header
            fieldnames = ['trial_number', 'error'] + list(self.trial_history[0]['params'].keys())
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            
            # Write data
            for trial_data in self.trial_history:
                row = {
                    'trial_number': trial_data['trial_number'],
                    'error': trial_data['error']
                }
                row.update(trial_data['params'])
                writer.writerow(row)
        
        print(f"\n  Trial history saved: {csv_path}")
        return csv_path
    
    def plot_convergence(self, study_name):
        """Generate publication-quality convergence plot"""
        if not self.trial_history:
            return None
        
        # Setup publication style
        plt.rcParams.update({
            'font.family': 'serif',
            'font.serif': ['Times New Roman', 'DejaVu Serif'],
            'font.size': 9,
            'axes.labelsize': 9,
            'axes.titlesize': 10,
            'xtick.labelsize': 8,
            'ytick.labelsize': 8,
            'legend.fontsize': 8,
            'figure.dpi': 300,
            'savefig.dpi': 300,
            'savefig.bbox': 'tight',
            'savefig.pad_inches': 0.02,
        })
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.16, 5.5), 
                                       gridspec_kw={'hspace': 0.35})
        
        trial_numbers = [t['trial_number'] for t in self.trial_history]
        errors = [t['error'] for t in self.trial_history]
        
        # Bayesian optimization starts after n_startup_trials
        n_startup = 5  # Should match TPESampler n_startup_trials
        
        # === Top: All trials with phase distinction ===
        # Random phase
        if len(trial_numbers) > n_startup:
            ax1.plot(trial_numbers[:n_startup], errors[:n_startup], 'o-', 
                    color='#999999', linewidth=1.2, markersize=4, alpha=0.6,
                    label='Random Sampling')
            ax1.plot(trial_numbers[n_startup:], errors[n_startup:], 'o-', 
                    color='#1f77b4', linewidth=1.2, markersize=4, alpha=0.7,
                    label='Bayesian Optimization')
        else:
            ax1.plot(trial_numbers, errors, 'o-', color='#1f77b4', 
                    linewidth=1.2, markersize=4, alpha=0.7, label='Trial Error')
        
        ax1.plot(trial_numbers, self.best_value_history, '-', color='#d62728',
                linewidth=1.5, label='Best So Far', zorder=100)
        
        # Add vertical line to mark Bayesian start
        if len(trial_numbers) > n_startup:
            ax1.axvline(x=n_startup-0.5, color='orange', linestyle='--', 
                       linewidth=1.5, alpha=0.7, label=f'Bayesian Start (Trial {n_startup})')
        
        ax1.set_xlabel('Trial Number', fontsize=9)
        ax1.set_ylabel('RMS Error (m)', fontsize=9)
        ax1.set_title('Optimization Convergence', fontsize=10, fontweight='bold')
        ax1.legend(loc='best', fontsize=7, framealpha=0.9)
        ax1.grid(True, alpha=0.3, linestyle=':')
        
        # Add best value annotation
        best_idx = np.argmin(errors)
        best_trial = trial_numbers[best_idx]
        best_error = errors[best_idx]
        ax1.annotate(f'Best: {best_error:.4f} m\n(Trial {best_trial})',
                    xy=(best_trial, best_error), xytext=(10, -30),
                    textcoords='offset points', fontsize=7,
                    bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.7),
                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
        
        # === Bottom: Parameter evolution ===
        param_names = list(self.trial_history[0]['params'].keys())
        colors = plt.cm.tab10(np.linspace(0, 1, len(param_names)))
        
        for i, param_name in enumerate(param_names):
            param_values = [t['params'][param_name] for t in self.trial_history]
            ax2.plot(trial_numbers, param_values, 'o-', 
                    color=colors[i], linewidth=1.0, markersize=3,
                    alpha=0.7, label=param_name)
        
        # Add vertical line for Bayesian start
        if len(trial_numbers) > n_startup:
            ax2.axvline(x=n_startup-0.5, color='orange', linestyle='--', 
                       linewidth=1.5, alpha=0.5)
        
        ax2.set_xlabel('Trial Number', fontsize=9)
        ax2.set_ylabel('Parameter Value', fontsize=9)
        ax2.set_title('Parameter Evolution', fontsize=10, fontweight='bold')
        ax2.legend(loc='best', fontsize=7, framealpha=0.9, ncol=2)
        ax2.grid(True, alpha=0.3, linestyle=':')
        
        # Add subfigure labels
        fig.text(0.02, 0.97, '(a)', fontsize=9, fontweight='bold')
        fig.text(0.02, 0.48, '(b)', fontsize=9, fontweight='bold')
        
        # Save
        plot_path = os.path.join(self.results_dir, f'{study_name}_convergence.pdf')
        plt.savefig(plot_path, format='pdf', dpi=300, bbox_inches='tight')
        plt.savefig(plot_path.replace('.pdf', '.png'), format='png', dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"  Convergence plot saved: {plot_path}")
        return plot_path

def main():
    parser = argparse.ArgumentParser(
        description='Auto-tuning tool for UAV controllers using Bayesian optimization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Single trajectory (fast)
  python3 auto_tuner.py --controller nmpc --trajectory circle --trials 10
  
  # Multi-trajectory (robust, recommended)
  python3 auto_tuner.py --controller nmpc --trajectory "circle,figure8,ellipse" --trials 20
  
  # Comprehensive evaluation
  python3 auto_tuner.py --controller nmpc --trajectory "circle,figure8,ellipse,spiral_circle" --trials 30
        '''
    )
    parser.add_argument('--controller', choices=['nmpc', 'pid'], required=True,
                       help='Controller type to tune')
    parser.add_argument('--trials', type=int, default=20,
                       help='Number of optimization trials (default: 20)')
    parser.add_argument('--trajectory', default='circle,ellipse,long_ellipse,tilted_ellipse,figure8,spiral_circle,cone3d,sine_wave,zigzag,polynomial,square,star',
                       help='Trajectory types (comma-separated for multi-task). '
                            'Options: circle, ellipse, long_ellipse, tilted_ellipse, figure8, '
                            'spiral_circle, cone3d, sine_wave, zigzag, polynomial, square, star. '
                            'Default: all 12 types for comprehensive tuning')
    parser.add_argument('--speed', type=float, default=1.0,
                       help='Speed level (default: 1.0). Deprecated, use --speeds instead')
    parser.add_argument('--speeds', type=str, default=None,
                       help='Comma-separated speed levels for multi-speed evaluation. '
                            'Example: "1.0,1.5,2.0". If not specified, defaults to "1.0,1.5" for robust tuning')
    parser.add_argument('--duration', type=float, default=20.0,
                       help='Duration per trial in seconds (default: 20.0)')
    parser.add_argument('--odom_topic', default='/vio/odometry_enu',
                        help='Odometry topic')
    parser.add_argument('--coordinate_frame', type=int, default=1,
                        help='Coordinate frame: 0=NED, 1=ENU')
    args = parser.parse_args()
    
    workspace = '/root/trajectory_project'
    
    # Parse trajectory list
    trajectory_list = [t.strip() for t in args.trajectory.split(',') if t.strip()]
    
    # Parse speed list
    if args.speeds:
        speed_list = [float(s.strip()) for s in args.speeds.split(',') if s.strip()]
    else:
        # Default: test 1.0x and 1.5x for robustness
        # 如果只有speed参数，使用speed参数
        if args.speed != 1.0:
             speed_list = [args.speed]
        else:
             speed_list = [1.0] # 默认为1.0
    
    print("="*60)
    print(f"Auto-Tuner Started")
    print("="*60)
    print(f"  Controller:  {args.controller.upper()}")
    print(f"  Trials:      {args.trials}")
    print(f"  Trajectories: {len(trajectory_list)} types")
    for i, traj in enumerate(trajectory_list, 1):
        print(f"    {i}. {traj}")
    print(f"  Speed Levels: {len(speed_list)} levels")
    for speed in speed_list:
        print(f"    • {speed}x")
    print(f"  Tests/Trial: {len(trajectory_list) * len(speed_list)}")
    print(f"  Duration:    {args.duration}s per test")
    print(f"  Odom:        {args.odom_topic}")
    print(f"  Frame:       {'ENU' if args.coordinate_frame==1 else 'NED'}")
    print("="*60)
    print("⚠️  Ensure Controller is RUNNING (Offboard mode) BEFORE starting!")
    print("="*60)
    
    # 简单的倒计时，给用户反应时间
    for i in range(3, 0, -1):
        print(f"Starting in {i}...")
        time.sleep(1)

    tuner = AutoTuner(args.controller, args.trajectory, args.speed, args.duration, 
                      args.trials, workspace, args.odom_topic, args.coordinate_frame)
    tuner.test_speeds = speed_list  # 传递速度列表
    
    # 创建Study（使用内存数据库避免权限问题）
    study_name = f"{args.controller}_tune_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    study = optuna.create_study(
        study_name=study_name,
        direction='minimize',
        # 使用内存存储，避免 SQLite 文件权限问题
        storage=None,  # None = in-memory storage
        sampler=TPESampler(
            n_startup_trials=5,  # 前5个trial随机采样，之后使用贝叶斯优化
            seed=42,             # 固定种子便于复现
            multivariate=True,   # 考虑参数间的相关性
            warn_independent_sampling=False
        )
    )
    
    try:
        study.optimize(tuner.objective, n_trials=args.trials)
    except KeyboardInterrupt:
        print("\nTuning interrupted by user.")
        
    print("\n" + "="*60)
    print("Tuning Completed!")
    print(f"Best Error: {study.best_value:.4f} m")
    print("Best Params:", study.best_params)
    print("="*60)
    
    # Save trial history and generate plots
    try:
        csv_path = tuner.save_trial_history(study_name)
        plot_path = tuner.plot_convergence(study_name)
        
        print("\n📊 Outputs:")
        print(f"  ✓ Trial history CSV: {csv_path}")
        if plot_path:
            print(f"  ✓ Convergence plot: {plot_path}")
            print(f"  ✓ Convergence plot (PNG): {plot_path.replace('.pdf', '.png')}")
        print(f"  ✓ Optuna database: {tuner.results_dir}/optuna.db")
        
    except Exception as e:
        print(f"\n⚠️  Warning: Failed to save results: {e}")
    
    print("\nDone!")
    
    # 保存最佳参数
    with open(os.path.join(tuner.results_dir, f"{study_name}_best.json"), 'w') as f:
        json.dump(study.best_params, f, indent=4)

if __name__ == "__main__":
    main()
