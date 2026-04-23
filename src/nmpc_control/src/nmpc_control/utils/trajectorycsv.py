# -*- coding: utf-8 -*-
"""
trajectory_dynamics_constrained.py

基于动力学约束的轨迹生成器：
1. 最大速度约束 (v_max)
2. 最大加速度约束 (a_max) 
3. 最大加加速度约束 (jerk_max) - S型曲线
4. 多圈高速巡航
"""

import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ==============================================================================
# 1. 动力学约束参数
# ==============================================================================

class DynamicsConstraints:
    """无人机动力学约束"""
    def __init__(self):
        # ============ 可调参数 ============
        self.v_max = 3.0         # 最大速度 m/s
        self.a_max = 4.0         # 最大加速度 m/s^2 (典型无人机: 2-6 m/s^2)
        self.j_max = 10.0        # 最大jerk m/s^3 (加速度变化率)
        self.yaw_rate_max = 2.0  # 最大偏航角速度 rad/s (~115 deg/s)
        # ==================================
        
    def get_accel_time(self, v_target):
        """计算从0加速到v_target所需的最短时间（考虑jerk约束）"""
        # S型曲线加速
        # 阶段1: jerk = +j_max (加速度从0增加到a_max)
        # 阶段2: jerk = 0 (恒定加速度a_max)  
        # 阶段3: jerk = -j_max (加速度从a_max减少到0)
        
        t_jerk = self.a_max / self.j_max  # jerk阶段时间
        v_jerk = 0.5 * self.j_max * t_jerk**2  # 单个jerk阶段增加的速度
        
        if v_target <= 2 * v_jerk:
            # 不需要恒定加速度阶段，只有jerk阶段
            t_total = 2 * math.sqrt(v_target / self.j_max)
        else:
            # 需要恒定加速度阶段
            v_const_accel = v_target - 2 * v_jerk
            t_const_accel = v_const_accel / self.a_max
            t_total = 2 * t_jerk + t_const_accel
            
        return t_total
    
    def get_accel_distance(self, v_target):
        """计算加速过程的距离"""
        t_jerk = self.a_max / self.j_max
        v_jerk = 0.5 * self.j_max * t_jerk**2
        d_jerk = (1/6) * self.j_max * t_jerk**3
        
        if v_target <= 2 * v_jerk:
            t_half = math.sqrt(v_target / self.j_max)
            d_total = 2 * (1/6) * self.j_max * t_half**3 + v_jerk * t_half
            return d_total
        else:
            v_const_accel = v_target - 2 * v_jerk
            t_const_accel = v_const_accel / self.a_max
            d_const_accel = v_jerk * t_const_accel + 0.5 * self.a_max * t_const_accel**2
            d_total = 2 * d_jerk + v_jerk * t_jerk + d_const_accel + (v_target - v_jerk) * t_jerk
            return d_total
        
    def print_info(self):
        print(f"\n{'='*50}")
        print("Dynamics Constraints:")
        print(f"  - Max velocity:     {self.v_max:.1f} m/s")
        print(f"  - Max acceleration: {self.a_max:.1f} m/s²")
        print(f"  - Max jerk:         {self.j_max:.1f} m/s³")
        print(f"  - Max yaw rate:     {np.degrees(self.yaw_rate_max):.1f} deg/s")
        print(f"\nDerived parameters:")
        print(f"  - Time to reach v_max:     {self.get_accel_time(self.v_max):.2f} s")
        print(f"  - Distance to reach v_max: {self.get_accel_distance(self.v_max):.2f} m")
        print(f"{'='*50}\n")

# ==============================================================================
# 2. S型速度曲线生成器
# ==============================================================================

class SCurveProfile:
    """S型速度曲线 (满足jerk约束的时间最优加速)"""
    
    def __init__(self, constraints: DynamicsConstraints):
        self.c = constraints
        
    def generate_profile(self, v_start, v_end, dt=0.01):
        """
        生成从v_start到v_end的S型速度曲线
        返回: times, velocities, accelerations, jerks
        """
        dv = v_end - v_start
        sign = 1 if dv >= 0 else -1
        dv = abs(dv)
        
        a_max = self.c.a_max
        j_max = self.c.j_max
        
        t_jerk = a_max / j_max  # 达到最大加速度的时间
        v_jerk = 0.5 * j_max * t_jerk**2  # jerk阶段速度变化量
        
        if dv <= 2 * v_jerk:
            # 三角形加速度曲线（达不到最大加速度）
            t_jerk_actual = math.sqrt(dv / j_max)
            t1, t2, t3 = t_jerk_actual, 0, t_jerk_actual
        else:
            # 梯形加速度曲线
            t1 = t_jerk
            t2 = (dv - 2 * v_jerk) / a_max
            t3 = t_jerk
            
        t_total = t1 + t2 + t3
        times = np.arange(0, t_total + dt, dt)
        
        velocities = []
        accelerations = []
        jerks = []
        
        for t in times:
            if t <= t1:
                # 阶段1: jerk = +j_max
                j = j_max * sign
                a = j_max * t * sign
                v = v_start + 0.5 * j_max * t**2 * sign
            elif t <= t1 + t2:
                # 阶段2: jerk = 0, a = a_max
                t_phase = t - t1
                j = 0
                a = a_max * sign
                v = v_start + v_jerk * sign + a_max * t_phase * sign
            else:
                # 阶段3: jerk = -j_max
                t_phase = t - t1 - t2
                j = -j_max * sign
                a = (a_max - j_max * t_phase) * sign
                v_at_t2 = v_start + (v_jerk + a_max * t2) * sign
                v = v_at_t2 + (a_max * t_phase - 0.5 * j_max * t_phase**2) * sign
                
            velocities.append(v)
            accelerations.append(a)
            jerks.append(j)
            
        return times, np.array(velocities), np.array(accelerations), np.array(jerks)

# ==============================================================================
# 3. 五次多项式求解器
# ==============================================================================

class MinJerkOptimizer:
    @staticmethod
    def solve_trajectory(waypoints, velocities, accelerations, avg_speed):
        N = len(waypoints)
        segments = []
        total_time = 0.0

        for i in range(N - 1):
            dist = np.linalg.norm(waypoints[i+1] - waypoints[i])
            v_avg = (np.linalg.norm(velocities[i]) + np.linalg.norm(velocities[i+1])) / 2
            if v_avg < 0.1:
                v_avg = avg_speed * 0.3
            T = max(dist / v_avg, 0.05)

            p0, v0, a0 = waypoints[i], velocities[i], accelerations[i]
            pf, vf, af = waypoints[i+1], velocities[i+1], accelerations[i+1]

            c0, c1, c2 = p0, v0, 0.5 * a0

            A = np.array([
                [T**3,   T**4,    T**5],
                [3*T**2, 4*T**3,  5*T**4],
                [6*T,    12*T**2, 20*T**3]
            ])
            
            coeffs = np.zeros((3, 6))
            for axis in range(3):
                delta_p = pf[axis] - (c0[axis] + c1[axis]*T + c2[axis]*T**2)
                delta_v = vf[axis] - (c1[axis] + 2*c2[axis]*T)
                delta_a = af[axis] - (2*c2[axis])
                
                try:
                    x = np.linalg.solve(A, [delta_p, delta_v, delta_a])
                    coeffs[axis] = [c0[axis], c1[axis], c2[axis], x[0], x[1], x[2]]
                except np.linalg.LinAlgError:
                    coeffs[axis] = [c0[axis], c1[axis], c2[axis], 0, 0, 0]

            segments.append({'T': T, 'coeffs': coeffs})
            total_time += T

        return segments, total_time

    @staticmethod
    def evaluate(t_query, segments):
        t_elapsed = 0.0
        active_seg = None
        
        for seg in segments:
            if t_query <= t_elapsed + seg['T']:
                active_seg = seg
                break
            t_elapsed += seg['T']
        
        if active_seg is None:
            active_seg = segments[-1]
            dt = active_seg['T']
        else:
            dt = t_query - t_elapsed

        coeffs = active_seg['coeffs']
        t_vec = np.array([1, dt, dt**2, dt**3, dt**4, dt**5])
        v_vec = np.array([0, 1, 2*dt, 3*dt**2, 4*dt**3, 5*dt**4])
        a_vec = np.array([0, 0, 2, 6*dt, 12*dt**2, 20*dt**3])

        return coeffs @ t_vec, coeffs @ v_vec, coeffs @ a_vec

# ==============================================================================
# 4. 工具函数
# ==============================================================================

def compute_yaw_from_velocity(vx, vy, last_yaw, threshold=0.05):
    if math.hypot(vx, vy) > threshold:
        return math.atan2(vy, vx)
    return last_yaw

def compute_yaw_rate(vx, vy, ax, ay, threshold=0.05):
    speed_sq = vx**2 + vy**2
    if speed_sq > threshold**2:
        return (vx * ay - vy * ax) / speed_sq
    return 0.0

def unwrap_yaw(yaw_array):
    result = np.zeros_like(yaw_array)
    result[0] = yaw_array[0]
    for i in range(1, len(yaw_array)):
        diff = yaw_array[i] - yaw_array[i-1]
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        result[i] = result[i-1] + diff
    return result

def yaw_to_quaternion(yaw):
    return math.cos(yaw*0.5), 0.0, 0.0, math.sin(yaw*0.5)

def generate_figure8_points(radius, height, n_points):
    """生成8字形航点和切线方向"""
    waypoints, tangents = [], []
    for i in range(n_points):
        theta = 2 * math.pi * i / n_points
        x = radius * math.sin(theta)
        y = radius * math.sin(theta) * math.cos(theta)
        waypoints.append([x, y, height])
        
        dx = radius * math.cos(theta)
        dy = radius * (math.cos(theta)**2 - math.sin(theta)**2)
        norm = math.hypot(dx, dy)
        tangents.append([dx/norm, dy/norm, 0] if norm > 1e-6 else [1,0,0])
    
    return np.array(waypoints), np.array(tangents)

# ==============================================================================
# 5. 主生成函数
# ==============================================================================

def generate_csv_file(filename="trajectory_generated.csv"):
    
    # ==================== 轨迹参数 ====================
    NUM_LAPS = 5              # 巡航圈数
    POINTS_PER_LAP = 32       # 每圈采样点
    HEIGHT = 1.5              # 飞行高度 m
    RADIUS = 3.0              # 8字形半径 m
    FREQ = 100                # 输出频率 Hz
    # ==================================================
    
    # 创建动力学约束
    constraints = DynamicsConstraints()
    constraints.print_info()
    
    v_cruise = constraints.v_max
    
    # 生成单圈8字形
    single_lap_wp, single_lap_tan = generate_figure8_points(RADIUS, HEIGHT, POINTS_PER_LAP)
    
    # --- 计算加速/减速段需要的航点数 ---
    accel_dist = constraints.get_accel_distance(v_cruise)
    
    # 估算8字形周长
    lap_circumference = 0
    for i in range(POINTS_PER_LAP):
        next_i = (i + 1) % POINTS_PER_LAP
        lap_circumference += np.linalg.norm(single_lap_wp[next_i] - single_lap_wp[i])
    
    dist_per_point = lap_circumference / POINTS_PER_LAP
    accel_points = max(3, int(np.ceil(accel_dist / dist_per_point)))
    
    print(f"Trajectory Planning:")
    print(f"  - Lap circumference: {lap_circumference:.2f} m")
    print(f"  - Accel distance: {accel_dist:.2f} m ({accel_points} points)")
    
    # --- 生成S型速度曲线 ---
    scurve = SCurveProfile(constraints)
    accel_times, accel_vels, accel_accs, _ = scurve.generate_profile(0, v_cruise, dt=0.02)
    decel_times, decel_vels, decel_accs, _ = scurve.generate_profile(v_cruise, 0, dt=0.02)
    
    accel_duration = accel_times[-1]
    decel_duration = decel_times[-1]
    
    print(f"  - Accel time: {accel_duration:.2f} s")
    print(f"  - Decel time: {decel_duration:.2f} s")
    
    # --- 构建完整轨迹 ---
    waypoints = []
    velocities = []
    accelerations = []
    
    # 阶段1: 加速段 (使用S型曲线)
    for i in range(accel_points + 1):
        idx = i % POINTS_PER_LAP
        waypoints.append(single_lap_wp[idx])
        
        # 速度沿切线方向，按S型曲线比例
        progress = i / accel_points
        vel_idx = min(int(progress * len(accel_vels)), len(accel_vels)-1)
        speed = accel_vels[vel_idx]
        acc = accel_accs[vel_idx]
        
        velocities.append(single_lap_tan[idx] * speed)
        accelerations.append(single_lap_tan[idx] * acc)
    
    # 阶段2: 巡航段 (恒定最大速度)
    start_idx = accel_points + 1
    cruise_points = NUM_LAPS * POINTS_PER_LAP
    
    for i in range(cruise_points):
        idx = (start_idx + i) % POINTS_PER_LAP
        waypoints.append(single_lap_wp[idx])
        velocities.append(single_lap_tan[idx] * v_cruise)
        
        # 巡航时向心加速度 (对于8字形轨迹的曲率)
        accelerations.append([0.0, 0.0, 0.0])  # 简化处理
    
    # 阶段3: 减速段 (使用S型曲线)
    decel_start_idx = (start_idx + cruise_points) % POINTS_PER_LAP
    decel_points = accel_points
    
    for i in range(1, decel_points + 1):
        idx = (decel_start_idx + i) % POINTS_PER_LAP
        waypoints.append(single_lap_wp[idx])
        
        progress = i / decel_points
        vel_idx = min(int(progress * len(decel_vels)), len(decel_vels)-1)
        speed = decel_vels[vel_idx]
        acc = decel_accs[vel_idx]
        
        velocities.append(single_lap_tan[idx] * speed)
        accelerations.append(single_lap_tan[idx] * acc)
    
    # 确保终点静止
    velocities[-1] = np.array([0.0, 0.0, 0.0])
    accelerations[-1] = np.array([0.0, 0.0, 0.0])
    
    waypoints = np.array(waypoints)
    velocities = np.array(velocities)
    accelerations = np.array(accelerations)
    
    print(f"  - Total waypoints: {len(waypoints)}")
    print(f"  - Accel: {accel_points}, Cruise: {cruise_points}, Decel: {decel_points}")

    # --- 多项式拟合 ---
    segments, total_dur = MinJerkOptimizer.solve_trajectory(
        waypoints, velocities, accelerations, v_cruise
    )
    print(f"  - Total duration: {total_dur:.2f} s")

    # --- 采样 ---
    time_steps = np.arange(0, total_dur, 1.0/FREQ)
    all_data = []
    last_yaw = 0.0
    
    for t in time_steps:
        pos, vel, acc = MinJerkOptimizer.evaluate(t, segments)
        
        yaw = compute_yaw_from_velocity(vel[0], vel[1], last_yaw)
        last_yaw = yaw
        yaw_rate = compute_yaw_rate(vel[0], vel[1], acc[0], acc[1])
        
        # 限制yaw_rate
        yaw_rate = np.clip(yaw_rate, -constraints.yaw_rate_max, constraints.yaw_rate_max)
        
        qw, qx, qy, qz = yaw_to_quaternion(yaw)
        
        all_data.append({
            't': t, 'x': pos[0], 'y': pos[1], 'z': pos[2],
            'vx': vel[0], 'vy': vel[1], 'vz': vel[2],
            'ax': acc[0], 'ay': acc[1], 'az': acc[2],
            'yaw': yaw, 'yaw_rate': yaw_rate,
            'qw': qw, 'qx': qx, 'qy': qy, 'qz': qz
        })
    
    # 平滑Yaw
    yaw_arr = np.array([d['yaw'] for d in all_data])
    yaw_unwrapped = unwrap_yaw(yaw_arr)
    for i, d in enumerate(all_data):
        d['yaw'] = yaw_unwrapped[i]
    
    # --- 写入CSV ---
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        header = ["t", "x", "y", "z", "vx", "vy", "vz", 
                  "ax", "ay", "az", "yaw", "yaw_rate", 
                  "qw", "qx", "qy", "qz"]
        writer.writerow(header)
        
        for d in all_data:
            row = [f"{d['t']:.4f}",
                   f"{d['x']:.6f}", f"{d['y']:.6f}", f"{d['z']:.6f}",
                   f"{d['vx']:.6f}", f"{d['vy']:.6f}", f"{d['vz']:.6f}",
                   f"{d['ax']:.6f}", f"{d['ay']:.6f}", f"{d['az']:.6f}",
                   f"{d['yaw']:.6f}", f"{d['yaw_rate']:.6f}",
                   f"{d['qw']:.6f}", f"{d['qx']:.6f}", f"{d['qy']:.6f}", f"{d['qz']:.6f}"]
            writer.writerow(row)
            
    print(f"\n✅ Saved: {os.path.abspath(filename)}")
    return filename, all_data, constraints

# ==============================================================================
# 6. 主程序
# ==============================================================================

if __name__ == "__main__":
    csv_file, all_data, constraints = generate_csv_file("trajectory_generated.csv")
    
    # 提取数据
    times = np.array([d['t'] for d in all_data])
    positions = np.array([[d['x'], d['y'], d['z']] for d in all_data])
    velocities = np.array([[d['vx'], d['vy'], d['vz']] for d in all_data])
    accelerations = np.array([[d['ax'], d['ay'], d['az']] for d in all_data])
    yaws = np.array([d['yaw'] for d in all_data])
    yaw_rates = np.array([d['yaw_rate'] for d in all_data])
    
    speed = np.linalg.norm(velocities, axis=1)
    acc_mag = np.linalg.norm(accelerations, axis=1)

    # --- 可视化 ---
    fig = plt.figure(figsize=(18, 12))
    
    # 3D轨迹
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(positions[:,0], positions[:,1], positions[:,2], 'b-', lw=0.8)
    ax1.scatter(*positions[0], c='g', s=100, marker='o', label='Start')
    ax1.scatter(*positions[-1], c='r', s=100, marker='x', label='End')
    ax1.set_title("3D Trajectory")
    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.set_zlabel("Z (m)")
    ax1.legend()
    
    # XY俯视图
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(positions[:,0], positions[:,1], 'b-', lw=0.8)
    step = len(times)//25
    for i in range(0, len(times), step):
        if speed[i] > 0.1:
            ax2.arrow(positions[i,0], positions[i,1], 
                      0.2*np.cos(yaws[i]), 0.2*np.sin(yaws[i]),
                      head_width=0.1, fc='red', ec='red', alpha=0.6)
    ax2.set_title("Top View + Yaw")
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)")
    ax2.axis('equal'); ax2.grid(True)
    
    # 速度曲线 + 约束线
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(times, speed, 'b-', lw=1.5, label='Speed')
    ax3.axhline(y=constraints.v_max, color='r', ls='--', lw=2, label=f'v_max={constraints.v_max} m/s')
    ax3.fill_between(times, 0, speed, alpha=0.3)
    ax3.set_title("Speed Profile (S-Curve Accel)")
    ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Speed (m/s)")
    ax3.legend(); ax3.grid(True)
    
    # 标注加速时间
    accel_time = constraints.get_accel_time(constraints.v_max)
    ax3.axvline(x=accel_time, color='g', ls=':', lw=1.5, alpha=0.7)
    ax3.text(accel_time+0.1, constraints.v_max*0.5, f'Accel\n{accel_time:.2f}s', fontsize=9, color='g')
    
    # 加速度曲线 + 约束线
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(times, accelerations[:,0], label='ax', alpha=0.8)
    ax4.plot(times, accelerations[:,1], label='ay', alpha=0.8)
    ax4.plot(times, acc_mag, 'k-', lw=1.5, label='|a|')
    ax4.axhline(y=constraints.a_max, color='r', ls='--', lw=2, label=f'a_max={constraints.a_max} m/s²')
    ax4.axhline(y=-constraints.a_max, color='r', ls='--', lw=2)
    ax4.set_title("Acceleration Profile")
    ax4.set_xlabel("Time (s)"); ax4.set_ylabel("Accel (m/s²)")
    ax4.legend(); ax4.grid(True)
    
    # 检查约束是否满足
    max_acc_actual = np.max(acc_mag)
    if max_acc_actual > constraints.a_max * 1.1:
        ax4.text(0.5, 0.9, f'⚠️ Max |a| = {max_acc_actual:.2f} (exceeds limit!)', 
                 transform=ax4.transAxes, color='red', fontsize=10,
                 bbox=dict(facecolor='yellow', alpha=0.8))
    else:
        ax4.text(0.5, 0.9, f'✅ Max |a| = {max_acc_actual:.2f} m/s²', 
                 transform=ax4.transAxes, color='green', fontsize=10,
                 bbox=dict(facecolor='lightgreen', alpha=0.8))
    
    # Yaw角
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(times, np.degrees(yaws), 'purple', lw=1.5)
    ax5.set_title("Yaw Angle")
    ax5.set_xlabel("Time (s)"); ax5.set_ylabel("Yaw (deg)")
    ax5.grid(True)
    
    # Yaw角速度 + 约束线
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.plot(times, np.degrees(yaw_rates), 'brown', lw=1.5)
    ax6.axhline(y=np.degrees(constraints.yaw_rate_max), color='r', ls='--', lw=2, 
                label=f'limit={np.degrees(constraints.yaw_rate_max):.0f}°/s')
    ax6.axhline(y=-np.degrees(constraints.yaw_rate_max), color='r', ls='--', lw=2)
    ax6.set_title("Yaw Rate")
    ax6.set_xlabel("Time (s)"); ax6.set_ylabel("Yaw Rate (deg/s)")
    ax6.legend(); ax6.grid(True)
    
    plt.tight_layout()
    plt.savefig("trajectory_analysis.png", dpi=150)
    plt.show()
    
    # 打印统计
    print("\n" + "="*60)
    print("Trajectory Statistics:")
    print(f"  - Duration: {times[-1]:.2f} s")
    print(f"  - Max speed: {np.max(speed):.2f} m/s (limit: {constraints.v_max} m/s)")
    print(f"  - Max accel: {max_acc_actual:.2f} m/s² (limit: {constraints.a_max} m/s²)")
    print(f"  - Max yaw rate: {np.degrees(np.max(np.abs(yaw_rates))):.1f} °/s (limit: {np.degrees(constraints.yaw_rate_max):.1f} °/s)")
    print(f"  - Accel time: {accel_time:.2f} s")
    print("="*60)