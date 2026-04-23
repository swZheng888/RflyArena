#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
微分平坦特性验证测试脚本

验证轨迹生成器中的 pos, vel, acc, jerk 是否满足微分关系：
- vel ≈ d(pos)/dt
- acc ≈ d(vel)/dt
- jerk ≈ d(acc)/dt

使用数值微分进行验证
"""

import numpy as np
import sys
sys.path.insert(0, '/root/trajectory_project/src/benchmark_utils/scripts')

from trajectory_generator import DifferentialFlatTrajectory

def numerical_derivative(func, t, dt=1e-6):
    """数值计算导数"""
    val_plus = func(t + dt)
    val_minus = func(t - dt)
    return (val_plus - val_minus) / (2 * dt)

def verify_trajectory(traj_gen, traj_type, num_samples=20):
    """验证单个轨迹类型的微分平坦特性"""
    print(f"\n{'='*60}")
    print(f"验证轨迹: {traj_type}")
    print('='*60)
    
    period = traj_gen.get_trajectory_period(traj_type)
    dt = 1e-5
    
    errors = {
        'vel': [],
        'acc': [],
        'jerk': []
    }
    
    for i in range(num_samples):
        t = (i / num_samples) * period
        
        try:
            ret = traj_gen.get_trajectory(traj_type, t)
            if len(ret) == 6:
                pos, vel, acc, jerk, yaw, yaw_dot = ret
            else:
                pos, vel, acc, yaw, yaw_dot = ret
                jerk = np.zeros(3)
        except Exception as e:
            print(f"  [错误] t={t:.3f}: {e}")
            continue
        
        # 数值计算导数
        def get_pos(t_):
            r = traj_gen.get_trajectory(traj_type, t_)
            return r[0]
        
        def get_vel(t_):
            r = traj_gen.get_trajectory(traj_type, t_)
            return r[1]
        
        def get_acc(t_):
            r = traj_gen.get_trajectory(traj_type, t_)
            return r[2]
        
        # 计算数值导数
        vel_numerical = numerical_derivative(get_pos, t, dt)
        acc_numerical = numerical_derivative(get_vel, t, dt)
        jerk_numerical = numerical_derivative(get_acc, t, dt)
        
        # 计算误差
        vel_error = np.linalg.norm(vel - vel_numerical)
        acc_error = np.linalg.norm(acc - acc_numerical)
        jerk_error = np.linalg.norm(jerk - jerk_numerical) if np.linalg.norm(jerk) > 0.001 else 0.0
        
        errors['vel'].append(vel_error)
        errors['acc'].append(acc_error)
        errors['jerk'].append(jerk_error)
    
    # 计算统计
    for key, vals in errors.items():
        if vals:
            avg = np.mean(vals)
            max_val = np.max(vals)
            status = "✓ 通过" if max_val < 0.01 else "✗ 失败"
            print(f"  {key:4s}: 平均误差={avg:.6f}, 最大误差={max_val:.6f}  {status}")
        else:
            print(f"  {key:4s}: 无数据")
    
    # 总体判断
    all_pass = all(np.max(v) < 0.01 for v in errors.values() if v)
    return all_pass, errors

def main():
    print("\n" + "="*60)
    print("微分平坦特性验证测试")
    print("="*60)
    
    traj_gen = DifferentialFlatTrajectory()
    traj_gen.set_params(speed_factor=1.0, amplitude=2.0, base_speed=1.0)
    
    # 测试所有轨迹类型
    traj_types = [
        'circle',
        'figure8',
        'ellipse',
        'tilted_ellipse',
        'long_ellipse',
        'spiral_circle',
        'cone3d',
        'sine_wave',
        'pyramid',
        'zigzag',
        'square',
        'star',
    ]
    
    results = {}
    for traj_type in traj_types:
        try:
            passed, errors = verify_trajectory(traj_gen, traj_type)
            results[traj_type] = passed
        except Exception as e:
            print(f"\n  [错误] 验证 {traj_type} 时出错: {e}")
            results[traj_type] = False
    
    # 总结
    print("\n" + "="*60)
    print("验证总结")
    print("="*60)
    
    passed_count = sum(1 for v in results.values() if v)
    total_count = len(results)
    
    for traj_type, passed in results.items():
        status = "✓ 通过" if passed else "✗ 需检查"
        print(f"  {traj_type:20s}: {status}")
    
    print(f"\n通过率: {passed_count}/{total_count}")
    
    if passed_count == total_count:
        print("\n✓ 所有轨迹的微分平坦特性验证通过！")
    else:
        print("\n✗ 部分轨迹的微分平坦特性需要检查！")

if __name__ == '__main__':
    main()
