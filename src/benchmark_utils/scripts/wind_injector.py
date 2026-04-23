#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wind_injector.py - RflySim风扰动注入模块

功能:
1. 通过RflySim API注入风扰动
2. 支持多种风扰动类型: 恒定风、阵风、湍流风

RflySim故障ID:
    123458: 恒定风 (Constant Wind) - 参数: X/Y/Z轴风速
    123459: 阵风 (Gust Wind) - 参数: 阵风强度 + 到达时间
    123540: 湍流风 (Turbulent Wind) - 参数: 湍流强度
    123541: 切向风 (Tangential Wind) - 参数: 切向风强度

使用方式:
    rosrun benchmark_utils wind_injector.py _wind_type:=constant _wind_speed:=3.0
"""

import rospy
import numpy as np
from threading import Thread
import time
import sys

# 尝试导入RflySim API
try:
    sys.path.append('/root/RflySimAPIs/PythonAPI')
    import PX4MavCtrlV4 as PX4MavCtrl
    RFLYSIM_AVAILABLE = True
except ImportError:
    RFLYSIM_AVAILABLE = False
    rospy.logwarn("PX4MavCtrlV4 not found, wind injection will be simulated only")


# 故障ID映射
FAULT_IDS = {
    'constant': 123458,    # 恒定风
    'gust': 123459,        # 阵风
    'turbulent': 123540,   # 湍流风
    'tangential': 123541,  # 切向风
}


class WindInjector:
    """RflySim风扰动注入器"""
    
    def __init__(self):
        # 飞机配置
        self.copter_id = rospy.get_param('~copter_id', 1)
        
        # 风扰动配置
        self.wind_type = rospy.get_param('~wind_type', 'constant')  # constant/gust/turbulent
        self.wind_speed = rospy.get_param('~wind_speed', 3.0)  # m/s
        self.wind_direction = rospy.get_param('~wind_direction', 0.0)  # 度 (0=北, 90=东)
        
        # 阵风配置
        self.gust_intensity = rospy.get_param('~gust_intensity', 5.0)  # m/s
        self.gust_arrival_time = rospy.get_param('~gust_arrival_time', 2.0)  # 秒
        
        # 湍流配置
        self.turbulent_intensity = rospy.get_param('~turbulent_intensity', 3.0)
        
        # 渐增风 (ramping) 配置 - 用于测试极限
        self.ramp_start_speed = rospy.get_param('~ramp_start_speed', 0.0)  # 起始风速
        self.ramp_max_speed = rospy.get_param('~ramp_max_speed', 5.0)     # 最大风速
        self.ramp_rate = rospy.get_param('~ramp_rate', 0.5)                # 每步增加量 m/s
        self.ramp_step_interval = rospy.get_param('~ramp_step_interval', 2.0)  # 每步间隔秒
        
        # 开启/关闭
        self.enabled = rospy.get_param('~enabled', True)
        self.start_delay = rospy.get_param('~start_delay', 5.0)  # 延迟开始
        
        # RflySim API
        self.mav = None
        if RFLYSIM_AVAILABLE:
            try:
                self.mav = PX4MavCtrl.PX4MavCtrler(self.copter_id)
                rospy.loginfo(f"RflySim API initialized for copter {self.copter_id}")
            except Exception as e:
                rospy.logerr(f"Failed to initialize RflySim API: {e}")
                self.mav = None
        
        # 状态
        self.running = True
        self.current_fault_id = 0  # 记录当前激活的故障ID
        
        rospy.loginfo("="*60)
        rospy.loginfo("风扰动注入器已初始化")
        rospy.loginfo(f"  类型: {self.wind_type}")
        rospy.loginfo(f"  风速: {self.wind_speed} m/s")
        rospy.loginfo(f"  方向: {self.wind_direction}°")
        rospy.loginfo(f"  飞机ID: {self.copter_id}")
        rospy.loginfo(f"  启动延迟: {self.start_delay}s")
        rospy.loginfo(f"  RflySim API: {'可用' if self.mav else '不可用'}")
        rospy.loginfo("="*60)
        
    def start(self):
        """启动风扰动注入"""
        if not self.enabled:
            rospy.loginfo("风扰动已禁用")
            return
            
        # 延迟启动
        rospy.loginfo(f"风扰动将在 {self.start_delay} 秒后开始...")
        rospy.sleep(self.start_delay)
        
        # 注入风扰动
        if self.wind_type == 'ramping':
            self._inject_ramping_wind()
        else:
            self._inject_wind()
        
        # 保持运行直到关闭
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and self.running:
            rate.sleep()
    
    def _inject_ramping_wind(self):
        """渐增风模式 - 风力从0逐渐增大测试极限"""
        if not self.mav:
            rospy.logerr("RflySim API 不可用，无法注入风扰动")
            return
            
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("开始渐增风测试 (Ramping Wind Test)")
        rospy.loginfo(f"  起始风速: {self.ramp_start_speed} m/s")
        rospy.loginfo(f"  最大风速: {self.ramp_max_speed} m/s")
        rospy.loginfo(f"  增加率: {self.ramp_rate} m/s/step")
        rospy.loginfo(f"  每步间隔: {self.ramp_step_interval} s")
        rospy.loginfo("="*60 + "\n")
        
        current_speed = self.ramp_start_speed
        dir_rad = np.radians(self.wind_direction)
        
        rate = rospy.Rate(1.0 / self.ramp_step_interval)
        
        while not rospy.is_shutdown() and self.running and current_speed <= self.ramp_max_speed:
            # 计算风分量
            wind_n = current_speed * np.cos(dir_rad)
            wind_e = current_speed * np.sin(dir_rad)
            
            # 发送风扰动
            silInt = np.zeros(8).astype(int).tolist()
            silFloat = np.zeros(20).astype(float).tolist()
            silInt[0] = FAULT_IDS['constant']
            silFloat[0] = wind_n
            silFloat[1] = wind_e
            silFloat[2] = 0.0
            
            try:
                self.mav.sendSILIntFloat(silInt, silFloat)
                rospy.loginfo(f"[RAMPING] 当前风速: {current_speed:.1f} m/s (方向: {self.wind_direction}°)")
            except Exception as e:
                rospy.logerr(f"发送风扰动失败: {e}")
                break
            
            current_speed += self.ramp_rate
            rate.sleep()
        
        rospy.loginfo(f"\n渐增风测试完成! 最终风速达到: {min(current_speed, self.ramp_max_speed):.1f} m/s")
            
    def _inject_wind(self):
        """注入风扰动"""
        if not self.mav:
            rospy.logerr("RflySim API 不可用，无法注入风扰动")
            return
            
        # 准备参数数组
        silInt = np.zeros(8).astype(int).tolist()
        silFloat = np.zeros(20).astype(float).tolist()
        
        if self.wind_type == 'constant':
            # 恒定风: 123458
            # 参数: X/Y/Z轴风速 (NED坐标系)
            dir_rad = np.radians(self.wind_direction)
            wind_n = self.wind_speed * np.cos(dir_rad)  # 北向
            wind_e = self.wind_speed * np.sin(dir_rad)  # 东向
            wind_d = 0.0  # 向下
            
            silInt[0] = FAULT_IDS['constant']
            silFloat[0] = wind_n
            silFloat[1] = wind_e
            silFloat[2] = wind_d
            
            rospy.loginfo(f">>> 恒定风已注入: [{wind_n:.2f}, {wind_e:.2f}, {wind_d:.2f}] m/s <<<")
            
        elif self.wind_type == 'gust':
            # 阵风: 123459
            # 参数: 阵风强度 + 到达时间
            silInt[0] = FAULT_IDS['gust']
            silFloat[0] = self.gust_intensity
            silFloat[1] = self.gust_arrival_time
            
            rospy.loginfo(f">>> 阵风已注入: 强度={self.gust_intensity} m/s, 到达时间={self.gust_arrival_time}s <<<")
            
        elif self.wind_type == 'turbulent':
            # 湍流风: 123540
            # 参数: 湍流强度
            silInt[0] = FAULT_IDS['turbulent']
            silFloat[0] = self.turbulent_intensity
            
            rospy.loginfo(f">>> 湍流风已注入: 强度={self.turbulent_intensity} <<<")
            
        else:
            rospy.logerr(f"未知的风扰动类型: {self.wind_type}")
            return
            
        # 发送到RflySim
        try:
            self.mav.sendSILIntFloat(silInt, silFloat)
            self.current_fault_id = silInt[0]  # 记录当前故障ID
            rospy.loginfo(f"风扰动命令已发送到RflySim (故障ID={self.current_fault_id})")
        except Exception as e:
            rospy.logerr(f"发送风扰动命令失败: {e}")
            
    def stop(self):
        """停止风扰动"""
        self.running = False
        
        # 清除风 - 使用恒定风ID + 风速=0
        if self.mav:
            try:
                silInt = np.zeros(8).astype(int).tolist()
                silFloat = np.zeros(20).astype(float).tolist()
                silInt[0] = 123458  # constant wind ID
                silFloat[0] = 0.0   # 北向 = 0
                silFloat[1] = 0.0   # 东向 = 0  
                silFloat[2] = 0.0   # 向下 = 0
                self.mav.sendSILIntFloat(silInt, silFloat)
                rospy.loginfo("风扰动已清除 (恒定风风速=0)")
            except Exception as e:
                rospy.logerr(f"清除失败: {e}")
        
        rospy.loginfo("风扰动注入器已停止")


def main():
    rospy.init_node('wind_injector', anonymous=True)
    
    injector = WindInjector()
    
    # 注册关闭回调
    rospy.on_shutdown(injector.stop)
    
    # 在线程中运行
    thread = Thread(target=injector.start)
    thread.daemon = True
    thread.start()
    
    rospy.spin()


if __name__ == '__main__':
    main()
