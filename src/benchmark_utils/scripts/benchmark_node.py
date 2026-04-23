#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
benchmark_node.py - 重构版本

无人机控制器 Benchmark 测试节点

核心改进:
- 清晰的6状态机：INIT → MOVE_TO_START → STABILIZE → EXECUTING → COMPLETED → ANALYZING
- 精确的数据记录窗口：仅EXECUTING阶段记录
- 模块化设计：数据记录与分析分离
- 空间适配：支持3×3×2m有限测试场地

任务类型:
- Task 1: 悬停稳态（hover）
- Task 2: 动态轨迹（dynamic）

坐标系:
- 支持NED/ENU自动转换
- 输出始终为ENU坐标系
"""

import rospy
import numpy as np
from enum import Enum
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
import os
import sys
import csv
from datetime import datetime

# 导入轨迹生成器
try:
    from trajectory_generator import DifferentialFlatTrajectory
except ImportError:
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "trajectory_generator",
        os.path.join(os.path.dirname(__file__), "trajectory_generator.py")
    )
    trajectory_generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(trajectory_generator)
    DifferentialFlatTrajectory = trajectory_generator.DifferentialFlatTrajectory


class FrameTransformer:
    """坐标系转换工具：NED <-> ENU"""
    
    @staticmethod
    def ned_to_enu_position(vec_ned):
        """NED -> ENU 位置转换: [x_n, y_n, z_n] -> [y_e, x_e, -z_e]"""
        vec_ned = np.asarray(vec_ned)
        return np.array([vec_ned[1], vec_ned[0], -vec_ned[2]])
    
    @staticmethod
    def ned_to_enu_velocity(vel_ned):
        """NED -> ENU 速度转换"""
        return FrameTransformer.ned_to_enu_position(vel_ned)


class BenchmarkState(Enum):
    """Benchmark状态机定义"""
    INIT = "init"                      # 初始化，等待起飞
    MOVE_TO_START = "move_to_start"    # 移动到起始点
    STABILIZE = "stabilize"            # 起始点稳定
    EXECUTING = "executing"            # 执行任务（记录数据）
    COMPLETED = "completed"            # 任务完成
    ANALYZING = "analyzing"            # 分析中


class TaskType(Enum):
    """任务类型枚举"""
    HOVER = "hover"      # 悬停稳态
    DYNAMIC = "dynamic"  # 动态轨迹


class BenchmarkNode:
    """
    Benchmark任务节点（重构版）
    
    核心职责：
    1. 状态机管理
    2. 轨迹生成与发布
    3. 控制数据记录器的开始/停止
    4. 触发分析器生成报告
    """
    
    def __init__(self):
        rospy.init_node('benchmark_node', anonymous=True)
        
        # 初始化各模块
        self._load_parameters()
        self._init_state_machine()
        # 只在动态轨迹任务时初始化轨迹生成器
        if self.task_type == TaskType.DYNAMIC:
            self._init_trajectory_generator()
        self._setup_ros_interface()
        
        rospy.loginfo("="  * 70)
        rospy.loginfo("Benchmark Node 已启动 (重构版)")
        rospy.loginfo("任务类型: %s", self.task_type.value)
        rospy.loginfo("空间限制: 5m x 5m x 5m")
        rospy.loginfo("坐标系: %s -> ENU", "NED" if self.coordinate_frame == 0 else "ENU")
        rospy.loginfo("=" * 70)
        
    def _load_parameters(self):
        """加载ROS参数"""
        # 基本参数
        task_type_str = rospy.get_param('~task_type', 'hover')
        self.task_type = TaskType(task_type_str)
        self.rate_hz = rospy.get_param('~rate', 100.0)
        self.coordinate_frame = rospy.get_param('~coordinate_frame', 1)  # 0=NED, 1=ENU
        self.frame_id = rospy.get_param('~frame_id', 'world')
        
        # 起飞检测参数
        self.takeoff_height_threshold = rospy.get_param('~takeoff_height_threshold', 0.6)
        self.takeoff_stable_time = rospy.get_param('~takeoff_stable_time', 2.0)
        
        # 起始点参数
        self.start_pos_tolerance = rospy.get_param('~start_pos_tolerance', 0.2)
        self.start_stabilize_time = rospy.get_param('~start_stabilize_time', 2.0)
        
        # 任务参数
        # 兼容两种参数名：shell脚本传 _duration，也支持 _task_duration
        self.task_duration = rospy.get_param('~task_duration',
                             rospy.get_param('~duration', 60.0))
        
        # Task 1: 悬停参数 - 抗扰测试在单一点原地不动
        # 默认悬停点: 中心 (0, 0, 2.5)
        default_hover_point = [0.0, 0.0, 2.5]
        hover_point_param = rospy.get_param('~hover_point', default_hover_point)
        if isinstance(hover_point_param, str):
            import ast
            try:
                self.hover_point = ast.literal_eval(hover_point_param)
            except:
                rospy.logwarn("无法解析hover_point参数，使用默认值")
                self.hover_point = default_hover_point
        else:
            self.hover_point = hover_point_param
        self.hover_point = np.array(self.hover_point)
        
        # Task 2: 动态轨迹参数（适配5x5空间）
        self.trajectory_type = rospy.get_param('~trajectory_type', 'figure8')
        self.trajectory_speed = rospy.get_param('~trajectory_speed', 0.8)  # 基础速度
        self.trajectory_amplitude = rospy.get_param('~trajectory_amplitude', 1.0)  # 统一半径
        self.speed_level = rospy.get_param('~speed_level', 1.0)
        self.num_loops = rospy.get_param('~num_loops', 2)
        
        # 日志参数
        self.log_dir = rospy.get_param('~log_dir', '~/.benchmark_logs')
        self.log_dir = os.path.expanduser(self.log_dir)
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        # 数据记录状态
        self.recording = False
        self.log_file = None
        self.log_writer = None
        self.log_file_path = None
        self.record_start_time = None
        
        # 统计信息
        self.sample_count = 0
            
        # ROS话题
        self.odom_topic = rospy.get_param('~odom_topic', '/mavros/local_position/odom')
        self.position_cmd_topic = rospy.get_param('~position_cmd_topic', '/position_cmd')
        
    def _init_state_machine(self):
        """初始化状态机"""
        self.state = BenchmarkState.INIT
        self.state_entry_time = rospy.Time.now()
        
        # 无人机当前状态
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_velocity = np.zeros(3)
        self.current_yaw = 0.0  # 当前航向角 (rad)
        
        # 起飞检测
        self.takeoff_start_time = None
        self.takeoff_done = False
        
        # 起始点相关
        self.start_point = np.array([0.0, 0.0, 1.0])
        self.start_yaw = 0.0  # 起始yaw角
        self.stabilize_start_time = None
        
        # 任务执行
        self.task_start_time = None
        self.task_end_time = None
        
        # Task 1: 悬停任务状态
        self.current_hover_idx = 0
        self.hover_arrival_time = None
        
        # Task 2: 动态轨迹状态
        self.traj_start_time = None
        self.trajectory_duration = 0.0
        
    def _init_trajectory_generator(self):
        """初始化轨迹生成器"""
        self.traj_gen = DifferentialFlatTrajectory(self.frame_id)
        
        # 应用速度倍率
        actual_speed = self.trajectory_speed * self.speed_level
        self.traj_gen.set_params(actual_speed, self.trajectory_amplitude)
        self.traj_gen.num_loops = self.num_loops
        
        rospy.loginfo("轨迹生成器初始化：类型=%s, 速度=%.2fx, 振幅=%.2fm", 
                     self.trajectory_type, self.speed_level, self.trajectory_amplitude)
        
    def _setup_ros_interface(self):
        """设置ROS发布者和订阅者"""
        # 发布者
        self.cmd_pub = rospy.Publisher(self.position_cmd_topic, PositionCommand, queue_size=10)
        self.state_pub = rospy.Publisher('~state', String, queue_size=10)
        self.phase_pub = rospy.Publisher('~phase', String, queue_size=10) 
        self.progress_pub = rospy.Publisher('~progress', Float32, queue_size=10)
        self.logging_control_pub = rospy.Publisher('/benchmark/logging_control', String, queue_size=10)
        self.path_pub = rospy.Publisher('~ref_path', Path, queue_size=10)
        
        # 订阅者
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=10)
        
        # 定时器
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._control_loop)
        self.status_timer = rospy.Timer(rospy.Duration(0.2), self._status_publish_loop)
        
    def _odom_callback(self, msg):
        """里程计回调 - 自动处理坐标系转换"""
        if self.coordinate_frame == 0:  # NED输入
            position_ned = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            vel_ned = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])
            self.current_position = FrameTransformer.ned_to_enu_position(position_ned)
            self.current_velocity = FrameTransformer.ned_to_enu_velocity(vel_ned)
        else:  # ENU输入
            self.current_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            self.current_velocity = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])
        
        # 从四元数提取yaw角
        q = msg.pose.pose.orientation
        # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)
            
            
        # 调试日志：检查高度只前10次或每5秒
        # rospy.loginfo_throttle(5.0, f"Odom received: raw_z={msg.pose.pose.position.z:.2f}, enu_z={self.current_position[2]:.2f}, frame={self.coordinate_frame}")

        # 检查起飞状态
        if not self.takeoff_done:
            rospy.loginfo_throttle(2.0, f"等待起飞... 当前高度(ENU): {self.current_position[2]:.2f}m (阈值: {self.takeoff_height_threshold}m)")
            self._check_takeoff()
            
    def _check_takeoff(self):
        """检查起飞状态"""
        current_height = self.current_position[2]
        
        if current_height >= self.takeoff_height_threshold:
            if self.takeoff_start_time is None:
                self.takeoff_start_time = rospy.Time.now()
            
            elapsed = (rospy.Time.now() - self.takeoff_start_time).to_sec()
            if elapsed >= self.takeoff_stable_time:
                self.takeoff_done = True
                rospy.loginfo("=" * 50)
                rospy.loginfo("起飞完成！高度: %.2fm", current_height)
                rospy.loginfo("=" * 50)
        else:
            self.takeoff_start_time = None
            
    def _control_loop(self, event):
        """主控制循环 - 状态机核心"""
        # 状态转换逻辑
        self._state_transition()
        
        # 根据当前状态执行相应动作
        if self.state == BenchmarkState.INIT:
            pass  # 等待起飞
            
        elif self.state == BenchmarkState.MOVE_TO_START:
            self._move_to_start_point()
            
        elif self.state == BenchmarkState.STABILIZE:
            self._stabilize_at_start()
            
        elif self.state == BenchmarkState.EXECUTING:
            self._execute_task()
            
        elif self.state == BenchmarkState.COMPLETED:
            self._handle_completion()
            
        elif self.state == BenchmarkState.ANALYZING:
            pass  # 分析阶段，停止发布控制指令
            
    def _state_transition(self):
        """状态机转换逻辑"""
        prev_state = self.state
        
        if self.state == BenchmarkState.INIT:
            if self.takeoff_done:
                self.state = BenchmarkState.MOVE_TO_START
                self._determine_start_point()
                
        elif self.state == BenchmarkState.MOVE_TO_START:
            dist = np.linalg.norm(self.current_position - self.start_point)
            
            # 记录移动开始时间（如果还没记录）
            if not hasattr(self, 'move_start_time') or self.move_start_time is None:
                self.move_start_time = rospy.Time.now()
                
            # 每2秒打印一次状态
            if hasattr(self, 'last_move_log_time'):
                if (rospy.Time.now() - self.last_move_log_time).to_sec() >= 2.0:
                    rospy.loginfo(f"移动到起点中... 距离: {dist:.2f}m (阈值: {self.start_pos_tolerance}m)")
                    self.last_move_log_time = rospy.Time.now()
            else:
                self.last_move_log_time = rospy.Time.now()
            
            if dist < self.start_pos_tolerance:
                self.state = BenchmarkState.STABILIZE
                self.stabilize_start_time = rospy.Time.now()
                self.move_start_time = None  # 重置
            else:
                # 超时检测 (30秒)
                move_timeout = 30.0
                elapsed = (rospy.Time.now() - self.move_start_time).to_sec()
                if elapsed > move_timeout:
                    rospy.logwarn(f"移动到起点超时 ({move_timeout}s), 强制进入稳定阶段. 当前距离: {dist:.2f}m")
                    self.state = BenchmarkState.STABILIZE
                    self.stabilize_start_time = rospy.Time.now()
                    self.move_start_time = None
                
        elif self.state == BenchmarkState.STABILIZE:
            if self.stabilize_start_time is not None:
                elapsed = (rospy.Time.now() - self.stabilize_start_time).to_sec()
                if elapsed >= self.start_stabilize_time:
                    self.state = BenchmarkState.EXECUTING
                    self.task_start_time = rospy.Time.now()
                    
                    # 开始记录数据
                    self._start_recording()
                    rospy.loginfo("=" * 70)
                    rospy.loginfo(">>> 开始记录数据 <<<")
                    rospy.loginfo("=" * 70)
                    
        elif self.state == BenchmarkState.EXECUTING:
            if self._is_task_complete():
                self.state = BenchmarkState.COMPLETED
                self.task_end_time = rospy.Time.now()
                
                # 停止记录数据
                self._stop_recording()
                rospy.loginfo("=" * 70)
                rospy.loginfo(">>> 停止记录数据 <<<")
                rospy.loginfo("=" * 70)
                
        elif self.state == BenchmarkState.COMPLETED:
            # 给logger一点时间保存文件
            elapsed = (rospy.Time.now() - self.task_end_time).to_sec()
            if elapsed > 1.0:
                self.state = BenchmarkState.ANALYZING
                self._trigger_analysis()
                
        # 状态切换日志
        if self.state != prev_state:
            self.state_entry_time = rospy.Time.now()
            rospy.loginfo("[状态转换] %s -> %s", prev_state.value, self.state.value)
            
    def _determine_start_point(self):
        """确定起始点和起始yaw"""
        if self.task_type == TaskType.HOVER:
            self.start_point = np.array(self.hover_point, dtype=float)
            self.start_yaw = 0.0  # 悬停任务默认yaw=0
        elif self.task_type == TaskType.DYNAMIC:
            # 使用轨迹的起始点和起始yaw
            # get_trajectory返回6个值: pos, vel, acc, jerk, yaw, yaw_dot
            try:
                ret = self.traj_gen.get_trajectory(self.trajectory_type, 0.0)
                rospy.loginfo(f"Trajectory Return Type: {type(ret)}, Length: {len(ret)}")
                pos, _, _, _, yaw, _ = ret
                self.start_yaw = float(yaw)
            except Exception as e:
                rospy.logerr(f"CRITICAL ERROR in get_trajectory unpacking: {e}")
                rospy.logerr(f"Trajectory Type: {self.trajectory_type}")
                # Fallback
                pos = np.array([0,0,1])
                self.start_yaw = 0.0

            # ================================================================
            # 关键：用无人机当前实际位置作为世界偏移，
            # 让圆/八字等轨迹从无人机实际悬停点出发，而不是世界原点
            # ================================================================
            drone_pos = self.current_position.copy()
            self.traj_gen.set_world_offset(drone_pos)

            # 轨迹起始点 = 无人机当前位置（偏移后 t=0 的位置）
            self.start_point = drone_pos.copy()

        rospy.loginfo("起始点设置为: [%.2f, %.2f, %.2f], yaw=%.2f rad (%.1f deg)",
                     self.start_point[0], self.start_point[1], self.start_point[2],
                     self.start_yaw, np.degrees(self.start_yaw))
                     
    def _is_task_complete(self):
        """判断任务是否完成"""
        if self.task_start_time is None:
            return False
            
        elapsed = (rospy.Time.now() - self.task_start_time).to_sec()
        
        # 基于时间的完成条件
        if elapsed >= self.task_duration:
            return True
            
        # Task 2特殊条件：轨迹圈数完成
        if self.task_type == TaskType.DYNAMIC and self.traj_start_time is not None:
            traj_elapsed = (rospy.Time.now() - self.traj_start_time).to_sec()
            if traj_elapsed >= self.trajectory_duration:
                return True
                
        return False
        
    def _move_to_start_point(self):
        """移动到起始点，同时调整yaw"""
        self._publish_position_cmd(self.start_point, np.zeros(3), np.zeros(3), self.start_yaw)
        
    def _stabilize_at_start(self):
        """在起始点稳定，保持正确的yaw"""
        self._publish_position_cmd(self.start_point, np.zeros(3), np.zeros(3), self.start_yaw)
        
    def _execute_task(self):
        """执行任务（根据任务类型）"""
        if self.task_type == TaskType.HOVER:
            self._execute_hover_task()
        elif self.task_type == TaskType.DYNAMIC:
            self._execute_dynamic_task()
            
    def _execute_hover_task(self):
        """执行悬停任务 - 在固定点原地悬停，测试抗扰能力"""
        # 始终发送同一个悬停点作为目标
        target = self.hover_point
        self._publish_position_cmd(target, np.zeros(3), np.zeros(3), 0.0)
        
    def _execute_dynamic_task(self):
        """执行动态轨迹任务"""
        # 初始化轨迹计时
        if self.traj_start_time is None:
            self.traj_start_time = rospy.Time.now()
            period = self.traj_gen.get_trajectory_period(self.trajectory_type)
            # 时间缩放软启动会压缩前 T 秒：tau(T)=0.5*T，需补偿这段时间差
            # 真正需要的实际时长 = N*period + 0.5*soft_start_duration
            soft_start_compensation = 0.0
            if self.traj_gen.soft_start_enabled:
                soft_start_compensation = 0.5 * self.traj_gen.soft_start_duration
            self.trajectory_duration = period * self.num_loops + soft_start_compensation
            rospy.loginfo("开始轨迹跟踪：%s, 周期=%.2fs, 圈数=%d, 总时长=%.2fs (含软启动补偿%.1fs)",
                         self.trajectory_type, period, self.num_loops,
                         self.trajectory_duration, soft_start_compensation)
                         
        # 计算轨迹时间
        traj_t = (rospy.Time.now() - self.traj_start_time).to_sec()
        
        # 生成轨迹点
        # trajectory_generator返回6个值: pos, vel, acc, jerk, yaw, yaw_dot
        try:
            ret = self.traj_gen.get_trajectory(self.trajectory_type, traj_t)
            pos, vel, acc, jerk, yaw, yaw_dot = ret
        except Exception as e:
            rospy.logerr_throttle(1.0, f"Error generating trajectory: {e}")
            pos = np.zeros(3)
            vel = np.zeros(3)
            acc = np.zeros(3)
            jerk = np.zeros(3)
            yaw = 0.0
            yaw_dot = 0.0
            
        # 发布控制指令 (包含完整微分平坦信息: jerk, yaw_dir, yaw_dir_dot)
        self._publish_position_cmd(pos, vel, acc, yaw, yaw_dot, jerk)
        
    def _handle_completion(self):
        """处理完成状态 - 返回起点"""
        # 返回起点位置（为下一个任务准备）
        self._publish_position_cmd(self.start_point, np.zeros(3), np.zeros(3), 0.0)
        
    def _trigger_analysis(self):
        """触发数据分析"""
        rospy.loginfo("=" * 70)
        rospy.loginfo("任务完成，开始分析数据...")
        rospy.loginfo("=" * 70)
        
        # 查找最新的CSV文件
        import glob
        csv_files = glob.glob(os.path.join(self.log_dir, "*.csv"))
        if not csv_files:
            rospy.logwarn("未找到CSV日志文件，跳过分析")
            return
            
        # 获取最新的CSV文件
        latest_csv = max(csv_files, key=os.path.getctime)
        rospy.loginfo("分析文件: %s", latest_csv)
        
        # 调用analyzer
        try:
            import subprocess
            analyzer_script = os.path.join(
                os.path.dirname(__file__), 
                "benchmark_analyzer.py"
            )
            
            # 提取任务名称
            basename = os.path.basename(latest_csv)
            task_name = basename.replace('.csv', '')
            
            # 后台运行analyzer
            cmd = [
                'python', analyzer_script,
                '--csv', latest_csv,
                '--name', task_name,
                '--output', self.log_dir
            ]
            # 自动调参时跳过图表生成以加速
            if rospy.get_param('~skip_plot', False):
                cmd.append('--no-plot')
            
            rospy.loginfo("启动分析器: %s", ' '.join(cmd))
            rospy.loginfo("启动分析器: %s", ' '.join(cmd))
            
            # 阻塞等待分析完成 (确保auto_tuner能获取结果)
            subprocess.run(cmd, check=True)
            
            rospy.loginfo("分析器运行完成")
            rospy.loginfo("结果已保存在: %s", self.log_dir)
            
            # 给一点时间让日志写入
            rospy.sleep(1.0)
            
            # 回到原点(0,0,1)再退出，方便下一轮调参
            rospy.loginfo("返回原点 [0, 0, 1]...")
            origin = np.array([0.0, 0.0, 1.0])
            rate = rospy.Rate(50)
            for _ in range(150):  # 3秒
                if rospy.is_shutdown():
                    break
                self._publish_position_cmd(origin, np.zeros(3), np.zeros(3), 0.0)
                rate.sleep()
            
            # 主动退出节点
            rospy.loginfo("Benchmark任务全部完成，节点自动退出")
            rospy.signal_shutdown("Benchmark Completed")
            sys.exit(0) # 强制退出
            
        except Exception as e:
            rospy.logerr("启动分析器失败: %s", str(e))
            rospy.loginfo("您可以手动运行分析：")
            rospy.loginfo("  rosrun benchmark_utils benchmark_analyzer.py --csv %s", latest_csv)
        
        rospy.loginfo("=" * 70)
        
    def _publish_position_cmd(self, position, velocity, acceleration, yaw, yaw_dot=0.0, jerk=None):
        """发布位置指令（ENU坐标系）- 包含完整的微分平坦信息"""
        msg = PositionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        
        msg.position.x = float(position[0])
        msg.position.y = float(position[1])
        msg.position.z = float(position[2])
        
        msg.velocity.x = float(velocity[0])
        msg.velocity.y = float(velocity[1])
        msg.velocity.z = float(velocity[2])
        
        msg.acceleration.x = float(acceleration[0])
        msg.acceleration.y = float(acceleration[1])
        msg.acceleration.z = float(acceleration[2])
        
        # Jerk (加加速度) - 如果没有提供则设为0
        if jerk is not None:
            msg.jerk.x = float(jerk[0])
            msg.jerk.y = float(jerk[1])
            msg.jerk.z = float(jerk[2])
        else:
            msg.jerk.x = 0.0
            msg.jerk.y = 0.0
            msg.jerk.z = 0.0
        
        # Yaw方向向量 (从yaw角度计算)
        msg.yaw_dir.x = float(np.cos(yaw))
        msg.yaw_dir.y = float(np.sin(yaw))
        msg.yaw_dir.z = 0.0
        
        # Yaw方向向量的导数 (从yaw_dot计算)
        msg.yaw_dir_dot.x = float(-np.sin(yaw) * yaw_dot)
        msg.yaw_dir_dot.y = float(np.cos(yaw) * yaw_dot)
        msg.yaw_dir_dot.z = 0.0
        
        msg.yaw = float(yaw)
        msg.yaw_dot = float(yaw_dot)
        
        # 轨迹标识
        msg.trajectory_id = 1
        msg.trajectory_flag = 0
        
        self.cmd_pub.publish(msg)

        # 记录数据 (target vs actual，包含加速度和jerk)
        acc_array = np.array([msg.acceleration.x, msg.acceleration.y, msg.acceleration.z])
        jerk_array = np.array([msg.jerk.x, msg.jerk.y, msg.jerk.z])
        self._record_data(position, velocity, acc_array, jerk_array, yaw, yaw_dot)
        
    def _status_publish_loop(self, event):
        """发布状态信息"""
        # 发布状态
        self.state_pub.publish(String(self.state.value))
        
        # 发布阶段描述
        phase_desc = self._get_phase_description()
        self.phase_pub.publish(String(phase_desc))
        
        # 发布进度
        progress = self._calculate_progress()
        self.progress_pub.publish(Float32(progress))
        
    def _get_phase_description(self):
        """获取阶段描述"""
        if self.state == BenchmarkState.INIT:
            return "等待起飞完成..."
        elif self.state == BenchmarkState.MOVE_TO_START:
            dist = np.linalg.norm(self.current_position - self.start_point)
            return f"前往起始点 (距离: {dist:.2f}m)"
        elif self.state == BenchmarkState.STABILIZE:
            if self.stabilize_start_time:
                elapsed = (rospy.Time.now() - self.stabilize_start_time).to_sec()
                remaining = max(0, self.start_stabilize_time - elapsed)
                return f"起始点稳定中 ({remaining:.1f}s)"
            return "起始点稳定中"
        elif self.state == BenchmarkState.EXECUTING:
            if self.task_type == TaskType.HOVER:
                return f"悬停抗扰测试 (固定点)"
            else:
                if self.traj_start_time:
                    elapsed = (rospy.Time.now() - self.traj_start_time).to_sec()
                    progress = (elapsed / self.trajectory_duration * 100) if self.trajectory_duration > 0 else 0
                    return f"{self.trajectory_type} 轨迹 ({progress:.1f}%)"
                return f"{self.trajectory_type} 轨迹执行中"
        elif self.state == BenchmarkState.COMPLETED:
            return "任务完成，保存数据..."
        elif self.state == BenchmarkState.ANALYZING:
            return "数据分析中..."
        return "未知状态"
        
    def _calculate_progress(self):
        """计算任务进度 (0.0 - 1.0)"""
        if self.state in [BenchmarkState.INIT, BenchmarkState.MOVE_TO_START, BenchmarkState.STABILIZE]:
            return 0.0
        elif self.state == BenchmarkState.EXECUTING:
            if self.task_start_time:
                elapsed = (rospy.Time.now() - self.task_start_time).to_sec()
                return min(elapsed / self.task_duration, 1.0)
            return 0.0
        elif self.state in [BenchmarkState.COMPLETED, BenchmarkState.ANALYZING]:
            return 1.0
        return 0.0
        
    def run(self):
        """运行节点"""
        rospy.spin()


    # =========================================================================
    # 数据记录功能
    # =========================================================================

    def _start_recording(self):
        """开始记录"""
        # 生成文件名
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        # task_name = rospy.get_param('/benchmark_node/task_type', 'unknown') # 这里的task_type是在初始化时定的
        task_name = "dynamic" if self.task_type == TaskType.DYNAMIC else "hover"
        
        filename = f"{task_name}_{timestamp}.csv"
        self.log_file_path = os.path.join(self.log_dir, filename)
        
        try:
            # 打开文件
            self.log_file = open(self.log_file_path, 'w', newline='')
            self.log_writer = csv.writer(self.log_file)
            
            # 写入header (扩展版：包含加速度和jerk用于能耗/平滑度分析)
            self.log_writer.writerow([
                'time',
                'target_x', 'target_y', 'target_z',
                'actual_x', 'actual_y', 'actual_z',
                'target_vx', 'target_vy', 'target_vz',
                'actual_vx', 'actual_vy', 'actual_vz',
                'target_ax', 'target_ay', 'target_az',
                'target_jerk_x', 'target_jerk_y', 'target_jerk_z',
                'target_yaw', 'actual_yaw', 'target_yaw_dot'
            ])
            
            # 重置状态
            self.recording = True
            self.record_start_time = rospy.Time.now()
            self.sample_count = 0
            
            rospy.loginfo("=" * 60)
            rospy.loginfo("开始记录数据")
            rospy.loginfo("文件: %s", self.log_file_path)
            rospy.loginfo("=" * 60)
            
        except Exception as e:
            rospy.logerr(f"无法创建日志文件: {e}")
            self.recording = False
        
    def _stop_recording(self):
        """停止记录"""
        self.recording = False
        
        if self.log_file:
            try:
                self.log_file.close()
                self.log_file = None
                rospy.loginfo("记录停止，文件已保存: %s", self.log_file_path)
                rospy.loginfo("总采样数: %d", self.sample_count)
            except Exception as e:
                rospy.logerr(f"关闭日志文件失败: {e}")

    def _record_data(self, target_pos, target_vel, target_acc=None, target_jerk=None, target_yaw=0.0, target_yaw_dot=0.0):
        """记录一行数据（扩展版：包含加速度和jerk）"""
        if not self.recording or self.log_writer is None:
            return

        try:
            # 计算时间（从开始记录算起）
            elapsed = (rospy.Time.now() - self.record_start_time).to_sec()

            # 处理可选参数
            if target_acc is None:
                target_acc = np.zeros(3)
            if target_jerk is None:
                target_jerk = np.zeros(3)

            # 写入CSV
            self.log_writer.writerow([
                elapsed,
                target_pos[0], target_pos[1], target_pos[2],
                self.current_position[0], self.current_position[1], self.current_position[2],
                target_vel[0], target_vel[1], target_vel[2],
                self.current_velocity[0], self.current_velocity[1], self.current_velocity[2],
                target_acc[0], target_acc[1], target_acc[2],
                target_jerk[0], target_jerk[1], target_jerk[2],
                target_yaw, self.current_yaw, target_yaw_dot
            ])

            self.sample_count += 1

            # 每100个样本刷新一次文件防止丢失
            if self.sample_count % 100 == 0:
                self.log_file.flush()

        except Exception as e:
            rospy.logwarn_throttle(1.0, f"记录数据失败: {e}")

if __name__ == '__main__':
    try:
        node = BenchmarkNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
