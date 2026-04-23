#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
traj_pub_node.py
功能：
1. 随机航点悬停
2. CSV轨迹多圈加速发布
3. 坐标系统一（ENU）
"""

import rospy
import numpy as np
import os
from enum import Enum
from std_msgs.msg import Header, String, Float32
from geometry_msgs.msg import Point, Vector3, Quaternion, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import tf2_ros

from trajectory_publisher.msg import TrajectoryPoint
from nmpc_control.utils.transforms import FrameTransformer


class FlightPhase(Enum):
    """飞行阶段枚举"""
    IDLE = 0
    GOTO_WAYPOINT = 1
    HOVERING_AT_WAYPOINT = 2
    GOTO_TRAJ_START = 3
    HOVERING_AT_TRAJ_START = 4
    TRACKING_TRAJECTORY = 5
    LAP_TRANSITION = 6
    HOLDING_FINAL = 7
    FINISHED = 8


class TrajectoryPublisher:
    def __init__(self):
        rospy.init_node('trajectory_publisher')
        
        # ===================== 参数 =====================
        csv_files_param = rospy.get_param('~csv_files', '')
        if csv_files_param:
            self.csv_files = [f.strip() for f in csv_files_param.split(',') if f.strip()]
        else:
            self.csv_files = []
        
        self.frame_id = rospy.get_param('~frame_id', 'world')
        self.rate = rospy.get_param('~rate', 100.0)
        self.loop = rospy.get_param('~loop', False)
        self.waypoint_threshold = rospy.get_param('~waypoint_threshold', 0.3)
        self.start_threshold = rospy.get_param('~start_threshold', 0.2)
        self.odom_type = rospy.get_param('~odom_type', 0)
        self.max_path_points = rospy.get_param('~max_path_points', 5000)
        self.csv_coord_type = rospy.get_param('~csv_coord_type', 0)
        
        # 随机航点参数
        self.num_random_waypoints = rospy.get_param('~num_random_waypoints', 3)
        self.random_range_x = rospy.get_param('~random_range_x', [-2.0, 2.0])
        self.random_range_y = rospy.get_param('~random_range_y', [-2.0, 2.0])
        self.random_range_z = rospy.get_param('~random_range_z', [0.8, 1.5])
        self.waypoint_hold_time = rospy.get_param('~waypoint_hold_time', 3.0)
        
        # 多圈加速参数
        self.num_laps = rospy.get_param('~num_laps', 5)
        self.initial_speed_factor = rospy.get_param('~initial_speed_factor', 0.5)
        self.speed_increment = rospy.get_param('~speed_increment', 0.25)
        self.max_speed_factor = rospy.get_param('~max_speed_factor', 2.0)
        self.lap_transition_time = rospy.get_param('~lap_transition_time', 1.0)
        
        # 轨迹起点悬停时间
        self.traj_start_hold_time = rospy.get_param('~traj_start_hold_time', 2.0)
        
        # 最终悬停参数
        self.final_hold_time = rospy.get_param('~final_hold_time', -1)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Trajectory Publisher Node")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Odom type: {'NED->ENU' if self.odom_type == 0 else 'ENU'}")
        rospy.loginfo(f"CSV coord type: {'NED->ENU' if self.csv_coord_type == 0 else 'ENU'}")
        rospy.loginfo(f"CSV files: {self.csv_files if self.csv_files else 'None'}")
        rospy.loginfo(f"Random waypoints: {self.num_random_waypoints}")
        rospy.loginfo(f"Traj start hold time: {self.traj_start_hold_time}s")
        rospy.loginfo("-" * 40)
        rospy.loginfo("Lap Configuration:")
        rospy.loginfo(f"  Laps: {self.num_laps}")
        rospy.loginfo(f"  Initial: {self.initial_speed_factor}x")
        rospy.loginfo(f"  Increment: +{self.speed_increment}x")
        rospy.loginfo(f"  Max: {self.max_speed_factor}x")
        rospy.loginfo("=" * 60)
        
        # ===================== 加载CSV轨迹 =====================
        self.trajectories = []
        for csv_file in self.csv_files:
            traj = self._load_trajectory(csv_file)
            if traj is not None:
                self.trajectories.append(traj)
        
        # ===================== 生成随机航点 =====================
        self.random_waypoints = self._generate_random_waypoints()
        
        # ===================== 状态变量 =====================
        self.current_position = None
        self.current_quat = None
        self.current_velocity = None
        self.current_angular_velocity = None
        
        self.phase = FlightPhase.IDLE
        self.current_waypoint_idx = 0
        self.current_traj_idx = 0
        self.current_lap = 0
        self.current_speed_factor = self.initial_speed_factor
        
        self.traj_start_time = None
        self.hover_start_time = None
        self.lap_transition_start_time = None
        self.traj_start_hold_start_time = None
        self.final_hold_start_time = None
        
        self.final_hold_position = None
        self.final_hold_quat = None
        
        self.hover_target_position = None
        self.hover_target_quat = None
        
        self.seq = 0
        self.odom_seq = 0
        
        # ===================== 发布者 =====================
        self.pub = rospy.Publisher('~target', TrajectoryPoint, queue_size=10)
        self.odom_enu_pub = rospy.Publisher('~odom_enu', Odometry, queue_size=10)
        self.odom_path_pub = rospy.Publisher('~odom_path', Path, queue_size=10)
        self.ref_path_pub = rospy.Publisher('~ref_path', Path, queue_size=10)
        self.odom_pose_pub = rospy.Publisher('~odom_pose', PoseStamped, queue_size=10)
        self.phase_pub = rospy.Publisher('~phase', String, queue_size=10)
        self.waypoint_pub = rospy.Publisher('~waypoints', Path, queue_size=10)
        self.speed_factor_pub = rospy.Publisher('~speed_factor', Float32, queue_size=10)
        self.lap_pub = rospy.Publisher('~current_lap', String, queue_size=10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        self.odom_path_msg = Path()
        self.odom_path_msg.header.frame_id = self.frame_id
        
        self.ref_path_msg = Path()
        self.ref_path_msg.header.frame_id = self.frame_id
        
        self._publish_waypoints_visualization()
        self._generate_all_ref_paths()
        
        self.vio_sub = rospy.Subscriber("/vio/odometry", Odometry, self.odom_callback)
        
        rospy.loginfo("Waiting for VIO odometry...")

    def _ned_to_enu_quat(self, quat_ned):
        """NED四元数转ENU"""
        try:
            return FrameTransformer.nedfrd_to_enuflu_rotation(quat_ned)
        except:
            return np.array([quat_ned[0], quat_ned[2], quat_ned[1], -quat_ned[3]])

    def _load_trajectory(self, csv_file):
        """加载CSV轨迹并转换到ENU坐标系"""
        try:
            if not os.path.exists(csv_file):
                import rospkg
                try:
                    rospack = rospkg.RosPack()
                    pkg_path = rospack.get_path('trajectory_publisher')
                    csv_file = os.path.join(pkg_path, 'trajectories', csv_file)
                except:
                    pass
            
            if not os.path.exists(csv_file):
                rospy.logwarn(f"File not found: {csv_file}")
                return None
            
            data = np.loadtxt(csv_file, delimiter=',', skiprows=1)
            n_points = len(data)
            dt = data[1, 0] - data[0, 0] if n_points > 1 else 0.01
            duration = data[-1, 0]
            
            # NED -> ENU 转换
            if self.csv_coord_type == 0:
                rospy.loginfo(f"Converting {os.path.basename(csv_file)} from NED to ENU...")
                
                # 位置
                pos_ned = data[:, 1:4].copy()
                data[:, 1] = pos_ned[:, 1]   # ENU.x = NED.y
                data[:, 2] = pos_ned[:, 0]   # ENU.y = NED.x
                data[:, 3] = -pos_ned[:, 2]  # ENU.z = -NED.z
                
                # 速度
                vel_ned = data[:, 4:7].copy()
                data[:, 4] = vel_ned[:, 1]
                data[:, 5] = vel_ned[:, 0]
                data[:, 6] = -vel_ned[:, 2]
                
                # 加速度
                acc_ned = data[:, 7:10].copy()
                data[:, 7] = acc_ned[:, 1]
                data[:, 8] = acc_ned[:, 0]
                data[:, 9] = -acc_ned[:, 2]
                
                # 角速度 FRD -> FLU
                omega_ned = data[:, 10:13].copy()
                data[:, 10] = omega_ned[:, 0]
                data[:, 11] = -omega_ned[:, 1]
                data[:, 12] = -omega_ned[:, 2]
                
                # 四元数
                for i in range(n_points):
                    quat_ned = data[i, 13:17].copy()
                    quat_enu = self._ned_to_enu_quat(quat_ned)
                    data[i, 13:17] = quat_enu
            
            start_pos = data[0, 1:4].copy()
            start_quat = data[0, 13:17].copy()
            start_quat /= np.linalg.norm(start_quat)
            
            end_pos = data[-1, 1:4].copy()
            end_quat = data[-1, 13:17].copy()
            end_quat /= np.linalg.norm(end_quat)
            
            traj = {
                'file': csv_file,
                'name': os.path.basename(csv_file),
                'data': data,
                'n_points': n_points,
                'dt': dt,
                'duration': duration,
                'start_pos': start_pos,
                'start_quat': start_quat,
                'end_pos': end_pos,
                'end_quat': end_quat,
            }
            
            rospy.loginfo(f"Loaded: {traj['name']} ({n_points} pts, {duration:.2f}s)")
            rospy.loginfo(f"  Start: [{start_pos[0]:.2f}, {start_pos[1]:.2f}, {start_pos[2]:.2f}]")
            rospy.loginfo(f"  End:   [{end_pos[0]:.2f}, {end_pos[1]:.2f}, {end_pos[2]:.2f}]")
            
            return traj
            
        except Exception as e:
            rospy.logerr(f"Failed to load {csv_file}: {e}")
            import traceback
            rospy.logerr(traceback.format_exc())
            return None

    def _generate_random_waypoints(self):
        """生成随机航点序列"""
        waypoints = []
        
        for i in range(self.num_random_waypoints):
            x = np.random.uniform(self.random_range_x[0], self.random_range_x[1])
            y = np.random.uniform(self.random_range_y[0], self.random_range_y[1])
            z = np.random.uniform(self.random_range_z[0], self.random_range_z[1])
            
            yaw = np.random.uniform(-np.pi, np.pi)
            quat = self._yaw_to_quat(yaw)
            
            waypoint = {
                'position': np.array([x, y, z]),
                'quat': quat,
                'yaw': yaw
            }
            waypoints.append(waypoint)
            
            rospy.loginfo(f"Waypoint {i+1}: [{x:.2f}, {y:.2f}, {z:.2f}]")
        
        return waypoints

    def _yaw_to_quat(self, yaw):
        """偏航角转四元数 [w, x, y, z]"""
        return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])

    def get_speed_factor_for_lap(self, lap):
        """计算指定圈数的速度因子"""
        factor = self.initial_speed_factor + lap * self.speed_increment
        return min(factor, self.max_speed_factor)

    def get_scaled_duration(self, original_duration):
        """根据速度因子计算缩放后的轨迹时长"""
        return original_duration / self.current_speed_factor

    def _publish_waypoints_visualization(self):
        """发布航点可视化"""
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = rospy.Time.now()
        
        for wp in self.random_waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wp['position'][0]
            pose.pose.position.y = wp['position'][1]
            pose.pose.position.z = wp['position'][2]
            pose.pose.orientation.w = wp['quat'][0]
            pose.pose.orientation.x = wp['quat'][1]
            pose.pose.orientation.y = wp['quat'][2]
            pose.pose.orientation.z = wp['quat'][3]
            path_msg.poses.append(pose)
        
        for traj in self.trajectories:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = traj['start_pos'][0]
            pose.pose.position.y = traj['start_pos'][1]
            pose.pose.position.z = traj['start_pos'][2]
            pose.pose.orientation.w = traj['start_quat'][0]
            pose.pose.orientation.x = traj['start_quat'][1]
            pose.pose.orientation.y = traj['start_quat'][2]
            pose.pose.orientation.z = traj['start_quat'][3]
            path_msg.poses.append(pose)
        
        self.waypoint_pub.publish(path_msg)

    def _generate_all_ref_paths(self):
        """生成所有轨迹的参考路径"""
        self.ref_path_msg = Path()
        self.ref_path_msg.header.frame_id = self.frame_id
        
        for traj in self.trajectories:
            data = traj['data']
            n_points = traj['n_points']
            step = max(1, n_points // 500)
            
            for i in range(0, n_points, step):
                row = data[i]
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.pose.position.x = row[1]
                pose.pose.position.y = row[2]
                pose.pose.position.z = row[3]
                
                quat = row[13:17]
                quat_norm = np.linalg.norm(quat)
                if quat_norm > 1e-6:
                    quat = quat / quat_norm
                
                pose.pose.orientation.w = quat[0]
                pose.pose.orientation.x = quat[1]
                pose.pose.orientation.y = quat[2]
                pose.pose.orientation.z = quat[3]
                
                self.ref_path_msg.poses.append(pose)

    def odom_callback(self, msg):
        """VIO里程计回调"""
        stamp = msg.header.stamp
        
        if self.odom_type == 0:  # NED -> ENU
            position_ned = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            self.current_position = FrameTransformer.ned_to_enu_position(position_ned)
            
            velocity_ned = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])
            self.current_velocity = FrameTransformer.ned_to_enu_position(velocity_ned)
            
            angular_vel_ned = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])
            self.current_angular_velocity = FrameTransformer.ned_to_enu_position(angular_vel_ned)
            
            quat_ned = np.array([
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ])
            self.current_quat = FrameTransformer.nedfrd_to_enuflu_rotation(quat_ned)
        else:
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
            self.current_angular_velocity = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])
            self.current_quat = np.array([
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z
            ])
        
        quat_norm = np.linalg.norm(self.current_quat)
        if quat_norm > 1e-6:
            self.current_quat /= quat_norm
        else:
            self.current_quat = np.array([1.0, 0.0, 0.0, 0.0])
        
        self._publish_odom_enu(stamp)
        self._publish_odom_pose(stamp)
        self._publish_tf(stamp)
        self._update_odom_path(stamp)

    def _publish_odom_enu(self, stamp):
        odom_msg = Odometry()
        odom_msg.header.seq = self.odom_seq
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = "base_link_enu"
        self.odom_seq += 1
        
        odom_msg.pose.pose.position.x = self.current_position[0]
        odom_msg.pose.pose.position.y = self.current_position[1]
        odom_msg.pose.pose.position.z = self.current_position[2]
        odom_msg.pose.pose.orientation.w = self.current_quat[0]
        odom_msg.pose.pose.orientation.x = self.current_quat[1]
        odom_msg.pose.pose.orientation.y = self.current_quat[2]
        odom_msg.pose.pose.orientation.z = self.current_quat[3]
        odom_msg.twist.twist.linear.x = self.current_velocity[0]
        odom_msg.twist.twist.linear.y = self.current_velocity[1]
        odom_msg.twist.twist.linear.z = self.current_velocity[2]
        odom_msg.twist.twist.angular.x = self.current_angular_velocity[0]
        odom_msg.twist.twist.angular.y = self.current_angular_velocity[1]
        odom_msg.twist.twist.angular.z = self.current_angular_velocity[2]
        
        self.odom_enu_pub.publish(odom_msg)

    def _publish_odom_pose(self, stamp):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = self.current_position[0]
        pose_msg.pose.position.y = self.current_position[1]
        pose_msg.pose.position.z = self.current_position[2]
        pose_msg.pose.orientation.w = self.current_quat[0]
        pose_msg.pose.orientation.x = self.current_quat[1]
        pose_msg.pose.orientation.y = self.current_quat[2]
        pose_msg.pose.orientation.z = self.current_quat[3]
        self.odom_pose_pub.publish(pose_msg)

    def _publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = "base_link_enu"
        t.transform.translation.x = self.current_position[0]
        t.transform.translation.y = self.current_position[1]
        t.transform.translation.z = self.current_position[2]
        t.transform.rotation.w = self.current_quat[0]
        t.transform.rotation.x = self.current_quat[1]
        t.transform.rotation.y = self.current_quat[2]
        t.transform.rotation.z = self.current_quat[3]
        self.tf_broadcaster.sendTransform(t)

    def _update_odom_path(self, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.current_position[0]
        pose.pose.position.y = self.current_position[1]
        pose.pose.position.z = self.current_position[2]
        pose.pose.orientation.w = self.current_quat[0]
        pose.pose.orientation.x = self.current_quat[1]
        pose.pose.orientation.y = self.current_quat[2]
        pose.pose.orientation.z = self.current_quat[3]
        
        self.odom_path_msg.header.stamp = stamp
        self.odom_path_msg.poses.append(pose)
        
        if len(self.odom_path_msg.poses) > self.max_path_points:
            self.odom_path_msg.poses.pop(0)
        
        self.odom_path_pub.publish(self.odom_path_msg)

    def get_distance_to(self, target_pos):
        """计算到目标点的距离"""
        if self.current_position is None:
            return float('inf')
        return np.linalg.norm(self.current_position - target_pos)

    def build_trajectory_msg(self, position, velocity, acceleration, angular_velocity, quat):
        msg = TrajectoryPoint()
        msg.header = Header()
        msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        self.seq += 1
        
        msg.position = Point(x=position[0], y=position[1], z=position[2])
        msg.velocity = Vector3(x=velocity[0], y=velocity[1], z=velocity[2])
        msg.acceleration = Vector3(x=acceleration[0], y=acceleration[1], z=acceleration[2])
        msg.angular_velocity = Vector3(x=angular_velocity[0], y=angular_velocity[1], z=angular_velocity[2])
        msg.orientation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])
        
        return msg

    def get_trajectory_point_scaled(self, traj, t, speed_factor):
        data = traj['data']
        duration = traj['duration']
        dt = traj['dt']
        n_points = traj['n_points']
        
        t_scaled = t * speed_factor
        t_scaled = np.clip(t_scaled, 0, duration)
        
        idx = t_scaled / dt
        i0 = int(idx)
        i1 = min(i0 + 1, n_points - 1)
        alpha = idx - i0
        
        row = (1 - alpha) * data[i0] + alpha * data[i1]
        
        position = row[1:4]
        velocity = row[4:7] * speed_factor
        acceleration = row[7:10] * (speed_factor ** 2)
        angular_velocity = row[10:13] * speed_factor
        quat = row[13:17]
        quat_norm = np.linalg.norm(quat)
        if quat_norm > 1e-6:
            quat /= quat_norm
        
        return position, velocity, acceleration, angular_velocity, quat

    def update_phase(self):
        """更新飞行阶段状态机"""
        now = rospy.Time.now()
        
        # IDLE
        if self.phase == FlightPhase.IDLE:
            if self.current_position is not None:
                if self.num_random_waypoints > 0 and len(self.random_waypoints) > 0:
                    self.phase = FlightPhase.GOTO_WAYPOINT
                    self.current_waypoint_idx = 0
                    rospy.loginfo("=" * 50)
                    rospy.loginfo("Starting random waypoints phase")
                    rospy.loginfo("=" * 50)
                elif len(self.trajectories) > 0:
                    self._start_trajectory_phase()
                else:
                    self._enter_final_hold()
        
        # GOTO_WAYPOINT
        elif self.phase == FlightPhase.GOTO_WAYPOINT:
            if self.current_waypoint_idx >= len(self.random_waypoints):
                if len(self.trajectories) > 0:
                    self._start_trajectory_phase()
                else:
                    self._enter_final_hold()
            else:
                wp = self.random_waypoints[self.current_waypoint_idx]
                dist = self.get_distance_to(wp['position'])
                
                if dist < self.waypoint_threshold:
                    self.phase = FlightPhase.HOVERING_AT_WAYPOINT
                    self.hover_start_time = now
                    self.hover_target_position = wp['position'].copy()
                    self.hover_target_quat = wp['quat'].copy()
                    
                    rospy.loginfo("=" * 50)
                    rospy.loginfo(f"✓ Reached waypoint {self.current_waypoint_idx + 1}/{len(self.random_waypoints)}")
                    rospy.loginfo(f"  Position: [{wp['position'][0]:.2f}, {wp['position'][1]:.2f}, {wp['position'][2]:.2f}]")
                    rospy.loginfo(f"  Hovering for {self.waypoint_hold_time:.1f}s...")
                    rospy.loginfo("=" * 50)
                else:
                    rospy.loginfo_throttle(1.0, 
                        f"Going to waypoint {self.current_waypoint_idx + 1}, dist={dist:.3f}m")
        
        # HOVERING_AT_WAYPOINT
        elif self.phase == FlightPhase.HOVERING_AT_WAYPOINT:
            elapsed = (now - self.hover_start_time).to_sec()
            remaining = self.waypoint_hold_time - elapsed
            
            if remaining <= 0:
                self.current_waypoint_idx += 1
                self.hover_start_time = None
                
                if self.current_waypoint_idx < len(self.random_waypoints):
                    self.phase = FlightPhase.GOTO_WAYPOINT
                    rospy.loginfo(f"Moving to waypoint {self.current_waypoint_idx + 1}")
                else:
                    if len(self.trajectories) > 0:
                        self._start_trajectory_phase()
                    else:
                        self._enter_final_hold()
            else:
                rospy.loginfo_throttle(1.0, f"Hovering... {remaining:.1f}s remaining")
        
        # GOTO_TRAJ_START
        elif self.phase == FlightPhase.GOTO_TRAJ_START:
            if self.current_traj_idx >= len(self.trajectories):
                self._enter_final_hold()
            else:
                traj = self.trajectories[self.current_traj_idx]
                dist = self.get_distance_to(traj['start_pos'])
                
                rospy.loginfo_throttle(1.0, 
                    f"Going to {traj['name']} start | "
                    f"Pos: [{self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}] | "
                    f"Target: [{traj['start_pos'][0]:.2f}, {traj['start_pos'][1]:.2f}, {traj['start_pos'][2]:.2f}] | "
                    f"dist={dist:.3f}m")
                
                if dist < self.start_threshold:
                    self.phase = FlightPhase.HOVERING_AT_TRAJ_START
                    self.traj_start_hold_start_time = now
                    self.hover_target_position = traj['start_pos'].copy()
                    self.hover_target_quat = traj['start_quat'].copy()
                    
                    rospy.loginfo("=" * 50)
                    rospy.loginfo(f"✓ Reached {traj['name']} start")
                    rospy.loginfo(f"  Hovering for {self.traj_start_hold_time:.1f}s before tracking...")
                    rospy.loginfo("=" * 50)
        
        # HOVERING_AT_TRAJ_START
        elif self.phase == FlightPhase.HOVERING_AT_TRAJ_START:
            elapsed = (now - self.traj_start_hold_start_time).to_sec()
            remaining = self.traj_start_hold_time - elapsed
            
            if remaining <= 0:
                self._start_lap()
            else:
                rospy.loginfo_throttle(0.5, f"Stabilizing... {remaining:.1f}s")
        
        # TRACKING_TRAJECTORY
        elif self.phase == FlightPhase.TRACKING_TRAJECTORY:
            traj = self.trajectories[self.current_traj_idx]
            t = (now - self.traj_start_time).to_sec()
            scaled_duration = self.get_scaled_duration(traj['duration'])
            
            progress = min(100.0, (t / scaled_duration) * 100)
            rospy.loginfo_throttle(1.0, 
                f"[{traj['name']}] Lap {self.current_lap + 1}/{self.num_laps} @ {self.current_speed_factor:.2f}x | "
                f"{progress:.0f}% ({t:.1f}/{scaled_duration:.1f}s)")
            
            if t >= scaled_duration:
                self._complete_lap()
        
        # LAP_TRANSITION
        elif self.phase == FlightPhase.LAP_TRANSITION:
            elapsed = (now - self.lap_transition_start_time).to_sec()
            
            if elapsed >= self.lap_transition_time:
                self._start_lap()
            else:
                remaining = self.lap_transition_time - elapsed
                rospy.loginfo_throttle(0.5, f"Lap transition... {remaining:.1f}s")
        
        # HOLDING_FINAL
        elif self.phase == FlightPhase.HOLDING_FINAL:
            if self.final_hold_time > 0:
                elapsed = (now - self.final_hold_start_time).to_sec()
                remaining = self.final_hold_time - elapsed
                
                if remaining <= 0:
                    self.phase = FlightPhase.FINISHED
                    rospy.loginfo("🏁 Mission complete!")
                else:
                    rospy.loginfo_throttle(2.0, f"Final hold... {remaining:.1f}s remaining")
            else:
                rospy.loginfo_throttle(5.0, "Holding final position")
        
        # FINISHED
        elif self.phase == FlightPhase.FINISHED:
            rospy.loginfo_throttle(5.0, "Mission finished!")

    def _enter_final_hold(self):
        """进入最终悬停"""
        now = rospy.Time.now()
        
        if self.current_position is not None:
            self.final_hold_position = self.current_position.copy()
            self.final_hold_quat = self.current_quat.copy()
        elif len(self.trajectories) > 0:
            last_traj = self.trajectories[-1]
            self.final_hold_position = last_traj['end_pos'].copy()
            self.final_hold_quat = last_traj['end_quat'].copy()
        else:
            self.final_hold_position = np.array([0.0, 0.0, 1.0])
            self.final_hold_quat = np.array([1.0, 0.0, 0.0, 0.0])
        
        self.final_hold_start_time = now
        self.phase = FlightPhase.HOLDING_FINAL
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("🛬 Entering final hold")
        rospy.loginfo(f"   Position: [{self.final_hold_position[0]:.2f}, "
                     f"{self.final_hold_position[1]:.2f}, {self.final_hold_position[2]:.2f}]")
        rospy.loginfo("=" * 50)

    def _start_trajectory_phase(self):
        """开始轨迹阶段"""
        self.current_traj_idx = 0
        self.current_lap = 0
        self.current_speed_factor = self.initial_speed_factor
        self.phase = FlightPhase.GOTO_TRAJ_START
        
        traj = self.trajectories[0]
        rospy.loginfo("=" * 50)
        rospy.loginfo("Starting trajectory phase")
        rospy.loginfo(f"Trajectory 1/{len(self.trajectories)}: {traj['name']}")
        rospy.loginfo(f"  Start: [{traj['start_pos'][0]:.2f}, {traj['start_pos'][1]:.2f}, {traj['start_pos'][2]:.2f}]")
        rospy.loginfo("=" * 50)

    def _start_lap(self):
        """开始新的一圈"""
        now = rospy.Time.now()
        traj = self.trajectories[self.current_traj_idx]
        
        self.current_speed_factor = self.get_speed_factor_for_lap(self.current_lap)
        self.traj_start_time = now
        self.phase = FlightPhase.TRACKING_TRAJECTORY
        
        scaled_duration = self.get_scaled_duration(traj['duration'])
        
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"🚀 Starting Lap {self.current_lap + 1}/{self.num_laps}")
        rospy.loginfo(f"   Trajectory: {traj['name']}")
        rospy.loginfo(f"   Speed: {self.current_speed_factor:.2f}x")
        rospy.loginfo(f"   Duration: {scaled_duration:.2f}s")
        rospy.loginfo("=" * 50)

    def _complete_lap(self):
        """完成一圈"""
        self.current_lap += 1
        traj = self.trajectories[self.current_traj_idx]
        
        rospy.loginfo(f"✓ Lap {self.current_lap}/{self.num_laps} completed @ {self.current_speed_factor:.2f}x")
        
        if self.current_lap >= self.num_laps:
            rospy.loginfo(f"✓ All {self.num_laps} laps of {traj['name']} completed!")
            
            self.current_traj_idx += 1
            self.current_lap = 0
            
            if self.current_traj_idx >= len(self.trajectories):
                if self.loop:
                    rospy.loginfo("Looping: Restarting")
                    self.current_waypoint_idx = 0
                    if self.num_random_waypoints > 0 and len(self.random_waypoints) > 0:
                        self.phase = FlightPhase.GOTO_WAYPOINT
                    elif len(self.trajectories) > 0:
                        self._start_trajectory_phase()
                    else:
                        self._enter_final_hold()
                else:
                    self._enter_final_hold()
            else:
                next_traj = self.trajectories[self.current_traj_idx]
                rospy.loginfo(f"Moving to trajectory {self.current_traj_idx + 1}/{len(self.trajectories)}: {next_traj['name']}")
                rospy.loginfo(f"  Start: [{next_traj['start_pos'][0]:.2f}, {next_traj['start_pos'][1]:.2f}, {next_traj['start_pos'][2]:.2f}]")
                self.phase = FlightPhase.GOTO_TRAJ_START
        else:
            self.phase = FlightPhase.LAP_TRANSITION
            self.lap_transition_start_time = rospy.Time.now()
            
            next_speed = self.get_speed_factor_for_lap(self.current_lap)
            rospy.loginfo(f"Preparing for lap {self.current_lap + 1} @ {next_speed:.2f}x")

    def get_current_target_msg(self):
        """获取当前目标消息"""
        now = rospy.Time.now()
        
        if self.phase == FlightPhase.IDLE:
            return self._get_hold_msg()
        
        elif self.phase == FlightPhase.GOTO_WAYPOINT:
            if self.current_waypoint_idx < len(self.random_waypoints):
                wp = self.random_waypoints[self.current_waypoint_idx]
                return self.build_trajectory_msg(
                    wp['position'], np.zeros(3), np.zeros(3), np.zeros(3), wp['quat']
                )
            return self._get_hold_msg()
        
        elif self.phase == FlightPhase.HOVERING_AT_WAYPOINT:
            if self.hover_target_position is not None:
                return self.build_trajectory_msg(
                    self.hover_target_position, np.zeros(3), np.zeros(3), np.zeros(3),
                    self.hover_target_quat
                )
            return self._get_hold_msg()
        
        elif self.phase == FlightPhase.GOTO_TRAJ_START:
            if self.current_traj_idx < len(self.trajectories):
                traj = self.trajectories[self.current_traj_idx]
                return self.build_trajectory_msg(
                    traj['start_pos'], np.zeros(3), np.zeros(3), np.zeros(3),
                    traj['start_quat']
                )
            return self._get_hold_msg()
        
        elif self.phase == FlightPhase.HOVERING_AT_TRAJ_START:
            if self.hover_target_position is not None:
                return self.build_trajectory_msg(
                    self.hover_target_position, np.zeros(3), np.zeros(3), np.zeros(3),
                    self.hover_target_quat
                )
            return self._get_hold_msg()
        
        elif self.phase == FlightPhase.TRACKING_TRAJECTORY:
            if self.current_traj_idx < len(self.trajectories):
                traj = self.trajectories[self.current_traj_idx]
                t = (now - self.traj_start_time).to_sec()
                scaled_duration = self.get_scaled_duration(traj['duration'])
                t = np.clip(t, 0, scaled_duration)
                
                pos, vel, acc, ang_vel, quat = self.get_trajectory_point_scaled(
                    traj, t, self.current_speed_factor
                )
                return self.build_trajectory_msg(pos, vel, acc, ang_vel, quat)
            return self._get_hold_msg()
        
        elif self.phase == FlightPhase.LAP_TRANSITION:
            if self.current_traj_idx < len(self.trajectories):
                traj = self.trajectories[self.current_traj_idx]
                return self.build_trajectory_msg(
                    traj['start_pos'], np.zeros(3), np.zeros(3), np.zeros(3),
                    traj['start_quat']
                )
            return self._get_hold_msg()
        
        elif self.phase in [FlightPhase.HOLDING_FINAL, FlightPhase.FINISHED]:
            if self.final_hold_position is not None:
                return self.build_trajectory_msg(
                    self.final_hold_position, np.zeros(3), np.zeros(3), np.zeros(3),
                    self.final_hold_quat
                )
            return self._get_hold_msg()
        
        else:
            return self._get_hold_msg()

    def _get_hold_msg(self):
        """获取悬停消息"""
        if self.current_position is not None and self.current_quat is not None:
            return self.build_trajectory_msg(
                self.current_position.copy(), np.zeros(3), np.zeros(3), np.zeros(3),
                self.current_quat.copy()
            )
        return self.build_trajectory_msg(
            np.array([0.0, 0.0, 1.0]), np.zeros(3), np.zeros(3), np.zeros(3),
            np.array([1.0, 0.0, 0.0, 0.0])
        )

    def run(self):
        rate = rospy.Rate(self.rate)
        counter = 0
        
        rospy.loginfo("Trajectory publisher running...")
        
        while not rospy.is_shutdown():
            self.update_phase()
            
            self.phase_pub.publish(self.phase.name)
            self.speed_factor_pub.publish(Float32(self.current_speed_factor))
            
            lap_info = f"Lap {self.current_lap + 1}/{self.num_laps} @ {self.current_speed_factor:.2f}x"
            self.lap_pub.publish(lap_info)
            
            msg = self.get_current_target_msg()
            self.pub.publish(msg)
            
            counter += 1
            if counter >= 100:
                self.ref_path_msg.header.stamp = rospy.Time.now()
                self.ref_path_pub.publish(self.ref_path_msg)
                self._publish_waypoints_visualization()
                counter = 0
            
            rate.sleep()


if __name__ == '__main__':
    try:
        node = TrajectoryPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())