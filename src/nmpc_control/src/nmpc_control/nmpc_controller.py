# 四旋翼NMPC控制器 20251124 SiweiZheng
#nmpc_controller.py
# ACADOS NMPC
from acados_template import AcadosOcp, AcadosOcpSolver
from nmpc_control.export_model import export_model
import numpy as np
import scipy.linalg
from os.path import dirname, join, abspath
from nmpc_control.utils import config
import time
import math
import casadi as ca
# np.set_printoptions(precision=3)  # 设置精度
np.set_printoptions(suppress=True)  # 禁用科学计数法输出

# ACADOS NMPC控制器
class NMPC_Controller:
    def __init__(self):
        self.ocp = AcadosOcp()       # OCP 优化问题
        self.model = export_model()  # 导出四旋翼物理模型

        self.Tf = 0.35                      # 预测时间长度(s) - 恢复原值
        self.N = 35                         # 预测步数 - 恢复原值
        self.nx = self.model.x.size()[0]    # 状态维度 13维度
        self.nu = self.model.u.size()[0]    # 控制输入维度 4维度
        self.ny = self.nx + self.nu         # 评估维度
        self.ny_e = self.nx                 # 终端评估维度

        # set ocp_nlp_dimensions
        self.nlp_dims     = self.ocp.dims
        self.nlp_dims.N   = self.N

        # parameters
        self.g0  = config.GRAVITY   # [m.s^2] accerelation of gravity
        self.mq  = config.MASS    # [kg] total mass (with one marker)
        self.Ct  = config.CT  # [N/rad/s2] Thrust coef
        self.Cd = config.CD          # 电机反扭系数 (Nm/krpm^2)
        self.CR = config.CR
        self.Wb = config.WB
        self.At = config.At
        self.Bt = config.Bt
        self.max_thrust =config.MAX_THRUST
        self.max_wx = config.MAX_WX
        self.max_wy = config.MAX_WY
        self.max_wz = config.MAX_WZ
        self.max_ep = 10
        self.max_ev = 30
        self.use_q0_positive = True
        # self.hov_w = (np.sqrt((self.mq*self.g0)/(4*self.Ct))-self.Wb)/self.CR  # 悬停时单个电机转速rad/s
        self.hov_w = (self.mq*self.g0)  # 悬停时单个电机转速rad/s
        # self.max_F = 4*self.Ct*(self.CR+self.Wb)**2
        self.max_F = 4*(self.At+self.Bt)*0.85
        self.minF = 4*(-self.Bt)*0.2
        # print(f"CT1: {CT1} N/Krpm^2")
        print(f"hovor thrust: {(self.hov_w/4-self.Bt)/self.At} ")
        Q = np.eye(self.nx)
        # Q[0,0] = 10      # x
        # Q[1,1] = 10      # y
        # Q[2,2] = 30      # z
        # Q[3,3] = 0        # qw
        # Q[4,4] = 0      # qx
        # Q[5,5] = 0     # qy
        # Q[6,6] = 0       # qz
        # Q[7,7] = 1        # vbx
        # Q[8,8] = 1        # vby
        # Q[9,9] = 3      # vbz

        # 保守优化：高位置权重 + 低速度权重
        # Q[0,0] = 5.0   # x - 高位置权重
        # Q[1,1] = 5.0   # y - 高位置权重
        # Q[2,2] = 5.0   # z - 最高位置权重
        # Q[3,3] = 1.0    # qw - 姿态权重
        # Q[4,4] = 1.0    # qx
        # Q[5,5] = 1.0    # qy
        # Q[6,6] = 1.0    # qz
        # Q[7,7] = 0.05    # vbx - 低速度权重（辅助）
        # Q[8,8] = 0.05    # vby
        # Q[9,9] = 0.05    # vbz
        # # ========================================================================
        # NMPC 权重矩阵配置（经 Optuna 优化 + 手动微调）
        # ========================================================================
        # 设计思路：
        # - 高位置权重（48.67）：优先保证轨迹跟踪精度
        # - 中等姿态权重（5.0）：防止过度倾斜，保持飞行稳定性（原值 3.34 偏低）
        # - 低速度权重（0.83）：辅助位置收敛，不过度约束
        # ========================================================================
        Q[0,0] = 48.6657624530981   # x - 位置权重（Optuna 优化值）
        Q[1,1] = 48.6657624530981   # y
        Q[2,2] = 48.6657624530981   # z
        Q[3,3] = 5.0    # qw - 姿态权重（手动提高：3.34 → 5.0，改善姿态响应）
        Q[4,4] = 5.0    # qx
        Q[5,5] = 5.0    # qy
        Q[6,6] = 5.0    # qz
        Q[7,7] = 0.8328259261326867    # vx - 速度权重（Optuna 优化值）
        Q[8,8] = 0.8328259261326867    # vy
        Q[9,9] = 0.8328259261326867    # vz


        R = np.eye(self.nu)   # 控制输入权重矩阵 - 降低以允许更快响应
        R[0,0] = 0.09   # F_total - 降低推力惩罚
        R[1,1] = 0.06   # wx - 降低角速度惩罚  
        R[2,2] = 0.06   # wy
        R[3,3] = 0.06   # wz

        self.ocp.cost.W = scipy.linalg.block_diag(Q, R)

        Vx = np.zeros((self.ny, self.nx))
        Vx[0,0] = 1.0
        Vx[1,1] = 1.0
        Vx[2,2] = 1.0
        Vx[3,3] = 1.0
        Vx[4,4] = 1.0
        Vx[5,5] = 1.0
        Vx[6,6] = 1.0
        Vx[7,7] = 1.0
        Vx[8,8] = 1.0
        Vx[9,9] = 1.0
        self.ocp.cost.Vx = Vx

        Vu = np.zeros((self.ny, self.nu))
        Vu[10,0] = 1.0
        Vu[11,1] = 1.0
        Vu[12,2] = 1.0
        Vu[13,3] = 1.0
        self.ocp.cost.Vu = Vu

        self.ocp.cost.W_e = 1.0 * Q  # 终端权重×3，引导预测末端收敛

        Vx_e = np.zeros((self.ny_e, self.nx))
        Vx_e[0,0] = 1.0
        Vx_e[1,1] = 1.0
        Vx_e[2,2] = 1.0
        Vx_e[3,3] = 1.0
        Vx_e[4,4] = 1.0
        Vx_e[5,5] = 1.0
        Vx_e[6,6] = 1.0
        Vx_e[7,7] = 1.0
        Vx_e[8,8] = 1.0
        Vx_e[9,9] = 1.0
        self.ocp.cost.Vx_e = Vx_e

        # 过程参考向量(状态+输入)
        self.ocp.cost.yref   = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.hov_w, 0, 0, 0])
        # 终端参考向量(状态)
        self.ocp.cost.yref_e = np.array([0.0, 0.0, 0.0, 1.0, 0, 0, 0, 0, 0, 0])

        # 构建约束
        # self.ocp.constraints.idxbx = np.array([0,1,2,3,4,5,6,7,8,9])
        # self.ocp.constraints.lbx = np.array([-self.max_ep, -self.max_ep, -self.max_ep, 
        #                                      -1.0, -1.0, -1.0, -1.0, 
        #                                      -self.max_ev, -self.max_ev, -self.max_ev])  # 电机最低转速输入
        # self.ocp.constraints.ubx = np.array([self.max_ep, self.max_ep, self.max_ep, 
        #                                      1.0, 1.0, 1.0, 1.0, 
        #                                      self.max_ev, self.max_ev, self.max_ev])  # 电机最低转速输入
        self.ocp.constraints.lbu = np.array([self.minF, -self.max_wx,-self.max_wy,-self.max_wz])  # 电机最低转速输入
        self.ocp.constraints.ubu = np.array([self.max_F,+self.max_wx,+self.max_wy,+self.max_wz])  # 电机最高转速输入
        self.ocp.constraints.x0  = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 初始状态
        self.ocp.constraints.idxbu = np.array([0, 1, 2, 3])  # 所有电机转速参与评估

        # ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
        # self.ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'  
        self.ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'  
        self.ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        self.ocp.solver_options.integrator_type = 'ERK'
        self.ocp.solver_options.print_level = 0

        # set prediction horizon
        self.ocp.solver_options.tf = self.Tf
        self.ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # 显然更快 ~100Hz
        # self.ocp.solver_options.nlp_solver_type = 'SQP'  # ~10Hz

        self.ocp.model = self.model  # 传入模型

        # ========================================================================
        # 构建编译 OCP 求解器（添加异常处理）
        # ========================================================================
        try:
            import rospy
            rospy.loginfo("正在初始化 ACADOS NMPC 求解器...")
            self.acados_solver = AcadosOcpSolver(self.ocp, json_file='acados_ocp.json')
            rospy.loginfo("✅ ACADOS 求解器初始化成功")
        except ImportError:
            # 非 ROS 环境（如单元测试）
            print("[NMPC] 正在初始化 ACADOS 求解器...")
            self.acados_solver = AcadosOcpSolver(self.ocp, json_file='acados_ocp.json')
            print("[NMPC] ✅ 求解器初始化成功")
        except Exception as e:
            error_msg = f"❌ ACADOS 求解器初始化失败: {e}\n" \
                        f"可能原因：\n" \
                        f"  1. ACADOS 未正确安装或编译\n" \
                        f"  2. c_generated_code 目录不存在或损坏\n" \
                        f"  3. 模型定义有误\n" \
                        f"解决方法：\n" \
                        f"  - 检查 ACADOS 安装: export ACADOS_SOURCE_DIR=/path/to/acados\n" \
                        f"  - 重新生成代码: python3 export_model.py"
            try:
                import rospy
                rospy.logerr(error_msg)
            except ImportError:
                print(error_msg)
            raise RuntimeError(f"NMPC Controller 初始化失败") from e
        
        # 初始化权重缓存
        self.current_Q = Q
        self.current_R = R
        
        print("NMPC Controller Init Done")
        
    def update_weights(self, Q_pos=None, Q_pos_xy=None, Q_pos_z=None, Q_att=None, Q_vel=None, Q_vel_xy=None, Q_vel_z=None):
        """
        动态更新权重矩阵
        
        Args:
            Q_pos: 位置权重 (XYZ统一，向后兼容)
            Q_pos_xy: XY位置权重 (优先于Q_pos)
            Q_pos_z: Z位置权重 (优先于Q_pos)
            Q_att: 姿态权重 (四元数)
            Q_vel: 速度权重 (XYZ统一，向后兼容)
            Q_vel_xy: XY速度权重 (优先于Q_vel)
            Q_vel_z: Z速度权重 (优先于Q_vel)
        """
        try:
            # 基础权重 (必须与__init__中的维度一致)
            Q = self.current_Q.copy()
            
            # 更新Q矩阵对角线元素
            if Q_pos is not None:
                Q[0,0] = Q[1,1] = Q[2,2] = Q_pos
            
            # XY/Z 独立设置（优先级高于 Q_pos）
            if Q_pos_xy is not None:
                Q[0,0] = Q[1,1] = Q_pos_xy
            if Q_pos_z is not None:
                Q[2,2] = Q_pos_z
            
            if Q_att is not None:
                Q[3,3] = Q[4,4] = Q[5,5] = Q[6,6] = Q_att
                
            if Q_vel is not None:
                Q[7,7] = Q[8,8] = Q[9,9] = Q_vel
            
            # XY/Z 速度独立设置（优先级高于 Q_vel）
            if Q_vel_xy is not None:
                Q[7,7] = Q[8,8] = Q_vel_xy
            if Q_vel_z is not None:
                Q[9,9] = Q_vel_z
                
            # 构建完整的W矩阵 (Q + R)
            W = scipy.linalg.block_diag(Q, self.current_R)
            
            # 更新过程权重 (stage costs)
            for i in range(self.N):
                self.acados_solver.cost_set(i, 'W', W)
            
            # 更新终端权重 (terminal cost)
            # 注意：ACADOS中动态更新W_e可能导致Crash (field W_e not available)，暂时禁用
            # W_e = Q.copy() 
            # try:
            #     self.acados_solver.cost_set(self.N, 'W_e', W_e)
            # except Exception as e:
            #     pass 
                
            # 更新缓存
            self.current_Q = Q
            # print(f"Weights Updated: Q_pos={Q[0,0]}, Q_vel={Q[7,7]}")
            return True
            
        except Exception as e:
            print(f"[NMPC] Weight update failed: {e}")
            return False
    def _quat_same_hemisphere(self, q_ref, q_cur):
        # q_ref, q_cur: shape (4,)
        if np.dot(q_ref, q_cur) < 0.0:
            return -q_ref
        return q_ref

    def _quat_normalize(self, q):
        n = np.linalg.norm(q)
        if n < 1e-9:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / n

    # 状态空间位点控制
    # current_state当前状态: [x, y, z, qw, qx, qy, qz, vbx, vby, vbz] 
    # goal_state目标状态:    [x, y, z, qw, qx, qy, qz, vbx, vby, vbz] 
    def nmpc_state_control(self, current_state, goal_state):
        _start = time.perf_counter()
        # Set initial condition, equality constraint
        #current_state = self._sanitize_quat_in_state(current_state.copy())
        goal_state[3:7] = self._quat_same_hemisphere(goal_state[3:7], current_state[3:7])
        self.acados_solver.set(0, 'lbx', current_state)
        self.acados_solver.set(0, 'ubx', current_state)

        y_ref = np.concatenate((goal_state, np.array([self.hov_w, 0, 0, 0])))
        # Set Goal State
        for i in range(self.N):
            self.acados_solver.set(i, 'yref', y_ref)   # 过程参考
        y_refN = goal_state 
        self.acados_solver.set(self.N, 'yref', y_refN)   # 终端参考

        # Solve Problem
        self.acados_solver.solve()
        # Get Solution
        w_opt_acados = np.ndarray((self.N, 4))  # 控制输入
        x_opt_acados = np.ndarray((self.N + 1, len(current_state)))   # 状态估计
        x_opt_acados[0, :] = self.acados_solver.get(0, "x")
        for i in range(self.N):
            w_opt_acados[i, :] = self.acados_solver.get(i, "u")
            x_opt_acados[i + 1, :] = self.acados_solver.get(i + 1, "x")
        # return w_opt_acados, x_opt_acados  # 返回控制输入和状态
        _end = time.perf_counter()
        _dt = _end - _start
        return _dt, w_opt_acados[0],x_opt_acados  # 返回最近控制输入 4 Vector
        # control_input = self.acados_solver.get(0, "u")
        # state_estimate = self.acados_solver.get(self.N, "x")
        # return control_input, state_estimate  # 返回所有控制输入和状态

    # ========================================================================
    # 轨迹跟踪控制 - 支持未来轨迹预览 (核心改进)
    # ========================================================================
    # ========================================================================
    # 轨迹跟踪控制 - 支持未来轨迹预览 (核心改进)
    # ========================================================================
    def nmpc_trajectory_control(self, current_state, trajectory_refs, yaw_dot_ref=0.0):
        """
        使用未来轨迹预览的NMPC控制
        
        与nmpc_state_control的区别：每个预测步骤使用不同的参考状态，
        使NMPC能够"看到"未来轨迹并提前优化控制。
        
        Args:
            current_state: 当前状态 (10,) [x,y,z,qw,qx,qy,qz,vx,vy,vz]
            trajectory_refs: 未来N+1步参考轨迹 (N+1, 10)
                trajectory_refs[i] = t + i*dt 时刻的参考状态
            yaw_dot_ref: 参考偏航角速度 (rad/s), 默认为0
            
        Returns:
            _dt: 求解时间
            control_input: 最优控制输入 (4,) [F, wx, wy, wz]
            predicted_states: 预测状态轨迹 (N+1, 10)
        """
        _start = time.perf_counter()
        
        # 设置初始状态约束
        self.acados_solver.set(0, 'lbx', current_state)
        self.acados_solver.set(0, 'ubx', current_state)
        
        # 为每个预测步骤设置对应的参考状态
        for i in range(self.N):
            ref_state = trajectory_refs[min(i, len(trajectory_refs)-1)].copy()
            # 确保四元数在同一半球
            ref_state[3:7] = self._quat_same_hemisphere(ref_state[3:7], current_state[3:7])
            # 构建完整参考向量 (状态 + 控制输入参考)
            # 使用 yaw_dot_ref 作为 wz 的参考值
            y_ref = np.concatenate((ref_state, np.array([self.hov_w, 0, 0, yaw_dot_ref])))
            self.acados_solver.set(i, 'yref', y_ref)
        
        # 设置终端参考 (使用最后一个点)
        ref_state_N = trajectory_refs[min(self.N, len(trajectory_refs)-1)].copy()
        ref_state_N[3:7] = self._quat_same_hemisphere(ref_state_N[3:7], current_state[3:7])
        self.acados_solver.set(self.N, 'yref', ref_state_N)
        
        # 求解
        self.acados_solver.solve()
        
        # 获取解
        w_opt_acados = np.ndarray((self.N, 4))
        x_opt_acados = np.ndarray((self.N + 1, len(current_state)))
        x_opt_acados[0, :] = self.acados_solver.get(0, "x")
        for i in range(self.N):
            w_opt_acados[i, :] = self.acados_solver.get(i, "u")
            x_opt_acados[i + 1, :] = self.acados_solver.get(i + 1, "x")
        
        _end = time.perf_counter()
        _dt = _end - _start
        return _dt, w_opt_acados[0], x_opt_acados

    # NMPC位置控制
    # goal_pos: 目标三维位置[x y z]
    def nmpc_position_control(self, current_state, goal_pos):
        goal_state = np.array([goal_pos[0], goal_pos[1], goal_pos[2], 1 ,0 ,0, 0, 0.0, 0.0, 0.0])
        _dt, control = self.nmpc_state_control(current_state, goal_state)
        #print(_dt,control)
        return _dt, control

# TEST
if __name__ == '__main__':
    print("ACADOS NMPC TEST")
    nmpc_controller = NMPC_Controller()

    current_state = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    goal_state =    np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for i in range(1000):
        _dt, w,_ = nmpc_controller.nmpc_state_control(current_state, goal_state)
        print(f"Iteration {i+1}: Control Input: {w}")

    # print("Control Input:")
    # print(w)
    # print("State Estimation:")
    # print(x)
