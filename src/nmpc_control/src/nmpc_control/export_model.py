# 导出四旋翼物理模型（控制：油门 + 角速度指令） 20250304 Wakkk
#export_model.py
from acados_template import AcadosModel
from casadi import SX, vertcat
from nmpc_control.utils import config
import sys

# 按你原来的习惯保留这句
sys.path.append('/home/robot/acados/interfaces/acados_template')


def export_model():
    """
    四旋翼刚体动力学 + 油门/角速度控制接口模型（适配 PX4 AttitudeTarget body_rate + thrust）

    状态向量 x:
        0  px       世界坐标系位置 x
        1  py       世界坐标系位置 y
        2  pz       世界坐标系位置 z
        3  q0       四元数实部（w）
        4  q1       四元数虚部 x
        5  q2       四元数虚部 y
        6  q3       四元数虚部 z
        7  vx       世界坐标系速度 x
        8  vy       世界坐标系速度 y
        9  vz       世界坐标系速度 z

    控制输入 u:
        0  u_throttle   归一化油门, 对应 PX4 mavros_msgs::AttitudeTarget.thrust  (0~1)
        1  wx_cmd       期望滚转角速度 (rad/s), 对应 body_rate.x
        2  wy_cmd       期望俯仰角速度 (rad/s), 对应 body_rate.
        3  wz_cmd       期望偏航角速度 (rad/s), 对应 body_rate.z
    """
    # -------------------------------------------------------------------------
    # 模型名称
    # -------------------------------------------------------------------------
    model_name = 'crazyflie'  # 保持不变，方便直接替换你原来的模型

    # -------------------------------------------------------------------------
    # 物理参数
    # -------------------------------------------------------------------------
    g0 = config.GRAVITY        # [m/s^2] 重力加速度
    mass = config.MASS        # [kg] 总质量（可按实机调整）

    # 转动惯量（这里不直接用在动力学里，但先保留，后续如需要可恢复刚体力矩方程）
    Ixx = config.Ixx      # [kg·m^2]
    Iyy = config.Iyy       # [kg·m^2]
    Izz = config.Izz       # [kg·m^2]

    # 电机与桨叶参数（当前模型不使用，只保留备查）
    Ct = config.CT     # 电机推力系数 (N/krpm^2)
    Cd = config.CD     # 电机反扭系数 (Nm/krpm^2)
    dq = config.dq          # [m] 电机间距
    Cr = config.CR
    Wb =config.WB
    l = dq / 2.0       # [m] 电机到机体中心距离
    
    # 角速度内环时间常数（模拟 PX4 rate loop 的响应）
    tau_w = 0.01  # [s] 一阶惯性时间常数，大约 50ms，可视情况微调

    # -------------------------------------------------------------------------
    # 状态变量 x
    # -------------------------------------------------------------------------
    # 世界坐标系位置
    px = SX.sym('px')
    py = SX.sym('py')
    pz = SX.sym('pz')

    # 姿态四元数（w, x, y, z）
    q0 = SX.sym('q0')
    q1 = SX.sym('q1')
    q2 = SX.sym('q2')
    q3 = SX.sym('q3')

    # 世界坐标系线速度
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    vz = SX.sym('vz')


    # 状态向量
    x = vertcat(px, py, pz,
                q0, q1, q2, q3,
                vx, vy, vz)

    # -------------------------------------------------------------------------
    # 控制输入 u ：油门 + 角速度指令
    # -------------------------------------------------------------------------
    F = SX.sym('F')  # 归一化油门 [0,1]
    wx_cmd = SX.sym('wx_cmd')          # 期望滚转角速度 [rad/s]
    wy_cmd = SX.sym('wy_cmd')          # 期望俯仰角速度 [rad/s]
    wz_cmd = SX.sym('wz_cmd')          # 期望偏航角速度 [rad/s]

    u = vertcat(F, wx_cmd, wy_cmd, wz_cmd)

    # -------------------------------------------------------------------------
    # xdot: 状态导数（符号变量）
    # -------------------------------------------------------------------------
    px_dot = SX.sym('px_dot')
    py_dot = SX.sym('py_dot')
    pz_dot = SX.sym('pz_dot')

    q0_dot = SX.sym('q0_dot')
    q1_dot = SX.sym('q1_dot')
    q2_dot = SX.sym('q2_dot')
    q3_dot = SX.sym('q3_dot')

    vx_dot = SX.sym('vx_dot')
    vy_dot = SX.sym('vy_dot')
    vz_dot = SX.sym('vz_dot')


    xdot = vertcat(px_dot, py_dot, pz_dot,
                   q0_dot, q1_dot, q2_dot, q3_dot,
                   vx_dot, vy_dot, vz_dot,)

    # -------------------------------------------------------------------------
    # 平动动力学：位置和速度
    # -------------------------------------------------------------------------
    # 位置导数 = 线速度（世界系）
    px_d = vx
    py_d = vy
    pz_d = vz


    # T = 4*Ct*((Cr*u_th+Wb)*(Cr*u_th+Wb))
    thrust_acc_b = F/mass
    thrust_accx_w = 2.0 * (q1 * q3 + q0 * q2) * thrust_acc_b
    thrust_accy_w = 2.0 * (-q0 * q1 + q2 * q3) * thrust_acc_b
    thrust_accz_w = 2*(0.5-q1**2-q2**2) * thrust_acc_b

    # 空气阻力系数 (Air Drag) - 优化后的值
    # 经验值：0.15 适合大多数小型四旋翼（0.3kg级别）
    # 原值 0.35 过高，会导致高速飞行时阻力过大
    k_drag_x = 0.15
    k_drag_y = 0.15
    k_drag_z = 0.1  # Z轴也添加适当阻力，提高稳定性
    
    vx_d = thrust_accx_w - k_drag_x * vx
    vy_d = thrust_accy_w - k_drag_y * vy
    vz_d = thrust_accz_w - g0 - k_drag_z * vz
    q0_d = -0.5 * (q1 * wx_cmd + q2 * wy_cmd + q3 * wz_cmd)
    q1_d =  0.5 * (q0 * wx_cmd - q3 * wy_cmd + q2 * wz_cmd)
    q2_d =  0.5 * (q3 * wx_cmd + q0 * wy_cmd - q1 * wz_cmd)
    q3_d =  0.5 * (q1 * wy_cmd - q2 * wx_cmd + q0 * wz_cmd)


    # -------------------------------------------------------------------------
    # 显式/隐式形式
    # -------------------------------------------------------------------------
    f_expl = vertcat(
        px_d, py_d, pz_d,
        q0_d, q1_d, q2_d, q3_d,
        vx_d, vy_d, vz_d,
    )

    f_impl = xdot - f_expl

    # -------------------------------------------------------------------------
    # algebraic variables & parameters（本模型暂无）
    # -------------------------------------------------------------------------
    z = []   # 无代数变量
    p = []   # 无外部参数，如需要可扩展

    # -------------------------------------------------------------------------
    # 构建 AcadosModel
    # -------------------------------------------------------------------------
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name

    return model
