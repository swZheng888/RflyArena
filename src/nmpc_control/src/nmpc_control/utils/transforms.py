# -*- coding: utf-8 -*-
"""
transforms.py
坐标系转换 & 四元数运算（封装成类）。
"""

import math
import numpy as np

class FrameTransformer(object):
    """
    提供 NED/FRD <-> ENU/FLU 的各种转换工具。
    """

    @staticmethod
    def ned_to_enu_position(vec_ned):
        """
        NED -> ENU 位置转换: [x_n, y_n, z_n] -> [y_e, x_e, -z_e]
        """
        vec_ned = np.asarray(vec_ned)
        return np.array([vec_ned[1], vec_ned[0], -vec_ned[2]])

    @staticmethod
    def frd_to_flu_ang_vel(ang_vel_frd):
        """
        机体系 FRD -> FLU 角速度转换。
        """
        ang_vel_frd = np.asarray(ang_vel_frd)
        return np.array([ang_vel_frd[0], -ang_vel_frd[1], -ang_vel_frd[2]])
    @staticmethod
    def quat_inv(q):
        """
        计算四元数的逆。
        四元数格式：[w, x, y, z]
        """
        q = np.asarray(q)
        return np.array([q[0], -q[1], -q[2], -q[3]])  # 逆四元数为原四元数的实部不变，虚部取负

    @staticmethod
    def quat_mult(q1, q2):
        """
        Hamilton 四元数乘法：q = q1 * q2
        四元数格式：[w, x, y, z]
        """
        q1 = np.asarray(q1)
        q2 = np.asarray(q2)
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ])
    @staticmethod
    def frd_to_flu_rotation(quat_ned_frd):
        """
        FRD -> FLU 的姿态转换（绕 x 轴翻转 y/z）。
        """
        q_frd2flu = np.array([0.0, 1.0, 0.0, 0.0])  # FRD到FLU的固定转换四元数

        # 取逆四元数再进行乘法
        q_frd2flu_inv = FrameTransformer.quat_inv(q_frd2flu)

        # 返回 FRD 到 FLU 的转换
        return FrameTransformer.quat_mult(quat_ned_frd, q_frd2flu)
    @staticmethod
    def frd_to_flu_trans(w_frd):
        """
        常见约定：Odometry.twist.twist.angular 多在机体系 FRD (x前 y右 z下)
        控制指令 body_rate 更符合 FLU (x前 y左 z上)
        FRD -> FLU: [wx, -wy, -wz]
        """
        w_frd = np.asarray(w_frd, dtype=float).reshape(3)
        return np.array([w_frd[0], -w_frd[1], -w_frd[2]], dtype=float)

    @staticmethod
    def ned_to_enu_rotation(quat):
        """
        NED -> ENU 的姿态转换。
        """
        q_ned2enu = np.array([0.0, math.sqrt(2.0) / 2.0, math.sqrt(2.0) / 2.0, 0.0])
        return FrameTransformer.quat_mult(q_ned2enu, quat)

    @staticmethod
    def nedfrd_to_enuflu_rotation(quat_ned_frd):
        quat_ned_frd = np.asarray(quat_ned_frd)
        
        Q_ENU_NED = np.array([0.0, np.sqrt(2)/2, np.sqrt(2)/2, 0.0])
        Q_FRD_FLU = np.array([0.0, 1.0, 0.0, 0.0])
        
        temp = FrameTransformer.quat_mult(quat_ned_frd, Q_FRD_FLU)
        quat_enu_flu = FrameTransformer.quat_mult(Q_ENU_NED, temp)
        
        # 只做归一化，不强制符号
        return quat_enu_flu / np.linalg.norm(quat_enu_flu)
        
        
        return quat_enu_flu
    @staticmethod
    def rotate_vector_by_quat(vec, quat):
        """
        使用四元数将 3D 向量 vec 进行旋转。
        四元数格式：[w, x, y, z]
        约定：quat 表示从“局部坐标系”到“全局坐标系”的旋转。
        """
        vec = np.asarray(vec)
        quat = np.asarray(quat)

        # 向量当作纯虚四元数 [0, vx, vy, vz]
        vq = np.array([0.0, vec[0], vec[1], vec[2]])

        # q * v * q_conj
        q_conj = FrameTransformer.quat_inv(quat)
        vq_rot = FrameTransformer.quat_mult(
            FrameTransformer.quat_mult(quat, vq),
            q_conj
        )
        # 取出旋转后的三维向量部分
        return vq_rot[1:]

    @staticmethod
    def flu_to_enu_velocity(vel_flu, quat_enu_flu):
        """
        将机体 FLU 坐标系下的速度转换到 ENU 坐标系。

        参数
        ----
        vel_flu : array-like, shape (3,)
            机体系 FLU 下的速度 [vx_flu, vy_flu, vz_flu]
        quat_enu_flu : array-like, shape (4,)
            四元数 [w, x, y, z]，表示从 FLU -> ENU 的旋转
            （即“机体系姿态”，机体坐标系到世界 ENU 坐标系）。

        返回
        ----
        vel_enu : np.ndarray, shape (3,)
            ENU 坐标系下的速度向量。
        """
        vel_flu = np.asarray(vel_flu)
        quat_enu_flu = np.asarray(quat_enu_flu)

        return FrameTransformer.rotate_vector_by_quat(vel_flu, quat_enu_flu)
