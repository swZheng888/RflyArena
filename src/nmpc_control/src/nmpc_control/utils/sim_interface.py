# -*- coding: utf-8 -*-
"""
sim_interface.py
RflySim / PX4MavCtrl 接口封装。
"""

from __future__ import annotations

import numpy as np

from config import START_COPTER_ID
import PX4MavCtrlV4
import VisionCaptureApi
import ReqCopterSim


class SimInterface:
    """
    对 PX4MavCtrlV4 和 CopterSim 做一层简单封装。
    """

    def __init__(self) -> None:
        VisionCaptureApi.isEnableRosTrans = True

        # 初始化 MavCtrl
        self.mavCtrl = PX4MavCtrlV4.PX4MavCtrler()
        self.mavCtrl.InitTrueDataLoop()
        print("[SimInterface] true pose x:", self.mavCtrl.truePosNED[0])

        # CopterSim 相关
        self.req = ReqCopterSim.ReqCopterSim()
        self.start_copter_id = START_COPTER_ID
        self.target_ip = self.req.getSimIpID(self.start_copter_id)

        print("[SimInterface] Request CopterSim Send data. TARGET_IP =", self.target_ip)

        # 如需从 CopterSim 读取 IMU，可自行开启下面代码：
        # self.req.sendReSimIP(self.start_copter_id)
        # vis = VisionCaptureApi.VisionCaptureApi(self.target_ip)
        # vis.sendImuReqCopterSim(self.start_copter_id, self.target_ip)

    def get_true_state_ned_frd(self):
        """
        返回 (position_ned, velocity_ned, quat_ned_frd, ang_vel_frd)，全部为 numpy 数组。
        """
        pos_ned = np.array([
            self.mavCtrl.truePosNED[0],
            self.mavCtrl.truePosNED[1],
            self.mavCtrl.truePosNED[2],
        ])
        vel_ned = np.array([
            self.mavCtrl.trueVelNED[0],
            self.mavCtrl.trueVelNED[1],
            self.mavCtrl.trueVelNED[2],
        ])
        quat_ned_frd = np.array([
            self.mavCtrl.trueAngQuatern[0],
            self.mavCtrl.trueAngQuatern[1],
            self.mavCtrl.trueAngQuatern[2],
            self.mavCtrl.trueAngQuatern[3],
        ])
        ang_vel_frd = np.array([
            self.mavCtrl.trueAngRate[0],
            self.mavCtrl.trueAngRate[1],
            self.mavCtrl.trueAngRate[2],
        ])

        return pos_ned, vel_ned, quat_ned_frd, ang_vel_frd


# 默认创建一个全局实例，方便其他模块直接 import 使用
sim = SimInterface()
