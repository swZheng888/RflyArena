#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
RL 轨迹跟踪控制节点 — RK3568 NPU (RKNN) 加速版

复用 control_rl_node.ControlRLNode 的全部状态机 / 起飞 / 观测构建 / 滤波 / 仪表盘逻辑,
只把后端从 onnxruntime CPU 换成 rknn_toolkit_lite2 在 RK3568 NPU 上推理。

依赖 (运行端,部署到 RK3568 板卡):
  pip3 install rknn-toolkit-lite2          # 板端轻量 runtime, 加载 .rknn 模型并调 NPU
  # 或对应板卡 BSP 自带的 librknnrt.so

模型由 scripts/convert_onnx_to_rknn.py 在 x86 开发机上离线转换得到 (rknn-toolkit2)。

新增 launch 参数 (其余参数与 control_rl_node 完全一致):
  model_path     (必须)  .rknn 模型路径 (RK3568 量化或浮点皆可)
  core_mask      ['auto'] 'auto' / 'npu_0' / 'npu_1' / 'npu_2' / 'all' — RK3568 单核 NPU
                          时通常用 'auto', 这里保留参数主要为了与 RK3588 共用脚本.
  warmup_iters   [20]     初始化时空跑次数,用于触发 NPU 调度器预热,
                          消除第一次推理 ~10ms 抖动.

发布话题保持不变 (含 /rl_control/solve_time_ms),便于与 ONNX/CPU 版本横向对比 NPU 推理耗时.
"""

import os
import sys
import time
import rospy
import numpy as np

# 复用原 ONNX 节点的所有非推理逻辑
from control_rl_node import ControlRLNode  # noqa: E402


# ===========================================================================
# RKNN 策略包装器 (板端 runtime)
# ===========================================================================
class RKNNPolicy:
    """加载 .rknn 模型并在 RK3568 NPU 上执行推理.

    与 control_rl_node.RLPolicy 接口完全一致 (predict(obs) -> action),
    保证可以无缝替换.
    """

    def __init__(self, model_path: str, core_mask: str = "auto", warmup_iters: int = 20):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"找不到 RKNN 模型文件: {model_path}")
        if not model_path.endswith(".rknn"):
            rospy.logwarn("[rl_control][rknn] 模型扩展名不是 .rknn (%s), 请确认已经过转换",
                          model_path)

        try:
            from rknnlite.api import RKNNLite
        except ImportError as e:
            rospy.logerr("[rl_control][rknn] 未安装 rknn-toolkit-lite2, 请执行:\n"
                         "  pip3 install rknn-toolkit-lite2\n"
                         "(板端需配套 librknnrt.so, RK3568 BSP 自带)")
            raise e

        self._rknn = RKNNLite()
        rospy.loginfo("[rl_control][rknn] 加载模型: %s", model_path)
        ret = self._rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError(f"RKNNLite.load_rknn 失败, 返回码={ret}")

        # RK3568 是单核 NPU, core_mask 实际不生效;
        # 保留参数为了和 RK3588 (NPU_CORE_0 / 1 / 2 / ALL) 脚本兼容.
        init_kwargs = {}
        cm = (core_mask or "auto").lower()
        if cm != "auto":
            try:
                mask_map = {
                    "npu_0": RKNNLite.NPU_CORE_0,
                    "npu_1": RKNNLite.NPU_CORE_1,
                    "npu_2": RKNNLite.NPU_CORE_2,
                    "all":   RKNNLite.NPU_CORE_0_1_2,
                }
                if cm in mask_map:
                    init_kwargs["core_mask"] = mask_map[cm]
                else:
                    rospy.logwarn("[rl_control][rknn] 未识别的 core_mask=%s, 走默认", core_mask)
            except AttributeError:
                # RK3568 上的 RKNNLite 可能没有 NPU_CORE_* 常量, 忽略
                rospy.logwarn("[rl_control][rknn] 当前 RKNNLite 不支持 core_mask, 忽略该参数")

        ret = self._rknn.init_runtime(**init_kwargs)
        if ret != 0:
            raise RuntimeError(f"RKNNLite.init_runtime 失败, 返回码={ret} "
                               f"(检查 librknnrt.so 是否存在 / NPU 驱动是否加载)")

        rospy.loginfo("[rl_control][rknn] NPU runtime 就绪, 开始预热 %d 次...", warmup_iters)
        dummy = np.zeros((1, 25), dtype=np.float32)
        t0 = time.perf_counter()
        for _ in range(max(1, warmup_iters)):
            self._rknn.inference(inputs=[dummy])
        warm_ms = (time.perf_counter() - t0) * 1000.0 / max(1, warmup_iters)
        rospy.loginfo("[rl_control][rknn] ✅ 预热完成, 平均 %.2f ms/次", warm_ms)

    def predict(self, obs: np.ndarray) -> np.ndarray:
        """obs (25,) -> action (4,) ∈ [-1, 1]"""
        x = obs.astype(np.float32).reshape(1, -1)
        out = self._rknn.inference(inputs=[x])[0]
        # RKNN 输出可能是 (1, 4) 也可能是 (4,) 视模型构建时配置而定
        out = np.asarray(out).reshape(-1)[:4]
        return np.clip(out, -1.0, 1.0)

    def release(self):
        try:
            if self._rknn is not None:
                self._rknn.release()
                self._rknn = None
        except Exception as e:
            rospy.logwarn("[rl_control][rknn] release 异常: %s", e)


# ===========================================================================
# RK3568 NPU 控制节点 (继承 ControlRLNode, 只替换推理后端)
# ===========================================================================
class ControlRLRKNNNode(ControlRLNode):

    def _init_params(self):
        super()._init_params()
        self.core_mask    = rospy.get_param("~core_mask",   "auto")
        self.warmup_iters = rospy.get_param("~warmup_iters", 20)

    def _init_policy(self):
        try:
            self.policy = RKNNPolicy(
                self.model_path,
                core_mask=self.core_mask,
                warmup_iters=int(self.warmup_iters),
            )
        except Exception as e:
            rospy.logerr("[rl_control][rknn] 初始化 NPU 策略失败: %s", e)
            sys.exit(1)


# ===========================================================================
if __name__ == "__main__":
    node = None
    try:
        node = ControlRLRKNNNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[rl_control][rknn] 节点被中断")
    except Exception as e:
        rospy.logerr("[rl_control][rknn] 程序异常: %s", e)
        raise
    finally:
        # 释放 NPU 句柄, 避免下次启动 init_runtime 报 busy
        if node is not None and hasattr(node, "policy") and isinstance(node.policy, RKNNPolicy):
            node.policy.release()
        rospy.loginfo("[rl_control][rknn] 程序退出")
