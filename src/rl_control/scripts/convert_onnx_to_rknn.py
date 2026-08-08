#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ONNX → RKNN 离线转换工具 (RK3568 NPU)

⚠️ 本脚本运行在 x86 开发机上, 不在飞控板/RK3568 板卡上运行.
   依赖: pip3 install rknn-toolkit2 (注意 toolkit2 与板端 toolkit-lite2 是两个包)

用法:
  # 纯浮点 (fp16) 转换 — 推理一致性最高, NPU 速度也已经远快于 CPU
  python3 convert_onnx_to_rknn.py \
      --onnx  ../model/policy.onnx \
      --rknn  ../model/policy_rk3568.rknn

  # 带 i8 量化 — 需提供 100~500 条真实/仿真观测做校准
  python3 convert_onnx_to_rknn.py \
      --onnx  ../model/policy.onnx \
      --rknn  ../model/policy_rk3568_i8.rknn \
      --quant --dataset calib_obs.txt

calib_obs.txt 格式: 每行一个 .npy 路径, 每个 .npy 文件 shape=(1,25), float32.
也支持直接放一个 .npy 路径, 用 --dataset_npy 一次传入 (N,25) 数组.

模型输入约定 (与 control_rl_node._build_obs 严格对齐): float32, shape=(1,25).
"""

import argparse
import os
import sys
import tempfile


def build_calib_dataset(npy_path: str) -> str:
    """把 (N,25) 的 .npy 拆成每行一个 npy 路径的 dataset.txt, 返回该文件路径"""
    import numpy as np
    arr = np.load(npy_path)
    assert arr.ndim == 2 and arr.shape[1] == 25, \
        f"校准数据 shape 必须是 (N,25), got {arr.shape}"
    tmp_dir = tempfile.mkdtemp(prefix="rknn_calib_")
    list_path = os.path.join(tmp_dir, "dataset.txt")
    with open(list_path, "w") as f:
        for i, row in enumerate(arr.astype("float32")):
            p = os.path.join(tmp_dir, f"obs_{i:05d}.npy")
            np.save(p, row.reshape(1, 25))
            f.write(p + "\n")
    print(f"[convert] 已生成校准 list: {list_path} ({arr.shape[0]} samples)")
    return list_path


def main():
    ap = argparse.ArgumentParser(description="ONNX → RKNN (RK3568) 转换")
    ap.add_argument("--onnx",    required=True, help="输入 ONNX 路径")
    ap.add_argument("--rknn",    required=True, help="输出 RKNN 路径")
    ap.add_argument("--target",  default="rk3568",
                    choices=["rk3566", "rk3568", "rk3588"],
                    help="目标平台 (默认 rk3568)")
    ap.add_argument("--quant",   action="store_true", help="启用 int8 量化")
    ap.add_argument("--dataset", default="",
                    help="量化校准 dataset.txt (每行一个 .npy 路径)")
    ap.add_argument("--dataset_npy", default="",
                    help="量化校准数据, 一个 (N,25) 的 .npy 文件 (自动拆成 dataset.txt)")
    ap.add_argument("--opt_level", type=int, default=3,
                    help="rknn.config optimization_level (默认 3)")
    ap.add_argument("--input_name",  default="obs",
                    help="ONNX 输入张量名 (默认 'obs', 训练导出脚本里写死)")
    ap.add_argument("--input_shape", default="1,25",
                    help="ONNX 输入固定形状 (逗号分隔, 默认 '1,25'). "
                         "因为训练导出带了 dynamic batch_size, RKNN 不支持动态形状, "
                         "这里把 batch 锁成 1.")
    args = ap.parse_args()

    if not os.path.exists(args.onnx):
        print(f"[convert] ✗ 找不到 ONNX: {args.onnx}", file=sys.stderr)
        sys.exit(1)

    try:
        from rknn.api import RKNN
    except ImportError:
        print("[convert] ✗ 未安装 rknn-toolkit2, 请执行: pip3 install rknn-toolkit2",
              file=sys.stderr)
        sys.exit(1)

    rknn = RKNN(verbose=True)

    # ── 1. config ──
    # 输入是 25 维 obs, 已经在节点内部 clip 到 [-20,20], 这里 mean=0 std=1 即可.
    print(f"[convert] target_platform = {args.target}")
    rknn.config(
        mean_values=[[0.0] * 25],
        std_values=[[1.0] * 25],
        target_platform=args.target,
        optimization_level=args.opt_level,
        quantized_dtype="asymmetric_quantized-8",
    )

    # ── 2. load onnx (锁定输入形状, RKNN 不支持动态 batch) ──
    try:
        in_shape = [int(x) for x in args.input_shape.split(",") if x.strip()]
    except ValueError:
        print(f"[convert] ✗ --input_shape 解析失败: {args.input_shape}", file=sys.stderr)
        sys.exit(2)
    print(f"[convert] load_onnx: {args.onnx}  inputs=['{args.input_name}'] shape={in_shape}")
    ret = rknn.load_onnx(
        model=args.onnx,
        inputs=[args.input_name],
        input_size_list=[in_shape],
    )
    if ret != 0:
        print(f"[convert] ✗ load_onnx 失败 ret={ret}", file=sys.stderr)
        sys.exit(ret)

    # ── 3. build ──
    dataset_path = ""
    if args.quant:
        if args.dataset_npy:
            dataset_path = build_calib_dataset(args.dataset_npy)
        elif args.dataset:
            dataset_path = args.dataset
        else:
            print("[convert] ✗ --quant 需要同时提供 --dataset 或 --dataset_npy",
                  file=sys.stderr)
            sys.exit(2)

    print(f"[convert] build: quantize={args.quant}, dataset={dataset_path or '<none>'}")
    ret = rknn.build(do_quantization=bool(args.quant),
                     dataset=dataset_path if args.quant else "")
    if ret != 0:
        print(f"[convert] ✗ build 失败 ret={ret}", file=sys.stderr)
        sys.exit(ret)

    # ── 4. export ──
    out_dir = os.path.dirname(os.path.abspath(args.rknn))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir, exist_ok=True)
    ret = rknn.export_rknn(args.rknn)
    if ret != 0:
        print(f"[convert] ✗ export_rknn 失败 ret={ret}", file=sys.stderr)
        sys.exit(ret)

    size_kb = os.path.getsize(args.rknn) / 1024.0
    print(f"[convert] ✅ 转换成功: {args.rknn}  ({size_kb:.1f} KB)")
    rknn.release()


if __name__ == "__main__":
    main()
