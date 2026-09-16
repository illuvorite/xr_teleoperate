#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eis_calibrate_imu_mount.py — IMU→相机安装角标定（P1.4）

约定: 用户戴上头显后保持头部绝对竖直, 视线水平前视 5 s.
脚本接收 --imu-source (csv / rosbag / sdk) 与 --output-yaml 路径, 估计
R_imu_to_camera (3x3 旋转) 并写回 yaml.

核心: 静止期 IMU 测得的重力方向 (a_mean) 在 IMU 坐标系.
     相机坐标系约定重力方向为 (0, -1, 0) (相机 y 轴向下).
     R_align = rotation_from_two_vectors(a_mean_unit, g_cam).

本脚本提供两种工作模式:
  1) --from-csv <path>   从 csv 读取 (t_us, ax, ay, az, gx, gy, gz)
  2) --from-synthetic    用合成数据演示, 不接硬件

输出 (yaml):
  stabilization:
    imu_to_camera_R: [[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]]
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from typing import List, Optional, Tuple

import numpy as np


G_CAM = np.array([0.0, -1.0, 0.0], dtype=float)


def rotation_from_two_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Rodrigues: 找 R 使 R a = b."""
    a = a / max(np.linalg.norm(a), 1e-12)
    b = b / max(np.linalg.norm(b), 1e-12)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        # 平行: c ≈ ±1
        if c > 0:
            return np.eye(3)
        # 180° 旋转, 任意正交轴
        axis = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        v = np.cross(a, axis)
        v = v / max(np.linalg.norm(v), 1e-12)
        K = _skew(v)
        return np.eye(3) + 2 * K @ K
    K = _skew(v / s)
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def quat_from_R(R: np.ndarray) -> np.ndarray:
    """R (3x3) -> (w, x, y, z)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S
    return np.array([w, x, y, z], dtype=float)


def load_csv(path: str, acc_divisor: float = 1000.0) -> Tuple[np.ndarray, np.ndarray]:
    ts, acc = [], []
    with open(path) as f:
        r = csv.reader(f)
        header = next(r, None)
        for row in r:
            if len(row) < 7:
                continue
            t = int(float(row[0]))
            a = np.array([float(row[1]), float(row[2]), float(row[3])]) / acc_divisor
            ts.append(t); acc.append(a)
    return np.asarray(ts), np.asarray(acc)


def synthesize_static(duration_s: float = 5.0, fs: int = 1000,
                      imu_gravity: np.ndarray = np.array([0, 0, 1.0]),
                      noise_g: float = 0.005) -> Tuple[np.ndarray, np.ndarray]:
    """合成静止期数据, IMU 测得重力 = imu_gravity (单位向量)."""
    n = int(duration_s * fs)
    ts = np.arange(n) * (1_000_000 // fs)
    acc = np.tile(imu_gravity, (n, 1)) + np.random.normal(0, noise_g, (n, 3))
    return ts, acc


def still_detect(acc: np.ndarray, tol: float = 0.05) -> np.ndarray:
    """对每行计算 |‖a‖-1|, 返回 mask: True 表示静止."""
    norms = np.linalg.norm(acc, axis=1)
    return np.abs(norms - 1.0) < tol


def estimate_R(acc_static: np.ndarray) -> np.ndarray:
    a_mean = np.mean(acc_static, axis=0)
    return rotation_from_two_vectors(a_mean, G_CAM)


def write_yaml_R(path: str, key: str, R: np.ndarray) -> None:
    """追加 / 替换 yaml 字段. 不引入 PyYAML 依赖, 用手写 patch."""
    import re
    block_lines = ["  stabilization:"]
    block_lines.append(f"    imu_to_camera_R:")
    for row in R:
        block_lines.append("      - ["
                           + ", ".join(f"{v: .8f}" for v in row) + "]")
    block = "\n".join(block_lines) + "\n"
    try:
        with open(path) as f:
            txt = f.read()
    except FileNotFoundError:
        with open(path, "w") as f:
            f.write(block)
        return
    # 替换 stabilization 整段
    new = re.sub(r"  stabilization:\n(?:    [^\n]*\n)+", block, txt, count=1)
    if new == txt:
        # 没匹配到, 在末尾追加
        if not txt.endswith("\n"):
            txt += "\n"
        new = txt + "\n" + block
    with open(path, "w") as f:
        f.write(new)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from-csv")
    ap.add_argument("--from-synthetic", action="store_true")
    ap.add_argument("--duration-s", type=float, default=5.0)
    ap.add_argument("--acc-divisor", type=float, default=1000.0)
    ap.add_argument("--output-yaml", required=True)
    ap.add_argument("--synthetic-gravity", default="0,0,1",
                    help="合成 IMU 重力方向 (csv: 3 floats)")
    args = ap.parse_args()

    if args.from_csv:
        ts, acc = load_csv(args.from_csv, acc_divisor=args.acc_divisor)
    elif args.from_synthetic:
        grav = np.array([float(x) for x in args.synthetic_gravity.split(",")])
        ts, acc = synthesize_static(duration_s=args.duration_s, imu_gravity=grav)
        print(f"[calib] synthesized {len(ts)} samples, g_imu={grav}", file=sys.stderr)
    else:
        ap.error("specify --from-csv or --from-synthetic")

    mask = still_detect(acc)
    n_static = int(mask.sum())
    if n_static < 50:
        print(f"[calib] ERROR: only {n_static} static samples, need >= 50",
              file=sys.stderr)
        return 1
    R = estimate_R(acc[mask])
    q = quat_from_R(R)
    print(f"[calib] static samples: {n_static}/{len(acc)}")
    print(f"[calib] R_imu_to_cam:\n{R}")
    print(f"[calib] quat (w,x,y,z): {q}")
    write_yaml_R(args.output_yaml, "imu_to_camera_R", R)
    print(f"[calib] wrote {args.output_yaml}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
