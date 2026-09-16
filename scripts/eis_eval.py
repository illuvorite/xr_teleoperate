#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eis_eval.py — EIS 防抖效果离线评估工具 (P3.1)

原理:
  对每帧用 ORB 特征 + Lucas-Kanade 光流估计 2D 旋转 (essential matrix RANSAC).
  累计每帧间旋转角 θ(t), 取 std(θ) 作为残余抖动 RMS (越小越稳).
  对比 stabilization enabled=true (off) vs enabled=false (on).

输入:
  --left-mp4  拆出的左眼视频 (任何工具: ffmpeg -i ... -vf crop=...)
  或: --sbs-mp4 + 自动拆左半

输出:
  - per-frame rotation time series
  - summary: mean, std, P95, drift_slope (度/秒)
  - 打印对比表

依赖: opencv-python, numpy
"""
import argparse
import csv
import json
import os
import sys
from typing import List, Optional, Tuple

import cv2
import numpy as np


def _split_sbs(sbs_path: str, out_path: str) -> str:
    cap = cv2.VideoCapture(sbs_path)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {sbs_path}")
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    half = w // 2
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(out_path, fourcc, fps, (half, h))
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        writer.write(frame[:, :half])
    cap.release(); writer.release()
    return out_path


def _estimate_rotation(prev_gray: np.ndarray, cur_gray: np.ndarray,
                       max_corners: int = 200) -> Optional[Tuple[float, np.ndarray]]:
    """用 ORB 特征 + Essential matrix 估计帧间相对旋转 (yaw/pitch/roll, 度)."""
    pts0 = cv2.goodFeaturesToTrack(prev_gray, maxCorners=max_corners,
                                   qualityLevel=0.01, minDistance=20)
    if pts0 is None or len(pts0) < 8:
        return None
    pts1, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, cur_gray, pts0, None)
    if pts1 is None:
        return None
    st = st.reshape(-1) == 1
    p0 = pts0[st]; p1 = pts1[st]
    if len(p0) < 8:
        return None
    E, _ = cv2.findEssentialMat(p0, p1, focal=1.0, pp=(0.0, 0.0),
                                method=cv2.RANSAC, prob=0.999, threshold=1.0)
    if E is None:
        return None
    _, R, t, _ = cv2.recoverPose(E, p0, p1, focal=1.0, pp=(0.0, 0.0))
    # 转欧拉角
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    pitch = np.arctan2(-R[2, 0], sy)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    roll = np.arctan2(R[2, 1], R[2, 2])
    eul = np.degrees([pitch, yaw, roll])
    # 旋转总角 = ||R - I||_F
    rot_angle = float(np.degrees(np.arccos(
        np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))))
    return rot_angle, eul


def evaluate(mp4_path: str, max_frames: int = 0, every_n: int = 1,
             label: str = "") -> dict:
    cap = cv2.VideoCapture(mp4_path)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {mp4_path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    rotations: List[float] = []   # 帧间总旋转 (°)
    eulers: List[List[float]] = []  # 累计 pitch/yaw/roll
    yaw_cum = 0.0
    t_list: List[float] = []
    prev_gray = None
    frame_idx = 0
    processed = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_idx % every_n != 0:
            frame_idx += 1; continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        if prev_gray is not None:
            res = _estimate_rotation(prev_gray, gray)
            if res is not None:
                rot, eul = res
                rotations.append(rot)
                eulers.append(list(eul))
                yaw_cum += eul[1]
                t_list.append(frame_idx / fps)
        prev_gray = gray
        frame_idx += 1
        processed += 1
        if 0 < max_frames <= processed:
            break
    cap.release()

    if not rotations:
        return {"label": label, "frames": 0}

    rot = np.asarray(rotations)
    eul = np.asarray(eulers)  # (N, 3) pitch, yaw, roll 增量
    # 估计 yaw drift slope (线性回归)
    if len(t_list) >= 2:
        t = np.asarray(t_list)
        yaw_cum_arr = np.cumsum(eul[:, 1])
        # 简单斜率: 首末差分
        slope = (yaw_cum_arr[-1] - yaw_cum_arr[0]) / (t[-1] - t[0])
    else:
        slope = 0.0
    # 残余抖动 = 相邻帧间 |rot| 的 RMS (代表"高频抖动"), 与平均分开
    # 估计方法: 对 rot 序列做中位数去基线 (低频漂移), 取 std
    rot_baseline = float(np.median(rot))
    rot_highfreq = rot - rot_baseline
    return {
        "label": label,
        "frames": len(rotations),
        "fps": fps,
        "rot_mean_deg": float(rot.mean()),
        "rot_std_deg": float(rot.std()),       # 总体 RMS
        "rot_rms_highfreq_deg": float(np.sqrt(np.mean(rot_highfreq ** 2))),
        "rot_p95_deg": float(np.percentile(rot, 95)),
        "rot_max_deg": float(rot.max()),
        "yaw_drift_deg_per_s": float(slope),
        "yaw_drift_deg_per_min": float(slope * 60.0),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--off", required=True, help="stabilization=off 视频 (左眼)")
    ap.add_argument("--on", required=True, help="stabilization=on 视频 (左眼)")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--every-n", type=int, default=1)
    ap.add_argument("--sbs-off", help="如果是 SBS 视频, 先拆左眼")
    ap.add_argument("--sbs-on")
    ap.add_argument("--split-out", default="/tmp/eis_eval_left.mp4")
    ap.add_argument("--csv", help="可选: 把 summary 写 csv")
    args = ap.parse_args()

    if args.sbs_off:
        args.off = _split_sbs(args.sbs_off, args.split_out)
    if args.sbs_on:
        out2 = args.split_out.replace(".mp4", "_on.mp4")
        args.on = _split_sbs(args.sbs_on, out2)

    if not os.path.exists(args.off):
        print(f"ERROR: missing {args.off}", file=sys.stderr)
        return 1
    if not os.path.exists(args.on):
        print(f"ERROR: missing {args.on}", file=sys.stderr)
        return 1

    print("evaluating OFF (no stabilization) ...", file=sys.stderr)
    r_off = evaluate(args.off, max_frames=args.max_frames,
                    every_n=args.every_n, label="off")
    print("evaluating ON  (stabilized)        ...", file=sys.stderr)
    r_on = evaluate(args.on, max_frames=args.max_frames,
                   every_n=args.every_n, label="on")

    # 打印对比表
    print()
    print("=" * 70)
    print(f"{'metric':<30} {'OFF':>15} {'ON':>15} {'reduction':>10}")
    print("-" * 70)
    keys = [
        ("rot_mean_deg", "frame rotation mean (°)"),
        ("rot_std_deg", "frame rotation std  (°)"),
        ("rot_rms_highfreq_deg", "high-freq jitter RMS (°)"),
        ("rot_p95_deg", "frame rotation P95  (°)"),
        ("rot_max_deg", "frame rotation max  (°)"),
        ("yaw_drift_deg_per_min", "yaw drift        (°/min)"),
    ]
    for k, name in keys:
        v_off = r_off.get(k, 0.0); v_on = r_on.get(k, 0.0)
        if abs(v_off) > 1e-9:
            red = (1.0 - v_on / v_off) * 100.0
            red_s = f"{red:+6.1f}%"
        else:
            red_s = "  n/a "
        print(f"{name:<30} {v_off:>15.4f} {v_on:>15.4f} {red_s:>10}")
    print("=" * 70)
    print(f"frames: off={r_off['frames']}  on={r_on['frames']}  fps={r_off['fps']:.1f}")
    print()
    print("interpretation:")
    print("  rot_rms_highfreq_deg (高频抖动 RMS) 越低越好, 减幅 >30% 算有效")
    print("  rot_std_deg         (总体 RMS)  反映整体抖动水平")
    print("  yaw_drift_deg_per_min 越接近 0 越好, < 1°/min 优秀")

    if args.csv:
        with open(args.csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["metric", "off", "on"])
            for k, _ in keys:
                w.writerow([k, r_off.get(k, 0.0), r_on.get(k, 0.0)])
        print(f"\ncsv -> {args.csv}")

    summary = {"off": r_off, "on": r_on,
               "rot_std_reduction_pct":
                   (1.0 - r_on["rot_std_deg"] / r_off["rot_std_deg"]) * 100.0
                   if r_off["rot_std_deg"] > 0 else 0.0,
               "rot_highfreq_reduction_pct":
                   (1.0 - r_on["rot_rms_highfreq_deg"] / r_off["rot_rms_highfreq_deg"]) * 100.0
                   if r_off["rot_rms_highfreq_deg"] > 0 else 0.0}
    print(f"\nsummary json:\n{json.dumps(summary, indent=2)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
