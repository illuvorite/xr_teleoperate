#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eis_dashboard.py — 实时 EIS 防抖状态面板。

读取 /tmp/eis_diag.json (由 stabilizer.py 周期 dump) 并以 1 Hz 刷新。

用法:
    python3 scripts/eis_dashboard.py
    python3 scripts/eis_dashboard.py --path /tmp/eis_diag.json --interval 0.5

面板字段:
    status         当前状态 (ok / degraded / warming / frozen / bypass / recovering)
    ok/bypass/spike 累计计数
    fix_deg        上一帧校正角度 (°)
    bias_dps       陀螺仪零偏 (dps)
    imu_dts_ms     IMU 采样间隔分位数
    imu_drops      IMU 丢样本数
    yaw_drift_deg_per_min 由 fix_deg 滚动估计
"""
import argparse
import json
import os
import sys
import time
from collections import deque
from typing import Optional


def _read_json(path: str) -> Optional[dict]:
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _fmt_status(s: str) -> str:
    color = {
        "ok": "\x1b[32m",         # green
        "degraded": "\x1b[33m",   # yellow
        "warming": "\x1b[36m",    # cyan
        "frozen": "\x1b[35m",     # magenta
        "bypass": "\x1b[31m",     # red
        "recovering": "\x1b[33m",
        "recalibrating": "\x1b[35m",
        "error": "\x1b[31m",
    }.get(s, "\x1b[0m")
    return f"{color}{s:<11}\x1b[0m"


def _bar(v: float, vmax: float, width: int = 30) -> str:
    if vmax <= 0:
        return " " * width
    n = int(min(1.0, v / vmax) * width)
    return "[" + "#" * n + " " * (width - n) + "]"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", default="/tmp/eis_diag.json")
    ap.add_argument("--interval", type=float, default=1.0)
    args = ap.parse_args()

    # 滑动窗口估计 yaw drift: 存最近 60 s 的 fix_deg * dt
    fix_window: deque = deque(maxlen=int(60.0 / max(args.interval, 0.1)))
    t_window: deque = deque(maxlen=fix_window.maxlen)
    last_ts_us: Optional[int] = None

    print(f"\x1b[2J\x1b[H", end="")  # clear
    print(f"EIS Dashboard  path={args.path}  interval={args.interval}s")
    print("=" * 60)
    try:
        while True:
            data = _read_json(args.path)
            if data is None:
                print(f"\r\x1b[Kwaiting {args.path} ...", end="", flush=True)
                time.sleep(args.interval)
                continue

            ts_us = data.get("ts", 0)
            if last_ts_us is not None and ts_us > last_ts_us:
                dt_s = (ts_us - last_ts_us) * 1e-6
                t_window.append(dt_s)
                fix_window.append(data.get("fix_deg", 0.0))
            last_ts_us = ts_us

            # yaw drift 估计: sum(|fix_deg|) / window_seconds
            total_drift = sum(fix_window)
            total_s = sum(t_window)
            drift_per_min = (total_drift / total_s * 60.0) if total_s > 0 else 0.0

            ok = data.get("ok", 0)
            bp = data.get("bypass", 0)
            sp = data.get("spike", 0)
            total = ok + bp
            bypass_pct = 100.0 * bp / total if total else 0.0
            spike_pct = 100.0 * sp / total if total else 0.0

            bias = data.get("gyro_bias_dps", [0, 0, 0])
            imu_p50 = data.get("imu_dts_ms_p50", 0.0)
            imu_p99 = data.get("imu_dts_ms_p99", 0.0)
            imu_n = data.get("imu_n_per_frame", 0)
            drops = data.get("imu_drops", 0)
            fix = data.get("fix_deg", 0.0)
            status = data.get("status", "?")

            print(f"\x1b[H\x1b[J", end="")
            print(f"EIS Dashboard   t={ts_us}   "
                  f"u={time.strftime('%H:%M:%S')}")
            print("=" * 60)
            print(f"  status     : {_fmt_status(status)}"
                  f"  fix_deg={fix:6.2f}°")
            print(f"  counts     : ok={ok:<6} bypass={bp:<4} "
                  f"spike={sp:<4}  drops={drops}")
            print(f"  bypass%    : {bypass_pct:5.1f}% "
                  f"{_bar(bypass_pct, 10.0)}  "
                  f"spike%={spike_pct:5.1f}% {_bar(spike_pct, 5.0)}")
            print(f"  yaw drift  : {drift_per_min:6.2f}°/min  (60s window)")
            print(f"  gyro bias  : "
                  f"x={bias[0]:+6.3f}  y={bias[1]:+6.3f}  z={bias[2]:+6.3f} dps")
            print(f"  IMU dt     : p50={imu_p50:.2f}ms "
                  f"p99={imu_p99:.2f}ms  N={imu_n}/frame")
            print()
            print("  legend: status ok=正常 warming=校准中 bypass=旁路")
            print("          frozen=姿态冻结  degraded=IMU 间隙  "
                  "recovering=冲击恢复")
            print("=" * 60)

            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nbye")
        return 0


if __name__ == "__main__":
    sys.exit(main())
