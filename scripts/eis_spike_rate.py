#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eis_spike_rate.py — 短期窗口看 spike 实际增长速率。

dashboard 报的 spike% 是历史累计平均, 偏长期. 本工具 60s 滑窗算 spike rate.
"""
import json
import sys
import time
from collections import deque

PATH = "/tmp/eis_diag.json"


def main():
    print("polling /tmp/eis_diag.json every 1s ...")
    print("=" * 60)
    samples = deque()  # (ts_us, ok, bypass, spike, drops)
    last = None
    start = time.monotonic()
    while time.monotonic() - start < 75:  # 60s 滑窗 + 15s warmup
        try:
            with open(PATH) as f:
                d = json.load(f)
        except (OSError, ValueError):
            time.sleep(1.0)
            continue
        ts = d.get("ts", 0)
        if last is None or ts != last:
            samples.append((ts, d.get("ok", 0), d.get("bypass", 0),
                            d.get("spike", 0), d.get("imu_drops", 0)))
            last = ts
        # 滑窗: 保留 60s 内
        now_us = samples[-1][0]
        while samples and (now_us - samples[0][0]) > 60_000_000:
            samples.popleft()
        if len(samples) >= 2:
            s0 = samples[0]; s1 = samples[-1]
            dt_s = (s1[0] - s0[0]) / 1e6
            if dt_s > 0:
                d_ok = s1[1] - s0[1]
                d_bp = s1[2] - s0[2]
                d_sp = s1[3] - s0[3]
                d_dr = s1[4] - s0[4]
                total = d_ok + d_bp
                spike_rate = 100.0 * d_sp / total if total else 0.0
                drops_rate = d_dr / dt_s  # drops/s
                print(f"\r[{dt_s:4.1f}s]  frames={total:5d}  "
                      f"spikes={d_sp:3d} ({spike_rate:5.1f}%)  "
                      f"drops={d_dr:3d} ({drops_rate:5.2f}/s)  "
                      f"  fix_deg={d.get('fix_deg', 0):.2f}°  "
                      f"status={d.get('status', '?')}",
                      end="", flush=True)
        time.sleep(1.0)
    print("\n\ndone.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye")
