#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_imu_stream.py — IMU 实机探针（方案文档 6.1，libscam 可用后第一时间执行）。

用途（在 192.168.2.203 上，放 ~/camera_sdk_test 运行）：
  1. 判定加速度单位：静止 |acc| ≈ 1.0 → g；≈1000 → milli-g（配置 acc_divisor_to_g）
  2. 判定重力轴 → axis_map 的依据
  3. 打印 11 组 IMU 的时间差分（burst 采样率/窗口）与曝光时间相位关系
  4. 手动单轴慢转 → 陀螺仪轴向/符号验证
  5. 保存一帧 JPG → 人工确认 3840x1200、编码区已裁、左右目方向

运行：
    cd ~/camera_sdk_test
    LD_LIBRARY_PATH=./build LD_PRELOAD=~/miniconda3/envs/tv/lib/libgomp.so.1 \
        ~/miniconda3/envs/tv/bin/python3 probe_imu_stream.py
"""
import math
import sys
import threading
import time

sys.path.insert(0, ".")
from scam_sdk import ScamSDK, CamFormat                       # noqa: E402


def main():
    sdk = ScamSDK("build/libscam.so")
    print("SDK version:", sdk.get_version())
    assert sdk.initialize(), "SCAM_Initialize failed"

    devs = sdk.enum_devices(max_count=6)
    print("devices:", devs)
    if not devs:
        sys.exit("no SCAM device")
    idx = devs[0]["number"]

    formats = sdk.get_device_formats(idx)
    print("formats:", formats)
    sel = next((i for i, f in enumerate(formats) if f["fmt"] != CamFormat.FORMAT_YUV422), 0)
    sdk.set_device_format(idx, sel)
    sdk.set_image_format(idx, CamFormat.FORMAT_NV12)

    frames = []
    done = threading.Event()

    def cb(fd):
        frames.append(fd)
        if len(frames) >= 30:
            done.set()

    print("opening device", idx, "...")
    sdk.open_device(idx, cb)
    deadline = time.time() + 10
    while not done.is_set() and time.time() < deadline:
        time.sleep(0.05)

    print("frames:", len(frames))
    if frames:
        # --- 1) 加速度单位与重力轴（取前 10 帧均值，要求静止） ---
        acc_mean = [0.0, 0.0, 0.0]
        n = 0
        for fd in frames[:10]:
            imu0 = fd.imu_data[0]["acc"]
            for k in range(3):
                acc_mean[k] += imu0[k]
            n += 1
        acc_mean = [v / n for v in acc_mean]
        mag = math.sqrt(sum(v * v for v in acc_mean))
        print("=" * 60)
        print("[1] acc mean = (%.3f, %.3f, %.3f)  |acc| = %.3f" % (*acc_mean, mag))
        print("    |acc|≈1.0 → 单位是 g（acc_divisor_to_g=1）；"
              "≈1000 → milli-g（acc_divisor_to_g=1000）")
        print("    重力分量最大的轴 = IMU 的竖直轴（对照 axis_map 配置）")

        # --- 3) IMU burst 时间结构与曝光相位 ---
        fd = frames[-1]
        ts = [m["uTime"] for m in fd.imu_data]
        print("=" * 60)
        print("[3] exposure start=%d end=%d dur=%d us" %
              (fd.start_exposure_time, fd.end_exposure_time,
               fd.end_exposure_time - fd.start_exposure_time))
        print("    imu uTime :", ts)
        print("    imu dt    :", [b - a for a, b in zip(ts, ts[1:])])
        print("    t_first - start_exp = %d us ; t_last - end_exp = %d us" %
              (ts[0] - fd.start_exposure_time, ts[-1] - fd.end_exposure_time))
        # 跨帧采样率估计
        if len(frames) >= 5:
            a, b = frames[-5], frames[-1]
            print("    burst-to-burst dt = %d us (frame period)" %
                  (b.imu_data[0]["uTime"] - a.imu_data[0]["uTime"]))
        # --- 磁力计 ---
        m0 = fd.mtt_data[0]
        print("    mtt[0]: t=%d mag=(%d,%d,%d) temp=%.1f" %
              (m0["uTime"], m0["mag"][0], m0["mag"][1], m0["mag"][2], m0["temp"]))

    # --- 4) 手动单轴慢转：峰值 + 带符号中位数（同时给 axis_map 与 axis_sign 依据） ---
    print("=" * 60)
    print("[4] 手动绕相机 X 轴慢转 10 秒（约 10~30°/s）：")
    print("    脚本将记录 gyro 三个轴的 |peak|（定 axis_map）和带符号 mean（定 axis_sign）")
    t_end = time.time() + 10
    peaks = [0.0, 0.0, 0.0]
    signed_acc = [[], [], []]
    frames.clear()
    samples = 0

    def cb2(fd):
        frames.append(fd)

    try:
        sdk.close_device(idx)
    except Exception:
        pass
    sdk.open_device(idx, cb2)
    while time.time() < t_end:
        time.sleep(0.1)
        if frames:
            g = frames[-1].imu_data[0]["gyro"]
            for k in range(3):
                peaks[k] = max(peaks[k], abs(g[k]))
                if abs(g[k]) > 1.0:  # 只在明显高于噪声底时计入 signed mean
                    signed_acc[k].append(float(g[k]))
            samples += 1
    print("    samples=%d" % samples)
    print("    gyro |peak| per axis (dps): (%.2f, %.2f, %.2f)" % tuple(peaks))
    print("    gyro signed mean (仅统计 |g|>1 dps 的样本) per axis:")
    for k in range(3):
        if signed_acc[k]:
            mean = sum(signed_acc[k]) / len(signed_acc[k])
            print("        axis %d  mean=%+.2f dps  n=%d  (== axis_sign[%d] = sign(物理正方向))" %
                  (k, mean, len(signed_acc[k]), k))
        else:
            print("        axis %d  无显著运动样本（请再转大一些）" % k)
    print("    用法：")
    print("      - peak 最大的轴  = 物理 X 旋转轴 → axis_map 中 X→该 axis")
    print("      - signed mean 的符号  = axis_sign 对应位的方向（+1 或 -1）")

    try:
        sdk.close_device(idx)
    except Exception:
        pass
    sdk.release()
    print("[5] 未自动保存图片。如需保存样例帧，请运行厂商 print_imu.py 按空格保存，")
    print("    确认图像为 3840x1200、左侧无编码区、左目在前右目在后。")
    print("DONE")


if __name__ == "__main__":
    main()
