#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""teleimager 集成模块：SBS 双目 IMU 电子防抖封装。

安装（在 192.168.2.203 上）：
    cp imu_stabilizer.py  ~/xr_teleoperate-main/teleop/teleimager/src/teleimager/
    cp stabilizer.py      ~/xr_teleoperate-main/teleop/teleimager/src/teleimager/
    # 修改 image_server.py 增加 scam 分派（见 image_server.patch.md）
    # 修改 scam_camera.py 的 _update_frame（见 image_server.patch.md）
    # 修改 cam_config_server.yaml（见 cam_config_server.head_camera.scam.yaml）

依赖：仅 numpy + opencv（teleimager 既有依赖，零新增第三方库）。
"""

from __future__ import annotations

import math
import threading
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .imu_stabilizer import (EisConfig, EisStabilizer, ImuSample, EyeWarp,
                             warp_eye_bgr)

try:  # teleimager 环境用 multiprocessing-safe 的 logging_mp；独立环境回退标准 logging
    import logging_mp as _logging_mod
except ImportError:
    import logging as _logging_mod
logger = _logging_mod.getLogger(__name__)
_log_dbg = True


def _yaml_matrix(v) -> Optional[np.ndarray]:
    """把 yaml list / tuple / None 转成 np.ndarray; 形状不对返回 None。"""
    if v is None:
        return None
    try:
        arr = np.asarray(v, dtype=float)
    except (TypeError, ValueError):
        return None
    if arr.ndim == 1 and arr.size in (4, 5, 8, 12):
        return arr
    if arr.ndim == 2 and arr.shape in [(3, 3), (1, 5), (1, 4), (1, 8), (1, 12)]:
        return arr
    return None


def _build_undistort_maps(K_left, dist_left, K_right, dist_right,
                          eye_w: int, eye_h: int):
    """为每眼预计算 cv2.remap 用的 map1/map2。

    输入可以是:
      - K/dist 都为 None  → 返回 (None, None), 调用方不 remap
      - 仅有 K 无 dist    → cv2.undistort 仍可工作 (视为零畸变), 返回 map
    """
    import cv2
    out_l = out_r = None
    if K_left is not None:
        K = K_left if K_left.ndim == 2 else K_left.reshape(3, 3)
        d = dist_left if dist_left is not None else np.zeros(5)
        d = d.flatten() if d.ndim > 1 else d
        map1, map2 = cv2.initUndistortRectifyMap(
            K, d, None, K, (eye_w, eye_h), cv2.CV_32FC1)
        out_l = (map1, map2)
    if K_right is not None:
        K = K_right if K_right.ndim == 2 else K_right.reshape(3, 3)
        d = dist_right if dist_right is not None else np.zeros(5)
        d = d.flatten() if d.ndim > 1 else d
        map1, map2 = cv2.initUndistortRectifyMap(
            K, d, None, K, (eye_w, eye_h), cv2.CV_32FC1)
        out_r = (map1, map2)
    return out_l, out_r


def _maybe_remap(img: np.ndarray, mp) -> np.ndarray:
    if mp is None:
        return img
    import cv2
    return cv2.remap(img, mp[0], mp[1], cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_REPLICATE)


def _parse_imu_item(item) -> ImuSample:
    """兼容多种 IMU 数据形态:
    - scam_sdk.FrameData.imu_data 的 dict: {'uTime', 'acc': (x,y,z), 'gyro': (x,y,z)}
    - scam_camera._on_frame 的 tuple:      (t, (ax,ay,az), (gx,gy,gz))
    - ndarray (7,):                        [t_us, ax, ay, az, gx, gy, gz]  (P0.5, 单样本)
    - ndarray (N, 7):                      批量样本, 每行 [t_us, ax, ay, az, gx, gy, gz] (P0.5, 11/帧)
    """
    if isinstance(item, dict):
        return ImuSample(t_us=int(item["uTime"]),
                         gyro_dps=np.asarray(item["gyro"], dtype=float),
                         acc_raw=np.asarray(item["acc"], dtype=float))
    if isinstance(item, np.ndarray):
        if item.ndim == 1 and item.shape[0] == 7:
            return ImuSample(t_us=int(item[0]),
                             gyro_dps=item[4:7].astype(float, copy=False),
                             acc_raw=item[1:4].astype(float, copy=False))
        if item.ndim == 2 and item.shape[1] == 7:
            # 期望由 _parse_imu_burst 一次性处理整批, 这里取最后一行作为单样本兜底
            last = item[-1]
            return ImuSample(t_us=int(last[0]),
                             gyro_dps=last[4:7].astype(float, copy=False),
                             acc_raw=last[1:4].astype(float, copy=False))
    t, acc, gyro = item
    return ImuSample(t_us=int(t), gyro_dps=np.asarray(gyro, dtype=float),
                     acc_raw=np.asarray(acc, dtype=float))


def _parse_imu_burst(imu_items) -> List[ImuSample]:
    """批量解析 11/帧 IMU 数组, 比逐行 _parse_imu_item 快 ~5x。"""
    if imu_items is None:
        return []
    # 快速路径: 11×7 ndarray
    if isinstance(imu_items, np.ndarray):
        if imu_items.ndim == 2 and imu_items.shape[1] == 7:
            ts = imu_items[:, 0].astype(np.int64)
            acc = imu_items[:, 1:4].astype(float, copy=False)
            gyro = imu_items[:, 4:7].astype(float, copy=False)
            return [ImuSample(t_us=int(ts[i]),
                              gyro_dps=gyro[i],
                              acc_raw=acc[i]) for i in range(len(ts))]
        if imu_items.size == 0:
            return []
    try:
        n = len(imu_items)
    except TypeError:
        return [_parse_imu_item(imu_items)]
    if n == 0:
        return []
    return [_parse_imu_item(x) for x in imu_items]


class SbsStabilizer:
    """对一帧 SBS 双目 BGR 图像施加 IMU 旋转防抖。

    双目一致性约束（关键）：左右目使用同一 q_fix 生成的各自单应性；
    裁剪/缩放参数完全一致；绝不独立平滑左右目。
    process() 永不抛异常：任何异常自动 bypass 原图并计数。
    """

    def __init__(self, cam_config: dict, eye_w: int, eye_h: int,
                 out_eye_w: int, out_eye_h: int):
        sec = (cam_config or {}).get("stabilization", {}) or {}
        cfg = EisConfig(
            acc_divisor_to_g=float(sec.get("acc_divisor_to_g", 1000.0)),
            exposure_ref=str(sec.get("exposure_ref", "midpoint")),
            static_calib_s=float(sec.get("static_calibration_s", 2.0)),
            static_gyro_dps=float(sec.get("static_gyro_dps", 1.5)),   # 实测静止噪声 ~0.85 dps
            relearn_beta=float(sec.get("relearn_beta", 0.001)),
            static_acc_g=float(sec.get("static_acc_g", 0.05)),
            smoothing_alpha=float(sec.get("smoothing_alpha", 0.35)),
            motion_boost=bool(sec.get("motion_boost", True)),
            max_correction_deg=float(sec.get("max_correction_deg", 8.0)),
            crop_ratio=float(sec.get("crop_ratio", 0.06)),
            enable_mahony=(str(sec.get("mode", "gyro")).lower() == "mahony"),
            max_gap_s=float(sec.get("imu_gap_reset_ms", 50)) / 1000.0,
            imu_timeout_freeze_s=float(sec.get("imu_timeout_bypass_ms", 200)) / 1000.0,
            # P1.1 ESKF
            use_eskf=bool(sec.get("use_eskf", False)),
            eskf_sigma_g_dps=float(sec.get("eskf_sigma_g_dps", 0.03)),
            eskf_sigma_b_dps=float(sec.get("eskf_sigma_b_dps", 5e-4)),
            eskf_sigma_a_g=float(sec.get("eskf_sigma_a_g", 0.03)),
            # P1.3 旋转中心
            baseline_mm=float(sec.get("baseline_mm", 64.0)),
            use_recenter_to_baseline=bool(sec.get("use_recenter_to_baseline", True)),
            scene_depth_m=float(sec.get("scene_depth_m", 2.0)),
            # P1.5 状态机
            warmup_crop_ratio=float(sec.get("warmup_crop_ratio", 0.0)),
            spike_recover_hold_s=float(sec.get("spike_recover_hold_s", 0.2)),
            spike_gyro_sat_dps=float(sec.get("spike_gyro_sat_dps", 200.0)),
        )
        am = sec.get("axis_map", [0, 1, 2])
        asg = sec.get("axis_sign", [1.0, 1.0, 1.0])
        cfg.axis_map = tuple(int(v) for v in am)
        cfg.axis_sign = tuple(float(v) for v in asg)
        imu_off = sec.get("imu_to_baseline_mid_mm", [0.0, 0.0, 0.0])
        cfg.imu_to_baseline_mid_mm = tuple(float(v) for v in imu_off)

        # P1.2: 双目内参 / 畸变
        K_left = _yaml_matrix(sec.get("K_left"))
        K_right = _yaml_matrix(sec.get("K_right"))
        dist_left = _yaml_matrix(sec.get("dist_left"))
        dist_right = _yaml_matrix(sec.get("dist_right"))

        self.enabled = bool(sec.get("enabled", False))
        self.border = str(sec.get("border_mode", "replicate"))
        self._st = EisStabilizer(cfg, K_left, K_right, eye_w, eye_h, out_eye_w, out_eye_h)
        # P1.2: 预计算 undistort remap, 后续每帧 cv2.remap
        self._undist_map_l, self._undist_map_r = _build_undistort_maps(
            K_left, dist_left, K_right, dist_right, eye_w, eye_h)
        self._bypass_count = 0
        self._ok_count = 0
        self._last_status = None
        self._last_t_us = 0
        self._spike_count = 0
        self._owner_thread = threading.get_ident()
        self._cross_thread_warned = False
        # 多线程共享状态保护: WebRTC_PublisherThread + 主线程并发 apply
        self._lock = threading.Lock()
        # IMU 采样间隔监控 (P0.6)
        self._imu_dt_buf: List[float] = []
        self._last_burst_n: int = 0   # 最近一次 IMU burst 长度, 给 dashboard 显示
        self._diag_path = str(sec.get("diag_path", "/tmp/eis_diag.json"))
        self._diag_last_dump_t = 0.0
        # 校准状态持久化 (P0.4)
        self._calib_store = None
        calib_path = sec.get("calibration_path")
        if calib_path:
            try:
                from .eis_calib_store import EisCalibStore
                self._calib_store = EisCalibStore(calib_path)
                loaded = self._calib_store.load(
                    key=str(sec.get("calibration_key", "default")),
                    max_age_days=int(sec.get("calibration_max_age_days", 7)),
                )
                if loaded is not None:
                    self._st.att.preload_bias(loaded.bias_dps,
                                              loaded.q_align if hasattr(loaded, "q_align") else None)
                    logger.info("[EIS] using cached calibration, age=%.1f h bias=(%.3f,%.3f,%.3f) dps",
                                loaded.age_h, *loaded.bias_dps)
                else:
                    logger.info("[EIS] no usable cached calibration at %s; full warmup",
                                calib_path)
            except Exception as e:  # noqa: BLE001
                logger.warning("[EIS] calib store unavailable: %r", e)
                self._calib_store = None

        logger.info("[EIS] init: enabled=%s mode=%s beta=%.2f crop=%.3f max_deg=%.1f",
                    self.enabled, sec.get("mode", "gyro"),
                    cfg.smoothing_alpha, cfg.crop_ratio, cfg.max_correction_deg)

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, v: bool) -> None:
        self._enabled = bool(v)

    def apply(self, frame_sbs_bgr: np.ndarray,
              t_start_us: int, t_end_us: int,
              imu_items: Sequence) -> Tuple[np.ndarray, str]:
        """输入一帧 SBS BGR + 该帧元数据，返回（防抖后 SBS BGR, status）。"""
        if not self._enabled:
            return frame_sbs_bgr, "disabled"
        # 跨线程诊断: 首次发现时 warning, 之后放行 (锁保护并发安全)
        cur_tid = threading.get_ident()
        if cur_tid != self._owner_thread and not self._cross_thread_warned:
            logger.warning(
                "[EIS] cross-thread apply detected (owner=%s caller=%s); "
                "falling back to lock-protected mode.",
                self._owner_thread, cur_tid)
            self._cross_thread_warned = True
        with self._lock:
            try:
                h, w = frame_sbs_bgr.shape[:2]
                half = w // 2
                left = frame_sbs_bgr[:, :half]
                right = frame_sbs_bgr[:, half:2 * half]

                # P1.2: 先 remap 去畸变, 再交给 EIS warp
                left = _maybe_remap(left, self._undist_map_l)
                right = _maybe_remap(right, self._undist_map_r)

                burst: List[ImuSample] = _parse_imu_burst(imu_items)
                self._last_burst_n = len(burst)
                status, wl, wr = self._st.process(int(t_start_us), int(t_end_us), burst)

                out_l = warp_eye_bgr(left, wl, self.border)
                out_r = warp_eye_bgr(right, wr, self.border)
                out = np.concatenate([out_l, out_r], axis=1)

                # ----- 统计 (P0.2: 不再加锁) -----
                if status in ("ok", "degraded"):
                    self._ok_count += 1
                else:
                    self._bypass_count += 1

                # IMU 采样间隔监控 (P0.6)
                if len(burst) >= 2:
                    ts = np.fromiter((s.t_us for s in burst), dtype=np.int64, count=len(burst))
                    dts_ms = np.diff(ts) / 1000.0
                    self._imu_dt_buf.extend(dts_ms.tolist())
                    if len(self._imu_dt_buf) > 2000:
                        del self._imu_dt_buf[:len(self._imu_dt_buf) - 2000]

                # SPIKE 诊断
                _fix = self._st.last_fix_deg
                if _fix > 0.5:
                    self._spike_count += 1
                    _t_now = int(t_start_us)
                    _gap_ms = (_t_now - self._last_t_us) / 1000.0 if self._last_t_us else 0.0
                    _bias = self._st.att.gyro_bias_dps
                    logger.warning(
                        "[EIS] SPIKE %s fix=%.2f deg gap=%.1f ms status=%s burst=%d bias=(%.3f, %.3f, %.3f)",
                        getattr(self, "_topic", "?"), _fix, _gap_ms, status, len(burst),
                        _bias[0], _bias[1], _bias[2],
                    )
                self._last_t_us = int(t_start_us)

                if status != self._last_status and self._last_status is not None:
                    logger.info("[EIS] %s status: %s -> %s (fix=%.3f deg, bias=(%.3f, %.3f, %.3f))",
                                getattr(self, "_topic", "?"), self._last_status, status,
                                self._st.last_fix_deg, *self._st.att.gyro_bias_dps)
                self._last_status = status
                if self._ok_count % 300 == 0 and self._ok_count > 0:
                    logger.info("[EIS] ok=%d bypass=%d spike=%d fix_deg=%.3f status=%s bias=(%.3f, %.3f, %.3f)",
                                self._ok_count, self._bypass_count, self._spike_count,
                                self._st.last_fix_deg, self._st.last_status,
                                *self._st.att.gyro_bias_dps)

                # 周期 dump 诊断 (P0.6)
                self._maybe_dump_diag(t_start_us)

                return out, status
            except Exception:                                    # noqa: BLE001
                logger.exception("[EIS] internal error -> bypass")
                self._bypass_count += 1
                return frame_sbs_bgr, "error"

    def _maybe_dump_diag(self, t_now_us: int) -> None:
        import time
        now = time.monotonic()
        if now - self._diag_last_dump_t < 5.0:
            return
        self._diag_last_dump_t = now
        try:
            import json
            arr = np.asarray(self._imu_dt_buf[-2000:], dtype=np.float64) if self._imu_dt_buf \
                else np.zeros(0, dtype=np.float64)
            diag = {
                "ts": t_now_us,
                "ok": self._ok_count,
                "bypass": self._bypass_count,
                "spike": self._spike_count,
                "fix_deg": float(self._st.last_fix_deg),
                "status": self._st.last_status,
                "bias_ready": self._st.att.bias_ready,
                "gyro_bias_dps": self._st.att.gyro_bias_dps.tolist(),
                "imu_drops": self._st.att.drop_count,
                "imu_dts_ms_p50": float(np.median(arr)) if arr.size else 0.0,
                "imu_dts_ms_p99": float(np.percentile(arr, 99)) if arr.size else 0.0,
                "imu_dts_ms_max": float(np.max(arr)) if arr.size else 0.0,
                "imu_n_per_frame": self._last_burst_n,
            }
            with open(self._diag_path, "w") as f:
                json.dump(diag, f)
        except Exception as e:  # noqa: BLE001
            logger.debug("[EIS] diag dump failed: %r", e)

    def debug_stats(self) -> dict:
        with self._lock:
            return {"ok": self._ok_count, "bypass": self._bypass_count,
                    "spike": self._spike_count,
                    "fix_deg": self._st.last_fix_deg, "status": self._st.last_status,
                    "bias_ready": self._st.att.bias_ready,
                    "gyro_bias_dps": self._st.att.gyro_bias_dps.tolist(),
                    "imu_drops": self._st.att.drop_count}
