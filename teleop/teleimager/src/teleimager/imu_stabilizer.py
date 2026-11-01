#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""imu_stabilizer.py — IMU 电子防抖 Python 原型（与 src/eis C++ 实现一一对应）。

用途：
  1. 无相机时的合成数据自检（test_imu_stabilizer.py）；
  2. 实机录制数据（帧 + 11 组 IMU/帧）的离线复算与参数扫描；
  3. teleimager 集成的算法源（teleimager_patch/stabilizer.py 即本文件的精简版）。

依赖：numpy（必需）、opencv-python（仅图像 warp 时需要）。
约定与 C++ 一致：相机系 x 右 y 下 z 向前；四元数 (w,x,y,z) 把相机系向量变换到
世界参考系（z 上）；时间戳微秒；陀螺仪 dps；加速度按 acc_divisor_to_g 换算为 g。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np


# ----------------------------------------------------------------------------
# 小工具：四元数与旋转
# ----------------------------------------------------------------------------
def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0, 0, 0])


def quat_from_rotvec(r: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return quat_normalize(np.array([1.0, r[0] * 0.5, r[1] * 0.5, r[2] * 0.5]))
    h = 0.5 * th
    return np.array([math.cos(h), *(r * (math.sin(h) / th))])


def rotvec_from_quat(q: np.ndarray) -> np.ndarray:
    q = q if q[0] >= 0 else -q
    s2 = q[1] ** 2 + q[2] ** 2 + q[3] ** 2
    if s2 < 1e-20:
        return 2.0 * q[1:4]
    s = math.sqrt(s2)
    th = 2.0 * math.atan2(s, q[0])
    return th * q[1:4] / s


def quat_to_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def compose_K_R_Kinv(K: np.ndarray, R: np.ndarray) -> np.ndarray:
    """H = K @ R @ K^-1（K 为 3x3 内参）。"""
    return K @ R @ np.linalg.inv(K)


def compose_K_R_Kinv_with_kinv(K: np.ndarray, K_inv: np.ndarray, R: np.ndarray) -> np.ndarray:
    """H = K @ R @ K_inv；预计算 K_inv 后省一次求逆。"""
    return K @ R @ K_inv


# ----------------------------------------------------------------------------
# 配置与数据结构
# ----------------------------------------------------------------------------
@dataclass
class EisConfig:
    acc_divisor_to_g: float = 1000.0          # 待确认：厂商 print_imu.py 为 /1000
    exposure_ref: str = "midpoint"            # midpoint | start | end
    static_calib_s: float = 2.0
    static_gyro_dps: float = 0.5
    static_acc_g: float = 0.05
    relearn_bias: bool = True
    relearn_beta: float = 0.001
    max_gap_s: float = 0.05
    imu_timeout_freeze_s: float = 0.20
    smoothing_alpha: float = 0.35             # β：每帧跟随比例；1 = 关闭防抖
    motion_boost: bool = True
    motion_boost_factor: float = 2.0
    motion_boost_thresh_deg: float = 3.0
    max_correction_deg: float = 8.0
    crop_ratio: float = 0.06
    enable_mahony: bool = False
    mahony_kp: float = 1.0
    mahony_ki: float = 0.1
    acc_reject_g: float = 0.08
    axis_map: Tuple[int, int, int] = (0, 1, 2)
    axis_sign: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    # P1.1 ESKF
    use_eskf: bool = False
    eskf_sigma_g_dps: float = 0.03      # ARW (rad/sqrt(s) 转 dps)
    eskf_sigma_b_dps: float = 5e-4      # bias 漂移 (dps/sqrt(s))
    eskf_sigma_a_g: float = 0.03        # 加计噪声 (g)
    # P1.3 旋转中心
    baseline_mm: float = 64.0           # 双目基线
    imu_to_baseline_mid_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_to_camera_R: Optional[np.ndarray] = None   # 3x3, 留 P1.4 标定写
    use_recenter_to_baseline: bool = True
    scene_depth_m: float = 2.0          # 用于旋转中心修正的近似场景深度
    # P1.5 状态机
    warmup_crop_ratio: float = 0.0
    spike_recover_hold_s: float = 0.2
    spike_gyro_sat_dps: float = 200.0   # 陀螺仪饱和阈值


@dataclass
class ImuSample:
    t_us: int
    gyro_dps: np.ndarray          # (3,) 原始 dps
    acc_raw: np.ndarray           # (3,) 原始（见 acc_divisor_to_g）


@dataclass
class EyeWarp:
    """单眼变换描述：输出像素 -> 期望图裁剪窗坐标 -> H^{-1} 采样实际图。"""
    H: np.ndarray                                  # 实际图像素 -> 期望图像素
    src_w: int = 0
    src_h: int = 0
    crop_x0: int = 0
    crop_y0: int = 0
    crop_w: int = 0
    crop_h: int = 0
    out_w: int = 0
    out_h: int = 0


STATUS_OK, STATUS_DEGRADED, STATUS_WARMING, STATUS_FROZEN, STATUS_BYPASS, \
    STATUS_RECOVERING, STATUS_RECALIBRATING = \
    "ok", "degraded", "warming", "frozen", "bypass", "recovering", "recalibrating"


# ----------------------------------------------------------------------------
# 姿态积分器（对应 C++ ImuAttitude）
# ----------------------------------------------------------------------------
class ImuAttitude:
    def __init__(self, cfg: EisConfig):
        self.cfg = cfg
        self._buf: List[Tuple[int, np.ndarray, np.ndarray, np.ndarray]] = []
        # 元素: (t_us, w[rad/s], a_up[3] 或全零, q[4])
        self._calib: List[Tuple[int, np.ndarray]] = []   # (t_us, w_raw dps)
        self._calib_done = False
        self._bias = np.zeros(3)
        self._mahony_int = np.zeros(3)
        self._drops = 0
        self._last_degraded = False
        self._preloaded = False  # P0.4: 标记是否使用了持久化校准

    def preload_bias(self, bias_dps, q_align=None) -> bool:
        """从持久化存储载入零偏，跳过 2 s warming。

        Returns: True 表示成功, False 表示参数不合法。
        q_align (可选): IMU→相机 安装角四元数, 当前仅记录不参与姿态积分
        (IMU→相机外参的真正解算留给 P1.4 安装角标定)。
        """
        try:
            bias = np.asarray(bias_dps, dtype=float).reshape(3)
        except Exception:
            return False
        if not np.all(np.isfinite(bias)):
            return False
        self._bias = bias
        self._calib_done = True
        self._preloaded = True
        return True

    @property
    def is_preloaded(self) -> bool:
        return self._preloaded

    # ---- 内部 ----
    def _map_axes(self, v: np.ndarray) -> np.ndarray:
        out = np.zeros(3)
        for i in range(3):
            out[i] = self.cfg.axis_sign[i] * float(v[self.cfg.axis_map[i]])
        return out

    def _is_still(self, w_raw_dps: np.ndarray) -> bool:
        return float(np.linalg.norm(w_raw_dps)) <= self.cfg.static_gyro_dps

    # ---- 对外 ----
    def push(self, s: ImuSample) -> None:
        cfg = self.cfg
        w_raw = self._map_axes(np.asarray(s.gyro_dps, dtype=float))
        a_g = self._map_axes(np.asarray(s.acc_raw, dtype=float)) / (cfg.acc_divisor_to_g or 1000.0)
        a_mag = float(np.linalg.norm(a_g))
        a_up = a_g / a_mag if a_mag > 1e-6 else np.zeros(3)
        acc_calib_ok = abs(a_mag - 1.0) < cfg.static_acc_g
        acc_mahony_ok = abs(a_mag - 1.0) < cfg.acc_reject_g

        if self._buf and s.t_us <= self._buf[-1][0]:
            self._drops += 1
            return

        # ---- 启动静止零偏校准 ----
        if not self._calib_done:
            still = self._is_still(w_raw) and acc_calib_ok
            if not still:
                self._calib.clear()
            else:
                self._calib.append((s.t_us, w_raw.copy()))
                span = (s.t_us - self._calib[0][0]) * 1e-6
                if span >= cfg.static_calib_s:
                    self._bias = np.mean([w for _, w in self._calib], axis=0)
                    self._calib_done = True
                    self._calib.clear()
                    self._buf.append((s.t_us, (w_raw - self._bias) * math.pi / 180.0,
                                      a_up.copy(), np.array([1.0, 0, 0, 0])))
            if not self._calib_done:
                return

        # ---- 运行期样本：零偏缓更新 + 积分 ----
        w = (w_raw - self._bias) * math.pi / 180.0
        if cfg.relearn_bias and self._is_still(w_raw) and acc_calib_ok \
                and float(np.linalg.norm(w)) * 180.0 / math.pi < cfg.static_gyro_dps:
            self._bias = (1 - cfg.relearn_beta) * self._bias + cfg.relearn_beta * w_raw

        self._last_degraded = False
        t_prev, w_prev, _, q_prev = self._buf[-1] if self._buf else (None, None, None, None)
        if t_prev is not None:
            dt = (s.t_us - t_prev) * 1e-6
            if dt <= 0:
                self._drops += 1
                return
            if dt > cfg.max_gap_s:
                self._last_degraded = True        # 间隙零阶保持
            w_eff = self._mahony(w, a_up if acc_mahony_ok else None, q_prev, dt)
            q = quat_normalize(quat_mul(q_prev, quat_from_rotvec(w_eff * dt)))
        else:
            q = np.array([1.0, 0, 0, 0])
        self._buf.append((s.t_us, w, a_up if acc_mahony_ok else np.zeros(3), q))

        # 保留 ~2 s
        t_last = self._buf[-1][0]
        while len(self._buf) > 3 and (t_last - self._buf[0][0]) > 2.0e6:
            self._buf.pop(0)

    def _mahony(self, w: np.ndarray, a_up: Optional[np.ndarray], q: np.ndarray,
                dt: float) -> np.ndarray:
        cfg = self.cfg
        if not cfg.enable_mahony or a_up is None or not np.any(a_up):
            return w
        R = quat_to_mat(q)
        v_est = R[2, :]                       # R^T @ (0,0,1)
        e = np.cross(a_up, v_est)
        self._mahony_int += cfg.mahony_ki * e * dt
        return w + cfg.mahony_kp * e + self._mahony_int

    # ---- 查询 ----
    def pose_at(self, t_us: int) -> Tuple[bool, np.ndarray, bool]:
        """返回 (found, q, degraded)。q 把相机系向量变换到世界参考系。"""
        if not self._calib_done or not self._buf:
            return False, np.array([1.0, 0, 0, 0]), False
        node = None
        for item in reversed(self._buf):
            if item[0] <= t_us:
                node = item
                break
        if node is None:
            return False, np.array([1.0, 0, 0, 0]), False
        t_n, w_n, _, q_n = node
        dt_us = t_us - t_n
        if dt_us > self.cfg.max_gap_s * 2.0 * 1e6:
            return False, np.array([1.0, 0, 0, 0]), False
        q = quat_normalize(quat_mul(q_n, quat_from_rotvec(w_n * (dt_us * 1e-6))))
        return True, q, self._last_degraded

    @property
    def bias_ready(self) -> bool:
        return self._calib_done

    @property
    def gyro_bias_dps(self) -> np.ndarray:
        return self._bias.copy()

    @property
    def last_t_us(self) -> int:
        return self._buf[-1][0] if self._buf else 0

    @property
    def drop_count(self) -> int:
        return self._drops


# ----------------------------------------------------------------------------
# 稳定器（对应 C++ EisStabilizer）
# ----------------------------------------------------------------------------
class EisStabilizer:
    def __init__(self, cfg: EisConfig, K_left: np.ndarray, K_right: np.ndarray,
                 eye_w: int, eye_h: int, out_w: int, out_h: int):
        self.cfg = cfg
        # P1.1: ESKF 切换
        if cfg.use_eskf:
            from .imu_eskf import ImuEskfAttitude
            self.att = ImuEskfAttitude(cfg)
        else:
            self.att = ImuAttitude(cfg)
        self.KL = K_left if K_left is not None else self._fallback_K(eye_w, eye_h)
        self.KR = K_right if K_right is not None else self._fallback_K(eye_w, eye_h)
        self.KL_inv = np.linalg.inv(self.KL)
        self.KR_inv = np.linalg.inv(self.KR)
        self.eye_w, self.eye_h = eye_w, eye_h
        self.out_w, self.out_h = out_w, out_h
        self.q_ref = np.array([1.0, 0, 0, 0])
        self._have_ref = False
        self._boost_hold = 0
        # P1.5 状态机
        self._recover_until_us = 0
        self._last_w_raw_dps = 0.0
        self.last_fix_deg = 0.0
        self.last_status = STATUS_WARMING
        self._last_H = (None, None)
        # P1.3 旋转中心: 每个 eye 的旋转中心相对主点的像素平移 (左上为正)
        self._recenter_offsets = self._compute_recenter_offsets()

    @staticmethod
    def _fallback_K(w: int, h: int) -> np.ndarray:
        return np.array([[float(w), 0, w / 2.0], [0, float(w), h / 2.0], [0, 0, 1]])

    def _build_warp(self, K: np.ndarray, K_inv: np.ndarray, R_fix: np.ndarray,
                    crop_ratio: float, recenter_px: Optional[np.ndarray] = None) -> EyeWarp:
        H = compose_K_R_Kinv_with_kinv(K, K_inv, R_fix)
        # P1.3: 旋转中心修正: H' = T(δ) @ H, δ 是 (dx, dy) 像素平移
        if recenter_px is not None and (recenter_px[0] != 0.0 or recenter_px[1] != 0.0):
            T = np.array([[1.0, 0.0, -recenter_px[0]],
                          [0.0, 1.0, -recenter_px[1]],
                          [0.0, 0.0, 1.0]], dtype=float)
            H = T @ H
        w = EyeWarp(H=H,
                    src_w=self.eye_w, src_h=self.eye_h,
                    out_w=self.out_w, out_h=self.out_h)
        r = min(0.2, max(0.0, crop_ratio))
        w.crop_w = int(round(self.eye_w * (1 - 2 * r)))
        w.crop_h = int(round(self.eye_h * (1 - 2 * r)))
        w.crop_x0 = (self.eye_w - w.crop_w) // 2
        w.crop_y0 = (self.eye_h - w.crop_h) // 2
        return w

    @staticmethod
    def _identity_warp(base: EyeWarp) -> EyeWarp:
        w = EyeWarp(**{**base.__dict__})
        w.H = np.eye(3)
        return w

    def _zero_crop_warp(self) -> EyeWarp:
        """Warming / bypass 用：不做裁剪、不做 warp，保留原始视野。"""
        return EyeWarp(H=np.eye(3),
                       src_w=self.eye_w, src_h=self.eye_h,
                       crop_x0=0, crop_y0=0,
                       crop_w=self.eye_w, crop_h=self.eye_h,
                       out_w=self.out_w, out_h=self.out_h)

    # ---- P1.3 旋转中心修正 ----
    def _compute_recenter_offsets(self) -> Tuple[np.ndarray, np.ndarray]:
        """计算每眼相对主点的像素平移, 把旋转中心从主点修正到基线中点。

        几何:
            头部旋转轴位于两眼基线中点 O (IMU 近似 O). 真实相机主点 C 在 O 之外.
            当 IMU 报告小角度旋转 R, 场景中一点 P 在相机系投影为 p=K^{-1} P.
            若 P 在 O 旋转下不动 (深度 Z 处), 像素平移为:
                δ = (K · (I-R)) · t_eye_in_imu
            t_eye_in_imu 是 eye 主点相对 O (IMU) 的平移 (米).

        简化: 用基线中点为 O, 每眼 t_eye = ∓(baseline/2, 0, 0) + imu_offset.
        输出: (offset_left_px, offset_right_px) 在 (fx·tx/Z, fy·ty/Z) 形式.
        """
        cfg = self.cfg
        if not cfg.use_recenter_to_baseline:
            return (np.zeros(2), np.zeros(2))
        # 左眼相对基线中点的相机平移 (相机系: x 右 y 下 z 前, 单位 mm)
        half = cfg.baseline_mm * 0.5
        t_left  = np.array([-half, 0.0, 0.0], dtype=float) + np.asarray(
            cfg.imu_to_baseline_mid_mm, dtype=float)
        t_right = np.array([+half, 0.0, 0.0], dtype=float) + np.asarray(
            cfg.imu_to_baseline_mid_mm, dtype=float)
        # scene_depth_m -> mm 与 t 一致
        Z = max(0.1, float(cfg.scene_depth_m)) * 1000.0
        # 像素平移 = (fx·tx/Z, fy·ty/Z)
        def _to_px(K, t):
            return np.array([K[0, 0] * t[0] / Z, K[1, 1] * t[1] / Z], dtype=float)
        return (_to_px(self.KL, t_left), _to_px(self.KR, t_right))

    def process(self, t_start_us: int, t_end_us: int,
                burst: Sequence[ImuSample]) -> Tuple[str, EyeWarp, EyeWarp]:
        cfg = self.cfg
        for s in burst or ():
            self.att.push(s)

        warmup_crop = float(getattr(cfg, "warmup_crop_ratio", 0.0) or 0.0)

        # warming: 不做裁剪, 保留原始视野
        if not self.att.bias_ready:
            self.last_status = STATUS_WARMING
            self.last_fix_deg = 0.0
            base = self._zero_crop_warp()
            return self.last_status, base, base

        # exposure instant
        if cfg.exposure_ref == "start":
            t_exp = t_start_us
        elif cfg.exposure_ref == "end":
            t_exp = t_end_us
        else:
            t_exp = t_start_us + (t_end_us - t_start_us) // 2

        found, q_att, degraded = self.att.pose_at(t_exp)
        if not found:
            if self._last_H[0] is not None:
                wl = self._build_warp(self.KL, self.KL_inv, np.eye(3), warmup_crop)
                wr = self._build_warp(self.KR, self.KR_inv, np.eye(3), warmup_crop)
                wl.H = self._last_H[0]
                wr.H = self._last_H[1]
                self.last_status = STATUS_FROZEN
                return self.last_status, wl, wr
            self.last_status = STATUS_BYPASS
            base = self._zero_crop_warp()
            return self.last_status, base, base

        # reference follow
        e_body = rotvec_from_quat(quat_mul(quat_conj(self.q_ref), q_att))
        beta = cfg.smoothing_alpha
        if cfg.motion_boost:
            if float(np.linalg.norm(e_body)) * 180 / math.pi > cfg.motion_boost_thresh_deg:
                self._boost_hold = 10
            if self._boost_hold > 0:
                beta = min(1.0, beta * cfg.motion_boost_factor)
                self._boost_hold -= 1
        if not self._have_ref:
            self.q_ref = q_att.copy()
            self._have_ref = True
        else:
            self.q_ref = quat_normalize(quat_mul(self.q_ref, quat_from_rotvec(e_body * beta)))

        # correction (clamped)
        q_fix = quat_mul(quat_conj(self.q_ref), q_att)
        v = rotvec_from_quat(q_fix)
        max_rad = cfg.max_correction_deg * math.pi / 180.0
        ang = float(np.linalg.norm(v))
        if ang > max_rad:
            v = v * (max_rad / ang)
            q_fix = quat_from_rotvec(v)
        self.last_fix_deg = float(np.linalg.norm(v)) * 180.0 / math.pi

        R_fix = quat_to_mat(q_fix)
        offset_l, offset_r = self._recenter_offsets
        wl = self._build_warp(self.KL, self.KL_inv, R_fix, cfg.crop_ratio, offset_l)
        wr = self._build_warp(self.KR, self.KR_inv, R_fix, cfg.crop_ratio, offset_r)
        self._last_H = (wl.H, wr.H)
        # P1.5: 状态机 — RECOVERING (spike 后短窗口) > OK/DEGRADED
        if burst and t_end_us < self._recover_until_us:
            self.last_status = STATUS_RECOVERING
        else:
            # 简易 spike 检测: 上一帧的 burst 中 |ω| > saturation 阈值
            if burst:
                last_w = float(np.max(np.linalg.norm(
                    np.asarray([s.gyro_dps for s in burst[-3:]]), axis=1)))
                if last_w > cfg.spike_gyro_sat_dps:
                    self._recover_until_us = t_end_us + int(
                        cfg.spike_recover_hold_s * 1e6)
                    self.last_status = STATUS_RECOVERING
                else:
                    self.last_status = STATUS_DEGRADED if degraded else STATUS_OK
            else:
                self.last_status = STATUS_DEGRADED if degraded else STATUS_OK
        return self.last_status, wl, wr


# ----------------------------------------------------------------------------
# 图像 warp（OpenCV 单次重映射；与 C++ image_warp 等价，供原型/集成用）
# ----------------------------------------------------------------------------
def warp_eye_bgr(frame_eye_bgr: np.ndarray, w: EyeWarp, border: str = "replicate") -> np.ndarray:
    """按 EyeWarp 输出 out_w x out_h 的 BGR 图（cv2.warpPerspective 一次完成
    旋转补偿 + 中央裁剪 + 缩放）。

    目标：dst(u,v) 采样自 p_src = Hinv @ C @ [u,v,1]，其中
    C = [[sx,0,tx],[0,sy,ty],[0,0,1]] 是"输出->期望图裁剪窗"的映射。
    cv2.warpPerspective(M) 内部计算 src(M^-1 p_dst)，故传 M = (Hinv @ C)^-1 = C^-1 @ H。
    """
    import cv2
    sx = w.crop_w / float(w.out_w)
    sy = w.crop_h / float(w.out_h)
    tx = w.crop_x0 - 0.5 + 0.5 * sx
    ty = w.crop_y0 - 0.5 + 0.5 * sy
    C = np.array([[sx, 0, tx], [0, sy, ty], [0, 0, 1.0]])
    M = np.linalg.inv(np.linalg.inv(w.H) @ C)   # == C^-1 @ H
    mode = cv2.BORDER_REPLICATE if border == "replicate" else cv2.BORDER_CONSTANT
    return cv2.warpPerspective(frame_eye_bgr, M, (w.out_w, w.out_h),
                               flags=cv2.INTER_LINEAR, borderMode=mode)
