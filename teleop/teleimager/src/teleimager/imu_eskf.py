#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""imu_eskf.py — 6-state Error-State Kalman Filter (P1.1)

状态：
    x = [δθ (3), b_g (3)]   误差状态 (15 维总姿态在外)

真值状态：
    q (4,)                  四元数姿态 (相机系 → 世界系)
    b_g (3,)                陀螺仪零偏 (dps)

过程模型 (IMU 1000 Hz, dt)：
    q_pred = q ⊗ exp((ω − b_g) · dt · π/180)
    b_g_pred = b_g
    F = I + dt · [[-⌈ω̂⌉, -I], [0, 0]]   (6x6)
    P_pred = F · P · F^T + Q

量测模型 (加速度计, 仅静止判定通过时)：
    a_pred = R(q)^T · g_world            (3,)
    a_meas = a_imu / ‖a_imu‖ · |g_acc|   (归一化, 然后用 ‖a_meas‖ 是否 ~1g 自适应)
    e = a_meas − a_pred
    H = [∂a_pred/∂δθ | 0]                (3x6)
    K = P · H^T · (H·P·H^T + R_a)^-1
    δx = K · e
    q = q ⊗ exp(δθ)
    b_g += δb_g
    P = (I − K·H) · P

陀螺仪噪声（ICM-42688-P 典型值）：
    ARW ≈ 0.03 dps/√Hz
    BI  ≈ 0.5 °/h = 0.5/3600 dps
Q (过程噪声, 6x6)：
    Q_θ = (σ_g · dt)^2 · I
    Q_b = (σ_b · dt)^2 · I
    Q_θb 交叉项: 0
R (量测噪声, 3x3)：
    R_a = σ_a^2 · I   σ_a ≈ 0.02 g

接口与原 ImuAttitude 一致：push / pose_at / gyro_bias_dps / bias_ready / drop_count / preload_bias
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np

from .imu_stabilizer import ImuSample


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def _quat_from_rotvec(r: np.ndarray) -> np.ndarray:
    th = float(np.linalg.norm(r))
    if th < 1e-12:
        return np.array([1.0, r[0] * 0.5, r[1] * 0.5, r[2] * 0.5], dtype=float)
    h = 0.5 * th
    s = math.sin(h) / th
    return np.array([math.cos(h), r[0] * s, r[1] * s, r[2] * s], dtype=float)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ], dtype=float)


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([1.0, 0, 0, 0], dtype=float)
    return q / n


def _quat_to_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


class ImuEskfAttitude:
    """与 ImuAttitude 同接口的 ESKF 实现。"""

    def __init__(self, cfg):
        self.cfg = cfg
        # 真值状态
        self._q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self._bias = np.zeros(3, dtype=float)
        # 误差状态协方差 (6x6)
        self._P = np.diag([
            (5 * math.pi / 180) ** 2,   # δθ 初始 5°
            (5 * math.pi / 180) ** 2,
            (5 * math.pi / 180) ** 2,
            (1.0) ** 2,                  # b_g 初始 1 dps
            (1.0) ** 2,
            (1.0) ** 2,
        ]).astype(float)
        # 噪声参数
        self._sigma_g_dps = float(getattr(cfg, "eskf_sigma_g_dps", 0.03))   # ARW
        self._sigma_b_dps = float(getattr(cfg, "eskf_sigma_b_dps", 5e-4))   # bias 漂移
        self._sigma_a_g = float(getattr(cfg, "eskf_sigma_a_g", 0.03))       # 加计噪声
        self._g_world = np.array([0.0, 0.0, 1.0], dtype=float)              # 世界系重力 (z 上)
        # 状态
        self._buf: List[Tuple[int, np.ndarray]] = []   # (t_us, q)
        self._drops = 0
        self._last_degraded = False
        self._calib_done = False
        self._preloaded = False
        self._g_mag_acc_for_norm = 9.81  # 占位, 实际在 push 时用 |a|·g_unit
        self._has_first = False

    # --- 对外接口 ---
    def preload_bias(self, bias_dps, q_align=None) -> bool:
        try:
            b = np.asarray(bias_dps, dtype=float).reshape(3)
        except Exception:
            return False
        if not np.all(np.isfinite(b)):
            return False
        self._bias = b
        self._calib_done = True
        self._preloaded = True
        return True

    def push(self, s: ImuSample) -> None:
        cfg = self.cfg
        w_raw = np.asarray(s.gyro_dps, dtype=float)
        a_raw = np.asarray(s.acc_raw, dtype=float) / (cfg.acc_divisor_to_g or 1000.0)

        if self._buf and s.t_us <= self._buf[-1][0]:
            self._drops += 1
            return

        # 静止判定 (与原 ImuAttitude 一致)
        a_mag = float(np.linalg.norm(a_raw))
        acc_calib_ok = abs(a_mag - 1.0) < cfg.static_acc_g
        w_norm_dps = float(np.linalg.norm(w_raw))
        is_still = (w_norm_dps <= cfg.static_gyro_dps) and acc_calib_ok

        if not self._has_first:
            # 第一个样本: 若有零偏, 直接积分; 无则等待
            if not self._calib_done:
                if is_still:
                    # 启动期: 用静止样本做零偏估计 (简化: 单样本即零偏)
                    self._bias = w_raw.copy()
                    self._calib_done = True
                else:
                    self._drops += 1
                    return
            self._q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
            self._has_first = True
            self._buf.append((s.t_us, self._q.copy()))
            return

        # 计算 dt
        t_prev, _ = self._buf[-1]
        dt = (s.t_us - t_prev) * 1e-6
        if dt <= 0:
            self._drops += 1
            return
        if dt > cfg.max_gap_s:
            self._last_degraded = True
        # dt 上限保护: 大间隔视为重启, 不积分
        if dt > 0.5:
            self._has_first = False
            self._buf.clear()
            return

        # --- 过程更新 ---
        w = (w_raw - self._bias) * math.pi / 180.0   # rad/s
        self._q = _quat_normalize(_quat_mul(self._q, _quat_from_rotvec(w * dt)))

        # F (6x6): δθ_dot = -⌈ω⌉·δθ − δb_g;   b_g_dot = 0
        F = np.eye(6)
        F[0:3, 0:3] -= _skew(w) * dt
        F[0:3, 3:6] = -np.eye(3) * dt

        # Q (6x6): 仅对角线
        sg = self._sigma_g_dps * math.pi / 180.0
        sb = self._sigma_b_dps
        Q = np.diag([(sg * dt) ** 2] * 3 + [(sb * dt) ** 2] * 3)
        self._P = F @ self._P @ F.T + Q

        # --- 量测更新 (仅静止时) ---
        if is_still and a_mag > 1e-6:
            a_unit = a_raw / a_mag
            R_q = _quat_to_mat(self._q)
            a_pred = R_q.T @ self._g_world
            e = a_unit - a_pred
            # Mahalanobis 门限: 3σ 拒绝
            H = np.zeros((3, 6))
            H[0:3, 0:3] = -_skew(a_pred)
            R_a = (self._sigma_a_g ** 2) * np.eye(3)
            S = H @ self._P @ H.T + R_a
            # 异常: 3σ
            d2 = float(e @ np.linalg.solve(S, e))
            if d2 < 9.0:
                K = self._P @ H.T @ np.linalg.inv(S)
                dx = K @ e
                # 状态注入
                self._q = _quat_normalize(_quat_mul(self._q, _quat_from_rotvec(dx[0:3])))
                self._bias = self._bias + dx[3:6]
                I_KH = np.eye(6) - K @ H
                self._P = I_KH @ self._P @ I_KH.T + K @ R_a @ K.T   # Joseph form

        # 运行期零偏缓更新 (仅静止)
        if cfg.relearn_bias and is_still and w_norm_dps < cfg.static_gyro_dps:
            self._bias = (1 - cfg.relearn_beta) * self._bias + cfg.relearn_beta * w_raw

        self._last_degraded = False
        self._buf.append((s.t_us, self._q.copy()))

        # 保留 ~2 s
        t_last = self._buf[-1][0]
        while len(self._buf) > 3 and (t_last - self._buf[0][0]) > 2.0e6:
            self._buf.pop(0)

    def pose_at(self, t_us: int) -> Tuple[bool, np.ndarray, bool]:
        if not self._calib_done or not self._buf:
            return False, np.array([1.0, 0, 0, 0]), False
        node = None
        for item in reversed(self._buf):
            if item[0] <= t_us:
                node = item
                break
        if node is None:
            return False, np.array([1.0, 0, 0, 0]), False
        t_n, q_n = node
        dt_us = t_us - t_n
        if dt_us > self.cfg.max_gap_s * 2.0 * 1e6:
            return False, np.array([1.0, 0, 0, 0]), False
        # ESKF 输出姿态不需要再前向积分: 用最近节点即可
        return True, q_n.copy(), self._last_degraded

    @property
    def bias_ready(self) -> bool:
        return self._calib_done

    @property
    def is_preloaded(self) -> bool:
        return self._preloaded

    @property
    def gyro_bias_dps(self) -> np.ndarray:
        return self._bias.copy()

    @property
    def last_t_us(self) -> int:
        return self._buf[-1][0] if self._buf else 0

    @property
    def drop_count(self) -> int:
        return self._drops
