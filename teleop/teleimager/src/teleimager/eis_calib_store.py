#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""eis_calib_store.py — EIS 校准状态持久化（P0.4）。

目标：让 teleimager 冷启动到 STATUS_OK < 500 ms，避免每次启动 2 s warming。

存储位置：~/.config/xr/eis_calib.json (yaml 字段 stabilization.calibration_path 可覆盖)
负载：
    {
      "version": 1,
      "calibrations": {
        "<key>": {
          "bias_dps": [bx, by, bz],
          "q_align":  [w, x, y, z],        # 可选：IMU→相机 安装角四元数
          "saved_at_unix": 1700000000.0,
          "n_samples":    1100,             # 校准时累积的样本数 (调试用)
          "device":       "scam"            # 设备标识
        }
      }
    }

约定：load() 会做 age 校验（默认 7 天），过期返回 None。
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from typing import Optional


CALIB_VERSION = 1


@dataclass
class EisCalib:
    bias_dps: list            # (3,) 陀螺仪零偏, dps
    q_align: Optional[list]   # (4,) IMU→相机安装角四元数 (w,x,y,z), 可选
    saved_at_unix: float
    age_h: float              # load 时计算
    n_samples: int
    device: str

    def is_fresh(self, max_age_days: int) -> bool:
        return (time.time() - self.saved_at_unix) <= max_age_days * 86400.0


class EisCalibStore:
    def __init__(self, path: str):
        self.path = os.path.expanduser(path)
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)

    def _read_all(self) -> dict:
        if not os.path.exists(self.path):
            return {"version": CALIB_VERSION, "calibrations": {}}
        try:
            with open(self.path, "r") as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return {"version": CALIB_VERSION, "calibrations": {}}
        if not isinstance(data, dict):
            return {"version": CALIB_VERSION, "calibrations": {}}
        data.setdefault("version", CALIB_VERSION)
        data.setdefault("calibrations", {})
        return data

    def save(self, key: str, bias_dps, q_align=None, *,
             n_samples: int = 0, device: str = "scam") -> None:
        data = self._read_all()
        data["version"] = CALIB_VERSION
        data["calibrations"][key] = {
            "bias_dps": [float(x) for x in bias_dps],
            "q_align": ([float(x) for x in q_align]
                        if q_align is not None else None),
            "saved_at_unix": time.time(),
            "n_samples": int(n_samples),
            "device": device,
        }
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, self.path)

    def load(self, key: str, max_age_days: int = 7) -> Optional[EisCalib]:
        data = self._read_all()
        entry = data.get("calibrations", {}).get(key)
        if not entry:
            return None
        try:
            bias = list(entry["bias_dps"])
            assert len(bias) == 3
        except (KeyError, TypeError, AssertionError):
            return None
        q_align = entry.get("q_align")
        saved_at = float(entry.get("saved_at_unix", 0.0))
        age_h = (time.time() - saved_at) / 3600.0 if saved_at else float("inf")
        calib = EisCalib(
            bias_dps=bias, q_align=q_align, saved_at_unix=saved_at,
            age_h=age_h, n_samples=int(entry.get("n_samples", 0)),
            device=str(entry.get("device", "?")))
        if not calib.is_fresh(max_age_days):
            return None
        return calib

    def clear(self, key: Optional[str] = None) -> None:
        if key is None:
            data = {"version": CALIB_VERSION, "calibrations": {}}
        else:
            data = self._read_all()
            data["calibrations"].pop(key, None)
        with open(self.path, "w") as f:
            json.dump(data, f, indent=2)
