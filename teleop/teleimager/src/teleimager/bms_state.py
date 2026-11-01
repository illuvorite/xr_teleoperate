"""bms_state.py — 机器人电池与系统状态采集（只读）。

原先是独立进程 ``web-console-react/robot-agent/robot_agent.py``，单独监听 8013。
合并进 teleimager 的 60001 服务后，只需要一个端口、一套自签证书，部署时也不用
再单独拉起一个 systemd 单元。

电量数据来自 DDS 话题 ``rt/lf/bmsstate``（unitree_hg.BmsState_）：
    soc          剩余电量百分比（BMS 直接给出，不是电压估算）
    soh          电池健康度
    bmsvoltage   电池组电压（mV）
    current      电流（mA，负=放电，正=充电）
    cell_vol     单体电压（mV，13 串）
    temperature  温度（℃）
    cycle        循环次数

设计取舍（沿用原 robot_agent.py）：
  * **只读**：只订阅，不发布任何 DDS 指令，不碰电机。
  * **绝不崩**：SDK 缺失 / DDS 断连 / 无数据，只让 battery 字段降级并说明原因，
    进程继续服务 —— 否则连"服务在线"都看不到。
  * **只初始化一次**：DDS ChannelFactory 在同一进程内只能初始化一次，而 teleimager
    的每个端口都会起一个 WebRTC_PublisherThread，所以用 ensure_worker() 做全局幂等。
"""
from __future__ import annotations

import os
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

BMS_TOPIC = "rt/lf/bmsstate"
STALE_AFTER = 3.0  # BMS 报文超过 3s 没更新即视为陈旧
SERVICE_NAMES = ("xr-teleimager", "xr-heartbeat", "xr-teleop")

_STATE: Dict[str, Any] = {
    "msg": None,
    "lastTs": 0.0,
    "frames": 0,
    "source": "init",
    "error": None,
}
_LOCK = threading.Lock()

_STARTED = False
_START_LOCK = threading.Lock()


# ---------------------------------------------------------------------------
# DDS 订阅
# ---------------------------------------------------------------------------
def dds_worker(iface: str) -> None:
    """后台线程：订阅 BMS 报文，把最新一帧写进 _STATE。"""
    try:
        from unitree_sdk2py.core.channel import (  # type: ignore
            ChannelFactoryInitialize,
            ChannelSubscriber,
        )
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_  # type: ignore
    except Exception as exc:  # noqa: BLE001
        with _LOCK:
            _STATE["source"] = "unavailable"
            _STATE["error"] = f"unitree_sdk2py 不可用：{exc}"
        print(f"[bms] SDK 导入失败：{exc}", file=sys.stderr, flush=True)
        return

    def on_msg(msg: Any) -> None:
        with _LOCK:
            _STATE["msg"] = msg
            _STATE["lastTs"] = time.time()
            _STATE["frames"] = int(_STATE["frames"]) + 1
            _STATE["source"] = "bms"
            _STATE["error"] = None

    # 初始化必须带重试：机器人重启时 systemd 拉起服务，eth0 可能还没配好地址，
    # cyclonedds 会报 "eth0: does not match an available interface"。
    attempt = 0
    while True:
        try:
            ChannelFactoryInitialize(0, iface)
            break
        except Exception as exc:  # noqa: BLE001
            attempt += 1
            delay = min(2 * attempt, 30)
            with _LOCK:
                _STATE["source"] = "unavailable"
                _STATE["error"] = (
                    f"DDS 网卡 {iface} 尚未就绪（第 {attempt} 次尝试失败，{delay}s 后重试）：{exc}"
                )
            print(
                f"[bms] ChannelFactoryInitialize({iface}) 失败（第 {attempt} 次），{delay}s 后重试：{exc}",
                file=sys.stderr,
                flush=True,
            )
            time.sleep(delay)

    sub = None
    attempt = 0
    while sub is None:
        try:
            sub = ChannelSubscriber(BMS_TOPIC, BmsState_)
            # 回调式订阅：Read() 在无发布者时会永久阻塞，回调不会。
            sub.Init(on_msg, 10)
        except Exception as exc:  # noqa: BLE001
            sub = None
            attempt += 1
            delay = min(2 * attempt, 30)
            with _LOCK:
                _STATE["source"] = "unavailable"
                _STATE["error"] = f"订阅 {BMS_TOPIC} 失败（第 {attempt} 次，{delay}s 后重试）：{exc}"
            print(f"[bms] 订阅失败（第 {attempt} 次），{delay}s 后重试：{exc}", file=sys.stderr, flush=True)
            time.sleep(delay)

    with _LOCK:
        _STATE["source"] = "bms"
        _STATE["error"] = None
    print(f"[bms] 已订阅 {BMS_TOPIC}（iface={iface}）", flush=True)

    while True:  # 保活线程；数据由回调写入
        time.sleep(5)
        with _LOCK:
            if _STATE["lastTs"] and time.time() - float(_STATE["lastTs"]) > 30:
                _STATE["error"] = "超过 30s 未收到 BMS 报文（机器人可能已断电或 DDS 断开）"


def ensure_worker(iface: Optional[str] = None) -> None:
    """幂等启动 DDS 订阅线程（同进程内多次调用只生效一次）。"""
    global _STARTED
    if os.environ.get("XR_BMS_DISABLED") == "1":
        with _LOCK:
            _STATE["source"] = "disabled"
            _STATE["error"] = "XR_BMS_DISABLED=1：电池采样已关闭"
        return

    with _START_LOCK:
        if _STARTED:
            return
        _STARTED = True

    target_iface = iface or os.environ.get("XR_BMS_IFACE") or os.environ.get("XR_AGENT_IFACE") or "eth0"
    threading.Thread(
        target=dds_worker,
        args=(target_iface,),
        name="dds-bms",
        daemon=True,
    ).start()


# ---------------------------------------------------------------------------
# 系统信息
# ---------------------------------------------------------------------------
def _run(cmd: List[str], timeout: float = 2.0) -> Optional[str]:
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return (out.stdout or "").strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def service_state(name: str) -> Dict[str, Any]:
    active = _run(["systemctl", "is-active", name])
    if active is None:
        return {"ok": False, "state": "unknown", "error": "systemctl 不可用"}
    return {"ok": active == "active", "state": active, "error": None if active == "active" else active}


def cpu_temp_c() -> Optional[float]:
    for zone in Path("/sys/class/thermal").glob("thermal_zone*"):
        try:
            val = float((zone / "temp").read_text().strip())
            return round(val / 1000.0, 1) if val > 200 else round(val, 1)
        except (OSError, ValueError):
            continue
    return None


def primary_ip() -> Optional[str]:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.3)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return None


def uptime_sec() -> Optional[float]:
    try:
        return float(Path("/proc/uptime").read_text().split()[0])
    except (OSError, ValueError, IndexError):
        return None


# ---------------------------------------------------------------------------
# 电量归一化
# ---------------------------------------------------------------------------
def battery_payload() -> Dict[str, Any]:
    with _LOCK:
        msg = _STATE["msg"]
        last_ts = float(_STATE["lastTs"])
        source = str(_STATE["source"])
        error = _STATE["error"]
        frames = int(_STATE["frames"])

    age_ms = int((time.time() - last_ts) * 1000) if last_ts else None
    stale = age_ms is None or age_ms > STALE_AFTER * 1000

    if msg is None:
        return {
            "percent": None, "soh": None, "voltage": None, "current": None,
            "charging": None, "temperatures": [], "cycles": None,
            "cellMinMv": None, "cellMaxMv": None,
            "source": source, "stale": True, "ageMs": age_ms, "frames": frames,
            "error": error or "尚未收到 BMS 报文",
        }

    def g(attr: str, default: Any = None) -> Any:
        val = getattr(msg, attr, default)
        return val() if callable(val) else val

    pack_mv = list(g("bmsvoltage", []) or [])
    cells = [v for v in (g("cell_vol", []) or []) if v and v > 1000]
    temps = [v for v in (g("temperature", []) or []) if v]
    current_ma = g("current", None)

    voltage = round(pack_mv[0] / 1000.0, 2) if pack_mv and pack_mv[0] else None
    current = round(current_ma / 1000.0, 2) if isinstance(current_ma, (int, float)) else None

    return {
        "percent": g("soc"),
        "soh": g("soh"),
        "voltage": voltage,
        "current": current,
        # 负电流为放电；正电流视为充电
        "charging": (current > 0.05) if current is not None else None,
        "temperatures": temps,
        "cycles": g("cycle"),
        "cellMinMv": min(cells) if cells else None,
        "cellMaxMv": max(cells) if cells else None,
        "source": "bms",
        "stale": stale,
        "ageMs": age_ms,
        "frames": frames,
        "error": error if msg is None else (error if stale else None),
    }


def system_status() -> Dict[str, Any]:
    """原来 8013 的 /status 响应体，字段保持不变，前端无需改解析。"""
    return {
        "hostname": socket.gethostname(),
        "ip": primary_ip(),
        "ts": int(time.time() * 1000),
        "uptimeSec": uptime_sec(),
        "cpuTempC": cpu_temp_c(),
        "battery": battery_payload(),
        "services": {name: service_state(name) for name in SERVICE_NAMES},
    }
