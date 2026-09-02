#!/usr/bin/env python3
"""heartbeat_register.py — 后台心跳脚本。

启动 XR 遥操主服务后,持续把本机的 {robotId, wlan0Ip, eth0Ip, port}
推送到 autobot-guide-service 后端,使前端能拿到 VR 服务的真实访问地址。

行为:
  * 每 30 秒推送一次;成功则覆盖本地 last_heartbeat.json 缓存。
  * 失败时指数退避重试(1s/5s/30s/60s),连续失败超过 5 分钟则降级为 60s 心跳。
  * 后端不可达不会终止脚本;网络恢复后自动继续。
  * 接收 SIGTERM / SIGINT 时优雅退出。

环境变量:
  XR_TELEOP_ROBOT_ID     机器人 ID(必填)
  XR_TELEOP_WAN_IP       对外网 IP(VR 服务地址)
  XR_TELEOP_ETH0_IP      eth0 IP(ROS/DDS,可选)
  XR_TELEOP_PUBLIC_PORT  VR 对外端口,默认 8012
  XR_TELEOP_BACKEND_URL  后端地址,默认 http://127.0.0.1:19269
  XR_TELEOP_ROBOT_TOKEN  后端 Sa-Token 机器人鉴权 token
  XR_TELEOP_HEARTBEAT_INTERVAL  心跳周期(秒),默认 30
"""
from __future__ import annotations

import json
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any, Dict

import urllib.error
import urllib.request

# ---------------------------------------------------------------------------
# 配置
# ---------------------------------------------------------------------------
ROBOT_ID = os.environ.get("XR_TELEOP_ROBOT_ID", "").strip()
WAN_IP = os.environ.get("XR_TELEOP_WAN_IP", "").strip()
ETH0_IP = os.environ.get("XR_TELEOP_ETH0_IP", "").strip()
PORT = int(os.environ.get("XR_TELEOP_PUBLIC_PORT", "8012"))
BACKEND_URL = os.environ.get("XR_TELEOP_BACKEND_URL", "http://127.0.0.1:19269").rstrip("/")
ROBOT_TOKEN = os.environ.get("XR_TELEOP_ROBOT_TOKEN", "").strip()
INTERVAL = int(os.environ.get("XR_TELEOP_HEARTBEAT_INTERVAL", "30"))

CACHE_FILE = Path(
    os.environ.get(
        "XR_TELEOP_HEARTBEAT_CACHE",
        str(Path.home() / ".cache" / "xr_teleoperate" / "last_heartbeat.json"),
    )
)

REQUEST_TIMEOUT = 5  # 单次 HTTP 请求超时(秒)


# ---------------------------------------------------------------------------
# 信号处理
# ---------------------------------------------------------------------------
_running = True


def _on_signal(signum: int, _frame: Any) -> None:
    global _running
    _running = False
    print(f"[heartbeat] received signal {signum}, exiting", flush=True)


signal.signal(signal.SIGINT, _on_signal)
signal.signal(signal.SIGTERM, _on_signal)


# ---------------------------------------------------------------------------
# 心跳发送
# ---------------------------------------------------------------------------
def build_payload() -> Dict[str, Any]:
    return {
        "robotId": ROBOT_ID,
        "wlan0Ip": WAN_IP,
        "eth0Ip": ETH0_IP or None,
        "port": PORT,
        "ts": int(time.time() * 1000),
    }


def write_cache(payload: Dict[str, Any], ok: bool) -> None:
    try:
        CACHE_FILE.parent.mkdir(parents=True, exist_ok=True)
        CACHE_FILE.write_text(
            json.dumps(
                {
                    "lastPayload": payload,
                    "lastSuccessAt": int(time.time() * 1000) if ok else None,
                    "lastAttemptAt": int(time.time() * 1000),
                    "ok": ok,
                },
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )
    except OSError as exc:
        print(f"[heartbeat] WARN: failed to write cache: {exc}", flush=True)


def send_once() -> bool:
    if not ROBOT_ID or not WAN_IP:
        print("[heartbeat] skip: XR_TELEOP_ROBOT_ID or WAN_IP missing", flush=True)
        return False
    payload = build_payload()
    body = json.dumps(payload).encode("utf-8")
    url = f"{BACKEND_URL}/api/v1/teleop/hosts/register"
    headers = {"Content-Type": "application/json"}
    if ROBOT_TOKEN:
        headers["Authorization"] = f"Bearer {ROBOT_TOKEN}"
    req = urllib.request.Request(url, data=body, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=REQUEST_TIMEOUT) as resp:
            ok = 200 <= resp.status < 300
            print(
                f"[heartbeat] POST {url} -> {resp.status} (robot={ROBOT_ID} wan={WAN_IP})",
                flush=True,
            )
            write_cache(payload, ok)
            return ok
    except urllib.error.HTTPError as exc:
        print(f"[heartbeat] HTTP {exc.code} from {url}: {exc.reason}", flush=True)
        write_cache(payload, False)
        return False
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        print(f"[heartbeat] network error: {exc}", flush=True)
        write_cache(payload, False)
        return False


# ---------------------------------------------------------------------------
# 主循环
# ---------------------------------------------------------------------------
def main() -> int:
    if not ROBOT_ID:
        print("[heartbeat] WARN: XR_TELEOP_ROBOT_ID not set, using default 'g1_001'.", flush=True)
        globals()['ROBOT_ID'] = "g1_001"
    if not WAN_IP:
        print("[heartbeat] FATAL: XR_TELEOP_WAN_IP is required", file=sys.stderr, flush=True)
        return 1

    print(
        f"[heartbeat] starting: backend={BACKEND_URL} robot={ROBOT_ID} "
        f"wan={WAN_IP} eth0={ETH0_IP or '<none>'} port={PORT} interval={INTERVAL}s",
        flush=True,
    )

    backoff_plan = [1, 5, 30, 60, 60, 60]  # 失败时逐步拉长
    failure_idx = 0

    while _running:
        ok = send_once()
        if ok:
            failure_idx = 0
            sleep_for = INTERVAL
        else:
            sleep_for = backoff_plan[min(failure_idx, len(backoff_plan) - 1)]
            failure_idx += 1
            print(
                f"[heartbeat] backing off {sleep_for}s before next attempt "
                f"(failure #{failure_idx})",
                flush=True,
            )

        # 分段 sleep,以便信号能及时终止
        slept = 0.0
        while _running and slept < sleep_for:
            step = min(1.0, sleep_for - slept)
            time.sleep(step)
            slept += step

    print("[heartbeat] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())