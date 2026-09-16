#!/usr/bin/env python3
"""heartbeat_register.py — 后台心跳脚本。

启动 XR 遥操主服务后,持续把本机的 {robotId, wanIp, eth0Ip, port}
推送到 autobot-guide-service 后端,使前端能拿到 VR 服务的真实访问地址。

行为:
  * 每 30 秒推送一次;成功则覆盖本地 last_heartbeat.json 缓存。
  * 失败时指数退避重试(1s/5s/30s/60s),连续失败超过 5 分钟则降级为 60s 心跳。
  * 后端不可达不会终止脚本;网络恢复后自动继续。
  * 接收 SIGTERM / SIGINT 时优雅退出。

环境变量:
  XR_TELEOP_ROBOT_ID     机器人 ID(必填,默认 g1_001)
  XR_TELEOP_WAN_IP       对外网 IP(VR 服务地址)。
                         **可以不配** —— 留空时用 teleop.utils.host_info 自动检测,
                         与 televuer 的 /host-info 端点共用同一份实现,保证两边 IP 一致。
  XR_TELEOP_ETH0_IP      eth0 IP(ROS/DDS,可选),留空同样自动检测。
  XR_TELEOP_PUBLIC_PORT  VR 对外端口,默认 8012
  XR_TELEOP_BACKEND_URL  后端地址,默认 http://127.0.0.1:19269
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
INTERVAL = int(os.environ.get("XR_TELEOP_HEARTBEAT_INTERVAL", "30"))

CACHE_FILE = Path(
    os.environ.get(
        "XR_TELEOP_HEARTBEAT_CACHE",
        str(Path.home() / ".cache" / "xr_teleoperate" / "last_heartbeat.json"),
    )
)

REQUEST_TIMEOUT = 5  # 单次 HTTP 请求超时(秒)


# ---------------------------------------------------------------------------
# 网卡地址自动检测
# ---------------------------------------------------------------------------
def detect_host_ips() -> Dict[str, str]:
    """自动检测本机的对外网 IP 与 eth0 IP。

    复用 ``teleop.utils.host_info``(与 televuer 的 ``/host-info`` 端点同一份实现),
    保证心跳上报的地址和页面实际使用的地址一致。

    systemd 启动时 ``sys.path[0]`` 是 ``scripts/`` 目录,仓库根不在路径上,因此这里
    主动把仓库根插进去,避免依赖 PYTHONPATH 配置。

    检测失败不抛异常,返回空 dict,由调用方决定是否 FATAL。
    """
    repo_root = Path(__file__).resolve().parent.parent
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    try:
        from teleop.utils.host_info import detect_ips

        info = detect_ips()
    except Exception as exc:  # noqa: BLE001 — 检测失败不应让心跳进程崩溃
        print(f"[heartbeat] WARN: 自动检测网卡地址失败: {exc}", flush=True)
        return {}
    return {
        "wan_ip": str(info.get("wan_ip") or "").strip(),
        "eth0_ip": str(info.get("eth0_ip") or "").strip(),
    }


def resolve_addresses() -> None:
    """补齐 WAN_IP / ETH0_IP:环境变量优先,缺失则自动检测。

    历史问题:这两个变量在 env.conf / systemd unit / 安装脚本里都没有被赋值,
    而 main() 又要求 WAN_IP 必填,导致 xr-heartbeat 服务一启动就 FATAL 退出,
    心跳从未真正上报。改为「可缺省 + 自动检测」后开箱即可工作。
    """
    global WAN_IP, ETH0_IP

    need_wan = not WAN_IP
    need_eth0 = not ETH0_IP
    if not (need_wan or need_eth0):
        return

    detected = detect_host_ips()
    if need_wan:
        WAN_IP = detected.get("wan_ip", "")
        if WAN_IP:
            print(f"[heartbeat] XR_TELEOP_WAN_IP 未配置,自动检测为 {WAN_IP}", flush=True)
    if need_eth0:
        ETH0_IP = detected.get("eth0_ip", "")
        if ETH0_IP:
            print(f"[heartbeat] XR_TELEOP_ETH0_IP 未配置,自动检测为 {ETH0_IP}", flush=True)


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
    """构造注册载荷。

    字段名以 ``wanIp`` 为准(与后端 TeleopHostRegisterDTO.wanIp、teleop_host.wan_ip、
    TeleopHostVO.wanIp 对齐)。同时额外带一份 ``wlan0Ip`` 别名,兼容尚未升级、
    仍只认旧字段名的后端 —— 多带一个冗余字段没有副作用,但可以让机器人与后端的
    升级顺序不再互相阻塞。
    """
    return {
        "robotId": ROBOT_ID,
        "wanIp": WAN_IP,
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

    # WAN_IP / ETH0_IP 允许缺省:缺失时自动检测,不再因为没配环境变量直接退出。
    resolve_addresses()
    if not WAN_IP or WAN_IP.startswith("127."):
        print(
            "[heartbeat] FATAL: 无法确定对外网 IP(wlan0)。请检查机器人网络,"
            "或在 /etc/xr-teleoperate/env.conf 中显式设置 XR_TELEOP_WAN_IP。",
            file=sys.stderr,
            flush=True,
        )
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