"""host_info.py — 本机网卡 / IP 检测工具。

被 start_xr_teleop.sh 通过 PYTHONPATH 调用,也供 TeleVuer 的
``/host-info`` 端点使用,保证两边看到的 IP 是同一份。

设计:
  * ``detect_ips()`` 返回 ``{wan_iface, wan_ip, eth0_ip}``。
  * wan 网卡检测顺序(与 start_xr_teleop.sh 保持一致):
      XR_TELEOP_IFACE (默认 wlan0) -> enP* -> eth0 -> 第一个非 loopback IPv4
  * eth0 仅作 ROS / DDS 通道返回,不参与 VR 对外地址。
  * 检测失败不抛异常,所有字段都有默认值。
"""
from __future__ import annotations

import glob
import os
import re
import socket
from typing import Optional, List


_IPV4_RE = re.compile(r"(?<=inet\s)(\d+(\.\d+){3})")


def _read_ipv4(iface: str) -> Optional[str]:
    try:
        # 优先用 ``ip`` 命令(Linux 标准工具),缺失则降级。
        with os.popen(f"ip -4 addr show {iface} 2>/dev/null") as fh:
            for line in fh:
                match = _IPV4_RE.search(line)
                if match:
                    return match.group(1)
    except OSError:
        return None
    return None


def _read_default_route_ip() -> Optional[str]:
    try:
        with os.popen("ip route get 1.1.1.1 2>/dev/null") as fh:
            for line in fh:
                if "src" in line:
                    parts = line.split()
                    if "src" in parts:
                        return parts[parts.index("src") + 1]
    except OSError:
        return None
    return None


def _list_candidate_ifaces() -> List[str]:
    """枚举可能的物理网卡(按优先级排序)。"""
    iface_env = os.environ.get("XR_TELEOP_IFACE", "wlan0")
    out: List[str] = []
    seen = set()

    def push(name: str) -> None:
        if name and name not in seen:
            seen.add(name)
            out.append(name)

    push(iface_env)
    # enP* 是 Jetson / 嵌入式设备常见的 PCI-E 网卡命名
    for cand in sorted(glob.glob("/sys/class/net/enP*")):
        push(os.path.basename(cand))
    push("eth0")
    push("wlan0")
    return out


def _hostname_i() -> Optional[str]:
    try:
        hostname = socket.gethostname()
        return socket.gethostbyname(hostname)
    except (socket.gaierror, OSError):
        return None


def detect_ips() -> dict:
    """返回当前主机的网卡信息。永远不抛异常。"""
    wan_ip: Optional[str] = None
    wan_iface: Optional[str] = None
    for candidate in _list_candidate_ifaces():
        ip = _read_ipv4(candidate)
        if ip and not ip.startswith("127."):
            wan_ip = ip
            wan_iface = candidate
            break
    if not wan_ip:
        wan_ip = _read_default_route_ip()
    if not wan_ip:
        host_ip = _hostname_i()
        if host_ip and not host_ip.startswith("127."):
            wan_ip = host_ip

    eth0_ip = _read_ipv4("eth0")

    return {
        "wan_iface": wan_iface,
        "wan_ip": wan_ip or "127.0.0.1",
        "eth0_ip": eth0_ip,
    }


def public_host() -> str:
    """TeleVuer 对外使用的 host(优先 XR_TELEOP_PUBLIC_HOST,否则自动检测)。"""
    override = os.environ.get("XR_TELEOP_PUBLIC_HOST")
    if override:
        return override
    return detect_ips()["wan_ip"]


def public_port() -> int:
    raw = os.environ.get("XR_TELEOP_PUBLIC_PORT", "8012")
    try:
        return int(raw)
    except ValueError:
        return 8012


def robot_id() -> str:
    return os.environ.get("XR_TELEOP_ROBOT_ID", "").strip()