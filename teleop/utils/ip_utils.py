"""xr_teleoperate - 本地 IPv4 地址自动检测

优先读取指定网卡，失败则回退到默认路由源地址。
所有依赖 IP 的模块统一使用此工具，避免散落硬编码。
"""

import socket
import os
import subprocess
from typing import Optional


def detect_local_ip(interface: Optional[str] = None) -> str:
    """检测本机对外可访问的 IPv4 地址（非回环）。

    检测顺序：
    1. 指定网卡的 IPv4（通过 socket bind）
    2. UDP connect 外部地址获取路由 IP
    3. 解析 /proc/net/fib_trie
    4. hostname -I
    5. 最终回退到 192.168.2.203
    """
    # 方法1: 指定网卡
    if interface:
        try:
            if os.path.exists(f"/sys/class/net/{interface}"):
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                try:
                    s.bind((interface, 0))
                    ip = s.getsockname()[0]
                    if ip and not ip.startswith("127."):
                        return ip
                except Exception:
                    pass
                finally:
                    s.close()
        except Exception:
            pass

    # 方法2: UDP 探测（获取默认路由源 IP）
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            if ip and not ip.startswith("127."):
                return ip
        finally:
            s.close()
    except Exception:
        pass

    # 方法3: /proc/net/fib_trie
    try:
        with open("/proc/net/fib_trie", "r") as f:
            lines = f.readlines()
        for i, line in enumerate(lines):
            if "/32 host" in line and i + 1 < len(lines):
                ip = lines[i + 1].strip().split()[-1]
                if ip and not ip.startswith("127."):
                    return ip
    except Exception:
        pass

    # 方法4: hostname -I
    try:
        result = subprocess.run(
            ["hostname", "-I"],
            capture_output=True,
            text=True,
            timeout=2,
        )
        if result.returncode == 0:
            ip = result.stdout.strip().split()[0]
            if ip and not ip.startswith("127."):
                return ip
    except Exception:
        pass

    return "192.168.2.203"


def resolve_img_server_ip(raw: str, interface: Optional[str] = None) -> str:
    """将用户输入的 IP 解析为实际值。支持 'auto'、空字符串或直接 IP。"""
    if not raw or raw.strip().lower() == "auto":
        return detect_local_ip(interface)
    return raw.strip()
