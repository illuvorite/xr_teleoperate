import vuer
from vuer import Vuer
from vuer.schemas import Box, Html, ImageBackground, Hands, MotionControllers, RandomizedLight, Urdf, WebRTCVideoPlane, WebRTCStereoVideoPlane, div, span
from multiprocessing import Value, Array, Process, shared_memory
import numpy as np
import asyncio
import json
import logging
import math
import re
import shutil
import threading
import time
import cv2
import os
from pathlib import Path
from typing import Any, Literal
from xml.etree import ElementTree
from urllib.parse import urlparse

from teleop.utils import host_info  # noqa: E402  (after stdlib imports)

# Dashboard HUD constants: a glass-card look consistent with the autobot console.
_DASHBOARD_FONT = "-apple-system, BlinkMacSystemFont, 'PingFang SC', sans-serif"
_DASHBOARD_TONES = {
    "ok": "#30d158",
    "warn": "#ff9f0a",
    "off": "#7d8490",
}

# Hide the vuer client's built-in programs dock (the "always-open" list of connected scene
# elements) and the corner Vuer link/logo. The class names are the CSS-module hashes baked
# into vuer==0.0.60 and are stable only for that pinned version.
#
# The menu toggle and the leva "Camera Control" panel are intentionally kept accessible so
# the operator can drag the initial view to a good framing (the camera persists per browser
# via the client's localStorage).
_VUER_UI_HIDE_CSS = """
._programConsole_eu9ak_307,   /* programs dock: the auto-open panel listing scene elements */
._iframeVuerLinkButton_gxqj6_1 /* corner Vuer link/logo */
{ display: none !important; }
"""

# The vuer 0.0.60 client shows its default grid plane unless the URL carries `?grid=false`.
# That is baked into the client bundle, so without patching it the grid would always show
# on a bare `?ws=` URL. This regex disables the default in the minified JS (verified against
# the installed build; 4 matches across the entry chunks).
_VUER_GRID_DISABLE_RE = re.compile(
    r'\(\(([A-Za-z_$]+)=([A-Za-z_$]+)\.grid\)==null\?void 0:\1\.toLowerCase\(\)\)!==\"false\"'
)

# 桌面浏览器里默认视点静止(OrbitControls.autoRotate=false),拖拽才转。
# 把 OrbitControls 类定义的默认 autoRotate 改成 true:所有实例开启自动环绕,
# 浏览器不拖拽画面也会自己转动(视点环绕);PICO 或主动拖拽时仍可覆盖。
# 注入点是类定义的唯一串(three/examples OrbitControls 源码):
#   Nn(this,"autoRotate",!1),Nn(this,"autoRotateSpeed",2)
# => Nn(this,"autoRotate",!0),Nn(this,"autoRotateSpeed",1.6)
_ORBIT_AUTOROTATE_RE = re.compile(
    r'Nn\(this,"autoRotate",!1\),Nn\(this,"autoRotateSpeed",2\)'
)
_ORBIT_AUTOROTATE_REPL = 'Nn(this,"autoRotate",!0),Nn(this,"autoRotateSpeed",1.6)'

# 电量上报注入脚本:让"真实头显直接打开 8012 页面"时,把 PICO 电量经 /ws/aux
# 推给 televuer,再由 televuer 广播给 autobot 控制台(/proxy/vr/{id}/_aux_ws)。
#
# 为什么必须做"非 iframe + 非代理"判断:
#   autobot 控制台是通过后端反向代理(/proxy/vr/{robotId}/)把该页面嵌进 iframe 的,
#   那种情况下页面跑在运营人员的电脑浏览器里,若照常上报,控制台会把运营电脑的
#   电量当成头显电量。因此只在"顶层窗口 + 非 /proxy/vr/ 路径"时才上报。
#
# 为什么还要做"仅头显"判断(2026-09-19 修):
#   即使排除了 iframe 与代理页面,"运营电脑直接在标签页打开 https://{wanIp}:8012/"
#   仍会上报 —— 那台电脑通常插着电源(Battery API 报 100%),于是与头显的真实电量
#   (如 24%)交替覆盖同一份服务端缓存,控制台就会看到"电量在 24% 与 100% 之间跳变"。
#   与 televuer 侧 `_is_headset_ua()` 同口径:只允许 XR/头显类客户端上报。
#
# 取电优先级:
#   1. window.__XR_TELEOP_BATTERY__ —— 可选的外部注入(PICO Web SDK 等),便于
#      后续在不动本文件的前提下换成厂商接口;
#   2. navigator.getBattery() —— Battery Status API(Chromium 系浏览器)。
#   两者都拿不到时不上报(宁可空着,也不要写假数据)。
_BATTERY_REPORT_JS = r"""
(() => {
  if (window.__xrTeleopBatteryInstalled) return;
  window.__xrTeleopBatteryInstalled = true;

  try {
    if (window.top !== window.self) return;
    if (location.pathname.indexOf('/proxy/vr/') === 0) return;
  } catch (e) { return; }

  // 只让"看起来是头显"的客户端上报(桌面浏览器一律不上报,避免污染头显电量)。
  const HEADSET_UA_RE = /pico|quest|oculus|vision ?pro|hmd|headset|android[^)]*\b(vr|xr)\b/i;
  if (!HEADSET_UA_RE.test(navigator.userAgent || '')) {
    console.info('[xr-teleop-battery] 非头显客户端,跳过头显电量上报。');
    return;
  }

  const WS_PATH = '/ws/aux';
  const KEEPALIVE_MS = 20000;
  const RECONNECT_MAX_MS = 30000;

  let ws = null;
  let attempt = 0;
  let keepalive = null;
  let batteryManager = null;

  function deviceLabel() {
    const ua = navigator.userAgent || '';
    const pico = ua.match(/PICO[^;)]*/i);
    if (pico) return pico[0];
    if (/quest|oculus/i.test(ua)) return 'Meta Quest';
    if (/vision ?pro/i.test(ua)) return 'Apple Vision Pro';
    return 'XR 设备';
  }

  function snapshot() {
    const ext = window.__XR_TELEOP_BATTERY__;
    if (ext && typeof ext.get === 'function') {
      try {
        const d = ext.get();
        if (d && typeof d.level === 'number' && isFinite(d.level)) {
          return {
            level: Math.max(0, Math.min(100, Math.round(d.level))),
            charging: typeof d.charging === 'boolean' ? d.charging : null,
            deviceName: d.deviceName || deviceLabel(),
          };
        }
      } catch (e) { /* 回落到 navigator.getBattery */ }
    }
    if (!batteryManager) return null;
    const level = Math.round(batteryManager.level * 100);
    if (!isFinite(level)) return null;
    return {
      level: Math.max(0, Math.min(100, level)),
      charging: !!batteryManager.charging,
      deviceName: deviceLabel(),
    };
  }

  function push(reason) {
    if (!ws || ws.readyState !== 1) return;
    const data = snapshot();
    if (!data) return;
    data.updatedAt = Date.now();
    try {
      ws.send(JSON.stringify({ type: 'battery-update', data: data, reason: reason }));
    } catch (e) { /* 已断线,等重连 */ }
  }

  function scheduleReconnect() {
    const delay = Math.min(1000 * Math.pow(2, attempt), RECONNECT_MAX_MS);
    attempt += 1;
    setTimeout(connect, delay);
  }

  function connect() {
    const proto = location.protocol === 'https:' ? 'wss:' : 'ws:';
    try {
      ws = new WebSocket(proto + '//' + location.host + WS_PATH);
    } catch (e) {
      scheduleReconnect();
      return;
    }
    ws.onopen = () => {
      attempt = 0;
      push('open');
      if (keepalive) clearInterval(keepalive);
      keepalive = setInterval(() => push('tick'), KEEPALIVE_MS);
    };
    ws.onclose = () => {
      if (keepalive) { clearInterval(keepalive); keepalive = null; }
      scheduleReconnect();
    };
  }

  if (navigator.getBattery) {
    navigator.getBattery().then((bm) => {
      batteryManager = bm;
      bm.addEventListener('levelchange', () => push('levelchange'));
      bm.addEventListener('chargingchange', () => push('chargingchange'));
      push('init');
    }).catch(() => {
      console.warn('[xr-teleop-battery] 读取电池信息失败,电量不会上报。');
    });
  } else {
    console.warn('[xr-teleop-battery] Battery Status API 不可用,电量不会上报。');
  }

  connect();
})();
"""

# 注意:注入脚本(_BATTERY_REPORT_JS / _AUTO_ORBIT_JS / _VUER_UI_HIDE_CSS)改动后必须
# 提升该 marker,否则已存在的 client_patch 缓存会被直接复用,新脚本不会生效。
_PATCHED_CLIENT_MARKER = ".patched-v9"

# 自动视点环绕注入脚本:周期性向 vuer 的 <canvas> 派发微弱 pointer 事件序列,
# 让原生相机拖拽逻辑以为鼠标在缓慢拖动。用户真实按下时暂停,抬起后恢复。
# 实现细节:
#   - 只对 pointer 事件生效,不影响 WebXR(PICO)的控制器输入。
#   - 每帧(xr cadence)推进一次微小偏移;约 10-20s 完成一整圈,幅度可控。
#   - 事件用 CanvasPointerEvent 构造,带 clientX/clientY,与真实拖拽一致。
_AUTO_ORBIT_JS = r"""
(() => {
  if (window.__vuerAutoOrbitInstalled) return;
  window.__vuerAutoOrbitInstalled = true;

  const SPEED = 0.5;      // deg/frame 的视点移动量 (负值反向)
  const STEP_MS = 100;    // 事件推进间隔(毫秒)
  let canvas = null;
  let active = true;      // 全局开关(未发现 canvas 时保持等待)

  function findCanvas() {
    const c = document.querySelector('canvas');
    if (c) return c;
    return null;
  }

  // 模拟一次"持续拖拽":pointerdown(dx,dy) → n 次 pointermove → pointerup
  function dispatchPointer(win, type, x, y) {
    try {
      const evt = new PointerEvent(type, {
        bubbles: true, cancelable: true,
        composed: true,
        pointerId: 1, pointerType: 'mouse',
        isPrimary: true,
        clientX: x, clientY: y,
        screenX: x, screenY: y,
        button: type === 'pointerup' ? -1 : 0,
        buttons: type === 'pointerup' ? 0 : 1,
      });
      canvas.dispatchEvent(evt);
      if (typeof win !== 'undefined' && win.document) {
        win.document.dispatchEvent(new PointerEvent(type, {
          bubbles: true, cancelable: true,
          clientX: x, clientY: y, pointerId: 1, pointerType: 'mouse', isPrimary: true,
          button: type === 'pointerup' ? -1 : 0,
          buttons: type === 'pointerup' ? 0 : 1,
        }));
      }
    } catch (e) { /* noop */ }
  }

  let userInteracting = false;
  window.addEventListener('pointerdown', () => { userInteracting = true; }, true);
  window.addEventListener('pointerup', () => { userInteracting = false; }, true);
  window.addEventListener('wheel', () => { userInteracting = true; }, true);

  let x = Math.floor(window.innerWidth * 0.2);
  let y = Math.floor(window.innerHeight * 0.5);
  let running = false;

  function step() {
    if (!canvas) {
      canvas = findCanvas();
      if (!canvas) { setTimeout(step, 500); return; }
    }
    if (userInteracting) { setTimeout(step, STEP_MS); return; }   // 用户操作中,暂停
    if (!active) { setTimeout(step, STEP_MS); return; }

    // 每个 STEP_MS 产生一次小幅移动的"拖拽"节奏:
    // 每次: down(i) → move(i + dx) → up —— 形成连续微拖动
    if (!running) {
      running = true;
      dispatchPointer(window, 'pointerdown', x, y);
      requestAnimationFrame(() => {
        const dx = Math.cos(performance.now() / 9000) * SPEED;
        const dy = Math.sin(performance.now() / 9000) * SPEED * 0.8;
        x += dx; y += dy;
        dispatchPointer(window, 'pointermove', x, y);
        dispatchPointer(window, 'pointerup', x, y);
        running = false;
      });
    }
    setTimeout(step, STEP_MS);
  }

  setTimeout(step, 800);   // 给 vuer 初始化留出时间

  // 监听环境变量类的开关(可在控制台改):window.__vuerAutoOrbitActive
  Object.defineProperty(window, '__vuerAutoOrbitActive', {
    get: () => active,
    set: (v) => { active = !!v; },
  });
})();
"""


def _auto_orbit_enabled_global() -> bool:
    return os.getenv("XR_TELEOP_AUTO_ORBIT", "1").strip() in ("1", "true", "yes", "on")


def _device_type_from_ua(ua: str) -> str:
    """Best-effort device type from a browser User-Agent string."""
    if not ua:
        return "未知设备"
    low = ua.lower()
    if "picobrowser" in low or "pico" in low:
        return "PICO 头显"
    if "quest" in low or "oculus" in low or "meta vr" in low:
        return "Meta Quest"
    if "vision pro" in low or "visionpro" in low:
        return "Apple Vision Pro"
    if "android" in low and ("vr" in low or "xr" in low):
        return "XR/安卓设备"
    if "mobile" in low:
        return "移动端"
    return ua.strip()[:80] or "未知设备"


# 与注入脚本 _BATTERY_REPORT_JS 里的 HEADSET_UA_RE 保持同一口径:只有头显类客户端
# 才允许上报/被接受 PICO 电量。
#
# 为什么需要这道门:在"非 iframe + 非 /proxy/vr/ 路径"两个条件下,直接用桌面浏览器
# 打开 https://{wanIp}:8012/ 仍会上报该电脑自己的电量(通常插电 = 100%),与头显的
# 真实电量交替覆盖服务端缓存 → 控制台看到 24% ↔ 100% 跳变。
#
# 环境变量 XR_TELEOP_BATTERY_ALLOW_ANY_UA=1 可放开该限制(型号识别不准时的应急开关)。
_HEADSET_UA_RE = re.compile(
    r"pico|quest|oculus|vision ?pro|hmd|headset|android[^)]*\b(vr|xr)\b",
    re.IGNORECASE,
)


def _is_headset_ua(ua: str) -> bool:
    """该 User-Agent 是否来自 XR 头显(决定是否接受其电量上报)。"""
    if os.getenv("XR_TELEOP_BATTERY_ALLOW_ANY_UA", "0").strip() in ("1", "true", "yes", "on"):
        return True
    return bool(ua) and bool(_HEADSET_UA_RE.search(ua))


def _patched_client_root() -> Path:
    """Return a patched copy of vuer's client build with client defaults changed.

    Patches are applied once into a cache directory, without touching the installed
    package and without requiring any URL query parameters:

      1. the client's default grid plane is disabled (the client otherwise shows it on any
         URL that lacks ``?grid=false``);
      2. a stylesheet hides the vuer built-in menu/dock (see _VUER_UI_HIDE_CSS);
      3. auto-orbit script so the desktop viewport orbits by itself (see _AUTO_ORBIT_JS);
      4. battery reporting script that pushes the headset battery to ``/ws/aux``
         (see _BATTERY_REPORT_JS).

    The server is pointed at this copy via ``Vuer.client_root`` before the Vuer instance is
    constructed, so both the root page and the ``/assets`` bundles are served from it.

    .. note::
       改动任一注入脚本后必须同时提升 ``_PATCHED_CLIENT_MARKER``,否则已存在的
       ``~/.cache/xr_teleoperate/client_patch`` 会被直接复用,新脚本不会生效。
    """
    original = Path(vuer.__file__).resolve().parent / "client_build"
    patched_dir = Path.home() / ".cache" / "xr_teleoperate" / "client_patch"
    marker = patched_dir / _PATCHED_CLIENT_MARKER
    if marker.exists():
        return patched_dir

    shutil.copytree(original, patched_dir, dirs_exist_ok=True)

    # 1. Disable the default grid: `a = <grid-expr>` becomes `a = false`.
    for js in patched_dir.glob("assets/**/*.js"):
        src = js.read_text(encoding="utf-8", errors="replace")
        patched, count = _VUER_GRID_DISABLE_RE.subn("false", src)
        if count or patched != src:
            js.write_text(patched, encoding="utf-8")

    # 1b. Enable OrbitControls.autoRotate by default so the desktop viewport
    # slowly orbits on its own (no dragging required). Only the chunk bytes
    # are patched; the marker bump below forces a rebuild of the cache.
    for js in patched_dir.glob("assets/**/*.js"):
        src = js.read_text(encoding="utf-8", errors="replace")
        patched, count = _ORBIT_AUTOROTATE_RE.subn(_ORBIT_AUTOROTATE_REPL, src)
        if count:
            js.write_text(patched, encoding="utf-8")
            print(f"[televuer] orbit autoRotate patch applied in {js.name} (x{count})")

    # 2. Inject the menu-hiding stylesheet into the root page.
    index = patched_dir / "index.html"
    html = index.read_text(encoding="utf-8")
    if "vuer-ui-hide" not in html:
        html = html.replace(
            "</head>",
            f'<style id="vuer-ui-hide">{_VUER_UI_HIDE_CSS}</style></head>',
            1,
        )

    # 2b. 桌面浏览器自动视点环绕(auto orbit):vuer 的主相机由鼠标拖拽(pointer
    # events)驱动,不拖就不动。注入一段小脚本,周期性向 <canvas> 派发微弱
    # pointermove 序列,让 vuer 的原生拖拽响应"以为鼠标在缓慢拖动"——
    # 视点自动环绕,无需用户按住。用户真实拖拽时脚本暂停,让出控制权。
    # 通过 XR_TELEOP_AUTO_ORBIT 环境变量可关闭(默认开启)。
    if _auto_orbit_enabled_global():
        auto_orbit_js = _AUTO_ORBIT_JS
        if "vuer-auto-orbit" not in html:
            html = html.replace(
                "</body>",
                f'<script id="vuer-auto-orbit">{auto_orbit_js}</script></body>',
                1,
            )

    # 2c. 电量上报:头显直接打开本页面时,把电量经 /ws/aux 推给本进程,再由本进程
    #     广播给 autobot 控制台。脚本内部会排除 iframe 嵌入与 /proxy/vr/ 代理访问,
    #     避免把运营电脑的电量误报成头显电量(详见 _BATTERY_REPORT_JS 注释)。
    if "xr-teleop-battery" not in html:
        html = html.replace(
            "</body>",
            f'<script id="xr-teleop-battery">{_BATTERY_REPORT_JS}</script></body>',
            1,
        )

    index.write_text(html, encoding="utf-8")

    marker.write_text("v9", encoding="utf-8")
    return patched_dir


class TeleVuer:
    # "正在摇操"的时间窗(秒):最近这么长时间内收到过手柄/手势运动数据才算在摇操。
    # 控制台的"摇操中"徽标与它对齐(过窗后回落为"设备已接入"),避免用锁存值后
    # 头显断开也一直显示"摇操中"。
    _TELEOP_ACTIVE_WINDOW_S = float(os.getenv("XR_TELEOP_ACTIVE_WINDOW_S", "6"))

    def __init__(self, use_hand_tracking: bool, binocular: bool=True, img_shape: tuple=None, display_fps: float=30.0,
                        display_mode: Literal["immersive", "pass-through", "ego"]="immersive", zmq: bool=False, webrtc: bool=False, webrtc_url: str=None,
                        cert_file: str=None, key_file: str=None, http_mode: bool=False,
                        webrtc_immersive_height: float=1.8, webrtc_ego_height: float=1.0,
                        webrtc_immersive_distance: float=2.0, webrtc_ego_distance: float=1.8,
                        static_root: str | None=None, dashboard: dict[str, Any] | None=None,
                        webrtc_url_left: str=None, webrtc_url_right: str=None, stereo_split_webrtc: bool=False):
        """
        TeleVuer class for OpenXR-based XR teleoperate applications.
        This class handles the communication with the Vuer server and manages image and pose data.

        :param use_hand_tracking: bool, whether to use hand tracking or controller tracking.
        :param binocular: bool, whether the application is binocular (stereoscopic) or monocular.
        :param img_shape: tuple, shape of the head image (height, width).
        :param display_fps: float, target frames per second for display updates (default: 30.0).
        
        :param display_mode: str, controls the VR viewing mode. Options are "immersive", "pass-through", and "ego".
        :param zmq: bool, whether to use zmq for image transmission.
        :param webrtc: bool, whether to use webrtc for real-time communication.
        :param webrtc_url: str, URL for the webrtc offer. must be provided if webrtc is True.
        :param cert_file: str, path to the SSL certificate file.
        :param key_file: str, path to the SSL key file.

        Note:

        - display_mode controls what the VR headset displays:
            * "immersive": fully immersive mode; VR shows the robot's first-person view (zmq or webrtc must be enabled).
            * "pass-through": VR shows the real world through the VR headset cameras; no image from zmq or webrtc is displayed (even if enabled).
            * "ego": a small window in the center shows the robot's first-person view, while the surrounding area shows the real world.
        
        - Only one image mode is active at a time.
        - Image transmission to VR occurs only if display_mode is "immersive" or "ego" and the corresponding zmq or webrtc option is enabled.
        - If zmq and webrtc simultaneously enabled, webrtc will be prioritized.

        --------------              -------------------           --------------       -----------------                     -------
         display_mode       |        display behavior         |    image to VR     |      image source        |               Notes
        --------------              -------------------           --------------       -----------------                     ------- 
           immersive        |   fully immersive view (robot)  |     Yes (full)     |     zmq or webrtc        |   if both enabled, webrtc prioritized
        --------------              -------------------           --------------       -----------------                     -------
         pass-through       |       Real world view (VR)      |         No         |          N/A             |  even if image source enabled, don't display
        --------------              -------------------           --------------       -----------------                     -------
              ego           |      ego view (robot + VR)      |    Yes (small)     |     zmq or webrtc        |   if both enabled, webrtc prioritized
        --------------              -------------------           --------------       -----------------                     -------

        """
        self.use_hand_tracking = use_hand_tracking
        self.binocular = binocular
        if img_shape is None:
            raise ValueError("[TeleVuer] img_shape must be provided.")
        self.img_shape = (img_shape[0], img_shape[1], 3)
        self.display_fps = display_fps
        self.img_height = self.img_shape[0]
        if self.binocular:
            self.img_width  = self.img_shape[1] // 2
        else:
            self.img_width  = self.img_shape[1]
        self.aspect_ratio = self.img_width / self.img_height
        self.display_mode = display_mode
        self.zmq = zmq
        self.webrtc = webrtc
        self.webrtc_url = webrtc_url
        self.webrtc_url_left = webrtc_url_left or webrtc_url
        self.webrtc_url_right = webrtc_url_right or webrtc_url
        self.stereo_split_webrtc = stereo_split_webrtc
        self.webrtc_immersive_height = webrtc_immersive_height
        self.webrtc_ego_height = webrtc_ego_height
        self.webrtc_immersive_distance = webrtc_immersive_distance
        self.webrtc_ego_distance = webrtc_ego_distance
        self.static_root = self._normalize_static_root(static_root)
        self.dashboard = self._normalize_dashboard_config(dashboard, self.static_root)

        # SSL certificate path resolution
        if http_mode:
            cert_file = None
            key_file = None
        else:
            env_cert = os.getenv("XR_TELEOP_CERT")
            env_key = os.getenv("XR_TELEOP_KEY")
            if cert_file is None or key_file is None:
                # 1.Try environment variables
                if env_cert and env_key:
                    cert_file = cert_file or env_cert
                    key_file = key_file or env_key
                else:
                    # 2.Try ~/.config/xr_teleoperate/
                    user_conf_dir = Path.home() / ".config" / "xr_teleoperate"
                    cert_path_user = user_conf_dir / "cert.pem"
                    key_path_user = user_conf_dir / "key.pem"

                    if cert_path_user.exists() and key_path_user.exists():
                        cert_file = cert_file or str(cert_path_user)
                        key_file = key_file or str(key_path_user)
                    else:
                        # 3.Fallback to package root (current logic)
                        current_module_dir = Path(__file__).resolve().parent.parent.parent
                        cert_file = cert_file or str(current_module_dir / "cert.pem")
                        key_file = key_file or str(current_module_dir / "key.pem")

        # Vuer's default URL points to vuer.ai when it runs on port 8012. That
        # is useful for the hosted client, but a Pico must load the local page
        # so its relative WebSocket connects to this process. Derive the host
        # from the WebRTC endpoint and allow an explicit public URL override.
        public_url = os.getenv("XR_TELEOP_PUBLIC_URL")
        if public_url:
            parsed_public_url = urlparse(public_url)
            public_host = parsed_public_url.hostname
            public_scheme = parsed_public_url.scheme or "https"
            public_port = parsed_public_url.port or 8012
        else:
            parsed_webrtc_url = urlparse(webrtc_url or "")
            public_host = parsed_webrtc_url.hostname or os.getenv("XR_TELEOP_PUBLIC_HOST", "127.0.0.1")
            public_scheme = "https" if cert_file else "http"
            public_port = 8012
        if not public_host:
            raise ValueError("[TeleVuer] XR_TELEOP_PUBLIC_URL must include a hostname.")

        vuer_kwargs = dict(host='0.0.0.0', port=public_port, domain=f"{public_scheme}://{public_host}:{public_port}",
                           cert=cert_file, key=key_file,
                           queries=dict(grid=False, initCamPos="0,1.5,2.4", initCamRot="-12,0,0"), queue_len=3)
        if self.static_root is not None:
            vuer_kwargs["static_root"] = str(self.static_root)
        # Serve the client from a patched copy of vuer's build: default grid off and the
        # built-in menu hidden, with no URL query parameters required. Vuer.client_root is
        # set before construction so the /assets route also serves the patched bundles.
        Vuer.client_root = _patched_client_root()
        self.vuer = Vuer(**vuer_kwargs)
        # 摇操设备身份:记录连接设备的 User-Agent(按 ws 对象映射),供 /connection 接口返回。
        self._ws_ua: dict = {}
        self._last_ua = ""
        self._teleop_ua = ""
        self._teleop_at = 0.0
        # 最近一次收到手柄/手势运动数据的时间(monotonic)。motion_data_ready 是"曾经
        # 收到过"的锁存值,不适合直接当"正在摇操"用,这里再记一个时间窗。
        self._motion_at = 0.0
        self._register_json_route("/connection", self._connection_info_json)
        # ------------------------------------------------------------------
        # autobot-guide-service-front 用的辅助通道:
        #   /host-info — 当前主机的 WAN/eth0 IP + 对外端口(供前端拼 iframe 地址)
        #   /battery   — PICO 头显电量(由浏览器内的 PICO Web SDK 主动上报,服务侧缓存)
        #   /ws/aux    — 与 iframe 平行的独立 WebSocket,前端业务代码直连,
        #                用于接收 PICO 电量等高频遥测推送
        # ------------------------------------------------------------------
        self._host_info_cache = host_info.detect_ips()
        self._battery_state: dict = {
            "level": None,
            "charging": None,
            "deviceName": None,
            "updatedAt": 0,
            "source": "none",  # none | browser | fallback
            "reportedBy": None,  # 上报方的 User-Agent(诊断用,便于定位"谁在报电量")
        }
        self._aux_ws_clients: set = set()
        self._aux_lock = threading.Lock()
        # id(ws) -> {"ua": str, "headset": bool, "device": str}:区分上报方身份,
        # 只接受头显类客户端的电量上报(详见 _is_headset_ua)。
        self._aux_ws_meta: dict = {}
        self._battery_rejected_ua: set = set()
        self._register_json_route("/host-info", self._host_info_json)
        self._register_json_route("/battery", self._battery_json)
        # 注册独立 WebSocket 路由:不走 Vuer 的下行 / 上行协议。
        self._register_aux_websocket()
        self._wrap_downlink()
        # 启动后自检自定义路由是否真的可达:路由被 vuer 自身的 catch-all / 静态路由
        # 抢占,或在旧版本上根本没注册成功时,这里会明确打日志(否则现象只是
        # "控制台电量不出数 / 设备状态显示无设备连接",无法定位)。
        self._schedule_route_selfcheck(public_scheme, public_port)
        self.vuer.add_handler("CAMERA_MOVE")(self.on_cam_move)
        if self.use_hand_tracking:
            self.vuer.add_handler("HAND_MOVE")(self.on_hand_move)
        else:
            self.vuer.add_handler("CONTROLLER_MOVE")(self.on_controller_move)

        if self.display_mode == "immersive":
            if self.dashboard["enabled"]:
                if not self.webrtc:
                    raise ValueError("[TeleVuer] static dashboard requires webrtc=True.")
                if self.binocular:
                    raise ValueError("[TeleVuer] static dashboard currently supports only monocular cameras.")
                fn = self.main_static_dashboard_monocular_webrtc
            elif self.webrtc:
                if self.binocular and self.stereo_split_webrtc:
                    fn = self.main_image_binocular_webrtc_split
                elif self.binocular:
                    fn = self.main_image_binocular_webrtc
                else:
                    fn = self.main_image_monocular_webrtc
            elif self.zmq:
                self.img2display_shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
                self.img2display = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.img2display_shm.buf)
                self.latest_frame = None
                self.new_frame_event = threading.Event()
                self.stop_writer_event = threading.Event()
                self.writer_thread = threading.Thread(target=self._xr_render_loop, daemon=True)
                self.writer_thread.start()
                fn = self.main_image_binocular_zmq if self.binocular else self.main_image_monocular_zmq
            else:
                raise ValueError("[TeleVuer] immersive mode requires zmq=True or webrtc=True.")
        elif self.display_mode == "ego":
            if self.webrtc:
                if self.binocular and self.stereo_split_webrtc:
                    fn = self.main_image_binocular_webrtc_ego_split
                elif self.binocular:
                    fn = self.main_image_binocular_webrtc_ego
                else:
                    fn = self.main_image_monocular_webrtc_ego
            elif self.zmq:
                self.img2display_shm = shared_memory.SharedMemory(create=True, size=np.prod(self.img_shape) * np.uint8().itemsize)
                self.img2display = np.ndarray(self.img_shape, dtype=np.uint8, buffer=self.img2display_shm.buf)
                self.latest_frame = None
                self.new_frame_event = threading.Event()
                self.stop_writer_event = threading.Event()
                self.writer_thread = threading.Thread(target=self._xr_render_loop, daemon=True)
                self.writer_thread.start()
                fn = self.main_image_binocular_zmq_ego if self.binocular else self.main_image_monocular_zmq_ego
            else:
                raise ValueError("[TeleVuer] ego mode requires zmq=True or webrtc=True.")
        elif self.display_mode == "pass-through":
            fn = self.main_pass_through
        else:
            raise ValueError(f"[TeleVuer] Unknown display_mode: {self.display_mode}")
        
        self.vuer.spawn(start=False)(fn)

        self.head_pose_shared = Array('d', 16, lock=True)
        self.left_arm_pose_shared = Array('d', 16, lock=True)
        self.right_arm_pose_shared = Array('d', 16, lock=True)
        self.motion_data_ready_shared = Value('b', False, lock=True)
        if self.use_hand_tracking:
            self.left_hand_position_shared = Array('d', 75, lock=True)
            self.right_hand_position_shared = Array('d', 75, lock=True)
            self.left_hand_orientation_shared = Array('d', 25 * 9, lock=True)
            self.right_hand_orientation_shared = Array('d', 25 * 9, lock=True)

            self.left_hand_pinch_shared = Value('b', False, lock=True)
            self.left_hand_pinchValue_shared = Value('d', 0.0, lock=True)
            self.left_hand_squeeze_shared = Value('b', False, lock=True)
            self.left_hand_squeezeValue_shared = Value('d', 0.0, lock=True)

            self.right_hand_pinch_shared = Value('b', False, lock=True)
            self.right_hand_pinchValue_shared = Value('d', 0.0, lock=True)
            self.right_hand_squeeze_shared = Value('b', False, lock=True)
            self.right_hand_squeezeValue_shared = Value('d', 0.0, lock=True)
        else:
            self.left_ctrl_trigger_shared = Value('b', False, lock=True)
            self.left_ctrl_triggerValue_shared = Value('d', 0.0, lock=True)
            self.left_ctrl_squeeze_shared = Value('b', False, lock=True)
            self.left_ctrl_squeezeValue_shared = Value('d', 0.0, lock=True)
            self.left_ctrl_thumbstick_shared = Value('b', False, lock=True)
            self.left_ctrl_thumbstickValue_shared = Array('d', 2, lock=True)
            self.left_ctrl_aButton_shared = Value('b', False, lock=True)
            self.left_ctrl_bButton_shared = Value('b', False, lock=True)

            self.right_ctrl_trigger_shared = Value('b', False, lock=True)
            self.right_ctrl_triggerValue_shared = Value('d', 0.0, lock=True)
            self.right_ctrl_squeeze_shared = Value('b', False, lock=True)
            self.right_ctrl_squeezeValue_shared = Value('d', 0.0, lock=True)
            self.right_ctrl_thumbstick_shared = Value('b', False, lock=True)
            self.right_ctrl_thumbstickValue_shared = Array('d', 2, lock=True)
            self.right_ctrl_aButton_shared = Value('b', False, lock=True)
            self.right_ctrl_bButton_shared = Value('b', False, lock=True)

        self.process = Process(target=self._vuer_run)
        self.process.daemon = True
        self.process.start()
    
    def _vuer_run(self):
        try:
            self.vuer.run()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Vuer encountered an error: {e}")
        finally:
            if hasattr(self, "stop_writer_event"):
                self.stop_writer_event.set()

    def _xr_render_loop(self):
        while not self.stop_writer_event.is_set():
            if not self.new_frame_event.wait(timeout=0.1):
                continue
            self.new_frame_event.clear()
            if self.latest_frame is None:
                continue
            latest_frame = self.latest_frame
            latest_frame = cv2.cvtColor(latest_frame, cv2.COLOR_BGR2RGB)
            self.img2display[:] = latest_frame
    
    def render_to_xr(self, image):
        if self.webrtc or self.display_mode == "pass-through":
            print("[TeleVuer] Warning: render_to_xr is ignored when webrtc is enabled or pass_through is True.")
            return
        self.latest_frame = image
        self.new_frame_event.set()

    @staticmethod
    def _normalize_static_root(static_root: str | None) -> Path | None:
        if static_root is None:
            return None
        root = Path(static_root).expanduser().resolve()
        if not root.is_dir():
            raise ValueError(f"[TeleVuer] static_root does not exist or is not a directory: {root}")
        return root

    @staticmethod
    def _vector(value: Any, name: str, length: int=3) -> list[float]:
        if not isinstance(value, (list, tuple)) or len(value) != length:
            raise ValueError(f"[TeleVuer] dashboard {name} must contain {length} numeric values.")
        try:
            return [float(item) for item in value]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"[TeleVuer] dashboard {name} must contain numeric values.") from exc

    @staticmethod
    def _positive_float(value: Any, name: str) -> float:
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"[TeleVuer] dashboard {name} must be numeric.") from exc
        if number <= 0:
            raise ValueError(f"[TeleVuer] dashboard {name} must be greater than zero.")
        return number

    @staticmethod
    def _validate_urdf_meshes(urdf_file: Path, static_root: Path) -> None:
        try:
            root = ElementTree.parse(urdf_file).getroot()
        except ElementTree.ParseError as exc:
            raise ValueError(f"[TeleVuer] failed to parse URDF: {urdf_file}") from exc

        missing_meshes = []
        for mesh in root.iter("mesh"):
            filename = mesh.attrib.get("filename")
            if not filename:
                continue
            mesh_path = Path(filename)
            if mesh_path.is_absolute() or ".." in mesh_path.parts:
                raise ValueError(f"[TeleVuer] URDF mesh path must remain relative to the assets root: {filename}")
            local_mesh = (urdf_file.parent / mesh_path).resolve()
            try:
                local_mesh.relative_to(static_root)
            except ValueError as exc:
                raise ValueError(f"[TeleVuer] URDF mesh path escapes static_root: {filename}") from exc
            if not local_mesh.is_file():
                missing_meshes.append(filename)

        if missing_meshes:
            sample = ", ".join(missing_meshes[:5])
            suffix = "" if len(missing_meshes) <= 5 else f" (+{len(missing_meshes) - 5} more)"
            raise ValueError(f"[TeleVuer] URDF references missing mesh files: {sample}{suffix}")

    def _normalize_dashboard_config(self, dashboard: dict[str, Any] | None, static_root: Path | None) -> dict[str, Any]:
        if dashboard is None:
            return {"enabled": False}
        if not isinstance(dashboard, dict):
            raise ValueError("[TeleVuer] dashboard configuration must be a mapping.")

        dashboard_settings = dashboard.get("dashboard", {})
        if not isinstance(dashboard_settings, dict):
            raise ValueError("[TeleVuer] dashboard.dashboard must be a mapping.")
        enabled = dashboard_settings.get("enabled", False)
        if not isinstance(enabled, bool):
            raise ValueError("[TeleVuer] dashboard.dashboard.enabled must be a boolean.")
        if not enabled:
            return {"enabled": False}
        if static_root is None:
            raise ValueError("[TeleVuer] static dashboard requires static_root.")

        static_url_prefix = dashboard_settings.get("static_url_prefix", "/static")
        if static_url_prefix != "/static":
            raise ValueError("[TeleVuer] Vuer 0.0.60 serves static_root only at /static.")

        robot_model = dashboard.get("robot_model", {})
        camera_screen = dashboard.get("camera_screen", {})
        status_panel = dashboard.get("status_panel", {})
        if not all(isinstance(section, dict) for section in (robot_model, camera_screen, status_panel)):
            raise ValueError("[TeleVuer] dashboard sections must be mappings.")

        urdf_relative_path = robot_model.get("urdf")
        if not isinstance(urdf_relative_path, str) or not urdf_relative_path:
            raise ValueError("[TeleVuer] robot_model.urdf must be a non-empty relative path.")
        urdf_path = Path(urdf_relative_path)
        if urdf_path.is_absolute() or ".." in urdf_path.parts:
            raise ValueError("[TeleVuer] robot_model.urdf must remain inside static_root.")
        urdf_file = (static_root / urdf_path).resolve()
        try:
            urdf_file.relative_to(static_root)
        except ValueError as exc:
            raise ValueError("[TeleVuer] robot_model.urdf must remain inside static_root.") from exc
        if not urdf_file.is_file():
            raise ValueError(f"[TeleVuer] robot_model.urdf does not exist: {urdf_file}")
        self._validate_urdf_meshes(urdf_file, static_root)

        material_type = robot_model.get("material_type", "standard")
        if not isinstance(material_type, str) or not material_type:
            raise ValueError("[TeleVuer] robot_model.material_type must be a non-empty string.")

        camera_rotation = camera_screen.get("rotation")
        if camera_rotation is not None:
            camera_rotation = self._vector(camera_rotation, "camera_screen.rotation")

        return {
            "enabled": True,
            "robot_model": {
                "enabled": robot_model.get("enabled", True) is not False,
                "src": f"{static_url_prefix}/{urdf_path.as_posix()}",
                "position": self._vector(robot_model.get("position", [-1.2, 0.0, -2.2]), "robot_model.position"),
                "rotation": self._vector(robot_model.get("rotation", [0.0, 0.0, 0.0]), "robot_model.rotation"),
                "scale": self._positive_float(robot_model.get("scale", 0.65), "robot_model.scale"),
                "material_type": material_type,
            },
            "camera_screen": {
                "enabled": camera_screen.get("enabled", True) is not False,
                "position": self._vector(camera_screen.get("position", [0.0, 1.35, -2.0]), "camera_screen.position"),
                "rotation": camera_rotation,
                "height": self._positive_float(camera_screen.get("height", self.webrtc_immersive_height), "camera_screen.height"),
                "distance": self._positive_float(camera_screen.get("distance", self.webrtc_immersive_distance), "camera_screen.distance"),
            },
            "status_panel": {
                "enabled": status_panel.get("enabled", True) is not False,
                "position": self._vector(status_panel.get("position", [1.35, 1.15, -2.0]), "status_panel.position"),
                "scale": self._positive_float(status_panel.get("scale", 0.5), "status_panel.scale"),
            },
        }

    @staticmethod
    def _dashboard_status_row(label: str, value: str, tone: str = "off") -> Any:
        accent = _DASHBOARD_TONES.get(tone, _DASHBOARD_TONES["off"])
        return div(
            div(
                span("●", style={"color": accent, "marginRight": "7px", "fontSize": "8px", "lineHeight": "1"}),
                span(label, style={"color": "#9aa4b2", "fontSize": "12px", "fontWeight": 500, "fontFamily": _DASHBOARD_FONT}),
                style={"display": "flex", "alignItems": "center", "gap": "5px", "minWidth": "0"},
            ),
            span(value, style={"color": "#e8eaf0", "fontSize": "12px", "fontWeight": 600, "fontFamily": _DASHBOARD_FONT, "whiteSpace": "nowrap"}),
            style={"display": "flex", "justifyContent": "space-between", "alignItems": "center", "gap": "12px", "padding": "6px 0"},
        )

    def _dashboard_status_panel(self, *, motion_data_ready=False, left_trigger=False, left_trigger_value=0.0, left_squeeze=False, left_squeeze_value=0.0, left_a_button=False, left_b_button=False, left_thumbstick=False, left_thumbstick_value=(0.0, 0.0), right_trigger=False, right_trigger_value=0.0, right_squeeze=False, right_squeeze_value=0.0, right_a_button=False, right_b_button=False, right_thumbstick=False, right_thumbstick_value=(0.0, 0.0), lowstate="Waiting", arm_feedback="Inactive", model_update="Stopped") -> Html:
        panel = self.dashboard["status_panel"]
        live_tone = "ok" if motion_data_ready else "warn"

        def row(label, value, tone="off"):
            return self._dashboard_status_row(label, value, tone)

        def section(title, accent):
            return div(
                title,
                style={"fontSize": "10px", "fontWeight": 700, "letterSpacing": "0.8px", "textTransform": "uppercase",
                       "color": accent, "margin": "12px 0 4px", "fontFamily": _DASHBOARD_FONT},
            )

        return Html(
            div(
                div(
                    div("G1 29 · Dex3 遥操作", style={"fontSize": "13px", "fontWeight": 600, "color": "#f2f4f8", "fontFamily": _DASHBOARD_FONT}),
                    div(
                        span("●", style={"color": _DASHBOARD_TONES[live_tone], "fontSize": "9px", "marginRight": "5px", "lineHeight": "1"}),
                        span("在线" if motion_data_ready else "等待中", style={"fontSize": "11px", "fontWeight": 600, "color": _DASHBOARD_TONES[live_tone], "fontFamily": _DASHBOARD_FONT}),
                        style={"display": "flex", "alignItems": "center"},
                    ),
                    style={"display": "flex", "justifyContent": "space-between", "alignItems": "center", "gap": "10px"},
                ),
                section("系统状态", "#30d158"),
                row("XR 连接", "已连接" if motion_data_ready else "等待中", live_tone),
                row("摄像头", "WEBRTC", "ok"),
                row("DDS", "已连接", "ok"),
                section("机器人", "#bf5af2"),
                row("电机状态", lowstate, "warn" if lowstate == "Waiting" else "ok"),
                row("臂反馈", arm_feedback, "ok" if arm_feedback == "Active" else "off"),
                row("模型", model_update, "ok" if model_update != "Stopped" else "off"),
                section("手柄", "#0a84ff"),
                row("左扳机", "按下" if left_trigger else "释放", "ok" if left_trigger else "off"),
                row("左握力", f"{left_squeeze_value:.2f}", "ok" if left_squeeze_value > 0 else "off"),
                row("右扳机", "按下" if right_trigger else "释放", "ok" if right_trigger else "off"),
                row("右握力", f"{right_squeeze_value:.2f}", "ok" if right_squeeze_value > 0 else "off"),
                style={
                    "width": "264px",
                    "padding": "18px 20px",
                    "borderRadius": "16px",
                    "background": "rgba(16, 20, 26, 0.78)",
                    "backdropFilter": "blur(18px)",
                    "-webkit-backdrop-filter": "blur(18px)",
                    "border": "1px solid rgba(255, 255, 255, 0.09)",
                    "boxShadow": "0 12px 40px rgba(0, 0, 0, 0.45), 0 0 0 0.5px rgba(255, 255, 255, 0.04) inset",
                    "pointerEvents": "none",
                    "fontFamily": _DASHBOARD_FONT,
                },
            ),
            key="robot-status-panel",
            transform=True,
            position=panel["position"],
            scale=panel["scale"],
        )

    def _dashboard_status_data(self) -> dict:
        if self.use_hand_tracking:
            with self.left_hand_pinch_shared.get_lock():
                left_trigger = bool(self.left_hand_pinch_shared.value)
                left_trigger_value = float(self.left_hand_pinchValue_shared.value)
            with self.left_hand_squeeze_shared.get_lock():
                left_squeeze = bool(self.left_hand_squeeze_shared.value)
                left_squeeze_value = float(self.left_hand_squeezeValue_shared.value)
            with self.right_hand_pinch_shared.get_lock():
                right_trigger = bool(self.right_hand_pinch_shared.value)
                right_trigger_value = float(self.right_hand_pinchValue_shared.value)
            with self.right_hand_squeeze_shared.get_lock():
                right_squeeze = bool(self.right_hand_squeeze_shared.value)
                right_squeeze_value = float(self.right_hand_squeezeValue_shared.value)
            left_a_button = False
            left_b_button = False
            left_thumbstick = False
            left_thumbstick_value = (0.0, 0.0)
            right_a_button = False
            right_b_button = False
            right_thumbstick = False
            right_thumbstick_value = (0.0, 0.0)
        else:
            with self.left_ctrl_trigger_shared.get_lock():
                left_trigger = bool(self.left_ctrl_trigger_shared.value)
                left_trigger_value = float(self.left_ctrl_triggerValue_shared.value)
            with self.left_ctrl_squeeze_shared.get_lock():
                left_squeeze = bool(self.left_ctrl_squeeze_shared.value)
                left_squeeze_value = float(self.left_ctrl_squeezeValue_shared.value)
            with self.left_ctrl_aButton_shared.get_lock():
                left_a_button = bool(self.left_ctrl_aButton_shared.value)
            with self.left_ctrl_bButton_shared.get_lock():
                left_b_button = bool(self.left_ctrl_bButton_shared.value)
            with self.left_ctrl_thumbstick_shared.get_lock():
                left_thumbstick = bool(self.left_ctrl_thumbstick_shared.value)
            with self.left_ctrl_thumbstickValue_shared.get_lock():
                left_thumbstick_value = (float(self.left_ctrl_thumbstickValue_shared[0]), float(self.left_ctrl_thumbstickValue_shared[1]))
            with self.right_ctrl_trigger_shared.get_lock():
                right_trigger = bool(self.right_ctrl_trigger_shared.value)
                right_trigger_value = float(self.right_ctrl_triggerValue_shared.value)
            with self.right_ctrl_squeeze_shared.get_lock():
                right_squeeze = bool(self.right_ctrl_squeeze_shared.value)
                right_squeeze_value = float(self.right_ctrl_squeezeValue_shared.value)
            with self.right_ctrl_aButton_shared.get_lock():
                right_a_button = bool(self.right_ctrl_aButton_shared.value)
            with self.right_ctrl_bButton_shared.get_lock():
                right_b_button = bool(self.right_ctrl_bButton_shared.value)
            with self.right_ctrl_thumbstick_shared.get_lock():
                right_thumbstick = bool(self.right_ctrl_thumbstick_shared.value)
            with self.right_ctrl_thumbstickValue_shared.get_lock():
                right_thumbstick_value = (float(self.right_ctrl_thumbstickValue_shared[0]), float(self.right_ctrl_thumbstickValue_shared[1]))

        with self.motion_data_ready_shared.get_lock():
            motion_data_ready = bool(self.motion_data_ready_shared.value)

        return {
            "motion_data_ready": motion_data_ready,
            "left_trigger": left_trigger, "left_trigger_value": left_trigger_value,
            "left_squeeze": left_squeeze, "left_squeeze_value": left_squeeze_value,
            "left_a_button": left_a_button, "left_b_button": left_b_button,
            "left_thumbstick": left_thumbstick, "left_thumbstick_value": left_thumbstick_value,
            "right_trigger": right_trigger, "right_trigger_value": right_trigger_value,
            "right_squeeze": right_squeeze, "right_squeeze_value": right_squeeze_value,
            "right_a_button": right_a_button, "right_b_button": right_b_button,
            "right_thumbstick": right_thumbstick, "right_thumbstick_value": right_thumbstick_value,
        }

    def _update_dashboard_status_panel(self, session) -> None:
        if not self.dashboard["status_panel"]["enabled"]:
            return
        data = self._dashboard_status_data()
        try:
            session.update @ self._dashboard_status_panel(**data)
        except AssertionError:
            pass

    def _session_is_active(self, session) -> bool:
        return session.CURRENT_WS_ID in session.vuer.ws

    async def _keep_session_alive(self, session):
        while self._session_is_active(session):
            await asyncio.sleep(0.5)

    def close(self):
        self.process.terminate()
        self.process.join(timeout=0.5)
        if self.display_mode in ("immersive", "ego") and not self.webrtc:
            self.stop_writer_event.set()
            self.new_frame_event.set()
            self.writer_thread.join(timeout=0.5)
            try:
                self.img2display_shm.close()
                self.img2display_shm.unlink()
            except:
                pass

    async def on_cam_move(self, event, session, fps=60):
        try:
            with self.head_pose_shared.get_lock():
                self.head_pose_shared[:] = event.value["camera"]["matrix"]
        except:
            pass

    def _wrap_downlink(self) -> None:
        """Capture the connecting client's User-Agent, keyed by the ws object identity.

        vuer 的 ``downlink`` 同时有 request(含 User-Agent)和 ws;包装它在连接时把
        ``id(ws) -> UA`` 记下来,断开时清理。之后从运动事件里能通过会话的 ws 反查
        到"真正在摇操的那台设备"的 UA。
        """
        original = self.vuer.downlink

        async def wrapped(request, ws):
            ua = request.headers.get("User-Agent", "")
            self._last_ua = ua
            self._ws_ua[id(ws)] = ua
            try:
                return await original(request, ws)
            finally:
                self._ws_ua.pop(id(ws), None)

        self.vuer.downlink = wrapped

    def _record_teleop_device(self, session) -> None:
        """记住正在发运动数据(摇操)的会话所对应的设备 UA。"""
        ws = session.vuer.ws.get(session.CURRENT_WS_ID)
        if ws is not None:
            self._teleop_ua = self._ws_ua.get(id(ws), self._last_ua)
            self._teleop_at = time.time()

    def _connection_info_json(self) -> str:
        """`GET /connection` 返回当前遥操连接与设备信息(供 autobot 摇操页轮询)。

        字段语义(控制台据此显示"摇操中 / 设备已接入 / 仅页面连接 / 无设备连接"):

          connected          该 8012 服务上是否有任何页面(查看者也算)连着 —— 服务级,
                             **不能**用来判断"有没有头显在摇操";
          teleoperating      最近 ``_TELEOP_ACTIVE_WINDOW_S`` 秒内还在收手柄/手势运动
                             数据 = 真的有人在摇操。注意不能用 motion_data_ready
                             的锁存值,否则断连后永远显示"摇操中";
          controllerCaptured 本次运行中是否曾收到过运动数据(有设备接入过);
          deviceType/userAgent 正在摇操(或最后接入)设备的 UA 识别结果;
          connectedAt        最近一次收到运动数据的 epoch 秒;
          motionIdleMs       距最近一次运动数据的毫秒数(前端可直接用,避免时钟相减);
          auxConnected       电量辅助通道是否有订阅者。

        该接口被控制台每 5s 轮询一次,任何异常都会让控制台退化成"无设备连接",
        因此这里整体兜底:即使内部状态读取失败也要返回合法 JSON。
        """
        try:
            ua = self._teleop_ua or self._last_ua
            motion_idle_ms = (
                int((time.monotonic() - self._motion_at) * 1000) if self._motion_at else None
            )
            teleoperating = bool(motion_idle_ms is not None and motion_idle_ms <= self._TELEOP_ACTIVE_WINDOW_S * 1000)
            return json.dumps({
                "connected": bool(self.vuer.ws),
                "deviceType": _device_type_from_ua(ua),
                "userAgent": ua,
                "teleoperating": teleoperating,
                "controllerCaptured": bool(self._teleop_ua),
                "connectedAt": int(self._teleop_at) if self._teleop_at else None,
                "motionIdleMs": motion_idle_ms,
                "auxConnected": bool(self._aux_ws_clients),
            })
        except Exception as exc:  # noqa: BLE001
            logging.warning("[TeleVuer] /connection 组装失败,返回降级结果: %s", exc)
            return json.dumps({
                "connected": False,
                "deviceType": "未知设备",
                "userAgent": "",
                "teleoperating": False,
                "controllerCaptured": False,
                "connectedAt": None,
                "motionIdleMs": None,
                "auxConnected": False,
                "error": "connection_state_unavailable",
            })

    async def on_controller_move(self, event, session, fps=60):
        """https://docs.vuer.ai/en/latest/examples/20_motion_controllers.html"""
        try:
            # ControllerData
            with self.left_arm_pose_shared.get_lock():
                self.left_arm_pose_shared[:] = event.value["left"]
            with self.right_arm_pose_shared.get_lock():
                self.right_arm_pose_shared[:] = event.value["right"]
            # ControllerState
            left_controller = event.value["leftState"]
            right_controller = event.value["rightState"]

            def extract_controllers(controllerState, prefix):
                # trigger
                with getattr(self, f"{prefix}_ctrl_trigger_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_trigger_shared").value = bool(controllerState.get("trigger", False))
                with getattr(self, f"{prefix}_ctrl_triggerValue_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_triggerValue_shared").value = float(controllerState.get("triggerValue", 0.0))
                # squeeze
                with getattr(self, f"{prefix}_ctrl_squeeze_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_squeeze_shared").value = bool(controllerState.get("squeeze", False))
                with getattr(self, f"{prefix}_ctrl_squeezeValue_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_squeezeValue_shared").value = float(controllerState.get("squeezeValue", 0.0))
                # thumbstick
                with getattr(self, f"{prefix}_ctrl_thumbstick_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_thumbstick_shared").value = bool(controllerState.get("thumbstick", False))
                with getattr(self, f"{prefix}_ctrl_thumbstickValue_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_thumbstickValue_shared")[:] = controllerState.get("thumbstickValue", [0.0, 0.0])
                # buttons
                with getattr(self, f"{prefix}_ctrl_aButton_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_aButton_shared").value = bool(controllerState.get("aButton", False))
                with getattr(self, f"{prefix}_ctrl_bButton_shared").get_lock():
                    getattr(self, f"{prefix}_ctrl_bButton_shared").value = bool(controllerState.get("bButton", False))

            extract_controllers(left_controller, "left")
            extract_controllers(right_controller, "right")
            with self.motion_data_ready_shared.get_lock():
                self.motion_data_ready_shared.value = True
            # 时间窗:供 /connection 判断"当前是否真的在摇操"(锁存值会一直为真)。
            self._motion_at = time.monotonic()
            self._record_teleop_device(session)
        except:
            pass

    async def on_hand_move(self, event, session, fps=60):
        """https://docs.vuer.ai/en/latest/examples/19_hand_tracking.html"""
        try:
            # HandsData
            left_hand_data = event.value["left"]
            right_hand_data = event.value["right"]
            left_hand = event.value["leftState"]
            right_hand = event.value["rightState"]
            # HandState
            def extract_hand_poses(hand_data, arm_pose_shared, hand_position_shared, hand_orientation_shared):
                with arm_pose_shared.get_lock():
                    arm_pose_shared[:] = hand_data[0:16]

                with hand_position_shared.get_lock():
                    for i in range(25):
                        base = i * 16
                        hand_position_shared[i * 3: i * 3 + 3] = [hand_data[base + 12], hand_data[base + 13], hand_data[base + 14]]

                with hand_orientation_shared.get_lock():
                    for i in range(25):
                        base = i * 16
                        hand_orientation_shared[i * 9: i * 9 + 9] = [
                            hand_data[base + 0], hand_data[base + 1], hand_data[base + 2],
                            hand_data[base + 4], hand_data[base + 5], hand_data[base + 6],
                            hand_data[base + 8], hand_data[base + 9], hand_data[base + 10],
                        ]

            def extract_hands(handState, prefix):
                # pinch
                with getattr(self, f"{prefix}_hand_pinch_shared").get_lock():
                    getattr(self, f"{prefix}_hand_pinch_shared").value = bool(handState.get("pinch", False))
                with getattr(self, f"{prefix}_hand_pinchValue_shared").get_lock():
                    getattr(self, f"{prefix}_hand_pinchValue_shared").value = float(handState.get("pinchValue", 0.0))
                # squeeze
                with getattr(self, f"{prefix}_hand_squeeze_shared").get_lock():
                    getattr(self, f"{prefix}_hand_squeeze_shared").value = bool(handState.get("squeeze", False))
                with getattr(self, f"{prefix}_hand_squeezeValue_shared").get_lock():
                    getattr(self, f"{prefix}_hand_squeezeValue_shared").value = float(handState.get("squeezeValue", 0.0))

            extract_hand_poses(left_hand_data, self.left_arm_pose_shared, self.left_hand_position_shared, self.left_hand_orientation_shared)
            extract_hand_poses(right_hand_data, self.right_arm_pose_shared, self.right_hand_position_shared, self.right_hand_orientation_shared)
            extract_hands(left_hand, "left")
            extract_hands(right_hand, "right")
            with self.motion_data_ready_shared.get_lock():
                self.motion_data_ready_shared.value = True
            # 时间窗:供 /connection 判断"当前是否真的在摇操"(锁存值会一直为真)。
            self._motion_at = time.monotonic()
            self._record_teleop_device(session)

        except:
            pass
    
    ## immersive MODE
    async def main_image_binocular_zmq(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=True,
                    hideRight=True
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True,
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )
        while True:
            session.upsert(
                [
                    ImageBackground(
                        self.img2display[:, :self.img_width],
                        aspect=self.aspect_ratio,
                        height=1,
                        distanceToCamera=1,
                        # The underlying rendering engine supported a layer binary bitmask for both objects and the camera. 
                        # Below we set the two image planes, left and right, to layers=1 and layers=2. 
                        # Note that these two masks are associated with left eye’s camera and the right eye’s camera.
                        layers=1,
                        format="jpeg",
                        quality=80,
                        key="background-left",
                        interpolate=True,
                    ),
                    ImageBackground(
                        self.img2display[:, self.img_width:],
                        aspect=self.aspect_ratio,
                        height=1,
                        distanceToCamera=1,
                        layers=2,
                        format="jpeg",
                        quality=80,
                        key="background-right",
                        interpolate=True,
                    ),
                ],
                to="bgChildren",
            )
            # 'jpeg' encoding should give you about 30fps with a 16ms wait in-between.
            await asyncio.sleep(1.0 / self.display_fps)

    async def main_image_monocular_zmq(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=True,
                    hideRight=True
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        while True:
            session.upsert(
                [
                    ImageBackground(
                        self.img2display,
                        aspect=self.aspect_ratio,
                        height=1,
                        distanceToCamera=1,
                        format="jpeg",
                        quality=80,
                        key="background-mono",
                        interpolate=True,
                    ),
                ],
                to="bgChildren",
            )
            await asyncio.sleep(1.0 / self.display_fps)

    async def main_image_binocular_webrtc(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=True,
                    hideRight=True
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        if not self._session_is_active(session):
            return
        try:
            session.upsert(
                [
                     WebRTCStereoVideoPlane(
                         src=self.webrtc_url,
                         iceServer=None,
                         iceServers=[],
                         key="video-quad",
                          # The stream is side-by-side: self.aspect_ratio is
                          # already calculated from one eye's width. Applying
                          # the old 1.3 multiplier stretched the XR view.
                          aspect=self.aspect_ratio,
                          height=self.webrtc_immersive_height,
                          distanceToCamera=self.webrtc_immersive_distance,
                          layout="stereo-left-right",
                          # The PICO WebXR eye order is reversed relative to
                          # the SBS stream produced by the camera.
                          invertStereo=False,
                     ),
                 ],
                 to="bgChildren",
             )
        except AssertionError:
            return
        await self._keep_session_alive(session)

    async def main_image_binocular_webrtc_ego_split(self, session):
        if self.use_hand_tracking:
            session.upsert(
                [
                    Hands(
                        stream=True,
                        key="hands",
                        hideLeft=True,
                        hideRight=True
                    ),
                ],
                to="bgChildren",
            )
        else:
            session.upsert(
                [
                    MotionControllers(
                        stream=True,
                        key="motionControllers",
                        left=True,
                        right=True,
                    ),
                ],
                to="bgChildren",
            )

        if not self._session_is_active(session):
            return
        try:
            session.upsert(
                [
                    WebRTCVideoPlane(
                        src=self.webrtc_url_left,
                        iceServer=None,
                        iceServers=[],
                        key="video-quad-left",
                        aspect=self.aspect_ratio,
                        height=self.webrtc_ego_height,
                        distanceToCamera=self.webrtc_ego_distance,
                        layers=1,
                    ),
                    WebRTCVideoPlane(
                        src=self.webrtc_url_right,
                        iceServer=None,
                        iceServers=[],
                        key="video-quad-right",
                        aspect=self.aspect_ratio,
                        height=self.webrtc_ego_height,
                        distanceToCamera=self.webrtc_ego_distance,
                        layers=2,
                    ),
                ],
                to="bgChildren",
            )
        except AssertionError:
            return
        await self._keep_session_alive(session)

    async def main_image_binocular_webrtc_split(self, session):
        if self.use_hand_tracking:
            session.upsert(
                [
                    Hands(
                        stream=True,
                        key="hands",
                        hideLeft=True,
                        hideRight=True
                    ),
                ],
                to="bgChildren",
            )
        else:
            session.upsert(
                [
                    MotionControllers(
                        stream=True,
                        key="motionControllers",
                        left=True,
                        right=True,
                    ),
                ],
                to="bgChildren",
            )

        if not self._session_is_active(session):
            return
        try:
            session.upsert(
                [
                    WebRTCVideoPlane(
                        src=self.webrtc_url_left,
                        iceServer=None,
                        iceServers=[],
                        key="video-quad-left",
                        aspect=self.aspect_ratio,
                        height=self.webrtc_immersive_height,
                        distanceToCamera=self.webrtc_immersive_distance,
                        layers=1,
                    ),
                    WebRTCVideoPlane(
                        src=self.webrtc_url_right,
                        iceServer=None,
                        iceServers=[],
                        key="video-quad-right",
                        aspect=self.aspect_ratio,
                        height=self.webrtc_immersive_height,
                        distanceToCamera=self.webrtc_immersive_distance,
                        layers=2,
                    ),
                ],
                to="bgChildren",
            )
        except AssertionError:
            return
        await self._keep_session_alive(session)



    async def main_image_monocular_webrtc(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=True,
                    hideRight=True
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        if not self._session_is_active(session):
            return
        try:
            session.upsert(
                [
                    WebRTCVideoPlane(
                        src=self.webrtc_url,
                        iceServer=None,
                        iceServers=[],
                        key="video-quad",
                         # Keep the XR plane at one-eye aspect ratio; the
                         # stereo layout handles the side-by-side stream.
                         aspect=self.aspect_ratio,
                        height=self.webrtc_immersive_height,
                        distanceToCamera=self.webrtc_immersive_distance,
                    ),
                ],
                to="bgChildren",
            )
        except AssertionError:
            return
        # 桌面浏览器里没有连续位姿输入时,画面保持静止。为了不拖拽也能让画面"活",
        # 给视频平面加一个缓慢的呼吸摆动(auto sway):仅在非 PICO(无真实遥操设备)
        # 时生效,幅度可经 XR_TELEOP_AUTO_SWAY 环境变量关闭(默认开启)。
        if self._auto_sway_enabled():
            self._add_auto_sway_task(session)
        await self._keep_session_alive(session)

    # ------------------------------------------------------------------
    # 桌面浏览器自动摆动(auto sway):让视频平面缓慢旋转,产生"画面在动"效果。
    # 只在没有 PICO 设备持续发位姿(未处于遥操中)时运行,避免干扰真实遥操。
    # ------------------------------------------------------------------
    def _auto_sway_enabled(self) -> bool:
        return os.getenv("XR_TELEOP_AUTO_SWAY", "1").strip() in ("1", "true", "yes", "on")

    def _add_auto_sway_task(self, session) -> None:
        """为当前会话启动自动摇摆协程(同一 session 只启一个)。"""
        _key = (id(session), getattr(session, "CURRENT_WS_ID", 0))
        if getattr(self, "_auto_sway_tasks", None) is None:
            self._auto_sway_tasks = set()
        if _key in self._auto_sway_tasks:
            return
        self._auto_sway_tasks.add(_key)

        async def sway_loop():
            try:
                t0 = time.time()
                period = float(os.getenv("XR_TELEOP_AUTO_SWAY_PERIOD", "9.0"))
                amp = float(os.getenv("XR_TELEOP_AUTO_SWAY_AMPLITUDE", "0.06"))
                while self._session_is_active(session):
                    # 用户开始遥操(有数据进来)时自动停止摆动,避免干扰。
                    if bool(self.motion_data_ready):
                        await asyncio.sleep(0.5)
                        continue
                    phase = (time.time() - t0) * (2 * math.pi / period)
                    # 小幅往复摆动:绕 Y(水平扫视)和 X(轻微俯仰)。
                    sway_y = amp * math.sin(phase)
                    sway_x = amp * 0.35 * math.sin(phase * 1.7)
                    try:
                        session.update @ WebRTCVideoPlane(
                            key="video-quad",
                            rotation=[sway_x, sway_y, 0.0],
                        )
                    except AssertionError:
                        break
                    await asyncio.sleep(0.15)
            finally:
                self._auto_sway_tasks.discard(_key)

        asyncio.ensure_future(sway_loop())

    async def main_static_dashboard_monocular_webrtc(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=True,
                    hideRight=True,
                    visible=False,
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True,
                    key="motionControllers",
                    left=True,
                    right=True,
                    visible=False,
                ),
                to="bgChildren",
            )

        if not self._session_is_active(session):
            return

        camera = self.dashboard["camera_screen"]
        model = self.dashboard["robot_model"]
        elements = []
        # Soft key light to give the URDF model some depth (no ground grid). RandomizedLight
        # is a pure drei primitive supported by the 0.0.60 client; drop it if the target
        # device struggles with the extra light pass.
        elements.append(
            RandomizedLight(
                key="dashboard-light",
                position=[3, 5, -6],
                intensity=1.1,
                ambient=0.45,
                radius=4,
                castShadow=True,
            )
        )
        if camera["enabled"]:
            video_plane = dict(
                src=self.webrtc_url,
                iceServer=None,
                iceServers=[],
                key="head-camera-screen",
                         aspect=self.aspect_ratio,
                height=camera["height"],
                distanceToCamera=camera["distance"],
                position=camera["position"],
            )
            if camera["rotation"] is not None:
                video_plane["rotation"] = camera["rotation"]
            elements.append(WebRTCVideoPlane(**video_plane))
        if model["enabled"]:
            elements.append(
                Urdf(
                    src=model["src"],
                    key="robot-model",
                    position=model["position"],
                    rotation=model["rotation"],
                    scale=model["scale"],
                    materialType=model["material_type"],
                )
            )
        if self.dashboard["status_panel"]["enabled"]:
            elements.append(self._dashboard_status_panel())

        try:
            session.upsert(elements, to="bgChildren")
        except AssertionError:
            return

        while self._session_is_active(session):
            try:
                self._update_dashboard_status_panel(session)
            except AssertionError:
                break
            await asyncio.sleep(0.5)

    ## ego MODE
    async def main_image_binocular_zmq_ego(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=True,
                    hideRight=True
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True,
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )
        while True:
            session.upsert(
                [
                    ImageBackground(
                        self.img2display[:, :self.img_width],
                        aspect=self.aspect_ratio,
                        height=0.75,
                        distanceToCamera=2,
                        # The underlying rendering engine supported a layer binary bitmask for both objects and the camera. 
                        # Below we set the two image planes, left and right, to layers=1 and layers=2. 
                        # Note that these two masks are associated with left eye’s camera and the right eye’s camera.
                        layers=1,
                        format="jpeg",
                        quality=80,
                        key="background-left",
                        interpolate=True,
                    ),
                    ImageBackground(
                        self.img2display[:, self.img_width:],
                        aspect=self.aspect_ratio,
                        height=0.75,
                        distanceToCamera=2,
                        layers=2,
                        format="jpeg",
                        quality=80,
                        key="background-right",
                        interpolate=True,
                    ),
                ],
                to="bgChildren",
            )
            # 'jpeg' encoding should give you about 30fps with a 16ms wait in-between.
            await asyncio.sleep(1.0 / self.display_fps)

    async def main_image_monocular_zmq_ego(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=True,
                    hideRight=True
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        while True:
            session.upsert(
                [
                    ImageBackground(
                        self.img2display,
                        aspect=self.aspect_ratio,
                        height=0.75,
                        distanceToCamera=2,
                        format="jpeg",
                        quality=80,
                        key="background-mono",
                        interpolate=True,
                    ),
                ],
                to="bgChildren",
            )
            await asyncio.sleep(1.0 / self.display_fps)

    async def main_image_binocular_webrtc_ego(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=True,
                    hideRight=True
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        if not self._session_is_active(session):
            return
        try:
            session.upsert(
                [
                    WebRTCStereoVideoPlane(
                        src=self.webrtc_url,
                        iceServer=None,
                        iceServers=[],
                        key="video-quad",
                         aspect=self.aspect_ratio,
                         height=self.webrtc_ego_height,
                         distanceToCamera=self.webrtc_ego_distance,
                         layout="stereo-left-right",
                         invertStereo=True,
                    ),
                ],
                to="bgChildren",
            )
        except AssertionError:
            return
        await self._keep_session_alive(session)

    async def main_image_monocular_webrtc_ego(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=True,
                    hideRight=True
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        if not self._session_is_active(session):
            return
        try:
            session.upsert(
                [
                    WebRTCVideoPlane(
                        src=self.webrtc_url,
                        iceServer=None,
                        iceServers=[],
                        key="video-quad",
                        aspect=self.aspect_ratio * 1.3,
                        height=self.webrtc_ego_height,
                        distanceToCamera=self.webrtc_ego_distance,
                    ),
                ],
                to="bgChildren",
            )
        except AssertionError:
            return
        await self._keep_session_alive(session)

    ## pass-through MODE
    async def main_pass_through(self, session):
        if self.use_hand_tracking:
            session.upsert(
                Hands(
                    stream=True,
                    key="hands",
                    hideLeft=True,
                    hideRight=True
                ),
                to="bgChildren",
            )
        else:
            session.upsert(
                MotionControllers(
                    stream=True, 
                    key="motionControllers",
                    left=True,
                    right=True,
                ),
                to="bgChildren",
            )

        while True:
            await asyncio.sleep(1.0 / self.display_fps)

    # ==================== common data ====================
    @property
    def head_pose(self):
        """np.ndarray, shape (4, 4), head SE(3) pose matrix from Vuer (basis OpenXR Convention)."""
        with self.head_pose_shared.get_lock():
            return np.array(self.head_pose_shared[:]).reshape(4, 4, order="F")

    @property
    def left_arm_pose(self):
        """np.ndarray, shape (4, 4), left arm SE(3) pose matrix from Vuer (basis OpenXR Convention)."""
        with self.left_arm_pose_shared.get_lock():
            return np.array(self.left_arm_pose_shared[:]).reshape(4, 4, order="F")

    @property
    def right_arm_pose(self):
        """np.ndarray, shape (4, 4), right arm SE(3) pose matrix from Vuer (basis OpenXR Convention)."""
        with self.right_arm_pose_shared.get_lock():
            return np.array(self.right_arm_pose_shared[:]).reshape(4, 4, order="F")

    # ==================== Hand Tracking Data ====================
    @property
    def left_hand_positions(self):
        """np.ndarray, shape (25, 3), left hand 25 landmarks' 3D positions."""
        with self.left_hand_position_shared.get_lock():
            return np.array(self.left_hand_position_shared[:]).reshape(25, 3)

    @property
    def right_hand_positions(self):
        """np.ndarray, shape (25, 3), right hand 25 landmarks' 3D positions."""
        with self.right_hand_position_shared.get_lock():
            return np.array(self.right_hand_position_shared[:]).reshape(25, 3)

    @property
    def left_hand_orientations(self):
        """np.ndarray, shape (25, 3, 3), left hand 25 landmarks' orientations (flattened 3x3 matrices, column-major)."""
        with self.left_hand_orientation_shared.get_lock():
            return np.array(self.left_hand_orientation_shared[:]).reshape(25, 9).reshape(25, 3, 3, order="F")

    @property
    def right_hand_orientations(self):
        """np.ndarray, shape (25, 3, 3), right hand 25 landmarks' orientations (flattened 3x3 matrices, column-major)."""
        with self.right_hand_orientation_shared.get_lock():
            return np.array(self.right_hand_orientation_shared[:]).reshape(25, 9).reshape(25, 3, 3, order="F")

    @property
    def left_hand_pinch(self):
        """bool, whether left hand is pinching."""
        with self.left_hand_pinch_shared.get_lock():
            return self.left_hand_pinch_shared.value

    @property
    def left_hand_pinchValue(self):
        """float, pinch strength of left hand."""
        with self.left_hand_pinchValue_shared.get_lock():
            return self.left_hand_pinchValue_shared.value

    @property
    def left_hand_squeeze(self):
        """bool, whether left hand is squeezing."""
        with self.left_hand_squeeze_shared.get_lock():
            return self.left_hand_squeeze_shared.value

    @property
    def left_hand_squeezeValue(self):
        """float, squeeze strength of left hand."""
        with self.left_hand_squeezeValue_shared.get_lock():
            return self.left_hand_squeezeValue_shared.value

    @property
    def right_hand_pinch(self):
        """bool, whether right hand is pinching."""
        with self.right_hand_pinch_shared.get_lock():
            return self.right_hand_pinch_shared.value

    @property
    def right_hand_pinchValue(self):
        """float, pinch strength of right hand."""
        with self.right_hand_pinchValue_shared.get_lock():
            return self.right_hand_pinchValue_shared.value

    @property
    def right_hand_squeeze(self):
        """bool, whether right hand is squeezing."""
        with self.right_hand_squeeze_shared.get_lock():
            return self.right_hand_squeeze_shared.value

    @property
    def right_hand_squeezeValue(self):
        """float, squeeze strength of right hand."""
        with self.right_hand_squeezeValue_shared.get_lock():
            return self.right_hand_squeezeValue_shared.value

    # ==================== Controller Data ====================
    @property
    def left_ctrl_trigger(self):
        """bool, left controller trigger pressed or not."""
        with self.left_ctrl_trigger_shared.get_lock():
            return self.left_ctrl_trigger_shared.value

    @property
    def left_ctrl_triggerValue(self):
        """float, left controller trigger analog value (0.0 ~ 1.0)."""
        with self.left_ctrl_triggerValue_shared.get_lock():
            return self.left_ctrl_triggerValue_shared.value

    @property
    def left_ctrl_squeeze(self):
        """bool, left controller squeeze pressed or not."""
        with self.left_ctrl_squeeze_shared.get_lock():
            return self.left_ctrl_squeeze_shared.value

    @property
    def left_ctrl_squeezeValue(self):
        """float, left controller squeeze analog value (0.0 ~ 1.0)."""
        with self.left_ctrl_squeezeValue_shared.get_lock():
            return self.left_ctrl_squeezeValue_shared.value

    @property
    def left_ctrl_thumbstick(self):
        """bool, whether left thumbstick is touched or clicked."""
        with self.left_ctrl_thumbstick_shared.get_lock():
            return self.left_ctrl_thumbstick_shared.value

    @property
    def left_ctrl_thumbstickValue(self):
        """np.ndarray, shape (2,), left thumbstick 2D axis values (x, y)."""
        with self.left_ctrl_thumbstickValue_shared.get_lock():
            return np.array(self.left_ctrl_thumbstickValue_shared[:])

    @property
    def left_ctrl_aButton(self):
        """bool, left controller 'A' button pressed."""
        with self.left_ctrl_aButton_shared.get_lock():
            return self.left_ctrl_aButton_shared.value

    @property
    def left_ctrl_bButton(self):
        """bool, left controller 'B' button pressed."""
        with self.left_ctrl_bButton_shared.get_lock():
            return self.left_ctrl_bButton_shared.value

    @property
    def right_ctrl_trigger(self):
        """bool, right controller trigger pressed or not."""
        with self.right_ctrl_trigger_shared.get_lock():
            return self.right_ctrl_trigger_shared.value

    @property
    def right_ctrl_triggerValue(self):
        """float, right controller trigger analog value (0.0 ~ 1.0)."""
        with self.right_ctrl_triggerValue_shared.get_lock():
            return self.right_ctrl_triggerValue_shared.value

    @property
    def right_ctrl_squeeze(self):
        """bool, right controller squeeze pressed or not."""
        with self.right_ctrl_squeeze_shared.get_lock():
            return self.right_ctrl_squeeze_shared.value

    @property
    def right_ctrl_squeezeValue(self):
        """float, right controller squeeze analog value (0.0 ~ 1.0)."""
        with self.right_ctrl_squeezeValue_shared.get_lock():
            return self.right_ctrl_squeezeValue_shared.value

    @property
    def right_ctrl_thumbstick(self):
        """bool, whether right thumbstick is touched or clicked."""
        with self.right_ctrl_thumbstick_shared.get_lock():
            return self.right_ctrl_thumbstick_shared.value

    @property
    def right_ctrl_thumbstickValue(self):
        """np.ndarray, shape (2,), right thumbstick 2D axis values (x, y)."""
        with self.right_ctrl_thumbstickValue_shared.get_lock():
            return np.array(self.right_ctrl_thumbstickValue_shared[:])

    @property
    def right_ctrl_aButton(self):
        """bool, right controller 'A' button pressed."""
        with self.right_ctrl_aButton_shared.get_lock():
            return self.right_ctrl_aButton_shared.value

    @property
    def right_ctrl_bButton(self):
        """bool, right controller 'B' button pressed."""
        with self.right_ctrl_bButton_shared.get_lock():
            return self.right_ctrl_bButton_shared.value

    @property
    def motion_data_ready(self):
        """bool, whether at least one hand or controller motion data event has been received."""
        with self.motion_data_ready_shared.get_lock():
            return self.motion_data_ready_shared.value

    # ======================================================================
    # autobot-guide-service 集成:动态 host-info / 电池 / 独立辅助 WS
    # ======================================================================
    def _register_json_route(self, path: str, handler) -> bool:
        """注册一个返回 JSON 的自定义 GET 路由,并回查路由表确认它真的生效。

        为什么不能"调用完 vuer.add_route 就算数":
          这些路径是给 autobot 控制台用的。一旦没进路由表(被 vuer 自身的静态 /
          catch-all 路由抢占、旧版本没有该方法、router 已冻结……),外部现象只有
          "控制台设备状态显示无设备连接 / 电量不出数",从日志完全看不出原因。
          这里注册完立即回查 aiohttp 路由表,失败打 ERROR,并在启动后做一次 HTTP
          自检(见 ``_schedule_route_selfcheck``)。
        """
        registered = False
        try:
            self.vuer.add_route(path, handler, content_type="application/json")
            registered = True
        except Exception as exc:  # noqa: BLE001
            logging.warning(
                "[TeleVuer] vuer.add_route(%s) 失败(%s),改为直接挂 aiohttp router", path, exc
            )
        if not registered:
            try:
                self.vuer.app.router.add_get(path, handler)
                registered = True
            except Exception as exc:  # noqa: BLE001
                logging.error(
                    "[TeleVuer] 自定义路由 %s 注册失败,控制台将取不到该接口: %s", path, exc
                )
                return False

        try:
            paths = {(route.get_info() or {}).get("path") for route in self.vuer.app.router.routes()}
            if path not in paths:
                logging.error(
                    "[TeleVuer] 自定义路由 %s 不在 aiohttp 路由表中(可能被其它路由抢占),"
                    "控制台会取不到该接口。当前路由: %s",
                    path, sorted(p for p in paths if p),
                )
                return False
        except Exception as exc:  # noqa: BLE001
            logging.debug("[TeleVuer] 路由表回查失败(忽略): %s", exc)
        return True

    def _schedule_route_selfcheck(self, scheme: str, port: int) -> None:
        """启动后(延迟)用本机 HTTP 请求打一遍自定义接口,把结果写进日志。

        作用:把"路由没生效"和"设备确实没上报"这两种完全不同的故障区分开 ——
        前者是 404 / 返回 HTML,后者才是业务上没有数据。
        环境变量:XR_TELEOP_ROUTE_SELFCHECK=0 关闭;XR_TELEOP_ROUTE_SELFCHECK_DELAY 调延迟。
        """
        if os.getenv("XR_TELEOP_ROUTE_SELFCHECK", "1").strip() in ("0", "false", "no", "off"):
            return

        def _probe() -> None:
            import ssl
            import urllib.error
            import urllib.request

            # vuer.run() 在本对象构造完成之后才启动 aiohttp,先等它起来。
            time.sleep(float(os.getenv("XR_TELEOP_ROUTE_SELFCHECK_DELAY", "10")))
            ctx = ssl._create_unverified_context() if scheme == "https" else None
            for path in ("/connection", "/battery", "/_aux_info"):
                url = f"{scheme}://127.0.0.1:{port}{path}"
                try:
                    with urllib.request.urlopen(url, timeout=5, context=ctx) as resp:
                        body = resp.read(300).decode("utf-8", "replace").replace("\n", " ")
                        ctype = resp.headers.get("Content-Type", "")
                        ok = "json" in ctype.lower()
                        logging.log(
                            logging.INFO if ok else logging.ERROR,
                            "[TeleVuer][self-check] GET %s -> %s %s body=%s%s",
                            path, resp.status, ctype, body,
                            "" if ok else "  ← 返回不是 JSON:该自定义路由很可能被 vuer 自身路由抢占/未生效",
                        )
                except urllib.error.HTTPError as exc:
                    logging.error(
                        "[TeleVuer][self-check] GET %s -> HTTP %s,该自定义路由不可用(控制台会取不到)",
                        path, exc.code,
                    )
                except Exception as exc:  # noqa: BLE001
                    logging.info(
                        "[TeleVuer][self-check] GET %s 探测失败(服务可能尚未就绪): %s", path, exc
                    )

        threading.Thread(target=_probe, name="televuer-route-selfcheck", daemon=True).start()

    def _register_aux_websocket(self) -> None:
        """注册 /ws/aux 独立 WebSocket,走浏览器原生 ws,不与 Vuer 协议耦合。

        用途:
          * PICO 头显的浏览器内 JS 通过此通道上报电量(避免 iframe 内 JS 桥接);
          * 服务端把最新电池状态广播给所有订阅者。

        实现细节:
          vuer 内部用 aiohttp.web.Application,直接挂接到 ``self.vuer.app.router``。
          注册后回查路由表:该路由若没生效,autobot 控制台的 `/proxy/vr/{id}/_aux_ws`
          桥接会握手失败并约每秒重连一次(现象:电量完全不刷新),必须能在日志里看到。
        """
        try:
            from aiohttp import web  # noqa: WPS433 (local import: aiohttp 是可选依赖)
        except ImportError:
            logging.warning(
                "[TeleVuer] aiohttp not available; /ws/aux disabled. "
                "Install with `pip install vuer[all]` to enable."
            )
            return

        try:
            self.vuer.app.router.add_get("/ws/aux", self._handle_aux_ws)
            self.vuer.app.router.add_get("/_aux_info", self._aux_info_probe)
        except Exception as exc:  # noqa: BLE001
            logging.error(
                "[TeleVuer] /ws/aux registration failed: %s —— 控制台电量通道将不可用", exc
            )
            return

        try:
            paths = {(route.get_info() or {}).get("path") for route in self.vuer.app.router.routes()}
            if "/ws/aux" not in paths:
                logging.error(
                    "[TeleVuer] /ws/aux 不在 aiohttp 路由表中(可能被其它路由抢占),"
                    "控制台电量通道不可用。当前路由: %s",
                    sorted(p for p in paths if p),
                )
            else:
                logging.info("[TeleVuer] /ws/aux registered on aiohttp router")
        except Exception as exc:  # noqa: BLE001
            logging.info("[TeleVuer] /ws/aux registered (路由表回查失败: %s)", exc)

    async def _aux_info_probe(self, request):
        """供前端探测辅助 WS 端点是否存在。"""
        from aiohttp import web

        return web.json_response(
            {
                "wsPath": "/ws/aux",
                "hostInfoPath": "/host-info",
                "batteryPath": "/battery",
                "protocols": ["battery-update"],
            }
        )

    def _host_info_json(self) -> str:
        """返回当前主机的对外地址,供 autobot 前端动态拼 iframe URL。"""
        # 每次请求都重新检测一次,适应 IP 变化(如 wlan0 DHCP 重连)。
        info = host_info.detect_ips()
        self._host_info_cache = info
        public_port = host_info.public_port()
        return json.dumps(
            {
                "robotId": host_info.robot_id() or None,
                "wanIface": info.get("wan_iface"),
                "wanIp": info.get("wan_ip"),
                "eth0Ip": info.get("eth0_ip"),
                "port": public_port,
                "viewerUrl": f"https://{info.get('wan_ip')}:{public_port}",
                "wsUrl": f"wss://{info.get('wan_ip')}:{public_port}",
                "detectedAt": int(time.time() * 1000),
            }
        )

    def _battery_payload(self) -> dict:
        """当前电量快照。

        额外给出服务端算好的新鲜度(``ageMs`` / ``stale``):前端据此判断数据是否过期,
        不必拿"机器人的 clock"和"运营电脑的 clock"直接相减 —— 两端时钟偏差会
        直接把新鲜数据误判成"数据过期"(或反过来永远不过期)。
        """
        payload = dict(self._battery_state)
        updated_at = int(payload.get("updatedAt") or 0)
        payload["connected"] = bool(self._aux_ws_clients)
        payload["ageMs"] = (int(time.time() * 1000) - updated_at) if updated_at > 0 else None
        payload["stale"] = bool(payload["ageMs"] is not None and payload["ageMs"] > 30_000)
        return payload

    def _battery_json(self) -> str:
        """返回最近一次浏览器上报的 PICO 头显电量。

        字段:
          level       int|None  0-100
          charging    bool|None
          deviceName  str|None 头显型号 / 序列号
          updatedAt   int       上次更新时间(epoch ms),0 表示从未上报
          ageMs       int|None  距上次上报的毫秒数(服务端计算,前端优先用它)
          source      str       none/browser/fallback
          reportedBy  str|None 上报方的 User-Agent(诊断"谁在报电量")
          connected   bool      是否有活跃 ws 客户端
          stale       bool      数据是否已超过 30s 未刷新
        """
        return json.dumps(self._battery_payload(), ensure_ascii=False)

    async def _handle_aux_ws(self, request):
        """处理 /ws/aux 升级请求;内部协议是 JSON 文本消息。"""
        from aiohttp import web

        ua = request.headers.get("User-Agent", "") or ""
        headset = _is_headset_ua(ua)
        ws = web.WebSocketResponse(heartbeat=20)
        await ws.prepare(request)
        self._aux_ws_clients.add(ws)
        self._aux_ws_meta[id(ws)] = {"ua": ua, "headset": headset, "device": _device_type_from_ua(ua)}
        if not headset:
            logging.info(
                "[TeleVuer] /ws/aux 客户端 %s 不是头显,只允许订阅、忽略其电量上报: %s",
                _device_type_from_ua(ua), ua[:120],
            )
        try:
            # 上线时立即推送当前电量快照
            snapshot = self._battery_payload()
            snapshot["connected"] = True
            await ws.send_str(json.dumps({"type": "battery-snapshot", "data": snapshot}, ensure_ascii=False))
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        payload = json.loads(msg.data)
                    except json.JSONDecodeError:
                        continue
                    self._handle_aux_message(payload, ws)
                elif msg.type == web.WSMsgType.ERROR:
                    break
        finally:
            self._aux_ws_clients.discard(ws)
            self._aux_ws_meta.pop(id(ws), None)
            await ws.close()

    def _handle_aux_message(self, payload: dict, ws=None) -> None:
        """处理来自浏览器侧的辅助消息(目前只关心电量)。

        电量只接受"头显类客户端"的上报:桌面浏览器(运营电脑,通常插电 = 100%)直接
        打开 8012 页面时会上报本机电量,与头显真实电量交替覆盖缓存,表现为控制台
        电量在两个数之间跳变 —— 这是 2026-09-19 修复的核心问题。
        """
        mtype = payload.get("type")
        if mtype != "battery-update":
            # 其它类型(例如前端用于保活的 ping)直接忽略。
            return
        meta = self._aux_ws_meta.get(id(ws)) if ws is not None else None
        if meta is not None and not meta.get("headset"):
            ua = meta.get("ua", "")
            if ua not in self._battery_rejected_ua:
                self._battery_rejected_ua.add(ua)
                logging.warning(
                    "[TeleVuer] 已忽略非头显客户端(%s)的电量上报:%s。"
                    "若确认该设备就是头显,可设 XR_TELEOP_BATTERY_ALLOW_ANY_UA=1 放开。",
                    meta.get("device") or _device_type_from_ua(ua), ua[:120],
                )
            return
        data = payload.get("data") or {}
        self._update_battery_state(
            {**data, "source": "browser", "reportedBy": (meta or {}).get("ua") or None}
        )

    def _update_battery_state(self, new_state: dict) -> None:
        """合并一次电量上报(无效字段不覆盖已有值,并广播给所有订阅者)。"""
        level = new_state.get("level")
        charging = new_state.get("charging")
        device_name = new_state.get("deviceName")

        if isinstance(level, bool) or not isinstance(level, (int, float)) or not math.isfinite(float(level)):
            level = None
        else:
            level = max(0, min(100, int(round(float(level)))))
        if not isinstance(charging, bool):
            charging = None
        if not isinstance(device_name, str) or not device_name.strip():
            device_name = None

        if level is None and charging is None and device_name is None:
            logging.debug("[TeleVuer] 丢弃一次没有有效字段的电量上报: %s", new_state)
            return

        with self._aux_lock:
            # 只在字段有效时更新:避免一次缺字段的上报把已知电量清空(前端会表现为
            # "电量未上报"与真实数值交替闪烁)。
            if level is not None:
                self._battery_state["level"] = level
            if charging is not None:
                self._battery_state["charging"] = charging
            if device_name is not None:
                self._battery_state["deviceName"] = device_name
            self._battery_state["updatedAt"] = int(time.time() * 1000)
            self._battery_state["source"] = (
                new_state.get("source") or self._battery_state.get("source") or "browser"
            )
            if new_state.get("reportedBy"):
                self._battery_state["reportedBy"] = new_state["reportedBy"]

        # 异步广播给其它订阅者
        snapshot = self._battery_payload()
        snapshot["connected"] = True
        msg = json.dumps({"type": "battery-snapshot", "data": snapshot}, ensure_ascii=False)
        dead: list = []
        for ws in list(self._aux_ws_clients):
            try:
                loop = asyncio.get_event_loop()
                loop.create_task(ws.send_str(msg))
            except Exception:  # noqa: BLE001
                dead.append(ws)
        for ws in dead:
            self._aux_ws_clients.discard(ws)
            self._aux_ws_meta.pop(id(ws), None)
