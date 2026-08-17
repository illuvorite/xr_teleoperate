import vuer
from vuer import Vuer
from vuer.schemas import Box, Html, ImageBackground, Hands, MotionControllers, RandomizedLight, Urdf, WebRTCVideoPlane, WebRTCStereoVideoPlane, div, span
from multiprocessing import Value, Array, Process, shared_memory
import numpy as np
import asyncio
import json
import re
import shutil
import threading
import time
import cv2
import os
from pathlib import Path
from typing import Any, Literal
from xml.etree import ElementTree

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

_PATCHED_CLIENT_MARKER = ".patched-v2"


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


def _patched_client_root() -> Path:
    """Return a patched copy of vuer's client build with client defaults changed.

    Two patches are applied once into a cache directory, without touching the installed
    package and without requiring any URL query parameters:

      1. the client's default grid plane is disabled (the client otherwise shows it on any
         URL that lacks ``?grid=false``);
      2. a stylesheet hides the vuer built-in menu/dock (see _VUER_UI_HIDE_CSS).

    The server is pointed at this copy via ``Vuer.client_root`` before the Vuer instance is
    constructed, so both the root page and the ``/assets`` bundles are served from it.
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
        if count:
            js.write_text(patched, encoding="utf-8")

    # 2. Inject the menu-hiding stylesheet into the root page.
    index = patched_dir / "index.html"
    html = index.read_text(encoding="utf-8")
    if "vuer-ui-hide" not in html:
        html = html.replace(
            "</head>",
            f'<style id="vuer-ui-hide">{_VUER_UI_HIDE_CSS}</style></head>',
            1,
        )
        index.write_text(html, encoding="utf-8")

    marker.write_text("v1", encoding="utf-8")
    return patched_dir


class TeleVuer:
    def __init__(self, use_hand_tracking: bool, binocular: bool=True, img_shape: tuple=None, display_fps: float=30.0,
                        display_mode: Literal["immersive", "pass-through", "ego"]="immersive", zmq: bool=False, webrtc: bool=False, webrtc_url: str=None,
                        cert_file: str=None, key_file: str=None, http_mode: bool=False,
                        webrtc_immersive_height: float=2.2, webrtc_ego_height: float=1.0,
                        webrtc_immersive_distance: float=1.6, webrtc_ego_distance: float=1.8,
                        static_root: str | None=None, dashboard: dict[str, Any] | None=None):
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

        vuer_kwargs = dict(host='0.0.0.0', cert=cert_file, key=key_file,
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
        self.vuer.add_route("/connection", self._connection_info_json, content_type="application/json")
        self._wrap_downlink()
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
                fn = self.main_image_binocular_webrtc if self.binocular else self.main_image_monocular_webrtc
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
                fn = self.main_image_binocular_webrtc_ego if self.binocular else self.main_image_monocular_webrtc_ego
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
        """`GET /connection` 返回当前遥操连接与设备信息(供 autobot 摇操页轮询)。"""
        ua = self._teleop_ua or self._last_ua
        return json.dumps({
            "connected": bool(self.vuer.ws),
            "deviceType": _device_type_from_ua(ua),
            "userAgent": ua,
            "teleoperating": bool(self.motion_data_ready),
            "controllerCaptured": bool(self._teleop_ua),
            "connectedAt": int(self._teleop_at) if self._teleop_at else None,
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
                        aspect=self.aspect_ratio * 1.3,
                        height=self.webrtc_immersive_height,
                        distanceToCamera=self.webrtc_immersive_distance,
                        layout="stereo-left-right"
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
                        aspect=self.aspect_ratio * 1.3,
                        height=self.webrtc_immersive_height,
                        distanceToCamera=self.webrtc_immersive_distance,
                    ),
                ],
                to="bgChildren",
            )
        except AssertionError:
            return
        await self._keep_session_alive(session)

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
                aspect=self.aspect_ratio * 1.3,
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
                        aspect=self.aspect_ratio * 1.3,
                        height=self.webrtc_ego_height,
                        distanceToCamera=self.webrtc_ego_distance,
                        layout="stereo-left-right"
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
