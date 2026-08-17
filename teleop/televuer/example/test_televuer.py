import argparse
import os
import sys
from pathlib import Path

import yaml
this_file = os.path.abspath(__file__)
project_root = os.path.abspath(os.path.join(os.path.dirname(this_file), '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

import time
import logging_mp
logger_mp = logging_mp.getLogger(__name__)
logger_mp.setLevel(logging_mp.INFO)

def load_display_config(path: str) -> dict:
    defaults = {
        "immersive": {"video_height": 1.8, "video_distance": 2.0},
        "ego": {"video_height": 0.7, "video_distance": 2.0},
        "ui": {"status_log_interval_sec": 2.0},
    }
    with open(path, "r", encoding="utf-8") as config_file:
        overrides = yaml.safe_load(config_file) or {}
    for section, values in overrides.items():
        if section in defaults and isinstance(values, dict):
            defaults[section].update(values)
    return defaults


def load_dashboard_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    if not isinstance(config, dict):
        raise ValueError("Static dashboard config must be a YAML mapping.")
    return config


def run_test_TeleVuer(args):
    display_config = load_display_config(args.display_config)
    use_hand_track = args.input_mode == "hand"
    use_webrtc = args.transport == "webrtc"
    dashboard_config = None
    assets_root = None
    if args.static_dashboard:
        if args.display_mode != "immersive":
            raise ValueError("Static dashboard supports only --display-mode immersive.")
        if not use_webrtc:
            raise ValueError("Static dashboard requires --transport webrtc.")
        dashboard_config = load_dashboard_config(args.dashboard_config)
        assets_root = str(Path(args.assets_root).expanduser().resolve())

    from televuer import TeleVuer
    from teleimager.image_client import ImageClient
    img_client = ImageClient(
        host=args.img_server_ip,
        request_bgr=not use_webrtc,
        subscribe_zmq=not use_webrtc,
    )
    camera_config = img_client.get_cam_config()
    head_config = camera_config["head_camera"]
    if use_webrtc and not head_config["enable_webrtc"]:
        raise RuntimeError("The image server has WebRTC disabled for head_camera.")
    if not use_webrtc and not head_config["enable_zmq"]:
        raise RuntimeError("The image server has ZMQ disabled for head_camera.")

    tv = TeleVuer(
        use_hand_tracking=use_hand_track,
        binocular=head_config["binocular"],
        img_shape=head_config["image_shape"],
        display_fps=head_config["fps"],
        display_mode=args.display_mode,
        zmq=not use_webrtc,
        webrtc=use_webrtc,
        webrtc_url=f"https://{args.img_server_ip}:{head_config['webrtc_port']}/offer",
        webrtc_immersive_height=display_config["immersive"]["video_height"],
        webrtc_ego_height=display_config["ego"]["video_height"],
        webrtc_immersive_distance=display_config["immersive"]["video_distance"],
        webrtc_ego_distance=display_config["ego"]["video_distance"],
        static_root=assets_root,
        dashboard=dashboard_config,
    )

    try:
        logger_mp.info(
            "TeleVuer ready: transport=%s, mode=%s, dashboard=%s, image server=%s, layout=%s",
            args.transport,
            args.display_mode,
            "static" if args.static_dashboard else "off",
            args.img_server_ip,
            display_config[args.display_mode] if args.display_mode in ("immersive", "ego") else "pass-through",
        )
        if not args.static_dashboard:
            input("Press Enter to start TeleVuer test...")
        last_status_log_time = 0.0
        while True:
            if not use_webrtc:
                tv.render_to_xr(img_client.get_head_frame().bgr)

            now = time.monotonic()
            if now - last_status_log_time >= display_config["ui"]["status_log_interval_sec"]:
                logger_mp.info(
                    "XR active | transport=%s | mode=%s | input=%s | dashboard=%s",
                    args.transport,
                    args.display_mode,
                    args.input_mode,
                    "static" if args.static_dashboard else "off",
                )
                last_status_log_time = now
            time.sleep(0.01 if not use_webrtc else 0.1)
    except KeyboardInterrupt:
        running = False
        logger_mp.warning("KeyboardInterrupt, exiting program...")
    finally:
        tv.close()
        logger_mp.warning("Finally, exiting program...")
        exit(0)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run the TeleVuer XR camera test.")
    parser.add_argument("--img-server-ip", default="192.168.2.203")
    parser.add_argument("--transport", choices=["webrtc", "zmq"], default="webrtc")
    parser.add_argument("--display-mode", choices=["immersive", "ego", "pass-through"], default="immersive")
    parser.add_argument("--input-mode", choices=["controller", "hand"], default="controller")
    parser.add_argument(
        "--display-config",
        default=str(Path(project_root) / "xr_display.yaml"),
        help="Path to the XR display layout YAML file.",
    )
    parser.add_argument(
        "--static-dashboard",
        action="store_true",
        help="Show the static G1 + Dex3, WebRTC, and status-panel cockpit.",
    )
    parser.add_argument(
        "--dashboard-config",
        default=str(Path(project_root) / "xr_dashboard.yaml"),
        help="Path to the static dashboard YAML file.",
    )
    parser.add_argument(
        "--assets-root",
        default=str(Path(project_root).parents[1] / "assets"),
        help="Directory exposed by Vuer at /static while the static dashboard is enabled.",
    )
    args, vuer_args = parser.parse_known_args()
    sys.argv = [sys.argv[0], *vuer_args]
    run_test_TeleVuer(args)
