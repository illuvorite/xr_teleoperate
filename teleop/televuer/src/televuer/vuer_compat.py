from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
import logging


logger = logging.getLogger(__name__)

_SUPPORTED_VUER_VERSION = "0.0.60"
_PATCH_MARKER = "tele-vuer-video-texture-refresh"
_PATTERNS = (
    ("s&&a&&!s.image.paused&&(s.needsUpdate=!0)", "s&&!s.image.paused&&(s.needsUpdate=!0)"),
    ("s&&C&&!s.image.paused&&(s.needsUpdate=!0)", "s&&!s.image.paused&&(s.needsUpdate=!0)"),
)


def apply_vuer_video_texture_patch() -> int:
    """Keep WebRTC VideoTexture updating outside immersive XR on Vuer 0.0.60.

    Vuer 0.0.60 only forces ``VideoTexture.needsUpdate`` while an immersive XR
    session is active. Some Chromium builds expose ``requestVideoFrameCallback``
    but invoke it only for the first MediaStream frame in the WebGL texture path,
    leaving the normal 8012 viewer frozen. The patch makes the existing render
    loop mark an actively playing video texture dirty on every frame.

    Returns the number of frontend chunks patched. Repeated calls are safe.
    """
    try:
        installed_version = version("vuer")
    except PackageNotFoundError:
        logger.warning("Vuer is not installed; skipping video texture compatibility patch.")
        return 0

    if installed_version != _SUPPORTED_VUER_VERSION:
        logger.warning(
            "Vuer %s is not the tested version %s; skipping video texture compatibility patch.",
            installed_version,
            _SUPPORTED_VUER_VERSION,
        )
        return 0

    import vuer

    client_build = Path(vuer.__file__).resolve().parent / "client_build"
    patched_count = 0
    already_patched = False

    for path in client_build.rglob("*.js"):
        source = path.read_text(encoding="utf-8", errors="ignore")
        if _PATCH_MARKER in source:
            already_patched = True
            continue

        updated = source
        for old, new in _PATTERNS:
            updated = updated.replace(old, new)

        if updated == source:
            continue

        updated = f"/* {_PATCH_MARKER} */\n{updated}"
        temporary_path = path.with_suffix(f"{path.suffix}.tmp")
        temporary_path.write_text(updated, encoding="utf-8")
        temporary_path.replace(path)
        patched_count += 1

    if patched_count:
        logger.info("Patched %d Vuer frontend chunks for continuous WebRTC playback.", patched_count)
    elif not already_patched:
        logger.warning("No compatible Vuer video texture code was found; frontend was not modified.")

    return patched_count
