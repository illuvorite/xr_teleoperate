# Copyright 2026 dsh-ssh fix workstream (remote teleimager latency fix)
#
# Jetson NVENC (nvv4l2h264enc) H.264 encoder bridge for the aiortc WebRTC
# sender in teleimager's image_server.py.
#
# Replaces the libx264 SOFTWARE encoder with the NVIDIA HARDWARE encoder:
#   appsrc (system BGR) -> videoconvert -> nvvidconv (to NVMM) ->
#   nvv4l2h264enc -> h264parse -> appsink (Annex-B / byte-stream)
#
# The conda env "tv" has no gobject-introspection typelibs of its own, so the
# module points GI_TYPELIB_PATH at the SYSTEM girepository; the system
# GStreamer 1.16 core then exposes the NVIDIA nvv4l2h264enc / nvvidconv
# plugins. If anything fails, callers fall back to the software encoder.

import os
import threading
import time

os.environ.setdefault("GI_TYPELIB_PATH", "/usr/lib/aarch64-linux-gnu/girepository-1.0")

_GST = None
_GI_LOCK = threading.Lock()


def _gst():
    global _GST
    if _GST is not None:
        return _GST
    with _GI_LOCK:
        if _GST is not None:
            return _GST
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst

        Gst.init(None)
        _GST = Gst
    return _GST


class JetsonNvEncH264:
    """One NVENC pipeline (one per aiortc H264Encoder instance)."""

    def __init__(self, width, height, bitrate, framerate, iframeinterval=15):
        self.width = int(width)
        self.height = int(height)
        self.bitrate = int(bitrate)
        self.framerate = max(1, int(framerate))
        self.iframeinterval = int(iframeinterval)
        self._pipe = None
        self._src = None
        self._sink = None
        self._closed = False
        self._first_sample_wait_ms = 300

    def start(self):
        Gst = _gst()
        launch = (
            f"appsrc name=src format=time is-live=true "
            f"! video/x-raw,format=BGR,width={self.width},height={self.height},"
            f"framerate={self.framerate}/1 "
            f"! videoconvert "
            f"! nvvidconv "
            f"! nvv4l2h264enc bitrate={self.bitrate} "
            f"iframeinterval={self.iframeinterval} idrinterval={self.iframeinterval} insert-sps-pps=true profile=0 maxperf-enable=true "
            f"! h264parse "
            f"! appsink name=sink sync=false drop=true"
        )
        self._pipe = Gst.parse_launch(launch)
        self._src = self._pipe.get_by_name("src")
        self._sink = self._pipe.get_by_name("sink")
        if self._src is None or self._sink is None:
            raise RuntimeError("NVENC pipeline elements missing")
        if self._pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("NVENC pipeline failed to start")

    def encode_frame(self, bgr_bytes, force_keyframe):
        """Push one BGR24 frame; return the encoded Annex-B bytes for it."""
        Gst = _gst()
        if self._closed:
            raise RuntimeError("NVENC pipeline closed")
        if force_keyframe:
            self._maybe_force_idr(True)
        buf = Gst.Buffer.new_allocate(None, len(bgr_bytes), None)
        ok, info = buf.map(Gst.MapFlags.WRITE)
        if not ok:
            raise RuntimeError("buffer map failed")
        buf.fill(0, bgr_bytes)
        buf.unmap(info)
        flow = self._src.emit("push-buffer", buf)
        if flow != Gst.FlowReturn.OK:
            raise RuntimeError(f"appsrc push failed: {flow}")
        if force_keyframe:
            self._maybe_force_idr(False)

        chunks = []
        got = 0
        while True:
            timeout_ns = self._first_sample_wait_ms * 1000 * 1000 if got == 0 else 1000 * 1000
            sample = self._sink.emit("try-pull-sample", timeout_ns)
            if sample is None:
                break
            got += 1
            sample_buf = sample.get_buffer()
            ok_map, info_map = sample_buf.map(Gst.MapFlags.READ)
            if ok_map:
                chunks.append(bytes(info_map.data))
                sample_buf.unmap(info_map)
            if got >= 8:
                break
        if got == 0:
            raise RuntimeError("NVENC produced no output for the frame")
        return b"".join(chunks)

    def _maybe_force_idr(self, value):
        """nvv4l2h264enc exposes 'force-idr' on some revisions; best-effort."""
        if self._pipe is None:
            return
        enc = self._pipe.get_by_name("nvv4l2h264enc0")
        if enc is None:
            return
        try:
            enc.set_property("force-idr", value)
        except Exception:
            pass

    def close(self):
        if self._closed or self._pipe is None:
            return
        self._closed = True
        try:
            if self._src is not None:
                self._src.emit("end-of-stream")
        except Exception:
            pass
        try:
            self._pipe.set_state(_gst().State.NULL)
        except Exception:
            pass

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


__all__ = ["JetsonNvEncH264"]