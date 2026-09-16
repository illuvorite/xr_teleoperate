"""Merchant SCAM camera adapter for Camera2201/RER stereo streams.

The merchant SDK returns one SBS image together with exposure timestamps and
11 ICM-42688 samples. This adapter deliberately keeps the SDK callback tiny:
copy the callback-owned buffer and metadata, then let the existing image
server consume only the newest frame.
"""
from __future__ import annotations

import ctypes
import logging_mp
import math
import os
import threading
import time
from typing import Optional

import cv2
import numpy as np

from .image_client import TripleRingBuffer

logger = logging_mp.getLogger(__name__)

SCAM_FORMAT_RGB24 = 2
SCAM_FORMAT_NV12 = 4
IMU_COUNT = 11


class _Imu(ctypes.Structure):
    _fields_ = [
        ("uTime", ctypes.c_uint64),
        ("fAccData_X", ctypes.c_float),
        ("fAccData_Y", ctypes.c_float),
        ("fAccData_Z", ctypes.c_float),
        ("fGyroData_X", ctypes.c_float),
        ("fGyroData_Y", ctypes.c_float),
        ("fGyroData_Z", ctypes.c_float),
    ]


class _Mtt(ctypes.Structure):
    _fields_ = [
        ("uTime", ctypes.c_uint64), ("iX", ctypes.c_int32),
        ("iY", ctypes.c_int32), ("iZ", ctypes.c_int32),
        ("Temp", ctypes.c_float), ("iStatusBit", ctypes.c_int32),
    ]


class _CamData(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.POINTER(ctypes.c_uint8)),
        ("width", ctypes.c_int), ("height", ctypes.c_int),
        ("bufSize", ctypes.c_int), ("format", ctypes.c_int),
        ("startExpouseTime", ctypes.c_uint64),
        ("endExpouseTime", ctypes.c_uint64),
        ("imu_data", _Imu * IMU_COUNT), ("mtt_data", _Mtt * 5),
    ]


_CALLBACK = ctypes.CFUNCTYPE(None, ctypes.POINTER(_CamData), ctypes.c_void_p)


class _ScamLibrary:
    def __init__(self, path: str):
        self.lib = ctypes.CDLL(path)
        self.lib.SCAM_Initialize.argtypes = []
        self.lib.SCAM_Initialize.restype = ctypes.c_int
        self.lib.SCAM_Release.argtypes = []
        self.lib.SCAM_Release.restype = None
        self.lib.SCAM_GetVersion.argtypes = []
        self.lib.SCAM_GetVersion.restype = ctypes.c_char_p
        self.lib.SCAM_GetLastError.argtypes = []
        self.lib.SCAM_GetLastError.restype = ctypes.c_int
        self.lib.SCAM_GetErrorText.argtypes = [ctypes.c_int]
        self.lib.SCAM_GetErrorText.restype = ctypes.c_char_p
        self.lib.SCAM_EnumDevices.argtypes = [ctypes.POINTER(_DeviceInfo), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
        self.lib.SCAM_EnumDevices.restype = ctypes.c_int
        self.lib.SCAM_GetDeviceFormats.argtypes = [ctypes.c_uint32, ctypes.POINTER(_FormatInfo), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
        self.lib.SCAM_GetDeviceFormats.restype = ctypes.c_int
        self.lib.SCAM_SetDeviceFormat.argtypes = [ctypes.c_uint32, ctypes.c_int]
        self.lib.SCAM_SetDeviceFormat.restype = ctypes.c_int
        self.lib.SCAM_SetImageFormat.argtypes = [ctypes.c_uint32, ctypes.c_int]
        self.lib.SCAM_SetImageFormat.restype = None
        self.lib.SCAM_OpenDevice.argtypes = [ctypes.c_uint32, _CALLBACK, ctypes.c_void_p]
        self.lib.SCAM_OpenDevice.restype = ctypes.c_int
        self.lib.SCAM_CloseDevice.argtypes = [ctypes.c_uint32]
        self.lib.SCAM_CloseDevice.restype = ctypes.c_int


class _DeviceInfo(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char * 256), ("vidpid", ctypes.c_char * 16),
                ("number", ctypes.c_uint32), ("isExist", ctypes.c_bool),
                ("isOpened", ctypes.c_bool)]


class _FormatInfo(ctypes.Structure):
    _fields_ = [("fmt", ctypes.c_int), ("width", ctypes.c_int),
                ("height", ctypes.c_int), ("fps", ctypes.c_int)]


class ScamCamera:
    """Duck-typed camera compatible with ImageServer's BaseCamera contract."""
    def __init__(self, cam_topic, img_shape, fps, library_path, device_index=0,
                 output_format="rgb24", enable_zmq=True, zmq_port=55555,
                 enable_webrtc=False, webrtc_port=66666, webrtc_codec=None,
                 stereo_fusion=False, stereo_crop_left=0, stereo_crop_right=0,
                 stereo_blend_alpha=0.5, stereo_right_shift_x=0,
                 stereo_sbs=False, stereo_shift_x=0, stereo_split_webrtc=False,
                 webrtc_port_left=None, webrtc_port_right=None,
                 sdk_hides_metadata_strip=True):
        self._cam_topic, self._img_shape, self._fps = cam_topic, img_shape, fps
        self._enable_zmq, self._zmq_port = enable_zmq, zmq_port
        self._enable_webrtc, self._webrtc_port = enable_webrtc, webrtc_port
        self._webrtc_codec = webrtc_codec
        self._stereo_fusion = bool(stereo_fusion)
        self._stereo_crop_left = 0 if sdk_hides_metadata_strip else max(0, int(stereo_crop_left))
        self._stereo_crop_right = max(0, int(stereo_crop_right))
        self._stereo_blend_alpha = float(stereo_blend_alpha)
        self._stereo_right_shift_x = int(stereo_right_shift_x)
        self._stereo_shift_x = int(stereo_shift_x)
        self._stereo_sbs = bool(stereo_sbs)
        self._stereo_split_webrtc = bool(stereo_split_webrtc)
        self._webrtc_port_left = webrtc_port_left if webrtc_port_left is not None else webrtc_port
        self._webrtc_port_right = webrtc_port_right if webrtc_port_right is not None else webrtc_port + 1
        self._ready = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._sequence = 0
        self._stabilizer = None
        self._frame_count = 0
        self._last_bgr = None
        self._last_log_t = time.monotonic()
        self._last_log_count = 0
        self._sdk = _ScamLibrary(os.path.expanduser(library_path))
        self._device_index = int(device_index)
        self._callback = _CALLBACK(self._on_frame)
        self._sdk_format = SCAM_FORMAT_NV12 if str(output_format).lower() == "nv12" else SCAM_FORMAT_RGB24
        self._zmq_buffer = TripleRingBuffer() if enable_zmq else None
        if enable_webrtc and stereo_split_webrtc:
            self._webrtc_buffer_left = TripleRingBuffer()
            self._webrtc_buffer_right = TripleRingBuffer()
            self._webrtc_buffer = None
        else:
            self._webrtc_buffer = TripleRingBuffer() if enable_webrtc else None
            self._webrtc_buffer_left = self._webrtc_buffer_right = None
        self._start_sdk()
        if self._webrtc_buffer is not None or self._webrtc_buffer_left is not None:
            # 预热：构造时同步消费首帧并写入发布缓冲，避免 server 发布线程首轮轮询
            # 遇到空缓冲触发 'no frame -> stop'（OpenCV 相机构造时已同步读到首帧，行为对齐）
            try:
                self._update_frame()
            except Exception as e:
                logger.warning("[ScamCamera] prime frame failed: %r", e)

    def _error(self):
        code = self._sdk.lib.SCAM_GetLastError()
        text = self._sdk.lib.SCAM_GetErrorText(code)
        return text.decode("utf-8", "replace") if text else str(code)

    def _start_sdk(self):
        if not self._sdk.lib.SCAM_Initialize():
            raise RuntimeError("SCAM_Initialize failed: " + self._error())
        formats = (_FormatInfo * 100)()
        count = ctypes.c_int()
        if not self._sdk.lib.SCAM_GetDeviceFormats(self._device_index, formats, 100, ctypes.byref(count)):
            self.release(); raise RuntimeError("SCAM_GetDeviceFormats failed: " + self._error())
        selected = 0
        for i in range(count.value):
            if formats[i].fmt != 3 and formats[i].width > 0:
                selected = i; break
        if not self._sdk.lib.SCAM_SetDeviceFormat(self._device_index, selected):
            self.release(); raise RuntimeError("SCAM_SetDeviceFormat failed: " + self._error())
        self._sdk.lib.SCAM_SetImageFormat(self._device_index, self._sdk_format)
        if not self._sdk.lib.SCAM_OpenDevice(self._device_index, self._callback, None):
            self.release(); raise RuntimeError("SCAM_OpenDevice failed: " + self._error())
        if not self._ready.wait(5.0):
            self.release(); raise RuntimeError("SCAM camera produced no frame within 5 seconds")
        logger.info("[ScamCamera] %s opened with SDK %s", self._cam_topic,
                    (self._sdk.lib.SCAM_GetVersion() or b"unknown").decode("utf-8", "replace"))

    def _on_frame(self, ptr, _user):
        # 回调必须极小：只深拷贝原始字节 + 元数据，彩色转换放到消费线程 _update_frame,
        # 避免阻塞 SDK 内部读帧循环（实测全幅 cvtColor 会卡死后续帧派发）。
        if not ptr or not ptr.contents.data: return
        meta = ptr.contents
        if meta.width <= 0 or meta.height <= 0 or meta.bufSize <= 0: return
        raw = ctypes.string_at(meta.data, meta.bufSize)
        # P0.5: 11 个 IMU 样本打包为 (11, 7) ndarray, 列: t_us, ax, ay, az, gx, gy, gz
        # 用 ctypes pointer cast 减少 ~80 次字段访问
        imu_arr = np.empty((IMU_COUNT, 7), dtype=np.float64)
        for i in range(IMU_COUNT):
            it = meta.imu_data[i]
            imu_arr[i, 0] = it.uTime
            imu_arr[i, 1] = it.fAccData_X
            imu_arr[i, 2] = it.fAccData_Y
            imu_arr[i, 3] = it.fAccData_Z
            imu_arr[i, 4] = it.fGyroData_X
            imu_arr[i, 5] = it.fGyroData_Y
            imu_arr[i, 6] = it.fGyroData_Z
        # P3.3: 防御 SDK 偶发把 IMU 序列 t_us 写成非严格单调 (实测 ~2/s drop)
        # 按 t_us 升序排序, 重复时间戳丢弃
        order = np.argsort(imu_arr[:, 0], kind="mergesort")
        imu_arr = imu_arr[order]
        # 去重 (相同样本重复)
        keep = np.ones(len(imu_arr), dtype=bool)
        keep[1:] = np.diff(imu_arr[:, 0]) > 0
        imu_arr = imu_arr[keep]
        with self._lock:
            self._sequence += 1
            self._latest = (raw, int(meta.width), int(meta.height), int(meta.format),
                            int(meta.startExpouseTime), int(meta.endExpouseTime),
                            imu_arr, self._sequence)
        self._ready.set()

    @staticmethod
    def _bgr_from_raw(raw, w, h, fmt):
        img = None
        if fmt == SCAM_FORMAT_RGB24:
            expected = w * h * 3
            if len(raw) >= expected:
                img = np.frombuffer(raw[:expected], np.uint8).reshape(h, w, 3)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif fmt == SCAM_FORMAT_NV12:
            expected = w * h * 3 // 2
            if len(raw) >= expected:
                nv12 = np.frombuffer(raw[:expected], np.uint8).reshape(h * 3 // 2, w)
                img = cv2.cvtColor(nv12, cv2.COLOR_YUV2BGR_NV12)
        return img

    def _transform(self, frame):
        if self._stereo_sbs:
            h, w = frame.shape[:2]
            left, right = frame[:, :w // 2], frame[:, w // 2:2 * (w // 2)]
            if self._stereo_crop_left:
                left = left[:, self._stereo_crop_left:]
                right = right[:, self._stereo_crop_left:]
            if self._stereo_crop_right:
                left = left[:, :-self._stereo_crop_right]
                right = right[:, :-self._stereo_crop_right]
            frame = cv2.hconcat([left, right])
        if self._stereo_shift_x:
            m = np.float32([[1, 0, self._stereo_shift_x], [0, 1, 0]])
            frame = cv2.warpAffine(frame, m, (frame.shape[1], frame.shape[0]), borderMode=cv2.BORDER_REPLICATE)
        if frame.shape[:2] != tuple(self._img_shape):
            frame = cv2.resize(frame, (self._img_shape[1], self._img_shape[0]), interpolation=cv2.INTER_AREA)
        return frame

    def attach_stabilizer(self, stabilizer):
        """IMU 电子防抖模块（见 stabilizer.py / imu_stabilizer.py）。"""
        self._stabilizer = stabilizer
        logger.info("[ScamCamera] %s EIS stabilizer attached (%s)",
                    self._cam_topic, type(stabilizer).__name__)

    def _update_frame(self):
        with self._lock: item = self._latest
        if item is None: return
        raw, w, h, fmt, t_start, t_end, imu, seq = item
        image = self._bgr_from_raw(raw, w, h, fmt)
        if image is None: return
        if self._stabilizer is not None and getattr(self._stabilizer, "enabled", False):
            try:
                image, _status = self._stabilizer.apply(image, t_start, t_end, imu)
            except Exception as e:  # 兜底：EIS 异常绝不阻塞采集线程
                logger.warning("[ScamCamera] EIS apply error: %r (bypass)", e)
        frame = self._transform(image)

        # 轻量速率统计（每 10s 一条，供 M3/M4 验收）
        self._frame_count += 1
        now = time.monotonic()
        if now - self._last_log_t >= 10.0:
            dt = now - self._last_log_t
            rate = (self._frame_count - self._last_log_count) / dt if dt > 0 else 0.0
            logger.info("[ScamCamera] %s capture rate: %.1f fps (10s window, seq=%d)",
                        self._cam_topic, rate, self._sequence)
            self._last_log_t, self._last_log_count = now, self._frame_count

        if self._stereo_split_webrtc:
            half = frame.shape[1] // 2
            left, right = frame[:, :half], frame[:, half:]
            if self._stereo_right_shift_x:
                m = np.float32([[1, 0, self._stereo_right_shift_x], [0, 1, 0]])
                right = cv2.warpAffine(right, m, (right.shape[1], right.shape[0]), borderMode=cv2.BORDER_REPLICATE)
            if self._enable_webrtc:
                self._webrtc_buffer_left.write(left); self._webrtc_buffer_right.write(right)
        elif self._enable_webrtc:
            self._webrtc_buffer.write(frame)
        if self._enable_zmq:
            ok, buf = cv2.imencode('.jpg', frame)
            if ok: self._zmq_buffer.write(buf.tobytes())

    def enable_webrtc(self): return self._enable_webrtc
    def enable_zmq(self): return self._enable_zmq
    def wait_until_ready(self, timeout=None):
        """与 BaseCamera 契约一致：等待首帧（_ready Event）。"""
        return self._ready.wait(timeout=timeout)
    def get_jpeg_bytes(self): return self._zmq_buffer.read() if self._zmq_buffer else None

    def get_bgr_frame(self):
        """返回最新 BGR 帧；首帧未就绪时短等待（防止 server 端 'no frame -> stop' 竞态），
        缓冲暂时为空时重复上一帧。"""
        if not self._webrtc_buffer:
            logger.warning("[ScamCamera] %s get_bgr_frame: webrtc_buffer is None (enable_webrtc=%s)",
                           self._cam_topic, self._enable_webrtc)
            return None
        f = self._webrtc_buffer.read()
        if f is not None:
            self._last_bgr = f
            return f
        if self._last_bgr is not None:
            return self._last_bgr
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.5:
            f = self._webrtc_buffer.read()
            if f is not None:
                self._last_bgr = f
                return f
            time.sleep(0.005)
        logger.warning("[ScamCamera] %s get_bgr_frame: no frame within 0.5s", self._cam_topic)
        return None  # 0.5s 仍无帧：交给 server 的看门狗路径
    def get_bgr_frame(self): return self._webrtc_buffer.read() if self._webrtc_buffer else None
    def get_bgr_frame_left(self): return self._webrtc_buffer_left.read() if self._webrtc_buffer_left else None
    def get_bgr_frame_right(self): return self._webrtc_buffer_right.read() if self._webrtc_buffer_right else None
    def get_zmq_port(self): return self._zmq_port
    def get_webrtc_port(self): return self._webrtc_port_left if self._stereo_split_webrtc else self._webrtc_port
    def get_webrtc_port_left(self): return self._webrtc_port_left
    def get_webrtc_port_right(self): return self._webrtc_port_right
    def get_webrtc_codec(self): return self._webrtc_codec
    def get_fps(self): return self._fps
    def release(self):
        self._stop.set()
        try: self._sdk.lib.SCAM_CloseDevice(self._device_index)
        except Exception: pass
        try: self._sdk.lib.SCAM_Release()
        except Exception: pass
        logger.info("[ScamCamera] Released %s", self._cam_topic)
