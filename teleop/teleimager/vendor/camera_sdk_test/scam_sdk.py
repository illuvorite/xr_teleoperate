# -*- coding: utf-8 -*-
"""
SCAM SDK Python Wrapper
-----------------------
This module provides Python bindings for the SCAM SDK (libscam.so) using ctypes.
Designed for high performance and safety, allowing colleagues to easily integrate
camera capture and NV12 Y-plane extraction into subsequent algorithms.
"""

import os
import sys
import ctypes

# ==================== 常量与枚举定义 ====================
class CamFormat:
    FORMAT_UNKNOWN = 0
    FORMAT_MJPG = 1
    FORMAT_RGB24 = 2
    FORMAT_YUV422 = 3
    FORMAT_NV12 = 4

    @staticmethod
    def get_name(fmt):
        names = {
            CamFormat.FORMAT_UNKNOWN: "UNKNOWN",
            CamFormat.FORMAT_MJPG: "MJPG",
            CamFormat.FORMAT_RGB24: "RGB24",
            CamFormat.FORMAT_YUV422: "YUV422",
            CamFormat.FORMAT_NV12: "NV12"
        }
        return names.get(fmt, "UNKNOWN")

# ==================== C++ 结构体映射 ====================

class DeviceInfo(ctypes.Structure):
    _fields_ = [
        ("name", ctypes.c_char * 256),
        ("vidpid", ctypes.c_char * 16),
        ("number", ctypes.c_uint32),
        ("isExist", ctypes.c_bool),
        ("isOpened", ctypes.c_bool)
    ]

    def to_dict(self):
        return {
            "name": self.name.decode("utf-8", errors="replace"),
            "vidpid": self.vidpid.decode("utf-8", errors="replace"),
            "number": self.number,
            "isExist": self.isExist,
            "isOpened": self.isOpened
        }

class sICM42688_XYZ_float(ctypes.Structure):
    _fields_ = [
        ("uTime", ctypes.c_uint64),
        ("fAccData_X", ctypes.c_float),
        ("fAccData_Y", ctypes.c_float),
        ("fAccData_Z", ctypes.c_float),
        ("fGyroData_X", ctypes.c_float),
        ("fGyroData_Y", ctypes.c_float),
        ("fGyroData_Z", ctypes.c_float)
    ]

    def to_dict(self):
        return {
            "uTime": self.uTime,
            "acc": (self.fAccData_X, self.fAccData_Y, self.fAccData_Z),
            "gyro": (self.fGyroData_X, self.fGyroData_Y, self.fGyroData_Z)
        }

class sAK09940_XYZ_int(ctypes.Structure):
    _fields_ = [
        ("uTime", ctypes.c_uint64),
        ("iX", ctypes.c_int32),
        ("iY", ctypes.c_int32),
        ("iZ", ctypes.c_int32),
        ("Temp", ctypes.c_float),
        ("iStatusBit", ctypes.c_int32)
    ]

    def to_dict(self):
        return {
            "uTime": self.uTime,
            "mag": (self.iX, self.iY, self.iZ),
            "temp": self.Temp,
            "status_bit": self.iStatusBit
        }

class FormatInfo(ctypes.Structure):
    _fields_ = [
        ("fmt", ctypes.c_int),  # CamFormat
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("fps", ctypes.c_int)
    ]

    def to_dict(self):
        return {
            "fmt": self.fmt,
            "fmt_name": CamFormat.get_name(self.fmt),
            "width": self.width,
            "height": self.height,
            "fps": self.fps
        }

class CamData(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.POINTER(ctypes.c_uint8)),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("bufSize", ctypes.c_int),
        ("format", ctypes.c_int),
        ("startExpouseTime", ctypes.c_uint64),
        ("endExpouseTime", ctypes.c_uint64),
        ("imu_data", sICM42688_XYZ_float * 11),
        ("mtt_data", sAK09940_XYZ_int * 5)
    ]

# ==================== 高级 Python 帧数据类 ====================

class FrameData:
    """
    对底层的 CamData 结构体进行封装，提供安全易用的数据提取方法。
    """
    def __init__(self, cam_data):
        self.width = cam_data.width
        self.height = cam_data.height
        self.buf_size = cam_data.bufSize
        self.format = cam_data.format
        self.start_exposure_time = cam_data.startExpouseTime
        self.end_exposure_time = cam_data.endExpouseTime
        
        # 复制 IMU 和 MTT 数据，断开与底层指针生命周期的绑定
        self.imu_data = [imu.to_dict() for imu in cam_data.imu_data]
        self.mtt_data = [mtt.to_dict() for mtt in cam_data.mtt_data]
        
        # 记录底层图像数据指针的整数地址值，用于后续按需提取数据
        self._raw_data_ptr_val = ctypes.cast(cam_data.data, ctypes.c_void_p).value

    def get_y_plane_bytes(self):
        """
        获取 NV12 格式下的 Y 平面数据（灰度图/亮度通道）。
        Y 平面大小为 width * height，位于 NV12 缓冲区的头部。
        :return: bytes 类型的 Y 平面数据拷贝（生命周期安全）
        """
        if not self._raw_data_ptr_val:
            return b""
        y_size = self.width * self.height
        return ctypes.string_at(self._raw_data_ptr_val, y_size)

    def get_y_plane_ndarray(self):
        """
        获取 NV12 格式下的 Y 平面数据，转换为 numpy.ndarray。
        :return: shape 为 (height, width), dtype 为 uint8 的 numpy 数组拷贝。
                 如果系统未安装 numpy，则抛出 ImportError。
        """
        import numpy as np
        if not self._raw_data_ptr_val:
            return None
        ptr = ctypes.cast(self._raw_data_ptr_val, ctypes.POINTER(ctypes.c_uint8))
        # 建立零拷贝的 view
        view = np.ctypeslib.as_array(ptr, shape=(self.height, self.width))
        # 拷贝数据以保证在回调结束后继续使用时的安全性
        return view.copy()

    def get_stereo_y_planes_ndarray(self):
        """
        将双目的 Y 平面图像（拼接好的左右目）切分成独立的左右目 Y 平面。
        - 针对双目图像分辨率为 3840x1200：
          - 左目：1920x1200（列 0-1919）
          - 右目：1920x1200（列 1920-3839）
        :return: (left_y_ndarray, right_y_ndarray) 二元组 (numpy.ndarray 拷贝)
                 如果系统未安装 numpy，则抛出 ImportError。
        """
        import numpy as np
        if not self._raw_data_ptr_val:
            return None, None
        
        ptr = ctypes.cast(self._raw_data_ptr_val, ctypes.POINTER(ctypes.c_uint8))
        view = np.ctypeslib.as_array(ptr, shape=(self.height, self.width))
        
        if self.width == 3840 and self.height == 1200:
            left_y = view[:, :1920].copy()
            right_y = view[:, 1920:].copy()
        else:
            # 泛化情况：平分宽度
            half_w = self.width // 2
            left_y = view[:, :half_w].copy()
            right_y = view[:, half_w:].copy()
            
        return left_y, right_y

    def get_stereo_y_planes_bytes(self):
        """
        将双目的 Y 平面数据（拼接好的左右目）切分成独立的左右目 Y 平面字节数据。
        优先使用高效率的 NumPy 实现，若无 NumPy 则使用纯 Python 逐行提取的降级方案。
        :return: (left_y_bytes, right_y_bytes) 二元组 (bytes)
        """
        try:
            left_y, right_y = self.get_stereo_y_planes_ndarray()
            if left_y is None or right_y is None:
                return b"", b""
            return left_y.tobytes(), right_y.tobytes()
        except ImportError:
            if not self._raw_data_ptr_val:
                return b"", b""
            
            # 纯 Python 降级方案
            if self.width == 3840 and self.height == 1200:
                left_y_list = []
                right_y_list = []
                for r in range(1200):
                    row_start = self._raw_data_ptr_val + r * 3840
                    left_y_list.append(ctypes.string_at(row_start, 1920))
                    right_y_list.append(ctypes.string_at(row_start + 1920, 1920))
                return b"".join(left_y_list), b"".join(right_y_list)
            else:
                half_w = self.width // 2
                left_y_list = []
                right_y_list = []
                for r in range(self.height):
                    row_start = self._raw_data_ptr_val + r * self.width
                    left_y_list.append(ctypes.string_at(row_start, half_w))
                    right_y_list.append(ctypes.string_at(row_start + half_w, half_w))
                return b"".join(left_y_list), b"".join(right_y_list)

    def get_uv_plane_bytes(self):
        """
        获取 NV12 格式下的 UV 交错平面数据。
        UV 平面大小为 width * height / 2，紧跟在 Y 平面后面。
        :return: bytes 类型的 UV 平面数据拷贝
        """
        if not self._raw_data_ptr_val:
            return b""
        offset = self.width * self.height
        uv_size = (self.width * self.height) // 2
        return ctypes.string_at(self._raw_data_ptr_val + offset, uv_size)

    def get_nv12_bytes(self):
        """
        获取完整的 NV12 原始字节数据（Y 连续 + UV 交错）。
        总大小为 width * height * 1.5。
        :return: bytes 类型的完整 NV12 数据拷贝
        """
        if not self._raw_data_ptr_val:
            return b""
        nv12_size = int(self.width * self.height * 1.5)
        return ctypes.string_at(self._raw_data_ptr_val, nv12_size)


# C++ 回调函数类型声明
# typedef void (*CamCallback)(const CamData* image, void* userData);
CAM_CALLBACK_TYPE = ctypes.CFUNCTYPE(None, ctypes.POINTER(CamData), ctypes.c_void_p)

# ==================== SDK 包装类 ====================

class ScamSDK:
    """
    SCAM SDK 封装类。
    """
    def __init__(self, lib_path=None, turbojpeg_path=None):
        # 自动搜索库路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        if not turbojpeg_path:
            possible_turbojpeg_paths = [
                os.path.join(current_dir, "build", "libturbojpeg.so.0"),
                os.path.join(current_dir, "libturbojpeg.so.0"),
                "libturbojpeg.so.0",
                "turbojpeg"
            ]
            for p in possible_turbojpeg_paths:
                if os.path.exists(p) or "/" not in p:
                    turbojpeg_path = p
                    break

        if not lib_path:
            possible_lib_paths = [
                os.path.join(current_dir, "build", "libscam.so"),
                os.path.join(current_dir, "libscam.so"),
                "libscam.so",
                "scam"
            ]
            for p in possible_lib_paths:
                if os.path.exists(p) or "/" not in p:
                    lib_path = p
                    break

        # 加载依赖库
        try:
            self.libturbojpeg = ctypes.CDLL(turbojpeg_path, mode=ctypes.RTLD_GLOBAL)
        except Exception as e:
            print(f"[Warning] Failed to load libturbojpeg from {turbojpeg_path}: {e}")
            
        try:
            self.lib = ctypes.CDLL(lib_path)
        except Exception as e:
            raise RuntimeError(f"Failed to load libscam from {lib_path}: {e}")

        self._setup_function_prototypes()
        # 缓存回调引用，防止被 Python 的垃圾回收（GC）机制销毁
        self._c_callback_cache = {}

    def _setup_function_prototypes(self):
        """
        设置所有 C API 函数的参数和返回类型。
        """
        # SCAM_API int SCAM_Initialize();
        self.lib.SCAM_Initialize.argtypes = []
        self.lib.SCAM_Initialize.restype = ctypes.c_int

        # SCAM_API void SCAM_Release();
        self.lib.SCAM_Release.argtypes = []
        self.lib.SCAM_Release.restype = None

        # SCAM_API const char* SCAM_GetVersion();
        self.lib.SCAM_GetVersion.argtypes = []
        self.lib.SCAM_GetVersion.restype = ctypes.c_char_p

        # SCAM_API const int SCAM_GetLastError();
        self.lib.SCAM_GetLastError.argtypes = []
        self.lib.SCAM_GetLastError.restype = ctypes.c_int

        # SCAM_API const char* SCAM_GetErrorText(int errorCode);
        self.lib.SCAM_GetErrorText.argtypes = [ctypes.c_int]
        self.lib.SCAM_GetErrorText.restype = ctypes.c_char_p

        # SCAM_API int SCAM_EnumDevices(DeviceInfo* devices, int maxCount, int* count);
        self.lib.SCAM_EnumDevices.argtypes = [ctypes.POINTER(DeviceInfo), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
        self.lib.SCAM_EnumDevices.restype = ctypes.c_int

        # SCAM_API int SCAM_OpenDevice(uint32_t deviceIndex, CamCallback callback=NULL, void* userData=NULL);
        self.lib.SCAM_OpenDevice.argtypes = [ctypes.c_uint32, CAM_CALLBACK_TYPE, ctypes.c_void_p]
        self.lib.SCAM_OpenDevice.restype = ctypes.c_int

        # SCAM_API int SCAM_CloseDevice(uint32_t deviceIndex);
        self.lib.SCAM_CloseDevice.argtypes = [ctypes.c_uint32]
        self.lib.SCAM_CloseDevice.restype = ctypes.c_int

        # SCAM_API int SCAM_GetDeviceFormats(uint32_t deviceIndex, FormatInfo* formats, int maxCount, int* count);
        self.lib.SCAM_GetDeviceFormats.argtypes = [ctypes.c_uint32, ctypes.POINTER(FormatInfo), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
        self.lib.SCAM_GetDeviceFormats.restype = ctypes.c_int

        # SCAM_API int SCAM_SetDeviceFormat(uint32_t deviceIndex, int formatIndex);
        self.lib.SCAM_SetDeviceFormat.argtypes = [ctypes.c_uint32, ctypes.c_int]
        self.lib.SCAM_SetDeviceFormat.restype = ctypes.c_int

        # SCAM_API int SCAM_GetDeviceFormatIndex(uint32_t deviceIndex);
        self.lib.SCAM_GetDeviceFormatIndex.argtypes = [ctypes.c_uint32]
        self.lib.SCAM_GetDeviceFormatIndex.restype = ctypes.c_int

        # SCAM_API void SCAM_SetImageFormat(uint32_t devIndex,CamFormat format);
        self.lib.SCAM_SetImageFormat.argtypes = [ctypes.c_uint32, ctypes.c_int]
        self.lib.SCAM_SetImageFormat.restype = None

        # SCAM_API bool SCAM_SaveToJPG(const uint8_t* buf, CamFormat fmt,int w, int h, const char* filename, int quality = 85);
        self.lib.SCAM_SaveToJPG.argtypes = [ctypes.POINTER(ctypes.c_uint8), ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
        self.lib.SCAM_SaveToJPG.restype = ctypes.c_bool

        # SCAM_API float SCAM_GetFrameRate(uint32_t deviceIndex);
        self.lib.SCAM_GetFrameRate.argtypes = [ctypes.c_uint32]
        self.lib.SCAM_GetFrameRate.restype = ctypes.c_float

    def initialize(self):
        """初始化 SDK"""
        return bool(self.lib.SCAM_Initialize())

    def release(self):
        """释放 SDK 资源"""
        self.lib.SCAM_Release()

    def get_version(self):
        """获取 SDK 版本"""
        version = self.lib.SCAM_GetVersion()
        return version.decode('utf-8', errors='replace') if version else ""

    def get_last_error(self):
        """获取最后一次发生的错误码"""
        return self.lib.SCAM_GetLastError()

    def get_error_text(self, error_code):
        """根据错误码获取错误描述"""
        text = self.lib.SCAM_GetErrorText(error_code)
        return text.decode('utf-8', errors='replace') if text else ""

    def enum_devices(self, max_count=6):
        """
        枚举系统中连接的可用设备。
        :return: 包含设备信息的 list[dict]
        """
        devices_array = (DeviceInfo * max_count)()
        count = ctypes.c_int(0)
        ret = self.lib.SCAM_EnumDevices(devices_array, max_count, ctypes.byref(count))
        if not ret:
            error_code = self.get_last_error()
            raise RuntimeError(f"EnumDevices failed: {self.get_error_text(error_code)}")
        
        result = []
        for i in range(count.value):
            if devices_array[i].isExist:
                result.append(devices_array[i].to_dict())
        return result

    def get_device_formats(self, device_index, max_count=100):
        """
        获取指定设备所支持的分辨率和格式列表。
        :return: 格式信息的 list[dict]
        """
        formats_array = (FormatInfo * max_count)()
        count = ctypes.c_int(0)
        ret = self.lib.SCAM_GetDeviceFormats(device_index, formats_array, max_count, ctypes.byref(count))
        if not ret:
            error_code = self.get_last_error()
            raise RuntimeError(f"GetDeviceFormats failed: {self.get_error_text(error_code)}")
        
        result = []
        for i in range(count.value):
            result.append(formats_array[i].to_dict())
        return result

    def set_device_format(self, device_index, format_index):
        """设置相机抓取的分辨率和格式索引，如果选定格式为 YUV422 则抛出异常"""
        formats = self.get_device_formats(device_index)
        if 0 <= format_index < len(formats):
            fmt_info = formats[format_index]
            if fmt_info.get("fmt") == CamFormat.FORMAT_YUV422:
                raise ValueError("SDK 不支持 YUV422 分辨率格式，请选择其他格式（如 MJPG）。")
        ret = self.lib.SCAM_SetDeviceFormat(device_index, format_index)
        if not ret:
            error_code = self.get_last_error()
            raise RuntimeError(f"SetDeviceFormat failed: {self.get_error_text(error_code)}")
        return True

    def get_device_format_index(self, device_index):
        """获取相机当前的视频捕获格式索引"""
        return self.lib.SCAM_GetDeviceFormatIndex(device_index)

    def set_image_format(self, device_index, image_format):
        """
        设置目标相机的输出图像格式 (例如 CamFormat.FORMAT_NV12)。
        SDK 会在内部将图像转换为此格式后回调输出。仅支持 NV12 与 RGB24。
        """
        if int(image_format) not in [CamFormat.FORMAT_NV12, CamFormat.FORMAT_RGB24]:
            raise ValueError("SDK 仅支持 NV12 模式与 RGB24 模式这两种输出工作模式。")
        self.lib.SCAM_SetImageFormat(device_index, int(image_format))

    def open_device(self, device_index, callback_fn):
        """
        打开指定设备并开启数据捕获。
        :param device_index: 设备逻辑索引
        :param callback_fn: Python 回调函数，格式为 callback(frame_data: FrameData) -> None
        """
        # 定义内部 C 语言兼容的回调包装器
        def c_callback_wrapper(image_ptr, user_data):
            if not image_ptr:
                return
            try:
                frame_data = FrameData(image_ptr.contents)
                callback_fn(frame_data)
            except Exception as e:
                # 捕获回调中的所有异常，防止传回 C 语言层引发崩溃
                print(f"[Python Callback Exception] {e}", file=sys.stderr)

        # 转换为 C 回调函数指针，并保存在 instance 的 cache 中以防 GC 回收
        c_callback = CAM_CALLBACK_TYPE(c_callback_wrapper)
        self._c_cache_key = device_index
        self._c_callback_cache[device_index] = c_callback

        ret = self.lib.SCAM_OpenDevice(device_index, c_callback, None)
        if not ret:
            error_code = self.get_last_error()
            if device_index in self._c_callback_cache:
                del self._c_callback_cache[device_index]
            raise RuntimeError(f"OpenDevice failed: {self.get_error_text(error_code)}")
        return True

    def close_device(self, device_index):
        """关闭设备并释放其回调函数缓存"""
        ret = self.lib.SCAM_CloseDevice(device_index)
        if device_index in self._c_callback_cache:
            del self._c_callback_cache[device_index]
        return bool(ret)

    def save_to_jpg(self, buf, fmt, width, height, filename, quality=85):
        """
        将捕获的图像缓冲区数据压缩并保存为 JPG 文件（通过 SDK 内置的 turbojpeg 压缩）。
        :param buf: 图像缓冲区指针或 bytes / numpy.ndarray 数组
        :param fmt: 图像颜色格式 (CamFormat)
        :param width: 宽度
        :param height: 高度
        :param filename: 保存的文件名
        :param quality: 压缩质量 (1-100)
        """
        # 如果是 bytes 或 numpy 数组，转换为 ctypes 指针
        if isinstance(buf, (bytes, bytearray)):
            c_buf = (ctypes.c_uint8 * len(buf)).from_buffer_copy(buf)
            buf_ptr = ctypes.cast(c_buf, ctypes.POINTER(ctypes.c_uint8))
        elif hasattr(buf, "__array_interface__"):  # numpy array
            import numpy as np
            flat_arr = np.ascontiguousarray(buf, dtype=np.uint8)
            buf_ptr = flat_arr.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
        elif isinstance(buf, ctypes.POINTER(ctypes.c_uint8)):
            buf_ptr = buf
        elif isinstance(buf, int):  # 内存地址
            buf_ptr = ctypes.cast(buf, ctypes.POINTER(ctypes.c_uint8))
        else:
            raise TypeError("buf must be bytes, numpy.ndarray, or ctypes pointer")

        ret = self.lib.SCAM_SaveToJPG(buf_ptr, int(fmt), width, height, filename.encode('utf-8'), quality)
        return bool(ret)

    def get_frame_rate(self, device_index):
        """获取相机当前的实时采集帧率"""
        return self.lib.SCAM_GetFrameRate(device_index)
