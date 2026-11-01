# -*- coding: utf-8 -*-
"""One-shot SYNC SDK probe: enumerate, open, capture groups, print metadata."""
import sys, time, math, threading, ctypes

SYNC_MAX_CAMERA_COUNT = 20
SYNC_IMU_SAMPLE_COUNT = 11
SYNC_MTT_SAMPLE_COUNT = 5

class SyncDeviceInfo(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("name", ctypes.c_char * 256),
        ("vidpid", ctypes.c_char * 16),
        ("path", ctypes.c_char * 256),
        ("number", ctypes.c_uint32),
        ("isExist", ctypes.c_bool),
        ("isOpened", ctypes.c_bool),
    ]

class SyncICM42688Data(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("uTime", ctypes.c_uint64),
        ("fAccData_X", ctypes.c_float),
        ("fAccData_Y", ctypes.c_float),
        ("fAccData_Z", ctypes.c_float),
        ("fGyroData_X", ctypes.c_float),
        ("fGyroData_Y", ctypes.c_float),
        ("fGyroData_Z", ctypes.c_float),
    ]

class SyncAK09940Data(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("uTime", ctypes.c_uint64),
        ("iX", ctypes.c_int32), ("iY", ctypes.c_int32), ("iZ", ctypes.c_int32),
        ("Temp", ctypes.c_float), ("iStatusBit", ctypes.c_int32),
    ]

class SyncFormatInfo(ctypes.Structure):
    _pack_ = 8
    _fields_ = [("fmt", ctypes.c_int), ("width", ctypes.c_int),
                ("height", ctypes.c_int), ("fps", ctypes.c_int)]

class SyncCamData(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("data", ctypes.POINTER(ctypes.c_uint8)),
        ("width", ctypes.c_int), ("height", ctypes.c_int),
        ("bufSize", ctypes.c_int), ("format", ctypes.c_int),
        ("startExpouseTime", ctypes.c_uint64),
        ("endExpouseTime", ctypes.c_uint64),
        ("imu_data", SyncICM42688Data * SYNC_IMU_SAMPLE_COUNT),
        ("mtt_data", SyncAK09940Data * SYNC_MTT_SAMPLE_COUNT),
        ("deviceIdx", ctypes.c_int),
        ("rawIndex", ctypes.c_uint32),
        ("isValid", ctypes.c_bool),
    ]

class SyncCamGroupData(ctypes.Structure):
    _pack_ = 8
    _fields_ = [
        ("camData", SyncCamData * SYNC_MAX_CAMERA_COUNT),
        ("deviceCount", ctypes.c_int),
    ]

lib = ctypes.CDLL("./build/libsyncsdk_bino.so")
lib.SYNC_GetVersion.restype = ctypes.c_char_p
lib.SYNC_GetErrorText.restype = ctypes.c_char_p
lib.SYNC_GetLastError.restype = ctypes.c_int
lib.SYNC_Initialize.restype = ctypes.c_int
lib.SYNC_EnumDevices.restype = ctypes.c_int
lib.SYNC_GetDeviceFormats.restype = ctypes.c_int
lib.SYNC_SetDeviceFormat.restype = ctypes.c_int
lib.SYNC_SetImageCallback.restype = ctypes.c_int
lib.SYNC_StartCapture.restype = ctypes.c_int
lib.SYNC_StartCaptureByIndices.restype = ctypes.c_int
lib.SYNC_StopCapture.restype = ctypes.c_int
lib.SYNC_IsCapturing.restype = ctypes.c_bool
ImageCallback = ctypes.CFUNCTYPE(None, ctypes.POINTER(SyncCamGroupData), ctypes.c_void_p)
# argtypes
lib.SYNC_EnumDevices.argtypes = [ctypes.POINTER(SyncDeviceInfo), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
lib.SYNC_GetDeviceFormats.argtypes = [ctypes.c_uint32, ctypes.POINTER(SyncFormatInfo), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
lib.SYNC_SetDeviceFormat.argtypes = [ctypes.c_uint32, ctypes.c_int]
lib.SYNC_SetImageCallback.argtypes = [ImageCallback, ctypes.c_void_p]
lib.SYNC_StartCapture.argtypes = [ctypes.c_int]
lib.SYNC_StartCaptureByIndices.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.c_int]

def err():
    return lib.SYNC_GetErrorText(lib.SYNC_GetLastError()).decode("utf-8", "replace")

print("version:", lib.SYNC_GetVersion().decode())
assert lib.SYNC_Initialize() == 1, "init fail " + err()

devs = (SyncDeviceInfo * SYNC_MAX_CAMERA_COUNT)()
count = ctypes.c_int()
assert lib.SYNC_EnumDevices(devs, SYNC_MAX_CAMERA_COUNT, ctypes.byref(count)) == 1, "enum fail " + err()
print("device count:", count.value)
alive = []
for i in range(SYNC_MAX_CAMERA_COUNT):
    d = devs[i]
    if d.isExist:
        alive.append(d)
        print("  dev %u: name=%r vidpid=%r path=%r opened=%d" % (d.number, d.name, d.vidpid, d.path, int(d.isOpened)))
if not alive:
    sys.exit("no devices")

idx = alive[0].number
formats = (SyncFormatInfo * 100)()
fc = ctypes.c_int()
assert lib.SYNC_GetDeviceFormats(idx, formats, 100, ctypes.byref(fc)) == 1, "formats fail"
print("formats for dev", idx)
for i in range(fc.value):
    print("   [%d] fmt=%d %dx%d %dfps" % (i, formats[i].fmt, formats[i].width, formats[i].height, formats[i].fps))

sel = 0
if fc.value > 0:
    sel = 0
assert lib.SYNC_SetDeviceFormat(idx, sel) == 1, "setfmt fail " + err()
print("set device format", sel)

groups = []
done = threading.Event()

def cb(image, user):
    if not image:
        done.set(); return
    groups.append(image.contents)
    if len(groups) >= 3:
        done.set()

cb_fn = ImageCallback(cb)
assert lib.SYNC_SetImageCallback(cb_fn, None) == 1, "setcb fail " + err()

idx_arr = (ctypes.c_int * 1)(idx)
ret = lib.SYNC_StartCaptureByIndices(idx_arr, 1)
print("StartCaptureByIndices ret:", ret, err() if ret != 1 else "")
if ret != 1:
    ret2 = lib.SYNC_StartCapture(1)
    print("StartCapture(1) ret:", ret2, err() if ret2 != 1 else "")

deadline = time.time() + 10
while not done.is_set() and time.time() < deadline:
    time.sleep(0.05)
print("groups:", len(groups), "capturing:", lib.SYNC_IsCapturing())

for gi, g in enumerate(groups[:3]):
    print("=" * 66)
    print("group %d deviceCount=%d" % (gi, g.deviceCount))
    for ci in range(min(g.deviceCount, 2)):
        c = g.camData[ci]
        print("  cam[%d] devIdx=%d rawIdx=%u valid=%d fmt=%d %dx%d buf=%d" %
              (ci, c.deviceIdx, c.rawIndex, int(c.isValid), c.format, c.width, c.height, c.bufSize))
        print("  exposure start=%d end=%d dur=%d us" % (c.startExpouseTime, c.endExpouseTime, c.endExpouseTime - c.startExpouseTime))
        ts = [c.imu_data[i].uTime for i in range(SYNC_IMU_SAMPLE_COUNT)]
        print("  imu times:", ts)
        dt = [ts[i + 1] - ts[i] for i in range(len(ts) - 1)]
        print("  imu dt:", dt)
        for i in range(SYNC_IMU_SAMPLE_COUNT):
            im = c.imu_data[i]
            print("    imu[%02d] t=%12d acc=(%9.4f,%9.4f,%9.4f) gyro=(%9.4f,%9.4f,%9.4f)" %
                  (i, im.uTime, im.fAccData_X, im.fAccData_Y, im.fAccData_Z,
                   im.fGyroData_X, im.fGyroData_Y, im.fGyroData_Z))
        a = c.imu_data[0]
        m = math.sqrt(a.fAccData_X ** 2 + a.fAccData_Y ** 2 + a.fAccData_Z ** 2)
        print("  |acc[0]| = %.4f  (1.0 => g; ~1000 => milli-g)" % m)
        if c.data and c.bufSize > 0:
            data = ctypes.string_at(c.data, c.bufSize)
            print("  saved frame bytes:", len(data))
            # Save via raw write: NV12 -> JPEG requires turbojpeg; save raw NV12 to file + Y plane PNG
            open("probe_frame_%d_%d.nv12" % (gi, ci), "wb").write(data)
            y = data[:c.width * c.height]
            try:
                import numpy as np
                yarr = np.frombuffer(y, np.uint8).reshape(c.height, c.width)
                import cv2
                cv2.imwrite("probe_frame_%d_%d_y.png" % (gi, ci), yarr)
                print("  saved Y-plane PNG 3840x1200 check ->", yarr.shape)
            except Exception as e:
                print("  numpy/cv2 save skipped:", repr(e))
lib.SYNC_StopCapture()
lib.SYNC_Release()
print("DONE")