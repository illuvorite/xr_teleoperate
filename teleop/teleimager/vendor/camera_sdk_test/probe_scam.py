# -*- coding: utf-8 -*-
"""One-shot SCAM probe: enumerate, open, capture 3 frames, print full metadata.
Run from ~/camera_sdk_test with LD_LIBRARY_PATH=./build
"""
import sys, time, threading
sys.path.insert(0, ".")
from scam_sdk import ScamSDK, CamFormat

sdk = ScamSDK("build/libscam.so")
print("version:", sdk.get_version())
if not sdk.initialize():
    print("initialize FAILED:", sdk.get_error_text(sdk.get_last_error()))
    sys.exit(1)

devs = sdk.enum_devices(max_count=6)
print("devices:", devs)
if not devs:
    sdk.release(); sys.exit(1)
idx = devs[0]["number"]
print("open device", idx)
formats = sdk.get_device_formats(idx)
print("formats:")
for f in formats:
    print("   ", f)

sel = 0
for i, f in enumerate(formats):
    if f["fmt"] != CamFormat.FORMAT_YUV422:
        sel = i
        break
print("selected format idx", sel)
sdk.set_device_format(idx, sel)
sdk.set_image_format(idx, CamFormat.FORMAT_NV12)

frames = []
stop = threading.Event()

def cb(fd):
    frames.append(fd)
    if len(frames) >= 3:
        stop.set()

ok = sdk.open_device(idx, cb)
print("open ok:", ok)

deadline = time.time() + 8
while not stop.is_set() and time.time() < deadline:
    time.sleep(0.05)

print("frames captured:", len(frames))
for n, fd in enumerate(frames[:3]):
    print("=" * 60)
    print(f"frame {n}: {fd.width}x{fd.height} buf={fd.buf_size} fmt={fd.format}")
    print("  startExpouse=%d us  endExpouse=%d us  duration=%d us"
          % (fd.start_exposure_time, fd.end_exposure_time,
             fd.end_exposure_time - fd.start_exposure_time))
    ts = [imu["uTime"] for imu in fd.imu_data]
    print("  imu timestamps:", ts)
    if len(ts) >= 2:
        print("  imu dt diffs:", [t2 - t1 for t1, t2 in zip(ts, ts[1:])])
    for i, imu in enumerate(fd.imu_data):
        print("  imu[%02d] t=%12d acc=(%9.4f,%9.4f,%9.4f) gyro=(%9.4f,%9.4f,%9.4f)" %
              (i, imu["uTime"], imu["acc"][0], imu["acc"][1], imu["acc"][2],
               imu["gyro"][0], imu["gyro"][1], imu["gyro"][2]))
    # acc magnitude to sanity-check units (g expected -> |a| ~ 1.0 if g units)
    import math
    m = math.sqrt(sum(a*a for a in fd.imu_data[0]["acc"]))
    print("  |acc[0]| = %.4f  (1.0 => g units; 0.001 => mg/1000)" % m)
    for i, mt in enumerate(fd.mtt_data):
        print("  mtt[%d] t=%d mag=(%d,%d,%d) temp=%.2f status=%d" %
              (i, mt["uTime"], mt["mag"][0], mt["mag"][1], mt["mag"][2],
               mt["temp"], mt["status_bit"]))
    try:
        fn = "probe_frame_%d.jpg" % n
        buf = fd.get_nv12_bytes()
        ret = sdk.save_to_jpg(buf, fd.format, fd.width, fd.height, fn, 85)
        print("  saved", fn, "ret", ret)
    except Exception as e:
        print("  save failed:", repr(e))

try:
    sdk.close_device(idx)
except Exception as e:
    print("close err", e)
sdk.release()
print("DONE")