import streamlit as st
from streamlit_webrtc import webrtc_streamer, VideoProcessorBase
import cv2
import pyrealsense2 as rs
import numpy as np

# ---------- RealSense 单例 ----------
class RealSenseRGB:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        return np.asanyarray(color_frame.get_data())

    def release(self):
        self.pipeline.stop()

# ---------- WebRTC 视频处理器（新接口）----------
class RSVideoProcessor(VideoProcessorBase):
    def __init__(self):
        self.realsense = RealSenseRGB()

    def recv(self, frame):
        img = self.realsense.get_frame()
        if img is None:
            return frame.to_ndarray(format="bgr24")
        return img

    # 可选：如果希望释放资源，可以在这里处理
    # def __del__(self):
    #     self.realsense.release()

# ---------- Streamlit 界面 ----------
st.title("RealSense RGB WebRTC Stream")
st.write("访问 `http://localhost:8080` 观看实时 RGB 视频")

# 使用新的 video_processor_factory 参数
ctx = webrtc_streamer(
    key="realsense-rgb",
    video_processor_factory=RSVideoProcessor,   # 改为此参数
    rtc_configuration={"iceServers": [{"urls": ["stun:stun.l.google.com:19302"]}]},
    media_stream_constraints={"video": True, "audio": False},
)

# 释放资源的简单处理（可选）
if ctx and ctx.state.playing:
    if hasattr(ctx, "video_processor") and ctx.video_processor:
        ctx.video_processor.realsense.release()