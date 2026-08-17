# ============================================================
# Stage 1: Base image with CUDA + System Dependencies
# Target: Ubuntu 20.04 (focal)
# Python: 3.8 (system default on Ubuntu 20.04)
# ============================================================
FROM nvidia/cuda:12.4.1-cudnn9-devel-ubuntu20.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

RUN apt-get update && apt-get install -y --no-install-recommends locales && locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ---- System Dependencies ----
RUN apt-get update && apt-get install -y --no-install-recommends \
    git git-lfs curl wget ca-certificates gnupg2 lsb-release \
    build-essential cmake pkg-config \
    # Python 3.8 is the default on Ubuntu 20.04
    python3.8 python3.8-dev python3.8-distutils python3.8-venv \
    python3-setuptools python3-wheel \
    python3-pip \
    # Image/video processing
    libopencv-dev libjpeg-dev libpng-dev libtiff-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran \
    # Graphics & XR
    libgl1-mesa-glx libglu1-mesa libgl1-mesa-dri \
    x11-apps libx11-dev libxext-dev libxrender-dev \
    # Audio (WebRTC)
    libasound2-dev libpulse-dev \
    # Networking & ZeroMQ
    libzmq3-dev libyaml-dev \
    # Physics & IK libraries
    libboost-all-dev libspdlog-dev \
    libeigen3-dev libnlopt-dev \
    # USB / Camera access
    libusb-1.0-0-dev libudev-dev \
    # Utilities
    vim nano htop screen tmux \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip for Python 3.8
RUN python3.8 -m pip install --no-cache-dir --upgrade pip setuptools wheel

# ---- Python Virtual Environment ----
ENV VIRTUAL_ENV=/opt/xr_teleoperate/venv
RUN python3.8 -m venv ${VIRTUAL_ENV}
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

# ============================================================
# Stage 2: Install Python Dependencies
# ============================================================
FROM base AS python-deps

WORKDIR /workspace

# Copy only dependency files first (layer caching)
COPY requirements.txt .
COPY teleop/teleimager/pyproject.toml teleop/teleimager/
COPY teleop/televuer/pyproject.toml teleop/televuer/
COPY teleop/robot_control/dex-retargeting/pyproject.toml teleop/robot_control/dex-retargeting/

# Install root-level requirements
RUN pip install --no-cache-dir -r requirements.txt

# Install submodule packages (editable mode without deps to avoid conflicts)
RUN pip install --no-cache-dir \
      -e teleop/teleimager --no-deps \
    && pip install --no-cache-dir \
      -e teleop/televuer \
    && pip install --no-cache-dir \
      -e teleop/robot_control/dex-retargeting --no-deps

# Install unitree_sdk2_python (cloned separately)
RUN git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git /tmp/unitree_sdk2 \
    && cd /tmp/unitree_sdk2 \
    && pip install --no-cache-dir -e . \
    && rm -rf /tmp/unitree_sdk2

# ============================================================
# Stage 3: Final image
# ============================================================
FROM base AS final

WORKDIR /workspace

# Copy installed Python packages from python-deps stage
COPY --from=python-deps ${VIRTUAL_ENV} ${VIRTUAL_ENV}
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

# ---- Copy Application Source ----
COPY teleop/ ./teleop/
COPY assets/ ./assets/
COPY scripts/ ./scripts/
COPY *.pkl ./
COPY *.md ./

# ---- Use project-builtin certificates ----
RUN mkdir -p /workspace/certs /root/.config/xr_teleoperate \
    && cp teleop/televuer/cert.pem /workspace/certs/cert.pem \
    && cp teleop/televuer/key.pem /workspace/certs/key.pem \
    && ln -sf /workspace/certs/cert.pem /root/.config/xr_teleoperate/cert.pem \
    && ln -sf /workspace/certs/key.pem /root/.config/xr_teleoperate/key.pem

# ---- Environment Variables ----
ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    XR_TELEOP_CERT=/workspace/certs/cert.pem \
    XR_TELEOP_KEY=/workspace/certs/key.pem \
    XR_TELEOP_ARM=G1_29 \
    XR_TELEOP_FREQUENCY=30.0 \
    XR_TELEOP_INPUT_MODE=controller \
    XR_TELEOP_DISPLAY_MODE=immersive \
    IMG_SERVER_IP=192.168.2.203 \
    DDS_INTERFACE=eth0

# ---- Install entrypoint script ----
COPY scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# ---- Create directories ----
RUN mkdir -p /workspace/data /workspace/guidelogs

# ---- Exposed Ports ----
EXPOSE 8012/tcp 8012/udp
EXPOSE 60001/tcp 60001/udp
EXPOSE 7441/udp 7447/udp 7557/udp
EXPOSE 7000/tcp
EXPOSE 9090/tcp

# ---- Use entrypoint for IP auto-detection ----
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
