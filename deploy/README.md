# xr_teleoperate Docker 部署指南

本目录包含 `xr_teleoperate` 项目的完整 Docker 容器化部署文件。

## 目录结构

```
deploy/
├── Dockerfile                  # 多阶段构建 Dockerfile（推荐）
├── Dockerfile.slim             # 单阶段精简 Dockerfile
├── docker-compose.yml          # 多容器编排配置
├── .env.example                # 环境变量模板
├── .dockerignore               # 构建上下文排除规则
├── requirements-docker.txt     # Python 依赖清单
├── scripts/
│   ├── entrypoint.sh           # 容器入口脚本（自动检测宿主机 IP）
│   ├── docker_healthcheck.sh   # 健康检查脚本
│   └── generate_avp_cert.sh    # Apple Vision Pro 证书生成脚本
└── README.md                   # 本文件
```

## 前置要求

### 1. 宿主机环境

- **操作系统**: Ubuntu 20.04 / 22.04
- **GPU**: NVIDIA GPU with CUDA 12.x support
- **Docker**: Docker Engine 24.0+
- **NVIDIA Container Toolkit**: 已安装并配置

### 2. 安装 NVIDIA Container Toolkit

```bash
# 配置 NVIDIA 包仓库
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 安装
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 配置 Docker 运行时
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 验证
docker run --rm --gpus all nvidia/cuda:12.4.1-cudnn9-devel-ubuntu20.04 nvidia-smi
```

### 3. 初始化 Git 子模块

```bash
cd xr_teleoperate-main
git submodule update --init --depth 1
```

## 快速开始

### 1. 配置环境变量

```bash
cd xr_teleoperate-main/deploy
cp .env.example .env

# 编辑 .env 文件，设置你的网络配置
# vim .env
```

关键变量说明：

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `XR_TELEOP_ARM` | `G1_29` | 机器人臂类型：G1_29, G1_23, H1_2, H1 |
| `XR_TELEOP_INPUT_MODE` | `controller` | 输入模式：hand, controller |
| `XR_TELEOP_DISPLAY_MODE` | `immersive` | 显示模式：immersive, ego, pass-through |
| `IMG_SERVER_IP` | 自动检测 | 宿主机 IP 地址（不设置则自动检测） |
| `DDS_INTERFACE` | `eth0` | CycloneDDS 网络接口 |
| `DISPLAY` | `:0` | X11 显示号（GUI 转发用） |

> **注意**: `IMG_SERVER_IP` 是宿主机实际 IP 地址（即 XR 设备连接 WiFi 后访问的 IP）。
> 如果不设置，容器启动时会自动检测。如果检测错误，请手动设置。

### 2. 准备证书

项目自带 `teleop/televuer/cert.pem` 和 `teleop/televuer/key.pem`，Dockerfile 会自动复制到镜像中。

**对于 Apple Vision Pro**，需要额外生成 CA 证书链：

```bash
cd xr_teleoperate-main/deploy/scripts
chmod +x generate_avp_cert.sh
./generate_avp_cert.sh
# 然后将生成的 rootCA.pem 通过 AirDrop 发送到 AVP 并安装
```

### 3. 构建并启动

```bash
# 构建镜像
docker compose build

# 启动所有服务（xr-teleoperate + teleimager 必选）
docker compose up -d

# 查看所有服务状态
docker compose ps

# 查看日志
docker compose logs -f xr-teleoperate
docker compose logs -f teleimager
```

### 4. 验证安装

```bash
# 进入容器
docker compose exec xr-teleoperate bash

# 在容器内验证
python -c "import torch; print(f'PyTorch: {torch.__version__}')"
python -c "import pinocchio; print(f'Pinocchio: {pinocchio.__version__}')"
python -c "import vuer; print(f'Vuer: {vuer.__version__}')"
python -c "from televuer import TeleVuer; print('Televuer OK')"
python -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('Unitree SDK OK')"
nvidia-smi
```

## 启动遥操作

### 默认启动（已配置在 docker-compose.yml 中）

```bash
# 直接启动所有服务即可，默认参数：
#   --arm G1_29
#   --input-mode controller
#   --display-mode immersive
#   --network-interface eth0
#   --img-server-ip <自动检测或 .env 中配置>
#   --motion
#   --static-dashboard
docker compose up -d
```

### 仿真模式

```bash
docker compose exec xr-teleoperate python teleop/teleop_hand_and_arm.py \
    --sim --ee=dex3 --record
```

### 物理机器人模式（自定义参数）

```bash
docker compose exec xr-teleoperate python teleop/teleop_hand_and_arm.py \
    --arm G1_29 \
    --input-mode controller \
    --display-mode immersive \
    --network-interface eth0 \
    --img-server-ip 192.168.2.203 \
    --motion \
    --static-dashboard
```

### 查看录制数据

录制数据保存在宿主机的 `xr_teleoperate-main/data/` 目录。

## 服务说明

### xr-teleoperate（主服务）

- **功能**: XR 遥操作主程序
- **启动参数**: `--arm G1_29 --input-mode controller --display-mode immersive --network-interface eth0 --img-server-ip <主机IP> --motion --static-dashboard`
- **端口**: 8012 (WebRTC), 60001 (图像服务), 7441/7447/7557 (DDS)
- **GPU**: 需要 NVIDIA GPU

### teleimager（必选服务）

- **功能**: 头戴摄像头图像采集与 WebRTC 流媒体
- **注意**: 这是**必选服务**，必须随 xr-teleoperate 一起启动
- **部署方式**: 与 xr-teleoperate 同机部署，使用 `network_mode: host`
- **端口**: 60001
- **IP 配置**: 监听宿主机所有 IP（0.0.0.0），通过 `IMG_SERVER_IP` 环境变量指定访问地址
- **依赖**: USB 摄像头设备

### simulation（仿真服务，可选）

- **功能**: Isaac Lab 仿真环境
- **镜像**: nvcr.io/nvidia/isaac-lab:2.1.0
- **GPU**: 需要高性能 NVIDIA GPU

## 网络架构

```
┌─────────────────────────────────────────────┐
│             宿主机 (Host)                     │
│  IP: 192.168.2.203 (自动检测或手动配置)        │
│                                              │
│  ┌─────────────────┐    ┌─────────────────┐ │
│  │ xr-teleoperate  │    │   teleimager    │ │
│  │ network: host   │    │  network: host  │ │
│  │ :::8012 (WebRTC)│    │ :::60001 (WebRTC)│ │
│  │ :::7441 (DDS)   │    │ :::7441 (DDS)   │ │
│  └─────────────────┘    └─────────────────┘ │
│         ▲                      ▲             │
│         │                      │             │
│    XR Headset              USB Camera       │
│    (WiFi 连接)              (头戴摄像头)      │
└─────────────────────────────────────────────┘
```

## IP 配置说明

### 自动检测（推荐）

容器启动时，`entrypoint.sh` 会自动检测宿主机 IP 并设置 `IMG_SERVER_IP`：

```bash
[entrypoint] Auto-detected host IP: 192.168.2.203
```

### 手动配置

在 `.env` 中设置：

```env
IMG_SERVER_IP=192.168.2.203
```

### 验证 IP 设置

```bash
# 查看容器日志中的 IP 配置
docker compose logs xr-teleoperate | grep IMG_SERVER_IP

# 在容器内验证
docker compose exec xr-teleoperate env | grep IMG_SERVER_IP
```

## 证书管理

### 项目自带证书

容器启动时自动使用 `teleop/televuer/cert.pem` 和 `teleop/televuer/key.pem`。

### Apple Vision Pro 证书

```bash
# 生成 AVP 专用证书链
./scripts/generate_avp_cert.sh

# 将 rootCA.pem 通过 AirDrop 发送到 AVP
# 在 AVP 上安装为受信证书
```

### 自定义证书

将自定义证书放到 `certs/` 目录并修改 `.env`：

```env
XR_TELEOP_CERT=/workspace/certs/cert.pem
XR_TELEOP_KEY=/workspace/certs/key.pem
```

## 故障排查

### GPU 不可用

```bash
# 检查 NVIDIA Container Toolkit
docker run --rm --gpus all nvidia/cuda:12.4.1-cudnn9-devel-ubuntu20.04 nvidia-smi

# 检查容器日志
docker compose logs xr-teleoperate | grep -i gpu
```

### teleimager 启动失败

```bash
# 检查 USB 摄像头
docker compose logs teleimager

# 进入容器检查设备
docker compose exec teleimager ls -la /dev/video*
docker compose exec teleimager v4l2-ctl --list-devices
```

### XR 设备无法连接

```bash
# 1. 检查端口开放
sudo ufw allow 8012/tcp
sudo ufw allow 8012/udp

# 2. 验证证书
docker compose exec xr-teleoperate python -c "
import ssl
ctx = ssl.create_default_context()
ctx.load_cert_chain('/workspace/certs/cert.pem', '/workspace/certs/key.pem')
print('SSL OK')
"

# 3. 测试 WebRTC（在容器内）
curl -k https://localhost:8012/
```

### IP 检测失败

如果自动检测到的 IP 不正确：

```bash
# 方法1: 在 .env 中手动设置
echo "IMG_SERVER_IP=192.168.2.203" >> .env

# 方法2: 重启容器
docker compose down
docker compose up -d

# 方法3: 检查宿主机 IP
ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1'
```

### USB 摄像头无法访问

```bash
# 检查设备权限
docker compose exec teleimager ls -la /dev/video*

# 查看摄像头信息
docker compose exec teleimager v4l2-ctl --list-devices
```

### 运行健康检查

```bash
docker compose exec xr-teleoperate bash /workspace/scripts/docker_healthcheck.sh
```

## 清理

```bash
# 停止并删除容器
docker compose down

# 删除镜像
docker rmi xr-teleoperate:latest teleimager:latest

# 删除数据卷（谨慎操作，会丢失录制数据）
docker compose down -v
```

## 安全注意事项

1. **证书文件**：项目自带的证书为开发用途，生产环境请重新生成
2. **特权模式**：当前配置使用 `privileged: true`，生产环境建议使用设备映射限制权限
3. **网络模式**：使用 `host` 网络模式，确保防火墙规则正确配置
4. **机器人安全**：遥操作前确保机器人处于安全状态，周围无人

## 相关链接

- [xr_teleoperate GitHub](https://github.com/unitreerobotics/xr_teleoperate)
- [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [TeleVuer](https://github.com/silencht/televuer)
- [TeleImager](https://github.com/silencht/teleimager)


