# XR-Teleoperate — Install Guide

## 一、安装后(在 Jetson 上)

```bash
# 1. 装 deb(首次)
sudo apt install ./XR-teleoperate_<ver>_arm64.deb

# 2. 创建 conda env + 装依赖(首次,约 5-15 分钟,需要联网)
sudo -u unitree xr-teleop-control install-env

# 3. 体检
sudo -u unitree xr-teleop-control doctor

# 4. 启动服务(图像服务 + 心跳)
sudo xr-teleop-control start

# 5. 设置开机自启(可选)
sudo xr-teleop-control enable

# 6. 需要遥操作时,再手动启动主程序(高危,会接管机器人手臂;Ctrl-C 结束)
sudo xr-teleop-control teleop
```

> **高危脚本说明**:`scripts/start_teleop.sh` 会初始化 DDS 并接管机器人手臂,
> 因此它**不注册 systemd 服务、不设开机自启、不会被任何流程自动启动**,只能像上面第 6 步那样人工单独执行。
> 服务管理只覆盖 `xr-teleimager` 与 `xr-heartbeat` 两个常驻服务。

## 二、目录结构(装完后)

```
/opt/xr-teleoperate-main/      # 项目代码(只 G1_29 mesh)
/etc/systemd/system/           # xr-teleimager/xr-heartbeat unit(不含 xr-teleop)
/etc/xr-teleoperate/env.conf    # 可改:XR_TELEOP_BACKEND_URL 等
/usr/bin/xr-teleop-control      # 管理 CLI
/var/log/xr-teleoperate/        # 日志
```

## 三、常用命令

```bash
xr-teleop-control status          # 两个服务状态
xr-teleop-control teleop          # 手动启动遥操作主程序(高危)
xr-teleop-control logs xr-teleimager -f   # 图像服务日志
xr-teleop-control restart        # 重启服务(不含主程序)
xr-teleop-control doctor         # 体检
```

## 四、升级到新版本

```bash
sudo apt install ./XR-teleoperate-newer_arm64.deb
# conda env 不动(已装的 tv 保留),只更新 /opt/xr-teleoperate-main 代码
xr-teleop-control restart
```

## 五、卸载

```bash
sudo xr-teleop-control uninstall  # 等价于 systemctl stop + dpkg --purge
# 或手动:
sudo systemctl stop xr-teleimager xr-heartbeat
sudo apt purge --purge xr_teleoperate
```

## 六、换机器人型号(目前包内只 G1_29)

```bash
sudo -u unitree xr-teleop-control install-assets --arm G1_23
# 下载其它型号 mesh 到 /opt/xr-teleoperate-main/assets/<arm>/
```

## 七、配置后端地址(心跳推给哪里)

```bash
sudo nano /etc/xr-teleoperate/env.conf
# 取消注释 / 修改:
# XR_TELEOP_BACKEND_URL=http://192.168.2.117:19269
sudo xr-teleop-control restart
```

## 八、GLIBC235 兼容层(SLAM SDK)

如果 `start_camera.sh` 启动报错提到 GLIBC,手动解压到 `~/glibc235/lib/aarch64-linux-gnu/`:

```bash
mkdir -p ~/glibc235
cd ~/glibc235
tar xf /path/to/ubuntu-22.04-libc6-deb.tar.gz
# 详情见项目 README
```

## 九、故障排查

- **登录用户不匹配**:`install-env` 校验 `$HOME == /home/unitree`,否则提示
- **8012 起不来**:`journalctl -u xr-teleop -n 100`(systemd 日志)
- **网络不可达**:`xr-teleop-control doctor` 看心跳可达性
- **PICO 头显无画面**:确认浏览器接受自自证书,直接访问 `https://<robot-ip>:8012/`