#!/usr/bin/env bash
# install_service.sh — CardModule 部署器调用的 systemd 安装脚本
#
# 用法(由 QS_RobotDeployer 调用):
#   sudo ./install_service.sh install   --root <deploy-dir>
#   sudo ./install_service.sh uninstall --root <deploy-dir>
#   sudo ./install_service.sh status    --root <deploy-dir>
#
# 与部署器两个开关的对应关系(重要):
#   install   ← 部署阶段。只写 unit + daemon-reload 并启动 ${SERVICE_SWITCH_SERVICE};
#                xr-teleop 相关的一切(unit 生成 / enable / start)都不做。
#   uninstall ← 「开机自启」开关关闭。disable ${SERVICE_SWITCH_SERVICE}(不 stop)。
#   「服务开关」由部署器直接操作 card-module.json 的 service.name(= xr-teleimager):
#   只做 systemctl start/stop,不经过本脚本。
#
# 高危脚本约束(必须保持):
#   scripts/start_xr_teleop.sh 会初始化 DDS 并接管机器人手臂,属于高危操作:
#     - 不生成 xr-teleop.service,即不把它注册为后台常驻服务;
#     - 不 enable(任何按钮/流程都不得把它设为开机自启),不开机自动启动;
#     - 不 start/restart(部署、安装、开机序列里都不出现)。
#   它只能由现场人员单独手动执行:
#     sudo -u unitree bash <payload>/scripts/start_xr_teleop.sh
#   部署器侧还有硬性兜底:unit 名黑名单 + ExecStart 检查,见到即拒绝 enable/start。
#
# 本模块含两个 systemd 服务:
#   xr-teleimager  头部双目相机 WebRTC 图像服务(60001);「服务开关」与「开机自启」目标
#   xr-heartbeat   本机 wlan0 地址心跳上报(把地址报给 autobot-guide-service)
#
# 两种包形态共用本脚本,脚本自行定位 payload:
#   deb : <root>/opt/xr-teleoperate-main/   (dpkg-deb -x 解出的目录树)
#   zip : <root>/                            (解压后即为 payload 根)
#
# 首次部署提示:运行依赖 conda 环境 tv。若机器人上还没有该环境,健康检查会失败,
# 需先执行(耗时较长,会下载 3GB+ 依赖):
#     sudo -u unitree xr-teleop-control install-env
# 也可用 XR_TELEOP_INSTALL_ENV=1 让本脚本在安装时顺带创建。

set -euo pipefail

# 只管理两个常驻服务;xr-teleop 刻意不在列 —— 高危脚本不得注册为 systemd 服务。
SERVICES=(xr-teleimager xr-heartbeat)
SERVICE_SWITCH_SERVICE="xr-teleimager"  # 「服务开关」/「开机自启」目标(与 card-module.json 的 service.name 一致)
ENV_DIR="/etc/xr-teleoperate"
ENV_FILE="$ENV_DIR/env.conf"
UNIT_DIR="/etc/systemd/system"
LOG_DIR="/var/log/xr-teleoperate"
SERVICE_USER="unitree"

usage() {
    cat <<EOF
用法:
    $0 <install|uninstall|status> --root <deploy-dir>

必传参数:
    --root <deploy-dir>   CardModule 解压后的部署根目录

子命令语义:
    install     安装/刷新 unit 并启动 ${SERVICE_SWITCH_SERVICE}(不登记开机自启;不涉及 xr-teleop)
    uninstall   取消 ${SERVICE_SWITCH_SERVICE} 开机自启(不 stop)
    status      输出 ${SERVICES[*]} 的 active/enabled 状态

说明:
    scripts/start_xr_teleop.sh 属于高危脚本,不生成 unit、不 enable、不 start,
    只能由现场人员单独手动执行。

可选环境变量:
    XR_TELEOP_INSTALL_ENV=1   安装时顺带创建 conda 环境 tv(耗时较长)
EOF
}

if [[ $# -lt 2 ]]; then
    usage
    exit 1
fi

ACTION="$1"
shift

ROOT_DIR=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --root)
            shift
            ROOT_DIR="${1:-}"
            ;;
        *)
            echo "未知参数: $1" >&2
            usage
            exit 1
            ;;
    esac
    shift
done

if [[ -z "$ROOT_DIR" ]]; then
    echo "缺少 --root 参数" >&2
    usage
    exit 1
fi

if [[ ! -d "$ROOT_DIR" ]]; then
    echo "部署目录不存在: $ROOT_DIR" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# 定位 payload 根(含 scripts/start_xr_teleop.sh 的那一层)
# ---------------------------------------------------------------------------
locate_payload() {
    local root="$1"

    # zip 形态:解压后就是 payload 根
    if [[ -f "$root/scripts/start_xr_teleop.sh" ]]; then
        echo "$root"
        return 0
    fi

    # deb 形态:dpkg-deb -x 解出的目录树
    if [[ -f "$root/opt/xr-teleoperate-main/scripts/start_xr_teleop.sh" ]]; then
        echo "$root/opt/xr-teleoperate-main"
        return 0
    fi

    # 兜底:在 root 下搜索(留足余量,兼容多套一层目录的打包方式)
    local found
    found="$(find "$root" -maxdepth 6 -type f -name start_xr_teleop.sh -print -quit 2>/dev/null || true)"
    if [[ -n "$found" ]]; then
        (cd "$(dirname "$found")/.." && pwd)
        return 0
    fi

    return 1
}

PAYLOAD="$(locate_payload "$ROOT_DIR" || true)"
if [[ -z "$PAYLOAD" ]]; then
    echo "未能在 $ROOT_DIR 下找到 payload(scripts/start_xr_teleop.sh)" >&2
    exit 1
fi

echo "[install_service] deploy-root = $ROOT_DIR"
echo "[install_service] payload     = $PAYLOAD"

# ---------------------------------------------------------------------------
# 生成 unit
# ---------------------------------------------------------------------------
write_units() {
    local payload="$1"

    # 刻意不生成 xr-teleop.service:
    #   ExecStart 会指向 scripts/start_xr_teleop.sh(高危脚本),一旦存在 unit,
    #   任何人都可能 systemctl enable/start 它,等价于把它注册成后台常驻服务 + 开机自启。
    #   因此这里连带清理历史版本可能残留的 unit(升级场景),确保现场不会再有该服务。
    if [[ -f "$UNIT_DIR/xr-teleop.service" ]]; then
        systemctl stop xr-teleop 2>/dev/null || true
        systemctl disable xr-teleop 2>/dev/null || true
        rm -f "$UNIT_DIR/xr-teleop.service"
        echo "[install_service] 已移除历史残留的高危 unit: xr-teleop.service(仅允许手动启动 start_xr_teleop.sh)"
    fi

    cat > "$UNIT_DIR/xr-teleimager.service" <<EOF
[Unit]
Description=XR Teleimager (Image Server) - 60001
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${SERVICE_USER}
WorkingDirectory=${payload}
EnvironmentFile=-${ENV_FILE}
ExecStart=${payload}/scripts/start_teleimager.sh
Restart=on-failure
RestartSec=5
StandardOutput=append:${LOG_DIR}/xr-teleimager.log
StandardError=append:${LOG_DIR}/xr-teleimager.log

[Install]
WantedBy=multi-user.target
EOF

    # 心跳上报本机地址(不再依赖 xr-teleop:它只允许人工手动启动,不能作为启动前置)。
    cat > "$UNIT_DIR/xr-heartbeat.service" <<EOF
[Unit]
Description=XR Teleoperator Heartbeat - reports robot address to backend
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${SERVICE_USER}
WorkingDirectory=${payload}
EnvironmentFile=-${ENV_FILE}
ExecStartPre=/bin/sleep 10
ExecStart=/usr/bin/env python3 ${payload}/scripts/heartbeat_register.py
Restart=always
RestartSec=30
StandardOutput=append:${LOG_DIR}/xr-heartbeat.log
StandardError=append:${LOG_DIR}/xr-heartbeat.log

[Install]
WantedBy=multi-user.target
EOF
}

# ---------------------------------------------------------------------------
# 子命令
# ---------------------------------------------------------------------------
do_install() {
    # 1. 运行目录
    mkdir -p "$LOG_DIR"
    chown -R "${SERVICE_USER}:${SERVICE_USER}" "$LOG_DIR" 2>/dev/null || true

    # 2. env.conf —— conffile 语义:已存在则保留现场改动
    mkdir -p "$ENV_DIR"
    if [[ ! -f "$ENV_FILE" ]]; then
        local seed=""
        for cand in "$PAYLOAD/.deb-build/etc/xr-teleoperate/env.conf" "$PAYLOAD/env.conf"; do
            if [[ -f "$cand" ]]; then
                seed="$cand"
                break
            fi
        done
        if [[ -n "$seed" ]]; then
            cp "$seed" "$ENV_FILE"
            echo "[install_service] 已从包内生成 $ENV_FILE"
        else
            cat > "$ENV_FILE" <<'EOF'
# 由 install_service.sh 生成的默认配置;完整说明见包内
# .deb-build/etc/xr-teleoperate/env.conf
XR_TELEOP_ROBOT_ID=g1_001
XR_TELEOP_PUBLIC_PORT=8012
XR_TELEOP_HEARTBEAT_INTERVAL=30
EOF
            echo "[install_service] 已生成默认 $ENV_FILE"
        fi
    else
        echo "[install_service] 保留已有 $ENV_FILE"
    fi

    # 3. 权限
    chown -R "${SERVICE_USER}:${SERVICE_USER}" "$PAYLOAD" 2>/dev/null || true
    find "$PAYLOAD/scripts" -type f -name '*.sh' -exec chmod +x {} + 2>/dev/null || true
    # 部署包是在 Windows 上打包的：解压后文件没有 unix 执行位（666，目录 777），
    # 而 scam 后端的 glibc-2.35 加载器（vendor/glibc235/.../ld-linux-aarch64.so.1）必须可执行，
    # 否则 start_teleimager.sh 只能回退到系统上的 /home/unitree/glibc235。这里统一补上。
    find "$PAYLOAD/teleop/teleimager/vendor" -name 'ld-linux*.so*' -exec chmod 755 {} + 2>/dev/null || true

    # 4. 管理 CLI(把 PROJECT_ROOT 指向本次 payload,使 install-env 也能用)
    local cli_src="$PAYLOAD/.deb-build/usr/bin/xr-teleop-control"
    if [[ -f "$cli_src" ]]; then
        sed "s#^PROJECT_ROOT=.*#PROJECT_ROOT=\"${PAYLOAD}\"#" "$cli_src" > /usr/bin/xr-teleop-control
        chmod 755 /usr/bin/xr-teleop-control
        echo "[install_service] 已安装 CLI: /usr/bin/xr-teleop-control"
    fi

    # 5. 写 unit(xr-teleimager / xr-heartbeat;不生成 xr-teleop)
    write_units "$PAYLOAD"
    systemctl daemon-reload

    # 6. 高危脚本:scripts/start_xr_teleop.sh 不生成 unit、不 enable、不 start。
    #    它只在现场需要遥操作时由人工单独执行:
    #        sudo -u ${SERVICE_USER} bash ${PAYLOAD}/scripts/start_xr_teleop.sh
    echo "[install_service] 高危脚本未注册为服务(仅允许手动启动): scripts/start_xr_teleop.sh"

    # 7. 「服务开关」目标:启动相机图像服务。
    #    部署器的「服务开关」只做 systemctl start/stop;这里的 enable 是该 unit 的包级默认
    #    (模块基线服务,需开机自启),与「开机自启」开关显示的是同一个 unit,两者互不耦合:
    #    enable/disable 只管开机自启,start/stop 只管当前运行。
    systemctl enable "$SERVICE_SWITCH_SERVICE" >/dev/null 2>&1 || true
    if systemctl restart "$SERVICE_SWITCH_SERVICE"; then
        echo "[install_service] started: ${SERVICE_SWITCH_SERVICE}"
    else
        echo "[install_service] WARN: ${SERVICE_SWITCH_SERVICE} 启动失败,查看: journalctl -u ${SERVICE_SWITCH_SERVICE} -n 200" >&2
    fi

    # 8. 心跳上报(不是 start_*.sh,只负责把本机地址报给后端)。
    #    不跑它控制台就拿不到机器人地址,只能回退静态配置,故一并拉起。
    systemctl enable xr-heartbeat >/dev/null 2>&1 || true
    systemctl restart xr-heartbeat >/dev/null 2>&1 \
        || echo "[install_service] WARN: xr-heartbeat 启动失败" >&2

    # 9. 可选:顺带创建 conda 环境(默认不做,避免部署超时)
    if [[ "${XR_TELEOP_INSTALL_ENV:-0}" == "1" && -f "$PAYLOAD/.deb-build/scripts/postinstall_env.sh" ]]; then
        echo "[install_service] XR_TELEOP_INSTALL_ENV=1,创建 conda 环境 tv(耗时较长)..."
        sudo -u "$SERVICE_USER" "$PAYLOAD/.deb-build/scripts/postinstall_env.sh" || \
            echo "[install_service] WARN: 创建 conda 环境失败,请稍后手动执行 install-env" >&2
    fi

    echo "[install_service] done. 若机器人尚无 conda 环境 tv,请先执行:"
    echo "[install_service]   sudo -u ${SERVICE_USER} xr-teleop-control install-env"
}

do_uninstall() {
    # 本脚本的 uninstall 由部署器「开机自启」开关触发,语义是"取消开机自启",
    # 不是卸载模块、也不 stop 服务(停服务属于「服务开关」)。
    systemctl disable "$SERVICE_SWITCH_SERVICE" 2>/dev/null || true
    echo "[install_service] 已取消开机自启: ${SERVICE_SWITCH_SERVICE}"
    echo "[install_service] (未停止服务,也未涉及 xr-heartbeat;高危脚本 start_xr_teleop.sh 仅手动启动)"
}

do_status() {
    for s in "${SERVICES[@]}"; do
        printf '%s: active=%s enabled=%s\n' "$s" \
            "$(systemctl is-active "$s" 2>/dev/null || true)" \
            "$(systemctl is-enabled "$s" 2>/dev/null || true)"
    done
    # 服务开关对应的 unit 状态单独输出一次,供部署器旁路读取
    systemctl is-active "$SERVICE_SWITCH_SERVICE" 2>/dev/null || true
}

case "$ACTION" in
    install)   do_install ;;
    uninstall) do_uninstall ;;
    status)    do_status ;;
    *)
        usage
        exit 1
        ;;
esac
