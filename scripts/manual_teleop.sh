#!/usr/bin/env bash
# manual_teleop.sh — 人工启停包装脚本（供 QS_RobotDeployer 等部署器调用）
#
# 契约（部署器按这些子命令调用，输出必须是 KEY=VALUE）：
#   start [--ee none|dex1|dex3|inspire|brainco]  后台拉起遥操主程序（IPC 模式，此步机器人不运动）
#   control                                      开始跟随（等价于终端按 R）
#   stop                                         停止并退出（会执行手臂回零）
#   status                                       打印 TELEOP_* 状态
#
# 约束：本脚本不注册 systemd unit、不 enable、不设开机自启，只能人工显式触发。
# 具体实现全部转调 scripts/start_teleop.sh，避免两份逻辑各自演化。
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="$SCRIPT_DIR/start_teleop.sh"

if [ ! -f "$TARGET" ]; then
    echo "ERROR=找不到 $TARGET，部署包不完整"
    exit 1
fi

exec bash "$TARGET" "$@"
