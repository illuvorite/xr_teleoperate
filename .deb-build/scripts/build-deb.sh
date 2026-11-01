#!/bin/bash
# scripts/build-deb.sh — 一键构建 XR-Teleoperate deb
# 在 ~/xr_teleoperate-main/ 下运行,生成 ./XR-teleoperate_<ver>_arm64.deb
set -euo pipefail

# 路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DEB_BUILD_DIR="$PROJECT_ROOT/.deb-build"
BUILD_ROOT="/tmp/xr-teleoperate-buildroot"
VERSION="$(cat "$DEB_BUILD_DIR/VERSION" 2>/dev/null | head -1 | tr -d '[:space:]')"
[ -z "$VERSION" ] && VERSION="1.0.0"
GITREV="$(git -C "$PROJECT_ROOT" rev-parse --short HEAD 2>/dev/null || echo 'local')"
ARCH="arm64"
# 包名统一格式:<name>_v<version>_<arch>.<ext>(deb / zip 一致)。
# 部署器会用文件名反解应用名/版本/架构,并与 deb 控制信息比对:
#   - 应用名/版本必须与 debian/control 的 Package/Version 一致(control 的 Version
#     按 Debian 规范不带 v,部署器会自动剥掉文件名里的 v 前缀再比);
#   - 构建 commit 记到 card-module.json.gitSha,不拼进文件名。
PKG_NAME="xr_teleoperate"
OUT_DEB="$PROJECT_ROOT/${PKG_NAME}_v${VERSION}_${ARCH}.deb"

echo "[build-deb] PROJECT_ROOT  = $PROJECT_ROOT"
echo "[build-deb] DEB_BUILD_DIR = $DEB_BUILD_DIR"
echo "[build-deb] VERSION       = $VERSION"
echo "[build-deb] GITREV        = $GITREV"
echo "[build-deb] OUT_DEB       = $OUT_DEB"

# 0. 必备检查
command -v dpkg-deb >/dev/null || { echo "dpkg-deb not found, install: apt install -y dpkg"; exit 1; }
[ -d "$PROJECT_ROOT/scripts" ] || { echo "PROJECT_ROOT/scripts missing"; exit 1; }

# 1. 准备 build_root
rm -rf "$BUILD_ROOT"
mkdir -p "$BUILD_ROOT/DEBIAN"
mkdir -p "$BUILD_ROOT/opt/xr-teleoperate-main"
mkdir -p "$BUILD_ROOT/etc/systemd/system"
mkdir -p "$BUILD_ROOT/etc/xr-teleoperate"
mkdir -p "$BUILD_ROOT/usr/bin"

# 2. 拷贝项目源码(排除 .git / node_modules / .kilo / __pycache__)
echo "[build-deb] copying project source..."
rsync -a --exclude='.git' --exclude='.kilo' --exclude='.workbuddy-ai' --exclude='node_modules' \
  --exclude='dist' --exclude='temp' \
  --exclude='__pycache__' --exclude='*.pyc' --exclude='*.bak-*' \
  --exclude='.cache' --exclude='start_*.log' \
  "$PROJECT_ROOT/" "$BUILD_ROOT/opt/xr-teleoperate-main/"

# 3. 裁剪 assets/ — 只保留 g1/,其它型号加占位
echo "[build-deb] trimming assets..."
cd "$BUILD_ROOT/opt/xr-teleoperate-main/assets"
for d in $(ls -1); do
  if [ "$d" = "g1" ] || [ "$d" = ".gitignore" ] || [ -f "$d" ]; then continue; fi
  rm -rf "$d"
  mkdir -p "$d"
  echo "# $d placeholder — see xr-teleop-control install-assets --arm $d" > "$d/.empty"
done
cd - >/dev/null

# 4. 拷贝 deb 元数据 + 服务单元 + CLI + env.conf
cp "$DEB_BUILD_DIR/VERSION" "$BUILD_ROOT/opt/xr-teleoperate-main/VERSION"
cp "$DEB_BUILD_DIR/README.install.md" "$BUILD_ROOT/opt/xr-teleoperate-main/README.install.md"
cp "$DEB_BUILD_DIR/requirements.txt" "$BUILD_ROOT/opt/xr-teleoperate-main/.deb-build/requirements.txt"
cp "$DEB_BUILD_DIR/tv-env.yml" "$BUILD_ROOT/opt/xr-teleoperate-main/.deb-build/tv-env.yml"

cp "$DEB_BUILD_DIR/debian/control" "$BUILD_ROOT/DEBIAN/control"
cp "$DEB_BUILD_DIR/debian/postinst" "$BUILD_ROOT/DEBIAN/postinst"
cp "$DEB_BUILD_DIR/debian/prerm" "$BUILD_ROOT/DEBIAN/prerm"
cp "$DEB_BUILD_DIR/debian/postrm" "$BUILD_ROOT/DEBIAN/postrm"
cp "$DEB_BUILD_DIR/debian/conffiles" "$BUILD_ROOT/DEBIAN/conffiles"
chmod 755 "$BUILD_ROOT/DEBIAN/postinst" "$BUILD_ROOT/DEBIAN/prerm" "$BUILD_ROOT/DEBIAN/postrm"

# 注意:刻意不打包 xr-teleop.service —— 它的 ExecStart 是 scripts/start_teleop.sh
# (高危脚本,会接管机器人手臂),一旦装进 /etc/systemd/system 就等价于把它注册成
# 后台常驻服务,还可能被 enable 成开机自启。该脚本只允许现场人工单独执行。
cp "$DEB_BUILD_DIR/etc/systemd/system/xr-teleimager.service" "$BUILD_ROOT/etc/systemd/system/"
cp "$DEB_BUILD_DIR/etc/systemd/system/xr-heartbeat.service" "$BUILD_ROOT/etc/systemd/system/"

cp "$DEB_BUILD_DIR/etc/xr-teleoperate/env.conf" "$BUILD_ROOT/etc/xr-teleoperate/"
cp "$DEB_BUILD_DIR/usr/bin/xr-teleop-control" "$BUILD_ROOT/usr/bin/"
chmod 755 "$BUILD_ROOT/usr/bin/xr-teleop-control"

# 4b. CardModule 部署清单 + 服务安装脚本(供 QS_RobotDeployer 的 deb 流程使用)
#     部署器会在解压目录下 maxdepth 3 搜 install_service.sh,因此两者都放 payload 根。
cp "$DEB_BUILD_DIR/card-module.json" "$BUILD_ROOT/opt/xr-teleoperate-main/card-module.json"
cp "$PROJECT_ROOT/install_service.sh" "$BUILD_ROOT/opt/xr-teleoperate-main/install_service.sh"
chmod 755 "$BUILD_ROOT/opt/xr-teleoperate-main/install_service.sh"
BUILD_TS="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
sed -i \
  -e "s/REPLACE_WITH_BUILD_DATE_UTC/$BUILD_TS/" \
  -e "s/REPLACE_WITH_UTC_NOW/$BUILD_TS/" \
  -e "s/\"gitSha\": \"\"/\"gitSha\": \"$GITREV\"/" \
  "$BUILD_ROOT/opt/xr-teleoperate-main/card-module.json"
echo "[build-deb] card-module.json + install_service.sh 已入包(buildDate=$BUILD_TS gitSha=$GITREV)"

# 4c. payload 内只保留一份 card-module.json。
#     rsync 会把 .deb-build/ 一起带进来,其中 .deb-build/card-module.json 只是构建输入;
#     若留在包里,部署器按通配读取时可能拿到多份清单(拼接后 JSON 解析失败)。
rm -f "$BUILD_ROOT/opt/xr-teleoperate-main/.deb-build/card-module.json"

# 5. 校验 build_root 大小
SIZE=$(du -sh "$BUILD_ROOT" | cut -f1)
echo "[build-deb] build_root size = $SIZE"

# 6. 打包
echo "[build-deb] dpkg-deb --build ..."
dpkg-deb --build "$BUILD_ROOT" "$OUT_DEB"
echo "[build-deb] DONE — $OUT_DEB"
ls -lh "$OUT_DEB"

# 7. 元数据 sanity check
echo "[build-deb] === control ==="
dpkg-deb -f "$OUT_DEB"
echo "[build-deb] === file list (sample) ==="
dpkg-deb -c "$OUT_DEB" | head -20
echo "[build-deb] ..."

# 8. 清理
rm -rf "$BUILD_ROOT"
echo "[build-deb] cleaned $BUILD_ROOT"