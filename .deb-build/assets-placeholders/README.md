# assets/ 占位说明(打包时各子目录只留 .empty)

deb 包默认**只打包 `assets/g1/`**(G1_29 机器人型号)。其它型号的 mesh + urdf 在打包时被替换为占位(`<model>/.empty` + 本 README)。

## 安装时如何取回其它型号

```bash
# 联网方案(待 xr-teleop-control implement-assets 命令)
sudo -u unitree xr-teleop-control install-assets --arm G1_23
```

## 当前 deb 支持的型号(打包时打包)

- `assets/g1/` ✅

## 当前 deb 占位(需要 install-assets 命令补全)

- `assets/g1_23/`   — G1 23 自由度机器人
- `assets/h1/`      — H1
- `assets/h1_2/`    — H1.2
- `assets/brainco_hand/`
- `assets/inspire_hand/`
- `assets/unitree_hand/`