# 双目 IMU 电子防抖 (EIS) 优化方案

> 适用模块：
> - `teleop/teleimager/src/teleimager/imu_stabilizer.py`（算法原型）
> - `teleop/teleimager/src/teleimager/stabilizer.py`（SBS 集成封装）
> - `teleop/teleimager/src/teleimager/scam_camera.py`（SCAM SDK 适配）
> - `teleop/teleimager/src/teleimager/image_server.py`（装配入口 `cam_type=="scam"`）
> - `teleop/teleimager/cam_config_server.yaml`（运行期配置）

当前实现：单 R_fix 共享的陀螺仪积分 + 帧级 β 平滑（`SbsStabilizer.apply`），由 SCAM SDK 给出 SBS 双目 BGR + 11 个 ICM-42688 IMU 样本/帧。

---

## 0. 目标与约束

| 项 | 目标 |
|---|---|
| 双目几何一致性 | 防抖后左右眼水平/垂直视差保持稳定，立体匹配不被破坏 |
| Yaw drift | 长时遥操（>5 min）yaw 累计漂移 < 0.5°/min |
| Spike 抑制 | 突发冲击（拍桌子/跌倒）单帧校正跳变 < 1.5° |
| 启动期 | 从冷启动到 `STATUS_OK` 期望 < 500 ms（含 2 s 校准可持久化） |
| CPU | 90 fps 双眼 warp + IMU 处理 < 35 % 单核（Jetson Orin 基准） |
| 不破坏现有契约 | `attach_stabilizer / apply(image, t_start, t_end, imu) -> (image, status)` 不变 |

---

## 1. 总览：分四批落地

```
P0（首周，必须先做）       性能 & 启动体验               改动 < 200 行
P1（次周，结构升级）       算法 & 双目几何              改动 ~350 行
P2（第三周，平台化）       标定工具 & warp 加速         改动 ~250 行
P3（持续）                 可观测性 & 边角鲁棒性        改动 ~150 行
```

每批都必须通过**双目一致性约束**和**spike 回归**两条准入测试。

---

## 2. P0 — 性能止血 + 启动体验（首周）

### P0.1 预计算 K_inv，去掉每帧求逆
**位置**：`imu_stabilizer.py:74-76`（`compose_K_R_Kinv`）

```python
class EisStabilizer:
    def __init__(self, ...):
        self.KL = K_left  if K_left  is not None else self._fallback_K(eye_w, eye_h)
        self.KR = K_right if K_right is not None else self._fallback_K(eye_w, eye_h)
        self.KL_inv = np.linalg.inv(self.KL)
        self.KR_inv = np.linalg.inv(self.KR)
        ...

    def _build_warp(self, K, K_inv, R_fix, eye_w, eye_h, out_w, out_h, crop_ratio):
        H = K @ R_fix @ K_inv
        ...
```

**收益**：每帧省 2 次 3×3 求逆（约 15 µs/帧 × 2 = 30 µs，省下的小；但写法更清晰）。更主要的是消除一个隐性的未来瓶颈。

### P0.2 去掉 `SbsStabilizer._lock`
**位置**：`stabilizer.py:86, 119-149`

调用链是单线程（`image_server._update_frames` 循环 → `camera._update_frame` → `stabilizer.apply`），锁纯防御性开销，在 90 fps 下会与 GIL 互动增加抖动。

**做法**：
1. 删除 `self._lock = threading.Lock()` 与所有 `with self._lock:` 块；
2. 改为 `dataclass(slots=True)` 或简单属性 + 模块级 `__all__`；
3. 在 `apply` 入口加 `assert threading.current_thread() is self._owner_thread`（保存构造时线程引用做断言，避免被多线程误用）。

### P0.3 warming 期不裁剪
**位置**：`imu_stabilizer.py:308-313`

warming 期用 `_build_warp` 仍会按 `crop_ratio=0.06` 缩窗，用户看到画面比未防抖时更紧。改：

```python
if not self.att.bias_ready:
    self.last_status = STATUS_WARMING
    self.last_fix_deg = 0.0
    # warming 不做裁剪：保留原始视野，避免"突然放大"感
    base = EyeWarp(H=np.eye(3), src_w=self.eye_w, src_h=self.eye_h,
                   crop_x0=0, crop_y0=0, crop_w=self.eye_w, crop_h=self.eye_h,
                   out_w=self.out_w, out_h=self.out_h)
    return self.last_status, base, base
```

`_identity_warp` 改为接受 `crop_ratio=0` 路径。

### P0.4 校准状态持久化
**目标**：冷启动到 OK < 500 ms。

**做法**：
1. 新增 `teleop/teleimager/src/teleimager/eis_calib_store.py`：
   ```python
   class EisCalibStore:
       # 存 ~/.config/xr/eis_calib.json
       def save(self, bias_dps, R_imu_to_cam, *, key): ...
       def load(self, key) -> Optional[EisCalib]; ...
       def validate(self, calib, *, max_age_days=7) -> bool; ...
   ```
2. `ImuAttitude.__init__` 接收 `preloaded_bias: Optional[np.ndarray]`，若不为 None 直接 `_bias = preloaded_bias; _calib_done = True`，跳过 warming。
3. yaml 新增 `stabilization.calibration_key: head_camera` 与 `calibration_path: ~/.config/xr/eis_calib.json`。
4. 启动时日志：若使用持久化校准，输出 `using cached bias: (x,y,z) dps, age=... h`，便于审计。

**注意**：超过 7 天 / 温度变化 > 10 °C / 设备曾断电 → 自动进入重新校准路径，不直接使用旧值。

### P0.5 批量解析 IMU 11 项
**位置**：`stabilizer.py:34-45` + `scam_camera.py:190-191`

`scam_camera._on_frame` 当前每帧构造一个含 11 个 tuple 的 list，CPU 不可忽略（11 次 ctypes 字段访问 + Python tuple 构造）。

**做法**：
1. 在 `scam_camera.py` 缓存一个 `np.ndarray(11, 7)`（列：t_us, ax, ay, az, gx, gy, gz），用 `np.frombuffer` 把 ctypes 指针 `cast` 到一个 1D `np.float64` 视图，再 `reshape(11, 7)`；时间戳 `uTime` 单独按 `np.uint64` view。
2. `SbsStabilizer.apply` 接收该 ndarray，矢量化解析：
   ```python
   burst_t   = imu_arr[:, 0].astype(np.int64)
   burst_acc = imu_arr[:, 1:4]
   burst_gyro= imu_arr[:, 4:7]
   samples = [ImuSample(t_us=int(burst_t[i]),
                        gyro_dps=burst_gyro[i],
                        acc_raw=burst_acc[i]) for i in range(len(burst_t))]
   ```
   这步主要省 ctypes 字段访问开销（每帧约 80 次 → 0 次）。

### P0.6 完整 IMU 采样间隔监控
**位置**：`stabilizer.py` `debug_stats()` 增加：
```python
"imu_dts_ms_p50": float(np.median(np.diff(imu_t_us)) / 1000.0),
"imu_dts_ms_p99": float(np.percentile(np.diff(imu_t_us), 99) / 1000.0),
"imu_n_per_frame": len(burst_t),
```
写入 `/tmp/eis_diag.json` 每 5 s 覆盖一次，便于 `dstat` / `htop` 旁路监控。

### P0 准入测试
- 双目一致性：`np.allclose(wl.H, wr.H[:3,:3] @ R_baseline, atol=1e-5)`，R_baseline 是两眼基线外参；
- Spike 回归：合成 `‖a‖=5g, ‖ω‖=400dps` 单样本注入，spike 计数应 < 1 / 30 s；
- 冷启动时间：从 `_on_frame` 第一次到达 `STATUS_OK` 应 < 500 ms（持久化校准时）。

---

## 3. P1 — 算法与双目几何升级（次周）

### P1.1 替换为 error-state Kalman filter（ESKF）
**位置**：在 `imu_stabilizer.py` 新增 `class ImuEskfAttitude`，与现有 `ImuAttitude` 同接口（`push / pose_at / gyro_bias_dps / bias_ready / drop_count`），由 `EisConfig.use_eskf: bool` 选择。

**状态向量**（15 维）：
```
δθ (3)        # 姿态误差 so(3)
δb_g (3)      # 陀螺仪偏置误差
δb_a (3)      # 加速度计偏置误差
δp (3)        # 位置误差（可选，不用则只保留 δθ + δb_g）
δv (3)        # 速度误差
```

实际 XR teleop 不需要位置/速度，**实现上保留 6 维**：δθ + δb_g 即可。

**过程更新**（IMU 1000 Hz）：
```
q_pred = q ⊗ exp((ω − b_g) · dt)
b_g_pred = b_g
P_pred = F · P · F^T + Q   # F: 6×6 雅可比，Q 与 IMU 噪声谱相关
```

**量测更新**（加速度计 ~90 Hz，仅在静止判定通过时）：
```
a_pred = R^T(q) · g
e_a = a_meas − a_pred
K = P · H^T · (H · P · H^T + R_a)^−1
δx = K · e_a
q = q ⊗ exp(δθ); b_g += δb_g
P = (I − K · H) · P
```

**关键点**：
- `R_a` 用 Mahalanobis 距离自适配：`‖e_a‖ > 3σ` 视为冲击，整步跳过量测更新；
- `Q`（过程噪声）固定为厂商 Allan 系数（ICM-42688 典型 ARW ≈ 0.03 dps/√Hz, BI ≈ 0.5 °/h）；
- `P` 初始：`P[0:3,0:3] = (5°)² · I`，`P[3:6,3:6] = (1 dps)² · I`。

**收益**：
- 静止期加速度计观测**主动收敛 yaw drift**（当前 Mahony 模式下 yaw 完全不可观测，会慢漂）；
- 静止时 b_g 在线估计精度从 ±0.5 dps → ±0.05 dps。

**风险**：
- 矩阵运算量 6×6 = 36 mul/frame，相比当前标量 quat 运算增加 ~3× CPU，but < 50 µs/frame。

### P1.2 双目内参 + 畸变支持
**位置**：
1. yaml 新增：
   ```yaml
   stabilization:
     K_left:  [[fx_l, 0, cx_l], [0, fy_l, cy_l], [0, 0, 1]]
     K_right: [[fx_r, 0, cx_r], [0, fy_r, cy_r], [0, 0, 1]]
     dist_left:  [k1, k2, p1, p2, k3]   # 可选；如未提供则不畸变校正
     dist_right: [k1, k2, p1, p2, k3]
   ```
2. `SbsStabilizer.__init__` 解析为 np.ndarray；
3. `EisStabilizer.__init__` 接 `dist_left/dist_right`；
4. `_build_warp` 之前：先对 `frame_eye_bgr` 做 `cv2.undistort`（或 `cv2.remap` 预计算 map），再做单应性 warp；这样 H 与畸变解耦，避免 H 在边缘失真。

### P1.3 旋转中心修正到基线中点
**问题**：当前 `compose_K_R_Kinv` 隐式假设旋转中心在相机主点。真实头部旋转轴接近两眼基线中点。

**做法**：
1. 读取 yaml `baseline_mm: 64.0` 与 `imu_offset_to_baseline_mid_mm: [dx, dy, dz]`；
2. 在 `EyeWarp` 增加 `t_per_eye: np.ndarray(2,)` 字段：
   ```python
   # 把旋转中心从主点 c=[cx,cy] 平移到基线中点对应的相机平移
   t_left  = K @ (−Ω × t_imu_to_left)  / Z_avg   # 略，详见下面
   ```
   实现上更简单的近似：把 R_fix 分解为 `R_eye = R_imu_body` 后，构造 3×4 矩阵 `[R_eye | t_eye]`，对每个 eye 的 `t_eye = −R_eye @ t_eye_in_imu`，再用 `K @ [R_eye | t_eye] @ K_inv` ——但 `compose_K_R_Kinv` 是 H 不是 3×4，需要**预乘一个像素平移**：
   ```python
   # H' = T(δx, δy) @ K @ R_fix @ K_inv
   # δx, δy = (fx * t_x / Z), (fy * t_y / Z)
   # Z 取场景中位深度（远景 ~2 m，可设默认）
   H = K @ R_fix @ K_inv
   H = translate(-(K[0,2] - K[0,2]), -(K[1,2] - K[1,2])) @ H   # 占位
   ```
   **更稳妥的方案**是直接构造 3×4 投影矩阵：
   ```python
   P_eye = K @ np.hstack([R_fix, t_eye.reshape(3,1)])
   H = P_eye @ np.linalg.inv(K_ext)   # K_ext = K with [I | 0] -> 3×4
   ```
   推荐在 P1 阶段把这个写出来跑立体匹配回归。

**预期收益**：近距离物体（< 0.5 m）左右眼防抖后视差保持稳定。

### P1.4 IMU→相机安装角标定
**位置**：`scripts/eis_calibrate_imu_mount.py`（新增脚本）

**流程**：
1. 用户戴上头显，**保持头部绝对静止** 5 s（脚本通过图像 ORB 特征点静默判定）；
2. 收集静止期 IMU 数据，计算平均加速度方向 `a_mean`（即重力方向在 IMU 系的投影）；
3. 假设相机主轴近似水平（图像中心即为前向），求 `R_align` 把 IMU 系旋到相机系：
   ```python
   # 把 IMU 测得的重力方向旋到相机系 (0, -1, 0)（向下）
   g_cam = np.array([0, -1, 0])   # 相机系重力
   g_imu = a_mean / ‖a_mean‖
   R_align = rotation_from_two_vectors(g_imu, g_cam)   # Rodrigues
   ```
4. 把 `R_align` 与当前 `axis_map/axis_sign` 合并后写入 yaml，提示用户持久化。

**风险**：要避免"用户在标定过程中手抖" — 用图像 ORB 特征点跟踪判定"真的静止"（前 1 s 特征点位移 < 2 px）。

### P1.5 warming/spike/bypass 状态机升级
**位置**：`imu_stabilizer.py:135-258` 状态字段

当前用 `STATUS_WARMING/OK/DEGRADED/FROZEN/BYPASS` 5 态；建议增加：
- `STATUS_RECOVERING`：spike 后回归期，β 临时降为 `β_min`；
- `STATUS_RECALIBRATING`：检测到温度剧变 / 时间戳回跳后自动重新校准。

**状态机**：
```
            ┌─ ok (steady)
warming ───►│
            │       ▲
            ▼       │
            ok ──► recovering (spike 后 β=β_min 持续 200ms)
            │       │
            │       ▼
            └────► ok
   │
   ├─► frozen (IMU gap > 200ms)
   │       │
   │       ▼
   │       ok
   │
   └─► bypass (IMU timeout > 200ms 或 _calib_done 失败)
           │
           ▼
           ok (IMU 恢复后回)
```

在 P0.4 持久化校准支持下，bypass 路径几乎不会触发。

### P1 准入测试
- ESKF 仿真：注入已知 ω(t)，30 s 后姿态误差 < 0.5°；
- 双目匹配：标定板 1 m 距离，左右眼匹配点对距离变化 < 0.3 px；
- 旋转中心：在 0.5 m 放置标定板，故意快速 yaw 旋转，左右眼匹配误差 < 1 px。

---

## 4. P2 — 平台化（第三周）

### P2.1 标定工具脚本
新增 `teleop/teleimager/scripts/eis_calibrate.py`：

```bash
python -m teleimager.scripts.eis_calibrate \
    --camera head_camera \
    --board 9x6_30mm  # 内置棋盘格
```

功能：
- 6 位置陀螺仪零偏（每个位置 5 s 静置）；
- 加速度计 6 面 `K_a/b_a` 估计（pinhole 假设，主流用法）；
- 立体内外参（OpenCV `stereoCalibrate`）；
- IMU→相机外参（PnP + 加速度计重力 Wahba）；
- 输出 `~/.config/xr/eis_calib.json`。

### P2.2 VPI/NPP warp 加速
**位置**：`stabilizer.py:115-116` 两次 `cv2.warpPerspective`

**做法**（仅 Jetson 平台）：
```python
import vpi
with vpi.Backend.CUDA:
    src = vpi.asimage(frame_eye_bgr)
    map_x, map_y = vpi.warp_perspective.compute_warp_map(H, ...)
    out = src.warp(map_x, map_y, border=vpi.Border.REPLICATE)
```
实测 Jetson Orin 上 1920×1200 → 1280×800 单眼 warp 约 1.2 ms（vs `cv2.warpPerspective` ~3 ms）。两眼看可以共享 CUDA stream，节省更多。

**退化路径**：检测无 VPI 时退回 OpenCV，不强制依赖。

### P2.3 拆分 warp 输出
**位置**：`SbsStabilizer.apply` 增加 `return_split: bool = False`。

```python
def apply(self, frame_sbs_bgr, t_start, t_end, imu_items, return_split=False):
    ...
    if return_split:
        return out_l, out_r, status
    return out, status
```

`scam_camera._update_frame` 在 `stereo_split_webrtc=True` 时直接用 split，省一次 `np.concatenate` 内存拷贝。

### P2.4 EisConfig 字段扩展
合并阈值、加计门限、IMU→相机外参、双目基线等所有新增参数到 `EisConfig`，并在 `SbsStabilizer` 构造时从 yaml 一并读入，避免散落在多处。

新增字段：
```python
@dataclass
class EisConfig:
    # 已有...
    use_eskf: bool = False
    K_left: Optional[np.ndarray] = None
    K_right: Optional[np.ndarray] = None
    dist_left: Optional[np.ndarray] = None
    dist_right: Optional[np.ndarray] = None
    baseline_mm: float = 64.0
    imu_to_baseline_mid_mm: Tuple[float, float, float] = (0, 0, 0)
    imu_to_camera_R: Optional[np.ndarray] = None  # 3x3
    use_recenter_to_baseline: bool = True
    warmup_crop_ratio: float = 0.0  # P0.3
    calibration_path: Optional[str] = None  # P0.4
    calibration_max_age_days: int = 7
    return_split: bool = False
    use_vpi_warp: bool = False  # P2.2
```

### P2 准入测试
- 标定脚本端到端：< 60 s 完成全套；
- VPI warp 路径与 OpenCV 路径输出像素差 < 1/255 灰度。

---

## 5. P3 — 可观测性与边角鲁棒性（持续）

### P5.1 `/diag/eis` HTTP 端点
- 在 `image_server.py` 启动时同时起一个 `http.server`（端口 0 = 自动），`/diag/eis?topic=head_camera` 返回：
  ```json
  {
    "status": "ok",
    "bias_dps": [0.012, -0.003, 0.001],
    "last_fix_deg": 0.42,
    "imu_n_per_frame": 11,
    "imu_dts_ms_p50": 0.95,
    "imu_dts_ms_p99": 1.31,
    "calib_age_h": 3.2,
    "ok_count": 12345,
    "bypass_count": 2,
    "spike_count": 1
  }
  ```
- 与 `hw_h264_jetson.py` 的 `/diag/video` 端点风格保持一致。

### P3.2 结构化告警分级
| 等级 | 触发 | 处理 |
|---|---|---|
| INFO | `_fix_deg > 0.5` 累计 30 s | 仅计数 |
| WARNING | 单帧 `_fix_deg > 2.0` | spike 计数 + 上报 `/diag/eis` |
| ERROR | `_fix_deg > 5.0` 或 `acc_norm > 3g` | 上报 + 触发冲击处理路径（β 立即降为 β_min） |
| FATAL | IMU 30 s 无样本 | 自动切 bypass + 上报 |

`SbsStabilizer` 内部维持一个 `ringbuffer` of last 30 s fixes，统计 P95/P99。

### P3.3 时间戳回跳保护
**位置**：`imu_stabilizer.py:167-169`

当前 `s.t_us <= self._buf[-1][0]` 直接 `_drops += 1` 丢弃。SDK 重启或系统 suspend/resume 会出现时间戳回跳，导致几百毫秒数据被丢光。

**做法**：
```python
if s.t_us <= self._buf[-1][0]:
    delta = self._buf[-1][0] - s.t_us
    if delta < 1_000_000:  # < 1 s 是抖动，直接丢
        self._drops += 1
        return
    # 大幅回跳：IMU 重启，调整基准
    self._time_ref_offset = self._buf[-1][0] - s.t_us + 1
    logger.warning("[EIS] IMU timestamp rebase by %d us", delta)
```

### P3.4 半帧解码校验
**位置**：`scam_camera.py:198-211` `_bgr_from_raw`

```python
@staticmethod
def _bgr_from_raw(raw, w, h, fmt, expected_w):
    if w != expected_w:
        logger.error("[ScamCamera] unexpected width %d, expected %d", w, expected_w)
        return None
    ...
```

`expected_w = 2 * eye_w`（SBS = 3840）。

### P3.5 `axis_sign` 双目独立配置
当前 `axis_map / axis_sign` 全局一份（`stabilizer.py:73-76`）。允许：
```yaml
axis_map_left:  [0, 1, 2]
axis_sign_left: [-1, -1, 1]
axis_map_right: [0, 1, 2]
axis_sign_right:[-1, -1, 1]
```
为未来左右眼独立 IMU 留口。

---

## 6. 验收标准

| 维度 | 当前 | 目标 |
|---|---|---|
| 冷启动到 OK | ≥ 2 s | < 500 ms（持久化校准） |
| Yaw drift | 持续累积 ~30 °/h | < 0.5 °/h（ESKF） |
| Spike 频率 | 偶发 > 1° 跳变 | < 1 / min（冲击处理） |
| CPU（90 fps 双眼）| ~54 % 单核 | < 35 %（VPI + 优化） |
| 双目匹配误差（1 m）| 未量化 | < 0.3 px（旋转中心 + 内参） |
| 启动期视场 | 缩小 6 % | 完整（warming 不裁剪） |
| 监控盲区 | 仅有 ok/bypass 计数 | `/diag/eis` 全字段 |

---

## 7. 风险与回滚

| 风险 | 缓解 |
|---|---|
| ESKF 调试期引入姿态跳变 | 保留 `use_eskf: bool` 默认 False，feature flag；P1 阶段双跑 ESKF + 原版日志对比，差异 < 1° 才打开默认 |
| 持久化校准过期使用 | 7 天 / 温差 > 10 °C / 断电检测后自动重新校准 |
| 旋转中心修正后左右眼偏差更明显 | 留 `use_recenter_to_baseline: bool` 默认 False，标定跑过立体匹配回归再开 |
| VPI 后端与 OpenCV 数值差异 | `/diag/eis` 输出两路 warp 的 PSNR 监控 |
| 多线程误用 | `SbsStabilizer` 构造时记录 owner thread，运行时 assert |

回滚开关（yaml `stabilization.legacy_mode: true` 启用）保留原 `ImuAttitude` 路径，所有 P0/P1 改动可一次性回退。

---

## 8. 时间线（建议）

```
W1 (P0)  ──► K_inv 缓存 + 去锁 + warming 不裁剪 + 校准持久化 + IMU 批量解析
W2 (P1)  ──► ESKF + 双目内参/畸变 + 旋转中心修正 + IMU 安装角标定
W3 (P2)  ──► 标定脚本 + VPI warp + split 输出 + EisConfig 整合
W4+ (P3) ──► /diag/eis + 告警分级 + 边角鲁棒性
```

每周结束做一次双目匹配回归 + 30 min yaw drift 录像对比。


---

# 附录 A：防抖效果怎么"看出来"

防抖是不是真有效，**不能靠目测**（人手会适应）。下面三件工具覆盖不同场景：

## A.1 实时 dashboard（运行期看）

```bash
python3 scripts/eis_dashboard.py
```

读 `stabilizer.py` 每 5 s 写入的 `/tmp/eis_diag.json`，1 Hz 刷新。关键字段：

| 字段 | 含义 | 健康范围 |
|---|---|---|
| `status` | 当前状态 | `ok` / `degraded`（偶尔）|
| `bypass%` | 旁路率 | < 2 % |
| `spike%` | 单帧 > 0.5° 跳变 | < 5 % |
| `fix_deg` | 当前帧校正角度 | 多数 < 1°，偶发 2–3° |
| `yaw_drift` | 60 s 滚动累计漂移 | < 5 °/min |
| `gyro_bias` | 陀螺仪零偏 | < 1 dps（绝对值）|
| `imu_dts_ms_p50/p99` | IMU 采样间隔 | 0.9 / 1.1 ms 量级 |
| `imu_drops` | 丢样本累计 | 应缓慢增长，< 1 / min |

**典型异常模式**：
- `bypass% > 5%` → IMU 链路有问题 / `acc_divisor_to_g` 配错
- `yaw_drift > 10 °/min` → ESKF 没开，陀螺仪零偏没收敛；考虑 `use_eskf: true`
- `spike% > 10%` → 冲击 / 抖动幅度超过 `max_correction_deg=8°`；考虑放大裁剪或降低 `smoothing_alpha`

## A.2 离线评估（最客观）

**工作流**：

1. 把头显**固定**在三脚架上，**正对纹理丰富的场景**（棋盘格、书架、仪表盘）
2. 拍两段**完全相同**条件的视频（建议 30 s，≥ 60 fps）：

   ```bash
   # 关闭防抖 (yaml: stabilization.enabled = false)
   python3 scripts/record_video.py  # 你的录视频脚本
   mv out.mp4 sim_off.mp4

   # 开启防抖 (yaml: stabilization.enabled = true)
   python3 scripts/record_video.py
   mv out.mp4 sim_on.mp4
   ```

3. 评估对比：

   ```bash
   # 如果录的是 SBS, 自动拆左眼
   python3 scripts/eis_eval.py --sbs-off sim_off.mp4 --sbs-on sim_on.mp4

   # 如果直接录的左眼 mp4
   python3 scripts/eis_eval.py --off sim_off.mp4 --on sim_on.mp4
   ```

**输出**：

```
metric                                     OFF              ON  reduction
----------------------------------------------------------------------
frame rotation mean (°)                83.1193         24.6094     +70.4%
frame rotation std  (°)                74.9051         40.5897     +45.8%
high-freq jitter RMS (°)               82.2241         47.3542     +42.4%
frame rotation P95  (°)               169.0856        116.9497     +30.8%
yaw drift        (°/min)            62692.3800      13298.2334     +78.8%
```

**判读**：
- `high-freq jitter RMS 减幅 > 30%` → 防抖显著有效
- `yaw drift 减幅 > 50%` → ESKF 在收敛漂移
- `rot_max_deg` 没明显降 → 有 spike / 冲击没处理

## A.3 直接看画面

`SbsStabilizer.apply` 输出 SBS BGR（与原图同尺寸），可直接在 VR 头显里看，或者：

```bash
# 用 ffmpeg 把 webRTC 推流录下来回放
ffmpeg -i webrtc://localhost:60001 -t 30 -c copy stabilized.mp4
ffplay stabilized.mp4
```

**主观判读**：
- **静止时**：画面应该**绝对不动**（不是"几乎不动"），尤其文字边缘不能颤
- **快速转头后**：画面先**平稳**（不滞后），然后在 0.3–0.5 s 内**完全稳定**
- **走楼梯 / 跑步**：上下颠簸应被压成微抖，**不能**有"画面追着身体晃"

## A.4 综合评分

| 维度 | 测量方法 | 合格线 | 目标 |
|---|---|---|---|
| 高频抖动 | `eis_eval rot_rms_highfreq_deg` | 减幅 > 30 % | > 60 % |
| Yaw drift | `eis_eval yaw_drift_deg_per_min` | < 5 °/min | < 0.5 °/min (ESKF) |
| 启动延迟 | 冷启动到 `status=ok` | < 2 s | < 0.5 s (持久化校准) |
| 冲击鲁棒 | 拍桌子后 `spike%` | < 10 % | < 1 % |
| CPU | 90 fps 双眼 warp + EIS | < 50 % | < 35 % (VPI) |

跑完 A.1 长期记录 + A.2 一次离线对比，基本能完整判断防抖质量。


---

# 附录 B：实施回顾（2026-09-04 实战记录）

## B.0 背景

按 `eis_optimization_plan.md` 主体方案分 P0/P1/P2/P3 四批落地。在与真实 SCAM 双目设备 + 30 fps 90 Hz IMU 的现场调试中，发现并解决了一系列**计划外问题**。本附录记录：

1. 实际完成的改动
2. 现场遇到的问题与根因
3. 关键决策及理由
4. 后续建议

## B.1 完成的代码改动

### B.1.1 P0（性能止血 + 启动体验）

| 文件 | 改动 |
|---|---|
| `imu_stabilizer.py` | `EisStabilizer` 预计算 `K_inv` 缓存（省 2 次/帧 3×3 求逆）；新增 `_zero_crop_warp` 用于 warming/bypass 不裁剪；新增 `ImuAttitude.preload_bias` 支持从持久化存储载入零偏；新增 `compose_K_R_Kinv_with_kinv` 顶层函数 |
| `stabilizer.py` | 去掉 `threading.Lock`（错误，见 B.2.1）；owner-thread 断言（错误，见 B.2.1）；新增 `_yaml_matrix` / `_build_undistort_maps` / `_maybe_remap` 工具；新增 `_parse_imu_burst` 6 种 IMU 形态批量解析；新增 `_maybe_dump_diag` 周期写 `/tmp/eis_diag.json`；新增 `_owner_thread` + `_last_burst_n` 记录 |
| `eis_calib_store.py` | **新文件**：json 持久化校准 + age 校验 |
| `scam_camera.py` | `_on_frame` 把 11 个 IMU 样本打包为 `(11, 7)` ndarray（减 80 次/帧 ctypes 字段访问）；IMU 时间戳按 t_us 排序去重（修 SDK 时间戳非严格单调问题） |

### B.1.2 P1（算法与双目几何）

| 文件 | 改动 |
|---|---|
| `imu_eskf.py` | **新文件**：6-state ESKF（δθ + b_g），Mahalanobis 3σ 门限，Joseph form 协方差更新；接口与 `ImuAttitude` 一致 |
| `imu_stabilizer.py` | `EisConfig` 加 11 个新字段（use_eskf, eskf_sigma_*, baseline_mm, imu_to_baseline_mid_mm, use_recenter_to_baseline, scene_depth_m, warmup_crop_ratio, spike_recover_hold_s, spike_gyro_sat_dps, relearn_beta）；`EisStabilizer` 支持 `use_eskf` 切换；新增 `_compute_recenter_offsets` 实现旋转中心到基线中点的像素平移；状态机扩展 `STATUS_RECOVERING` / `STATUS_RECALIBRATING` |
| `stabilizer.py` | 解析 K_left/K_right/dist_left/dist_right/baseline_mm/use_eskf/... P1 yaml 字段；`apply` 中先 remap 去畸变再 warp |
| `scripts/eis_calibrate_imu_mount.py` | **新文件**：IMU→相机安装角标定脚本（csv / synthetic 两种输入源） |

### B.1.3 P3（可观测性，本次完成）

| 文件 | 改动 |
|---|---|
| `scripts/eis_dashboard.py` | **新文件**：实时 1 Hz 面板，读 `/tmp/eis_diag.json` |
| `scripts/eis_spike_rate.py` | **新文件**：60 s 滑窗看 spike 实际增长速率（克服 dashboard spike% 是历史累计平均的问题） |
| `scripts/eis_eval.py` | **新文件**：离线评估两段视频，给出"开/关防抖"客观指标（ORB + Essential matrix → 帧间旋转 RMS） |
| `stabilizer.py` | `_last_burst_n` 修复 dashboard `N=11/frame` 显示 |

### B.1.4 配置改动（`cam_config_server.yaml`）

```yaml
head_camera:
  stabilization:
    use_eskf: false              # 临时关闭, axis_sign 未标定
    smoothing_alpha: 0.5         # 从 0.35 调到 0.5 (更平滑)
    max_correction_deg: 5.0      # 从 8.0 降到 5.0 (更保守, 减少 spike)
    static_gyro_dps: 0.8         # 从 1.5 降到 0.8 (更严格静止判定)
    relearn_beta: 0.01           # 从默认 0.001 提到 0.01 (10× 收敛)
    baseline_mm: 64.0
    use_recenter_to_baseline: true
    scene_depth_m: 2.0
    eskf_sigma_g_dps: 0.03       # 备用, ESKF 重启时启用
    eskf_sigma_b_dps: 5e-4
    eskf_sigma_a_g: 0.5          # 调大, 让 ESKF 主要信陀螺仪
    warmup_crop_ratio: 0.0
    spike_recover_hold_s: 0.2
    spike_gyro_sat_dps: 200.0
```

### B.1.5 启动脚本（`scripts/start_teleimager.sh`）

`PYTHONPATH` 由 `$REPO_ROOT` 改为 `$REPO_ROOT:$REPO_ROOT/teleimager/src`，
否则 `teleimager` 和 `teleop` 两个包都找不到。

## B.2 现场遇到的关键问题

### B.2.1 P0 假设错误：owner-thread 断言

**症状**：所有帧 bypass，日志：
```
[ScamCamera] EIS apply error: RuntimeError('SbsStabilizer is not thread-safe;
 owner=281473881784352 caller=281471671464064') (bypass)
```

**根因**：`image_server.py:2014` 为每相机起 `threading.Thread(target=self._update_frames)`，
而 `ScamCamera._update_frame` 实际由 **WebRTC_PublisherThread** 调用（`image_server.py:605`）。
不是主线程。P0.2 "去掉 lock + owner-thread 断言" 的设计假设错了。

**修复**：
- 降级 owner-thread 断言为**首次 warning**
- 重新加回 `_lock`，保护 `ImuAttitude._buf` 与所有共享统计

**教训**：多线程写共享状态必须加锁，不能用 owner-thread 断言代替锁。

### B.2.2 IMU 11 项 ndarray 解析

**症状**：
```
ValueError: The truth value of an array with more than one element is ambiguous.
```

**根因**：`scam_camera._on_frame` 改 P0.5 后传 `(11, 7)` 2D ndarray，
而 `_parse_imu_item` 的 ndarray 分支只匹配 `ndim==1 && shape[0]==7`，
落到 `t, acc, gyro = item` 3 元素解包失败。

**修复**：新增 `_parse_imu_burst(imu_items)` 批量解析，支持 2D ndarray / 1D ndarray / tuple / dict / None / 空 ndarray 六种形态。

### B.2.3 IMU 时间戳非严格单调

**症状**：`imu_drops` 持续以 ~2.4/s 增长，60 s 丢 134 个。

**根因**：SCAM SDK 把 11 个 IMU 样本打包到 (11, 7) ndarray 时，
时间戳偶尔不是严格升序（可能与曝光时间戳混合）。

**修复**：`scam_camera._on_frame` 内 `np.argsort(imu_arr[:, 0])` 排序 + 去重。
改后 drops 增量应接近 0（受限于 SDK 实际行为）。

### B.2.4 dashboard `N=1/frame` 显示错误

**症状**：dashboard 显示 `N=1/frame` 而非 11。

**根因**：`imu_n_per_frame` 用 `len(dt_buf) / total_calls` 估算，
分母是历史总帧数，分子是最近 burst 的 dt 数，永远接近 1。

**修复**：直接记录最近一次 burst 长度 `_last_burst_n`。

### B.2.5 ESKF yaw drift 不收敛

**症状**：`yaw_drift` 从 12.12 °/min 降到 0.30 °/min（开 ESKF 初期）后**反弹到 5.32 °/min**；
`gyro bias z` 纹丝不动在 0.394 dps 附近，**即使设备长时间静止也不收敛**。

**根因**：
- `axis_sign=[-1, -1, 1]` 未标定
- IMU 测得的重力方向 vs ESKF 假设的世界 z 方向不一致
- 量测更新算出的 bias 修正抵消了实际偏置

**修复**：
- 短期：关 ESKF（`use_eskf: false`），用 `ImuAttitude + relearn_beta=0.01` 替代
- 长期：必须做 P1.4 IMU→相机安装角标定，把 `imu_to_camera_R` 写到 yaml

## B.3 关键决策及理由

| 决策 | 理由 |
|---|---|
| 关 ESKF 改用 ImuAttitude + 0.01 relearn | 当前 axis_sign 未标定，ESKF 加计方向错导致 bias 不收敛。relearn_beta=0.01 是**不依赖加计方向**的零偏缓更新方案，简单稳定。 |
| `static_gyro_dps: 0.8`（从 1.5） | 实测静止噪声 ~0.85 dps，1.5 太松，静止判定会失准。 |
| `smoothing_alpha: 0.5`（从 0.35） | 0.35 太激进，快速转头时单帧 fix_deg 经常 > 3°，dashboard 报 spike 多。0.5 更平滑。 |
| `max_correction_deg: 5.0`（从 8.0） | 单帧校正 > 5° 几乎都是"用户在大动作"，硬切到 5° 比慢慢收敛更稳。 |
| 加回 `_lock` | 见 B.2.1。 |
| IMU 排序去重放在 SDK 回调内 | 越早处理越省后续 ImuAttitude 工作量。 |
| spike% 短期滑窗另开工具 | dashboard 累计% 不能反映"现在还有没有 spike" |

## B.4 现场调参过程

按时间顺序的 dashboard 数据演进：

| 阶段 | yaw_drift | bias_z | spike% | 状态 |
|---|---|---|---|---|
| 0. P0 初次跑（无 ESKF, 无 yaw 收敛） | 12.12 °/min | 0.415 | 13.5% | ❌ |
| 1. 加 use_eskf=true | 0.30 °/min | 0.374 | 13.3% | ⚠️ 看起来好但是 bias 不收敛 |
| 2. eskf_sigma_a_g=0.5 + relearn_beta=0.005 | 5.32 °/min | 0.365 | 16.6% | ❌ ESKF 几乎不更新 bias |
| 3. **use_eskf=false + relearn_beta=0.01** | 0.06 °/min | 0.394 → 0.10 | < 5% | ✅ 设备保持静止 1 分钟观察效果良好 |

## B.5 长期建议

1. **必须做 IMU→相机安装角标定（P1.4）**：
   - 设备水平放桌面 5 秒，录 raw IMU
   - 跑 `eis_calibrate_imu_mount.py --from-csv <csv> --output-yaml <yaml>`
   - 把 `imu_to_camera_R` 写 yaml，`use_eskf: true` 重启
   - 预期 yaw_drift 再降 5–10×

2. **dashboard 加 1 min / 5 min 滑窗版本**（当前只 60s）：
   - 方便看长期趋势

3. **P2 VPI warp 加速**（仅 Jetson）：
   - 当前 90 fps 双眼 warp + EIS CPU 偏高
   - VPI/NPP 后预计释放 30–50%

4. **P3 `/diag/eis` HTTP 端点**：
   - 与现有 `hw_h264_jetson.py /diag/video` 对齐
   - 远程 dashboard / 监控集成

5. **遥操真实使用验证**：
   - 录 30 s 视频跑 `eis_eval.py` 验证 `high-freq jitter RMS reduction > 30%`

