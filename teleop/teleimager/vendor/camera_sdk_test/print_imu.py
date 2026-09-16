# -*- coding: utf-8 -*-
import os
import sys
import time
import datetime
import argparse
import threading
import ctypes

from scam_sdk import ScamSDK, CamFormat, FrameData

# 全局标志与状态
running = True
paused = False
save_requested = False

frame_count = 0
last_fps_time = None
fps_frame_count = 0
current_fps = 0.0

# 状态消息缓存（用于在60FPS高频刷新下驻留显示保存成功提示）
status_message = ""
status_message_time = 0.0

# 最新图像帧缓存（线程安全）
latest_frame = None
frozen_frame = None
frame_lock = threading.Lock()
print_lock = threading.Lock()

def save_frame_async(sdk, frame_copy, folder_name):
    global status_message, status_message_time
    try:
        # 在大文件夹下创建对应的子文件夹
        os.makedirs(folder_name, exist_ok=True)

        # 准备图像保存路径
        jpg_path = os.path.join(folder_name, "image.jpg")
        raw_bytes = frame_copy['raw_bytes']
        width = frame_copy['width']
        height = frame_copy['height']
        img_format = frame_copy['format']

        # 调用 SDK 接口根据其色彩格式压缩并保存为 JPG
        success = sdk.save_to_jpg(raw_bytes, img_format, width, height, jpg_path, 90)

        # 准备 IMU 数据保存路径并对齐格式输出
        txt_path = os.path.join(folder_name, "imu_data.txt")
        with open(txt_path, "w", encoding="utf-8") as f:
            f.write(f"相机设备逻辑号: {frame_copy['device_index']}\n")
            f.write(f"图像分辨率: {width}x{height}\n")
            f.write(f"曝光开始时间戳: {frame_copy['start_exposure_time']} us\n")
            f.write(f"曝光结束时间戳: {frame_copy['end_exposure_time']} us\n")
            f.write(f"实际曝光时长: {frame_copy['end_exposure_time'] - frame_copy['start_exposure_time']} us\n\n")

            # 使用固定宽度格式化头部，保证与数据行完全对齐
            header = f"{'Index':<6}, {'Timestamp(us)':<15}, {'Acc_X(g)':<10}, {'Acc_Y(g)':<10}, {'Acc_Z(g)':<10}, {'Gyro_X(dps)':<12}, {'Gyro_Y(dps)':<12}, {'Gyro_Z(dps)':<12}\n"
            f.write(header)

            for idx, imu in enumerate(frame_copy['imu_data']):
                acc = imu['acc']
                gyro = imu['gyro']
                # 加速度除以 1000.0 换算为 g 单元并实现与表头对齐
                row = f"{idx+1:<6d}, {imu['uTime']:<15d}, {acc[0]/1000.0:<10.4f}, {acc[1]/1000.0:<10.4f}, {acc[2]/1000.0:<10.4f}, {gyro[0]:<12.3f}, {gyro[1]:<12.3f}, {gyro[2]:<12.3f}\n"
                f.write(row)

        if success:
            msg = f"[成功] 已保存当前图像和 IMU 数据至: {folder_name}"
        else:
            msg = f"[警告] 文件夹 {folder_name} 已创建，但图像保存失败。"
    except Exception as e:
        msg = f"[错误] 保存数据失败: {e}"

    # 更新全局状态提示，驻留显示
    with print_lock:
        status_message = msg
        status_message_time = time.time()

def on_frame_captured(frame: FrameData, device_index: int, sdk: ScamSDK):
    global frame_count, fps_frame_count, last_fps_time, current_fps, save_requested, status_message, status_message_time, latest_frame, frozen_frame

    # 缓存最新的数据帧
    with frame_lock:
        latest_frame = frame

    # 统计帧率（仅在非暂停状态下统计，保证统计的客观性）
    if not paused:
        frame_count += 1
        fps_frame_count += 1
        now = time.time()
        if last_fps_time is None:
            last_fps_time = now
        elif now - last_fps_time >= 1.0:
            elapsed = now - last_fps_time
            current_fps = fps_frame_count / elapsed
            fps_frame_count = 0
            last_fps_time = now

    # 检测并处理保存数据请求
    if save_requested:
        save_requested = False
        # 如果是暂停状态，我们保存被冻结的那一帧；否则保存当前最新帧
        active_frame = frozen_frame if paused else frame
        if active_frame:
            # 同步进行深拷贝，断开与底层指针的生命周期绑定
            frame_copy = {
                # 通用方式拷贝当前格式的整张图像字节流
                'raw_bytes': ctypes.string_at(active_frame._raw_data_ptr_val, active_frame.buf_size),
                'format': active_frame.format,
                'width': active_frame.width,
                'height': active_frame.height,
                'start_exposure_time': active_frame.start_exposure_time,
                'end_exposure_time': active_frame.end_exposure_time,
                'imu_data': list(active_frame.imu_data),
                'device_index': device_index
            }
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            # 保存到大文件夹 SaveData 下的子文件夹中
            folder_name = os.path.join("SaveData", f"save_{timestamp}")

            # 将磁盘 IO 与 JPG 压缩放到后台线程，防止阻塞回调导致相机丢帧
            threading.Thread(target=save_frame_async, args=(sdk, frame_copy, folder_name), daemon=True).start()

    # 决定当前用来渲染界面的帧数据对象
    display_frame = frozen_frame if paused else frame
    if not display_frame:
        return

    # 获取当前是否需要显示状态消息（提示显示3秒）
    show_status = ""
    if status_message and (time.time() - status_message_time < 3.0):
        show_status = status_message

    with print_lock:
        # 仅移动光标到左上角，不执行 \033[2J 全屏清除，以完全消除闪烁现象
        sys.stdout.write("\033[H")

        # 1. 打印设备基本信息及帧头元数据
        state_str = "(已暂停)" if paused else "(运行中)"
        fmt_name = CamFormat.get_name(display_frame.format)
        duration = display_frame.end_exposure_time - display_frame.start_exposure_time
        print(f"=== SCAM SDK IMU 数据读取器 {state_str} ===\033[K")
        print(f"相机 {device_index}: {fmt_name} {display_frame.width}x{display_frame.height}, 大小={display_frame.buf_size}, 实时帧率={current_fps:.2f} (总帧数: {frame_count})\033[K")
        print(f"曝光: 开始={display_frame.start_exposure_time} us, 结束={display_frame.end_exposure_time} us, 曝光时长={duration} us\033[K")
        print(f"{'-' * 80}\033[K")

        # 2. 对齐打印当前帧携带的 11 组 IMU 传感器浮点值（每行显示 2 组）
        for i in range(0, 11, 2):
            if i + 1 < 11:
                imu1 = display_frame.imu_data[i]
                imu2 = display_frame.imu_data[i+1]
                print(
                    f"[{i+1:2d}] A: {imu1['acc'][0]/1000.0:6.3f},{imu1['acc'][1]/1000.0:6.3f},{imu1['acc'][2]/1000.0:6.3f} G: {imu1['gyro'][0]:6.1f},{imu1['gyro'][1]:6.1f},{imu1['gyro'][2]:6.1f} | "
                    f"[{i+2:2d}] A: {imu2['acc'][0]/1000.0:6.3f},{imu2['acc'][1]/1000.0:6.3f},{imu2['acc'][2]/1000.0:6.3f} G: {imu2['gyro'][0]:6.1f},{imu2['gyro'][1]:6.1f},{imu2['gyro'][2]:6.1f}\033[K"
                )
            else:
                imu1 = display_frame.imu_data[i]
                print(
                    f"[{i+1:2d}] A: {imu1['acc'][0]/1000.0:6.3f},{imu1['acc'][1]/1000.0:6.3f},{imu1['acc'][2]/1000.0:6.3f} G: {imu1['gyro'][0]:6.1f},{imu1['gyro'][1]:6.1f},{imu1['gyro'][2]:6.1f}\033[K"
                )

        # 3. 打印当前帧 IMU 样本的开始与结束时间戳范围
        if display_frame.imu_data:
            t_start = display_frame.imu_data[0]['uTime']
            t_end = display_frame.imu_data[-1]['uTime']
            print(f"IMU 时间戳范围: {t_start} us ~ {t_end} us (跨度: {t_end - t_start} us)\033[K")

        # 4. 显示最近的文件保存成功提示（存在时才显示）
        if show_status:
            print(f"{'-' * 80}\033[K")
            print(f"{show_status}\033[K")
        else:
            print(f"\033[K")
            print(f"\033[K")

        # 5. 在底部持续打印的操作提示，方便观看
        print(f"{'-' * 80}\033[K")
        print(f"操作提示：\033[K")
        print(f"  - 按 s 键：{'恢复终端数据刷新' if paused else '暂停终端数据刷新'}。\033[K")
        print(f"  - 按 空格 键：在当前目录的 'SaveData' 文件夹下自动创建子文件夹，\033[K")
        print(f"    保存当前图像 (image.jpg) 与对应的 11 组对齐格式化后的 IMU 传感器数据 (imu_data.txt)。\033[K")
        print(f"  - 按 Ctrl+C 键：退出程序。\033[K")

        # 清除当前输出位置到终端底部的所有内容
        sys.stdout.write("\033[J")
        sys.stdout.flush()

def select_format_interactively(sdk, target_device):
    # 获取设备支持的所有格式列表
    formats = sdk.get_device_formats(target_device)
    if not formats:
        raise RuntimeError("该设备没有支持的图像分辨率格式列表。")

    # 找到第一个非 YUV422 格式的索引作为默认选项
    default_format_idx = 0
    for i, fmt in enumerate(formats):
        if fmt.get('fmt') != CamFormat.FORMAT_YUV422:
            default_format_idx = i
            break

    # 如果是非交互式终端，直接返回默认的非 YUV422 格式索引和 NV12
    if not sys.stdin.isatty():
        print(f"检测到非交互式终端，自动选择默认配置（分辨率: {default_format_idx}, 工作模式: NV12）。")
        return default_format_idx, CamFormat.FORMAT_NV12

    # 1. 直接列出相机支持的所有分辨率与格式，用户输入索引，默认 default_format_idx
    print("\n--- 请选择相机分辨率格式与帧率 ---")
    for i, fmt in enumerate(formats):
        suffix = " (SDK暂不支持此格式)" if fmt.get('fmt') == CamFormat.FORMAT_YUV422 else ""
        print(f"  [{i}] 格式: {fmt['fmt_name']}, 分辨率: {fmt['width']}x{fmt['height']}, 帧率: {fmt['fps']} fps{suffix}")

    # 交互式读入分辨率格式选择，默认 default_format_idx
    format_choice_idx = default_format_idx
    while True:
        try:
            inp = input(f"请输入分辨率格式序号 [0-{len(formats)-1}，默认 {default_format_idx}]: ").strip()
            if not inp:
                format_choice_idx = default_format_idx
                break
            idx = int(inp)
            if 0 <= idx < len(formats):
                if formats[idx].get('fmt') == CamFormat.FORMAT_YUV422:
                    print("SDK 不支持 YUV422 分辨率格式，请选择其他格式（如 MJPG）。")
                    continue
                format_choice_idx = idx
                break
            else:
                print(f"无效的选择，请输入 0 到 {len(formats)-1} 之间的序号。")
        except ValueError:
            print("请输入有效的数字。")

    selected_fmt = formats[format_choice_idx]
    print(f"已选定摄像头格式: {selected_fmt['fmt_name']}, 分辨率: {selected_fmt['width']}x{selected_fmt['height']}, 帧率: {selected_fmt['fps']} fps")

    # 2. 选择工作模式 (选择 SDK 图像数据输出色彩格式 NV12 或 RGB24)
    sdk_formats = [
        {"name": "NV12 模式", "val": CamFormat.FORMAT_NV12},
        {"name": "RGB24 模式", "val": CamFormat.FORMAT_RGB24}
    ]
    print("\n--- 请选择输出图像数据工作模式 ---")
    for i, f_info in enumerate(sdk_formats):
        print(f"  [{i + 1}] {f_info['name']}")

    sdk_format_val = CamFormat.FORMAT_NV12
    while True:
        try:
            inp = input(f"请输入工作模式序号 [1-2，默认 1]: ").strip()
            if not inp:
                sdk_format_val = sdk_formats[0]['val']
                break
            idx = int(inp) - 1
            if 0 <= idx < len(sdk_formats):
                sdk_format_val = sdk_formats[idx]['val']
                break
            else:
                print("无效的选择，请输入 1 或 2。")
        except ValueError:
            print("请输入有效的数字。")

    sdk_fmt_name = "NV12" if sdk_format_val == CamFormat.FORMAT_NV12 else "RGB24"
    print(f"已选定工作模式: {sdk_fmt_name}\n")

    return format_choice_idx, sdk_format_val

def main():
    global running, paused, save_requested, latest_frame, frozen_frame

    parser = argparse.ArgumentParser(description="SCAM SDK 连续 IMU 数据打印 Demo")
    parser.add_argument("--device", type=int, default=-1, help="指定打开的相机设备逻辑号，默认选择第一个可用设备。")
    parser.add_argument("--format-index", type=int, default=-1, help="指定相机抓取的分辨率和格式索引。如果未指定（默认 -1），将进入交互式选择。")
    parser.add_argument("--sdk-format", type=str, default="", choices=["NV12", "RGB24"], help="指定 SDK 输出色彩格式 (NV12 或 RGB24)。如果未指定，将通过交互式选择。")
    args = parser.parse_args()

    # 首次启动执行一次全局清屏，确保后面的覆盖式重绘起点干净
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()

    # 1. 实例化 SDK 并加载依赖
    try:
        sdk = ScamSDK()
    except Exception as e:
        print(f"加载 SDK 失败: {e}", file=sys.stderr)
        return 1

    print(f"SDK 加载成功. 版本: {sdk.get_version()}")

    # 2. 初始化 SDK
    if not sdk.initialize():
        print(f"SDK 初始化失败: {sdk.get_error_text(sdk.get_last_error())}", file=sys.stderr)
        return 1

    # 初始化为交互式键盘读取环境
    old_settings = None
    fd = sys.stdin.fileno()
    if sys.stdin.isatty():
        import termios
        import tty
        old_settings = termios.tcgetattr(fd)

    try:
        # 3. 枚举设备并确定目标设备逻辑号
        devices = sdk.enum_devices()
        if not devices:
            print("错误: 未检测到任何可用相机设备。", file=sys.stderr)
            return 1

        target_device = -1
        for dev in devices:
            if dev['isExist'] and not dev['isOpened']:
                if target_device == -1 and args.device == -1:
                    target_device = dev['number']

        if args.device != -1:
            target_device = args.device
        elif target_device == -1:
            # 如果全部已打开或不存在，选择逻辑号最小的那个
            target_device = min(dev['number'] for dev in devices)

        print(f"选择打开设备逻辑号: {target_device}")

        # 获取数据格式索引与 SDK 输出色彩格式（工作模式）
        if args.format_index == -1 and not args.sdk_format:
            try:
                target_format_index, sdk_format_val = select_format_interactively(sdk, target_device)
            except KeyboardInterrupt:
                print("\n用户取消了配置选择，程序退出。")
                return 0
            except Exception as e:
                print(f"交互式选择发生异常: {e}，将自动选择默认配置", file=sys.stderr)
                target_format_index = 0
                sdk_format_val = CamFormat.FORMAT_NV12
        else:
            target_format_index = args.format_index if args.format_index != -1 else 0
            if args.sdk_format == "RGB24":
                sdk_format_val = CamFormat.FORMAT_RGB24
            else:
                sdk_format_val = CamFormat.FORMAT_NV12

        # 检查选定的格式索引有效性
        formats = sdk.get_device_formats(target_device)
        if target_format_index >= len(formats) or target_format_index < 0:
            print(f"错误: 指定的格式索引 {target_format_index} 超出范围 (0-{len(formats)-1})。", file=sys.stderr)
            return 1

        target_fmt = formats[target_format_index]
        if target_fmt.get('fmt') == CamFormat.FORMAT_YUV422:
            print(f"错误: SDK 不支持 YUV422 格式的分辨率 (格式索引 {target_format_index})。请选择其他格式分辨率。", file=sys.stderr)
            return 1

        print(f"正在设置捕获格式索引: {target_format_index} (分辨率 {target_fmt['width']}x{target_fmt['height']} @ {target_fmt['fps']}fps)")
        sdk.set_device_format(target_device, target_format_index)

        # 设置目标输出色彩格式 (NV12 或 RGB24)
        sdk_fmt_name = "NV12" if sdk_format_val == CamFormat.FORMAT_NV12 else "RGB24"
        print(f"正在设置工作模式为: {sdk_fmt_name}")
        sdk.set_image_format(target_device, sdk_format_val)

        # 启动抓图并注册帧数据回调
        sdk.open_device(target_device, lambda frame: on_frame_captured(frame, target_device, sdk))

        print("\n采集流成功开启！")
        print("按 's' 键可暂停/恢复控制台打印（数据继续采集），按 空格 键保存当前图像及 IMU 数据，按 Ctrl+C 退出程序。")

        # 若是标准交互式 TTY，则启用 cbreak 模式实现单个字符即时监听
        if old_settings:
            import select
            tty.setcbreak(fd)
            # 在切换模式后，再次清屏，防止之前的缓冲干扰首帧渲染
            sys.stdout.write("\033[2J\033[H")
            sys.stdout.flush()

        # 主交互键盘循环
        while running:
            if old_settings:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    ch = sys.stdin.read(1)
                    if ch == '\x03':  # cbreak 模式下的 Ctrl+C
                        running = False
                        break
                    elif ch.lower() == 's':
                        paused = not paused
                        if paused:
                            # 暂停时，捕获当前最新一帧用于保持在屏幕上冻结显示
                            with frame_lock:
                                frozen_frame = latest_frame
                        else:
                            # 恢复时，清空冻结帧引用，恢复流动显示
                            frozen_frame = None
                    elif ch == ' ':
                        save_requested = True
            else:
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C 中断信号，准备释放资源并退出...")
    except Exception as e:
        print(f"\n程序运行发生异常: {e}", file=sys.stderr)
    finally:
        # 恢复终端的原始 termios配置
        if old_settings:
            import termios
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print("已恢复原始终端设置。")

        # 优雅关闭设备和释放 SDK
        print("正在关闭相机设备...")
        try:
            sdk.close_device(target_device)
        except Exception:
            pass

        print("正在释放 SDK 资源...")
        try:
            sdk.release()
        except Exception:
            pass

        print("SDK 资源已完全释放，程序安全退出。")

if __name__ == "__main__":
    sys.exit(main())
