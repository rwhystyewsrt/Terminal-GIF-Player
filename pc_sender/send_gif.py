#!/usr/bin/env python3
"""
PC端GIF发送上位机
读取cat.gif, 解码缩放为RGB565, 通过USB CDC串口发送到STM32F103RET6
驱动ST7735S TFT显示屏循环播放
"""

import sys
import os
import time
import struct
import configparser
from pathlib import Path

try:
    from PIL import Image
except ImportError:
    print("错误: 需要安装 Pillow 库")
    print("请运行: pip install pillow pyserial")
    sys.exit(1)

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("错误: 需要安装 pyserial 库")
    print("请运行: pip install pyserial")
    sys.exit(1)


# ==================== 协议常量 ====================
MAGIC = b'\xDE\xAD\xBE\xEF'

CMD_FRAME_INFO   = 0x01  # 帧信息: 帧数, 宽, 高, 每帧大小
CMD_FRAME_DATA   = 0x02  # 帧数据: 帧索引, 数据大小, RGB565数据
CMD_START_PLAY   = 0x03  # 开始播放
CMD_STOP         = 0x04  # 停止
CMD_ERASE_FLASH  = 0x05  # 擦除Flash存储区

RESP_ACK    = 0x01
RESP_NACK   = 0x02
RESP_READY  = 0x03
RESP_BUSY   = 0x04
RESP_DONE   = 0x05


def read_config(config_path="config.ini"):
    """读取 config.ini 配置文件"""
    cfg = configparser.ConfigParser()
    cfg.read(config_path, encoding='utf-8')

    return {
        'gif_path': cfg.get('General', 'GifPath', fallback='./assets/cat.gif'),
        'width': cfg.getint('General', 'Width', fallback=120),
        'frame_delay_ms': cfg.getint('General', 'FrameDelayMs', fallback=50),
        'alpha_threshold': cfg.getint('General', 'AlphaThreshold', fallback=128),
        'port': cfg.get('Serial', 'Port', fallback='COM3'),
        'baud_rate': cfg.getint('Serial', 'BaudRate', fallback=2000000),
        'timeout': cfg.getint('Serial', 'Timeout', fallback=5),
    }


def list_serial_ports():
    """列出可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("  未找到可用串口!")
        return
    for p in ports:
        desc = f"  {p.device} - {p.description}"
        if p.hwid:
            desc += f" [HWID: {p.hwid}]"
        print(desc)


def decode_gif_frames(gif_path, target_width, alpha_threshold=128):
    """解码GIF并缩放所有帧为RGB565格式

    Returns:
        list of dict: [{'width': w, 'height': h, 'delay_ms': d, 'data': bytes}, ...]
    """
    print(f"正在解码GIF: {gif_path}")

    img = Image.open(gif_path)
    frames = []
    frame_idx = 0

    while True:
        try:
            img.seek(frame_idx)
        except EOFError:
            break

        # 获取帧延迟
        duration = img.info.get('duration', 50)
        if duration <= 0:
            duration = 50  # 默认50ms

        # 获取当前帧（RGBA格式）
        frame = img.convert('RGBA')

        # 计算缩放后的高度（等比例）
        orig_w, orig_h = frame.size
        if orig_w == 0 or orig_h == 0:
            frame_idx += 1
            continue

        scale = target_width / orig_w
        target_height = int(orig_h * scale)
        if target_height < 1:
            target_height = 1

        # 缩放
        frame = frame.resize((target_width, target_height), Image.LANCZOS)

        # 转换为RGB565
        rgb565 = rgba_to_rgb565(frame, alpha_threshold, target_width, target_height)

        frames.append({
            'width': target_width,
            'height': target_height,
            'delay_ms': duration,
            'data': rgb565,
        })

        frame_idx += 1
        print(f"\r  解码帧 {frame_idx}...", end='', flush=True)

    print(f"\n  共解码 {len(frames)} 帧")
    return frames


def rgba_to_rgb565(image, alpha_threshold, width, height):
    """将PIL RGBA图像转换为RGB565字节流"""
    pixels = image.load()
    result = bytearray(width * height * 2)

    for y in range(height):
        for x in range(width):
            r, g, b, a = pixels[x, y]

            if a < alpha_threshold:
                # 透明像素 → 黑色
                r, g, b = 0, 0, 0
            elif a < 255:
                # 半透明 → 混合黑色背景
                alpha_factor = a / 255.0
                r = int(r * alpha_factor)
                g = int(g * alpha_factor)
                b = int(b * alpha_factor)

            # RGB565: R(5bit) G(6bit) B(5bit)
            r5 = (r >> 3) & 0x1F
            g6 = (g >> 2) & 0x3F
            b5 = (b >> 3) & 0x1F

            pixel = (r5 << 11) | (g6 << 5) | b5
            idx = (y * width + x) * 2
            result[idx] = (pixel >> 8) & 0xFF
            result[idx + 1] = pixel & 0xFF

    return bytes(result)


def send_packet(ser, cmd, payload=b''):
    """发送协议包"""
    packet = MAGIC + bytes([cmd])
    packet += struct.pack('<H', len(payload))  # 2字节payload长度
    packet += payload
    ser.write(packet)
    ser.flush()


def recv_response(ser, timeout=10):
    """接收响应包"""
    ser.timeout = timeout

    # 读取MAGIC (4 bytes)
    magic = ser.read(4)
    if len(magic) < 4:
        return None, None

    if magic != MAGIC:
        # 同步：查找下一个MAGIC
        print(f"  警告: 收到无效MAGIC, 尝试同步...")
        return None, None

    # 读取响应码
    resp = ser.read(1)
    if len(resp) < 1:
        return None, None

    # 读取payload长度
    len_bytes = ser.read(2)
    if len(len_bytes) < 2:
        return resp[0], b''

    payload_len = struct.unpack('<H', len_bytes)[0]
    payload = ser.read(payload_len) if payload_len > 0 else b''

    return resp[0], payload


def wait_ready(ser, timeout=30):
    """等待STM32就绪"""
    print("等待STM32就绪...")
    ser.timeout = timeout
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    # 等待READY信号
    start = time.time()
    while time.time() - start < timeout:
        magic = ser.read(4)
        if magic == MAGIC:
            resp = ser.read(1)
            if resp and resp[0] == RESP_READY:
                # 读取剩余payload长度
                len_bytes = ser.read(2)
                print("  STM32已就绪!")
                return True
        elif magic:
            # 非MAGIC字节，跳过
            continue
    return False


def wait_for_response(ser, expected_codes, timeout=10):
    """等待指定响应码之一，返回 (resp_code, payload)"""
    ser.timeout = timeout
    while True:
        magic = ser.read(4)
        if len(magic) < 4:
            return None, None
        if magic != MAGIC:
            continue
        resp = ser.read(1)
        if len(resp) < 1:
            return None, None
        len_bytes = ser.read(2)
        if len(len_bytes) < 2:
            return resp[0], b''
        payload_len = struct.unpack('<H', len_bytes)[0]
        payload = ser.read(payload_len) if payload_len > 0 else b''
        if resp[0] in expected_codes:
            return resp[0], payload


def wait_ready_signal(ser, timeout=10):
    """等待STM32发送READY信号"""
    ser.timeout = timeout
    while True:
        magic = ser.read(4)
        if len(magic) < 4:
            return False
        if magic != MAGIC:
            continue
        resp = ser.read(1)
        if len(resp) < 1:
            return False
        len_bytes = ser.read(2)
        if resp[0] == RESP_READY:
            return True


def send_frames(ser, frames, frame_delay_ms):
    """发送所有帧到STM32"""
    num_frames = len(frames)
    if num_frames == 0:
        print("错误: 没有帧可发送")
        return False

    w = frames[0]['width']
    h = frames[0]['height']
    frame_data_size = w * h * 2
    total_size = num_frames * frame_data_size

    print(f"\n帧信息: {num_frames}帧, {w}x{h}, 每帧{frame_data_size}字节, 总计{total_size}字节")

    # 1. 发送帧信息
    print("\n[1/4] 发送帧信息...")
    info_payload = struct.pack('<HHHHI', num_frames, w, h, frame_data_size, frame_delay_ms)
    send_packet(ser, CMD_FRAME_INFO, info_payload)

    resp_code, _ = recv_response(ser)
    if resp_code != RESP_ACK:
        print(f"  错误: 帧信息未被确认 (响应: {resp_code})")
        return False
    print("  帧信息已确认")

    # 等待READY (STM32在ACK后发送READY)
    if not wait_ready_signal(ser):
        print("  错误: 未收到READY信号")
        return False

    # 2. 擦除Flash
    print("\n[2/4] 擦除Flash存储区...")
    send_packet(ser, CMD_ERASE_FLASH, struct.pack('<I', total_size))

    resp_code, _ = recv_response(ser, timeout=30)
    if resp_code != RESP_ACK:
        print(f"  错误: Flash擦除失败 (响应: {resp_code})")
        return False
    print("  Flash擦除完成")

    # 等待READY
    if not wait_ready_signal(ser, timeout=30):
        print("  错误: 擦除后未收到READY信号")
        return False

    # 3. 逐帧发送
    print(f"\n[3/4] 发送 {num_frames} 帧...")
    chunk_size = 512

    for i, frame in enumerate(frames):
        data = frame['data']

        # 发送帧头
        header = struct.pack('<HI', i, len(data))
        send_packet(ser, CMD_FRAME_DATA, header)

        # 分块发送帧数据
        total_sent = 0
        while total_sent < len(data):
            chunk = data[total_sent:total_sent + chunk_size]
            ser.write(chunk)
            total_sent += len(chunk)

        ser.flush()

        # 等待写入确认 (ACK + READY)
        resp_code, _ = recv_response(ser, timeout=15)
        if resp_code != RESP_ACK:
            print(f"\n  错误: 帧 {i+1} 写入失败 (响应: {resp_code})")
            return False

        # 等待READY (为下一帧准备)
        if i < num_frames - 1:
            if not wait_ready_signal(ser, timeout=10):
                print(f"\n  错误: 帧 {i+1} 后未收到READY")
                return False

        pct = (i + 1) / num_frames * 100
        print(f"\r  发送帧 {i+1}/{num_frames} ({pct:.0f}%)", end='', flush=True)

    print()

    # 4. 发送播放指令
    print("\n[4/4] 发送播放指令...")
    send_packet(ser, CMD_START_PLAY)

    resp_code, _ = recv_response(ser)
    if resp_code != RESP_ACK:
        print(f"  错误: 播放指令未被确认 (响应: {resp_code})")
        return False

    print("  播放已开始! STM32将循环播放GIF\n")
    return True


def main():
    # 查找config.ini
    script_dir = Path(__file__).parent
    config_path = script_dir / "config.ini"
    if not config_path.exists():
        config_path = Path("config.ini")

    if not config_path.exists():
        print("错误: 找不到 config.ini 文件")
        sys.exit(1)

    print(f"读取配置: {config_path}")
    config = read_config(str(config_path))

    gif_path = config['gif_path']
    target_width = config['width']
    frame_delay_ms = config['frame_delay_ms']
    alpha_threshold = config['alpha_threshold']
    port = config['port']
    baud_rate = config['baud_rate']
    timeout = config['timeout']

    # 检查GIF文件
    if not os.path.exists(gif_path):
        # 尝试相对路径
        alt_path = os.path.join(os.path.dirname(__file__), gif_path)
        if os.path.exists(alt_path):
            gif_path = alt_path
        alt_path = os.path.join(os.path.dirname(__file__), '..', gif_path)
        if os.path.exists(alt_path):
            gif_path = os.path.abspath(alt_path)
        if not os.path.exists(gif_path):
            print(f"错误: GIF文件不存在: {gif_path}")
            print("请在 config.ini 中设置正确的 GifPath")
            sys.exit(1)

    print(f"GIF路径: {gif_path}")
    print(f"目标宽度: {target_width}px")
    print(f"串口: {port} @ {baud_rate} baud")
    print()

    # 解码GIF
    frames = decode_gif_frames(gif_path, target_width, alpha_threshold)
    if not frames:
        print("错误: 未能解码任何帧")
        sys.exit(1)

    # 显示帧信息
    total_bytes = sum(len(f['data']) for f in frames)
    print(f"\n总数据量: {total_bytes:,} 字节 ({total_bytes / 1024:.1f} KB)")
    print(f"预计传输时间: {total_bytes / (baud_rate / 10):.1f} 秒 (理论)")

    # 列出可用串口
    print("\n可用串口:")
    list_serial_ports()

    # 连接串口
    print(f"\n连接串口 {port}...")
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            timeout=timeout,
            write_timeout=timeout,
        )
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 {port}: {e}")
        print("\n可用串口:")
        list_serial_ports()
        sys.exit(1)

    print(f"  已连接: {ser.name}")

    try:
        # 等待STM32就绪
        if not wait_ready(ser):
            print("错误: STM32无响应，请检查:")
            print("  1. STM32是否已烧录程序并连接USB")
            print("  2. 串口号是否正确")
            print("  3. 是否已安装STM32 USB驱动")
            sys.exit(1)

        # 发送帧
        if not send_frames(ser, frames, frame_delay_ms):
            print("发送失败!")
            sys.exit(1)

        print("=" * 50)
        print("GIF已发送完毕，STM32正在循环播放...")
        print("按 Ctrl+C 退出")
        print("=" * 50)

        # 保持运行，等待用户中断
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\n正在停止...")
        try:
            send_packet(ser, CMD_STOP)
            time.sleep(0.5)
        except Exception:
            pass
    finally:
        ser.close()
        print("串口已关闭")


if __name__ == '__main__':
    main()