#!/usr/bin/env python3
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial 未安装，请先安装: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# 默认协议参数（与工程当前配置保持一致）
DEFAULT_SOF = 0x55
DEFAULT_EOF = 0xFF
DEFAULT_TX_ID = 0x02  # 自瞄->电控（AimbotFrame）
TX_FRAME_LEN = 33     # AimbotFrame_SCM_t 总字节数


def hex_line(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def parse_tx_frame(frame: bytes):
    # 结构: SOF(0) ID(1) Aimbotstate(2) AimbotTarget(3) Pitch(4..7) Yaw(8..11)
    #       TargetPitchSpeed(12..15) TargetYawSpeed(16..19) SystemTimer(20..23)
    #       EOF(24) PitchRelativeAngle(25..28) YawRelativeAngle(29..32)
    aimbot_state = frame[2]
    aimbot_target = frame[3]
    pitch = struct.unpack_from('<f', frame, 4)[0]
    yaw = struct.unpack_from('<f', frame, 8)[0]
    t_pitch_spd = struct.unpack_from('<f', frame, 12)[0]
    t_yaw_spd = struct.unpack_from('<f', frame, 16)[0]
    system_timer = struct.unpack_from('<f', frame, 20)[0]
    pitch_rel = struct.unpack_from('<f', frame, 25)[0]
    yaw_rel = struct.unpack_from('<f', frame, 29)[0]
    return {
        'Aimbotstate': aimbot_state,
        'AimbotTarget': aimbot_target,
        'Pitch': pitch,
        'Yaw': yaw,
        'TargetPitchSpeed': t_pitch_spd,
        'TargetYawSpeed': t_yaw_spd,
        'SystemTimer': system_timer,
        'PitchRelativeAngle': pitch_rel,
        'YawRelativeAngle': yaw_rel,
    }


def view_tx(port: str, baud: int, sof: int, eof: int, tx_id: int, raw: bool):
    # 尝试以非独占方式打开串口，方便与主程序并行观察（具体是否可行取决于驱动）
    ser = serial.Serial(port=port, baudrate=baud, timeout=0.1, exclusive=False)
    print(f"[tx-view] Opened {port} @ {baud} baud; SOF=0x{sof:02X} EOF=0x{eof:02X} TX_ID=0x{tx_id:02X}")
    buf = bytearray()

    try:
        while True:
            try:
                b = ser.read(1)
            except serial.SerialException as e:
                # 设备被占用或瞬断：等待并重试
                print(f"[tx-view] read error: {e}. Will retry...", file=sys.stderr)
                time.sleep(0.2)
                try:
                    if not ser.is_open:
                        ser.open()
                except Exception:
                    pass
                continue
            if not b:
                time.sleep(0.001)
                continue
            buf += b

            # 对齐 SOF
            while buf and buf[0] != sof:
                buf.pop(0)

            if len(buf) < 2:
                continue
            _id = buf[1]
            if _id != tx_id:
                # 非 TX 帧，丢 1 字节重对齐
                buf.pop(0)
                continue

            need = TX_FRAME_LEN
            if len(buf) < need:
                continue
            # EOF 在索引 24
            if buf[24] != eof:
                buf.pop(0)
                continue

            frame = bytes(buf[:need])
            buf = buf[need:]
            ts = time.time()

            if raw:
                print(f"[{ts:.3f}] TX: {hex_line(frame)}")
            try:
                p = parse_tx_frame(frame)
                print(
                    f"[{ts:.3f}] TX: state={p['Aimbotstate']} target={p['AimbotTarget']} "
                    f"yaw={p['Yaw']:.3f} pitch={p['Pitch']:.3f} "
                    f"y_spd={p['TargetYawSpeed']:.3f} p_spd={p['TargetPitchSpeed']:.3f} timer={p['SystemTimer']:.3f}"
                )
            except Exception as e:
                print(f"[parse][TX] error: {e}")
    except KeyboardInterrupt:
        print("\n[tx-view] exit by user")
    finally:
        ser.close()


def main():
    ap = argparse.ArgumentParser(description="仅查看自瞄->电控（TX）帧的串口监听器")
    ap.add_argument('--port', default='/dev/gimbal', help='串口设备，默认 /dev/gimbal')
    ap.add_argument('--baud', type=int, default=115200, help='波特率，默认 115200')
    ap.add_argument('--sof', type=lambda x: int(x, 0), default=DEFAULT_SOF, help='帧头 SOF，支持 0x 前缀')
    ap.add_argument('--eof', type=lambda x: int(x, 0), default=DEFAULT_EOF, help='帧尾 EOF，支持 0x 前缀')
    ap.add_argument('--tx-id', type=lambda x: int(x, 0), default=DEFAULT_TX_ID, help='自瞄->电控 帧ID，默认 0x02')
    ap.add_argument('--raw', action='store_true', help='同时打印原始十六进制')
    args = ap.parse_args()

    view_tx(args.port, args.baud, args.sof, args.eof, args.tx_id, args.raw)


if __name__ == '__main__':
    main()
