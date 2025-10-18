#!/usr/bin/env python3
import argparse
import binascii
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial 未安装，请先安装: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# 默认协议参数（与你当前配置一致）
DEFAULT_SOF = 0x55
DEFAULT_EOF = 0xFF
DEFAULT_RX_ID = 0x01  # 电控->自瞄（IMU+mode）
DEFAULT_TX_ID = 0x02  # 自瞄->电控（AimbotFrame）

# 固定帧长度
RX_FRAME_LEN = 25  # GimabalImuFrame_SCM_t
TX_FRAME_LEN = 33  # AimbotFrame_SCM_t（依据 C++ 结构体定义的字段顺序计算）


def hex_line(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def parse_rx_frame(frame: bytes):
    # 结构: SOF(0), ID(1), TimeStamp(2..5), q0(6..9), q1(10..13), q2(14..17), q3(18..21), robot_id(22), mode(23), EOF(24)
    timestamp = struct.unpack_from('<I', frame, 2)[0]
    q0 = struct.unpack_from('<f', frame, 6)[0]
    q1 = struct.unpack_from('<f', frame, 10)[0]
    q2 = struct.unpack_from('<f', frame, 14)[0]
    q3 = struct.unpack_from('<f', frame, 18)[0]
    robot_id = frame[22]
    mode = frame[23]
    return {
        'timestamp': timestamp,
        'q': (q0, q1, q2, q3),
        'robot_id': robot_id,
        'mode': mode,
    }


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
    # frame[24] 是 EOF
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


def sniff(port: str, baud: int, sof: int, eof: int, rx_id: int, tx_id: int, only_rx: bool, only_tx: bool, raw: bool):
    if only_rx and only_tx:
        print("不能同时指定 --only-rx 与 --only-tx", file=sys.stderr)
        sys.exit(2)

    ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
    print(f"[sniffer] Opened {port} @ {baud} baud; SOF=0x{sof:02X} EOF=0x{eof:02X} RX_ID=0x{rx_id:02X} TX_ID=0x{tx_id:02X}")
    buf = bytearray()

    try:
        while True:
            chunk = ser.read(1)
            if not chunk:
                # idle sleep a bit
                time.sleep(0.001)
                continue
            buf += chunk

            # 对齐 SOF
            while buf and buf[0] != sof:
                buf.pop(0)

            if len(buf) < 2:
                continue
            _id = buf[1]

            # 仅 RX
            if not only_tx and _id == rx_id:
                need = RX_FRAME_LEN
                if len(buf) < need:
                    continue
                if buf[need - 1] != eof:
                    # 丢一个字节重对齐
                    buf.pop(0)
                    continue
                frame = bytes(buf[:need])
                buf = buf[need:]
                ts = time.time()
                if raw:
                    print(f"[{ts:.3f}] RX: {hex_line(frame)}")
                try:
                    parsed = parse_rx_frame(frame)
                    print(f"[{ts:.3f}] RX: robot_id={parsed['robot_id']} mode={parsed['mode']} q=({parsed['q'][0]:.3f},{parsed['q'][1]:.3f},{parsed['q'][2]:.3f},{parsed['q'][3]:.3f}) timestamp={parsed['timestamp']}")
                except Exception as e:
                    print(f"[parse][RX] error: {e}")
                continue

            # 仅 TX
            if not only_rx and _id == tx_id:
                need = TX_FRAME_LEN
                if len(buf) < need:
                    continue
                if buf[24] != eof:
                    # EOF 按当前 C++ 结构位于索引 24
                    buf.pop(0)
                    continue
                frame = bytes(buf[:need])
                buf = buf[need:]
                ts = time.time()
                if raw:
                    print(f"[{ts:.3f}] TX: {hex_line(frame)}")
                try:
                    parsed = parse_tx_frame(frame)
                    print(f"[{ts:.3f}] TX: state={parsed['Aimbotstate']} target={parsed['AimbotTarget']} yaw={parsed['Yaw']:.3f} pitch={parsed['Pitch']:.3f} y_spd={parsed['TargetYawSpeed']:.3f} p_spd={parsed['TargetPitchSpeed']:.3f}")
                except Exception as e:
                    print(f"[parse][TX] error: {e}")
                continue

            # 未知 ID 或不关心的方向，丢 1 字节重对齐，避免卡住
            buf.pop(0)

    except KeyboardInterrupt:
        print("\n[sniffer] exit by user")
    finally:
        ser.close()


def main():
    p = argparse.ArgumentParser(description="SCM 串口监听/解析器")
    p.add_argument('--port', default='/dev/gimbal', help='串口设备路径，默认 /dev/gimbal')
    p.add_argument('--baud', type=int, default=115200, help='波特率，默认 115200')
    p.add_argument('--sof', type=lambda x: int(x, 0), default=DEFAULT_SOF, help='帧头 SOF，支持 0x 前缀')
    p.add_argument('--eof', type=lambda x: int(x, 0), default=DEFAULT_EOF, help='帧尾 EOF，支持 0x 前缀')
    p.add_argument('--rx-id', type=lambda x: int(x, 0), default=DEFAULT_RX_ID, help='电控->自瞄 帧ID')
    p.add_argument('--tx-id', type=lambda x: int(x, 0), default=DEFAULT_TX_ID, help='自瞄->电控 帧ID')
    g = p.add_mutually_exclusive_group()
    g.add_argument('--only-rx', action='store_true', help='只打印电控->自瞄（RX）帧')
    g.add_argument('--only-tx', action='store_true', help='只打印自瞄->电控（TX）帧')
    p.add_argument('--raw', action='store_true', help='同时打印原始十六进制数据')

    args = p.parse_args()
    sniff(args.port, args.baud, args.sof, args.eof, args.rx_id, args.tx_id, args.only_rx, args.only_tx, args.raw)


if __name__ == '__main__':
    main()
