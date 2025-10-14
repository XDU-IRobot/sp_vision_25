#!/usr/bin/env python3
import serial
import struct
import math

# 串口配置
# PORT = "/dev/ttyACM0"
# BAUDRATE = 115200  # 根据你的设备调整
PORT  ="/dev/gimbal"
BAUDRATE =328125
# 每帧长度（根据实际协议修改）
FRAME_LEN = 32
SOF = 0x55  # 帧头

def quaternion_to_euler(x, y, z, w):
    """
    四元数 -> 欧拉角 (Yaw, Pitch, Roll)
    输出单位为度
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # 转为度
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)

def parse_frame(frame_bytes):
    """
    根据你的数据帧解析四元数
    假设四元数在固定位置，这里以 float32 每个分量 4 字节为例
    """
    # 示例：假设四元数从第 6 个字节开始，顺序为 x, y, z, w
    q_bytes = frame_bytes[6:22]  # 4 个 float，每个 4 字节
    q = struct.unpack('<ffff', q_bytes)  # little-endian
    return q

def main():
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"Connected to {PORT} at {BAUDRATE} baud")

    buffer = bytearray()
    while True:
        data = ser.read(ser.in_waiting or 1)
        if not data:
            continue
        buffer += data

        while len(buffer) >= FRAME_LEN:
            # 找到帧头
            if buffer[0] != SOF:
                buffer.pop(0)
                continue

            frame = buffer[:FRAME_LEN]
            buffer = buffer[FRAME_LEN:]

            try:
                q = parse_frame(frame)
                yaw, pitch, roll = quaternion_to_euler(*q)
                print(f"Euler angles: Yaw={yaw:.2f}°, Pitch={pitch:.2f}°, Roll={roll:.2f}°")
            except struct.error:
                print("Frame parse error")

if __name__ == "__main__":
    main()
