#!/usr/bin/env python3
import serial, struct, time, sys, binascii

port = "/dev/gimbal"     # 或 /dev/ttyACM0
baud = 115200
timeout = 0.5

try:
    s = serial.Serial(port, baud, timeout=timeout)
except Exception as e:
    print("Open serial failed:", e)
    sys.exit(1)

# 确保 DTR/RTS 拉高（很多固件需要）
s.setDTR(True)
s.setRTS(True)
time.sleep(0.05)

# 构造 frame：<BB BB f f f f f f f B>
# SOF, ID, AimbotState, AimbotTarget, Pitch, Yaw, TargetPitchSpeed, TargetYawSpeed, SystemTimer, PitchRelativeAngle, YawRelativeAngle, EOF
sof = 0x55
aid = 0x02
aimbot_state = 0x01    # bit0=有目标 的示例
aimbot_target = 0x01
# 使用度为单位的值（standard4.yaml 中配置为度）
pitch = 0.0
yaw = 0.0
t_pitch_speed = 0.0
t_yaw_speed = 0.0
system_timer = 0.0
pitch_rel = 0.0
yaw_rel = 0.0
eof = 0xFF

payload = struct.pack('<BBBBfffffffB',
                      sof, aid, aimbot_state, aimbot_target,
                      pitch, yaw, t_pitch_speed, t_yaw_speed, system_timer, pitch_rel, yaw_rel,
                      eof)

print("TX:", binascii.hexlify(payload))
s.write(payload)
s.flush()
time.sleep(0.05)

# 读取上行（如果 MCU 立即回复，会看到 ID=0x01 的帧）
rx = s.read(256)
if rx:
    print("RX (hex):", binascii.hexlify(rx))
else:
    print("No RX data received (timeout {})".format(timeout))

s.close()