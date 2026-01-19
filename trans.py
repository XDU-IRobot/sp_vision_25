#!/usr/bin/env python3
"""四元数转旋转矩阵"""

import numpy as np

# ============ 在这里输入你的四元数 ============
# 格式: (w, x, y, z) - 标量在前

# 无补偿标定得到旋转
# x= -0.182176
# y= 0.179328
# z= -0.940636
# w= 0.223286

# 使用R_gimbal2imubody左乘原始4元数得到补偿后标定
x= -0.245099
y= 0.467985
z= 0.525484
w= 0.666920

# ===========================================

# 归一化
q = np.array([w, x, y, z])
q = q / np.linalg.norm(q)
w, x, y, z = q

print(f"输入四元数 (w,x,y,z): ({w:.6f}, {x:.6f}, {y:.6f}, {z:.6f})")
print(f"模长: {np.linalg.norm([w,x,y,z]):.6f}\n")

# 四元数转旋转矩阵
R = np.array([
    [1 - 2*(y**2 + z**2),     2*(x*y - w*z),     2*(x*z + w*y)],
    [    2*(x*y + w*z), 1 - 2*(x**2 + z**2),     2*(y*z - w*x)],
    [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
])

print("旋转矩阵 R =")
print(R)
print(f"\n行列式: {np.linalg.det(R):.6f}")

# 转欧拉角 (roll, pitch, yaw)
sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
if sy > 1e-6:
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arctan2(-R[2, 0], sy)
    yaw = np.arctan2(R[1, 0], R[0, 0])
else:
    roll = np.arctan2(-R[1, 2], R[1, 1])
    pitch = np.arctan2(-R[2, 0], sy)
    yaw = 0

print(f"\n欧拉角:")
print(f"  Roll:  {np.degrees(roll):8.3f}°")
print(f"  Pitch: {np.degrees(pitch):8.3f}°")
print(f"  Yaw:   {np.degrees(yaw):8.3f}°")
