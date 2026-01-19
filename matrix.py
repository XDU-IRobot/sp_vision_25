#!/usr/bin/env python3
"""
旋转矩阵计算脚本
计算给定矩阵的逆矩阵，并支持左乘操作
"""

import numpy as np

# ============ 定义矩阵 ============
# 矩阵格式：[第1行, 第2行, 第3行]
# [0,  0, 1]
# [-1, 0, 0]
# [0,  1, 0]

M = np.array([
    [0,  0, 1],
    [-1, 0, 0],
    [0,  1, 0]
])

print("=" * 60)
print("原始矩阵 M:")
print("=" * 60)
print(M)
print()

# ============ 计算逆矩阵 ============
M_inv = np.linalg.inv(M)

print("=" * 60)
print("逆矩阵 M^(-1):")
print("=" * 60)
print(M_inv)
print()

# ============ 验证：M * M^(-1) = I ============
I = np.dot(M, M_inv)
print("=" * 60)
print("验证 M × M^(-1) = I (单位矩阵):")
print("=" * 60)
print(I)
print()

# ============ 计算行列式 ============
det = np.linalg.det(M)
print("=" * 60)
print(f"行列式 det(M) = {det:.6f}")
print("(旋转矩阵的行列式应该为 ±1)")
print("=" * 60)
print()

# ============ 左乘示例 ============
print("=" * 60)
print("左乘操作示例:")
print("=" * 60)

# 示例：用一个旋转矩阵 R 左乘 M^(-1)
# 你可以修改这个矩阵为你需要的旋转矩阵
R = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
])

print("\n旋转矩阵 R (示例，单位矩阵):")
print(R)
print()

# 计算 R × M^(-1)
result = np.dot(R, M_inv)

print("R × M^(-1) =")
print(result)
print()

# ============ 如果你有具体的旋转矩阵，在下面修改 ============
print("=" * 60)
print("自定义左乘计算:")
print("=" * 60)

# ✅ 在这里输入你的旋转矩阵
R_custom = np.array([[-0.83391043,  0.35472342,  0.42280575],
 [-0.48540053, -0.83596955, -0.25601021],
 [ 0.26263992, -0.41871972,  0.86930666]])

print("\n你的旋转矩阵 R_custom:")
print(R_custom)
print()

# 计算 R_custom × M^(-1)
result_custom = np.dot(R_custom, M_inv)

print("R_custom × M^(-1) =")
print(result_custom)
print()

# ============ 也可以计算 M^(-1) × R ============
result_right = np.dot(M_inv, R_custom)
print("M^(-1) × R_custom =")
print(result_right)
print()

# ============ 格式化输出（方便复制到代码中）============
print("=" * 60)
print("C++ 代码格式（用于复制）:")
print("=" * 60)

def matrix_to_cpp(mat, var_name="M_inv"):
    """将 numpy 矩阵转换为 C++ Eigen 格式"""
    print(f"\nEigen::Matrix3d {var_name};")
    print(f"{var_name} << ")
    for i in range(3):
        row = ", ".join([f"{mat[i,j]:8.5f}" for j in range(3)])
        if i < 2:
            print(f"  {row},")
        else:
            print(f"  {row};")

matrix_to_cpp(M_inv, "M_inv")
matrix_to_cpp(result_custom, "result")

print("\n" + "=" * 60)
print("计算完成！")
print("=" * 60)
