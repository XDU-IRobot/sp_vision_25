#!/usr/bin/env python3
"""
旋转矩阵计算脚本
计算给定矩阵的逆矩阵，并支持左乘操作
"""

import numpy as np

# 定义旋转矩阵R_camera2gimbal(3*3)
R_camera2gimbal = np.array([[0.00046423063676173171, 0.033770528223625203, 0.99942950522456209],
                             [-0.99970300483287733, 0.024367462669785803, -0.00035901413901253575],
                             [-0.02436568525675643, -0.99913251282626891, 0.03377181066266309]])
# 相机自身旋转矩阵
R_camera = np.array([[-1, 0, 0],
                     [0, -1, 0],
                     [0, 0, 1]])
# 纠正后的矩阵
R_correct = R_camera2gimbal @ R_camera
print("R_camera2gimbal_correct:")
print(R_correct)