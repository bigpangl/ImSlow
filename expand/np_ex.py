"""
Author:     LanHao
Date:       2021/4/15 13:49
Python:     python3.6

"""

import numpy as np


def rotation_3d(position: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """
    工具函数,用于依仗特定轴旋转特定向量特定角度
    position:原坐标(x, y, z)
    axis:旋转的坐标轴(ex, ey, ez)
    angle: 旋转弧度
    """

    ex, ey, ez = axis
    ex, ey, ez = [x / np.sqrt(ex ** 2 + ey ** 2 + ez ** 2)  # 归一化
                  for x in axis]
    s, c = np.sin(angle), np.cos(angle),
    matrix1 = np.array([[ex ** 2, ex * ey, ex * ez],
                        [ey * ex, ey ** 2, ey * ez],
                        [ex * ez, ey * ez, ez ** 2]])
    matrix2 = np.array([[c, -ez * s, ey * s],
                        [ez * s, c, -ex * s],
                        [-ey * s, ex * s, c]])
    matrix = (1 - c) * matrix1 + matrix2
    return matrix.dot(np.array(position).reshape(3, 1)).reshape(1, 3)[0]
