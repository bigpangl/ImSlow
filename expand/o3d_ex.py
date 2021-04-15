"""
Author:     LanHao
Date:       2021/4/15 13:49
Python:     python3.6

"""

import math
import numpy as np

import open3d as o3d

from . import BASE_X, BASE_Y, BASE_Z
from .np_ex import rotation_3d


def create_cylinder(r: float, center: np.ndarray, normal: np.ndarray, split_n: int = 10, loss: float = 1e-10):
    """
    :param r: 半径
    :param center: 中心点
    :param normal: 拉伸方向和长度
    :param split_n: 底面圆应该被分割的段数量
    :param loss: 判断是否平行时的精度损失
    :return: open3d. TriangleMesh 对象
    """

    normal_length = np.linalg.norm(normal)
    v_normal: np.ndarray = normal / normal_length

    vector_not_parallel = None

    for base_vector in [BASE_X, BASE_Y, BASE_Z]:
        cos_base = v_normal.dot(base_vector)
        if 1 - abs(cos_base) > loss:  # 判断向量非平行
            vector_not_parallel = base_vector
            break

    assert vector_not_parallel is not None, Exception("未能成功得到平面外的另个向量")
    v_on_plane_u = np.cross(v_normal, vector_not_parallel)
    v_on_plane_u /= np.linalg.norm(v_on_plane_u)  # base u 单位向量化

    v_on_plane_v = np.cross(v_on_plane_u, v_normal)  # 满足右手定则的base u 和base v
    v_on_plane_v /= np.linalg.norm(v_on_plane_v)  # base v 单位向量化

    step_radian = math.pi * 2 / split_n
    up_center = center + normal  # 拉伸后停止位置的中心点
    points_total = []
    points = []  # 存放圆外轮廓上所有的点,依着传入的normal 为法向量,拟时针排序,需要收尾相连
    points_up = []
    base_r_v = v_on_plane_u * r

    for i in range(split_n):
        v_i = rotation_3d(base_r_v, normal, step_radian * i)
        point_i = center + v_i
        point_up_i = up_center + v_i
        points.append(point_i)
        points_up.append(point_up_i)

    # 需要传入o3d 中Triangle Mesh 的points 对象
    points_total.extend(points)
    points_total.extend(points_up)
    points_total.append(center)
    points_total.append(up_center)
    triangle_mesh: o3d.geometry.TriangleMesh = o3d.geometry.TriangleMesh()
    triangle_mesh.vertices = o3d.utility.Vector3dVector(points_total)

    length_total_points = len(points_total)
    points_len = len(points)
    for i in range(len(points)):
        # 底部三角形
        triangle_mesh.triangles.append([length_total_points - 2, i % points_len, (i - 1) % points_len])
        # 顶部三角形
        triangle_mesh.triangles.append(
            [length_total_points - 1, (i % points_len) + points_len, (i + 1) % points_len + points_len])
        # 侧边三角形
        triangle_mesh.triangles.append([i % points_len, (i + 1) % points_len, (i + 1) % points_len + points_len])
        triangle_mesh.triangles.append([i % points_len, (i + 1) % points_len + points_len, i % points_len + points_len])
    return triangle_mesh
