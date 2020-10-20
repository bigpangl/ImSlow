"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/17
Python:     python3.6

"""
import logging
from itertools import repeat
from collections import Iterable
from datetime import datetime

import copy
from typing import List, Dict, Tuple
from enum import Enum

import open3d as o3d
import numpy as np

from .core import BooleanOperation, Triangle, Plane, BSPNode, split_triangle_by_plane, to_triangle_mesh, BSPTree, \
    make_normals,split_triangle_mesh

logger = logging.getLogger(__name__)

STATIC_ERROR = 1e-10


# @log_use_time
# def split_triangle_mesh(mesh: o3d.open3d_pybind.geometry.TriangleMesh, tree: BSPTree,
#                         error: float = None) -> \
#         Tuple[List[List[Triangle]], List[List[Triangle]], List[List[Triangle]], List[List[Triangle]]]:
#     """
#     使用bsp 树分割原有的triangle_mesh,得到out_triangle,in_triangle,on_triangle
#     :param mesh:
#     :param tree:
#     :param error:
#     :return:
#     """
#     if error is None:
#         error = STATIC_ERROR
#
#     out_triangles: List[List[Triangle]] = []
#     in_triangles: List[List[Triangle]] = []
#     on_same_triangles: List[List[Triangle]] = []
#     on_diff_triangles: List[List[Triangle]] = []
#
#     for angle_index, mesh_angle in enumerate(mesh.triangles):
#         vertices = o3d.utility.Vector3dVector()
#         for i in range(3):
#             vertices.append(mesh.vertices[mesh_angle[i]])
#
#         triangle_mid = Triangle(vertices, mesh.triangle_normals[angle_index])
#
#         task_queue: List[Tuple[Triangle, BSPNode, int]] = [(triangle_mid, tree.head, 0)]
#         while task_queue:
#             triangle_use, node_use, surface_status_use = task_queue.pop()  # 此前计算的相关信息
#
#             out_mid, in_mid, on_same_mid, on_diff_mid = split_triangle_by_plane(triangle_use, node_use.plane, error)
#             if node_use.out_node is None:
#                 out_triangles.append(out_mid)  # 通过plane 切割,确定在几何体外部
#             else:
#                 # 可能在内部,也可能在内部,要继续讨论
#                 task_queue.extend(zip(
#                     out_mid,
#                     repeat(node_use.out_node),
#                     repeat(False)
#                 ))
#
#             if node_use.in_node is None:
#
#                 if surface_status_use == 0:  # 剔除曾经在表面然后现在又在内部的三角面
#                     in_triangles.append(in_mid)
#                 elif surface_status_use == 1:  # 同向
#                     on_same_triangles.append(in_mid)
#                 else:  # 异向
#                     on_diff_triangles.append(in_mid)
#
#                 on_same_mid and on_same_triangles.append(on_same_mid)
#                 on_diff_mid and on_diff_triangles.append(on_diff_mid)
#             else:
#                 task_queue.extend(zip(
#                     in_mid,
#                     repeat(node_use.in_node),
#                     repeat(surface_status_use)
#                 ))
#                 task_queue.extend(zip(
#                     on_same_mid,
#                     repeat(node_use.in_node),
#                     repeat((1))
#                 ))
#                 task_queue.extend(zip(
#                     on_diff_mid,
#                     repeat(node_use.in_node),
#                     repeat(2)
#                 ))
#
#     return out_triangles, in_triangles, on_same_triangles, on_diff_triangles


# class BooleanOperationUtils:
#     """
#
#     几何体布尔运算
#
#     """
#
#     @classmethod
#     def execute_boolean_operation(cls, geom1: o3d.open3d_pybind.geometry.TriangleMesh,
#                                   geom2: o3d.open3d_pybind.geometry.TriangleMesh,
#                                   operation: BooleanOperation,
#                                   error: float = None) -> o3d.open3d_pybind.geometry.TriangleMesh:
#
#         if error is None:
#             error = STATIC_ERROR
#
#         make_normals(geom1)
#         make_normals(geom2)
#
#         # BSP 树存储数据,广度搜索
#         geom2_tree: BSPTree = BSPTree.create_from_triangle_mesh(geom2)
#         start_time = datetime.now()
#         geom1_tree: BSPTree = BSPTree.create_from_triangle_mesh(geom1)
#         end_time = datetime.now()
#         logger.debug(f"球体加载耗时:{end_time - start_time}")
#         triangles_all: List[List[List[Triangle]]] = []  # 存放所有的三角面片,多层嵌套列表Triangle
#
#         out_triangles, in_triangles, on_same_triangles, on_diff_triangles = split_triangle_mesh(geom1, geom2_tree)
#
#         out_triangles2, in_triangles2, on_same_triangles2, on_diff_triangles2 = split_triangle_mesh(geom2, geom1_tree)
#
#         if operation == BooleanOperation.Union:  # 并集
#             triangles_all.append(out_triangles)
#             triangles_all.append(out_triangles2)
#             triangles_all.append(on_same_triangles)
#
#         if operation == BooleanOperation.Intersect:  # 相交部分
#             triangles_all.append(in_triangles)
#             triangles_all.append(in_triangles2)
#             if len(in_triangles) != 0 and len(in_triangles2) != 0:  # 如果等于0，就未相交了
#                 triangles_all.append(on_same_triangles)
#
#         if operation == BooleanOperation.Difference:  # 不同部分
#
#             triangles_all.append(out_triangles)
#
#             triangle_in_2: List[List[Triangle]] = []
#
#             for single_triangle in in_triangles2:
#                 single_triangle: List[Triangle]
#                 new_single_triangle: List[Triangle] = []
#                 for single in single_triangle:
#                     vertices = o3d.utility.Vector3dVector()
#                     vertices.append(single.Vertices[2])
#                     vertices.append(single.Vertices[1])
#                     vertices.append(single.Vertices[0])
#                     new_single_triangle.append(Triangle(vertices, single.Normal * -1))
#                 triangle_in_2.append(new_single_triangle)
#
#             triangles_all.append(triangle_in_2)
#             triangles_all.append(on_diff_triangles)
#
#         iteral_use = (angle for list_angle1 in triangles_all for list_angle2 in list_angle1 for angle in list_angle2)
#         mesh = to_triangle_mesh(iteral_use)
#         return mesh
