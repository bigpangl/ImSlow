"""

Project:    ImSlow
Author:     LanHao
Date:       2020/11/2
Python:     python3.6

"""
from enum import Enum
import logging
from itertools import repeat
from datetime import datetime

import open3d as o3d
import numpy as np

from .core import Triangle, BSPTree, to_triangle_mesh


class BooleanOperation(Enum):
    Union = 1  # 合并
    Difference = 2  # 不同
    Intersect = 3  # 相交


class BooleanOperationUtils:
    """

    几何体布尔运算

    """

    @classmethod
    def execute_boolean_operation(cls, geom1: o3d.open3d_pybind.geometry.TriangleMesh,
                                  geom2: o3d.open3d_pybind.geometry.TriangleMesh,
                                  operation: BooleanOperation,
                                  ):
        # BSP 树存储数据,广度搜索
        geom2_tree = BSPTree.create_with_triangle_mesh(geom2) # 柱
        geom1_tree = BSPTree.create_with_triangle_mesh(geom1) # 立方体

        triangles_all = []  # 存放所有的三角面片,多层嵌套列表Triangle

        out_triangles, in_triangles, on_same_triangles, on_diff_triangles = geom2_tree.split_triangle_mesh(geom1)
        logging.debug(f"结束圆柱分割立方体")
        out_triangles2, in_triangles2, on_same_triangles2, on_diff_triangles2 = geom1_tree.split_triangle_mesh(geom2)
        logging.debug(f"结束立方体分割圆柱")
        if operation == BooleanOperation.Union:  # 并集
            triangles_all.append(out_triangles)
            triangles_all.append(out_triangles2)
            triangles_all.append(on_same_triangles)
        if operation == BooleanOperation.Intersect:  # 相交部分
            triangles_all.append(in_triangles)
            triangles_all.append(in_triangles2)
            if len(in_triangles) != 0 and len(in_triangles2) != 0:  # 如果等于0，就未相交了
                triangles_all.append(on_same_triangles)

        if operation == BooleanOperation.Difference:  # 不同部分

            triangles_all.append(out_triangles)

            triangle_in_2 = []
            for single_triangle in in_triangles2:
                new_single_triangle = []
                for single in single_triangle:
                    vertices = np.asarray([
                        single.Vertices[2],
                        single.Vertices[1],
                        single.Vertices[0]
                    ], dtype=np.float64)
                    new_single_triangle.append(Triangle(vertices, single.Normal * -1))
                triangle_in_2.append(new_single_triangle)
            triangles_all.append(triangle_in_2)
            triangles_all.append(on_diff_triangles)
            if len(on_same_triangles)>0:
                logging.debug(f"out:{len(out_triangles)},in:{len(in_triangles)},"
                              f"on_same:{len(on_same_triangles)},on_diff:{len(on_diff_triangles)}")
                task_node = [geom2_tree.head]
                while task_node:
                    node = task_node.pop()
                    if node.plane.Normal[2] == -1:
                        logging.debug(f"node plane:{node.plane},triangles:{len(node.triangles)},out:{node.out_node},in:{node.in_node}")
                    if node.out_node:
                        task_node.append(node.out_node)
                    if node.in_node:
                        task_node.append(node.in_node)
            # 待删除
            # assert len(on_same_triangles)==0,Exception("")
            # triangles_all.append(on_diff_triangles)

        iteral_use = (angle for list_angle1 in triangles_all for list_angle2 in list_angle1 for angle in list_angle2)
        mesh = to_triangle_mesh(iteral_use)
        return mesh
