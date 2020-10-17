"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/17
Python:     python3.6

"""
import unittest

import copy
import logging

from typing import List, Dict

import open3d as o3d
import numpy as np

from open3dExp.core import Triangle, Plane, split_triangle_by_plane

logging.basicConfig(level=logging.DEBUG)

logger = logging.getLogger(__name__)


class Open3dExpTestPlane(unittest.TestCase):
    def test_project(self):
        """
        测试垂点
        :return:
        """
        plane = Plane(np.asarray([0, 0, 0]), np.asarray([0, 0, 1]))
        vertice = plane.project(np.asarray([1, 1, 1]))
        self.assertEqual(vertice[2], 0, f"{vertice}")

        self.assertEqual(plane.project(np.asarray([0, 0, 1]))[2], 0, "垂点出现错误")
        self.assertEqual(plane.project(np.asarray([0, 0, 0]))[2], 0, "垂点出现错误")

    def test_intersection(self):
        """
        测试交点
        :return:
        """
        plane = Plane(np.asarray([0, 0, 0]), np.asarray([0, 0, 1]))

        self.assertEqual(plane.intersection(np.asarray([1, 1, 1]), np.asarray([2, 2, 2]))[2], 0, "交点出现错误")  # 同向
        self.assertEqual(plane.intersection(np.asarray([1, 1, -1]), np.asarray([2, 2, 2]))[2], 0, "交点出现错误")  # 点在异侧
        self.assertEqual(plane.intersection(np.asarray([1, 1, 0]), np.asarray([2, 2, 2]))[2], 0, "交点出现错误")  # 其中一点在平面上
        self.assertEqual(plane.intersection(np.asarray([0, 0, 0]), np.asarray([2, 2, 2]))[2], 0, "交点出现错误")  # 其中一点在平面中心


class Open3dExpTestCompare(unittest.TestCase):
    def test_out_triangles(self):
        vertices = o3d.utility.Vector3dVector()
        points = [
            [1, 0, 0],
            [0, 1, 0],
            [-1, 0, 0],
        ]
        for point in points:
            vertices.append(np.asarray(point))

        triangle: Triangle = Triangle(vertices, np.asarray([0, 0, 1]))
        plane = Plane(np.asarray([-0.5, 0, 0]), np.asarray([0, 0, 1]))
        out_angle, in_angle, on_angle = split_triangle_by_plane(triangle,plane)
        logger.debug(f"外部三角形:{out_angle}")
        logger.debug(f"内部三角形:{in_angle}")
        logger.debug(f"重叠三角形:{on_angle}")
        logger.debug(triangle)
        logger.debug(plane)


# pcd = o3d.io.read_point_cloud("Human skeleton.ply")
# o3d.visualization.draw_geometries([pcd])

# mesh:o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_sphere()
# mesh.compute_vertex_normals() # 生成法向量
# triangles:List[Triangle] = []
# for triangle_index,singe_triangle in enumerate(mesh.triangles):
#     # 存放三角形的点
#     vertices:o3d.open3d_pybind.utility.Vector3dVector = o3d.utility.Vector3dVector()
#     for i in range(3):
#         vertices.append(mesh.vertices[singe_triangle[i]])
#
#     # logging.debug(len(mesh.triangle_normals))
#     angle = Triangle(vertices, mesh.triangle_normals[triangle_index])
#     plane = Plane(angle.Vertices[0],angle.Normal)
#     logging.debug(plane.check_in(angle.Vertices[0]))
#     logging.debug(plane.check_in(angle.Vertices[1]))
#     logging.debug(plane.check_in(angle.Vertices[2]))
# triangles.append()
# plane = Plane(np.asarray([0,0,0]),np.asarray([0,0,1]))
# logging.debug(plane.check_in(np.asarray([1,1,0])))
# logging.debug(plane.check_in(np.asarray([1,0,0])))
# logging.debug(len(triangles))
# logging.debug(triangles[0])
# vertices:open3d.open3d_pybind.utility.Vector3dVector = copy.deepcopy(mesh.vertices)
# # logging.debug(len(vertices))
# # logging.debug(vertices[0])
# data = vertices.pop()
# # logging.debug(data)
# point_a = np.array([1,1,1],dtype=np.float64)
#
# triangles:open3d.open3d_pybind.utility.Vector3iVector = mesh.triangles
# logging.debug(triangles[0])
# logging.debug(dir(mesh))
# normals = mesh.triangle_normals
# logging.debug(type(normals))


if __name__ == '__main__':
    unittest.main()
