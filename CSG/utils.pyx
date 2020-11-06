# cython: language_level=3

"""
Author:     LanHao
Date:       2020/11/5
Python:     python3.6

用于适配一些其他工具包重的转换功能

"""
import numpy as np
cimport numpy as np
import open3d as o3d

from .core cimport Triangle


cdef class Open3dTranslate:

    @staticmethod
    def to_mesh(iteral):
        """
        这是一个容易变得费时的操作,如何优化此处的时间呢？
        :param iteral:
        :return:
        """
        cdef:
            list triangle_index
            int i, index

        mesh = o3d.geometry.TriangleMesh()

        for triangle in iteral:
            triangle_index = []
            for i in range(3):
                select = np.where((mesh.vertices == triangle.Vertices[i]).all(1))[0]
                if len(select) > 0:
                    index = select[0]
                else:
                    mesh.vertices.append(triangle.Vertices[i])
                    index = len(mesh.vertices) - 1

                triangle_index.append(index)
            triangle_index_np = np.asarray(triangle_index, dtype=np.int32)
            mesh.triangles.append(triangle_index_np)
        return mesh


    @staticmethod
    def to_triangles(mesh):
        cdef:
            list triangles = []
            int triangle_index
            np.ndarray vertices
            Triangle triangle

        for triangle_index in range(len(mesh.triangles)):  # 遍历三角形
            singe_triangle = mesh.triangles[triangle_index]
            # 存放三角形的点
            vertices = np.zeros(shape=(0, 3), dtype=np.float64)
            for i in range(3):
                vertices = np.append(vertices, [mesh.vertices[singe_triangle[i]]], axis=0)
            triangle = Triangle(vertices)
            triangles.append(triangle)

        return triangles
