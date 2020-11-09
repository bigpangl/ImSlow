# cython: language_level=3

"""
Author:     LanHao
Date:       2020/11/5
Python:     python3.6

用于适配一些其他工具包重的转换功能

"""
import math
import logging
import numpy as np
cimport numpy as np
import open3d as o3d

from .core cimport Triangle, Polygon, get_cos_by

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

# 实现基于简单多边形的几何体拉伸
cpdef stretching_mesh(np.ndarray vertices, np.ndarray direction):
    v_one = vertices
    v_use = vertices

    polygon = Polygon(v_one)
    cos_value = get_cos_by(polygon.normal, direction)
    if cos_value > 0:
        # 同向需要转向
        polygon.flip()
        v_one = np.flipud(v_one)  # 底面朝向应该和拉伸方向相反
    else:
        v_use = np.flipud(v_use)
    triangles = polygon.to_triangles()
    points2 = v_use + direction
    points_2_use = v_one + direction  # 记录朝向相关问题

    polygon2 = Polygon(points2)
    cos_value = get_cos_by(polygon2.normal, direction)

    triangles2 = polygon2.to_triangles()

    triangles.extend(triangles2)
    number_length = len(v_one)
    for i in range(number_length - 1):
        point_i = v_one[i]
        point_i_up = points_2_use[i]

        point_i2 = v_one[(i + 1) % number_length]
        point_i2_up = points_2_use[(i + 1) % number_length]
        triangles.append(Triangle(np.asarray([point_i, point_i_up, point_i2_up])))
        triangles.append(Triangle(np.asarray([point_i2_up, point_i2, point_i])))
    triangles.append(Triangle(np.asarray([v_one[-1], points_2_use[-1], points_2_use[0]])))
    triangles.append(Triangle(np.asarray([points_2_use[0], v_one[0], v_one[-1]])))

    mesh = Open3dTranslate.to_mesh(triangles)
    return mesh
