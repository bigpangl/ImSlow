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

from EarClipping.core import clipping, green_value, Plane as Clipplane

from .core cimport Triangle, get_cos_by

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
cpdef stretching_mesh(np.ndarray vertices, np.ndarray direction, clip_plane):
    triangles = []
    plane_normal_cos = get_cos_by(clip_plane.normal, direction)
    if plane_normal_cos == 0:
        raise Exception("不可斜着拉伸")
    elif plane_normal_cos < 0:
        # 使得得到的平面的法向量和拉伸方向在一个夹角小于90°的范围内
        clip_plane = Clipplane(clip_plane.v_bais.copy(), clip_plane.u_bais.copy(), clip_plane.origin.copy())

    point_np_all = []
    for point in vertices:
        uv = clip_plane.uv(point)
        point_np_all.append(uv)

    value_check = green_value(point_np_all)
    if value_check < 0:
        point_np_all = np.flipud(point_np_all)  # 如果是顺时针,就转向所有的点

    data = clipping(point_np_all)

    for triangle in data:
        v1 = clip_plane.to_xyz(triangle[0])
        v2 = clip_plane.to_xyz(triangle[1])
        v3 = clip_plane.to_xyz(triangle[2])
        triangle_mid = Triangle(np.asarray([v1, v2, v3]))
        triangle_2 = triangle_mid.clone()
        triangle_2.Vertices += direction
        triangle_mid.flip()  # 强制逆时针后,必须转向
        triangles.append(triangle_mid)
        triangles.append(triangle_2)  #

    number_length = len(point_np_all)
    for i in range(number_length):
        point_i = clip_plane.to_xyz(point_np_all[i%number_length])
        point_i_up = point_i + direction
        point_i2 = clip_plane.to_xyz(point_np_all[(i + 1) % number_length])
        point_i2_up = point_i2 + direction
        # 顺时针的方向
        triangle_1 = Triangle(np.asarray([point_i, point_i2_up,point_i_up]))
        triangle_2 = Triangle(np.asarray([point_i2_up, point_i,point_i2]))
        triangles.append(triangle_1)
        triangles.append(triangle_2)
    mesh = Open3dTranslate.to_mesh(triangles)
    return mesh
