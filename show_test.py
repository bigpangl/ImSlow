"""
Author: LanHao
Date:2020/10/16
Python: python 3.6
"""
import logging
from datetime import datetime
import numpy as np
import open3d as o3d

from CSG.core import Triangle, Plane, Node, CSG

logging.basicConfig(level=logging.DEBUG)


#
# from open3dExp.utils import BooleanOperation,BooleanOperationUtils
# # from open3dExp.boolean import BooleanOperationUtils,BooleanOperation
#
# logging.basicConfig(level=logging.DEBUG,format="%(asctime)s:%(message)s ")
#
#
# mesh: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_sphere(10)
# mesh.compute_vertex_normals()
# # mesh: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_box(10,10,10)
# #
# mesh2: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_box(5,5,5).translate((8, 0, 0))
# mesh2.compute_vertex_normals()
# # mesh.compute_vertex_normals()
# start_time = datetime.now()
# mesh_new = BooleanOperationUtils.execute_boolean_operation(mesh, mesh2, BooleanOperation.Difference)
# end_time = datetime.now()
# logging.debug(f"累计执行时间:{end_time-start_time}")
# o3d.io.write_triangle_mesh("mesh.ply",mesh_new)
# mesh_new = mesh_new.sample_points_uniformly(number_of_points=1500000)
# # mesh_new = o3d.io.read_triangle_mesh("mesh.ply")
# logging.debug("布尔操作完毕")
# o3d.visualization.draw_geometries([mesh_new])


def to_triangle_mesh(iteral):
    """
    将任何可迭代对象(迭代出来的为一个Triangle)转换为open3d 中的TriangleMesh 对象,期间会进行重复点的剔除工作

    这个过程,从目前的效果来看,本身耗时并不多

    :param iteral:
    :return:
    """

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
        # mesh.triangle_normals.append(triangle.Normal)

    return mesh


def get_triangles_with_mesh(mesh):
    triangles = []  # 初始化,用于存放三角形

    for triangle_index in range(len(mesh.triangles)):  # 遍历三角形
        singe_triangle = mesh.triangles[triangle_index]
        # 存放三角形的点
        vertices = np.zeros(shape=(0, 3), dtype=np.float64)
        for i in range(3):
            vertices = np.append(vertices, [mesh.vertices[singe_triangle[i]]], axis=0)
        triangle = Triangle(vertices)

        triangles.append(triangle)  # 会总处理

    return triangles


mesh2: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_cylinder(1,5)

mesh = o3d.geometry.TriangleMesh.create_box(5, 5, 5).translate((-2, -2, -2.5))

start = datetime.now()
triangles = get_triangles_with_mesh(mesh)
triangles2 = get_triangles_with_mesh(mesh2)
csg_a = CSG.from_trianagles(triangles)
csg_b = CSG.from_trianagles(triangles2)
csg_c = csg_a.to_subtract(csg_b)
triangles3 = csg_c.to_triangles()
mesh = to_triangle_mesh(triangles3)
end = datetime.now()
mesh.compute_convex_hull()
mesh = mesh.sample_points_uniformly(number_of_points=150000)
logging.debug(f"总耗时:{end - start}")
o3d.visualization.draw_geometries([mesh])
