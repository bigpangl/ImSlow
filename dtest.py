"""
Author: LanHao
Date:2020/10/16
Python: python 3.6
"""
import logging

import numpy as np
import open3d as o3d
from open3dExp.core import BSPTree

logging.basicConfig(level=logging.DEBUG)


mesh: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_sphere()
tree = BSPTree.create_from_triangle_mesh(mesh)

# mesh_box = o3d.geometry.TriangleMesh.create_box(width=20,
#                                                 height=1.0,
#                                                 depth=1.0)
# print(mesh_box.PointCloud)
#
# mesh_box.compute_vertex_normals()
# mesh_box.paint_uniform_color([0.9, 0.1, 0.1])
# mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
# mesh_sphere.compute_vertex_normals()
# mesh_sphere.paint_uniform_color([0.1, 0.1, 0.7])
# mesh_cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.3,
#                                                           height=4.0)
# mesh_cylinder.compute_vertex_normals()
# mesh_cylinder.paint_uniform_color([0.1, 0.9, 0.1])
# mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
#     size=0.6, origin=[-2, -2, -2])
#
# mesh_all = mesh_box
# vertexs = np.asarray(mesh_all.vertices)
# for ver in np.asarray(mesh_all.triangles):
#     print(vertexs[ver[0]])
#     print(vertexs[ver[1]])
#     print(vertexs[ver[2]])
#     print("===")

# o3d.visualization.draw_geometries([mesh_all])

