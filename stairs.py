"""
Author:     LanHao
Date:       2020/11/4
Python:     python3.6

"""

import logging
from datetime import datetime
import numpy as np
import open3d as o3d

from open3dExp.utils import BooleanOperation,BooleanOperationUtils
from open3dExp.core import BSPTree,to_triangle_mesh,Triangle

logging.basicConfig(level=logging.DEBUG,format="%(asctime)s:%(message)s ")

mesh = o3d.geometry.TriangleMesh.create_cylinder(1,5) # 圆柱
mesh.compute_vertex_normals()

# tree = BSPTree.create_with_triangle_mesh(mesh)
# task_node = [tree.head]
#
# while task_node:
#     node = task_node.pop()
#     if node.plane.Normal[2] == -1:
#         logging.debug(f"node plane:{node.plane}")
#     if node.out_node:
#         task_node.append(node.out_node)
#     if node.in_node:
#         task_node.append(node.in_node)


mesh2 = o3d.geometry.TriangleMesh.create_box(5,5,5).translate((-2,-2,2.5)) # 立方体
mesh2.compute_vertex_normals()

mesh_new = BooleanOperationUtils.execute_boolean_operation(mesh2, mesh, BooleanOperation.Difference)
mesh_new = mesh_new.sample_points_uniformly(number_of_points=1500000)
o3d.visualization.draw_geometries([mesh_new])
# logging.debug(np.asarray(mesh.triangle_normals))