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
from open3dExp.core import create_bsp_tree_with_triangle_mesh,to_triangle_mesh

logging.basicConfig(level=logging.DEBUG,format="%(asctime)s:%(message)s ")


mesh = o3d.geometry.TriangleMesh.create_cylinder(1,5)
mesh.compute_vertex_normals()

mesh2 = o3d.geometry.TriangleMesh.create_box(5,5,5).translate((-2,-2,-2.5))
mesh2.compute_vertex_normals()

# tree_qiu = create_bsp_tree_with_triangle_mesh(mesh)
# mesh_handle = to_triangle_mesh(tree_qiu.traverse())
mesh_new = BooleanOperationUtils.execute_boolean_operation(mesh2, mesh, BooleanOperation.Difference)
mesh_new = mesh_new.sample_points_uniformly(number_of_points=1500000)
# mesh_new = mesh_handle.sample_points_uniformly(number_of_points=1500000)
o3d.visualization.draw_geometries([mesh_new])