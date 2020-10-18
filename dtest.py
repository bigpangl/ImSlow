"""
Author: LanHao
Date:2020/10/16
Python: python 3.6
"""
import logging

import numpy as np
import open3d as o3d
from open3dExp.core import BSPTree,BooleanOperationUtils,BooleanOperation

logging.basicConfig(level=logging.DEBUG)


mesh: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_box(10,10,10)

mesh2: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_box(5,5,5).translate((10, 0, 0))
# mesh.compute_vertex_normals()

mesh_new = BooleanOperationUtils.execute_boolean_operation(mesh,mesh2,BooleanOperation.Difference)
logging.debug("布尔操作完毕")
o3d.visualization.draw_geometries([mesh_new])

