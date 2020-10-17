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

mesh_new = BooleanOperationUtils.execute_boolean_operation(mesh,mesh2,BooleanOperation.Intersect)

logging.debug(np.asarray(mesh_new.triangles))
o3d.visualization.draw_geometries([mesh_new])

