"""
Author: LanHao
Date:2020/10/16
Python: python 3.6
"""
import logging
from datetime import datetime
import numpy as np
import open3d as o3d

from open3dExp.core import BooleanOperation,BooleanOperationUtils
# from open3dExp.boolean import BooleanOperationUtils,BooleanOperation

logging.basicConfig(level=logging.DEBUG,format="%(asctime)s:%(message)s ")


mesh: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_sphere(10)
mesh.compute_vertex_normals()
# mesh: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_box(10,10,10)
#
mesh2: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_box(5,5,5).translate((8, 0, 0))
mesh2.compute_vertex_normals()
# mesh.compute_vertex_normals()
start_time = datetime.now()
mesh_new = BooleanOperationUtils.execute_boolean_operation(mesh, mesh2, BooleanOperation.Difference)
end_time = datetime.now()
logging.debug(f"累计执行时间:{end_time-start_time}")
o3d.io.write_triangle_mesh("mesh.ply",mesh_new)
mesh_new = mesh_new.sample_points_uniformly(number_of_points=1500000)
# mesh_new = o3d.io.read_triangle_mesh("mesh.ply")
logging.debug("布尔操作完毕")
o3d.visualization.draw_geometries([mesh_new])


