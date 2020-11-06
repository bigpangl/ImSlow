"""
Author: LanHao
Date:2020/10/16
Python: python 3.6
"""
import logging
from datetime import datetime
import numpy as np
import open3d as o3d

from CSG.core import Triangle, Plane, Node
from CSG.csg import CSG
from CSG.utils import Open3dTranslate

logging.basicConfig(level=logging.DEBUG)

mesh = o3d.geometry.TriangleMesh.create_cylinder(1,5)
# mesh = o3d.geometry.TriangleMesh.create_sphere(10)

mesh2 = o3d.geometry.TriangleMesh.create_box(5, 5, 5).translate((-2, -2, -2.5))

start = datetime.now()
triangles = Open3dTranslate.to_triangles(mesh)
triangles2 = Open3dTranslate.to_triangles(mesh2)
csg_a = CSG.from_trianagles(triangles)
csg_b = CSG.from_trianagles(triangles2)
start_bool = datetime.now()
csg_c = csg_b.to_subtract(csg_a)
# csg_c = csg_a.to_union(csg_b)

end_bool = datetime.now()
logging.debug(f"布尔运算耗时:{end_bool - start_bool}(BSP 树构建和布尔运算汇总时间)")
start_to_triangles = datetime.now()
triangles3 = csg_c.to_triangles()
end_to_triangles = datetime.now()
logging.debug(f"取出三角形耗时:{end_to_triangles - start_to_triangles}")
triangles_use = triangles3
mesh = Open3dTranslate.to_mesh(triangles_use)
end = datetime.now()
# mesh.compute_convex_hull()
mesh = mesh.sample_points_uniformly(number_of_points=150000)
logging.debug(f"总耗时:{end - start}")
o3d.visualization.draw_geometries([mesh])

# o3d.visualization.draw_geometries([to_triangle_mesh(csg_a.to_triangles())])
