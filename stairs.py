"""
Author:     LanHao
Date:       2020/11/4
Python:     python3.6

"""

import logging
import unittest
import numpy as np
from datetime import datetime
import open3d as o3d

from CSG.core import Triangle, Plane, Node, get_cos_by, Polygon
from CSG.csg import CSG
from CSG.linked import SingleLinkedNode
from CSG.utils import Open3dTranslate, stretching_mesh

logging.basicConfig(level=logging.DEBUG,format="%(asctime)s:%(message)s ")

meshs = []
data = {"top_ear": {"stretch": {"x": 1, "y": 0, "z": 0}, "length": 1310.0,
                    "vertexs": [{"x": 0, "y": 2780, "z": 1590}, {"x": 0, "y": 2300, "z": 1590},
                                {"x": 0, "y": 2300, "z": 1422}, {"x": 0, "y": 2780, "z": 1422}]},
        "foot_ear": {"stretch": {"x": 1, "y": 0, "z": 0}, "length": 1310.0,
                     "vertexs": [{"x": 0, "y": 480, "z": 257}, {"x": 0, "y": 0, "z": 257},
                                 {"x": 0, "y": 0, "z": 0}, {"x": 0, "y": 480, "z": 0}]},
        "body": {"stretch": {"x": 1, "y": 0, "z": 0}, "length": 1270.0,
                 "vertexs": [{"x": 0, "y": 0, "z": 0}, {"x": 0, "y": 0, "z": 257}, {"x": 0, "y": 480, "z": 257},
                             {"x": 0, "y": 480, "z": 424}, {"x": 0, "y": 740, "z": 424},
                             {"x": 0, "y": 740, "z": 591}, {"x": 0, "y": 1000, "z": 591},
                             {"x": 0, "y": 1000, "z": 757}, {"x": 0, "y": 1260, "z": 757},
                             {"x": 0, "y": 1260, "z": 924}, {"x": 0, "y": 1520, "z": 924},
                             {"x": 0, "y": 1520, "z": 1091}, {"x": 0, "y": 1780, "z": 1091},
                             {"x": 0, "y": 1780, "z": 1257}, {"x": 0, "y": 2040, "z": 1257},
                             {"x": 0, "y": 2040, "z": 1424}, {"x": 0, "y": 2300, "z": 1424},
                             {"x": 0, "y": 2300, "z": 1590}, {"x": 0, "y": 2780, "z": 1590},
                             {"x": 0, "y": 2780, "z": 1422}, {"x": 0, "y": 2520, "z": 1422},
                             {"x": 0, "y": 300, "z": 0}]}}

for key, value in data.items():
    value: dict
    points = []
    for vertex in value["vertexs"]:
        points.append([vertex["x"], vertex["y"], vertex["z"]])
    points = np.asarray(points)
    stretch = value["stretch"]
    length = value["length"]
    direction = np.asarray([stretch["x"], stretch["y"], stretch["z"]]) * length
    mesh = stretching_mesh(points, direction)
    meshs.append(mesh)
    # break
logging.debug(f"主体部分绘制成功")
# 合并
csg = None
for mesh in meshs:
    csg_mid = CSG.from_trianagles(Open3dTranslate.to_triangles(mesh))
    if csg is None:
        csg = csg_mid
    else:
        csg = csg.to_union(csg_mid)

add_length = 20
open_holes = [{"x": 300.0, "y": 100.0, "z": 257, "r": 30.0, "d": {"x": 0, "y": 0, "z": -1},
               "length": 257+add_length},
              {"x": 970.0, "y": 100.0, "z": 257.88366668686183, "r": 30, "d": {"x": 0, "y": 0, "z": -1},
               "length": 257+add_length},
              {"x": 300.0, "y": 2680.0, "z": 1590.883666686862, "r": 30, "d": {"x": 0, "y": 0, "z": -1},
               "length": 168+add_length},
              {"x": 970.0, "y": 2680.0, "z": 1590.883666686862, "r": 30, "d": {"x": 0, "y": 0, "z": -1},
               "length": 168+add_length}]
# mesh = Open3dTranslate.to_mesh(csg.to_triangles())
# mesh = mesh.sample_points_uniformly(number_of_points=300000)
#
# o3d.visualization.draw_geometries([mesh])

holes = []

for hole in open_holes:
    hole_mesh = o3d.geometry.TriangleMesh.create_cylinder(hole["r"],hole["length"])
    hole_mesh.translate((hole["x"],hole["y"],hole["z"]-hole["length"]/2))
    # holes.append(hole_mesh)
    #
    csg_mid = CSG.from_trianagles(Open3dTranslate.to_triangles(hole_mesh))
    logging.debug(f"open hole 开口初始化CSG 完成")
    if csg is None:
        csg = csg_mid
    else:
        csg = csg.to_subtract(csg_mid)
    logging.debug(f"完成一个开口")
mesh = Open3dTranslate.to_mesh(csg.to_triangles())
mesh = mesh.sample_points_uniformly(number_of_points=900000)

holes.append(mesh)
o3d.visualization.draw_geometries(holes)