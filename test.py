"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/12
Python:     python3.6

"""
import logging
from types import FunctionType, MethodType
from inspect import isgeneratorfunction
import numpy as np

from ImSlow.gemoetry import XYZ, Triangle, Plane,MeshGeom

logging.basicConfig(level=logging.DEBUG)


a = XYZ(1, 1, 0)
b = XYZ(1, 2, 0)
c = XYZ(0, 0, 0)
vertexs = {
    a:0,
    b:1,
    c:2
}
angles = [
    [0,1,2]
]
normals = [
    XYZ(0,0,1)
]
mesh_geom = MeshGeom(vertexs,angles,normals)

logging.debug(f"顶点数据:{mesh_geom.Vertexs}")
logging.debug(f"三角形数据:{mesh_geom.Triangles}")
logging.debug(f"法向量数据:{mesh_geom.Normals}")
