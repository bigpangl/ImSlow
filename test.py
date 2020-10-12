"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/12
Python:     python3.6

"""
import logging
from typing import List
from types import FunctionType, MethodType
from inspect import isgeneratorfunction
import numpy as np

from ImSlow.gemoetry import XYZ, Triangle, Plane, MeshGeom
from ImSlow.tree import BSPTree

logging.basicConfig(level=logging.DEBUG)

a = XYZ(0, 0, 0)
b = XYZ(0, 10, 0)
c = XYZ(10, 10, 0)
d = XYZ(10, 0, 0)

a2 = XYZ(0, 0, 10)
b2 = XYZ(0, 10, 10)
c2 = XYZ(10, 10, 10)
d2 = XYZ(10, 0, 10)

a3 = XYZ(20, 0, 0)
b3 = XYZ(20, 10, 0)
c3 = XYZ(30, 10, 0)
d3 = XYZ(30, 0, 0)

a4 = XYZ(20, 0, 10)
b4 = XYZ(20, 10, 10)
c4 = XYZ(30, 10, 10)
d4 = XYZ(30, 0, 10)

vertexs = {
    a: 0,
    b: 1,
    c: 2,
    d: 3,
    a2: 4,
    b2: 5,
    c2: 6,
    d2: 7,

    a3: 8,
    b3: 9,
    c3: 10,
    d3: 11,

    a4: 12,
    b4: 13,
    c4: 14,
    d4: 15
}

angles = [
    [0, 1, 2],
    [0, 2, 3],

    [4, 5, 6],
    [4, 6, 7],

    [1, 2, 6],
    [1, 6, 5],

    [2, 6, 7],
    [2, 7, 3],

    [0, 1, 5],
    [0, 5, 4],

    [0, 3, 7],
    [0, 7, 4],

#

    [8, 9, 10],
    [0+8, 2+8, 3+8],

    [4+8, 5+8, 6+8],
    [4+8, 6+8, 7+8],

    [1+8, 2+8, 6+8],
    [1+8, 6+8, 5+8],

    [2+8, 6+8, 7+8],
    [2+8, 7+8, 3+8],

    [0+8, 1+8, 5+8],
    [0+8, 5+8, 4+8],

    [0+8, 3+8, 7+8],
    [0+8, 7+8, 4+8],
]

normals = [
    XYZ(0, 0, -1),
    XYZ(0, 0, -1),

    XYZ(0, 0, 1),
    XYZ(0, 0, 1),
    XYZ(0, 1, 0),
    XYZ(0, 1, 0),
    XYZ(1, 0, 0),
    XYZ(1, 0, 0),
    XYZ(-1, 0, 0),
    XYZ(-1, 0, 0),
    XYZ(0, -1, 0),
    XYZ(0, -1, 0),
    XYZ(0, 0, -1),
    XYZ(0, 0, -1),
    XYZ(0, 0, 1),
    XYZ(0, 0, 1),
    XYZ(0, 1, 0),
    XYZ(0, 1, 0),
    XYZ(1, 0, 0),
    XYZ(1, 0, 0),
    XYZ(-1, 0, 0),
    XYZ(-1, 0, 0),
    XYZ(0, -1, 0),
    XYZ(0, -1, 0),

]
mesh_geom = MeshGeom(vertexs, angles, normals)

logging.debug(f"顶点数据:{mesh_geom.Vertexs}")
logging.debug(f"三角形数据:{mesh_geom.Triangles}")
logging.debug(f"法向量数据:{mesh_geom.Normals}")


vertexs_d: List[XYZ] = mesh_geom.Vertexs
angles_: List[Triangle] = mesh_geom.Triangles
normals: List[XYZ] = mesh_geom.Normals
bsptree = BSPTree()
for angle in angles_:
    bsptree.append(angle)

logging.debug(bsptree)

logging.debug(bsptree.check_in(XYZ(20,0,0)))
