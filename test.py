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

from ImSlow.gemoetry import XYZ, Triangle, Plane

logging.basicConfig(level=logging.DEBUG)


def af() -> [FunctionType, MethodType]:
    for i in range(10):
        yield i


a = XYZ(1, 1, 0)
b = XYZ(1, 2, 0)
c = XYZ(0, 0, 0)
d = XYZ(0, 0, 1)
triangle = Triangle(a, b, c, normal=d)
plane = Plane(triangle.Vertexs[0], triangle.Normal)
logging.debug(plane.check_in(XYZ(2,1,0.1)))
