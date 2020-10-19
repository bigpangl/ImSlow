"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/12
Python:     python3.6

"""
import logging
import unittest
from typing import List
from types import FunctionType, MethodType

import open3d as o3d
from inspect import isgeneratorfunction
import numpy as np

logging.basicConfig(level=logging.DEBUG)


class TestUse(unittest.TestCase):
    def test_import_booleanoperation(self):
        from open3dExp.cycore import BooleanOperation
        logging.debug(BooleanOperation.Union)

    def test_triangle(self):
        """
        测试triangle 的相关功能
        :return:
        """
        from open3dExp.cycore import Triangle
        points = o3d.utility.Vector3dVector()
        points.append(np.asarray([0,0,0]))
        points.append(np.asarray([1,0,0]))
        points.append(np.asarray([0,1,0]))
        normal = np.asarray([0,0,1])
        angle = Triangle(points,normal)
        logging.debug(angle)
        logging.debug(angle.check_in(np.asarray([1,1,0])))

if __name__ == '__main__':
    unittest.main()
