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
from inspect import isgeneratorfunction
import numpy as np

from ImSlow.gemoetry import XYZ, Triangle, Plane, MeshGeom, Line, BoundingBox
from ImSlow.tree import BSPTree
from ImSlow.utils import BooleanOperation, BooleanOperationUtils

logging.basicConfig(level=logging.DEBUG)


class IMSlowTest(unittest.TestCase):

    @unittest.skip("")
    def test_plane_check_in(self):
        plane = Plane(XYZ(5, 5, 0), XYZ(-1, 0, 0))
        check_point = XYZ(5, 5, 0)
        status = plane.check_in(check_point)
        self.assertEqual(status, 0, f"点{check_point} 应该在平面{plane}上")

    # @unittest.skip("")
    def test_intersection(self):
        """
        测试交点是否正确
        :return:
        """
        plane = Plane(XYZ(1, 1, 0), XYZ(0, 0, 1))
        point: XYZ = plane.intersection(Line(XYZ(0, 0, 1), XYZ(0, 0, -1)))

    # @unittest.skip("")
    def test_boundingbox(self):
        box = BoundingBox(XYZ(0, 0, 0), XYZ(10, 10, 10))

        tree = BSPTree.create_from_geom(box)
        # 内部点
        self.assertEqual(tree.check_in(XYZ(5, 5, 5)), True, "校验错误")
        self.assertEqual(tree.check_in(XYZ(6, 6, 6)), True, "校验错误")

        # 8个顶点
        self.assertEqual(tree.check_in(XYZ(0, 0, 0)), True, "校验错误")
        self.assertEqual(tree.check_in(XYZ(10, 0, 0)), True, "校验错误")
        self.assertEqual(tree.check_in(XYZ(0, 10, 0)), True, "校验错误")
        self.assertEqual(tree.check_in(XYZ(0, 0, 10)), True, "校验错误")
        self.assertEqual(tree.check_in(XYZ(10, 10, 0)), True, "校验错误")
        self.assertEqual(tree.check_in(XYZ(0, 10, 10)), True, "校验错误")
        self.assertEqual(tree.check_in(XYZ(10, 0, 10)), True, "校验错误")
        self.assertEqual(tree.check_in(XYZ(10, 10, 10)), True, "校验错误")

        # 外部点切面点
        self.assertEqual(tree.check_in(XYZ(11, 0, 0)), False, "校验错误")
        self.assertEqual(tree.check_in(XYZ(0, 11, 0)), False, "校验错误")
        self.assertEqual(tree.check_in(XYZ(0, 0, 11)), False, "校验错误")
        # 外部非切面点
        self.assertEqual(tree.check_in(XYZ(-1, -1, 0)), False, "校验错误")
        self.assertEqual(tree.check_in(XYZ(11, 11, 0)), False, "校验错误")

        logging.debug(f"常规包围框顶点判断正确")

    # @unittest.skip("")
    def test_boolean_intersect_A(self):
        """
        没有共面的切面

        :return:
        """
        box = BoundingBox(XYZ(0, 0, 0), XYZ(10, 10, 10))

        box2 = BoundingBox(XYZ(5, 5, 5), XYZ(15, 15, 15))

        geom: MeshGeom = BooleanOperationUtils.execute_boolean_operation(box, box2, BooleanOperation.Intersect)  # 交
        for angele in geom.Triangles:
            logging.debug(f"三角形:{angele},normal:{angele.Normal}")
        tree = BSPTree.create_from_geom(geom)
        self.assertEqual(tree.check_in(XYZ(10, 10, 10)), True, "校验失败")
        self.assertEqual(tree.check_in(XYZ(12, 12, 12)), False, "校验失败")
        self.assertEqual(tree.check_in(XYZ(3, 3, 3)), False, "校验失败")
        self.assertEqual(tree.check_in(XYZ(5, 5, 5)), True, "校验失败")

    @unittest.skip("暂时跳过几何体运算相关")
    def test_boolean_intersect_B(self):
        """
        有共面的切面
        :return:
        """
        box = BoundingBox(XYZ(0, 0, 0), XYZ(10, 10, 10))

        box2 = BoundingBox(XYZ(5, 5, 5), XYZ(10, 10, 10))

        geom: MeshGeom = BooleanOperationUtils.execute_boolean_operation(box, box2, BooleanOperation.Intersect)  # 交
        tree = BSPTree.create_from_geom(geom)

        self.assertEqual(tree.check_in(XYZ(10, 10, 10)), True, "校验失败")
        self.assertEqual(tree.check_in(XYZ(12, 12, 12)), False, "校验失败")
        self.assertEqual(tree.check_in(XYZ(3, 3, 3)), False, "校验失败")
        self.assertEqual(tree.check_in(XYZ(5, 5, 5)), True, "校验失败")


if __name__ == '__main__':
    unittest.main()
