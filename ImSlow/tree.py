"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/12
Python:     python3.6

"""
import logging
from typing import List
from .gemoetry import GeomObjectIn, XYZ, Plane, Triangle

logger = logging.getLogger(__name__)


# BSP 树此处不考虑通过平衡进行优化

class BSPNode:
    """
    BSP 树中节点
    """
    plane: Plane
    angles: List[Triangle]  # 共面的三角形,最后一定会落脚在共面三角形
    upp: "BSPNode" = None  # up 是顶部朝向
    downp: "BSPNode" = None  # down 是内部朝向

    def __init__(self, plane: Plane):
        self.plane = plane
        self.angles = []

        # TODO 处理各个三角面该处于哪个分支


class BSPTree:
    """
    将一个几何体对象,转换到BSP 树中做碰撞检测判断
    """
    head: BSPNode

    def __init__(self):
        self.head = None

    def append(self, angle: Triangle):
        """
        添加一个平面分割
        :param node:
        :return:
        """
        plane: Plane = Plane(angle.Vertexs[0], angle.Normal)  # 点法式创建

        if self.head is None:
            self.head = BSPNode(plane)

        # 遍历判断
        node: BSPNode = self.head
        while node:
            in_check1 = node.plane.check_in(angle.Vertexs[0])
            in_check2 = node.plane.check_in(angle.Vertexs[1])
            in_check3 = node.plane.check_in(angle.Vertexs[2])

            if in_check1 == 0 and in_check2 == 0 and in_check3 == 0:
                node.angles.append(angle)
                break
            if in_check1 <= 0 and in_check2 <= 0 and in_check3 <= 0:
                if node.downp is None:
                    node.downp = BSPNode(plane)
                node = node.downp
            if in_check1 >= 0 and in_check2 >= 0 and in_check3 >= 0:  # 在内部
                if node.upp is None:
                    node.upp = BSPNode(plane)
                node = node.upp

    def check_in(self, ver: XYZ) -> bool:
        """
        判断一个点是否在几何体中
        :param ver:
        :return:
        """
        result = False

        node: BSPNode = self.head
        while node:
            status = node.plane.check_in(ver)
            if status > 0:
                node = node.upp
                if node is None:
                    result = False
                    break
                else:
                    continue
            elif status <= 0:
                node = node.downp
                if node is None:
                    result = True
                    break
                else:
                    continue

        return result
