"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/12
Python:     python3.6

"""

import logging
from typing import List
from .gemoetry import GeomObjectIn, XYZ, Plane, Triangle, Line

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
        添加一个平面分割,如何通过弹栈操作,避免最大递归深度的问题
        :param angle:
        :return:
        """
        angles = [angle]  # 此种方式会较慢

        while angles:

            angle = angles.pop()
            plane: Plane = Plane(angle.Vertexs[0], angle.Normal)  # 点法式创建

            if self.head is None:
                self.head = BSPNode(plane)

            # 遍历判断
            node: BSPNode = self.head
            while node:
                checks: List[float] = [node.plane.check_in(tmp) for tmp in angle.Vertexs]

                out_point: List[XYZ] = []
                in_point: List[XYZ] = []
                on_point: List[XYZ] = []
                for i, check in enumerate(checks):
                    if check == 0:
                        on_point.append(angle.Vertexs[i])
                    elif check > 0:  # 外侧
                        out_point.append(angle.Vertexs[i])
                    elif check < 0:  # 内侧
                        in_point.append(angle.Vertexs[i])

                if len(out_point) > 0 and len(in_point) > 0:  # 被切割了
                    # 相交的线条
                    lines: List[Line] = []
                    for start in out_point:
                        for end in in_point:
                            lines.append(Line(start, end))
                    cross_points: List[XYZ] = []
                    # 处理单个线条
                    for line in lines:
                        line: Line
                        cross_mid = node.plane.intersection(line)
                        cross_points.append(cross_mid)
                    # 得到了交点
                    if len(cross_points) == 1:  # 一个交点,说明一个点在平面上，内外点各一个
                        ange_new_out = Triangle(on_point[0], out_point[0], cross_points[0],
                                                normal=angle.Normal)  # 新的三角面
                        ange_new_in = Triangle(on_point[0], in_point[0], cross_points[0], normal=angle.Normal)

                        angles.append(ange_new_in)
                        angles.append(ange_new_out)
                    elif len(cross_points) == 2:  # 两个交点,分居两侧
                        # out angle
                        angles.append(Triangle(out_point[0], cross_points[0], cross_points[1], normal=angle.Normal))
                        if len(out_point) == 2:  # 两个外部点
                            angles.append(Triangle(out_point[0], out_point[1], cross_points[1]))
                        # in angle
                        angles.append(Triangle(in_point[0], cross_points[0], cross_points[1], normal=angle.Normal))
                        if len(in_point) == 2:
                            angles.append(Triangle(in_point[0], in_point[1], cross_points[1], normal=angle.Normal))
                    else:
                        raise Exception
                    break

                else:  # 没有被切割,也就是都在同侧或者在该平面上
                    if len(on_point) == 3:  # 都在平面上
                        node.angles.append(angle)  # 不需要重新创建对象
                        break
                    elif len(out_point) > 0:  # 平面外侧
                        if node.upp is None:
                            node.upp = BSPNode(plane)
                        node = node.upp
                    elif len(in_point) > 0:  # 平面内侧
                        if node.downp is None:
                            node.downp = BSPNode(plane)
                        node = node.downp
                    else:
                        raise Exception

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

            if status > 0:  # 平面外部
                node = node.upp
                if node is None:
                    result = False
                    break
                else:
                    continue
            elif status <= 0:  # 内部
                node = node.downp
                if node is None:
                    result = True
                    break
                else:
                    continue

        return result

    @classmethod
    def create_from_geom(cls, geom: GeomObjectIn) -> "BSPTree":
        """
        从几何体转变成BSP 树
        :param geom:
        :return:
        """
        angles_: List[Triangle] = geom.Triangles
        bsp_tree: BSPTree = BSPTree()
        for angle in angles_:
            bsp_tree.append(angle)
        return bsp_tree
