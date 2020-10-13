"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/13
Python:     python3.6

尝试进行几何体相关的计算？

"""
from typing import List
from enum import Enum
import logging

from .gemoetry import GeomObjectIn, Triangle, XYZ
from .tree import BSPTree, BSPNode

logger = logging.getLogger(__name__)


class BooleanOperation(Enum):
    """
    对照revit 中几何体计算方式
    """
    Union = 1  # 合并
    Difference = 2  # 不同
    Intersect = 3  # 相交


def out_triangles(geom: GeomObjectIn, tree: BSPTree) -> List[Triangle]:
    """
    从geom 中取出在tree 外部的三角面
    :param geom:
    :param tree:
    :return:
    """
    back_angles: List[Triangle] = []
    use_angles: List[Triangle] = geom.Triangles

    while use_angles:
        logger.debug(f"============")
        node: BSPNode = tree.head
        angle: Triangle = use_angles.pop()
        while node:
            checks: List[float] = [node.plane.check_in(mid) for mid in angle.Vertexs]
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

            if len(out_point) > 0 and len(in_point) > 0:  # 被当前节点分割,需要进一步处理
                # 部分未实际相交但是和面相交的面也会被分到此处,用于后续的计算,

                logger.debug(f"面相交,延后处理")
                break
            else:  # 未被当前节点分割,需要继续找后续的节点
                if len(out_point) > 0:  # 平面外侧
                    if node.upp is None:
                        logger.debug(f"plane:{node.plane}")
                        logger.debug(f"angle:{angle}")
                        back_angles.append(angle)
                        break
                    else:  # 该平面外侧还有东西,非凸多面体,需要再次判断
                        node = node.upp
                elif len(in_point) >= 0:  # 都在平面内侧
                    if node.downp is not None:  # 内侧要有需要对比的位置
                        node = node.downp
                    else:  # 没有需要对比的位置了
                        break
                else:
                    raise Exception("不应该有此种情况")

    return back_angles


class BooleanOperationUtils:

    @classmethod
    def execute_boolean_operation(cls, geom1: GeomObjectIn, geom2: GeomObjectIn, operation: BooleanOperation):
        """
        对标revit 接口用法,对两个几何体取布尔运算处理
        :param geom1:
        :param geom2:
        :param operation:
        :return:
        """
        angles: List[Triangle] = []
        bsp_tree: BSPTree = BSPTree.create_from_geom(geom2)  # 用于对比的树

        if operation == BooleanOperation.Intersect:  # 相交部分
            angles.extend(out_triangles(geom1, bsp_tree))

        logger.debug(angles)
