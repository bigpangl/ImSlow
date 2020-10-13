"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/13
Python:     python3.6

尝试进行几何体相关的计算？

"""
import copy
from typing import List, Dict
from enum import Enum
import logging

from .gemoetry import GeomObjectIn, Triangle, XYZ, Line, MeshGeom, Plane
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
    use_angles: List[Triangle] = copy.copy(geom.Triangles)

    while use_angles:
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

                    use_angles.append(ange_new_in)
                    use_angles.append(ange_new_out)

                elif len(cross_points) == 2:  # 两个交点,分居两侧
                    # out angle
                    use_angles.append(Triangle(out_point[0], cross_points[0], cross_points[1], normal=angle.Normal))
                    if len(out_point) == 2:  # 两个外部点
                        use_angles.append(Triangle(out_point[0], out_point[1], cross_points[1]))

                    # in angle
                    use_angles.append(Triangle(in_point[0], cross_points[0], cross_points[1], normal=angle.Normal))
                    if len(in_point) == 2:
                        use_angles.append(Triangle(in_point[0], in_point[1], cross_points[1], normal=angle.Normal))
                else:
                    raise Exception
                break


            else:  # 未被当前节点分割,需要继续找后续的节点
                if len(out_point) > 0:  # 平面外侧
                    if node.upp is None:
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


def in_triangles(geom: GeomObjectIn, tree: BSPTree) -> List[Triangle]:
    """
    取得在BSPtree 内部的三角形
    :param geom:
    :param tree:
    :return:
    """
    # TODO 整个函数暂时未详细确定
    back_angles: List[Triangle] = []
    use_angles: List[Triangle] = copy.copy(geom.Triangles)

    while use_angles:
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

                    use_angles.append(ange_new_in)
                    use_angles.append(ange_new_out)

                elif len(cross_points) == 2:  # 两个交点,分居两侧
                    # out angle
                    use_angles.append(Triangle(out_point[0], cross_points[0], cross_points[1], normal=angle.Normal))
                    if len(out_point) == 2:  # 两个外部点
                        use_angles.append(Triangle(out_point[0], out_point[1], cross_points[1]))

                    # in angle
                    use_angles.append(Triangle(in_point[0], cross_points[0], cross_points[1], normal=angle.Normal))
                    if len(in_point) == 2:
                        use_angles.append(Triangle(in_point[0], in_point[1], cross_points[1], normal=angle.Normal))
                else:
                    raise Exception
                break

            else:  # 未被当前节点分割,需要继续找后续的节点
                if len(out_point) > 0:  # 平面外侧
                    if node.upp is None:

                        # back_angles.append(angle)
                        break
                    else:  # 该平面外侧还有东西,非凸多面体,需要再次判断
                        node = node.upp
                elif len(in_point) >= 0:  # 都在平面内侧
                    if node.downp is None:
                        back_angles.append(angle)  # 核定该平面在内侧
                        break
                    else:
                        node = node.downp
                else:
                    raise Exception("不应该有此种情况")
    return back_angles


class BooleanOperationUtils:
    """

    几何体布尔运算

    """

    @classmethod
    def execute_boolean_operation(cls, geom1: GeomObjectIn, geom2: GeomObjectIn,
                                  operation: BooleanOperation) -> MeshGeom:
        """
        对标revit 接口用法,对两个几何体取布尔运算处理
        :param geom1:
        :param geom2:
        :param operation:
        :return:
        """
        angles: List[Triangle] = []
        bsp_tree: BSPTree = BSPTree.create_from_geom(geom2)  # 用于对比的树

        # 第一个需要处理的部分
        if operation == BooleanOperation.Intersect:  # 相交部分
            angles.extend(in_triangles(geom1, bsp_tree))
        else:
            angles.extend(out_triangles(geom1, bsp_tree))
        bsp_tree1: BSPTree = BSPTree.create_from_geom(geom1)

        if operation == BooleanOperation.Union:
            angles.extend(out_triangles(geom2, bsp_tree1))
        else:
            angles_new = in_triangles(geom2, bsp_tree1)
            angle_new_use: List[Triangle] = []

            if operation == BooleanOperation.Difference:
                for single_angle in angles_new:
                    angle_new_use.append(Triangle(
                        single_angle.Vertexs[0],
                        single_angle.Vertexs[1],
                        single_angle.Vertexs[2],
                        normal=single_angle.Normal * -1
                    ))

            else:
                angle_new_use = angles_new

            angles.extend(angle_new_use)

        # 处理所有的三角面,形成新的几何体
        vertexs: Dict[XYZ, int] = {}
        triangles: List[List[int]] = []
        normals: List[XYZ] = []

        for angle in angles:
            angle_vertexs: List[XYZ] = angle.Vertexs
            angles_index: List[int] = []
            for point in angle_vertexs:
                if point not in vertexs:
                    vertexs[point] = len(vertexs)

                angles_index.append(vertexs[point])
            triangles.append(angles_index)
            normals.append(angle.Normal)
        return MeshGeom(vertexs, triangles, normals)
