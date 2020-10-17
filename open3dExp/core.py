"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/17
Python:     python3.6

"""
import logging
from typing import List, Dict, Tuple
from enum import Enum

import open3d as o3d
import numpy as np

logger = logging.getLogger(__name__)


def make_normals(geom: o3d.open3d_pybind.geometry.TriangleMesh) -> None:
    """
    尝试对TriangleMesh 生成法向量
    :param geom:
    :return:
    """
    if len(geom.triangle_normals) == 0:
        geom.compute_vertex_normals()


class BooleanOperation(Enum):
    """
    对照revit 中几何体计算方式
    """
    Union = 1  # 合并
    Difference = 2  # 不同
    Intersect = 3  # 相交


class Triangle:
    """

    定义一个可以用于判断平面的三角形

    """
    _normal: np.ndarray
    _vertices: o3d.open3d_pybind.utility.Vector3dVector

    def __init__(self, points: o3d.open3d_pybind.utility.Vector3dVector, normal: np.ndarray):
        # TODO 待添加更多限制
        assert len(points) == 3, Exception("三角形初始化时,必须有三个点")
        v1 = points[0] - points[1]
        v2 = points[2] - points[1]
        normal_count = np.cross(v2,v1)
        normal_count = normal_count/np.linalg.norm(normal_count)
        logger.debug(f"求出来的法向量:{normal_count}")
        logger.debug(f"传入的法向量:{normal}")
        self._vertices = points
        self._normal = normal

    @property
    def Normal(self) -> np.ndarray:
        return self._normal

    @property
    def Vertices(self) -> o3d.open3d_pybind.utility.Vector3dVector:
        return self._vertices

    def __str__(self):
        return f"{self.__class__.__name__} Vertices:\n{np.asarray(self.Vertices)} \n Normal:\t{self.Normal}"

    def __repr__(self):
        return self.__str__()


class Plane:
    """
    点法式定义平面
    """
    _normal: np.ndarray
    _vertice: np.ndarray

    def __init__(self, vertice: np.ndarray, normal: np.ndarray):
        self._normal = normal
        self._vertice = vertice

    @property
    def Normal(self) -> np.ndarray:
        return self._normal

    @property
    def Vertice(self) -> np.ndarray:
        return self._vertice

    def check_in(self, vertice: np.ndarray) -> float:
        """
        判断一个点是否在平面上
        :param vertice:
        :return:
        """
        return np.matmul(self.Normal, (vertice - self.Vertice).T)

    def __str__(self):
        return f"Center:{self.Vertice},Normal:{self.Normal}"

    def project(self, vertice: np.ndarray) -> np.ndarray:
        """
        求平面外一点到平面内一点的垂点
        :param vertice:
        :return:
        """
        v1 = self.Vertice - vertice

        if np.linalg.norm(v1) == 0:  # 向量长度
            return self.Vertice

        return vertice + self.Normal * (v1.dot(self.Normal) / np.linalg.norm(self.Normal))

    def intersection(self, start: np.ndarray, end: np.ndarray) -> np.ndarray:
        """
        求直线和平面的交点
        :param start:
        :param end:
        :return:
        """
        status_start = self.check_in(start)
        status_end = self.check_in(end)

        if status_end == 0 and status_end == 0:
            raise Exception("整条直线都在平面上,不应该有此逻辑")
        elif status_end == 0:
            return end
        elif status_start == 0:
            return start
        else:
            # 通过减少中间的重复计算提高了精度,此处计算出来的点应该能确保在平面上
            project_point: np.ndarray = self.project(end)  # 点到平面的垂点
            v_line: np.ndarray = start - end  # 直线向量
            v_project: np.ndarray = project_point - end
            intersection_point: np.ndarray = end + v_line * np.linalg.norm(v_project) * np.linalg.norm(
                v_project) / v_line.dot(v_project)
            assert self.check_in(intersection_point) == 0, Exception("交点求出来无法通过点法式平面验证")
            return intersection_point


class BSPNode:
    """
    BSP 树的节点信息
    """
    plane: Plane
    triangles: List[Triangle]  # 落在切面上的三角形
    out_node: "BSPNode"  # 在几何体切面外
    in_node: "BSPNode"  # 在几何体切面内

    def __init__(self, plane: Plane):
        self.plane = plane
        self.triangles = []
        self.out_node = None
        self.in_node = None


def split_triangle_by_plane(triangle: Triangle, plane: Plane) -> Tuple[List[Triangle], List[Triangle], List[Triangle]]:
    """
    用平面切割三角形,最终得到多个碎三角形,依次分别表示在平面外，平面内，平面上的三角形

    是按照右手定则写的提取规则,提取后的三角形满足右手定则

    通过初步的测试,如果传入的三角形不满足右手定则,也能正确分割,但是分割后的三角形将保持传入三角形的点顺序（顺时针or 逆时针）

    :param triangle:
    :param plane:
    :return:
    """
    check_triangles: List[Triangle] = [triangle]  # 形如广度搜索的分割方式
    out_triangles: List[Triangle] = []
    in_triangles: List[Triangle] = []
    on_triangles: List[Triangle] = []

    while check_triangles:
        triangle_current: Triangle = check_triangles.pop()
        vertices_on_triangle: o3d.open3d_pybind.utility.Vector3dVector = o3d.utility.Vector3dVector()  # 定义在平面上的三角形

        for i, vertice in enumerate(triangle_current.Vertices):
            check_status: float = plane.check_in(vertice)

            vertices_new_use_angle: o3d.open3d_pybind.utility.Vector3dVector = o3d.utility.Vector3dVector()
            vertices_new_use_angle.append(vertice)
            other_vertices: o3d.open3d_pybind.utility.Vector3dVector = o3d.utility.Vector3dVector()

            if check_status == 0:
                vertices_on_triangle.append(vertice)
            else:

                for n in range(i + 1, i + 3):
                    # 需要有一个三角行了
                    vertice_mid: np.ndarray = triangle_current.Vertices[n % 3]  # 循环取点,保证右手定则

                    vertice_check: float = plane.check_in(vertice_mid)

                    if (vertice_check >= 0 and check_status > 0) or (
                            vertice_check <= 0 and check_status < 0):  # 确保同侧
                        vertices_new_use_angle.append(vertice_mid)

                    else:  # 异侧,需要求交点
                        other_vertices.append(vertice_mid)
                        intersect: np.ndarray = plane.intersection(vertices_new_use_angle[0],
                                                                   vertice_mid)  # 交点
                        vertices_new_use_angle.append(intersect)

                other_vertices_length = len(other_vertices)
                if other_vertices_length > 0:
                    other_vertices.append(vertices_new_use_angle[2])
                    if other_vertices_length == 1:  # 异侧点一个,需要继续处理,长度未获取更新后的
                        other_vertices.append(vertices_new_use_angle[1])
                    else:
                        vertices_mid_: o3d.open3d_pybind.utility.Vector3dVector = o3d.utility.Vector3dVector()
                        vertices_mid_.append(vertices_new_use_angle[2])
                        vertices_mid_.append(vertices_new_use_angle[1])
                        vertices_mid_.append(other_vertices[0])
                        check_triangles.append(Triangle(vertices_mid_, triangle_current.Normal))
                    check_triangles.append(Triangle(other_vertices, triangle_current.Normal))

                if len(vertices_new_use_angle) == 3:
                    if check_status < 0:
                        in_triangles.append(Triangle(vertices_new_use_angle, triangle_current.Normal))
                    else:
                        out_triangles.append(Triangle(vertices_new_use_angle, triangle_current.Normal))
                else:
                    raise Exception("逻辑异常：不应该出现有一个点在内侧却找不到内侧三角形")
                break  # 至关重要的跳出
        if len(vertices_on_triangle) == 3:
            on_triangles.append(Triangle(vertices_on_triangle, triangle_current.Normal))  # 平面上的三角形添加

    return out_triangles, in_triangles, on_triangles


class BSPTree:
    """
    BSP 树,暂时不尝试进行平衡BSP 树优化
    """
    head: BSPNode

    def __init__(self):
        self.head = None

    @classmethod
    def create_from_triangle_mesh(cls, mesh: o3d.open3d_pybind.geometry.TriangleMesh) -> "BSPTree":
        """
        尝试基于一个TriangleMesh 生成一个BSP 树
        :param mesh:
        :return:
        """

        make_normals(mesh)
        tree: BSPTree = BSPTree()

        for triangle_index, singe_triangle in enumerate(mesh.triangles):

            # 存放三角形的点
            vertices: o3d.open3d_pybind.utility.Vector3dVector = o3d.utility.Vector3dVector()
            for i in range(3):
                vertices.append(mesh.vertices[singe_triangle[i]])

            triangle: Triangle = Triangle(vertices, mesh.triangle_normals[triangle_index])
            v1 = triangle.Vertices[0] - triangle.Vertices[1]
            v2 = triangle.Vertices[2] - triangle.Vertices[1]
            if tree.head is None:
                tree.head = BSPNode(Plane(triangle.Vertices[0], triangle.Normal))

            # TODO 需要处理分割平面的信息
            task_queue: List[Tuple[BSPNode, Triangle]] = [(tree.head, triangle)]
            while task_queue:
                logger.debug(f"===============")
                node_mid, triangle_mid_use = task_queue.pop()

                out_triangles, in_triangles, on_triangles = split_triangle_by_plane(triangle_mid_use,
                                                                                    node_mid.plane)

                for triangle_tmp in on_triangles:
                    logger.debug(f"直接放入某个节点")
                    node_mid.triangles.append(triangle_tmp)  # 这里的处理是正常的

                if len(out_triangles)>0:
                    logger.debug(f"out 一侧存在数据{len(out_triangles)},待处理")
                    if node_mid.out_node is None:
                        logger.debug(f"out 侧无子节点,即将自动生成子节点")
                        # TODO mesh 三角形的法向量异常

                        node_mid.out_node = BSPNode(Plane(out_triangles[0].Vertices[0], out_triangles[0].Normal))
                        # TODO 检查一下三个点是否在该平面上
                        for point in out_triangles[0].Vertices:
                            logger.debug(f"检查点是否在平面内:{node_mid.out_node.plane.check_in(point)}")
                    for triangle_tmp in out_triangles:
                        task_queue.append((node_mid.out_node, triangle_tmp))  # 将各个子三角形添加到任务列表中等待处理

                        # node_mid.out_node.triangles.append(triangle_tmp)
                if len(in_triangles)>0:
                    logger.debug(f"in side 一侧存在数据{len(in_triangles)},待处理")
                    if node_mid.in_node is None:
                        logger.debug("inside 一侧无节点,即将自动生成子节点")
                        node_mid.in_node =BSPNode(Plane(in_triangles[0].Vertices[0],in_triangles[0].Normal))
                        # TODO 检查一下三个点是否在该平面上
                    for point in in_triangles[0].Vertices:
                            logger.debug(f"检查点是否在平面内:{node_mid.in_node.plane.check_in(point)}")
                    for triangle_tmp in in_triangles:
                        task_queue.append((node_mid.in_node,triangle_tmp))
            logger.debug("结束一个单个三角形存放")
            # break
        return tree


class BooleanOperationUtils:
    """

    几何体布尔运算

    """

    @classmethod
    def execute_boolean_operation(cls, geom1: o3d.open3d_pybind.geometry.TriangleMesh,
                                  geom2: o3d.open3d_pybind.geometry.TriangleMesh,
                                  operation: BooleanOperation) -> o3d.open3d_pybind.geometry.TriangleMesh:
        """
        对标revit 接口用法,对两个几何体取布尔运算处理
        :param geom1:
        :param geom2:
        :param operation:
        :return:
        """
        make_normals(geom1)
        make_normals(geom2)
