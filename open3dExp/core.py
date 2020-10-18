"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/17
Python:     python3.6

"""
import logging
import copy
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
    _error: float

    def __init__(self, vertice: np.ndarray, normal: np.ndarray, error: float = 1e-10):
        self._normal = normal
        self._vertice = vertice
        self._error = error

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
        # 这里会根据误差,进行判断
        value = np.matmul(self.Normal, (vertice - self.Vertice).T)

        return [0, value][abs(value) > self._error]

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
        back_point: np.ndarray = None
        status_start = self.check_in(start)
        status_end = self.check_in(end)

        if status_end == 0 and status_end == 0:
            raise Exception("整条直线都在平面上,不应该有此逻辑")
        elif status_end == 0:
            back_point = end
        elif status_start == 0:
            back_point = start
        else:
            # 通过减少中间的重复计算提高了精度,此处计算出来的点应该能确保在平面上
            project_point: np.ndarray = self.project(end)  # 点到平面的垂点
            v_line: np.ndarray = start - end  # 直线向量
            v_project: np.ndarray = project_point - end
            intersection_point: np.ndarray = end + v_line * np.linalg.norm(v_project) * np.linalg.norm(
                v_project) / v_line.dot(v_project)
            back_point = intersection_point
        return back_point


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


def split_triangle_by_plane(triangle: Triangle, plane: Plane, error: float = 1e-10) -> Tuple[
    List[Triangle], List[Triangle], List[Triangle],List[Triangle]]:
    """
    用平面切割三角形,最终得到多个碎三角形,依次分别表示在平面外，平面内，平面上的三角形（法向量方向）

    是按照右手定则写的提取规则,提取后的三角形满足右手定则

    通过初步的测试,如果传入的三角形不满足右手定则,也能正确分割,但是分割后的三角形将保持传入三角形的点顺序（顺时针or 逆时针）

    :param triangle:
    :param plane:
    :return:
    """
    check_triangles: List[Triangle] = [triangle]  # 形如广度搜索的分割方式
    out_triangles: List[Triangle] = []
    in_triangles: List[Triangle] = []
    on_same:List[Triangle] = []
    on_diff:List[Triangle] = []

    while check_triangles:
        triangle_current: Triangle = check_triangles.pop()
        vertices_on_triangle: o3d.open3d_pybind.utility.Vector3dVector = o3d.utility.Vector3dVector()  # 定义在平面上的三角形

        for i, vertice in enumerate(triangle_current.Vertices):
            check_status: float = plane.check_in(vertice)

            vertices_new_use_angle: o3d.open3d_pybind.utility.Vector3dVector = o3d.utility.Vector3dVector()
            vertices_new_use_angle.append(vertice)
            other_vertices: o3d.open3d_pybind.utility.Vector3dVector = o3d.utility.Vector3dVector()

            if check_status == 0:  # 误差范围内的
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
            triangle_append = Triangle(vertices_on_triangle, triangle_current.Normal)
            cos_value = triangle_append.Normal.dot(plane.Normal) / (np.linalg.norm(triangle_append.Normal) * np.linalg.norm(plane.Normal))

            if abs(1-cos_value)<error:
                on_same.append(triangle_append)
            else:
                on_diff.append(triangle_append)
    return out_triangles, in_triangles, on_same,on_diff


class BSPTree:
    """
    BSP 树,暂时不尝试进行平衡BSP 树优化
    """
    head: BSPNode

    def __init__(self):
        self.head = None

    @classmethod
    def create_from_triangle_mesh(cls, mesh: o3d.open3d_pybind.geometry.TriangleMesh,
                                  error: float = 1e-10) -> "BSPTree":
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
            plane_mid = Plane(triangle.Vertices[0], normal=triangle.Normal, error=error)
            if tree.head is None:
                tree.head = BSPNode(Plane(triangle.Vertices[0], triangle.Normal))

            # TODO 需要处理分割平面的信息
            task_queue: List[Tuple[BSPNode, Triangle]] = [(tree.head, triangle)]
            while task_queue:
                node_mid, triangle_mid_use = task_queue.pop()

                out_triangles, in_triangles, on_same,on_diff = split_triangle_by_plane(triangle_mid_use,
                                                                                    node_mid.plane)

                for triangle_tmp in on_same:
                    node_mid.triangles.append(triangle_tmp)  # 这里的处理是正常的
                for triangle_tmp in on_diff:
                    node_mid.triangles.append(triangle_tmp)
                if len(out_triangles) > 0:
                    if node_mid.out_node is None:
                        node_mid.out_node = BSPNode(
                            Plane(out_triangles[0].Vertices[0], out_triangles[0].Normal, error=error))
                    for triangle_tmp in out_triangles:
                        task_queue.append((node_mid.out_node, triangle_tmp))  # 将各个子三角形添加到任务列表中等待处理

                if len(in_triangles) > 0:
                    if node_mid.in_node is None:
                        node_mid.in_node = BSPNode(
                            Plane(in_triangles[0].Vertices[0], in_triangles[0].Normal, error=error))
                    for triangle_tmp in in_triangles:
                        task_queue.append((node_mid.in_node, triangle_tmp))
            # break
        return tree

    @classmethod
    def to_triangle_mesh(cls, tree: "BSPTree", error: float = 1e-10) -> o3d.open3d_pybind.geometry.TriangleMesh:
        """
        将BSP 树转换成open3d特定格式
        :param tree:
        :return:
        """
        mesh: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh()
        node_cache: List[BSPNode] = [tree.head]
        while node_cache:
            node_use: BSPNode = node_cache.pop()
            for triangle in node_use.triangles:
                triangle_index = []
                for i in range(3):
                    index = -1
                    for v_i, vertice in enumerate(mesh.vertices):
                        vertice: np.ndarray
                        length = np.linalg.norm(vertice - triangle.Vertices[i])
                        if abs(length) <= error:  # 同一个点
                            index = v_i
                            break
                    if index < 0:
                        mesh.vertices.append(triangle.Vertices[i])
                        index = len(mesh.vertices) - 1
                    triangle_index.append(index)

                triangle_index_np = np.asarray(triangle_index, dtype=np.int32)
                mesh.triangles.append(triangle_index_np)
                mesh.triangle_normals.append(triangle.Normal)
            if node_use.out_node:
                node_cache.append(node_use.out_node)
            if node_use.in_node:
                node_cache.append(node_use.in_node)

        return mesh


def split_triangle_mesh_by_bsp_tree(mesh: o3d.open3d_pybind.geometry.TriangleMesh, tree: BSPTree,
                                    error: float = 1e-10) -> Tuple[
    List[Triangle], List[Triangle], List[Triangle],List[Triangle]]:
    """
    使用bsp 树分割原有的triangle_mesh,得到out_triangle,in_triangle,on_triangle

    :param mesh:
    :param tree:
    :return:
    """
    out_triangles: List[Triangle] = []
    in_triangles: List[Triangle] = []
    on_same_triangles: List[Triangle] = []
    on_diff_triangles:List[Triangle] = []

    for angle_index, mesh_angle in enumerate(mesh.triangles):
        vertices = o3d.utility.Vector3dVector()
        for i in range(3):
            vertices.append(mesh.vertices[mesh_angle[i]])

        triangle_mid = Triangle(vertices, mesh.triangle_normals[angle_index])
        task_queue: List[Tuple[Triangle, BSPNode, int]] = [(triangle_mid, tree.head, 0)]
        while task_queue:
            triangle_use, node_use, surface_status_use = task_queue.pop()  # 此前计算的相关信息

            out_mid, in_mid, on_same_mid,on_diff_mid = split_triangle_by_plane(triangle_use, node_use.plane, error)
            if node_use.out_node is None:
                out_triangles.extend(out_mid)  # 通过plane 切割,确定在几何体外部
            else:
                # 可能在内部,也可能在内部,要继续讨论
                for out_single_trangle in out_mid:
                    task_queue.append((out_single_trangle, node_use.out_node, False))

            if node_use.in_node is None:
                for in_single_trangle in in_mid:
                    if surface_status_use == 0:  # 剔除曾经在表面然后现在又在内部的三角面
                        in_triangles.append(in_single_trangle)
                    elif surface_status_use == 1: # 同向
                        on_same_triangles.append(in_single_trangle)
                    else:# 异向
                        on_diff_triangles.append(in_single_trangle)

                if on_same_mid:  # 直接就在表面且无in node
                    on_same_triangles.extend(on_same_mid)
                if on_diff_mid:
                    on_diff_triangles.extend(on_diff_mid)
            else:
                for in_single_trangle in in_mid:
                    task_queue.append((in_single_trangle, node_use.in_node, surface_status_use))
                for on_single_trangle in on_same_mid:
                    task_queue.append((on_single_trangle, node_use.in_node, 1))
                for on_single_trangle in on_diff_mid:
                    task_queue.append(((on_single_trangle,node_use.in_node,2)))
    return out_triangles, in_triangles, on_same_triangles,on_diff_triangles


class BooleanOperationUtils:
    """

    几何体布尔运算

    """

    @classmethod
    def execute_boolean_operation(cls, geom1: o3d.open3d_pybind.geometry.TriangleMesh,
                                  geom2: o3d.open3d_pybind.geometry.TriangleMesh,
                                  operation: BooleanOperation,
                                  error: float = 1e-10) -> o3d.open3d_pybind.geometry.TriangleMesh:
        """
        对标revit 接口用法,对两个几何体取布尔运算处理
        :param geom1:
        :param geom2:
        :param operation:
        :return:
        """
        mesh: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh()  # 这是需要返回的最终几何体
        make_normals(geom1)
        make_normals(geom2)

        def append(angle: Triangle) -> None:
            """
            添加点和三角形到mesh中,此处操作费事需优化
            :param angle:
            :return:
            """
            triangle_index = []
            for i in range(3):
                index = -1
                # for v_i, vertice in enumerate(mesh.vertices):
                #     vertice: np.ndarray
                #     length = np.linalg.norm(vertice - angle.Vertices[i])
                #     if abs(length) <= error:  # 同一个点
                #         index = v_i
                #         break
                # if index < 0:
                mesh.vertices.append(angle.Vertices[i])
                index = len(mesh.vertices) - 1

                triangle_index.append(index)

            triangle_index_np = np.asarray(triangle_index, dtype=np.int32)
            mesh.triangles.append(triangle_index_np)
            mesh.triangle_normals.append(angle.Normal)

        # BSP 树存储数据,广度搜索
        geom2_tree: BSPTree = BSPTree.create_from_triangle_mesh(geom2)
        geom1_tree: BSPTree = BSPTree.create_from_triangle_mesh(geom1)
        triangles_all: List[Triangle] = []  # 存放所有的三角面片
        out_triangles, in_triangles, on_same_triangles,on_diff_triangles = split_triangle_mesh_by_bsp_tree(geom1, geom2_tree)
        out_triangles2, in_triangles2, on_same_triangles2,on_diff_triangles2 = split_triangle_mesh_by_bsp_tree(geom2, geom1_tree)
        if operation == BooleanOperation.Union: # 并集
            triangles_all.extend(out_triangles)
            triangles_all.extend(out_triangles2)
            triangles_all.extend(on_same_triangles)

        if operation == BooleanOperation.Intersect:  # 相交部分
            triangles_all.extend(in_triangles)
            triangles_all.extend(in_triangles2)
            if len(in_triangles)!=0 and len(in_triangles2)!=0: # 如果等于0，就未相交了
                triangles_all.extend(on_same_triangles)

        if operation == BooleanOperation.Difference:  # 不同部分

            triangles_all.extend(out_triangles)

            triangle_in_2: List[Triangle] = []

            for single_triangle in in_triangles2:
                vertices = o3d.utility.Vector3dVector()
                vertices.append(single_triangle.Vertices[2])
                vertices.append(single_triangle.Vertices[1])
                vertices.append(single_triangle.Vertices[0])
                triangle_in_2.append(Triangle(vertices, single_triangle.Normal * -1))

            triangles_all.extend(triangle_in_2)

            triangles_all.extend(on_diff_triangles)

        for final_triangle in triangles_all:
            append(final_triangle)

        return mesh
