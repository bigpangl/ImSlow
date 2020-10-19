# cython: language_level=3
from enum import Enum
import logging
from datetime import datetime

import numpy as np
cimport numpy as np
from cpython.ref cimport PyObject

import open3d as o3d

logger = logging.getLevelName(__name__)

cpdef public float STATIC_ERROR = 1e-10

def make_normals(geom: o3d.open3d_pybind.geometry.TriangleMesh) -> None:
    """
    尝试对TriangleMesh 生成法向量
    :param geom:
    :return:
    """
    if len(geom.triangle_normals) == 0:
        geom.compute_vertex_normals()


class BooleanOperation(Enum):
    Union = 1  # 合并
    Difference = 2  # 不同
    Intersect = 3  # 相交


cdef class Triangle:
    cdef public np.ndarray Normal
    cdef public Vertices
    def __init__(self, points, np.ndarray normal):
        self.Vertices = points
        self.Normal = normal

cdef class Plane:
    """
    点法式定义平面
    """
    cdef public  np.ndarray Normal
    cdef public  np.ndarray Vertice
    cdef public float error

    def __init__(self, np.ndarray vertice, np.ndarray normal, float error= STATIC_ERROR):

        self.Normal = normal
        self.Vertice = vertice
        self.error = error

    cpdef float check_in(self, vertice: np.ndarray):
        # 这里会根据误差,进行判断
        value = np.matmul(self.Normal, (vertice - self.Vertice).T)

        return [0, value][abs(value) > self.error]

    def __str__(self):
        return f"Center:{self.Vertice},Normal:{self.Normal}"

    cpdef np.ndarray project(self, vertice: np.ndarray):

        cdef np.ndarray v1 = self.Vertice - vertice

        if np.linalg.norm(v1) == 0:  # 向量长度
            return self.Vertice  # 平面中心点即垂点
        return vertice + self.Normal * (v1.dot(self.Normal) / np.linalg.norm(self.Normal))

    cpdef np.ndarray intersection(self, start: np.ndarray, end: np.ndarray):
        cdef np.ndarray back_point
        cdef np.ndarray v_line
        cdef np.ndarray v_project

        cdef float status_start = self.check_in(start)
        cdef float status_end = self.check_in(end)

        if status_start == 0 and status_end == 0:
            raise Exception("整条直线都在平面上,不应该有此逻辑")
        elif status_end == 0:
            back_point = end
        elif status_start == 0:
            back_point = start
        else:
            # 通过减少中间的重复计算提高了精度,此处计算出来的点应该能确保在平面上
            v_line = start - end  # 直线向量
            v_project = self.project(end) - end
            back_point = end + v_line * np.linalg.norm(v_project) * np.linalg.norm(
                v_project) / v_line.dot(v_project)
        return back_point

cdef class BSPNode:
    """
    BSP 树的节点信息
    """
    cdef public Plane plane
    cdef public triangles  # 落在切面上的三角形
    cdef public BSPNode out_node  # 在几何体切面外
    cdef public BSPNode in_node  # 在几何体切面内

    def __init__(self, Plane plane):
        self.plane = plane
        self.triangles = []
        self.out_node = None
        self.in_node = None

def split_triangle_by_plane(Triangle triangle, Plane plane, float error = STATIC_ERROR):
    """
    用平面切割三角形,最终得到多个碎三角形,依次分别表示在平面外，平面内，平面上的三角形（法向量方向）

    是按照右手定则写的提取规则,提取后的三角形满足右手定则

    通过初步的测试,如果传入的三角形不满足右手定则,也能正确分割,但是分割后的三角形将保持传入三角形的点顺序（顺时针or 逆时针）

    :param triangle:
    :param plane:
    :param error:
    :return:
    """
    cdef Triangle triangle_current, triangle_append
    cdef int n, i, out_vertices_length, other_vertices_length
    cdef np.ndarray vertice, vertice_mid
    cdef float check_status, vertice_check, cos_value

    check_triangles = [triangle]  # 形如广度搜索的方式分割方式
    out_triangles = []
    in_triangles = []
    on_same = []
    on_diff = []

    while check_triangles:
        triangle_current = check_triangles.pop()
        vertices_on_triangle = o3d.utility.Vector3dVector()  # 定义在平面上的三角形
        out_vertices_length = len(triangle_current.Vertices)

        for i in range(out_vertices_length):
            vertice = triangle_current.Vertices[i]
            check_status = plane.check_in(vertice)

            vertices_new_use_angle = o3d.utility.Vector3dVector()
            vertices_new_use_angle.append(vertice)

            other_vertices = o3d.utility.Vector3dVector()

            if check_status == 0:  # 误差范围内的
                vertices_on_triangle.append(vertice)
            else:
                for n in range(i + 1, i + 3):
                    # 需要有一个三角行了
                    vertice_mid = triangle_current.Vertices[n % 3]  # 循环取点,保证右手定则

                    vertice_check = plane.check_in(vertice_mid)

                    if (vertice_check >= 0 and check_status > 0) or (
                            vertice_check <= 0 and check_status < 0):  # 确保同侧
                        vertices_new_use_angle.append(vertice_mid)

                    else:  # 异侧,需要求交点
                        other_vertices.append(vertice_mid)
                        vertices_new_use_angle.append(plane.intersection(vertices_new_use_angle[0], vertice_mid))

                other_vertices_length = len(other_vertices)

                if other_vertices_length > 0:
                    other_vertices.append(vertices_new_use_angle[2])
                    if other_vertices_length == 1:  # 异侧点一个,需要继续处理,长度未获取更新后的
                        other_vertices.append(vertices_new_use_angle[1])
                    else:
                        vertices_mid_ = o3d.utility.Vector3dVector()
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
            cos_value = triangle_append.Normal.dot(plane.Normal) / (
                    np.linalg.norm(triangle_append.Normal) * np.linalg.norm(plane.Normal))

            if abs(1 - cos_value) < error:
                on_same.append(triangle_append)
            else:
                on_diff.append(triangle_append)
    return out_triangles, in_triangles, on_same, on_diff

cpdef to_triangle_mesh(iteral, float voxel_size=0.0001):
    cdef Triangle angle
    cdef int index
    cdef np.ndarray triangle_index_np

    mesh = o3d.geometry.TriangleMesh()

    for angle in iteral:
        triangle_index = []
        for i in range(3):
            mesh.vertices.append(angle.Vertices[i])
            index = len(mesh.vertices) - 1

            triangle_index.append(index)

        triangle_index_np = np.asarray(triangle_index, dtype=np.int32)

        mesh.triangles.append(triangle_index_np)
        mesh.triangle_normals.append(angle.Normal)

    # 使用自带的库剔除重复的顶点
    # 此处需要注意精度保留
    # TODO 是否需要自己去实现
    mesh = mesh.simplify_vertex_clustering(voxel_size=voxel_size,
                                           contraction=o3d.geometry.SimplificationContraction.Average)
    return mesh

cdef class BSPTree:
    cdef public BSPNode head

    def __init__(self):
        self.head = None

    @classmethod
    def create_from_triangle_mesh(cls, mesh: o3d.open3d_pybind.geometry.TriangleMesh, error: float = None) -> "BSPTree":
        """
        尝试基于一个TriangleMesh 生成一个BSP 树
        :param mesh:
        :return:
        """

        cdef BSPTree tree
        cdef Triangle triangle
        cdef int triangle_index,triangles_length


        if error is None:
            error = STATIC_ERROR

        make_normals(mesh)
        tree = BSPTree()
        triangles_length = len(mesh.triangles)
        for triangle_index in range(triangles_length):
            singe_triangle = mesh.triangles[triangle_index]
            # 存放三角形的点
            vertices = o3d.utility.Vector3dVector()
            for i in range(3):
                vertices.append(mesh.vertices[singe_triangle[i]])

            triangle = Triangle(vertices, mesh.triangle_normals[triangle_index])
            # plane_mid = Plane(triangle.Vertices[0], normal=triangle.Normal, error=error)
            if tree.head is None:
                tree.head = BSPNode(Plane(triangle.Vertices[0], triangle.Normal))

            task_queue = [(tree.head, triangle)]  # 类似于广度搜索,将各个三角形分配下去

            while task_queue:
                node_mid, triangle_mid_use = task_queue.pop()

                out_triangles, in_triangles, on_same, on_diff = split_triangle_by_plane(triangle_mid_use,
                                                                                        node_mid.plane)

                # 以extend 替代 for 循环添加
                node_mid.triangles.extend(on_same)  # 这里的处理是正常的
                node_mid.triangles.extend(on_diff)

                if len(out_triangles) > 0:
                    if node_mid.out_node is None:
                        node_mid.out_node = BSPNode(
                            Plane(out_triangles[0].Vertices[0], out_triangles[0].Normal, error=error))

                    task_queue.extend(zip([node_mid.out_node for _ in range(len(out_triangles))], out_triangles))

                if len(in_triangles) > 0:
                    if node_mid.in_node is None:
                        node_mid.in_node = BSPNode(
                            Plane(in_triangles[0].Vertices[0], in_triangles[0].Normal, error=error))

                    task_queue.extend(zip([node_mid.in_node for _ in range(len(in_triangles))], in_triangles))
        return tree

    # def iter_triangles(self):
    #     """
    #     以迭代器方式返回所有的三角面
    #     :return:
    #     """
    #     node_cache: List[BSPNode] = [self.head]
    #
    #     while node_cache:
    #         node_use: BSPNode = node_cache.pop()
    #         for triangle in node_use.triangles:
    #             yield triangle
    #
    #         if node_use.out_node:
    #             node_cache.append(node_use.out_node)
    #         if node_use.in_node:
    #             node_cache.append(node_use.in_node)

    # def to_triangle_mesh(self,error: float = 1e-10) -> o3d.open3d_pybind.geometry.TriangleMesh:
    #     """
    #     将BSP 树转换成open3d特定格式
    #     :param tree:
    #     :return:
    #     """
    #
    #     logger.error("未完善BSP 导出功能,关键点在于几何体布尔运算最后的重复点计算部分")
    #     return to_triangle_mesh(self.iter_triangles,error)
