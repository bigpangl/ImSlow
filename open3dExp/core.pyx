# cython: language_level=3
from enum import Enum
import logging
from itertools import repeat
from datetime import datetime

import numpy as np
cimport numpy as np
from cpython.ref cimport PyObject
# from libcpp.vector cimport vector

import open3d as o3d

ctypedef np.int_t DTYPE_t
ctypedef np.float_t DTYPE_f

cpdef public float STATIC_ERROR = 1e-10


class BooleanOperation(Enum):
    Union = 1  # 合并
    Difference = 2  # 不同
    Intersect = 3  # 相交


cdef class Triangle:  # 数据存储以np.ndarray
    cdef public np.ndarray Normal
    cdef public np.ndarray Vertices
    def __cinit__(self, np.ndarray[DTYPE_f, ndim=2] points, np.ndarray[DTYPE_f, ndim=1] normal):
        self.Vertices = points
        self.Normal = normal

cdef class Plane:
    """
    点法式定义平面
    """
    cdef public  np.ndarray Normal
    cdef public  np.ndarray Vertice
    cdef public float error

    def __cinit__(self, np.ndarray[DTYPE_f, ndim=1] vertice, np.ndarray[DTYPE_f, ndim=1] normal,
                  float error= STATIC_ERROR):

        self.Normal = normal
        self.Vertice = vertice
        self.error = error

    cdef float check_in(self, np.ndarray[DTYPE_f, ndim=1] vertice):
        # 这里会根据误差,进行判断
        cdef float value = np.matmul(self.Normal, (vertice - self.Vertice).T)

        return [0, value][abs(value) > self.error]

    cdef np.ndarray[DTYPE_f,ndim=1] project(self, np.ndarray[DTYPE_f,ndim=1] vertice):


        cdef np.ndarray[DTYPE_f,ndim=1] v1 = self.Vertice - vertice

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
        vertices_on_triangle = np.zeros(shape=(0, 3), dtype=np.float64)  # 定义在平面上的三角形
        out_vertices_length = len(triangle_current.Vertices)

        for i in range(out_vertices_length):
            vertice = triangle_current.Vertices[i]
            check_status = plane.check_in(vertice)

            vertices_new_use_angle = np.asarray([vertice], dtype=np.float64)
            # vertices_new_use_angle.append(vertice)

            other_vertices = np.zeros(shape=(0, 3), dtype=np.float64)

            if check_status == 0:  # 误差范围内的
                vertices_on_triangle = np.append(vertices_on_triangle, [vertice], axis=0)
                # vertices_on_triangle.append(vertice)
            else:
                for n in range(i + 1, i + 3):
                    # 需要有一个三角行了
                    vertice_mid = triangle_current.Vertices[n % 3]  # 循环取点,保证右手定则

                    vertice_check = plane.check_in(vertice_mid)

                    if (vertice_check >= 0 and check_status > 0) or (
                            vertice_check <= 0 and check_status < 0):  # 确保同侧
                        vertices_new_use_angle = np.append(vertices_new_use_angle, [vertice_mid], axis=0)
                        # vertices_new_use_angle.append(vertice_mid)

                    else:  # 异侧,需要求交点
                        other_vertices = np.append(other_vertices, [vertice_mid], axis=0)
                        # other_vertices.append(vertice_mid)
                        vertices_new_use_angle = np.append(vertices_new_use_angle,
                                                           [plane.intersection(vertices_new_use_angle[0], vertice_mid)],
                                                           axis=0)
                        # vertices_new_use_angle.append()

                other_vertices_length = len(other_vertices)

                if other_vertices_length > 0:
                    other_vertices = np.append(other_vertices, [vertices_new_use_angle[2]], axis=0)
                    if other_vertices_length == 1:  # 异侧点一个,需要继续处理,长度未获取更新后的
                        other_vertices = np.append(other_vertices, [vertices_new_use_angle[1]], axis=0)
                    else:
                        vertices_mid_ = np.asarray([
                            vertices_new_use_angle[2],
                            vertices_new_use_angle[1],
                            other_vertices[0]
                        ], dtype=np.float)
                        # vertices_mid_.append()
                        # vertices_mid_.append()
                        # vertices_mid_.append()
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

cdef BSPTree create_from_triangle_mesh(mesh: o3d.open3d_pybind.geometry.TriangleMesh, float error=STATIC_ERROR):
    cdef BSPTree tree
    cdef Triangle triangle
    cdef int triangle_index, triangles_length

    if error is None:
        error = STATIC_ERROR

    tree = BSPTree()
    triangles_length = len(mesh.triangles)

    for triangle_index in range(triangles_length):
        singe_triangle = mesh.triangles[triangle_index]
        # 存放三角形的点
        vertices = np.zeros(shape=(0, 3), dtype=np.float64)
        for i in range(3):
            vertices = np.append(vertices, [mesh.vertices[singe_triangle[i]]], axis=0)

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

cdef split_triangle_mesh(mesh: o3d.open3d_pybind.geometry.TriangleMesh, BSPTree tree, float error = STATIC_ERROR):
    # 分割mesh
    out_triangles = []
    in_triangles = []
    on_same_triangles = []
    on_diff_triangles = []

    for angle_index, mesh_angle in enumerate(mesh.triangles):

        vertices = np.zeros(shape=(0, 3), dtype=np.float64)
        for i in range(3):
            vertices = np.append(vertices, [mesh.vertices[mesh_angle[i]]], axis=0)
            # vertices.append()

        triangle_mid = Triangle(vertices, mesh.triangle_normals[angle_index])

        task_queue = [(triangle_mid, tree.head, 0)]
        while task_queue:
            triangle_use, node_use, surface_status_use = task_queue.pop()  # 此前计算的相关信息

            out_mid, in_mid, on_same_mid, on_diff_mid = split_triangle_by_plane(triangle_use, node_use.plane, error)
            if node_use.out_node is None:
                out_triangles.append(out_mid)  # 通过plane 切割,确定在几何体外部
            else:
                # 可能在内部,也可能在内部,要继续讨论
                task_queue.extend(zip(
                    out_mid,
                    repeat(node_use.out_node),
                    repeat(False)
                ))

            if node_use.in_node is None:

                if surface_status_use == 0:  # 剔除曾经在表面然后现在又在内部的三角面
                    in_triangles.append(in_mid)
                elif surface_status_use == 1:  # 同向
                    on_same_triangles.append(in_mid)
                else:  # 异向
                    on_diff_triangles.append(in_mid)

                on_same_mid and on_same_triangles.append(on_same_mid)
                on_diff_mid and on_diff_triangles.append(on_diff_mid)
            else:
                task_queue.extend(zip(
                    in_mid,
                    repeat(node_use.in_node),
                    repeat(surface_status_use)
                ))
                task_queue.extend(zip(
                    on_same_mid,
                    repeat(node_use.in_node),
                    repeat((1))
                ))
                task_queue.extend(zip(
                    on_diff_mid,
                    repeat(node_use.in_node),
                    repeat(2)
                ))

    return out_triangles, in_triangles, on_same_triangles, on_diff_triangles


class BooleanOperationUtils:
    """

    几何体布尔运算

    """

    @classmethod
    def execute_boolean_operation(cls, geom1: o3d.open3d_pybind.geometry.TriangleMesh,
                                  geom2: o3d.open3d_pybind.geometry.TriangleMesh,
                                  operation: BooleanOperation,
                                  float error= STATIC_ERROR):
        # a =  vector[int]()

        cdef BSPTree geom2_tree, geom1_tree

        # BSP 树存储数据,广度搜索
        geom2_tree = create_from_triangle_mesh(geom2)
        start_time = datetime.now()
        geom1_tree = create_from_triangle_mesh(geom1)

        end_time = datetime.now()
        logging.debug(f"球体加载耗时:{end_time - start_time}")

        triangles_all = []  # 存放所有的三角面片,多层嵌套列表Triangle

        out_triangles, in_triangles, on_same_triangles, on_diff_triangles = split_triangle_mesh(geom1, geom2_tree)

        out_triangles2, in_triangles2, on_same_triangles2, on_diff_triangles2 = split_triangle_mesh(geom2, geom1_tree)

        if operation == BooleanOperation.Union:  # 并集
            triangles_all.append(out_triangles)
            triangles_all.append(out_triangles2)
            triangles_all.append(on_same_triangles)

        if operation == BooleanOperation.Intersect:  # 相交部分
            triangles_all.append(in_triangles)
            triangles_all.append(in_triangles2)
            if len(in_triangles) != 0 and len(in_triangles2) != 0:  # 如果等于0，就未相交了
                triangles_all.append(on_same_triangles)

        if operation == BooleanOperation.Difference:  # 不同部分

            triangles_all.append(out_triangles)

            triangle_in_2 = []

            for single_triangle in in_triangles2:
                new_single_triangle = []
                for single in single_triangle:
                    vertices = np.asarray([
                        single.Vertices[2],
                        single.Vertices[1],
                        single.Vertices[0]
                    ], dtype=np.float64)
                    new_single_triangle.append(Triangle(vertices, single.Normal * -1))
                triangle_in_2.append(new_single_triangle)

            triangles_all.append(triangle_in_2)
            triangles_all.append(on_diff_triangles)

        iteral_use = (angle for list_angle1 in triangles_all for list_angle2 in list_angle1 for angle in list_angle2)
        mesh = to_triangle_mesh(iteral_use)
        return mesh
