# cython: language_level=3
from enum import Enum
import random
import logging
from itertools import repeat
from datetime import datetime

cimport numpy as np
import numpy as np

np.import_array()

import open3d as o3d

np.import_array()

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
    def center(self):
        return np.mean(self.Vertices, axis=0)
    # cdef public np.ndarray center(self):
    #     return np.mean(self.Vertices, axis=0)

cdef class Plane:
    # 点法式 定义平面
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

    cdef np.ndarray[DTYPE_f, ndim=1] project(self, np.ndarray[DTYPE_f, ndim=1] vertice):

        cdef np.ndarray[DTYPE_f, ndim=1] v1 = self.Vertice - vertice

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
    # BSP 树的节点信息
    cdef public Plane plane
    cdef public list triangles  # 落在切面上的三角形
    cdef public BSPNode out_node  # 在几何体切面外
    cdef public BSPNode in_node  # 在几何体切面内

    def __init__(self, Plane plane):
        self.plane = plane
        self.triangles = []
        self.out_node = None
        self.in_node = None

# 用一个平面分割三角形，不应该出现不断分割的情况？是为何出现不断分割的情况的呢？
cdef tuple[list] split_triangle_by_plane(Triangle triangle, Plane plane, float error = STATIC_ERROR):
    # 分割
    cdef Triangle triangle_current, triangle_append
    cdef int n, i, out_vertices_length, other_vertices_length
    cdef np.ndarray vertice, vertice_mid
    cdef float check_status, vertice_check, cos_value

    cdef np.ndarray[DTYPE_f, ndim=2] vertices_on_triangle, vertices_new_use_angle, other_vertices

    cdef list check_triangles = [triangle]  # 形如广度搜索的方式分割方式
    cdef list out_triangles = []
    cdef list in_triangles = []
    cdef list on_same = []
    cdef list on_diff = []

    while check_triangles:
        logging.debug(f"check_triangles loop：{len(check_triangles)}")
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
            else:
                for n in range(i + 1, i + 3):
                    # 需要有一个三角行了
                    vertice_mid = triangle_current.Vertices[n % 3]  # 循环取点,保证右手定则

                    vertice_check = plane.check_in(vertice_mid)

                    if (vertice_check >= 0 and check_status > 0) or (
                            vertice_check <= 0 and check_status < 0):  # 确保同侧
                        vertices_new_use_angle = np.append(vertices_new_use_angle, [vertice_mid], axis=0)

                    else:  # 异侧,需要求交点
                        other_vertices = np.append(other_vertices, [vertice_mid], axis=0)
                        vertices_new_use_angle = np.append(vertices_new_use_angle,
                                                           [plane.intersection(vertices_new_use_angle[0], vertice_mid)],
                                                           axis=0)

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

cdef object to_triangle_mesh(iteral, float voxel_size=0.0001):
    cdef Triangle angle
    cdef int index
    cdef np.ndarray triangle_index_np
    cdef np.ndarray select
    mesh = o3d.geometry.TriangleMesh()

    for angle in iteral:
        triangle_index = []
        for i in range(3):
            select = np.where((mesh.vertices == angle.Vertices[i]).all(1))[0]
            if len(select) > 0:
                index = select[0]
            else:
                mesh.vertices.append(angle.Vertices[i])
                index = len(mesh.vertices) - 1

            triangle_index.append(index)

        triangle_index_np = np.asarray(triangle_index, dtype=np.int32)

        mesh.triangles.append(triangle_index_np)
        mesh.triangle_normals.append(angle.Normal)
    return mesh

cdef class BSPTree:
    cdef public BSPNode head

    def __init__(self):
        self.head = None

cdef Plane get_plane_by_pca(np.ndarray[DTYPE_f, ndim=2] points):
    # 通过pca 求到拟合的平面,两边点的数量几乎类似,此处方法应该是正确的
    cdef Plane plane
    cdef np.ndarray[DTYPE_f, ndim=2] averages, avgs, data_adjust, featVec
    cdef np.ndarray[DTYPE_f, ndim=1] featValue, v1, v2, normal
    cdef np.ndarray[long long, ndim=1] index
    cdef int m, n

    average = np.mean(points, axis=0)  # 中心点
    m, n = np.shape(points)  # N x 3
    avgs = np.tile(average, (m, 1))
    data_adjust = points - avgs
    cov_x = np.cov(data_adjust.T)  # 计算协方差矩阵

    featValue, featVec = np.linalg.eig(cov_x)  # 求解协方差矩阵的特征值和特征向量
    index = np.argsort(-featValue)  # 依照featValue进行从大到小排序
    v1 = featVec[index[0]]
    v2 = featVec[index[1]]
    normal = np.cross(v1, v2)
    plane = Plane(average, normal)
    return plane

# 通过众多三角形面片，尝试获得一个平面来分割所有的三角形
cdef Plane get_plane_try_pca(list triangles):
    cdef list out_triangles_i, in_triangles_i, on_same_i, on_diff_i
    cdef Plane plane = None
    cdef np.ndarray[DTYPE_f,ndim=2] vertices
    cdef Triangle triangle
    cdef int triangles_n_random_choose
    cdef int out_n = 0
    cdef int in_n = 0
    cdef int on_n = 0

    logging.debug(f"进入 get_plane_try_pca")
    triangles_n_random_choose = len(triangles)//3
    if triangles_n_random_choose<10:
        triangles_n_random_choose = len(triangles)
    triangles = random.sample(triangles,triangles_n_random_choose)
    vertices = np.asarray([triangle.center() for triangle in triangles],dtype=np.float64)

    logging.debug(f"结束中心点计算：{len(vertices)}")
    if len(vertices)<3:
        return Plane(triangles[0].center(),triangles[0].Normal)

    plane = get_plane_by_pca(vertices)

    logging.debug(f"结束plane初次计算，triangles :{triangles_n_random_choose}  {len(triangles)}")
    for triangle in triangles:
        out_triangles_i, in_triangles_i, on_same_i, on_diff_i = split_triangle_by_plane(triangle,plane)
        logging.debug(f"out_i:{out_triangles_i},in_i:{len(in_triangles_i)},on_i:{len(on_same_i)+len(on_diff_i)}")
        out_n+=len(out_triangles_i)
        in_n+=len(in_triangles_i)
        on_n+=len(on_diff_i)
        on_n+=len(on_same_i)
    logging.debug(f"结束循环")
    if ((out_n+in_n+on_n)-len(triangles))/2 > len(triangles):# 不适合用pca 做平面提取
        logging.debug(f"因为被分割的平面数占比太多，放弃pca 获取平面")
        plane = Plane(triangles[0].center(),triangles[0].Normal)
    logging.debug(f"end get_plane_try_pca")
    return plane

cdef BSPNode get_by_triangles(list triangles):
    # 生成Node
    cdef BSPNode node = None
    cdef Plane plane
    cdef np.ndarray[DTYPE_f, ndim=2] vertices
    plane = get_plane_try_pca(triangles)
    node = BSPNode(plane)

    # vertices = np.asarray([triangle.center() for triangle in triangles],
    #                                                        dtype=np.float64)
    # if len(triangles) > 50:  # 经验数据
    #     plane = get_plane_by_pca(np.asarray(vertices))
    #     node = BSPNode(plane)
    #
    # elif len(triangles) > 0:
    #     plane = Plane(triangles[0].Vertices[0], triangles[0].Normal)
    #     node = BSPNode(plane)
    return node

cdef BSPTree create_from_triangle_mesh(mesh: o3d.open3d_pybind.geometry.TriangleMesh, float error=STATIC_ERROR):
    # 尝试基于PCA 做启发式算法的优化
    cdef BSPTree tree  # 返回的bsp树
    cdef BSPNode node = None  # 节点信息
    cdef Plane plane  # 平面

    cdef Triangle triangle  # 单个三角形面片
    cdef list triangles  # 存放所有的三角形面片
    cdef list task_queue, out_triangles, in_triangles, on_same, on_diff, out_triangles_i, in_triangles_i, on_same_i, on_diff_i  # 广度优先的插入方式时的任务队列
    cdef int triangle_index  # 第几个三角形

    task_queue = []
    tree = BSPTree()
    triangles = []  # 初始化,用于存放三角形

    for triangle_index in range(len(mesh.triangles)):  # 遍历三角形
        singe_triangle = mesh.triangles[triangle_index]
        # 存放三角形的点
        vertices = np.zeros(shape=(0, 3), dtype=np.float64)
        for i in range(3):
            vertices = np.append(vertices, [mesh.vertices[singe_triangle[i]]], axis=0)

        triangle = Triangle(vertices, mesh.triangle_normals[triangle_index])
        triangles.append(triangle)  # 会总处理

    # 初始化一个节点,需要初始化一个平面

    node = get_by_triangles(triangles)

    if node is None:
        return tree
    else:
        tree.head = node

    task_queue.append((node, triangles))

    while task_queue:
        node, triangles = task_queue.pop()  # 取出当前需要计算的结果

        out_triangles = []
        in_triangles = []
        for triangle in triangles:
            out_triangles_i, in_triangles_i, on_same_i, on_diff_i = split_triangle_by_plane(triangle, node.plane)
            out_triangles.extend(out_triangles_i)
            in_triangles.extend(in_triangles_i)
            node.triangles.extend(on_diff_i)
            node.triangles.extend(on_same_i)
        if len(out_triangles) > 0:  # 需要处理在平面外侧的数据
            if node.out_node is None:
                plane = get_plane_try_pca(out_triangles)
                node.out_node = BSPNode(plane)
            task_queue.append((node.out_node, out_triangles))
        logging.debug(f"本次需要判断的triangles:{len(triangles)},还剩余:{len(task_queue)},out:{len(out_triangles)},in:{len(in_triangles)}")
        if len(in_triangles) > 0:
            if node.in_node is None:
                plane = get_plane_try_pca(in_triangles)
                node.in_node = BSPNode(plane)
            task_queue.append((node.in_node, in_triangles))

    return tree

cdef BSPTree create_from_triangle_mesh_2(mesh: o3d.open3d_pybind.geometry.TriangleMesh, float error=STATIC_ERROR):
    cdef BSPTree tree
    cdef BSPNode node_mid
    cdef Triangle triangle
    cdef int triangle_index, triangles_length
    cdef list task_queue, out_triangles, in_triangles, on_same, on_diff
    cdef np.ndarray[DTYPE_t, ndim=1] singe_triangle
    cdef np.ndarray[DTYPE_f, ndim=2] vertices

    tree = BSPTree()
    triangles_length = len(mesh.triangles)

    for triangle_index in range(triangles_length):
        singe_triangle = mesh.triangles[triangle_index]
        # 存放三角形的点
        vertices = np.zeros(shape=(0, 3), dtype=np.float64)
        for i in range(3):
            vertices = np.append(vertices, [mesh.vertices[singe_triangle[i]]], axis=0)

        triangle = Triangle(vertices, mesh.triangle_normals[triangle_index])
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

cdef tuple[list] split_triangle_mesh(mesh: o3d.open3d_pybind.geometry.TriangleMesh, BSPTree tree,
                                     float error = STATIC_ERROR):
    # 分割mesh
    cdef BSPNode node_use
    cdef list task_queue, out_triangles, in_triangles, on_same_triangles, on_diff_triangles, out_mid, in_mid, on_same_mid, on_diff_mid
    cdef int angle_index, surface_status_use
    cdef Triangle  triangle_mid, triangle_use
    cdef np.ndarray[DTYPE_f, ndim=2] vertices
    cdef np.ndarray[DTYPE_t, ndim=1] mesh_angle

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
        cdef np.ndarray[DTYPE_f, ndim=2] vertices
        cdef BSPTree geom2_tree, geom1_tree
        cdef list triangles_all, out_triangles, in_triangles, on_same_triangles, on_diff_triangles, out_triangles2, in_triangles2, on_same_triangles2, on_diff_triangles2, triangle_in_2, new_single_triangle
        logging.debug(f"进入几何体布尔运算状态")
        # BSP 树存储数据,广度搜索
        geom2_tree = create_from_triangle_mesh(geom2)
        logging.debug(f"结束第一个几何体bsp 树生成")
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
