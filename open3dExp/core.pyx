# cython: language_level=3
"""

扩展库的核心部分,试图通过cython 进行速度上的提升

部分位置无法完全采用cython 语法:numpy 内部float 和 double 的自动转换

author:LanHao

"""
import random
import logging
from itertools import repeat

import open3d as o3d
import numpy as np
cimport numpy as np

np.import_array()

ctypedef np.int_t DTYPE_t
ctypedef np.float_t DTYPE_f

cpdef public float STATIC_ERROR = 1e-10  # 精度问题始终存在,所以这里进行了一个取舍

# 计算两个向量的夹角cos值
cpdef float cos_with_vectors(np.ndarray  vector1, np.ndarray  vector2):
    cdef float cos_value = vector1.dot(vector2) / (np.linalg.norm(vector1) * np.linalg.norm(vector2))
    return cos_value

cdef class Triangle:  # 数据存储以np.ndarray
    cdef public np.ndarray Normal
    cdef public np.ndarray Vertices

    def __init__(self, np.ndarray  points, np.ndarray  normal):
        self.Vertices = points
        self.Normal = normal

    # 寻找中心点,通过np mean 函数取均值
    def center(self):
        return np.mean(self.Vertices, axis=0)

    def __str__(self):
        return f"Vertices:{self.Vertices},Normal:{self.Normal}"

    def __repr__(self):
        return f"Triangle:{self.__str__()}"

cdef class Line:
    """
    定义一条直线
    """
    cdef public np.ndarray Origin
    cdef public np.ndarray Vector

    def __init__(self, np.ndarray origin, np.ndarray vector):
        self.Origin = origin
        self.Vector = vector/np.linalg.norm(vector) # 单位向量

    @classmethod  # 通过线条上两个点确定一条直线
    def create_by_points(cls, np.ndarray start, np.ndarray  end):
        return cls(start, end - start)

    @classmethod  # 通过经过点,线条方向向量确定直线
    def create_by_origin(cls, np.ndarray  origin, np.ndarray  vector):
        return cls(origin, vector)

cdef class Plane:
    # 点法式 定义平面
    cdef public  np.ndarray Normal
    cdef public  np.ndarray Origin
    cdef float NormalLength

    def __init__(self, np.ndarray normal, np.ndarray origin):
        self.Normal = normal
        self.Origin = origin
        self.NormalLength = np.linalg.norm(self.Normal)

    @classmethod
    def create_by_origin(cls, np.ndarray origin, np.ndarray normal):
        return cls(normal, origin)

    @classmethod
    def create_by_triangle(cls, Triangle triangle):
        return cls(triangle.Normal, triangle.Vertices[0])

    # 判断一个点与平面的相对位置
    cpdef float distance(self, np.ndarray  vertex):
        cdef float value = np.matmul(self.Normal, (vertex - self.Origin).T) / self.NormalLength
        return [0, value][abs(value) > STATIC_ERROR]  # 误差范围内的点,认为在平面内部

    # 计算一个点到平面的投影点
    cpdef np.ndarray  project(self, np.ndarray  vertex):

        cdef np.ndarray  v1 = self.Origin - vertex

        if np.linalg.norm(v1) == 0:  # 向量长度
            return self.Origin  # 平面中心点/外部点vertex 即垂点

        return vertex + self.Normal * (v1.dot(self.Normal) / np.linalg.norm(self.Normal))  # 通过向量计算投影点

    # 计算一条直线和平面的交点
    cpdef np.ndarray  intersect(self, Line line):

        cdef float cos_btn_line_and_normal = cos_with_vectors(line.Vector, self.Normal)
        cdef np.ndarray  vertex_back, v_project

        if cos_btn_line_and_normal == 0:  # 垂直
            raise Exception(f"直线和平面平行,无法计算交点cos value:{cos_btn_line_and_normal}")

        if self.distance(line.Origin) == 0:  # 此时认为,小于一定误差时：点在平面上
            vertex_back = line.Origin
        else:
            # 通过减少中间的重复计算提高了精度,此处计算出来的点应该能确保在平面上
            v_project = self.project(line.Origin) - line.Origin  # 垂点到外部点的向量
            vertex_back = line.Origin + line.Vector * np.linalg.norm(v_project) * np.linalg.norm(
                v_project) / line.Vector.dot(v_project)

        return vertex_back

    # 平面分割三角形
    cpdef tuple[list] split_triangle(self, Triangle triangle, list out_triangles=None, list in_triangles=None,
                                     list on_same=None, list on_diff=None):

        cdef:
            list out_vertices = []
            list on_vertices = []
            list in_vertices = []
            list check_status = []

            np.ndarray  vertex, vertex_next

            int index_i, index_i_next

            float check_tmp, cos_value
        # 提供基于已有的列表做后续添加,处理一些特定的情况中,列表拼接额外费时<;利用python 列表的引用传递

        if out_triangles is None:
            out_triangles = []
        if in_triangles is None:
            in_triangles = []
        if on_same is None:
            on_same = []
        if on_diff is None:
            on_diff = []

        # 缓存所有点和平面的位置关系
        for index_i in range(len(triangle.Vertices)):
            vertex = triangle.Vertices[index_i]
            check_value = self.distance(vertex)
            # check_value = [0, check_value][abs(check_value) > STATIC_ERROR]  # 处理精度取舍
            check_status.append(check_value)

        for index_i in range(len(triangle.Vertices)):

            index_i_next = (index_i + 1) % 3  # 特定除以3

            vertex = triangle.Vertices[index_i]  # 不用除以3
            vertex_next = triangle.Vertices[index_i_next]

            # 处理各个点
            if check_status[index_i] == 0:
                on_vertices.append(vertex)
                # 需要处理添加中间位置
                out_vertices.append(vertex)
                in_vertices.append(vertex)
            elif check_status[index_i] > 0:
                out_vertices.append(vertex)
            else:
                in_vertices.append(vertex)

            check_tmp = check_status[index_i] * check_status[index_i_next]

            if check_tmp < 0:  # 不同向
                try:
                    vertex = self.intersect(Line.create_by_points(vertex, vertex_next)) # TODO 预估,计算线于直线的交点计算错误
                except Exception as e:
                    logging.error(f"{e}")
                    logging.error(check_status)
                    raise Exception(f"同侧计算错误{index_i}，{index_i_next}")
                out_vertices.append(vertex)
                on_vertices.append(vertex)
                in_vertices.append(vertex)
        if len(on_vertices) == 3:
            # 额外计算法向量是否相同
            cos_value = cos_with_vectors(self.Normal, triangle.Normal)
            if cos_value > 0:
                on_same.append(triangle)
            else:
                on_diff.append(triangle)
        else:
            # TODO 分割后的三角形为何不能使用原本三角形的法向量？并且差异较大
            self.get_triangles(out_vertices, triangle.Normal, out_triangles)
            self.get_triangles(in_vertices, triangle.Normal, in_triangles)
        return out_triangles, in_triangles, on_same, on_diff

    # 满足右手定则的顺序点，返回三角形
    cdef list get_triangles(self, list vertices, np.ndarray  normal, list triangles_back=None):
        cdef:
            list use_vertices
            np.ndarray  vertex_j
            int vertices_n = len(vertices)
            int i, j
            Triangle triangle
        if triangles_back is None:
            triangles_back = []

        assert vertices_n < 5, Exception("三角形在分割时,被分割出来的部分,不可能存在超过5个点")

        if vertices_n < 3:
            return triangles_back

        for i in range(0, vertices_n - 1, 2):
            use_vertices = []
            for j in range(3):
                vertex_j = vertices[(i + j) % vertices_n]
                use_vertices.append(vertex_j)
            triangle = Triangle(np.asarray(use_vertices, dtype=np.float64), normal)
            # logging.debug(f"重新定义三角形时:{triangle}")
            triangles_back.append(triangle)
        return triangles_back

    def __str__(self):
        return f"Origin:{self.Origin},Normal:{self.Normal}"

    def __repr__(self):
        return self.__str__()

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

def to_triangle_mesh(iteral):
    """
    将任何可迭代对象(迭代出来的为一个Triangle)转换为open3d 中的TriangleMesh 对象,期间会进行重复点的剔除工作

    这个过程,从目前的效果来看,本身耗时并不多

    :param iteral:
    :return:
    """
    cdef:
        list triangle_index
        Triangle triangle
        int index
        np.ndarray triangle_index_np
        np.ndarray select


    mesh = o3d.geometry.TriangleMesh()

    for triangle in iteral:
        triangle_index = []
        for i in range(3):
            select = np.where((mesh.vertices == triangle.Vertices[i]).all(1))[0]
            if len(select) > 0:
                index = select[0]
            else:
                mesh.vertices.append(triangle.Vertices[i])
                index = len(mesh.vertices) - 1

            triangle_index.append(index)

        triangle_index_np = np.asarray(triangle_index, dtype=np.int32)

        mesh.triangles.append(triangle_index_np)
        mesh.triangle_normals.append(triangle.Normal)
    return mesh

cdef class SplitTask:
    """
    被定义用来表示一个任务
    """

    cdef public Triangle triangle
    cdef public BSPNode node
    cdef public int status

    def __init__(self, Triangle triangle, BSPNode node, int status):
        self.triangle = triangle
        self.node = node
        self.status = status

cdef class BSPTree:
    cdef public BSPNode head

    def __init__(self):
        self.head = None

    def traverse(self):
        """
        因为采取的是列表,从返回的数据上，应该不能归属于广度or深度。而是简单的归属于先序遍历问题
        :return:
        """
        task_queue = [self.head]

        while task_queue:
            node = task_queue.pop()
            for triangle in node.triangles:
                yield triangle
            if node.in_node is not None:
                task_queue.append(node.in_node)
            if node.out_node is not None:
                task_queue.append(node.out_node)

    def check_in(self, np.ndarray vertex):
        """
        判断一个点在几何体内部还是外部还是表面
        :param vertex:
        :return: -1 内部,0 表面,1 外部
        """

        node = self.head
        had_surface = False

        while node:
            check_value = node.plane.distance(vertex)
            if check_value > 0 and node.out_node is None:
                return 1
            elif check_value > 0 and node.out_node is not None:
                node = node.out_node
                continue

            if check_value < 0 and node.in_node is None:
                if had_surface:
                    return 0  # 表面
                else:
                    return -1  # 内部
            elif check_value < 0 and node.in_node is not None:
                node = node.in_node
                continue

            if check_value == 0:
                # 可能在表面
                had_surface = True
                if node.in_node is not None:
                    node = node.in_node
                elif node.out_node is not None:
                    node = node.out_node
                else:
                    return 0

    # 以一个BSP 树分割一个三角形面片,分割的方式出现错误
    cpdef tuple[list] split_triangle(self, Triangle triangle, list out_triangles=None, list in_triangles=None,
                                     list on_same=None, list on_diff=None):
        cdef:
            list task_queue = [SplitTask(triangle, self.head, 0)]
        if out_triangles is None:
            out_triangles = []
        if in_triangles is None:
            in_triangles = []
        if on_same is None:
            on_same = []
        if on_diff is None:
            on_diff = []

        while task_queue:  # 每一个三角形,都需要从tree 的顶部开始计算,分割出需要处理的各个子三角形
            # surface_status_use 0 表示没有处理,
            task_use = task_queue.pop()  # 此前计算的相关信息
            # logging.debug(f"node plane:{task_use.node.plane}")
            out_mid, in_mid, on_same_mid, on_diff_mid = task_use.node.plane.split_triangle(task_use.triangle)

            if task_use.node.out_node is None:
                if out_mid:
                    out_triangles.append(out_mid)  # 通过plane 切割,确定在几何体外部
            else:
                # 可能在内部,也可能在内部,要继续讨论
                add_task_queue(task_queue, task_use.node.out_node, task_use.status, out_mid)
                # 后添加
                add_task_queue(task_queue, task_use.node.out_node, 1, on_same_mid)
                add_task_queue(task_queue, task_use.node.out_node, 2, on_diff_mid)


            if task_use.node.in_node is None:
                if len(in_mid)>0:
                    # 剔除曾经在表面然后现在又在内部的三角面
                    if task_use.status == 0:  # 没有出现过,在几何体表现的情况
                        in_triangles.append(in_mid)
                    elif task_use.status == 1:  # 同向
                        on_same.append(in_mid)
                    else:  # 异向
                        on_diff.append(in_mid)
                 # len(on_same_mid) > 0 and on_same.append(on_same_mid)
                # len(on_diff_mid) > 0 and on_diff.append(on_diff_mid)
            else:  # 继续处理
                # 仍然在内部的三角面
                add_task_queue(task_queue, task_use.node.in_node, task_use.status, in_mid)
                # 记录
                add_task_queue(task_queue, task_use.node.in_node, 1, on_same_mid)
                add_task_queue(task_queue, task_use.node.in_node, 2, on_diff_mid)


            if len(on_same_mid)>0:
                if task_use.node.out_node is None and task_use.node.in_node is None: # 已经无法继续进行了
                    on_same.append(on_same_mid)
            # # 现在不再是on_same_mid 就可以在平面内外部
            # len(on_same_mid) > 0 and on_same.append(on_same_mid)
            # len(on_diff_mid) > 0 and on_diff.append(on_diff_mid)

        return out_triangles, in_triangles, on_same, on_diff

    # 通过triangleMesh 生成BSP 树
    @classmethod
    def create_with_triangle_mesh(cls,mesh):

        cdef:
            BSPTree tree  # 返回的bsp树
            BSPNode node = None  # 节点信息
            Plane plane  # 平面
            Triangle triangle  # 单个三角形面片
            list triangles, task_queue, out_triangles, in_triangles, on_same, on_diff, out_triangles_i, in_triangles_i, on_same_i, on_diff_i

        task_queue = []
        tree = cls()
        triangles = get_triangles_with_mesh(mesh)  # 初始化,用于存放三角形

        # 初始化一个节点,需要初始化一个平面
        node = get_node_by_triangles(triangles)

        if node is None:
            return tree
        else:
            tree.head = node

        task_queue.append((tree.head, triangles))

        while task_queue:
            node, triangles = task_queue.pop()  # 取出当前需要计算的结果
            out_triangles = []
            in_triangles = []
            for triangle in triangles:
                node.plane.split_triangle(triangle, out_triangles, in_triangles, node.triangles, node.triangles)

            if len(out_triangles) > 0:  # 需要处理在平面外侧的数据
                if node.out_node is None:
                    node.out_node = get_node_by_triangles(out_triangles)
                task_queue.append((node.out_node, out_triangles))

            if len(in_triangles) > 0:
                if node.in_node is None:
                    node.in_node = get_node_by_triangles(in_triangles)
                task_queue.append((node.in_node, in_triangles))

        return tree

    cpdef split_triangle_mesh(self,mesh: o3d.open3d_pybind.geometry.TriangleMesh):
        cdef:
            list task_queue, out_triangles, in_triangles, on_same_triangles, on_diff_triangles
            int angle_index
            Triangle  triangle_mid
            np.ndarray  vertices, mesh_angle

            SplitTask task_use

        # 嵌套列表,减少中间过程中的重组
        out_triangles = []
        in_triangles = []
        on_same_triangles = []
        on_diff_triangles = []

        for angle_index, mesh_angle in enumerate(mesh.triangles):

            vertices = np.zeros(shape=(0, 3), dtype=np.float64)
            for i in range(3):
                vertices = np.append(vertices, [mesh.vertices[mesh_angle[i]]], axis=0)

            # 构造了三角形Triangle

            triangle_mid = Triangle(vertices, mesh.triangle_normals[angle_index])
            self.split_triangle(triangle_mid, out_triangles, in_triangles, on_same_triangles, on_diff_triangles)

        return out_triangles, in_triangles, on_same_triangles, on_diff_triangles

cdef Plane get_plane_by_pca(np.ndarray  points):
    # 通过pca 求到拟合的平面,两边点的数量几乎类似
    cdef:
        Plane plane
        np.ndarray average, avgs, data_adjust, feat_vec, cov_x, feat_value, normal, index
        int m, n

    average = np.mean(points, axis=0)  # 中心点
    m, n = np.shape(points)  # N x 3
    avgs = np.tile(average, (m, 1))
    data_adjust = points - avgs
    cov_x = np.cov(data_adjust.T)  # 计算协方差矩阵
    try:
        feat_value, feat_vec = np.linalg.eig(cov_x)  # 求解协方差矩阵的特征值和特征向量
        index = np.argsort(-feat_value)  # 依照featValue进行从大到小排序
        normal = np.cross(feat_vec[index[0]], feat_vec[index[1]])  # 中途发生了错误,关于精度的？
        plane = Plane.create_by_origin(average, normal)
    except Exception as e:
        logging.error(e)
        logging.error(cov_x)
        raise Exception("PCA 拟合平面时发生错误")
    return plane

# 通过众多三角形面片，尝试获得一个平面来分割所有的三角形
cdef Plane get_plane_try_pca(list triangles, int max_triangles=5, float check_value=0.5):
    cdef:
        list out_triangles_i, in_triangles_i, on_same_i, on_diff_i
        Plane plane = None
        np.ndarray  vertices
        Triangle triangle
        int triangles_n_total

        int out_n = 0  # 平面外
        int in_n = 0  # 平面内
        int on_n = 0  # 被分割的三角形
        int some_n = 0  # 共面
        float mid_n
        float quality

    triangles_n_total = len(triangles)
    if triangles_n_total < max_triangles:
        return Plane.create_by_origin(triangles[0].Vertices[0], triangles[0].Normal)

    triangles = random.sample(triangles, max_triangles)
    vertices = np.asarray([triangle.center() for triangle in triangles], dtype=np.float64)

    plane = get_plane_by_pca(vertices)
    for triangle in triangles:
        out_triangles_i, in_triangles_i, on_same_i, on_diff_i = plane.split_triangle(triangle)
        if len(out_triangles_i) > 0 and len(in_triangles_i) > 0:  # 点在异侧,说明这个三角形被分割开了,splited
            on_n += 1
        elif len(out_triangles_i) > 0:  # 同侧,不过三角形和平面相连
            out_n += 1
        elif len(in_triangles_i) > 0:  # 同侧,不过三角形和平面相连
            in_n += 1
        else:  # 都在平面上
            some_n += 1
    mid_n = (out_n + in_n) / 2

    # 这个值越大,说明两侧越均匀,中间被分割的三角形越少,
    quality = 1 - (on_n / max_triangles) - (out_n - mid_n) ** 2 - (in_n - mid_n) ** 2

    if quality < check_value:  # 说明此时选取的平面集,不适合用pca 做平面分割
        plane = Plane.create_by_origin(triangles[0].Vertices[0], triangles[0].Normal)
    return plane

# 根据三角形集,选择优化后的平面生成Node
cdef BSPNode get_node_by_triangles(list triangles):
    cdef:
        BSPNode node = None
        Plane plane

    plane = get_plane_try_pca(triangles)  #
    node = BSPNode(plane)
    return node

cdef list get_triangles_with_mesh(mesh):
    cdef:
        np.ndarray singe_triangle, vertices
        list triangles
        int triangle_index
        Triangle triangle

    triangles = []  # 初始化,用于存放三角形

    for triangle_index in range(len(mesh.triangles)):  # 遍历三角形
        singe_triangle = mesh.triangles[triangle_index]
        # 存放三角形的点
        vertices = np.zeros(shape=(0, 3), dtype=np.float64)
        for i in range(3):
            vertices = np.append(vertices, [mesh.vertices[singe_triangle[i]]], axis=0)
        triangle = Triangle(vertices, mesh.triangle_normals[triangle_index])
        triangles.append(triangle)  # 会总处理

    return triangles



cdef add_task_queue(list task_queue, BSPNode node, int status, list out_triangles):
    cdef Triangle triangle

    for triangle in out_triangles:
        task_queue.append(SplitTask(triangle, node, status))


