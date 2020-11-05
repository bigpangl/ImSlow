# cython: language_level=3

"""
Author:     LanHao
Date:       2020/11/4
Python:     python3.6

vector 使用numpy基础对象

"""

import copy
import logging
import traceback
import random
import numpy as np
cimport numpy as np

np.import_array()

cdef float EPSILON = 1e-5
cdef int TRIANGLE_NUMBER = 5
cdef float PCA_CHECK_VALUE = 0.6

cdef class Plane:
    cdef public np.ndarray Normal
    cdef public float W  # 应该是一个点法式中的系数

    def __str__(self):
        return f"Normal:{self.Normal.tolist()},W:{self.W}"

    def __repr__(self):
        return f"<Plane:{id(self)}><{self.__str__()}>"

    def __init__(self, np.ndarray normal, float w):
        self.Normal = normal
        self.W = w

    @classmethod
    def from_points(cls, np.ndarray a, np.ndarray b, np.ndarray c):
        """
        生成三个点所在平面
        :param a:
        :param b:
        :param c:
        :return:
        """
        cdef:
            np.ndarray normal

        normal = np.cross(b - a, c - a)
        normal = normal / np.linalg.norm(normal)  #单位长度
        return cls(normal, normal.dot(a))

    @classmethod
    def from_origin_normal(cls, np.ndarray origin, np.ndarray normal):
        normal = normal / np.linalg.norm(normal)
        return cls(normal, normal.dot(origin))

    cpdef void flip(self):
        self.Normal *= -1  # 向量取反
        self.W *= -1  #参数 取反

    cdef Plane clone(self):  # 复制平面
        return Plane(self.Normal.copy(), self.W)  # 复制此平面,防止因为在别处翻转导致原始位置发生变化s

    # 计算一个点到平面的距离,来自于点法式,同时提前计算了W 值以及单位向量了Normal
    cpdef float distance(self, np.ndarray vertex):
        return self.Normal.dot(vertex) - self.W

    # 用一个平面分割三角形
    cpdef void split_triangle(self, Triangle triangle, list coplanarfront, list coplanarback, list front, list back):
        cdef:
            int COPLANAR = 0
            int FRONT = 1
            int BACK = 2
            int SPANNING = 3
            int triangle_type = 0
            list types_all = []
            list f_tmp, b_tmp

            np.ndarray vertice, vi, vj
            float t_tmp
            int type_tmp, i, j, ti, tj

        for vertice in triangle.Vertices:
            t_tmp = self.distance(vertice)
            type_tmp = BACK if (t_tmp < -EPSILON) else (FRONT if t_tmp > EPSILON else COPLANAR)
            triangle_type |= type_tmp
            types_all.append(type_tmp)

        if triangle_type == COPLANAR:
            (coplanarfront if self.Normal.dot(triangle.plane.Normal) > 0 else coplanarback).append(triangle)
        elif triangle_type == FRONT:
            front.append(triangle)
        elif triangle_type == BACK:
            back.append(triangle)
        elif triangle_type == SPANNING:  # 被分列开了
            f_tmp = []
            b_tmp = []
            for i in range(len(triangle.Vertices)):
                j = (i + 1) % (len(triangle.Vertices))
                ti = types_all[i]
                tj = types_all[j]

                vi = triangle.Vertices[i].copy()
                vj = triangle.Vertices[j].copy()

                # 独立的点,存放在前后
                if ti != BACK:
                    f_tmp.append(vi)
                if ti != FRONT:
                    b_tmp.append(vi.copy() if ti != BACK else vi)  # 两边都添加点?

                if (ti | tj) == SPANNING:
                    t = (self.W - self.Normal.dot(vi)) / self.Normal.dot(vj - vi)  # 求交点
                    v = vi + ((vj - vi) * t)
                    f_tmp.append(v)
                    b_tmp.append(v)

            if len(f_tmp) >= 3:
                self.get_triangles(f_tmp, front)
            if len(b_tmp) >= 3:
                self.get_triangles(b_tmp, back)

    # 满足右手定则的顺序点，返回三角形,这是自己提供的方法
    cdef list get_triangles(self, list vertices, list triangles_back=None):
        cdef:
            list use_vertices
            np.ndarray  vertex_j
            int vertices_n = len(vertices)
            int i, j
            Triangle triangle

        if triangles_back is None:
            triangles_back = []

        assert vertices_n < 5, Exception("三角形在分割时,被分割出来的部分,不可能存在超过5个点")
        assert vertices_n >= 3, Exception("不应该传入小于三个点的多边形")

        for i in range(0, vertices_n - 1, 2):
            use_vertices = []
            for j in range(3):
                vertex_j = vertices[(i + j) % vertices_n]
                use_vertices.append(vertex_j)

            triangle = Triangle(np.asarray(use_vertices, dtype=np.float64))
            triangles_back.append(triangle)

        return triangles_back

cdef class Triangle:
    """
    多边形,同时也可以是三角形// 重点指三角形
    """
    cdef public np.ndarray Vertices
    cdef public Plane plane

    def __init__(self, np.ndarray vertices):
        self.Vertices = vertices
        self.plane = Plane.from_points(vertices[0], vertices[1], vertices[2])

    cpdef Triangle clone(self):
        # 复制一个平面
        cdef np.ndarray vertices = self.Vertices.copy()  # 深度复制
        return Triangle(vertices)

    cpdef void  flip(self):
        # 翻转一个几何体对象
        self.Vertices = np.flipud(self.Vertices)  # 翻转并且clone 新对象
        self.plane.flip()

    cpdef center(self):
        return np.mean(self.Vertices, axis=0)

    def __str__(self):
        """
        便于查看
        :return:
        """
        return f"Vertices:{self.Vertices.tolist()},Plane:{self.plane}"

    def __repr__(self):
        return f"<Triangle {id(self)}><{self.__str__()}>"

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
        plane = Plane.from_origin_normal(average, normal)  # average, normal
    except Exception as e:
        logging.error(e)
        logging.error(cov_x)
        raise Exception(f"PCA 拟合平面时发生错误:{traceback.format_exc()}")
    return plane

cdef Plane get_plane_try_pca(list triangles):
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
    if triangles_n_total < TRIANGLE_NUMBER:
        return Plane.from_points(triangles[0].Vertices[0], triangles[0].Vertices[1], triangles[0].Vertices[2])

    triangles = random.sample(triangles, TRIANGLE_NUMBER)

    vertices = np.asarray([triangle.center() for triangle in triangles], dtype=np.float64)
    try:
        plane = get_plane_by_pca(vertices)
    except Exception as e:
        logging.error(e)

    if plane is not None:
        for triangle in triangles:
            out_triangles_i = []
            in_triangles_i = []
            on_same_i = []
            on_diff_i = []

            plane.split_triangle(triangle, on_same_i, on_diff_i, out_triangles_i, in_triangles_i)

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
        quality = 1 - (on_n / TRIANGLE_NUMBER) - (out_n - mid_n) ** 2 - (in_n - mid_n) ** 2
        if quality < PCA_CHECK_VALUE:  # 说明此时选取的平面集,不适合用pca 做平面分割
            plane = Plane.from_points(triangles[0].Vertices[0], triangles[0].Vertices[1], triangles[0].Vertices[2])
    else:
        plane = Plane.from_points(triangles[0].Vertices[0], triangles[0].Vertices[1], triangles[0].Vertices[2])

    return plane

cdef class Node:
    """
    定义BSP Tree 的节点
    """
    cdef Plane plane
    cdef Node front
    cdef Node back
    cdef list triangles

    def __init__(self, list triangles=None):
        self.plane = None
        self.front = None
        self.back = None
        self.triangles = []
        if triangles:
            self.build(triangles)

    cpdef Node clone(self):
        cdef:
            Node node = Node()
            Triangle triangle

        # 选择存在的进行复制,否则就是前面的对象
        node.plane = self.plane and self.plane.clone()
        node.front = self.front and self.front.clone()
        node.back = self.back and self.back.clone()
        node.triangles = [triangle.clone() for triangle in self.triangles]
        return node

    #翻转几何体?递归
    cpdef void invert(self):
        cdef:
            Triangle triangle

        for triangle in self.triangles:
            triangle.flip()
        self.plane.flip()

        if self.front:
            self.front.invert()
        if self.back:
            self.back.invert()

        self.front, self.back = self.back, self.front  # 反过来

    # 翻转几何,列表做堆栈
    # cpdef void invert(self):
    #     cdef:
    #         Triangle triangle
    #         list task_que = [self]
    #         Node node
    #     while task_que:
    #         node = task_que.pop()
    #
    #         for triangle in node.triangles:
    #             triangle.flip()
    #         node.plane.flip()
    #
    #         node.front, node.back = node.back, node.front  # 反过来
    #
    #         if node.front:
    #             task_que.append(node.front)
    #         if node.back:
    #             task_que.append(node.back)

    #去除在一个bsp 树内部的三角面片后返回新的面片
    # cpdef clip_triangles(self, list triangles):
    #     cdef:
    #         list front
    #         list back
    #         Triangle triangle
    #
    #     if not self.plane:
    #         return triangles
    #
    #     front = []
    #     back = []
    #     for triangle in triangles:
    #         self.plane.split_triangle(triangle, front, back, front, back)
    #
    #     if self.front:
    #         front = self.front.clip_triangles(front)
    #     if self.back:
    #         back = self.back.clip_triangles(back)
    #     else:
    #         back = []
    #
    #     front.extend(back)
    #
    #     return front

    #去除在一个bsp 树内部的三角面片后返回新的面片
    cpdef clip_triangles(self, list triangles, list triangles_out=None):
        cdef:
            list front
            list back
            Triangle triangle

        if not self.plane:
            return triangles
        if triangles_out is None:
            triangles_out = []

        front = []
        back = []

        for triangle in triangles:
            self.plane.split_triangle(triangle, front, back, front, back)

        if self.front:
            self.front.clip_triangles(front, triangles_out)
        else:
            triangles_out.extend(front) # 拼接进去

        if self.back:
            self.back.clip_triangles(back, triangles_out)

        return triangles_out

    # 通过递归,遍历self 节点,将每个节点中的三角形同传入的BSP 树进行处理，得到BSP 树 内部以外的三角面片
    cpdef void clip_to(self, Node bsp):
        self.triangles = bsp.clip_triangles(self.triangles)
        if self.front:
            self.front.clip_to(bsp)
        if self.back:
            self.back.clip_to(bsp)

    cpdef list all_triangles(self):

        cdef list triangles = copy.deepcopy(self.triangles)

        if self.front:
            triangles.extend(self.front.all_triangles())
        if self.back:
            triangles.extend(self.back.all_triangles())

        return triangles

    cpdef build(self, list triangles):
        cdef:
            Triangle triangle

        if not triangles:
            return None

        if not self.plane:
            # 在没有平面时,要尝试通过pca 进行平面拟合
            self.plane = get_plane_try_pca(triangles)

        front = []
        back = []
        for triangle in triangles:
            self.plane.split_triangle(triangle, self.triangles, self.triangles, front, back)
        if front:
            if not self.front:
                self.front = Node()
            self.front.build(front)
        if back:
            if not self.back:
                self.back = Node()
            self.back.build(back)


