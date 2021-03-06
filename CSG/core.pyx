# cython: language_level=3

"""
Author:     LanHao
Date:       2020/11/4
Python:     python3.6

vector 使用numpy基础对象

"""
import math
import logging
import traceback
import random
import numpy as np
cimport numpy as np

from .linked cimport SingleLinkedNode

np.import_array()

cdef float EPSILON = 1e-10
cdef float DISTANCE_EPSILON = EPSILON
cdef int TRIANGLE_NUMBER = 5
cdef float PCA_CHECK_VALUE = 0.6

cpdef get_cos_by(np.ndarray v1, np.ndarray v2):
    # 仅保留计算值本身的精度,不人工进行取舍
    cdef:
        double v1_length = np.linalg.norm(v1)
        double v2_length = np.linalg.norm(v2)

    assert v1_length != 0, Exception(f"计算向量角度,向量长度不可以为0,{v1}")
    assert v2_length != 0, Exception(f"计算向量角度,向量长度不可以为0,{v2}")

    return v1.dot(v2) / (v1_length * v2_length)

cpdef get_angle_by(np.ndarray v1, np.ndarray v2):
    # 仅保留计算值本身的精度,不人工进行取舍
    cdef float cos_value = get_cos_by(v1, v2)

    if cos_value > 1:
        cos_value = 1
    elif cos_value < -1:
        cos_value = -1

    return math.acos(cos_value) / math.pi * 180

cdef class Plane:
    def __str__(self):
        return f"Normal:{self.Normal.tolist()},W:{self.W}"

    def __repr__(self):
        return f"<Plane:{id(self)}><{self.__str__()}>"

    def __cinit__(self, np.ndarray origin, np.ndarray normal, float w):
        self.Origin = origin
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
            float normal_distance

        normal = np.cross(b - a, c - a)
        normal_distance = np.linalg.norm(normal)
        assert normal_distance > 0, Exception(
            f"法向量长度为0:{normal_distance},三个点存在共点:{a},{b},{c},{b - a},{c - a}\r\n{traceback.format_exc()}")
        normal = normal / normal_distance  #单位长度
        return cls(a, normal, normal.dot(a))

    @classmethod
    def from_origin_normal(cls, np.ndarray origin, np.ndarray normal):
        normal = normal / np.linalg.norm(normal)
        return cls(origin, normal, normal.dot(origin))

    cpdef void flip(self):
        self.Normal *= -1  # 向量取反
        self.W *= -1  #参数 取反

    cpdef Plane clone(self):  # 复制平面
        return Plane(self.Origin.copy(), self.Normal.copy(), self.W)  # 复制此平面,防止因为在别处翻转导致原始位置发生变化s

    # 计算一个点到平面的距离,来自于点法式,同时提前计算了W 值以及单位向量了Normal
    cpdef float distance(self, np.ndarray vertex):
        # 会人工进行精度上的取舍
        cdef:
            float distance = 0
            float angle
            np.ndarray vector_origin_v = vertex - self.Origin

        if np.linalg.norm(vector_origin_v) > EPSILON:  # 计算距离的点到平面origin 点距离相当远
            angle = get_angle_by(vector_origin_v, self.Normal)  # 如果在90°就返回0
            if abs(90 - angle) > EPSILON:
                distance = self.Normal.dot(vertex) - self.W
                if abs(distance) < DISTANCE_EPSILON:
                    distance = 0
        return distance

    # 用一个平面分割三角形
    cpdef void split_triangle(self, Triangle triangle, list coplanarfront, list coplanarback, list front, list back):
        # 求三角形交点的方式是错误的
        cdef:
            list check_cache = []
            list on_vertex = []
            list front_vertex = []
            list back_vertex = []

        for vertex in triangle.Vertices:
            check_cache.append(self.distance(vertex))
        for i in range(3):
            vertex_i = triangle.Vertices[i]
            if check_cache[i]==0:
                on_vertex.append(vertex_i)
                front_vertex.append(vertex_i)
                back_vertex.append(vertex_i)
            elif check_cache[i]>0:
                front_vertex.append(vertex_i)
            else:
                back_vertex.append(vertex_i)
            if check_cache[i]*check_cache[(i+1)%3]<0: # 异向
                vertex_i_next = triangle.Vertices[(i+1)%3]
                t = (self.W - self.Normal.dot(vertex_i)) / self.Normal.dot(vertex_i_next - vertex_i)  # 求交点
                v = vertex_i + ((vertex_i_next - vertex_i) * t)
                front_vertex.append(v)
                back_vertex.append(v)

        if len(on_vertex)>=3:
            self.get_triangles(on_vertex,(coplanarfront if self.Normal.dot(triangle.plane.Normal) > 0 else coplanarback))
        else:
            if len(front_vertex)>=3:
                self.get_triangles(front_vertex,front)
            if len(back_vertex)>=3:
                self.get_triangles(back_vertex,back)


    #满足右手定则的顺序点，返回三角形,这是自己提供的方法
    cdef void get_triangles(self, list vertices, list triangles_back):

        cdef:
            list use_vertices
            np.ndarray  vertex_j
            int vertices_n = len(vertices)
            int i, j
            Triangle triangle

        assert vertices_n < 5, Exception("三角形在分割时,被分割出来的部分,不可能存在超过5个点")
        assert vertices_n >= 3, Exception("不应该传入小于三个点的多边形")

        for i in range(0, vertices_n - 1, 2):
            use_vertices = []
            for j in range(3):
                vertex_j = vertices[(i + j) % vertices_n]
                use_vertices.append(vertex_j)
            try:
                triangle = Triangle(np.asarray(use_vertices, dtype=np.float64))
                triangles_back.append(triangle)
            except Exception as e:
                logging.error(e)
                logging.error(traceback.format_exc())

cdef class Triangle:
    """
    多边形,同时也可以是三角形// 重点指三角形
    """
    def __cinit__(self, np.ndarray vertices):
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

    cpdef np.ndarray center(self):
        return np.mean(self.Vertices, axis=0)

    # 检查一个点是否在三角形内部
    cpdef int vertex_in(self, np.ndarray vertex):

        cdef:
            np.ndarray v1, v2, v3
            int back = 0

        if abs(self.plane.distance(vertex)) > 0:
            back = 1
        else:
            v1 = self.Vertices[0] - vertex
            v2 = self.Vertices[1] - vertex
            v3 = self.Vertices[2] - vertex
            angle1 = get_angle_by(v1, v2)
            angle2 = get_angle_by(v2, v3)
            angle3 = get_angle_by(v3, v1)
            angles_all = angle1 + angle2 + angle3
            if angle1 == 180 or angle2 == 180 or angle3 == 180:
                back = 0
            else:
                # logging.debug(f"三个角度之和：{angles_all}")
                if abs((360 - angles_all)) <= EPSILON:  # 误差判断
                    back = -1
                else:
                    back = 1

        return back

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
            # logging.debug(f"丢弃pca 选择")
            plane = triangles[0].plane.clone()
    else:
        plane = triangles[0].plane.clone()
        # plane = Plane.from_points(triangles[0].Vertices[0], triangles[0].Vertices[1], triangles[0].Vertices[2])

    return plane

cdef class Node:
    """
    定义BSP Tree 的节点
    """
    def __cinit__(self, list triangles=None):
        self.plane = None
        self.front = None
        self.back = None
        self.triangles = []
        # logging.debug(f"triangles 不为空,需要进行build")
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
        # cdef:
        #     list task_que = [self]
        #     Triangle triangle
        #
        # while task_que:
        #     # logging.debug(f"剩余task数量:{len(task_que)}")
        #     node = task_que.pop()
        #     for triangle in node.triangles:
        #         triangle.flip()
        #     node.plane.flip()
        #     node.front, node.back = node.back, node.front  # 反过来
        #     if node.front:
        #         task_que.append(node.front)
        #     if node.back:
        #         task_que.append(node.back)
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

    #去除在一个bsp 树内部的三角面片后返回新的面片
    cpdef clip_triangles(self, list triangles, list triangles_out):
        cdef:
            list front
            list back
            Triangle triangle

        if not self.plane:
            return triangles

        front = []
        back = []

        for triangle in triangles:
            self.plane.split_triangle(triangle, front, back, front, back)

        if self.front:
            self.front.clip_triangles(front, triangles_out)
        else:
            triangles_out.extend(front)  # 拼接进去

        if self.back:
            self.back.clip_triangles(back, triangles_out)

        return triangles_out

    # 通过递归,遍历self 节点,将每个节点中的三角形同传入的BSP 树进行处理，得到BSP 树 内部以外的三角面片
    cpdef void clip_to(self, Node bsp):
        # cdef:
        #     list triangles
        #     list task_que = [self]
        #
        # while task_que:
        #     node = task_que.pop()
        #     triangles = []
        #     bsp.clip_triangles(node.triangles, triangles)
        #     node.triangles = triangles
        #     if node.front:
        #         task_que.append(node.front)
        #     if node.back:
        #         task_que.append(node.back)
        cdef:
            list triangles = []

        bsp.clip_triangles(self.triangles, triangles)
        self.triangles = triangles
        if self.front:
            self.front.clip_to(bsp)
        if self.back:
            self.back.clip_to(bsp)

    def all_triangles(self):
        """
        产生一个迭代器,而不是在过程中重组各个列表
        :return:
        """
        cdef:
            Triangle triangle
            list task_node = [self]
            list triangles = [triangle.clone() for triangle in self.triangles]
            Node node
            list triangles_back = []
        while task_node:
            node = task_node.pop()
            if node.front:
                task_node.append(node.front)
            if node.back:
                task_node.append(node.back)
            for i in range(len(node.triangles)):
                triangles_back.append(node.triangles[i].clone())
                # yield node.triangles[i].clone()
        return triangles_back
    def build(self, triangles):
        # 出现结果异常,会不断导致创建新的节点,生成选择plane 无法
        cdef:
            Triangle triangle
            list front
            list back
            list task_que = [(self, triangles)]

        while task_que:
            front = []
            back = []
            node_current, triangles_current = task_que.pop()
            # logging.debug(f"初始化的数据:{len(triangles_current)}")

            if not triangles_current:
                continue

            if not node_current.plane:
                # 在没有平面时,要尝试通过pca 进行平面拟合
                node_current.plane = get_plane_try_pca(triangles_current)
            # logging.debug(f"平面选择:{node_current.plane}")
            # logging.debug(f"需要处理的三角形数据:{triangles_current}")
            for triangle in triangles_current:
                node_current.plane.split_triangle(triangle, node_current.triangles, node_current.triangles, front, back)
            # logging.debug(f"total:{len(triangles_current)},front:{len(front)},back:{len(back)}")

            if front:
                if not node_current.front:
                    # logging.debug(f"front 自定义")
                    node_current.front = Node()
                task_que.append((node_current.front, front))

                # self.front.build(front)
            if back:
                if not node_current.back:
                    # logging.debug(f"back 自定义")
                    node_current.back = Node()
                task_que.append((node_current.back, back))
                # self.back.build(back)

