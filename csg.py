"""
Author:     LanHao
Date:       2020/11/4
Python:     python3.6

https://github.com/evanw/csg.js/blob/master/csg.js

尝试参照以上链接的javascript 实现csg 功能

"""

import copy


class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def clone(self):
        """
        复制
        :return:
        """
        return Vector(self.x, self.y, self.z)

    def negated(self):
        """
        取反
        :return:
        """
        return Vector(-self.x, -self.y, -self.z)

    def plus(self, a):
        """
        相加
        :param a:
        :return:
        """
        return Vector(self.x + a.x, self.y + a.y, self.z + a.z)

    def minus(self, a):
        """
        相减
        :param a:
        :return:
        """
        return Vector(self.x - a.x, self.y - a.y, self.z - a.z)

    def times(self, a):
        """
        乘法
        :param a:
        :return:
        """

        return Vector(self.x * a, self.y * a, self.z * a)

    def dividedBy(self, a):
        return Vector(self.x / a, self.y / a, self.z / a)


class Plane:
    """
    定义一个平面
    """

    EPSILON = 1e-5  #

    def __init__(self, normal, w):
        self.normal = normal
        self.w = w

    @staticmethod
    def fromPoints(cls, a, b, c):
        n = b.minus(a).cross(c.minus(a)).unit()
        return cls(n, n.dot(a))

    def clone(self):
        return Plane(self.normal.clone(), self.w)

    def flip(self):
        self.normal = self.normal.negated()
        self.w = -self.w

    def splitPolygon(self, polygon, coplanarFront, coplanarBack, front, back):
        COPLANNAR = 0
        FRONT = 1
        BACK = 2
        SPANNING = 3
        polygontype = 0
        types = []
        for vertice in polygon.vertices:
            t = self.normal.dot(vertice.pos) - self.w
            type_mid = BACK if t < -self.EPSILON else (FRONT if t > self.EPSILON else COPLANNAR)
            polygontype |= type_mid
            types.append(type)

        if polygontype == COPLANNAR:
            data = coplanarFront if self.normal.dot(polygon.plane.normal) > 0 else coplanarBack
            data.append(polygon)

        elif polygontype == FRONT:
            front.append(polygon)
        elif polygontype == BACK:
            back.append(polygon)
        elif polygontype == SPANNING:
            f = []
            b = []
            length = len(polygon.vertices)
            for i in range(polygon.vertices):
                j = (i + 1) % length
                ti = types[i]
                tj = types[j]
                vi = polygon.vertices[i]
                vj = polygon.vertices[j]
                if ti != BACK:
                    f.append(vi)
                if ti != FRONT:
                    b.append(vi.clone() if ti != BACK else vi)
                if ((ti | tj) == SPANNING):
                    t = (self.w - self.normal.dot(vi.pos)) / self.normal.dot(vj.pos.minus(vi.pos))
                    v = vi.interpolate(vj, t)
                    f.append(v)
                    b.append(v.clone())
            if len(f) >= 3:
                front.append(Polygon(f, polygon.shared))
            if len(b) >= 3:
                back.append(polygon(b, polygon.shared))

            pass


class Polygon:
    """
    多边形？
    """

    def __init__(self, vertices, shared):
        self.vertices = vertices
        self.shared = shared
        self.plane = Plane.fromPoints(vertices[0].pos, vertices[1].pos, vertices[2].pos)

    def clone(self):
        vertices = copy.deepcopy(self.vertices)
        return Polygon(vertices, self.shared)

    def flip(self):
        for vertice in self.vertices:
            vertice.flip()

        self.plane.flip()


class Node:
    def __init__(self, polygons=None):
        self.plane = None
        self.front = None
        self.back = None
        self.polygons = []

        if polygons:
            self.build(polygons)

    def clone(self):

        node = Node()
        node.plane = self.plane if self.plane is None else self.plane.clone()
        node.front = self.front if self.front is None else self.front.clone()
        node.back = self.back if self.back is None else self.back.clone()

        node.polygons = copy.deepcopy(self.polygons)

    # Convert solid space to empty space and empty space to solid space. 翻转空间
    def invert(self):
        for polygon in self.polygons:
            polygon.flip()

        self.plane.flip()
        if self.front is not None:
            self.front.invert()
        if self.back is not None:
            self.back.invert()

        self.front, self.back = self.back, self.front  # 交换方向

    def clipPolygons(self, polygons):
        if self.plane is None:
            return copy.deepcopy(polygons)
        front = []
        back = []
        for polygon in polygons:
            self.plane.splitPolygon(polygon, front, back, front, back)
        if self.front is not None:
            self.front.clipPolygons(front)
        if self.back is not None:
            self.back.clipPolygons(back)
        else:
            back = []
        return front + back

    # Remove all polygons in this BSP tree that are inside the other BSP tree 剔除BSP 树中的多边形
    def clipTo(self, bsp):
        self.polygons = bsp.clipPolygons(self.polygons)
        if self.front:
            self.front.clipTo(bsp)
        if self.back:
            self.back.clipTo(bsp)

    def allPolygons(self):

        polygons = copy.deepcopy(self.polygons)
        if self.front:
            polygons = polygons + self.front.allPolygons()
        if self.back:
            polygons = polygons + self.back.allPolygons()

        return polygons

    def build(self, polygons):
        """
        构造BSP 树
        :param polygons:
        :return:
        """
        if not polygons:
            return None

        if not self.plane:
            self.plane = polygons[0].plane.clone()
        front = []
        back = []
        for polygon in polygons:
            self.plane.splitPolygon(polygon, self.polygons, self.polygons, front, back)

        if len(front):
            if not self.front:
                self.front = Node()
            self.front.build(front)
        if len(back):
            if not self.back:
                self.back = Node()
            self.back.build(back)


class CSG:
    def __init__(self):
        self.polygons = []  # 列表存放的？

    @classmethod
    def fromPolygons(cls, polygons):
        csg = cls()
        csg.polygons = polygons
        return csg

    def clone(self):
        """
        clone 操作
        :return:
        """
        csg = CSG()
        csg.polygons = copy.deepcopy(self.polygons)
        return csg

    def toPolygons(self):
        return self.polygons

    def union(self, csg):
        a = Node(self.clone().polygons)
        b = Node(csg.clone().polygons)
        a.clipTo(b)
        b.clipTo(a)
        b.invert()
        b.clipTo(a)
        b.invert()
        a.build(b.allPolygons())
        return CSG.fromPolygons(a.allPolygons())

    def subtract(self, csg):
        a = Node(self.clone().polygons)
        b = Node(csg.clone().polygons)
        a.invert()  # 空间翻转
        a.clipTo(b)
        b.clipTo(a)
        b.invert()
        b.clipTo(a)
        b.invert()
        a.build(b.allPolygons())
        a.invert()
        return CSG.fromPolygons(a.allPolygons())

    def intersect(self, csg):
        a = Node(self.clone().polygons)
        b = Node(csg.clone().polygons)
        a.invert()
        b.clipTo(a)
        b.invert()
        a.clipTo(b)
        b.clipTo(a)
        a.build(b.allPolygons())
        a.invert()
        return CSG.fromPolygons(a.allPolygons())

    def inverse(self):
        csg = self.clone()
        for polygon in csg.polygons:
            polygon.flip()
        return csg
