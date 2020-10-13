"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/12
Python:     python3.6

"""
import abc
import logging
import math

from typing import List, Dict
from types import FunctionType, MethodType

import numpy as np

logger = logging.getLogger(__name__)


class ErrorGeometry(Exception):
    """
    对象不异常
    """
    pass


class ErrorNormal(Exception):
    """
    法向量传入和实际值不同
    """
    pass


class XYZ:
    """
    表示单个点,亦或者向量
    """
    _x: [float, int]
    _y: [float, int]
    _z: [float, int]

    def __init__(self, x=0, y=0, z=0):
        """
        实例化
        :param x:3D 空间中X 轴
        :param y:3D 空间中Y 轴
        :param z:3D 空间中Z 轴
        """
        self._x = x
        self._y = y
        self._z = z

    @property
    def X(self):
        return self._x

    @X.setter
    def X(self, value: float):
        self._x = value

    @property
    def Y(self):
        return self._y

    @Y.setter
    def Y(self, value: float):
        self._y = value

    @property
    def Z(self):
        return self._z

    @Z.setter
    def Z(self, value: float):
        self._z = value

    def __add__(self, other):
        """
        向量加法
        :param other:XYZ
        :return:
        """
        assert isinstance(other, XYZ), ErrorGeometry
        return XYZ(self.X + other.X, self.Y + other.Y, self.Z + other.Z)

    def __sub__(self, other):
        """
        向量减法
        :param other:XYZ
        :return:
        """
        assert isinstance(other, XYZ), ErrorGeometry
        return XYZ(self.X - other.X, self.Y - other.Y, self.Z - other.Z)

    def __mul__(self, other: [int, float]) -> "XYZ":
        """
        向量乘法
        :param other:
        :return:
        """
        return XYZ(self.X * other, self.Y * other, self.Z * other)

    @property
    def Length(self) -> float:
        return math.sqrt(self.X ** 2 + self.Y ** 2 + self.Z ** 2)

    def normal_size(self) -> "XYZ":
        """
        保持向量方向不变,长度变成1
        :return:
        """
        length = self.Length
        return XYZ(self.X / length, self.Y / length, self.Z / length)

    def angle_to_xyz(self, point: "XYZ") -> float:
        """
        计算两个向量的角度
        :param point:
        :return:
        """
        radian = self.radian_to_xyz(point)
        return (radian / math.pi) * 180

    def radian_to_xyz(self, point: "XYZ") -> float:
        """
        返回两个向量夹角的弧度
        :param point:
        :return:
        """
        a = self.to_ndarray()
        b = point.to_ndarray()
        cos_angle = a.dot(b) / (np.linalg.norm(a) * np.linalg.norm(b))
        radian = math.acos(cos_angle)
        return radian

    def cross_product(self, v_in: "XYZ") -> "XYZ":
        """
        求法向量,命令参照于revit XYZ 对象
        :param v_in:
        :return:
        """

        pass

    def __str__(self):
        """
        输出时,如何显示相关
        :return:
        """

        return f"X:{self.X} Y:{self.Y} Z:{self.Z}"

    def __repr__(self):
        """
        特定时候打印信息
        :return:
        """
        return f'<{self.__module__}.{type(self).__name__} object at {hex(id(self))}><{self.__str__()}>'

    def to_list(self) -> List[float]:
        """
        转换成python list，修改list 将无法修改XYZ 对象
        :return:
        """
        return [self.X, self.Y, self.Z]

    def to_ndarray(self) -> np.ndarray:
        """
        转换成np 矩阵
        :return:
        """
        return np.array(self.to_list(), dtype=np.float)

    def __hash__(self):
        logger.debug(f"调用hash算法")
        return super(XYZ, self).__hash__()


class Triangle:
    """
    定义一个三角形所需参数
    """
    _vertexs: List[XYZ]  # 点信息
    _ver_ids: List[int]  # 该点所在的index
    _normal: XYZ  # 传入的法向量

    def __init__(self, ver1: XYZ, ver2: XYZ, ver3: XYZ, index1: int = None, index2: int = None, index3: int = None,
                 normal: XYZ = None):
        self._vertexs = [ver1, ver2, ver3]
        self._ver_ids = [index1, index2, index3]

        normal_count: np.ndarray = np.cross((ver2 - ver1).to_ndarray(), (ver2 - ver3).to_ndarray())  # 叉乘
        if normal is not None:
            mid: np.ndarray = np.cross(normal_count, normal.to_ndarray())  # 如果平行,此矩阵应该为[0,0,0]
            assert not any(mid), ErrorNormal
            normal_count = normal.to_ndarray()

        self._normal = XYZ(normal_count[0], normal_count[1], normal_count[2])

    # 判断求出的法向量和传入的发向量是否平行

    def get_index(self, index_in: int) -> int:
        """
        如果这个三角形是定义在几何体内部,所对应的点坐标,需要记录序号值
        :param index_in:
        :return:
        """
        return self._ver_ids[index_in]

    @property
    def Vertexs(self) -> List[XYZ]:
        return self._vertexs

    @property
    def Normal(self) -> XYZ:
        return self._normal


    def __str__(self):
        return f"{self._vertexs}"

    def __repr__(self):
        return f'<{self.__module__}.{type(self).__name__} object at {hex(id(self))}><{self.__str__()}>'


class Line:
    """
    定义线条
    """
    _start: XYZ
    _end: XYZ

    def __init__(self, start: XYZ, end: XYZ):
        self._start = start
        self._end = end

    @property
    def Start(self) -> XYZ:
        return self._start

    @property
    def End(self) -> XYZ:
        return self._end


class Plane:
    """
    定义一个平面
    """
    _center: XYZ
    _normal: XYZ

    def __init__(self, center: XYZ, normal: XYZ):
        """
        点法式创建一个平面
        :param center:
        :param normal:
        """
        assert isinstance(center, XYZ), Exception
        assert isinstance(normal, XYZ), Exception
        self._center = center
        self._normal = normal

    def check_in(self, xyz: XYZ, value_check=1e-10) -> float:
        """
        检测一个点是否在平面上,0 表示在其上,大于0 和小于0 分别表示两个方向
        施加精度限制

        :return:
        """
        value = self._normal.X * (xyz.X - self._center.X) + self._normal.Y * (
                xyz.Y - self._center.Y) + self._normal.Z * (xyz.Z - self._center.Z)
        if value > 0:
            value = value if value > value_check else 0
        else:
            value = value if value < -value_check else 0

        return value

    @property
    def Normal(self) -> XYZ:
        """
        法向量
        :return:
        """
        return self._normal

    @property
    def Center(self) -> XYZ:
        """
        平面的中心,暂时用不着
        :return:
        """
        return self._center

    def __str__(self):
        return f"Center:{self._center},Normal:{self._normal}"

    def __repr__(self):
        return f'<{self.__module__}.{type(self).__name__} object at {hex(id(self))}><{self.__str__()}>'

    def project(self, point: XYZ) -> XYZ:
        """
        求点到直线的垂点
        :param point:
        :return:
        """
        v = self._center - point

        radian = v.radian_to_xyz(self._normal)

        v_new: XYZ = self._normal.normal_size() * (v.Length * math.cos(radian))
        project_point: XYZ = point + v_new
        return project_point

    def intersection(self, line: Line) -> XYZ:
        """
        求平面和直线的交点

        注意,此处的交点会因为计算误差,存在一定的差异,会导致计算出来的点,不完全在该平面上,所以内部引入了一个误差值做校验

        :param line:
        :return:
        """
        use_point: XYZ = None
        another_point: XYZ = None
        if self.check_in(line.Start):
            use_point = line.Start
            another_point = line.End
        else:
            use_point = line.End
            another_point = line.Start

        project_point: XYZ = self.project(use_point)  # 点到平面的垂点
        v_line: XYZ = another_point - use_point  # 直线的向量
        v_project: XYZ = project_point - use_point
        radian = v_line.radian_to_xyz(v_project)  # 夹角的弧度
        v_line_length = v_project.Length / math.cos(radian)
        intersection_point: XYZ = use_point + v_line.normal_size() * v_line_length  #
        check_value = abs(self.check_in(intersection_point))
        assert check_value <= 1e-10, f"计算出来的交点不在平面上,status:{check_value},point:{intersection_point}"  # 确定该点在交点上
        return intersection_point


class GeomObjectIn(abc.ABC):
    """
    定义一个几何体对象的虚类,以便后续自定义特殊形状时便于使用
    """

    @property
    @abc.abstractmethod
    def Vertexs(self) -> List[XYZ]:
        """
        一个几何体应该具备的所有定点信息,注意使用时的引用传递特性
        :return:
        """
        pass

    @property
    @abc.abstractmethod
    def Triangles(self) -> List[Triangle]:
        """
        返回该几何体所有的三角形信息
        :return:
        """
        pass

    @property
    @abc.abstractmethod
    def Normals(self) -> List[XYZ]:
        """
        返回该几何体按序对应的三角形的法向量
        :return:
        """
        pass


class MeshGeom(GeomObjectIn):
    """
    这个是用于通过点面法向量导入模型的工具
    """
    _vertexs: Dict[XYZ, int]
    _triangele: List[Triangle]
    _normals: List[XYZ]
    _vertex_list_cache: List[XYZ]

    def __init__(self, vertexs: Dict[XYZ, int], triangels: List[List[int]], normals: List[XYZ]):
        self._vertexs = vertexs
        self._normals = normals
        self._vertex_list_cache = list(self._vertexs.keys())

        triangle_mid: List[Triangle] = []
        for count in range(len(triangels)):
            angle: List[int] = triangels[count]

            tmp_angle: Triangle = Triangle(
                self._vertex_list_cache[angle[0]],
                self._vertex_list_cache[angle[1]],
                self._vertex_list_cache[angle[2]],
                angle[0],  # 序号
                angle[1],
                angle[2],
                self._normals[count]
            )
            triangle_mid.append(tmp_angle)

        self._triangele = triangle_mid

    @property
    def Vertexs(self) -> List[XYZ]:
        return self._vertex_list_cache

    @property
    def Triangles(self) -> List[Triangle]:
        return self._triangele

    @property
    def Normals(self):
        return self._normals


class BoundingBox(GeomObjectIn):
    """
    包围框样式的障碍物几何体
    """
    _vertexs: Dict[XYZ, int]
    _triangele: List[Triangle]
    _vertex_list_cache: List[XYZ]

    def __init__(self, start: XYZ, end: XYZ):
        self._vertexs = {
            start: 0,
            XYZ(end.X, start.Y, start.Z): 1,
            XYZ(end.X, end.Y, start.Z): 2,
            XYZ(start.X, end.Y, start.Z): 3,
            XYZ(start.X, start.Y, end.Z): 4,
            XYZ(end.X, start.Y, end.Z): 5,
            end: 6,
            XYZ(start.X, end.Y, end.Z): 7
        }
        self._vertex_list_cache = list(self._vertexs.keys())
        self._triangele = [
            Triangle(self._vertex_list_cache[0], self._vertex_list_cache[1], self._vertex_list_cache[2],
                     normal=XYZ(0, 0, -1)),
            Triangle(self._vertex_list_cache[0], self._vertex_list_cache[2], self._vertex_list_cache[3],
                     normal=XYZ(0, 0, -1)),

            Triangle(self._vertex_list_cache[4], self._vertex_list_cache[5], self._vertex_list_cache[6],
                     normal=XYZ(0, 0, 1)),
            Triangle(self._vertex_list_cache[4], self._vertex_list_cache[6], self._vertex_list_cache[7],
                     normal=XYZ(0, 0, 1)),

            Triangle(self._vertex_list_cache[1], self._vertex_list_cache[2], self._vertex_list_cache[6],
                     normal=XYZ(1, 0, 0)),
            Triangle(self._vertex_list_cache[1], self._vertex_list_cache[6], self._vertex_list_cache[5],
                     normal=XYZ(1, 0, 0)),

            Triangle(self._vertex_list_cache[0], self._vertex_list_cache[3], self._vertex_list_cache[7],
                     normal=XYZ(-1, 0, 0)),
            Triangle(self._vertex_list_cache[0], self._vertex_list_cache[7], self._vertex_list_cache[4],
                     normal=XYZ(-1, 0, 0)),

            Triangle(self._vertex_list_cache[0], self._vertex_list_cache[1], self._vertex_list_cache[5],
                     normal=XYZ(0, -1, 0)),
            Triangle(self._vertex_list_cache[0], self._vertex_list_cache[5], self._vertex_list_cache[4],
                     normal=XYZ(0, -1, 0)),

            Triangle(self._vertex_list_cache[3], self._vertex_list_cache[2], self._vertex_list_cache[6],
                     normal=XYZ(0, 1, 0)),
            Triangle(self._vertex_list_cache[3], self._vertex_list_cache[6], self._vertex_list_cache[7],
                     normal=XYZ(0, 1, 0)),
        ]

    @property
    def Vertexs(self) -> List[XYZ]:
        return list(self._vertexs.keys())

    @property
    def Triangles(self) -> List[Triangle]:
        return self._triangele

    @property
    def Normals(self) -> List[XYZ]:
        return [angle.Normal for angle in self._triangele]
