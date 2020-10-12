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
        normal_count = np.cross((ver2 - ver1).to_ndarray(), (ver2 - ver3).to_ndarray())  # 叉乘
        logger.debug(normal_count)
        if normal is not None:
            mid: np.ndarray = np.cross(normal_count, normal.to_ndarray())  # 如果平行,此矩阵应该为[0,0,0]
            assert not any(mid), ErrorNormal
            normal_count = normal

        self._normal = normal_count

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

        self._center = center
        self._normal = normal

    def check_in(self, xyz: XYZ) -> float:
        """
        检测一个点是否在平面上,0 表示在其上,大于0 和小于0 分别表示两个方向
        :return:
        """
        return self._normal.X * (xyz.X - self._center.X) + self._normal.Y * (
                xyz.Y - self._center.Y) + self._normal.Z * (xyz.Z - self._center.Z)

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
    def Triangles(self) -> [FunctionType, MethodType]:
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
