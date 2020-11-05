"""
Author:     LanHao
Date:       2020/11/5
Python:     python3.6

"""
import logging
import unittest
import numpy as np
from datetime import datetime
import open3d as o3d

from .core import Triangle, Plane, Node,CSG

logging.basicConfig(level=logging.DEBUG)

def to_triangle_mesh(iteral):
    """
    将任何可迭代对象(迭代出来的为一个Triangle)转换为open3d 中的TriangleMesh 对象,期间会进行重复点的剔除工作

    这个过程,从目前的效果来看,本身耗时并不多

    :param iteral:
    :return:
    """


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
        # mesh.triangle_normals.append(triangle.Normal)

    return mesh


def get_triangles_with_mesh(mesh):

    triangles = []  # 初始化,用于存放三角形

    for triangle_index in range(len(mesh.triangles)):  # 遍历三角形
        singe_triangle = mesh.triangles[triangle_index]
        # 存放三角形的点
        vertices = np.zeros(shape=(0, 3), dtype=np.float64)
        for i in range(3):
            vertices = np.append(vertices, [mesh.vertices[singe_triangle[i]]], axis=0)
        triangle = Triangle(vertices)

        triangles.append(triangle)  # 会总处理

    return triangles

class CSGTest(unittest.TestCase):
    def test_plane_instance(self):
        """
        测试一个平面的定义,翻转,距离（分正负）
        :return:
        """
        points = np.asarray(
            [[2, 0, 0],
             [-2, 2, 0],
             [-2, 0, 0], ]
        )
        normal = np.asarray([0, 0, 1])
        plane = Plane.from_points(points[0], points[1], points[2])
        cos_value = normal.dot(plane.Normal) / (np.linalg.norm(normal)) * np.linalg.norm(plane.Normal)
        self.assertEqual(cos_value, 1, "平面的向量需要和法向量平行")
        plane.flip()  # 翻转平面
        cos_value = normal.dot(plane.Normal) / (np.linalg.norm(normal)) * np.linalg.norm(plane.Normal)
        self.assertEqual(cos_value, -1, "翻转后平面的向量需要和法向量平行但方向相反")

        distance = plane.distance(np.asarray([0, 0, 0]))
        self.assertEqual(distance, 0, "原点到该平面距离应该为0")
        self.assertEqual(plane.distance(np.asarray([0, 0, 1])), -1, "翻转平面后,该点在平面in 方向,长度为1")
        self.assertEqual(plane.distance(np.asarray([0, 0, 2])), -2, "翻转平面后,该点在平面in 方向,长度为2")
        self.assertEqual(plane.distance(np.asarray([0, 0, -2])), 2, "翻转平面后,该点在平面in 方向,长度为2")
        plane.flip()
        self.assertEqual(plane.distance(np.asarray([0, 0, 2])), 2, "翻转回去后,该点在平面out 方向,长度为2")

    def test_plane_split_polygon(self):
        """
        测试平面分割三角形

        :return:
        """
        points_plane = np.asarray([
            [0, 0, 0],
            [0, 0, 1],
            [0, 1, 0]
        ])
        normal = np.asarray([-1, 0, 0])  # x 轴负方向

        plane = Plane.from_points(points_plane[0], points_plane[1], points_plane[2])
        cos_value = normal.dot(plane.Normal) / (np.linalg.norm(normal)) * np.linalg.norm(plane.Normal)
        self.assertEqual(cos_value, 1, "定义的平面法向量和预想的不一样")

        polygon = Triangle(np.asarray([
            [2, 0, 0],
            [-2, 2, 0],
            [-2, 0, 0],
        ]))
        front = []
        back = []
        plane.split_triangle(polygon, front, back, front, back)
        self.assertEqual(len(front), 2, "分割后的三角形front数量异常")
        self.assertEqual(len(back), 1, "分割后的三角形back数量异常")

        triangle = Triangle(np.asarray([
            [0, 0, 0],
            [0, 0, 1],
            [0, 1, 0],
        ]))
        front = []
        back = []

        plane.split_triangle(triangle, front, back, front, back)
        self.assertEqual(len(front), 1, "front 需要为1个")
        self.assertEqual(len(back), 0, "back 不能有")

        plane.flip()  # 翻转后
        front = []
        back = []
        plane.split_triangle(triangle, front, back, front, back)
        self.assertEqual(len(front), 0, "front 需要为0个")
        self.assertEqual(len(back), 1, "back 有一个")

    def test_triangle_intance(self):
        vertices = np.asarray([
            [2, 0, 0],
            [-2, 2, 0],
            [-2, 0, 0],
        ])
        polygon = Triangle(vertices)

    # @unittest.skip("跳过中间测试步骤,在其他地方已经使用到,间接测试")
    def test_bsp_node_build(self):
        # 定义一个长宽高5x5x5 的立方体
        mesh: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_sphere(10)

        triangles = get_triangles_with_mesh(mesh)

        node = Node()
        node.build(triangles)


        mesh = to_triangle_mesh(node.all_triangles())
        mesh.compute_convex_hull()
        mesh = mesh.sample_points_uniformly(number_of_points=1500000)
        o3d.visualization.draw_geometries([mesh])

    @unittest.skip("跳过整体UI 效果查看")
    def test_csg(self):
        mesh: o3d.open3d_pybind.geometry.TriangleMesh =  o3d.geometry.TriangleMesh.create_sphere(10)

        mesh2 = o3d.geometry.TriangleMesh.create_box(5, 5, 5).translate(
            (8, 0, 0))
        triangles = get_triangles_with_mesh(mesh)
        triangles2 = get_triangles_with_mesh(mesh2)
        csg_a = CSG.from_trianagles(triangles)
        csg_b = CSG.from_trianagles(triangles2)
        start = datetime.now()
        csg_c = csg_a.to_subtract(csg_b)
        end = datetime.now()
        logging.debug(f"球体参与计算后,耗时:{end-start}")
        triangles3 = csg_c.to_triangles()
        mesh = to_triangle_mesh(triangles3)
        mesh.compute_convex_hull()
        mesh = mesh.sample_points_uniformly(number_of_points=1500000)
        o3d.visualization.draw_geometries([mesh])
        pass


if __name__ == '__main__':
    unittest.main()
