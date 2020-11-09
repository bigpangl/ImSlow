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

from .core import Triangle, Plane, Node, get_cos_by, Polygon
from .csg import CSG
from .linked import SingleLinkedNode
from .utils import Open3dTranslate, stretching_mesh

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

    @unittest.skip("跳过中间测试步骤,在其他地方已经使用到,间接测试")
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
        mesh: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_sphere(10)

        mesh2 = o3d.geometry.TriangleMesh.create_box(5, 5, 5).translate(
            (8, 0, 0))
        triangles = get_triangles_with_mesh(mesh)
        triangles2 = get_triangles_with_mesh(mesh2)
        csg_a = CSG.from_trianagles(triangles)
        csg_b = CSG.from_trianagles(triangles2)
        start = datetime.now()
        csg_c = csg_a.to_subtract(csg_b)
        end = datetime.now()
        logging.debug(f"球体参与计算后,耗时:{end - start}")
        triangles3 = csg_c.to_triangles()
        mesh = to_triangle_mesh(triangles3)
        mesh.compute_convex_hull()
        mesh = mesh.sample_points_uniformly(number_of_points=1500000)
        o3d.visualization.draw_geometries([mesh])
        pass


def get_normals(points):
    normal = None
    for i in range(len(points) - 1):
        point_i_front = points[i - 1]
        point_i = points[i]
        point_i_next = points[i + 1]
        v1 = point_i_next - point_i
        v2 = point_i_front - point_i
        normal = np.cross(v1, v2)  # 多边形所在平面的法向量

        front_points = 0
        next_points = 0
        plane = Plane.from_origin_normal(point_i, np.cross(normal, v2))
        for point in points:
            distnace_mid = plane.distance(point)
            if distnace_mid > 0:
                front_points += 1
            elif distnace_mid < 0:
                next_points += 1
        if next_points > 0 and front_points > 0:
            continue

        front_points = 0
        next_points = 0
        plane = Plane.from_origin_normal(point_i, np.cross(normal, v1))

        for point in points:
            distnace_mid = plane.distance(point)
            if distnace_mid > 0:
                front_points += 1
            elif distnace_mid < 0:
                next_points += 1
        if next_points > 0 and front_points > 0:
            continue
        break

    return normal / np.linalg.norm(normal)


def get_triangles(points):
    """
    需要判断面的同向与否以及是否在三角形内部
    :param points:
    :param plane_normal: 满足右手定则的,多边形法向量
    :return:
    """
    points_plane_normal = get_normals(points)
    logging.debug(f"获取到的多边形右手定则法向量:{points_plane_normal}")
    trangles = []
    linked = SingleLinkedNode(points[0])
    linked_current = linked
    linked_end = None  # 链表的尾巴

    # 构建单向链表
    for i in range(1, len(points)):
        current_point = points[i]
        linked_end = SingleLinkedNode(current_point)
        linked_current.next = linked_end
        linked_current = linked_end

    linked_current = linked  # 指向链表的头
    while True:

        current_next = linked_current.next  # 第i+1
        current_next_2 = current_next and current_next.next  # 第i+2

        point_1 = linked_current.vertex
        point_2 = current_next and current_next.vertex
        point_3 = current_next_2 and current_next_2.vertex

        if point_1 is not None and point_2 is not None and point_3 is not None:
            logging.debug(f"{point_1},{point_2},{point_3}")
            # 判断不共点以及共线
            if np.linalg.norm(point_1 - point_2) == 0:
                logging.debug(f"共点，重新计算")

                current_next.next = None  # 孤立此点
                linked_current.next = current_next_2  # head 位置未移动,少了一个点,本次逻辑中不会出现
                continue
            if np.linalg.norm(point_3 - point_2) == 0:
                logging.debug(f"3-2 共点,这是异常的")

                current_next.next = current_next_2.next
                current_next_2.next = None
                continue

            triangle = Triangle(np.asarray([point_1, point_2, point_3]))

            v1 = point_1 - point_2
            v2 = point_3 - point_2

            normal_t = np.cross(v2, v1)

            if 1 - get_cos_by(v1, v2) < 1e-5:
                # 共线三角形
                logging.debug(f"共线:重新计算")
                linked_current.next = current_next_2

                continue

            if get_cos_by(normal_t, points_plane_normal) > 0:
                logging.debug(f"这是一个凸角")
                # 凸角
                # 判断其他点不在此三角形内部
                node_check = current_next_2.next  # 从下一个点开始检查

                status = True
                while node_check:
                    point_check = node_check.vertex
                    status_check = triangle.vertex_in(point_check)
                    if status_check <= 0:
                        logging.debug(f"存在一个点在三角形内部or 边上")
                        status = False
                        break
                    else:
                        node_check = node_check.next

                if status:
                    trangles.append(triangle)
                    linked_current.next = current_next_2
                    current_next.next = None

                else:
                    # 是凸角但是有别的顶点在此三角形内部
                    logging.debug(f"三角形内部")
                    linked_end.next = linked_current
                    linked_current.next = None
                    linked_current = current_next  # 滚动现在的顶角
            else:
                # 不是凸角
                logging.debug(f"不是凸角")
                linked_end.next = linked_current
                linked_current.next = None
                linked_end = linked_current  # end 移动到真实位置
                linked_current = current_next  # 滚动现在的顶角


        else:
            logging.debug(f"三个点不都存在")
            break

    logging.debug(f"是凸角:{trangles}")
    return trangles


class PolygonTest(unittest.TestCase):
    """
    尝试测试简单多边形分割成多个三角形
    """

    @unittest.skip("跳过中间过程测试")
    def test_area(self):
        """
        尝试测试面积法判断凸角的凸性
        :return:
        """

        trangles = []

        v1 = np.asarray([0, 0, 0])
        v2 = np.asarray([3, 0, 0])
        v3 = np.asarray([3, 3, 0])
        v4 = np.asarray([2, 3, 0])
        v5 = np.asarray([2, 2, 0])
        v6 = np.asarray([1, 2, 0])

        v7 = np.asarray([1, 1, 0])
        v8 = np.asarray([0, 1, 0])

        points = [v1, v2, v3, v4, v5, v6, v7, v8]

        trangles = get_triangles(points)

        mesh = Open3dTranslate.to_mesh(trangles)
        mesh = mesh.sample_points_uniformly(number_of_points=150000)
        o3d.visualization.draw_geometries([mesh])

    @unittest.skip("暂时性跳过")
    def test_stairs(self):
        """
        用于尝试楼梯主体的拉伸和合并
        :return:
        """
        meshs = []
        data = {"top_ear": {"stretch": {"x": 1, "y": 0, "z": 0}, "length": 1310.0,
                            "vertexs": [{"x": 0, "y": 2780, "z": 1590}, {"x": 0, "y": 2300, "z": 1590},
                                        {"x": 0, "y": 2300, "z": 1422}, {"x": 0, "y": 2780, "z": 1422}]},
                "foot_ear": {"stretch": {"x": 1, "y": 0, "z": 0}, "length": 1310.0,
                             "vertexs": [{"x": 0, "y": 480, "z": 257}, {"x": 0, "y": 0, "z": 257},
                                         {"x": 0, "y": 0, "z": 0}, {"x": 0, "y": 480, "z": 0}]},
                "body": {"stretch": {"x": 1, "y": 0, "z": 0}, "length": 1270.0,
                         "vertexs": [{"x": 0, "y": 0, "z": 0}, {"x": 0, "y": 0, "z": 257}, {"x": 0, "y": 480, "z": 257},
                                     {"x": 0, "y": 480, "z": 424}, {"x": 0, "y": 740, "z": 424},
                                     {"x": 0, "y": 740, "z": 591}, {"x": 0, "y": 1000, "z": 591},
                                     {"x": 0, "y": 1000, "z": 757}, {"x": 0, "y": 1260, "z": 757},
                                     {"x": 0, "y": 1260, "z": 924}, {"x": 0, "y": 1520, "z": 924},
                                     {"x": 0, "y": 1520, "z": 1091}, {"x": 0, "y": 1780, "z": 1091},
                                     {"x": 0, "y": 1780, "z": 1257}, {"x": 0, "y": 2040, "z": 1257},
                                     {"x": 0, "y": 2040, "z": 1424}, {"x": 0, "y": 2300, "z": 1424},
                                     {"x": 0, "y": 2300, "z": 1590}, {"x": 0, "y": 2780, "z": 1590},
                                     {"x": 0, "y": 2780, "z": 1422}, {"x": 0, "y": 2520, "z": 1422},
                                     {"x": 0, "y": 300, "z": 0}]}}

        for key, value in data.items():
            value: dict
            points = []
            for vertex in value["vertexs"]:
                points.append([vertex["x"], vertex["y"], vertex["z"]])
            points = np.asarray(points)
            stretch = value["stretch"]
            length = value["length"]
            direction = np.asarray([stretch["x"], stretch["y"], stretch["z"]]) * length
            mesh = stretching_mesh(points, direction)
            meshs.append(mesh)

        # 合并
        csg = None
        for mesh in meshs:
            csg_mid = CSG.from_trianagles(Open3dTranslate.to_triangles(mesh))
            if csg is None:
                csg = csg_mid
            else:
                csg = csg.to_union(csg_mid)

        open_holes = [{"x": 300.0, "y": 100.0, "z": 257.88366668686183, "r": 30.0, "d": {"x": 0, "y": 0, "z": -1},
                       "length": 257.88366668686183},
                      {"x": 970.0, "y": 100.0, "z": 257.88366668686183, "r": 30, "d": {"x": 0, "y": 0, "z": -1},
                       "length": 257.88366668686183},
                      {"x": 300.0, "y": 2680.0, "z": 1590.883666686862, "r": 30, "d": {"x": 0, "y": 0, "z": -1},
                       "length": 168.16251284070796},
                      {"x": 970.0, "y": 2680.0, "z": 1590.883666686862, "r": 30, "d": {"x": 0, "y": 0, "z": -1},
                       "length": 168.16251284070796}]
        mesh = Open3dTranslate.to_mesh(csg.to_triangles())
        mesh = mesh.sample_points_uniformly(number_of_points=300000)

        o3d.visualization.draw_geometries([mesh])


        for hole in open_holes:
            hole_mesh = o3d.geometry.TriangleMesh.create_cylinder(hole["r"],hole["length"])
            hole_mesh.translate((hole["x"],hole["y"],hole["z"]-hole["length"]/2))
            csg_mid = CSG.from_trianagles(Open3dTranslate.to_triangles(hole_mesh))
            csg = csg.to_union(csg_mid)

        mesh = Open3dTranslate.to_mesh(csg.to_triangles())
        mesh = mesh.sample_points_uniformly(number_of_points=300000)

        o3d.visualization.draw_geometries([mesh])


class PcaTest(unittest.TestCase):
    def test_plane_data(self):
        normal = np.asarray([-0.8910065241883504, -0.4539904997395813, -4.021878553617213e-15])
        w = -283.0703430175781

        vertices = np.asarray([[275.72949016875157, 82.3664424312258, 193.41275001514637],
         [275.3848393917956, 83.0428576668101, 188.19825027085392],
         [275.62978906177324, 82.56211687119202, 185.83636853366346]])

        triangle = Triangle(vertices)
        logging.debug(triangle)
        for vertex in vertices:
            logging.debug(f"distance:{triangle.plane.distance(vertex)}")

if __name__ == '__main__':
    unittest.main()
