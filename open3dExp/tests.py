"""

Project:    ImSlow
Author:     LanHao
Date:       2020/11/2
Python:     python3.6

"""
import logging
import unittest
import numpy as np
import open3d as o3d
from datetime import datetime

logging.basicConfig(level=logging.DEBUG)

from .core import Triangle, Plane, Line,cos_with_vectors,create_bsp_tree_with_triangle_mesh,to_triangle_mesh,split_triangle_mesh


class CoreTest(unittest.TestCase):

    @unittest.skip("暂时跳过测试")
    def test_plane_split_triangle(self):
        origin = np.asarray([0, 0, 0])
        normal = np.asarray([1, 0, 0, ])
        plane = Plane.create_by_origin(origin, normal)
        points = np.asarray([
            [2, 0, 0],
            [-2, 2, 0],
            [-2, 0, 0],
        ])
        normal_triangle = np.asarray([0, 0, 1])
        triangle = Triangle(points, normal_triangle)
        out_t, in_t, on_same, on_diff = plane.split_triangle(triangle)
        logging.info(f"out:{out_t}")
        logging.info(f"in_t:{in_t}")
        logging.info(f"on_same:{on_same}")
        logging.info(f"on_diff:{on_diff}")
        self.assertTrue(len(out_t) == 1,   f"分割失败,外侧三角形:{out_t}")
        self.assertTrue(len(in_t) == 2,   f"分割失败,外侧三角形:{in_t}")
        self.assertTrue(len(on_same) == 0,   f"分割失败,外侧三角形:{on_same}")
        self.assertTrue(len(on_diff) == 0,   f"分割失败,外侧三角形:{on_diff}")

    @unittest.skip("暂时跳过测试")
    def test_plane_split_triangle_data(self):
        """
        此处是根据具体的一个错误来求证得到进行进一步测试,针对情况就是一条边在平面上
        :return:
        """
        origin = np.asarray([8,5,0])
        normal = np.asarray([0,1,0 ])
        plane = Plane.create_by_origin(origin, normal)
        points = np.asarray([
            [8,0,5],
            [8,5,5],
            [8,5,0],
        ])
        normal_triangle = np.asarray([-1, 0, 0])
        triangle = Triangle(points, normal_triangle)
        out_t, in_t, on_same, on_diff = plane.split_triangle(triangle)
        logging.info(f"out:{out_t}")
        logging.info(f"in_t:{in_t}")
        logging.info(f"on_same:{on_same}")
        logging.info(f"on_diff:{on_diff}")

    @unittest.skip("跳过测试")
    def test_cos_with_vectors(self):
        """
        测试夹角cos 计算
        :return:
        """
        v1 = np.asarray([0,1,0]) # y方向
        v2 = np.asarray([1,0,0]) # x 方向
        cos_value = cos_with_vectors(v1,v2)
        self.assertTrue(cos_value==0,f"cos 夹角计算失败:{v1},{v2}:{cos_value}")

    # @unittest.skip("跳过测试环节")
    def test_single_bsp_tree(self):

        mesh_qiu: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_sphere(10)
        # mesh_qiu: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_box(5,5,5).translate((4, 4, 0))
        mesh_lifang: o3d.open3d_pybind.geometry.TriangleMesh = o3d.geometry.TriangleMesh.create_box(5,5,5).translate((8,0, 0))
        mesh_qiu.compute_vertex_normals()
        mesh_lifang.compute_vertex_normals()
        start = datetime.now()
        tree_qiu = create_bsp_tree_with_triangle_mesh(mesh_qiu)
        end = datetime.now()
        logging.info(f"bsp 树构造费时:{end - start}")
        tree_lifang = create_bsp_tree_with_triangle_mesh(mesh_lifang)
        # start = datetime.now()
        # logging.info(tree_lifang.check_in(np.asarray([15, 0, 0])))

        out_triangles, in_triangles, on_same_triangles, on_diff_triangles = split_triangle_mesh(mesh_qiu, tree_lifang)
        triangles_all = out_triangles #+in_triangles+ on_same_triangles+ on_diff_triangles
        # end = datetime.now()
        # logging.info(f"分割切面费时:{end - start}")
        iteral_use = (list_angle2 for list_angle1 in triangles_all for list_angle2 in list_angle1)
        # logging.info(f"out_triangles:{len(out_triangles)}")
        # logging.info(f"in_triangles:{len(in_triangles)}")
        # logging.info(f"on_same_triangles:{len(on_same_triangles)}")
        # logging.info(f"on_diff_triangles:{len(on_diff_triangles)}")
        mesh_handle = to_triangle_mesh(iteral_use)

        # mesh_qiu = to_triangle_mesh(tree_qiu.traverse())
        # mesh_lifang = to_triangle_mesh(tree_lifang.traverse())

        # mesh_handle= mesh_qiu.sample_points_uniformly(number_of_points=1500000)
        # mesh_qiu = mesh_qiu.sample_points_uniformly(number_of_points=1500000)
        mesh_lifang = mesh_lifang.sample_points_uniformly(number_of_points=1500000)
        o3d.visualization.draw_geometries([mesh_handle])


if __name__ == '__main__':
    unittest.main()