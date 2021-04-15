"""
Author:     LanHao
Date:       2021/4/15 13:55
Python:     python3.6

"""

import numpy as np
import open3d as o3d

from expand.o3d_ex import create_cylinder

if __name__ == '__main__':
    r = 3
    center = np.asarray([1, 1, 1])
    normal = np.asarray([1, 0, 0]) * 10
    mesh = create_cylinder(r, center, normal)
    o3d.visualization.draw_geometries([mesh])
