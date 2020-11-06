# cython: language_level=3

"""
Author:     LanHao
Date:       2020/11/6
Python:     python3.6

"""
import numpy as np

cimport numpy as np

cdef class SingleLinkedNode:

    def __init__(self, np.ndarray vertex):
        self.vertex = vertex
