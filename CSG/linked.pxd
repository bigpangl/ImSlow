# cython: language_level=3
"""
Author:     LanHao
Date:       2020/11/6
Python:     python3.6



"""
cimport numpy as np



cdef class SingleLinkedNode:
    cdef public SingleLinkedNode next
    cdef public np.ndarray vertex