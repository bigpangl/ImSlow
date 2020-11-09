# cython: language_level=3

cimport numpy as np

cpdef  get_cos_by(np.ndarray, np.ndarray)

cdef class Triangle:
    cdef public np.ndarray Vertices
    cdef public Plane plane

    cpdef Triangle clone(self)
    cpdef void flip(self)
    cpdef np.ndarray center(self)

    # 判断一个点相对一个三角形的位置,外部,边上,内部
    cpdef int vertex_in(self, np.ndarray)

cdef class Plane:
    cdef public np.ndarray Origin
    cdef public np.ndarray Normal
    cdef public float W

    cpdef void flip(self)
    cpdef Plane clone(self)
    cpdef float distance(self, np.ndarray)
    cpdef void split_triangle(self, Triangle, list, list, list, list)
    cdef void  get_triangles(self, list, list)

cdef Plane get_plane_by_pca(np.ndarray)

cdef Plane get_plane_try_pca(list)

cdef class Node:
    cdef public Plane plane
    cdef public Node front
    cdef public Node back
    cdef public list triangles

    cpdef Node clone(self)
    cpdef void invert(self)
    cpdef clip_triangles(self, list, list)
    cpdef void clip_to(self, Node)

cdef class Polygon:
    cdef public np.ndarray vertices
    cdef public np.ndarray normal

    cpdef np.ndarray get_normal(self)

    cpdef list to_triangles(self)

    cpdef void flip(self)