# cython: language_level=3

"""
Author:     LanHao
Date:       2020/11/5
Python:     python3.6

"""
import copy
from typing import List

from .core import Node


class CSG:
    triangles:List
    def __cinit__(self):
        self.triangles = []

    @classmethod
    def from_trianagles(cls, triangles):
        csg = cls()
        csg.triangles = triangles
        return csg

    def clone(self):
        csg = CSG()
        csg.triangles = copy.deepcopy(self.triangles)
        return csg

    def to_triangles(self):

        return [triangle.clone() for triangle in self.triangles]


    def to_union(self, csg):
        a = Node(self.to_triangles())
        b = Node(csg.to_triangles())
        a.clip_to(b)
        b.clip_to(a)
        b.invert()
        b.clip_to(a)
        b.invert()
        a.build(b.all_triangles())
        return CSG.from_trianagles(a.all_triangles())


    def to_subtract(self, csg):
        node_a = Node(self.to_triangles())
        node_b = Node(csg.to_triangles())
        node_a.invert()
        node_a.clip_to(node_b)
        node_b.clip_to(node_a)
        node_b.invert()
        node_b.clip_to(node_a)
        node_b.invert()
        node_a.build(node_b.all_triangles())
        node_a.invert()
        return CSG.from_trianagles(node_a.all_triangles())

    def to_intersect(self, csg):
        nodea = Node(self.to_triangles())
        nodeb = Node(csg.to_triangles())
        nodea.invert()
        nodeb.clip_to(nodea)
        nodeb.invert()
        nodea.clip_to(nodeb)
        nodeb.clip_to(nodea)
        nodea.build(nodeb.all_triangles())
        nodea.invert()
        return CSG.from_trianagles(nodea.all_triangles())
