"""

Project:    ImSlow
Author:     LanHao
Date:       2020/10/19
Python:     python3.6

"""
from distutils.core import Extension,setup
# from setuptools import setup,Extension
from Cython.Build import cythonize
import numpy as np

extensions = [
    Extension("",["./CSG/*.pyx"],include_dirs=[np.get_include()],language = "c++",),
]

setup(
    name='CSG',
    ext_modules=cythonize(extensions,annotate=True,),
)
