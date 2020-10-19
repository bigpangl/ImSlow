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
    Extension("",["./open3dExp/*.pyx"],include_dirs=[np.get_include()]),
]

setup(
    name='open3dExp',
    ext_modules=cythonize(extensions,annotate=True),
)
