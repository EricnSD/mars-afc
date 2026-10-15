from setuptools import setup
from Cython.Build import cythonize

setup(
    name="ODE Module",
    ext_modules=cythonize("ode_funcs.pyx")
)
