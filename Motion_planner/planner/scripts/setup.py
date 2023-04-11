import setuptools
from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup

settings = generate_distutils_setup(
    packages=setuptools.find_packages()
)

setup(requires=['numpy', 'PyYAML', 'scipy', 'pytorch'], **settings)
