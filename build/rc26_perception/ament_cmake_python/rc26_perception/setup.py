from setuptools import find_packages
from setuptools import setup

setup(
    name='rc26_perception',
    version='1.0.0',
    packages=find_packages(
        include=('rc26_perception', 'rc26_perception.*')),
)
