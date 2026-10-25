from setuptools import find_packages
from setuptools import setup

setup(
    name='map_trans_pkg',
    version='0.0.1',
    packages=find_packages(
        include=('map_trans_pkg', 'map_trans_pkg.*')),
)
