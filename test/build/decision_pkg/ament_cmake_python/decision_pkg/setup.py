from setuptools import find_packages
from setuptools import setup

setup(
    name='decision_pkg',
    version='0.0.1',
    packages=find_packages(
        include=('decision_pkg', 'decision_pkg.*')),
)
