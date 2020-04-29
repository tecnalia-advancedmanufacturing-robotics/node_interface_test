#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

CONFIG = generate_distutils_setup(
    packages=['test_service'],
    package_dir={'': 'nodes'}
)

setup(**CONFIG)
