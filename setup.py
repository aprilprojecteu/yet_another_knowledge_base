#!/usr/bin/env python3

# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
setup_args = generate_distutils_setup(
    packages=['yet_another_knowledge_base'],
    package_dir={
        'yet_another_knowledge_base': 'common/src/yet_another_knowledge_base'}
)

setup(**setup_args)
