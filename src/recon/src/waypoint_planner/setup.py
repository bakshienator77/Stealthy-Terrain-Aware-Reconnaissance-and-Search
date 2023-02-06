#!/usr/bin/env python
#
# This notice must appear in all copies of this file and its derivatives.
#
# Created under Program: AIDTR
#
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['waypoint_planner'],
     package_dir={'': 'src'},
     install_requires=['numpy', 'rospy', 'scikit-learn', 'scipy', 'matplotlib']
)

setup(**setup_args)