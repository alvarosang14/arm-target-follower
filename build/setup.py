#!/usr/bin/env python3
#package_data={'GrabberControls2Gui': ['templates/*.ui']},

from setuptools import setup
from os.path import normpath

setup(
    name='ROBOTICSLAB_TFG',
    version='0.0.5',
    description='Uc3m TFG - Python CLI tool',
    py_modules=['run', 'cameraDetection', 'headController'],
    package_dir={'': normpath('/home/alvaro/my_project/src/')},
    install_requires=['begins==0.9'],
    entry_points={
        'gui_scripts': ['tfg-gui = run:main.start']
    }
)
