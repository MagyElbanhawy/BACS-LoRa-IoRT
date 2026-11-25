"""Merged package setup for bacs_scheduler.

Restores the four original console-script entry points from the master
branch (bacs_sender, bacs_receiver, bacs_candidates, bacs_fusion) and adds
the new scheduler_node entry point plus the launch/ data_files glob needed
by session.launch.py.
"""
import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = 'bacs_scheduler'

setup(
    name=PACKAGE_NAME,
    version='1.1.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'numpy', 'scipy', 'gtsam'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'bacs_sender = bacs_scheduler.sender_node:main',
            'bacs_receiver = bacs_scheduler.receiver_node:main',
            'bacs_candidates = bacs_scheduler.candidate_node:main',
            'bacs_fusion = bacs_scheduler.fusion_node:main',
            'scheduler_node = bacs_scheduler.scheduler_node:main',
            'scheduler_node_bacs = bacs_scheduler.scheduler_node_bacs:main',
            'bacs_receiver_node = bacs_scheduler.receiver_node:main',
        ],
    },
)
