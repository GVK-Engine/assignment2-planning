from setuptools import setup
import os
from glob import glob

package_name = 'ras598_assignment_2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('*.png')),
        ('share/' + package_name, glob('*.yaml')),
        ('share/' + package_name, glob('*.rviz')),
        ('share/' + package_name, ['grading_scout.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'planner_node = ras598_assignment_2.planner_node:main',
        ],
    },
)
