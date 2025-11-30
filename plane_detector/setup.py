from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'plane_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Horizontal plane detection for RoboCup@Home using RANSAC algorithm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plane_detector_node = plane_detector.plane_detector_node:main',
            'plane_recorder_node = plane_detector.plane_recorder_node:main',
        ],
    },
)
