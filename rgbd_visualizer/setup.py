from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rgbd_visualizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='RGB-D dataset visualization tools for plane detection development',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_publisher = rgbd_visualizer.bag_publisher:main',
            'rgbd_viewer = rgbd_visualizer.rgbd_viewer:main',
            'pointcloud_generator = rgbd_visualizer.pointcloud_generator:main',
        ],
    },
)
