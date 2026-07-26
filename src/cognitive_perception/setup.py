import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cognitive_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeffrey Navin',
    maintainer_email='jeffreynavin05@gmail.com',
    description='Ground-truth and placeholder camera/lidar perception nodes for the cognitive navigation project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = cognitive_perception.perception_node:main',
            'camera_node = cognitive_perception.camera_node:main',
            'lidar_node = cognitive_perception.lidar_node:main',
        ],
    },
)
