import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cognitive_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeffrey Navin',
    maintainer_email='jeffreynavin05@gmail.com',
    description='One-command startup of the complete cognitive navigation stack, RViz configuration, and the debugging visualization_node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualization_node = cognitive_bringup.visualization_node:main',
        ],
    },
)
