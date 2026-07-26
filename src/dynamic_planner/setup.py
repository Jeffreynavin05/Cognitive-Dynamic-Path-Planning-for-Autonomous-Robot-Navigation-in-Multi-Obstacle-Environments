import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'dynamic_planner'

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
    description='Risk-aware local path planner producing /cmd_vel_nav for the cognitive navigation project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = dynamic_planner.planner_node:main',
        ],
    },
)
