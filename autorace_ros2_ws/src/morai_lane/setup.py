from setuptools import setup
from glob import glob
import os

package_name = 'morai_lane'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='autorace migration',
    maintainer_email='noreply@example.com',
    description='ROS2 port of the original morai_lane camera lane nodes.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'lane_right = morai_lane.lane_right:main',
            'lane_left = morai_lane.lane_left:main',
            'lane_twist = morai_lane.lane_twist:main',
        ],
    },
)
