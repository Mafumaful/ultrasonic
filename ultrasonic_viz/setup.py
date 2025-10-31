from setuptools import setup
import os
from glob import glob

package_name = 'ultrasonic_viz'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mini',
    maintainer_email='mini@todo.todo',
    description='Ultrasonic sensor visualization node for ROS2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ultrasonic_viz_node = ultrasonic_viz.ultrasonic_viz_node:main',
        ],
    },
)
