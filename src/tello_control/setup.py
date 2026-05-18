import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tello_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thyron001',
    maintainer_email='tyrone.novillo@ucuenca.edu.ec',
    description='Sistema modular ROS2 para control y monitoreo del dron DJI Tello.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_connector   = tello_control.drone_connector:main',
            'telemetry_monitor = tello_control.telemetry_monitor:main',
            'object_detector   = tello_control.object_detector:main',
            'mission_planner   = tello_control.mission_planner:main',
        ],
    },
)
