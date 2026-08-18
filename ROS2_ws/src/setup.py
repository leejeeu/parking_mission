import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'parking_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ijiyu',
    maintainer_email='guabcd05028@gmail.com',
    description='자율주행 주차 미션: Nav2 기반 출발지 -> 지정 주차영역(A/B) 자율 주차',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_navigator = parking_mission.parking_navigator:main',
            'odom_publisher = parking_mission.localization.odom_publisher:main',
            'cmd_vel_bridge = parking_mission.cmd_vel_bridge:main',
        ],
    },
)
