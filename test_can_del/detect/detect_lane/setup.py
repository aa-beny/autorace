from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'detect_lane'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'), glob(os.path.join('launch','*launch.[pxy][yma]'))),
        (os.path.join('share',package_name,'config'), glob(os.path.join('config/*.yaml'))),
        (os.path.join('share',package_name,'config'), glob(os.path.join('config/*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anderson',
    maintainer_email='anderson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_lane = detect_lane.detect_lane:main',
            'detect_traffic = detect_lane.detect_traffic:main',
            'detect_parking_grid = detect_lane.detect_parking_grid:main',
            'hsv_param_adjustment = detect_lane.hsv_param_adjustment:main',
            'hsv_traffic_light=detect_lane.hsv_traffic_light:main',
            'key_pub_signs = detect_lane.key_pub_signs:main',
        ],
    },
)
