import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'typego_config'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'profiles'),
         glob('config/profiles/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guojun',
    maintainer_email='chengj0803@gmail.com',
    description='Single-source robot configuration loader and service node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'config_service_node = typego_config.service_node:main',
            'typego-config = typego_config.cli:main',
        ],
    },
)
