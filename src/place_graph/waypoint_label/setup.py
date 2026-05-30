import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'waypoint_label'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guojun',
    maintainer_email='chengj0803@gmail.com',
    description='VLM-based region semantic labeler for the TypeGo place graph.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_labeler_node = '
            'waypoint_label.semantic_labeler_node:main',
        ],
    },
)
