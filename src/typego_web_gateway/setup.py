import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'typego_web_gateway'


def frontend_files():
    """Install built frontend assets if they exist under frontend/dist/."""
    base = 'frontend/dist'
    data = []
    if not os.path.isdir(base):
        return data
    for root, _dirs, files in os.walk(base):
        rel = os.path.relpath(root, base)
        dest = os.path.join('share', package_name, 'frontend')
        if rel != '.':
            dest = os.path.join(dest, rel)
        data.append((dest, [os.path.join(root, f) for f in files]))
    return data


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
        (os.path.join('share', package_name, 'resource'),
         ['resource/foxglove_layout.json']),
    ] + frontend_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guojun',
    maintainer_email='chengj0803@gmail.com',
    description='Web-based operator HMI for TypeGo ROS SDK.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gateway_node = typego_web_gateway.gateway_node:main',
        ],
    },
)
