from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'mybot_explore'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mybot Maintainer',
    maintainer_email='dev@example.com',
    description='Frontier-based autonomous exploration (static map + coverage layer).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'explore_node = mybot_explore.node:main',
        ],
    },
)
