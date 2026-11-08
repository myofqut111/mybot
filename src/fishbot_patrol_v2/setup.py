from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'fishbot_patrol_v2'


setup(
    name=package_name,
    version='0.4.9',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'tools'), glob('tools/*.sh')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fishbot Maintainer',
    maintainer_email='dev@example.com',
    description='Patrol execution with resumable progress, failure policies and JSONL reports.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol_executor_v2 = fishbot_patrol_v2.node:main',
            'patrol_viewer = fishbot_patrol_v2.result_viewer:main',
            'patrol_console = fishbot_patrol_v2.console:main',
        ],
    },
)
