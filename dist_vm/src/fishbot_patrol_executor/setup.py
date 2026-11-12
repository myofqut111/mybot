from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'fishbot_patrol_executor'


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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fishbot Maintainer',
    maintainer_email='dev@example.com',
    description='Executes fishbot patrol routes with precise Nav2 pose goals.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_executor = fishbot_patrol_executor.executor_node:main',
        ],
    },
)
