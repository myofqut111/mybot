from glob import glob
from setuptools import find_packages, setup


package_name = 'fishbot_patrol_task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fishbot Maintainer',
    maintainer_email='dev@example.com',
    description='Dispatches and tracks inspection tasks at fishbot patrol points.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_task_manager = '
            'fishbot_patrol_task_manager.task_manager_node:main',
        ],
    },
)
