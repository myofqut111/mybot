from glob import glob
import os
import re

from setuptools import find_packages, setup


package_name = 'mybot_patrol_v2'


def _version():
    """版本只写一处：mybot_patrol_v2/__init__.py 的 __version__。"""
    here = os.path.abspath(os.path.dirname(__file__))
    with open(os.path.join(here, package_name, '__init__.py'), encoding='utf-8') as fp:
        match = re.search(r"__version__\s*=\s*['\"]([^'\"]+)", fp.read())
    return match.group(1) if match else '0.0.0'


setup(
    name=package_name,
    version=_version(),
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
    maintainer='Mybot Maintainer',
    maintainer_email='dev@example.com',
    description='Patrol execution with resumable progress, failure policies and JSONL reports.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol_executor_v2 = mybot_patrol_v2.node:main',
            'patrol_viewer = mybot_patrol_v2.result_viewer:main',
            'patrol_console = mybot_patrol_v2.console:main',
        ],
    },
)
