import os
import re
from glob import glob

from setuptools import find_packages, setup

package_name = 'mybot_slip'


def _version():
    """版本只写一处：mybot_slip/__init__.py 的 __version__。"""
    here = os.path.abspath(os.path.dirname(__file__))
    path = os.path.join(here, package_name, '__init__.py')
    with open(path, encoding='utf-8') as fp:
        match = re.search(r"__version__\s*=\s*['\"]([^'\"]+)", fp.read())
    return match.group(1) if match else '0.0.0'


setup(
    name=package_name,
    version=_version(),
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='my',
    maintainer_email='my@example.com',
    description='差速轮打滑检测与恢复',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slip_node = mybot_slip.slip_node:main',
            'slip_executor = mybot_slip.slip_executor:main',
            'cmd_mux = mybot_slip.cmd_mux:main',
        ],
    },
)
