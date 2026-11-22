from glob import glob
import os
import re

from setuptools import find_packages, setup

package_name = 'mybot_console'


def _version():
    """版本只写一处：mybot_console/__init__.py 的 __version__。"""
    here = os.path.abspath(os.path.dirname(__file__))
    with open(os.path.join(here, package_name, '__init__.py'), encoding='utf-8') as fp:
        match = re.search(r"__version__\s*=\s*['\"]([^'\"]+)", fp.read())
    return match.group(1) if match else '0.0.0'


setup(
    name=package_name,
    version=_version(),
    packages=find_packages(exclude=['test']),
    # JS 依赖（Leaflet / roslib / eventemitter2）必须跟着包一起装：
    # 普通 colcon build 只拷 .py，不加 package_data 的话 /static/leaflet.js 会 404，
    # 地图直接白屏。
    package_data={package_name: ['static/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # static 只走 package_data 一处。装两份（package_data + share）在
        # --symlink-install 下会多建一套符号链接，之前就因为 install 里留了一条
        # 悬空链接，build 报 No such file or directory: .../launch/mock.launch.py。
        # server._static_dir() 优先找包目录，找不到才退回 share，所以这里不用再装。
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mybot Maintainer',
    maintainer_email='dev@example.com',
    description='Browser console: navigation, mapping, patrol, camera, logs and an AI helper.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'robot_console = mybot_console.node:main',
            'mock_robot = mybot_console.mock_robot:main',
        ],
    },
)
