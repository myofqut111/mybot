from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fishbot_console'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    # Leaflet / roslib 这些 JS 必须跟着包一起装：普通 colcon build 只拷 .py，
    # 不加 package_data 的话页面上 /static/leaflet.js 会 404，地图直接白屏。
    package_data={package_name: ['static/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fishbot Maintainer',
    maintainer_email='dev@example.com',
    description='Browser console: navigation, mapping, patrol, camera, logs and an AI helper.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'robot_console = fishbot_console.node:main',
        ],
    },
)
