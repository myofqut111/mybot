from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fishbot_console'

setup(
    name=package_name,
    version='0.4.5',
    packages=find_packages(exclude=['test']),
    # Leaflet / roslib 杩欎簺 JS 蹇呴』璺熺潃鍖呬竴璧疯锛氭櫘閫?colcon build 鍙嫹 .py锛?    # 涓嶅姞 package_data 鐨勮瘽椤甸潰涓?/static/leaflet.js 浼?404锛屽湴鍥剧洿鎺ョ櫧灞忋€?    package_data={package_name: ['static/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # static 鍙蛋 package_data 涓€澶勩€傝涓や唤锛坧ackage_data + share锛夊湪
        # --symlink-install 涓嬩細澶氬缓涓€濂楃鍙烽摼鎺ワ細涔嬪墠灏卞洜涓?install 閲岀暀浜?        # 涓€鏉℃偓绌洪摼鎺ワ紝build 鎶?No such file or directory: .../launch/mock.launch.py銆?        # server._static_dir() 浼樺厛鎵惧寘鐩綍锛屾壘涓嶅埌鎵嶉€€鍥?share锛屾墍浠ヨ繖閲屼笉鐢ㄥ啀瑁呫€?    ],
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
            'mock_robot = fishbot_console.mock_robot:main',
        ],
    },
)
