import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'mybot_vision'


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name),
         glob('*.pt') + glob('*.engine')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mybot Maintainer',
    maintainer_email='dev@example.com',
    description='ESP32 OV3660 WiFi camera streaming and YOLO object detection for mybot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = mybot_vision.camera_publisher:main',
            'yolo_detector = mybot_vision.yolo_detector:main',
            'capture_images = mybot_vision.capture_images:main',
        ],
    },
)
