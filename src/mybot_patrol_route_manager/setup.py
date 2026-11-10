from setuptools import find_packages, setup


package_name = 'mybot_patrol_route_manager'


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mybot Maintainer',
    maintainer_email='dev@example.com',
    description='Records AMCL poses and persistently manages mybot patrol routes.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'route_manager = mybot_patrol_route_manager.route_manager_node:main',
        ],
    },
)
