import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'usv_learn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch and config files next to the package
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='baris',
    maintainer_email='baris@todo.todo',
    description='Learning package for the USV ROS 2 migration',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'fake_gps = usv_learn.fake_gps:main',
            'distance_monitor = usv_learn.distance_monitor:main',
        ],
    },
)
