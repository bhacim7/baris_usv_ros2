from glob import glob

from setuptools import find_packages, setup

package_name = 'usv_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bhacim7',
    maintainer_email='barishacim@gmail.com',
    description="The USV's ROS 2 nodes; they run on the boat and, unchanged, in simulation",
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'heading_source = usv_nodes.heading_source:main',
        ],
    },
)
