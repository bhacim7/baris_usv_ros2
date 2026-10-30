from glob import glob

from setuptools import find_packages, setup

package_name = 'usv_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/courses', glob('courses/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/models/usv_boat', glob('models/usv_boat/model.*')),
        ('share/' + package_name + '/models/usv_boat/meshes', glob('models/usv_boat/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Simulation of the USV: boat model, worlds, sim_bridge',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim_bridge = usv_sim.sim_bridge:main',
            'fake_perception = usv_sim.fake_perception:main',
        ],
    },
)
