"""The whole simulation with one command.

    ros2 launch usv_sim sim.launch.py                      # Golcuk course, GUI, running
    ros2 launch usv_sim sim.launch.py paused:=true         # start paused (press play)
    ros2 launch usv_sim sim.launch.py gui:=false           # no Gazebo window (faster)
    ros2 launch usv_sim sim.launch.py course:=<name>       # another usv_sim/courses/<name>.yaml

What starts, in this order:
  1. make_course_world.py builds the world from the course file into /tmp, so the world
     can never be older than the course (no more "remember to regenerate").
  2. When that has finished: Gazebo (ros_gz_sim), ros_gz_bridge, sim_bridge,
     fake_perception and heading_source, each with its YAML file from config/.
     Our three nodes restart by themselves if they die (respawn), as on the boat.

Ctrl+C stops everything.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('usv_sim')
    course = LaunchConfiguration('course').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'

    course_file = os.path.join(share, 'courses', course + '.yaml')
    world_file = os.path.join('/tmp', f'usv_sim_{course}.sdf')
    config = os.path.join(share, 'config')
    heading_config = os.path.join(get_package_share_directory('usv_nodes'), 'config',
                                  'heading_source.yaml')

    make_world = ExecuteProcess(
        cmd=['python3', os.path.join(share, 'scripts', 'make_course_world.py'),
             course_file, world_file],
        output='screen')

    # -r: run at once; -s: server only (no window); -v 3: show gzmsg lines (HullBuoyancy)
    gz_args = ' '.join(['-v 3'] + ([] if paused else ['-r']) + ([] if gui else ['-s'])
                       + [world_file])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
        parameters=[{'config_file': os.path.join(config, 'ros_gz_bridge.yaml')}],
        output='screen')

    ours = [
        Node(package='usv_sim', executable='sim_bridge', output='screen',
             parameters=[os.path.join(config, 'sim_bridge.yaml')],
             respawn=True, respawn_delay=2.0),
        Node(package='usv_sim', executable='fake_perception', output='screen',
             parameters=[os.path.join(config, 'fake_perception.yaml'), {'course': course}],
             respawn=True, respawn_delay=2.0),
        Node(package='usv_nodes', executable='heading_source', output='screen',
             parameters=[heading_config],
             respawn=True, respawn_delay=2.0),
    ]

    start_rest = RegisterEventHandler(OnProcessExit(
        target_action=make_world, on_exit=[gazebo, bridge] + ours))

    return [
        # Our boat model (usv_boat) for Gazebo; VRX and HullBuoyancy paths come from the
        # sourced setup files
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        make_world,
        start_rest,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('course', default_value='golcuk_2026_10_14',
                              description='usv_sim/courses/<course>.yaml'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = Gazebo without a window'),
        DeclareLaunchArgument('paused', default_value='false',
                              description='true = start paused, press play in Gazebo'),
        OpaqueFunction(function=setup),
    ])
