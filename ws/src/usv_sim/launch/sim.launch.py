"""The whole simulation with one command.

    ros2 launch usv_sim sim.launch.py                      # Golcuk course, GUI, running
    ros2 launch usv_sim sim.launch.py paused:=true         # start paused (press play)
    ros2 launch usv_sim sim.launch.py gui:=false           # no Gazebo window (faster)
    ros2 launch usv_sim sim.launch.py course:=<name>       # another usv_sim/courses/<name>.yaml
    ros2 launch usv_sim sim.launch.py start_task:=TASK1_STATE_ENTER   # not the course's own
    ros2 launch usv_sim sim.launch.py navigator:=false     # drive by hand (ros2 topic pub)

What starts, in this order:
  1. make_course_world.py builds the world from the course file into /tmp, so the world
     can never be older than the course (no more "remember to regenerate").
  2. When that has finished: Gazebo (ros_gz_sim), ros_gz_bridge, sim_bridge,
     fake_perception, heading_source and navigator, each with its YAML file from config/.
     The navigator also gets the course file's `mission` section (start task, waypoints,
     Task 3 colour). It starts in MANUAL, as on the boat:
         ros2 service call /usv/set_manual std_srvs/srv/SetBool "{data: false}"
     Our nodes restart by themselves if they die (respawn), as on the boat.

Ctrl+C stops everything.
"""

import os

from ament_index_python.packages import get_package_share_directory
import yaml
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
    nodes_config = os.path.join(get_package_share_directory('usv_nodes'), 'config')
    heading_config = os.path.join(nodes_config, 'heading_source.yaml')
    navigator_config = os.path.join(nodes_config, 'navigator.yaml')

    # The course's mission section on top of navigator.yaml
    with open(course_file) as f:
        mission = yaml.safe_load(f).get('mission', {})
    navigator_overrides = {k: v for k, v in mission.items() if k != 'waypoints'}
    navigator_overrides.update({'waypoints.' + name: [float(x) for x in latlon]
                                for name, latlon in mission.get('waypoints', {}).items()})
    start_task = LaunchConfiguration('start_task').perform(context)
    if start_task:
        navigator_overrides['start_task'] = start_task
    with_navigator = LaunchConfiguration('navigator').perform(context).lower() == 'true'

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
    if with_navigator:
        ours.append(Node(package='usv_nodes', executable='navigator', output='screen',
                         parameters=[navigator_config, navigator_overrides],
                         respawn=True, respawn_delay=2.0))

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
        DeclareLaunchArgument('start_task', default_value='',
                              description="navigator start task; empty = the course's own"),
        DeclareLaunchArgument('navigator', default_value='true',
                              description='false = no navigator, drive with ros2 topic pub'),
        OpaqueFunction(function=setup),
    ])
