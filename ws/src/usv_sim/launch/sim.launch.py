"""
The whole simulation with one command.

    ros2 launch usv_sim sim.launch.py                      # Golcuk course, GUI, running
    ros2 launch usv_sim sim.launch.py paused:=true         # start paused (press play)
    ros2 launch usv_sim sim.launch.py gui:=false           # no Gazebo window (faster)
    ros2 launch usv_sim sim.launch.py course:=<name>       # another usv_sim/courses/<name>.yaml
    ros2 launch usv_sim sim.launch.py start_task:=TASK1_STATE_ENTER   # not the course's own
    ros2 launch usv_sim sim.launch.py navigator:=false     # drive by hand (ros2 topic pub)
    ros2 launch usv_sim sim.launch.py rviz:=true           # also RViz on the world map
    ros2 launch usv_sim sim.launch.py course:=roboboat_2026 course_east:=182.5 \
        course_north:=952.2 course_heading:=180    # put the course elsewhere: boat start
                                                   # (Gazebo x, y) and heading; layout follows

Experiments (phase 5.6), one command each:
    auto:=true                 navigator starts in AUTO (no set_manual call needed)
    record:=<name>             ros2 bag record into /ws/bags/<name> (the topics run_report
                               needs); the name must not exist yet
    compass_offset_deg:=8.0    fake_perception: ZED compass reads 8 deg too high
    detect_prob:=0.5           fake_perception: YOLO finds a buoy in half of the frames
    max_range_m:=10.0          fake_perception: no detections beyond 10 m
  e.g.  ros2 launch usv_sim sim.launch.py auto:=true record:=exp_compass8 compass_offset_deg:=8.0

Map measurements (phase 6 R1.6):
    survey:=true               no navigator; survey_drive drives the course's survey route
                               from Gazebo's true pose, the same path every run
    seed:=<n>                  the same sensor noise every run (sim_bridge, fake_perception);
                               0 = different every run
    true_distance:=false       RoboBoat detections: ZED depth used as the distance (legacy, D2)
    from_camera:=false         RoboBoat detections: placed from the GPS antenna (legacy, D1)
  e.g.  ros2 launch usv_sim sim.launch.py course:=roboboat_2026 survey:=true seed:=1 \
            record:=map_base
        then: python3 /ws/tools/map_report.py /ws/bags/map_base --course roboboat_2026

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
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

PERCEPTION_KNOBS = ('compass_offset_deg', 'detect_prob', 'max_range_m')
# Measurement-chain fixes of /usv/detections (phase 6 R1.7): launch argument -> parameter
CHAIN_SWITCHES = {'true_distance': 'detection_true_distance',
                  'from_camera': 'detection_from_camera'}
# What ws/tools/run_report.py reads, plus what is handy to replay
RECORD_TOPICS = ('/usv/nav_status', '/usv/thrusters/pwm', '/usv/sim/navsat', '/usv/sim/imu',
                 '/usv/fix', '/usv/heading', '/usv/buoys', '/usv/detections', '/usv/map',
                 '/usv/speed', '/usv/imu')


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

    with open(course_file) as f:
        course_data = yaml.safe_load(f)

    # Course placed elsewhere from the command line: a placed copy in /tmp, used by the
    # world generator and fake_perception alike
    placement = [LaunchConfiguration(n).perform(context)
                 for n in ('course_east', 'course_north', 'course_heading')]
    if any(placement):
        if not all(placement):
            raise RuntimeError('give course_east, course_north and course_heading together')
        course_data.pop('anchor', None)
        course_data['placement'] = {'east': float(placement[0]), 'north': float(placement[1]),
                                    'heading_deg': float(placement[2])}
        course_file = os.path.join('/tmp', f'usv_sim_{course}_placed.yaml')
        with open(course_file, 'w') as f:
            yaml.safe_dump(course_data, f, sort_keys=False)

    # The course's mission section on top of navigator.yaml
    mission = course_data.get('mission', {})
    navigator_overrides = {k: v for k, v in mission.items() if k != 'waypoints'}
    navigator_overrides.update({'waypoints.' + name: [float(x) for x in latlon]
                                for name, latlon in mission.get('waypoints', {}).items()})
    start_task = LaunchConfiguration('start_task').perform(context)
    if start_task:
        navigator_overrides['start_task'] = start_task
    with_navigator = LaunchConfiguration('navigator').perform(context).lower() == 'true'
    navigator_overrides['start_manual'] = \
        LaunchConfiguration('auto').perform(context).lower() != 'true'

    # The declination perception adds is a per-venue setting on the boat (+6 Istanbul,
    # -6 Sarasota); here it follows the course, i.e. a correctly set compass.
    # Experiment knobs: a value given on the command line replaces fake_perception.yaml
    perception_overrides = {
        'course': course,
        'course_file': course_file,
        'magnetic_declination_deg': float(course_data['magnetic_field']['declination_deg']),
    }
    for name in PERCEPTION_KNOBS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            perception_overrides[name] = float(value)
    for arg, param in CHAIN_SWITCHES.items():
        value = LaunchConfiguration(arg).perform(context).lower()
        if value:
            perception_overrides[param] = value == 'true'
    record = LaunchConfiguration('record').perform(context)
    survey = LaunchConfiguration('survey').perform(context).lower() == 'true'
    if survey:
        with_navigator = False        # survey_drive is the only one commanding the thrusters
    seed = LaunchConfiguration('seed').perform(context)
    bridge_overrides = {}
    if seed:
        # Not the same numbers in both nodes; 0 stays 0 (different every run)
        perception_overrides['random_seed'] = int(seed)
        bridge_overrides['random_seed'] = int(seed) + 1000 if int(seed) else 0

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
             parameters=[os.path.join(config, 'sim_bridge.yaml'), bridge_overrides],
             respawn=True, respawn_delay=2.0),
        Node(package='usv_sim', executable='fake_perception', output='screen',
             parameters=[os.path.join(config, 'fake_perception.yaml'), perception_overrides],
             respawn=True, respawn_delay=2.0),
        Node(package='usv_nodes', executable='heading_source', output='screen',
             parameters=[heading_config],
             respawn=True, respawn_delay=2.0),
        # The world map, from power-on (written to /ws/bags/map_<course>.json on Ctrl+C)
        Node(package='usv_nodes', executable='mapper', output='screen',
             parameters=[os.path.join(nodes_config, 'mapper.yaml'),
                         {'map_file': os.path.join('/ws/bags', f'map_{course}.json')}],
             respawn=True, respawn_delay=2.0),
    ]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        # RViz on the map: the markers live in usv_map, tied to RViz's fixed frame 'map'
        ours.append(Node(package='tf2_ros', executable='static_transform_publisher',
                         arguments=['--frame-id', 'map', '--child-frame-id', 'usv_map'],
                         output='log'))
        ours.append(Node(package='rviz2', executable='rviz2', output='log',
                         arguments=['-d', os.path.join(nodes_config, 'map.rviz')]))
    if with_navigator:
        ours.append(Node(package='usv_nodes', executable='navigator', output='screen',
                         parameters=[navigator_config, navigator_overrides],
                         respawn=True, respawn_delay=2.0))
    if survey:
        with open(os.path.join(config, 'fake_perception.yaml')) as f:
            antenna = yaml.safe_load(f)['fake_perception']['ros__parameters'][
                'gnss_antenna_mount_m']
        ours.append(Node(package='usv_sim', executable='survey_drive', output='screen',
                         parameters=[{'course': course, 'course_file': course_file,
                                      'gnss_antenna_mount_m': antenna}]))
    if record:
        ours.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', os.path.join('/ws/bags', record)]
            + list(RECORD_TOPICS),
            output='screen'))

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
        DeclareLaunchArgument('rviz', default_value='false',
                              description='true = open RViz on the world map'),
        DeclareLaunchArgument('course_east', default_value='',
                              description='boat start x in the Gazebo world (m); the layout '
                              'follows. Give all three course_* or none'),
        DeclareLaunchArgument('course_north', default_value='',
                              description='boat start y in the Gazebo world (m)'),
        DeclareLaunchArgument('course_heading', default_value='',
                              description='boat start heading, compass deg (0 north, 90 east)'),
        DeclareLaunchArgument('auto', default_value='false',
                              description='true = navigator starts in AUTO'),
        DeclareLaunchArgument('record', default_value='',
                              description='bag name under /ws/bags; empty = no recording'),
        DeclareLaunchArgument('survey', default_value='false',
                              description='true = no navigator, survey_drive drives the '
                              "course's survey route from the true pose"),
        DeclareLaunchArgument('seed', default_value='',
                              description='sensor noise seed for sim_bridge and '
                              'fake_perception; empty = YAML value'),
    ] + [
        DeclareLaunchArgument(name, default_value='',
                              description='fake_perception override; empty = YAML value')
        for name in PERCEPTION_KNOBS
    ] + [
        DeclareLaunchArgument(arg, default_value='',
                              description=f'true/false: fake_perception {param}; empty = '
                              'YAML value (true)')
        for arg, param in CHAIN_SWITCHES.items()
    ] + [
        OpaqueFunction(function=setup),
    ])
