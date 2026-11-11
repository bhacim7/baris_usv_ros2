"""
The whole simulation with one command.

    ros2 launch usv_sim sim.launch.py                      # Golcuk course, GUI, running
    ros2 launch usv_sim sim.launch.py paused:=true         # start paused (press play)
    ros2 launch usv_sim sim.launch.py gui:=false           # no Gazebo window (faster)
    ros2 launch usv_sim sim.launch.py course:=<name>       # another usv_sim/courses/<name>.yaml
    ros2 launch usv_sim sim.launch.py start_task:=TASK1_STATE_ENTER   # not the course's own
    ros2 launch usv_sim sim.launch.py navigator:=false     # drive by hand (ros2 topic pub)
    ros2 launch usv_sim sim.launch.py rviz:=true           # also RViz on the world map
    ros2 launch usv_sim sim.launch.py boat:=marine         # the boat's name = its namespace
    ros2 launch usv_sim sim.launch.py course:=roboboat_2027 course_east:=182.5 \
        course_north:=952.2 course_heading:=180    # put the course elsewhere: boat start
                                                   # (Gazebo x, y) and heading; layout follows

RoboBoat 2027 (course:=roboboat_2027): one course file, several starts and missions
    start:=readiness           where the boat starts: before the 2026 Task 1 gates
    start:=competition         the start of the 2027 course: the Task 1 field ahead
    layout:=primer             Task 1 as drawn in the 2027 Primer (the course file's
                               default); layout:=<n> a random field, seed n
    level:=core                Task 1 level with a random layout: core, advanced (the EXIT
                               moves while the boat is in the field), disruptive (and up to
                               two colours swap)
    The marina window lights after the boat has stayed 3 s in the green bay; the water
    hit is, until the turret (roadmap step 7), a course event given by hand:
        ros2 topic pub --once /azure/sim/event std_msgs/msg/String "{data: fire_out}"
    mission:=readiness         what the navigator runs: 2026 Task 1 + Task 2, for the Proof
                               of Readiness video (`roboboat_2027` joins with roadmap step 6)
    empty = the course file's `default`

Experiments (phase 5.6), one command each:
    auto:=true                 navigator starts in AUTO (no set_manual call needed)
    record:=<name>             ros2 bag record into /ws/bags/<name> (the topics run_report
                               needs); the name must not exist yet
    compass_offset_deg:=8.0    fake_perception: ZED compass reads 8 deg too high
    detect_prob:=0.5           fake_perception: YOLO finds a buoy in half of the frames
    max_range_m:=10.0          fake_perception: no detections beyond 10 m
  e.g.  ros2 launch usv_sim sim.launch.py auto:=true record:=exp_compass8 compass_offset_deg:=8.0

GCS link (RoboBoat 2027 step 5a):
    link:=true                 telemetry_bridge on the boat, over UDP on this computer (the
                               Ubiquiti network) and a pretend RFD radio (fake_radio.py:
                               /tmp/usv_rfd_gcs for the GCS, /tmp/usv_rfd_<boat> for the
                               boat); talk to it with ws/tools/gcs_cli.py

Map measurements (phase 6 R1.6):
    survey:=true               no navigator; survey_drive drives the course's survey route
                               from Gazebo's true pose, the same path every run
    seed:=<n>                  the same sensor noise every run (sim_bridge, fake_perception);
                               0 = different every run
    true_distance:=false       RoboBoat detections: ZED depth used as the distance (legacy, D2)
    from_camera:=false         RoboBoat detections: placed from the GPS antenna (legacy, D1)
    add_radius:=false          RoboBoat detections: front of the object, no radius (D6)
  e.g.  ros2 launch usv_sim sim.launch.py course:=roboboat_2027 survey:=true seed:=1 \
            record:=map_base
        then: python3 /ws/tools/map_report.py /ws/bags/map_base --course roboboat_2027

What starts, in this order:
  1. make_course_world.py builds the world from the course file into /tmp, so the world
     can never be older than the course (no more "remember to regenerate").
  2. When that has finished: Gazebo (ros_gz_sim), ros_gz_bridge, sim_bridge,
     fake_perception, heading_source and navigator, each with its YAML file from config/.
     The navigator also gets the course file's `mission` section (start task, waypoints,
     Task 3 colour). It starts in MANUAL, as on the boat:
         ros2 service call /azure/set_manual std_srvs/srv/SetBool "{data: false}"
     Our nodes restart by themselves if they die (respawn), as on the boat.

The boats (RoboBoat 2027): Azure (AGX Thor) and Marine (AGX Orin). Our nodes and the bridge
run in the boat's namespace, boat:=azure (default) or boat:=marine, so every topic and
service is /azure/... or /marine/... (/azure/fix, /azure/set_manual). One boat in Gazebo
for now: the name only changes the namespace.

Ctrl+C stops everything.
"""

import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from usv_core import boat as usv_boat
from usv_sim import course as course_mod
import yaml

PERCEPTION_KNOBS = ('compass_offset_deg', 'detect_prob', 'max_range_m')
# Measurement-chain fixes of /usv/detections (phase 6 R1.7): launch argument -> parameter
CHAIN_SWITCHES = {'true_distance': 'detection_true_distance',
                  'from_camera': 'detection_from_camera',
                  'add_radius': 'detection_add_radius'}
# What ws/tools/run_report.py reads, plus what is handy to replay (in the boat's namespace)
RECORD_TOPICS = ('nav_status', 'thrusters/pwm', 'sim/navsat', 'sim/imu', 'fix', 'heading',
                 'buoys', 'detections', 'map', 'speed', 'imu', 'map/markers')


def setup(context):
    share = get_package_share_directory('usv_sim')
    course = LaunchConfiguration('course').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    # The boat's profile (usv_nodes/config/boats/<boat>.yaml): its name is the namespace
    boat = LaunchConfiguration('boat').perform(context)
    boats = os.path.join(get_package_share_directory('usv_nodes'), 'config', 'boats')
    if not os.path.isfile(os.path.join(boats, boat + '.yaml')):
        known = sorted(f[:-len('.yaml')] for f in os.listdir(boats) if f.endswith('.yaml'))
        raise RuntimeError(f'boat:={boat}: no such profile; there are {", ".join(known)}')
    with open(os.path.join(boats, boat + '.yaml')) as f:
        profile = usv_boat.load(yaml.safe_load(f))

    course_file = os.path.join(share, 'courses', course + '.yaml')
    world_file = os.path.join('/tmp', f'usv_sim_{course}.sdf')
    config = os.path.join(share, 'config')
    nodes_config = os.path.join(get_package_share_directory('usv_nodes'), 'config')
    heading_config = os.path.join(nodes_config, 'heading_source.yaml')
    navigator_config = os.path.join(nodes_config, 'navigator.yaml')

    with open(course_file) as f:
        course_data = yaml.safe_load(f)

    # One start and one mission of the course (roboboat_2027 has several; empty = its
    # default). A start that is not the default, or a course placed elsewhere from the
    # command line: a copy in /tmp, used by the world generator and fake_perception alike
    start = LaunchConfiguration('start').perform(context)
    mission_name = LaunchConfiguration('mission').perform(context)
    try:
        course_data = course_mod.select(course_data, start or None, mission_name or None)
    except ValueError as e:
        raise RuntimeError(f'course:={course}: {e}')
    placement = [LaunchConfiguration(n).perform(context)
                 for n in ('course_east', 'course_north', 'course_heading')]
    if any(placement):
        if not all(placement):
            raise RuntimeError('give course_east, course_north and course_heading together')
        course_data.pop('anchor', None)
        course_data['placement'] = {'east': float(placement[0]), 'north': float(placement[1]),
                                    'heading_deg': float(placement[2])}
    # The Task 1 field (RoboBoat 2027): another seed or level
    field_args = {k: LaunchConfiguration(k).perform(context) for k in ('layout', 'level')}
    if any(field_args.values()):
        if 'task1_field' not in course_data:
            raise RuntimeError(f'course:={course} has no task1_field for layout:= / level:=')
        course_data['task1_field'] = dict(course_data['task1_field'])
        if field_args['layout']:
            course_data['task1_field']['layout'] = field_args['layout']
        if field_args['level']:
            course_data['task1_field']['level'] = field_args['level']
    if any(placement) or start or any(field_args.values()):
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
    # The rule book follows the course: RoboBoat classes -> the RoboBoat run (phase 6 R2)
    navigator_overrides['competition'] = (
        'roboboat' if course_data.get('classes') == 'roboboat' else 'teknofest')
    lookahead = LaunchConfiguration('lookahead_m').perform(context)
    if lookahead:
        navigator_overrides['roboboat.drive.lookahead_m'] = float(lookahead)
    channel_lookahead = LaunchConfiguration('channel_lookahead_m').perform(context)
    if channel_lookahead:
        navigator_overrides['roboboat.drive.channel_lookahead_m'] = float(channel_lookahead)
    # The course file's hints are worked out from its layout, i.e. exact; a team member on
    # the shore is metres off. hint_error_m moves each hint that far in a random direction
    # (the same directions for the same seed), to test the boat finds the task itself.
    hint_error = LaunchConfiguration('hint_error_m').perform(context)
    if hint_error and float(hint_error) > 0:
        rng = random.Random(int(LaunchConfiguration('seed').perform(context) or 0) + 7)
        for key in [k for k in navigator_overrides if k.startswith('roboboat.hint.')]:
            text = str(navigator_overrides[key]).strip()
            if not text:
                continue
            ahead, left = (float(v) for v in text.split(','))
            a = rng.uniform(0.0, 2.0 * math.pi)
            navigator_overrides[key] = (f'{ahead + float(hint_error) * math.cos(a):.2f},'
                                        f'{left + float(hint_error) * math.sin(a):.2f}')
    navigator_overrides['start_manual'] = \
        LaunchConfiguration('auto').perform(context).lower() != 'true'
    # What this boat can do (its profile): the mission manager refuses other tasks
    navigator_overrides['boat_capabilities'] = ','.join(profile.capabilities())

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
        namespace=boat,
        parameters=[{'config_file': os.path.join(config, 'ros_gz_bridge.yaml')}],
        output='screen')

    ours = [
        Node(package='usv_sim', executable='sim_bridge', namespace=boat, output='screen',
             parameters=[os.path.join(config, 'sim_bridge.yaml'), bridge_overrides],
             respawn=True, respawn_delay=2.0),
        Node(package='usv_sim', executable='fake_perception', namespace=boat, output='screen',
             parameters=[os.path.join(config, 'fake_perception.yaml'), perception_overrides],
             respawn=True, respawn_delay=2.0),
        Node(package='usv_nodes', executable='heading_source', namespace=boat, output='screen',
             parameters=[heading_config],
             respawn=True, respawn_delay=2.0),
        # The world map, from power-on (written to /ws/bags/map_<course>.json on Ctrl+C)
        Node(package='usv_nodes', executable='mapper', namespace=boat, output='screen',
             parameters=[os.path.join(nodes_config, 'mapper.yaml'),
                         {'map_file': os.path.join('/ws/bags', f'map_{course}.json')}],
             respawn=True, respawn_delay=2.0),
    ]
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        # RViz on the map: the markers live in usv_map, tied to RViz's fixed frame 'map'
        ours.append(Node(package='tf2_ros', executable='static_transform_publisher',
                         arguments=['--frame-id', 'map', '--child-frame-id', 'usv_map'],
                         output='log'))
        # map.rviz shows Azure's markers; another boat gets a copy with its namespace
        rviz_config = os.path.join(nodes_config, 'map.rviz')
        if boat != 'azure':
            with open(rviz_config) as f:
                text = f.read().replace('/azure/', f'/{boat}/')
            rviz_config = os.path.join('/tmp', f'usv_map_{boat}.rviz')
            with open(rviz_config, 'w') as f:
                f.write(text)
        ours.append(Node(package='rviz2', executable='rviz2', output='log',
                         arguments=['-d', rviz_config]))
    if with_navigator:
        ours.append(Node(package='usv_nodes', executable='navigator', namespace=boat,
                         output='screen',
                         parameters=[navigator_config, navigator_overrides],
                         respawn=True, respawn_delay=2.0))
    if survey:
        with open(os.path.join(config, 'fake_perception.yaml')) as f:
            antenna = yaml.safe_load(f)['/**']['ros__parameters'][
                'gnss_antenna_mount_m']
        ours.append(Node(package='usv_sim', executable='survey_drive', namespace=boat,
                         output='screen',
                         parameters=[{'course': course, 'course_file': course_file,
                                      'gnss_antenna_mount_m': antenna}]))
    link = LaunchConfiguration('link').perform(context).lower() == 'true'
    if link:
        ours.append(Node(package='usv_nodes', executable='telemetry_bridge', namespace=boat,
                         output='screen',
                         parameters=[os.path.join(nodes_config, 'telemetry_bridge.yaml'),
                                     {'boat_number': profile.number,
                                      'udp_listen_port': profile.udp_port,
                                      'gcs_host': '127.0.0.1',
                                      'serial_port': f'/tmp/usv_rfd_{boat}'}],
                         respawn=True, respawn_delay=2.0))
    if record:
        ours.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', os.path.join('/ws/bags', record)]
            + [f'/{boat}/{topic}' for topic in RECORD_TOPICS],
            output='screen'))

    start_rest = RegisterEventHandler(OnProcessExit(
        target_action=make_world, on_exit=[gazebo, bridge] + ours))

    radio = []
    if link:
        # The pretend RFD network, before the boat's bridge opens its end
        radio.append(ExecuteProcess(
            cmd=['python3', os.path.join(share, 'scripts', 'fake_radio.py'), 'gcs', boat],
            output='screen'))
    return radio + [
        LogInfo(msg=f'Boat {profile.name} (number {profile.number}, {profile.computer}); can: '
                + (', '.join(profile.capabilities()) or 'nothing extra')),
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
        DeclareLaunchArgument('boat', default_value='azure',
                              description='azure or marine: the namespace of our nodes'),
        DeclareLaunchArgument('start', default_value='',
                              description="one of the course's starts (roboboat_2027: "
                              "readiness, competition); empty = the course's default"),
        DeclareLaunchArgument('mission', default_value='',
                              description="one of the course's missions (roboboat_2027: "
                              "readiness); empty = the course's default"),
        DeclareLaunchArgument('layout', default_value='',
                              description='Task 1: primer (as drawn) or a random field seed; '
                              "empty = the course file's"),
        DeclareLaunchArgument('level', default_value='',
                              description='Task 1 level: core, advanced, disruptive; empty = '
                              "the course file's"),
        DeclareLaunchArgument('link', default_value='false',
                              description='true = telemetry_bridge over UDP and a pretend '
                              'RFD radio (GCS link)'),
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
        DeclareLaunchArgument('lookahead_m', default_value='',
                              description='RoboBoat path following: Pure Pursuit look-ahead '
                              '(m); empty = navigator.yaml'),
        DeclareLaunchArgument('channel_lookahead_m', default_value='',
                              description='RoboBoat: Pure Pursuit look-ahead through the '
                              'Task 2 channel (m); empty = navigator.yaml'),
        DeclareLaunchArgument('hint_error_m', default_value='',
                              description="RoboBoat: move each of the course file's hints "
                              'this far in a random direction (seeded); empty = exact'),
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
