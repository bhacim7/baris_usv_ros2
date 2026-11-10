# USV ROS 2 Architecture

Status: **approved** (2026-10-22). Section 7 matches the implemented `usv_interfaces`.
Progress is tracked in section 15; open requests in `open-items.md`.

## 1. Purpose and scope

Port the field-proven multiprocess autonomy stack of the USV (TEKNOFEST 2026) to
ROS 2 Jazzy, test it in Gazebo Harmonic / VRX, run it on the boat's Jetson AGX Thor,
and then adapt it to RoboBoat.

- Legacy reference: `github.com/bhacim7/mutliprocess`, commit `5df3f5b` (local copy in
  `~/usv_legacy`, read-only).
- The legacy system stays deployable as a fallback until the ROS 2 version has completed
  the same course at sea.

## 2. Principles

1. **Port, do not rewrite.** Algorithms move into a ROS-free Python library and are wrapped
   by nodes. Behaviour that was tuned on the water must survive the port.
2. **Same nodes in simulation and on the boat.** Only the hardware bridges differ
   (`fc_bridge` / `perception` on the boat, `sim_bridge` / `fake_perception` in simulation).
3. **Every algorithm is testable without ROS.** `usv_core` has unit tests that compare its
   output with the legacy code on the same inputs.
4. **Naming.** Everything in code and in the ROS graph is English. Prefix `usv`.
5. **Measure before tuning.** Simulation validates logic; gains are tuned from field data
   (rosbag), not guessed.

### Invariants carried over from the legacy code

These are not cleaned up during the port, because the gains were tuned against them:

| Invariant | Legacy location | Why it must stay |
|---|---|---|
| `NOMINAL_DT = 0.02` while the loop runs at 25 Hz | `utils/planner.py` | PID and Pure Pursuit D/I gains are scaled by `dt / NOMINAL_DT` |
| Task 3 pixel error `cx - 640` with `Kp_PIXEL = 0.3` | `core/nav_process.py` | Visual-servo gain is tuned in pixels |
| Mixer slew limit uses the previous commanded PWM per channel | `apply_motor_mixer()` | `MAX_PWM_CHANGE = 60` per cycle; state moves into the mixer itself |
| Servo write thinning (`SERVO_MIN_PWM_DELTA`, `SERVO_REFRESH_S`) | `hardware/MainSystem2.py` | Prevents the MAVLink load that once stalled GPS for ~1.5 s |

## 3. Packages

| Package | Build type | Content |
|---|---|---|
| `usv_interfaces` | ament_cmake | Custom messages and services |
| `usv_core` | ament_python | ROS-free library. Shared: `navigation`, `planner` (A*, Pure Pursuit), `costmap` (the planner's grid), `mixer`, `heading`, `detection`, `classes`, `nav_config`, `commands`, `safety`, `relay`, `nav_base`; TEKNOFEST only (prefix `teknofest_`, renamed 2026-11-01): `teknofest_navigator`, `teknofest_mission`, `teknofest_task3`, `teknofest_object_memory`, `teknofest_guidance`, `teknofest_costmap`; RoboBoat only (prefix `roboboat_`, plus `world_map`): `roboboat_run`, `roboboat_navigator`, `roboboat_course`, `roboboat_drive`, `roboboat_tasks`, `world_map`. Files are split by job, not mirrored between the profiles. Unit and golden tests |
| `usv_nodes` | ament_python | `navigator`, `heading_source`, `mapper` (written); `fc_bridge`, `perception`, `telemetry_bridge`, `recorder` (phase 7) |
| `usv_sim` | ament_python | `sim_bridge`, `fake_perception`, `survey_drive`, course files (`courses/*.yaml`) and world generator, boat model, `sim.launch.py` |
| `usv_gz_plugins` | ament_cmake | Gazebo systems (C++): `HullBuoyancy`, buoyancy from the hull mesh. Simulation only; skipped where VRX is absent |
| `usv_bringup` | ament_python | Boat launch files and per-node YAML parameters (replaces `config.py`). Not created yet: until phase 7 the YAML files live in each package's `config/` and the launch in `usv_sim` |
| `usv_learn` | ament_python | Tutorial package, not part of the system |

## 4. Node graph

```
        BOAT                                   SIMULATION
 fc_bridge    perception              sim_bridge    fake_perception
    │  ▲          │                       │  ▲            │
    ▼  │          ▼   same topics, same messages          ▼
 ┌─────────────────────────────────────────────────────────────┐
 │ /usv/fix  /usv/speed  /usv/heading/fc  /usv/heading/zed     │
 │ /usv/buoys (TEKNOFEST)  /usv/detections  /usv/imu (RoboBoat)│
 │        heading_source -> /usv/heading                       │
 │        mapper (/usv/detections -> /usv/map)                 │
 │        navigator (competition: teknofest | roboboat)        │
 │ /usv/thrusters/pwm   /usv/nav_status   /usv/plan            │
 └─────────────────────────────────────────────────────────────┘
       ▲ services                          │
 telemetry_bridge (RFD900x <-> GCS)     recorder (CSV, costmap)
```

| Node | Legacy origin | Responsibility |
|---|---|---|
| `fc_bridge` | `USVController` | Wraps the existing pymavlink class. Publishes position, speed, FC heading and relay state. Applies `ThrusterPwm` with the legacy write thinning. Neutralises thrusters if no command arrives for 0.5 s. Handles relay and emergency stop |
| `perception` | `camera_process` | ZED (pyzed) + YOLO + object memory in one process. Publishes buoys, ZED heading, attitude and FPS. Writes the HUD video (frames never leave the process) |
| `heading_source` | heading selection in `nav_process` | Selects ZED, FC or fused heading (`HEADING_SOURCE`) and publishes `/usv/heading` |
| `navigator` | `nav_process` | 25 Hz timer: mission state machine, local costmap, A*, Task 2 corridor, LOS/PID and Pure Pursuit, mixer. Publishes `ThrusterPwm` and `NavStatus`. Parameter `competition` picks the rule book: `teknofest` (legacy, GPS points, `/usv/buoys`) or `roboboat` (the run from `/usv/map` and `/usv/detections`); GCS services, modes and the emergency latch are shared (`usv_core.nav_base`) |
| `telemetry_bridge` | `telem_process`, `utils/telem.py` | RFD900x JSON protocol unchanged. Turns GCS commands into service calls and echoes `aq` after a successful call |
| `recorder` | CSV + `CostmapRecorder` | Competition deliverables: vehicle CSV (2 Hz, legacy header) and costmap record |
| `sim_bridge` | new | Stands in for `fc_bridge`: commands to Gazebo, FC-side sensors back, with the real hardware's rates and errors (section 10) |
| `fake_perception` | new | Stands in for `perception`: buoys from ground truth within camera FOV and range, ZED heading and attitude, all with ZED-like errors |
| `mapper` | new (RoboBoat) | Persistent world map from power-on, also in MANUAL: fuses `/usv/detections` into landmarks (`usv_core.world_map`), publishes `/usv/map`, writes the map to JSON on shutdown |
| `survey_drive` | new, simulation only | Drives a course's fixed survey route from Gazebo's true pose, for map-quality measurements (section 16) |

`navigator` is deliberately one node. Its internals are split into classes (section 8);
splitting it into several nodes would add message latency inside a 25 Hz control loop for
no benefit, and becomes trivial later once the classes exist.

## 5. Topics

All topics use the default reliable QoS (depth 10) unless marked *latched*
(transient local, depth 1). Camera images are not published.

**One course, several starts and missions (2026-11-10):** `roboboat_2026.yaml` became
`roboboat_2027.yaml`: the 2026 Task 1-2 stay (Proof of Readiness video), the Task 3-6
elements are out (full copy: `usv_sim/test/data/roboboat_2026_full.yaml`), the 2027 course
joins in the same map. A course file may hold `starts` and `missions`; `course.select`
picks one of each (`sim.launch.py start:=readiness mission:=readiness`, or the file's
`default`). The placement names the start it places, so the layout stays put whichever
start is used.

**Namespaces (2026-11-06, RoboBoat 2027, two boats):** every node uses relative names and
runs in its boat's namespace: Azure (AGX Thor) `/azure/...`, Marine (AGX Orin)
`/marine/...` (`sim.launch.py boat:=azure|marine`). The tables write `/usv/...` for "the
boat's namespace". Parameter files use the key `/**` so they apply in any namespace. Bags
recorded before this change have `/usv/...`; the tools read both (`ws/tools/bag_topics.py`).
Frame ids are not prefixed yet: TF is used by RViz only, for one boat; prefixes come with a
common frame for two boats.

| Topic | Type | Publisher | Main subscribers |
|---|---|---|---|
| `/usv/fix` | `sensor_msgs/NavSatFix` | `fc_bridge` / `sim_bridge` | navigator, perception, telemetry_bridge, recorder |
| `/usv/speed` | `std_msgs/Float32` (m/s, ground speed) | `fc_bridge` / `sim_bridge` | navigator, telemetry_bridge, recorder |
| `/usv/heading/fc` | `std_msgs/Float32` (deg) | `fc_bridge` / `sim_bridge` | heading_source |
| `/usv/heading/zed` | `std_msgs/Float32` (deg) | `perception` / `fake_perception` | heading_source |
| `/usv/heading` | `std_msgs/Float32` (deg, 0-360, clockwise from north, declination applied) | `heading_source` | navigator, perception, telemetry_bridge, recorder |
| `/usv/attitude` | `geometry_msgs/Vector3Stamped` (x = roll deg, y = pitch deg) | `perception` / `fake_perception` | recorder |
| `/usv/imu` | `sensor_msgs/Imu`, the ZED's gyro (angular velocity only) | `perception` / `fake_perception` | mapper (yaw rate) |
| `/usv/relay_state` | `std_msgs/Int8` (-1 unknown, 0 off, 1 on) | `fc_bridge` / `sim_bridge` | telemetry_bridge |
| `/usv/buoys` | `usv_interfaces/BuoyArray` | `perception` / `fake_perception` | navigator, recorder |
| `/usv/perception/fps` | `std_msgs/Float32` | `perception` | telemetry_bridge |
| `/usv/thrusters/pwm` | `usv_interfaces/ThrusterPwm` | `navigator` | fc_bridge / sim_bridge, telemetry_bridge, perception (HUD) |
| `/usv/nav_status` | `usv_interfaces/NavStatus` (10 Hz) | `navigator` | telemetry_bridge, recorder, perception (HUD) |
| `/usv/mission/waypoints` | `usv_interfaces/WaypointArray` *latched* | `navigator` | telemetry_bridge |
| `/usv/plan` | `nav_msgs/Path` | `navigator` | RViz, recorder |
| `/usv/detections` | `usv_interfaces/DetectionArray` (every camera frame) | `perception` / `fake_perception` (RoboBoat courses) | mapper |
| `/usv/map` | `usv_interfaces/LandmarkArray` (2 Hz) | `mapper` | navigator (R2-R3), telemetry_bridge, recorder |
| `/usv/map/markers` | `visualization_msgs/MarkerArray`, frame `usv_map` | `mapper` | RViz |

## 6. Services

One service per GCS command. The GCS protocol itself does not change.

| Service | Type | Server | GCS command |
|---|---|---|---|
| `/usv/set_waypoint` | `usv_interfaces/SetWaypoint` | navigator | `set_gps` |
| `/usv/set_task` | `usv_interfaces/SetTask` | navigator | `set_task` |
| `/usv/set_manual` | `std_srvs/SetBool` | navigator | `set_manual` |
| `/usv/set_target_color` | `usv_interfaces/SetTargetColor` | navigator | `set_target_color` |
| `/usv/set_relay` | `std_srvs/SetBool` | fc_bridge | `set_relay` |
| `/usv/emergency_stop` | `std_srvs/Trigger` | fc_bridge (sim: sim_bridge) | `emergency_stop` |
| `/usv/navigator/emergency_stop` | `std_srvs/Trigger` | navigator | `emergency_stop` (telemetry_bridge calls both; two servers cannot share one name) |

`report_status` is not mapped: telemetry is broadcast on the boat's own clock.

## 7. Custom interfaces (draft)

```
# Buoy.msg
uint32 id
uint8 class_id                      # 0 red, 1 yellow, 2 black, 3 orange, 4 green
float64 latitude
float64 longitude
bool position_ok                    # false = only seen beyond MAP_MAX_RANGE_M
uint32 seen_count
builtin_interfaces/Time last_seen
float32 distance_m                  # NaN = unknown
int32 cx                            # last pixel column, -1 = unknown
int32 cy                            # -1 = unknown
int32 area_px

# BuoyArray.msg
std_msgs/Header header
uint32 image_width
Buoy[] buoys

# ThrusterPwm.msg                   # PWM in microseconds; FC channels are fc_bridge parameters
std_msgs/Header header
uint16 rear_left
uint16 rear_right
uint16 front_left
uint16 front_right
uint16 steer
bool force                          # emergency / neutralise: bypass slew limit and write thinning

# NavStatus.msg
std_msgs/Header header
string task_state                   # legacy state names, e.g. TASK1_STATE_ENTER
bool manual
bool mission_active
bool emergency
float64 target_latitude
float64 target_longitude
float32 target_distance_m
float32 advised_course_deg
float32 angle_error_deg
float32 control_error_deg
string target_color
string target_color_source          # "cfg" or "drone"

# Waypoint.msg
uint8 index                         # GPS1..GPS8
float64 latitude
float64 longitude

# WaypointArray.msg
Waypoint[] waypoints

# SetWaypoint.srv
uint8 index
float64 latitude
float64 longitude
---
bool success
string message

# SetTask.srv
string task
---
bool success
string message

# SetTargetColor.srv
string color
---
bool success
string message
```

## 8. Navigator internals

The legacy loop mixes mission logic and motor commands (more than 20 direct
`apply_motor_mixer` calls, many inside the Task 3 search). The port separates them:

```
inputs ──> MissionStateMachine ──Setpoint──> Controller ──effort──> Mixer ──> ThrusterPwm
                  │                              │
             costmap + A*                  LOS / PID / Pure Pursuit
```

- **MissionStateMachine** owns task states, waypoints and the Task 3 search. It outputs a
  `Setpoint` and never touches motors. Setpoint modes:
  `IDLE`, `GOTO(lat, lon)`, `TURN_TO_HEADING(deg, max_pwm)`, `VISUAL_SERVO(pixel_error)`,
  `HOLD_COURSE`, `STOP`.
- **Controller** turns a setpoint into forward and yaw effort, using the legacy LOS guidance,
  Direct-Drive PID, A* + Pure Pursuit, spot-turn and taper logic unchanged.
- **Mixer** is the legacy `apply_motor_mixer` with its slew state held internally.

All three live in `usv_core` and are unit-tested without ROS. The node only wires topics,
services and the 25 Hz timer.

## 9. Safety

| Situation | Behaviour |
|---|---|
| `emergency_stop` | `fc_bridge`: thrusters to 1500 immediately (no slew) and relay off. `navigator`: latched `emergency` state, publishes neutral. Software keeps running; telemetry keeps flowing. The FC is **not** disarmed |
| Leaving emergency | Operator turns the relay on (`set_relay`), then re-selects the task (`set_task`). Relay on alone never makes the boat move: the navigator releases its latch only on a `set_task` received while `/usv/relay_state` reports on; a task chosen before that is stored but does not release. Manual/auto mode is kept; guidance starts fresh |
| Relay | Independent command; can be switched at any time without affecting the software |
| `navigator` stalls or dies | `fc_bridge` neutralises after 0.5 s without `ThrusterPwm`; launch `respawn` restarts a dead node |
| Node receives SIGTERM | Catches `ExternalShutdownException`; `navigator` publishes neutral before exiting |
| Manual mode / mission not started | Mission state does not advance; motors neutral (legacy behaviour) |

Open: detecting a node that is alive but stuck (legacy heartbeat watchdog). Not covered by
`respawn`; to be designed.

## 10. Simulation

- **World:** VRX `nbpark` (Nathan Benderson Park, the RoboBoat venue); course elements added.
  `sydney_regatta` remains available.
- **Boat:** custom model derived from VRX `roboboat01` (1.15 m catamaran) to match the real
  boat (about 1.3 m, about 45 kg): four thrusters, the front two rotating. Rotation uses the
  same mechanism as the WAM-V: a vertical revolute joint plus
  `gz::sim::systems::JointPositionController` (in `roboboat01` this joint exists but is
  locked). First step of the model work is a test of this rotation.
  Fallback: WAM-V with a custom four-thruster YAML layout.
- **Rule:** the software must not be able to tell the simulation from the boat: same topics,
  units, rates, delays and errors. Every stand-in copies exactly one real node, and every
  number lives in a YAML file with its source, to be replaced by a measurement.
- **`sim_bridge`** stands in for `fc_bridge` + Orange Cube + ESCs + MG995 servos. Commands:
  legacy write thinning, 0.5 s timeout, PWM -> N table, steering at servo speed, relay and
  emergency stop services. Sensors: `/usv/fix` (non-RTK drift, 2 Hz as in the video, 1e-7
  deg), `/usv/speed`, `/usv/heading/fc`, `/usv/relay_state`. Gazebo NavSat, IMU and
  odometry reach it through `ros_gz_bridge` on `/usv/sim/*`.
- **`fake_perception`** stands in for `perception` (ZED 2i): `/usv/buoys` (TEKNOFEST courses)
  or `/usv/detections` (RoboBoat courses, every box measured with the R1.7 fixes),
  `/usv/heading/zed`, `/usv/imu` (gyro), `/usv/attitude`, `/usv/perception/fps`. The real `heading_source` then makes
  `/usv/heading`, in simulation as on the boat.
  Buoys come from known world positions within the camera FOV and range, with ZED-like
  errors, because the real YOLO model does not recognise simulated buoys: this layer
  validates mission, planning and control, not detection.
- **Later, for detection:** a Gazebo RGB-D camera with the ZED 2i geometry and a camera-source
  option in `perception` (pyzed on the boat, ROS image topics in simulation); `.svo`
  playback to measure detection and distance on real recordings.
- **Hydrodynamics:** identify real values from field data (coast-down for surge drag,
  turning test for yaw drag).

### Boat model status (`usv_sim/models/usv_boat`, 2026-10-26)

| Item | Value | Source |
|---|---|---|
| Hull | Catamaran, 1.182 x 0.78 m, hull centre lines y = +/-0.26 m | Technical drawing |
| Mass | 45 kg | Team |
| GNSS antenna | x = -0.46, y = 0, z = +0.68 (0.75 m behind, 0.50 m above the ZED) | Measured on the boat 2026-11-01 |
| Thrusters | Front x = +0.363, rear x = -0.493, y = +/-0.245, 7 cm below the keel | Technical drawing |
| Steering | Front pair, +/-45 deg (1100 PWM = 45 deg left, 1900 = 45 deg right) | Team, to be measured |
| Buoyancy | `usv_gz_plugins::HullBuoyancy`: water pressure integrated over the hull mesh, with VRX waves (replaced the VRX cylinder model on 2026-10-26) | STL; heave ~4 050 N/m, roll ~290 Nm/rad, pitch ~340 Nm/rad, reserve 128 kg |
| Centre of gravity | x = -0.045 m, z = +0.05 m | x over the STL centre of buoyancy (level trim ASSUMED); z guessed |
| Yaw drag | nR = 28, nRR = 35 | Back-calculated from the legacy field measurement (spot turn 35-57 deg/s at PWM offset 200); consistent with the 2026-10-14 video (~33-43 deg/s at +/-156) |
| Surge drag | xU = 70.5, xUU = 39.2 | Fitted from the 2026-10-14 video: 1.25-1.31 m/s on straight legs at PWM 1680 (GPS track); to be confirmed with a vehicle CSV |
| Sway drag | yV = 273, yVV = 152 | 6 x the earlier surge guess (side area ~6x frontal area); kept, turning checked |
| Thrust | +91 / -64 N, linear in PWM, propeller 0.09 m | Degz Utras on 7S (8 kgf at 24 V scaled); reverse and linearity ASSUMED |
| Hull shape | Mesh from `ROBO27_MAIN_BODY_SIM.STL` (pre-build CAD), ~8 000 triangles; 0.247 m wide, 0.32 m deep, draft ~0.13 m at 45 kg | STL; hull spacing kept at the drawing's +/-0.26 m (the STL has +/-0.30) |
| Other visuals | Primitive shapes (lids, profiles, box, ZED, bumper, mast) | Photos |

Lessons recorded in the model header:
- VRX `Surface` clamps the submerged depth at `hull_radius`, so at most half of each
  cylinder counts. A radius sized to carry exactly the mass at half depth leaves no reserve:
  the first model sank (centre at z = -0.375). Replaced by `HullBuoyancy`, which has no
  such limit; a visual mesh alone never changes the physics.
- Drag cannot be scaled by mass: it follows hull shape. Equal surge and sway drag (inherited
  from `roboboat01`) made the boat slide sideways instead of turning.
- Steering sign, measured: positive angle (counter-clockwise from above) on a **rear**
  thruster turns the boat right; on a **front** thruster it turns the boat left. With the
  team's definition (1900 PWM = thrusters right = boat turns right), `sim_bridge` maps
  1900 PWM to -0.785 rad.

Gazebo topics (`gz.msgs.Double`):
`/usv_boat/thrusters/{rear_left,rear_right,front_left,front_right}/thrust` (N) and
`/usv_boat/thrusters/{front_left,front_right}/pos` (rad).

## 11. Recording

| Output | Producer |
|---|---|
| Processed camera video with HUD | `perception` |
| Vehicle telemetry CSV (2 Hz, legacy header) | `recorder` |
| Costmap record | `recorder` |
| Full run for replay and analysis | `ros2 bag record` (simulation: `sim.launch.py record:=<name>`; reports `ws/tools/run_report.py`, `ws/tools/map_report.py`) |
| World map (JSON, every landmark) | `mapper`, on shutdown |

## 12. Parameters

`config.py` becomes one YAML file per node in `usv_bringup/config/`. Values are copied
unchanged from commit `5df3f5b`, including their comments. Waypoints change at runtime
only through `/usv/set_waypoint`; the live set is published on `/usv/mission/waypoints`,
so every node sees the same values (the legacy `spawn` copy problem disappears).

## 13. Communication

- RFD900x link and JSON protocol unchanged (short keys, sequence `q`, ack `aq`, waypoint
  mini packets, 0.3 s broadcast).
- Wi-Fi is allowed at RoboBoat. A network GCS or remote ROS 2 tools are possible later;
  heavy topics stay on the boat.

## 14. Deferred decisions

| Item | When |
|---|---|
| MAVROS instead of the wrapped `USVController` | Separate experiment after phase 7 |
| Heading sampled at image time for buoy positions (heading dominates the measured ~0.5 m error) | After parity with legacy |
| Task 3 waypoints and search inputs | Updated in the field |
| RoboBoat-specific tasks and reporting | Adaptation phase, from the RoboBoat rules |
| Stuck-node detection | Before sea trials |
| ZED SDK and YOLO/TensorRT on Jetson AGX Thor | Phase 6 |
| Legacy issues to fix after parity, each measured in simulation first: ZED depth (distance along the optical axis) used as the range to a buoy, up to ~30 % short at 45 deg; small far boxes: the 5 x 5 depth window spills below the box onto the water, so a buoy ~20 m away is measured at 11-14 m and can even pass the 12 m trust gate; buoys placed from the GNSS antenna instead of the camera (0.9 m); HUD speed (VFR_HUD) reads ~half the GPS speed and feeds the Pure Pursuit lookahead; no guard against a GPS freeze + jump (1.9 s, 8 m in the 2026-10-14 video); late buoy avoidance (three contacts in that run); ZED magnetometer not calibrated on the boat (heading error ~4-8 deg, heading dependent); attitude from the ZED pose probably in the camera's IMAGE frame (CSV roll/pitch may be pitch/yaw) | After parity |
| Legacy navigator quirks kept for parity (phase 5): pre-flight passes on GPS latitude != 0 and heading != 0 only (no fix quality, data age, waypoints or perception check; a silent heading source publishes 0.0); Task 3 servo assumes the image centre at 640 px; in some Task 3 transition cycles no thruster command is issued (the last one holds for a cycle); the Task 3 anchor accepts unconfirmed far positions | After parity |

## 15. Roadmap

| Phase | Work | Done when |
|---|---|---|
| 3 | This document | Approved |
| 4 | `usv_interfaces`; `usv_core` (`navigation`, `planner`) with tests; boat model; `sim_bridge`; `fake_perception` | Boat driven manually in simulation; buoys on `/usv/buoys` |

Phase 4 progress (2026-10-30): **done criteria met** (boat driven in simulation through ROS; buoys on `/usv/buoys`).

| Item | State |
|---|---|
| `usv_interfaces` | Done |
| `usv_core`: `navigation`, `planner`, `object_memory`, `mixer`, `heading` (ZED filter, declination as a parameter, source selection), `detection` (box + depth -> buoy position, whole camera frame -> object memory) | Done, 522 equivalence and behaviour tests passing |
| `usv_nodes`: `heading_source` | Done and tested: first node that runs on the boat and in simulation |
| Boat model | Four thrusters, steerable front pair; buoyancy, yaw and surge drag calibrated (surge from the legacy video); hull mesh from the CAD STL |
| `sim_bridge` | Done and tested: commands (relay, emergency stop, timeout) and FC-side sensors |
| Course `golcuk_2026_10_14` (provisional, from the video) + world generator | Written: `courses/*.yaml` is the single source; `make_course_world.py` builds the world (site origin, magnetic field, buoys) |
| `fake_perception` | Done and tested: heading, attitude, FPS, buoys (simulated ZED + YOLO -> the boat's own detection code -> `/usv/buoys` at 30 Hz) |

Phase 5 progress (2026-10-31): the navigator brain is ported to `usv_core` (`nav_config`, `costmap`, `guidance`, `mission`, `task3`, `commands`, `navigator`, `safety`, `relay`; since 2026-11-01 `teknofest_mission`, `teknofest_task3`, `teknofest_navigator`) and matches the legacy loop cycle for cycle on every golden scenario (`test/golden/`). Deliberate changes: the emergency stop latches as in section 9 (neutral from the same cycle; released by relay on + `set_task`) instead of ending the program; relay retries move to `fc_bridge`. 5.5: the `navigator` node runs in `sim.launch.py`.

Phase 5 **done criteria met** (2026-11-01): Golcuk course in Gazebo from Task 1 to FINISHED, no corridor contact with the default simulated sensors. 5.6 comparison with the 2026-10-14 video (`ws/tools/run_report.py`, `ws/tools/golcuk_2026_10_14_reference.yaml`), single runs each:

| Run | Task 2 start -> FINISHED | Task 2 mean speed | Corridor contacts | Closest corridor buoy |
|---|---|---|---|---|
| Video (legacy boat) | ~68 s (red hit) | ~1.0-1.3 m/s | 3 (#18, #21, one more) | - |
| Simulation, default sensors | 126 s | 0.63 m/s | 0 | #19 +0.83 m |
| Detections only within 10 m | 108 s | 0.73 m/s | 0 | #21 +0.36 m |
| YOLO misses half the frames | 117 s | 0.68 m/s | 0 | #4 +0.96 m |
| ZED compass +8 deg | 125 s | 0.68 m/s | **1** (#17, -0.01 m) | |

Reading: the hull itself reaches 1.33 m/s at 1680 PWM (straight-line test), so the slower simulated run is the navigator weaving round every buoy it sees. Late detection makes the path straighter, faster and closer to the buoys the real boat hit; a compass error produces the first contact (buoys are placed with the heading, so 8 deg puts a buoy 10 m away 1.4 m sideways). Per-frame misses change little (the object memory bridges them). Consistent with the real contacts coming from perception and compass error rather than from the code - but single runs with random noise, not proof. The remaining speed gap in turns depends on unmeasured hardware (steering angle range, assumed +/-45 deg; sway drag, guessed).

Next: phase 6.

Remaining phases:

Reordered on 2026-11-01: the boat cannot go in the water for a while, so the RoboBoat
adaptation comes first, in simulation; the boat phases follow and then run alongside it.

| Phase | Work | Done when |
|---|---|---|
| 5 | `navigator` (mixer, controller, mission) | Done: Tasks 1, 2, 3 completed in simulation |
| 6 | RoboBoat in simulation (section 16): course and 13-class perception (R1), persistent map from power-on (R1), competition profile in the navigator (R2), Tasks 1, 2, 3, 6, 5 (R3), RoboCommand reporting (R4), fix-after-parity items measured in simulation (R5) | Qualifying criteria met in simulation: core of two tasks, advanced of two, two in sequence, heartbeat |
| 7 | Boat image (arm64, Thor), `fc_bridge`, `perception`, `heading_source`, `telemetry_bridge`, `recorder`, audio | Sensors flowing on land and at the pier; GCS connects unchanged |
| 8 | Sea trials, then alongside the simulation | The ROS 2 version completes RoboBoat tasks on the water |

Phase 6 progress (2026-11-01): R1.1-R1.6 done, R1.7 D1 + D2 done (measurement-chain target met), D4 + D5 + D6 done, R1 closed; R2.1 (navigator base split, RoboBoat run skeleton, profile switch) done; R2.2 (course model on the map: Task 1 gates and gate frame, channel and debris, Task 3, marina slips, Task 6 zones; `ws/tools/course_report.py`) done: on `survey1_d6` with the GPS true, gates and channel within 0.11-0.14 m, frame 0.8 deg, hints 0.24-0.61 m; on the boat's own map (GPS drift) gates 0.8-1.5 m, frame 4.6 deg, hints 0.9-4.0 m. The best slip is given only once the whole marina is seen. Next for R2.4: take the frame's direction from the compass while passing straight through the Task 1 gates, free of the GPS; R2.3 (`roboboat_drive`: one Pure Pursuit path follower with the TEKNOFEST tuning, moves DriveTo with A* on the map's landmarks, PassGate and Circle refined from the camera frame, Search) written; `guidance` / `costmap` split into the shared grid and `teknofest_guidance` / `teknofest_costmap`; R2.4 (`roboboat_tasks`: Task 1 entry through both gates with the frame's direction from the compass, return to the gates with A*, exit the other way; `ws/tools/roboboat_report.py`: gate crossings, contacts and steering from the ground truth) done in Gazebo: entry and exit clean, no contact (t1_run1/2), but weaving on straight legs (~10 s period, +/-30-40 deg) with the TEKNOFEST Pure Pursuit look-ahead (~1.3 m): RoboBoat gets its own `roboboat.drive.lookahead_m`: 1.3 / 2.5 / 4.0 m measured on Task 1 (run 54.6 / 46.0 / 43.5 s), 4.0 chosen, 2.0 m on A*-bent paths (4.0 cut a buoy wall's corner). **R2 done** (2026-11-01): Task 1 entry and exit clean with no contact on three seeds (runs 43.5 / 45.5 / 46.5 s). Next: R3, Task 2; D3 deferred (fix age now 0.14 m; the real GPS rate is in the unseen `.param`). 13 RoboBoat classes; one course reader for
the world generator and `fake_perception`; `roboboat_2026` at Nathan Benderson Park (vessels,
docks, indicators); per-frame detections and the persistent world map (`mapper`, RViz); the
fixed survey route and `map_report.py`. First measurement in section 16. Next: R1.7, the
measurement-chain fixes D2 + D1, then D4, D5, D3.

## 16. RoboBoat decisions (2026-11-01)

Source: RoboBoat 2026 Team Handbook (February 2026, Nathan Benderson Park). The 2027 handbook
is expected around November 2026; differences are adapted when it appears.

| Decision | Why |
|---|---|
| No waypoints dropped by eye from the GCS | In 2026 they cost time and some were wrong (even outside the course). Tasks are found from the map |
| GPS is used for: placing every detection on the map (fix + heading + ZED range + pixel bearing), the boat's own position, the lat/lon the reports require, and remembering places (entry gate for the exit, Task 3 yellow buoy and marina for Task 6) | Everything the GPS can do well without operator input |
| Mapping starts at power-on, also in MANUAL | Courses are laid out in any orientation; driving by hand to the Task 1 start already maps what the boat sees |
| The vehicle receives no control information in autonomous mode (handbook 5.3.1) | The whole run is decided on board; the GCS only listens |
| Class names in English, the dataset renamed to match | Naming principle (section 2) |
| Task 4 (water and ball) stays on its own Orin Nano for now | Not part of this system yet; may move to the Thor later |
| Docking with the camera only; a 3D LiDAR may come later; the legacy 2D RPLidar is never used | Hardware decision |
| The boat is the four-thruster Thor boat of this port | Same model as in simulation |
| Task order: 1, 2, 3, 6, 5, 4 | Task 1 is mandatory; Task 2 is closest to TEKNOFEST Task 2 and scores most |
| One `navigator` node, two rule books (`competition: teknofest / roboboat`); commands, modes, emergency latch and relay shared in `usv_core.nav_base` | The safety logic exists once; TEKNOFEST can be removed later by dropping its rule book |
| Task 1 "entry" and "exit" are the same two gates, passed at the start and, the other way, at the end. The boat is put down at least 6 ft before the first gate, facing it, so it sees both gates at the start; the map brings it back to them for the exit | Handbook 3.2.1, 3.5; team practice |
| Where tasks are: optional hints given before the run as "ahead_m,left_m" from the first Task 1 gate (seen from the shore; the jury gives no map), else the boat explores; it recognises the task itself either way | No GPS points by eye; hints relative to the gates carry no GPS error |
| Microphone for Task 6 exists (worked in 2026 with an FFT); ported as a node in phase 7, a simulated sound event until then | |

### Map quality (R1.6)

Measured, not guessed: `sim.launch.py survey:=true seed:=<n> record:=<name>` drives the course
file's `survey` route from Gazebo's true pose (usv_sim/survey.py, the same path every run, no
navigator), and `ws/tools/map_report.py` compares every sighting and the final map with the
truth. Each sighting is recomputed with one input at a time swapped for the truth (GPS fix,
heading, ZED range, antenna instead of camera as origin), so the error each input brings is
known before anything is changed.

Target (revised 2026-10-31, after the first measurement), two numbers:
- Measurement chain, with the GPS swapped for the truth: confirmed landmarks seen from within
  6 m, mean error < 0.4 m, none above 0.8 m. This is what perception and the mapper control.
- Absolute map error: reported, no target. Without RTK the receiver's own drift sets it
  (~1 m); improving it needs RTK or visual odometry. The first target (< 0.5 m absolute) could
  not be met by any fix in this chain.

First measurement (`survey_base`, seed 1, 2026-10-31; 8 383 sightings within 12 m):

| | Mean sighting error |
|---|---|
| As recorded | 2.75 m (along the line of sight -2.56 m: placed too close) |
| Only the range wrong | 1.74 m: geometry, not ZED noise (-0.28 m at 0-15 deg off-axis, -3.41 m beyond 45 deg) |
| Only the GPS wrong | 1.01 m: receiver drift 1.02 m, fix age 0.25 m (mean age 0.36 s) |
| Only the origin wrong | 0.94 m (antenna 0.89 m behind the camera in the model then; measured 0.75 m on 2026-11-01) |
| Only the heading wrong | 0.34 m: 0.35 deg straight, mean 9.2 deg (up to 20 deg) above 30 deg/s yaw rate |
| All inputs true | 0.06 m (the tool's own residual) |

Map: 259 landmarks, 121 confirmed, 202 more than 3 m from any object of their kind; confirmed
landmarks seen from within 6 m: mean 1.99 m, max 2.68 m.

D1 + D2 (R1.7, 2026-11-01), same route and seed, legacy chain re-run for the comparison:

| | Legacy chain (`survey_legacy`) | D1 + D2 (`survey_d1d2`) |
|---|---|---|
| Mean sighting error, as recorded | 3.06 m | 1.39 m |
| Same, GPS true | 2.43 m | 0.52 m |
| Range error left (distance used - true) | -0.56 to -3.20 m, worse off-axis | -0.18 to -0.30 m at every angle (front surface instead of centre) |
| Absolute map, confirmed seen from <= 6 m | mean 2.33 m, max 2.89 m | mean 1.06 m, max 2.83 m (GPS drift ~1.25 m this run) |
| Measurement-chain map, same | mean 1.74 m, max 2.66 m: FAIL | **mean 0.28 m, max 0.63 m: PASS** |
| Confirmed ghosts (absolute / chain) | 80 / 66 | 12 / 9 |

What is left in the chain: heading lag in fast turns (mean |error| 9 deg above 30 deg/s), the
~0.2 m front-surface offset, and duplicates (130 unconfirmed ghosts). These runs used the
antenna at 0.89 m behind the camera, before the tape measurement; model and settings now use
0.75 m.

D4 + D5 (R1.7), first on the recorded `survey_d1d2` sightings mapped again with the settings
off and on (`map_report.py --map`), then a new run with every fix and the measured antenna
(`survey_r17`):

| | `survey_d1d2`, D4 + D5 off | `survey_d1d2`, on | `survey_r17`, on |
|---|---|---|---|
| Chain map: confirmed seen from <= 6 m | mean 0.28 m, max 0.63 m | mean 0.26 m, max 0.55 m | **mean 0.25 m, max 0.42 m** |
| Chain map: confirmed duplicates | 20 | 7 | 11 |
| Chain map: confirmed ghosts | 9 | 5 | **2** |
| Absolute map: confirmed duplicates / ghosts | 29 / 12 | 14 / 6 | 9 / 13 |
| Absolute map: confirmed seen from <= 6 m | mean 1.06 m | mean 1.04 m | mean 1.62 m |

The real black buoys 0.65 m from red ones (#10, #13) stay confirmed with D5 on. In
`survey_r17` the GPS receiver drifted more (mean 1.51 m, up to 3.6 m), which alone sets the
absolute map: buoys seen while the fix was ~3 m off went to separate landmarks (e.g. #4). The
measurement chain is now limited by the front-surface offset (-0.18 to -0.33 m) and the
heading lag above 30 deg/s (mean 5.5 deg); fix age adds 0.14 m.

D6 (`survey1_d6`, every fix on): range error left -0.08 to -0.15 m (poles -0.12, buoys -0.09;
flat marks -0.2 to -0.35 m, seen obliquely). Chain map: confirmed seen from <= 6 m **mean
0.14 m, max 0.30 m**; 3 confirmed duplicates, 3 confirmed ghosts. Absolute map: mean 1.17 m,
max 2.20 m with the receiver drifting 1.36 m on average. R1 is done: what is left of the map
error is the GPS receiver, which needs real data first (open-items 4f).

Fixes to the RoboBoat measurement chain (`measure_boxes`, the mapper), each a parameter so the
report measures it alone; the TEKNOFEST chain and the golden tests are not touched:

| | Fix | Error it removes |
|---|---|---|
| D1 | Place from the camera, not the GPS antenna (0.75 m behind and 0.50 m above it, measured 2026-11-01). Done (R1.7): `detection_from_camera`, `camera_from_antenna_m` | Constant along-heading offset |
| D2 | ZED depth -> distance: depth / cos(pixel angle). Done (R1.7): `detection_true_distance` | Objects off the image centre placed too close (~30 % at 45 deg) |
| D3 | Boat position at the frame time: last fix + speed x age along the heading | GPS age (2 Hz + delay, up to ~0.8 m at 1.3 m/s) |
| D4 | Turning: sigma += distance x tan(heading_lag_s x yaw rate); no new landmark above new_landmark_max_turn_dps. Written (R1.7), yaw rate from `/usv/imu` | Heading filter lag in turns, duplicates |
| D6 | + the class radius (`usv_core.classes.OBJECT_RADIUS_M`, handbook sizes; flat marks 0): the ZED sees the front, the map wants the centre. Done (R1.7): `detection_add_radius` | The -0.2 to -0.3 m left after D2 |
| D5 | A landmark is not confirmed while a landmark of another kind within confusion_radius_m has confusion_ratio x its sightings and the two were never in the same frame (a real neighbour, e.g. a black buoy 0.65 m from a red one, is seen in the same frames as a box of its own). Written (R1.7) | Ghosts from YOLO class confusion |

Not fixable here: the receiver's own drift without RTK (~1 m wandering over a minute); the
map says where to look, the camera says exactly where it is.

