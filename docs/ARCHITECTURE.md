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
| `usv_core` | ament_python | ROS-free library: `navigation`, `planner`, `object_memory`, `mixer`, `heading`, `detection`, `mission`, `controller`. Unit tests |
| `usv_nodes` | ament_python | `navigator`, `fc_bridge`, `perception`, `heading_source`, `telemetry_bridge`, `recorder` |
| `usv_sim` | ament_python | `sim_bridge`, `fake_perception`, boat model, worlds |
| `usv_gz_plugins` | ament_cmake | Gazebo systems (C++): `HullBuoyancy`, buoyancy from the hull mesh. Simulation only; skipped where VRX is absent |
| `usv_bringup` | ament_python | Launch files and per-node YAML parameters (replaces `config.py`) |
| `usv_learn` | ament_python | Tutorial package, not part of the system |

## 4. Node graph

```
        BOAT                                   SIMULATION
 fc_bridge    perception              sim_bridge    fake_perception
    │  ▲          │                       │  ▲            │
    ▼  │          ▼   same topics, same messages          ▼
 ┌─────────────────────────────────────────────────────────────┐
 │ /usv/fix  /usv/speed  /usv/heading  /usv/buoys              │
 │                        navigator                            │
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
| `navigator` | `nav_process` | 25 Hz timer: mission state machine, local costmap, A*, Task 2 corridor, LOS/PID and Pure Pursuit, mixer. Publishes `ThrusterPwm` and `NavStatus` |
| `telemetry_bridge` | `telem_process`, `utils/telem.py` | RFD900x JSON protocol unchanged. Turns GCS commands into service calls and echoes `aq` after a successful call |
| `recorder` | CSV + `CostmapRecorder` | Competition deliverables: vehicle CSV (2 Hz, legacy header) and costmap record |
| `sim_bridge` | new | Stands in for `fc_bridge`: commands to Gazebo, FC-side sensors back, with the real hardware's rates and errors (section 10) |
| `fake_perception` | new | Stands in for `perception`: buoys from ground truth within camera FOV and range, ZED heading and attitude, all with ZED-like errors |

`navigator` is deliberately one node. Its internals are split into classes (section 8);
splitting it into several nodes would add message latency inside a 25 Hz control loop for
no benefit, and becomes trivial later once the classes exist.

## 5. Topics

All topics use the default reliable QoS (depth 10) unless marked *latched*
(transient local, depth 1). Camera images are not published.

| Topic | Type | Publisher | Main subscribers |
|---|---|---|---|
| `/usv/fix` | `sensor_msgs/NavSatFix` | `fc_bridge` / `sim_bridge` | navigator, perception, telemetry_bridge, recorder |
| `/usv/speed` | `std_msgs/Float32` (m/s, ground speed) | `fc_bridge` / `sim_bridge` | navigator, telemetry_bridge, recorder |
| `/usv/heading/fc` | `std_msgs/Float32` (deg) | `fc_bridge` / `sim_bridge` | heading_source |
| `/usv/heading/zed` | `std_msgs/Float32` (deg) | `perception` / `fake_perception` | heading_source |
| `/usv/heading` | `std_msgs/Float32` (deg, 0-360, clockwise from north, declination applied) | `heading_source` | navigator, perception, telemetry_bridge, recorder |
| `/usv/attitude` | `geometry_msgs/Vector3Stamped` (x = roll deg, y = pitch deg) | `perception` / `fake_perception` | recorder |
| `/usv/relay_state` | `std_msgs/Int8` (-1 unknown, 0 off, 1 on) | `fc_bridge` / `sim_bridge` | telemetry_bridge |
| `/usv/buoys` | `usv_interfaces/BuoyArray` | `perception` / `fake_perception` | navigator, recorder |
| `/usv/perception/fps` | `std_msgs/Float32` | `perception` | telemetry_bridge |
| `/usv/thrusters/pwm` | `usv_interfaces/ThrusterPwm` | `navigator` | fc_bridge / sim_bridge, telemetry_bridge, perception (HUD) |
| `/usv/nav_status` | `usv_interfaces/NavStatus` (10 Hz) | `navigator` | telemetry_bridge, recorder, perception (HUD) |
| `/usv/mission/waypoints` | `usv_interfaces/WaypointArray` *latched* | `navigator` | telemetry_bridge |
| `/usv/plan` | `nav_msgs/Path` | `navigator` | RViz, recorder |

## 6. Services

One service per GCS command. The GCS protocol itself does not change.

| Service | Type | Server | GCS command |
|---|---|---|---|
| `/usv/set_waypoint` | `usv_interfaces/SetWaypoint` | navigator | `set_gps` |
| `/usv/set_task` | `usv_interfaces/SetTask` | navigator | `set_task` |
| `/usv/set_manual` | `std_srvs/SetBool` | navigator | `set_manual` |
| `/usv/set_target_color` | `usv_interfaces/SetTargetColor` | navigator | `set_target_color` |
| `/usv/set_relay` | `std_srvs/SetBool` | fc_bridge | `set_relay` |
| `/usv/emergency_stop` | `std_srvs/Trigger` | fc_bridge and navigator | `emergency_stop` |

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
| Leaving emergency | Operator turns the relay on (`set_relay`), then re-selects the task (`set_task`). Relay on alone never makes the boat move |
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
- **`fake_perception`** stands in for `perception` (ZED 2i): `/usv/buoys`, `/usv/heading/zed`,
  `/usv/attitude`, `/usv/perception/fps`. The real `heading_source` then makes
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
| Full run for replay and analysis | `ros2 bag record` |

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

Phase 5 progress (2026-10-31): the navigator brain is ported to `usv_core` (`nav_config`, `costmap`, `guidance`, `mission`, `task3`, `commands`, `navigator`, `safety`, `relay`) and matches the legacy loop cycle for cycle on every golden scenario (`test/golden/`). Deliberate changes: the emergency stop latches (neutral from the same cycle until an explicit reset, then MANUAL) instead of ending the program; relay retries move to `fc_bridge`. Next: 5.5 ROS `navigator` node, 5.6 full Golcuk run in Gazebo.

Remaining phases:

| Phase | Work | Done when |
|---|---|---|
| 5 | `navigator` (mixer, controller, mission) | Tasks 1, 2, 3 completed in simulation |
| 6 | Boat image (arm64), `fc_bridge`, `perception`, `heading_source`, `telemetry_bridge`, `recorder` | Sensors flowing on land and at the pier; GCS connects unchanged |
| 7 | Sea trials | ROS 2 version completes the same course as the legacy system |
| 8 | RoboBoat adaptation | Defined from the RoboBoat rules |
