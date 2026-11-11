"""
RoboCommand: the judges' system, reached from the GCS only (RoboBoat 2027, step 5d).

The boats never talk to RoboCommand: their data comes to the GCS over the GCS link
(Ubiquiti + RFD), and the GCS passes it on over the Ethernet cable in the operations tent;
tasks from the judges come back the same way (as a GCS link `task` command, inside ROS a
usv_interfaces/TaskRequest). Three layers, so a new year's protocol changes one of them:

  transport  how bytes travel: tcp.py (RoboBoat 2026: TCP, $R + length + payload + !!);
             MQTT (RobotX 2026, two-way) to be added when the 2027 handbook says so
  codec      what the bytes mean: the official protobuf messages (RoboNation's .proto,
             generated code in robocommand/msgs)
  bridge     what we send and when: bridge.py (heartbeat per boat at 1 Hz, at most 5
             messages a second, sequence numbers), independent of both
"""
