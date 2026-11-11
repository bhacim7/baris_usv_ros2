# A task asked of this boat from outside: by the judges (RoboCommand, relayed by the GCS) or
# by the operator at the GCS. Fixed inside ROS whatever the outside protocol is (RoboBoat
# 2027 step 5c); the mission manager (step 6) decides what to do with it.
std_msgs/Header header
uint32 request_id                   # the sender's number for it (the GCS link's cq)
string source                       # "robocommand" or "gcs"
string task                         # what: e.g. "delivery", "rally", "badge", "abort"
bool urgent                         # true = drop the current task for it, then resume
float64 latitude                    # where, if the task has a place; NaN if not
float64 longitude
string[] keys                       # the task's other parameters, as text: keys[i] = values[i]
string[] values                     # (e.g. shape = triangle, count = 2, badge = logistics)
