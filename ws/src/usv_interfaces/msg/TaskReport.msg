# Something a task has to report (RoboBoat 2027 step 6d): a gate passed, an object found,
# a point reached. Goes boat -> GCS (GCS link, acked) -> RoboCommand, where the GCS's codec
# turns it into that year's message; kinds it has no message for are only logged.
std_msgs/Header header
uint32 report_id                    # counts up per boat run (the GCS acks it, drops copies)
string task                         # the task it came from (e.g. T1_ENTRY, rally)
string type                         # gate, object, delivery, docking, arrived, ...
float64 latitude                    # where (the boat, or the object); NaN if no place
float64 longitude
string[] keys                       # the other fields, as text: keys[i] = values[i]
string[] values                     # (e.g. gate = entry, color = green)
