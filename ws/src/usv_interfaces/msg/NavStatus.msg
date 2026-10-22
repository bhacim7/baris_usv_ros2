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