# Everything one camera frame found, with what it was measured from
std_msgs/Header header              # stamp = frame time
uint32 image_width
float64 boat_latitude               # the GPS fix used (0 = none yet)
float64 boat_longitude
float32 heading_deg                 # the heading used (NaN = none yet)
float32 origin_forward_m            # where positions were placed from, boat frame, from the
float32 origin_left_m               # GPS antenna: 0, 0 = the antenna (legacy), else the camera
Detection[] detections
