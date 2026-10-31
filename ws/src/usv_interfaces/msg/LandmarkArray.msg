# The world map: everything seen since power-on (mapper, /usv/map)
std_msgs/Header header
float64 origin_latitude             # map origin = first GPS fix
float64 origin_longitude
Landmark[] landmarks
