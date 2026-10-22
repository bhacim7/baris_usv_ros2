# PWM in microseconds. Flight-controller channels are fc_bridge parameters.
std_msgs/Header header
uint16 rear_left
uint16 rear_right
uint16 front_left
uint16 front_right
uint16 steer
bool force                          # emergency / neutralise: bypass slew limit and write thinning