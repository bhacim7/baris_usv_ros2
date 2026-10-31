# One object found in one camera frame (perception / fake_perception), already measured:
# range from the ZED depth, compass bearing from the heading + the pixel column, position
# from the GPS fix along that bearing.
string class_name                   # usv_core.classes, e.g. buoy_red, indicator_green
uint8 class_id                      # position of class_name in the model's class list
float32 confidence                  # YOLO score; 1.0 in simulation
int32 x1                            # box in pixels
int32 y1
int32 x2
int32 y2
float32 distance_m                  # NaN = no valid depth
float32 bearing_deg                 # compass, 0-360; NaN = no heading
bool position_ok                    # distance valid (0.5-15 m), heading and fix present
float64 latitude                    # NaN unless position_ok
float64 longitude
