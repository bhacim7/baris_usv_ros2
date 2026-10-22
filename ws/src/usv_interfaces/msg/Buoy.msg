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