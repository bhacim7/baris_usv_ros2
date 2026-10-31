# One object of the world map (mapper): every sighting of it fused into one position
uint32 id
string kind                         # class name; the two indicator classes are one kind,
                                    # 'indicator', whose colour can change between runs
string color                        # indicators: 'red' or 'green' (recent sightings); else ''
float64 latitude
float64 longitude
float32 east_m                      # metres from the map origin (LandmarkArray)
float32 north_m
float32 sigma_m                     # 1-sigma position uncertainty
uint32 sightings
bool confirmed                      # seen often enough to be trusted
float32 closest_m                   # nearest range it was seen from
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
