# This represents a 2-D grid map, in which each cell represents the probability of
# occupancy.

Header header 

#MetaData for the map
nav_msgs/MapMetaData info

# The map data, in row-major order, starting with (0,0).
float64[] x
float64[] y
float64[] theta
float64[] distance
string[] name
string[] uuid
