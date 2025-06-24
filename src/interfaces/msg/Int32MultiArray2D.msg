# A 2D array of int32 values with a header.
# Equivalent to an int32 data[rows][cols]

std_msgs/Header header

uint32 rows
uint32 cols
int32[] data # flattened