# A 2D array of float32 values with a header.
# Equivalent to a float32 data[rows][cols]

std_msgs/Header header

uint32 rows
uint32 cols
float32[] data  # flattened data