# Livox costom pointcloud format. copied from https://github.com/Livox-SDK/livox_ros_driver2/blob/master/msg/CustomPoint.msg at Jul 3 2025.

uint32 offset_time      # offset time relative to the base time
float32 x               # X axis, unit:m
float32 y               # Y axis, unit:m
float32 z               # Z axis, unit:m
uint8 reflectivity      # reflectivity, 0~255
uint8 tag               # livox tag
uint8 line              # laser number in lidar

