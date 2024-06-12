std_msgs/Header header

int16 id

# The detected point position and orientation
geometry_msgs/Pose pose

# The distance from the sensor origin to the detected point
float32 distance_to_collision

# Corner coordinates of the bounding box
geometry_msgs/Point rear_bottom_right
geometry_msgs/Point rear_bottom_left
geometry_msgs/Point front_bottom_right
geometry_msgs/Point front_bottom_left

# The detected point absolute speed vector
geometry_msgs/Vector3 absolute_speed

# The detected point absolute angular speed vector (RPY)
geometry_msgs/Vector3 absolute_angular_speed

# The detected point relative speed vector
geometry_msgs/Vector3 relative_speed

# The detected point relative angular speed vector (RPY)
geometry_msgs/Vector3 relative_angular_speed

# The detected point absolute acceleration
geometry_msgs/Vector3 absolute_accel

# The detected point relative acceleration
geometry_msgs/Vector3 relative_accel

# Target length
float32 length

# Target width
float32 width

# Target height
float32 height
