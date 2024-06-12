std_msgs/Header header

# Position, Orientation, Velocities and Accelerations
geometry_msgs/Pose world_position # Position and orientation

float64 yaw 

float64 speed_kmh
geometry_msgs/Twist local_velocity # Linear and angular velocities using the car as origin
geometry_msgs/Twist global_velocity # Velocity vectors oriented in relation to the map

float64 accel_x
float64 accel_y
