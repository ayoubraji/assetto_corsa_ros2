std_msgs/Header header

# General data
uint8 car_id
uint8 leaderboard_position
string driver_name
int8 lap_count
int32 best_lap # Best Lap in milliseconds

# Position, Orientation, Velocities and Accelerations
float64 normalized_spline_position
geometry_msgs/Pose world_position # Position and orientation

float64 speed_kmh
geometry_msgs/Twist local_velocity # Linear and angular velocities using the car as origin
geometry_msgs/Twist global_velocity # Velocity vectors oriented in relation to the map

float64 accel_x
float64 accel_y

# Actuation data
float64 steer_angle
float64 gas_pedal_sts
float64 brake_pedal_sts
uint8 actual_gear
uint16 rpm
float64 cg_height

# Tyres data
float64 slipangle_fl
float64 slipangle_fr
float64 slipangle_rl
float64 slipangle_rr

float64 slipratio_fl
float64 slipratio_fr
float64 slipratio_rl
float64 slipratio_rr

float64 dy_fl
float64 dy_fr
float64 dy_rl
float64 dy_rr

float64 mz_fl
float64 mz_fr
float64 mz_rl
float64 mz_rr

float64 load_fl
float64 load_fr
float64 load_rl
float64 load_rr

float64 nd_slip_fl
float64 nd_slip_fr
float64 nd_slip_rl
float64 nd_slip_rr

float64 pressure_fl
float64 pressure_fr
float64 pressure_rl
float64 pressure_rr

float64 suspensiontravel_fl
float64 suspensiontravel_fr
float64 suspensiontravel_rl
float64 suspensiontravel_rr

float64 camber_fl
float64 camber_fr
float64 camber_rl
float64 camber_rr

float64 loadedradius_fl
float64 loadedradius_fr
float64 loadedradius_rl
float64 loadedradius_rr

# Other stuff
float64 dist_l		# Distance between front-left wheel and left boundary line used by the AI
float64 dist_r		# Distance between front-right wheel and right boundary line used by the AI
