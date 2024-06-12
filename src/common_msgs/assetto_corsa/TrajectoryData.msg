std_msgs/Header header

geometry_msgs/PoseArray left_lane		# Left boundary line
geometry_msgs/PoseArray right_lane		# Right boundary line
geometry_msgs/PoseArray fast_lane		# AC trajectory
float64[] brake_arr                     # Brake level in AC trajectory
float64[] throttle_arr                  # Throttle level in AC trajectory
float64[] speed_arr                     # Speed in AC trajectory
