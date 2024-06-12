# TODO add more fields as needed

std_msgs/Header header

# slip angles for each tire
float64[] tire_slip_angles
float64[] slip_angle

# tire temperature
float64[] thermal_state

# aero
float64 drag_coefficient
float64 lift_coefficient_front
float64 lift_coefficient_rear

float64 cg_height
