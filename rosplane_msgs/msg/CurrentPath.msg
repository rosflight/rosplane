# Current path output from the path manager, input to path follower

# header
std_msgs/Header header

# @warning va_d must always be valid,
# r and q need to be valid if path_type == LINE_PATH
# c, rho, and, lambda need to be valid if path_type == ORBIT_PATH
uint8 path_type		# Indicates strait line or orbital path
float32 va_d		# Desired airspeed (m/s)
float32[3] r		# Vector to origin of straight line path (m)
float32[3] q		# Unit vector, desired direction of travel for line path
float32[3] c		# Center of orbital path (m)
float32 rho		# Radius of orbital path (m)
int8 lamda		# Direction of orbital path (clockwise is 1, counterclockwise is -1)

uint8 ORBIT_PATH = 0
uint8 LINE_PATH = 1

int8 CLOCKWISE = 1
int8 COUNT_CLOCKWISE = -1
