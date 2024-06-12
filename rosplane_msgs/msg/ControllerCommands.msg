# Controller commands output from the path follower, input to autopilot controller

# header
std_msgs/Header header

# @warning Va_c, h_c and chi_c have always to be valid, the aux array is optional
float32 va_c		# Commanded airspeed (m/s)
float32 h_c		    # Commanded altitude (m)
float32 chi_c		# Commanded course (rad)
float32 phi_ff		# feed forward command for orbits (rad)
float32[4] aux		# Optional auxiliary commands
bool aux_valid		# Auxiliary commands valid

# @warning The following commands by default are set by the controller itself. Set the override
# parameter for the controller to use these commands.
float32 theta_c		# Commanded pitch (rad)
float32 phi_c		# Commanded roll (rad)
uint8 alt_zone		# Zone in the altitude state machine

uint8 ZONE_TAKE_OFF = 0
uint8 ZONE_CLIMB = 1
uint8 ZONE_DESEND = 2
uint8 ZONE_ALTITUDE_HOLD = 3
