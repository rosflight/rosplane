# Internal controller commands output from the outer loops of the autopilot

# header
std_msgs/Header header
float32 theta_c		# Commanded pitch (rad)
float32 phi_c		# Commanded roll (rad)
uint8 alt_zone		# Zone in the altitude state machine

uint8 ZONE_TAKE_OFF = 0
uint8 ZONE_CLIMB = 1
uint8 ZONE_DESEND = 2
uint8 ZONE_ALTITUDE_HOLD = 3
