# New waypoint, input to path manager

# header
std_msgs/Header header

# @warning w and Va_d always have to be valid; the chi_d is optional.
float32[3] w		# Waypoint in local NED (m)
float32 chi_d		# Desired course at this waypoint (rad)
bool chi_valid		# Desired course valid (dubin or fillet paths)
float32 va_d		# Desired airspeed (m/s)
bool set_current	# Sets this waypoint to be executed now! Starts a new list
bool clear_wp_list  # Removes all waypoints and returns to origin.  The rest of
                    # this message will be ignored
