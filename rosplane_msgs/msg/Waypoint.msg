# New waypoint, input to path manager

# header
std_msgs/Header header

# @warning w and Va_d always have to be valid; the chi_d is optional.
float32[3] w		# Waypoint in local NED (m) or LLA
bool lla            # Set this flag true if waypoint is LLA and not local NED
float32 chi_d		# Desired course at this waypoint (rad)
bool use_chi		# Use chi_d as specified, using a Dubin's path. Otherwise, use fillet
float32 va_d		# Desired airspeed (m/s)
bool set_current	# Sets a waypoint at the vehicle's current position
bool clear_wp_list  # Removes all waypoints and returns to origin.  The rest of
                    # this message will be ignored
