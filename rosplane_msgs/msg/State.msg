# State message for ROSplane.

std_msgs/Header header

float32 p_n             # north (m)
float32 p_e             # east (m)
float32 p_d             # down (m)
float32 v_x             # Inertial north velocity - u (expressed in body frame) (m/s)
float32 v_y             # Inertial east velocity - v (expressed in body frame) (m/s)
float32 v_z             # Inertial down velocity - w (expressed in body frame) (m/s)

float32 p               # Body frame rollrate (rad/s)
float32 q               # Body frame pitchrate (rad/s)
float32 r               # Body frame yawrate (rad/s)
float32 phi		          # Roll angle (rad)
float32 theta           # Pitch angle (rad)
float32 psi		          # Yaw angle (rad)

float32 b_x             # Gyro bias x (rad/s)
float32 b_y             # Gyro bias y (rad/s)
float32 b_z             # Gyro bias z (rad/s)

geometry_msgs/Quaternion quat		# Quaternion (wxyz, NED) body to inertial

float64 initial_lat 	  # Initial/origin latitude (lat. deg)
float64 initial_lon 	  # Initial/origin longitude (lon. deg) 
float64 initial_alt 	  # Initial/origin altitude (m) 

float32 wn		# Wind north component (m/s)
float32 we		# Wind east component (m/s)

float32 va		# Airspeed (m/s)
float32 alpha	# Angle of attack (rad)
float32 beta	# Slide slip angle (rad)
float32 chi		# Course angle (rad)
