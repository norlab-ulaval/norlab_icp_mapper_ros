# Source enum
uint8 SOURCE_GPS = 0 # Message contains position information only
uint8 SOURCE_IMU = 1 # Message contains orientation, linear acceleration and angular velocity information
uint8 SOURCE_IMU_GPS = 2 # Message contains orientation, linear acceleration and angular velocity information

std_msgs/Header header
uint8 source

# GPS data
geometry_msgs/Point position
float64[9] position_covariance # Row-major x, y, z

# IMU data
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes
