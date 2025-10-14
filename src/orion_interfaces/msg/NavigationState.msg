# ORION Navigation State Message
# Complete state estimate with uncertainty and metadata

std_msgs/Header header

# Pose estimate (position + orientation)
geometry_msgs/PoseWithCovariance pose

# Velocity estimate (linear + angular)
geometry_msgs/TwistWithCovariance twist

# Navigation status
uint8 NAV_STATUS_INITIALIZING = 0
uint8 NAV_STATUS_CONVERGED = 1
uint8 NAV_STATUS_DEGRADED = 2
uint8 NAV_STATUS_LOST = 3
uint8 status

# Confidence metrics (0.0 - 1.0)
float32 overall_confidence
float32 position_confidence
float32 velocity_confidence
float32 attitude_confidence

# Active sensor modalities (bitmask)
uint8 SENSOR_IMU = 1
uint8 SENSOR_LIDAR = 2
uint8 SENSOR_CAMERA = 4
uint8 SENSOR_GNSS = 8
uint8 active_sensors

# Estimated position drift since last global fix (meters)
float32 estimated_drift

# Time since last global relocalization (seconds)
float32 time_since_global_fix

# Number of features tracked (for visual/LiDAR odometry)
uint32 tracked_features
