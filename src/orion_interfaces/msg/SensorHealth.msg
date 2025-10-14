# ORION Sensor Health Status
# Monitoring and diagnostics for individual sensors

std_msgs/Header header

# Sensor identification
string sensor_name
string sensor_type  # "imu", "lidar", "camera", "gnss", etc.

# Health status
uint8 HEALTH_OK = 0
uint8 HEALTH_WARN = 1
uint8 HEALTH_ERROR = 2
uint8 HEALTH_STALE = 3
uint8 HEALTH_DISCONNECTED = 4
uint8 health_status

# Diagnostics
float32 update_rate_hz          # Current update rate
float32 expected_rate_hz        # Expected update rate
float32 data_quality            # Quality metric (0.0-1.0)
uint32 dropped_messages         # Messages dropped/lost
builtin_interfaces/Time last_update_time

# Error/warning messages
string status_message
string[] diagnostic_messages
