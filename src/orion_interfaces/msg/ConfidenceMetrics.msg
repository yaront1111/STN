# ORION Confidence Estimation Metrics
# Detailed breakdown of navigation confidence

std_msgs/Header header

# Overall confidence (0.0 - 1.0)
float32 overall_confidence

# Per-component confidence
float32 position_xy_confidence
float32 position_z_confidence
float32 velocity_confidence
float32 attitude_confidence

# Sensor-specific contributions
float32 imu_contribution
float32 lidar_contribution
float32 camera_contribution
float32 gnss_contribution

# Covariance-based metrics
float32 position_uncertainty_m     # 1-sigma uncertainty in meters
float32 velocity_uncertainty_ms    # 1-sigma uncertainty in m/s
float32 attitude_uncertainty_deg   # 1-sigma uncertainty in degrees

# Environment assessment
float32 feature_richness           # Environment feature quality (0.0-1.0)
float32 geometric_diversity        # LiDAR geometry quality (0.0-1.0)
float32 lighting_quality           # Visual conditions (0.0-1.0)

# Temporal consistency
float32 innovation_consistency     # Measurement innovation check (0.0-1.0)
float32 temporal_smoothness        # State estimate smoothness (0.0-1.0)

# Failure prediction
float32 failure_probability        # Estimated probability of tracking loss
uint32 time_to_failure_estimate_s  # Estimated seconds until failure (0 = unknown)
