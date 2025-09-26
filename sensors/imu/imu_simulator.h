#pragma once

#include "sensors/imu/imu_driver.h"

#include <random>
#include <thread>

namespace aion {

/**
 * IMU simulator for testing and development
 * Generates realistic IMU data with configurable noise and trajectories
 */
class IMUSimulator : public IMUDriver {
 public:
  enum class Trajectory {
    STATIC,             // No motion (for bias calibration)
    CONSTANT_VELOCITY,  // Straight line motion
    CIRCULAR,           // Circular motion in horizontal plane
    FIGURE8,            // Figure-8 pattern
    RANDOM_WALK         // Random accelerations
  };

  struct Config {
    Trajectory trajectory;
    double rate_hz;

    // Motion parameters
    Eigen::Vector3d initial_position;
    Eigen::Vector3d initial_velocity;  // m/s
    double circular_radius;   // m (for circular/figure8)
    double circular_period;   // seconds

    // Noise parameters (override calibration if set)
    double gyro_noise_density;    // rad/s/sqrt(Hz), 0 = use calib
    double accel_noise_density;   // m/s^2/sqrt(Hz), 0 = use calib
    double gyro_bias_stability;   // rad/s, 0 = use calib
    double accel_bias_stability;  // m/s^2, 0 = use calib

    // Temperature simulation
    double temp_variation;        // Celsius peak-to-peak
    double temp_period;         // seconds
    double base_temp;            // Celsius

    // Random seed (0 = use time)
    unsigned int seed;

    Config()
        : trajectory(Trajectory::STATIC),
          rate_hz(200.0),
          initial_position(0.0, 0.0, 0.0),
          initial_velocity(0.0, 0.0, 0.0),
          circular_radius(10.0),
          circular_period(20.0),
          gyro_noise_density(0.0),
          accel_noise_density(0.0),
          gyro_bias_stability(0.0),
          accel_bias_stability(0.0),
          temp_variation(2.0),
          temp_period(300.0),
          base_temp(25.0),
          seed(0) {}
  };

  explicit IMUSimulator(const Config& config = Config());
  ~IMUSimulator() override;

  // IMUDriver interface
  bool start() override;
  void stop() override;
  std::string getName() const override { return "IMUSimulator"; }
  double getRate() const override { return config_.rate_hz; }

  // Get/set configuration
  const Config& getConfig() const { return config_; }
  void setConfig(const Config& cfg) { config_ = cfg; }

  // Get current true state (for testing)
  struct TrueState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d acceleration;
    double time;
  };
  TrueState getTrueState() const;

 private:
  void simulationThread();
  void generateMotion(double t, TrueState& state);
  void addNoise(IMUData& imu);

  // Generate trajectories
  void staticTrajectory(double t, TrueState& state);
  void constantVelocityTrajectory(double t, TrueState& state);
  void circularTrajectory(double t, TrueState& state);
  void figure8Trajectory(double t, TrueState& state);
  void randomWalkTrajectory(double t, TrueState& state);

  Config config_;
  std::thread sim_thread_;

  // True state tracking
  mutable std::mutex state_mutex_;
  TrueState true_state_;

  // Random number generation
  std::mt19937 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};

  // Bias random walk state
  Eigen::Vector3d gyro_bias_walk_{0.0, 0.0, 0.0};
  Eigen::Vector3d accel_bias_walk_{0.0, 0.0, 0.0};

  // Random walk trajectory state
  Eigen::Vector3d random_angular_vel_{0.0, 0.0, 0.0};
  Eigen::Vector3d random_accel_{0.0, 0.0, 0.0};

  // Timing
  double start_time_{0.0};
  double last_time_{0.0};
};

}  // namespace aion