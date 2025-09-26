#include "sensors/imu/imu_simulator.h"
#include "core/utils/logging.h"
#include "core/utils/time_utils.h"

#include <chrono>

namespace aion {

IMUSimulator::IMUSimulator(const Config& config) : config_(config) {
  // Initialize RNG
  unsigned int seed = config_.seed;
  if (seed == 0) {
    auto ticks = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    seed = static_cast<unsigned int>(ticks & 0xFFFFFFFF);
  }
  rng_.seed(seed);

  // Initialize true state
  true_state_.position = config_.initial_position;
  true_state_.velocity = config_.initial_velocity;
  true_state_.orientation = Eigen::Quaterniond::Identity();
  true_state_.angular_velocity = Eigen::Vector3d::Zero();
  true_state_.acceleration = Eigen::Vector3d::Zero();
  true_state_.time = 0.0;

  // Use calibration noise params if not overridden
  if (config_.gyro_noise_density <= 0.0) {
    config_.gyro_noise_density = calibration_.gyro_noise_density;
  }
  if (config_.accel_noise_density <= 0.0) {
    config_.accel_noise_density = calibration_.accel_noise_density;
  }
  if (config_.gyro_bias_stability <= 0.0) {
    config_.gyro_bias_stability = calibration_.gyro_bias_stability;
  }
  if (config_.accel_bias_stability <= 0.0) {
    config_.accel_bias_stability = calibration_.accel_bias_stability;
  }
}

IMUSimulator::~IMUSimulator() {
  stop();
}

bool IMUSimulator::start() {
  if (is_running_) {
    LOG_WARN("IMU simulator already running");
    return false;
  }

  is_running_ = true;
  start_time_ = TimeUtils::now();
  last_time_ = start_time_;

  sim_thread_ = std::thread(&IMUSimulator::simulationThread, this);

  LOG_INFO("IMU simulator started: trajectory={}, rate={:.0f}Hz",
           static_cast<int>(config_.trajectory), config_.rate_hz);
  return true;
}

void IMUSimulator::stop() {
  if (!is_running_) return;

  is_running_ = false;
  if (sim_thread_.joinable()) {
    sim_thread_.join();
  }

  LOG_INFO("IMU simulator stopped");
}

IMUSimulator::TrueState IMUSimulator::getTrueState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return true_state_;
}

void IMUSimulator::simulationThread() {
  aion::RateController rate_controller(config_.rate_hz);

  while (is_running_) {
    double current_time = TimeUtils::now();
    double t = current_time - start_time_;  // Time since start
    double dt = current_time - last_time_;

    // Generate true motion
    TrueState state;
    generateMotion(t, state);
    state.time = current_time;

    // Store true state
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      true_state_ = state;
    }

    // Create IMU measurement in body frame
    IMUData imu;
    imu.timestamp = current_time;

    // Transform to body frame (assuming NED to body)
    Eigen::Matrix3d R_bn = state.orientation.toRotationMatrix().transpose();
    imu.angular_velocity = state.angular_velocity;  // Already in body frame

    // Specific force = accel - gravity (in body frame)
    Eigen::Vector3d gravity_ned(0, 0, 9.80665);
    Eigen::Vector3d specific_force_ned = state.acceleration - gravity_ned;
    imu.acceleration = R_bn * specific_force_ned;

    // Temperature simulation
    double temp_phase = 2.0 * M_PI * t / config_.temp_period;
    imu.temperature = config_.base_temp +
                     0.5 * config_.temp_variation * std::sin(temp_phase);

    // Add realistic noise
    addNoise(imu);

    // Update bias random walk
    if (dt > 0 && dt < 1.0) {
      double sqrt_dt = std::sqrt(dt);
      gyro_bias_walk_ += Eigen::Vector3d(
          normal_(rng_) * config_.gyro_bias_stability * sqrt_dt,
          normal_(rng_) * config_.gyro_bias_stability * sqrt_dt,
          normal_(rng_) * config_.gyro_bias_stability * sqrt_dt);

      accel_bias_walk_ += Eigen::Vector3d(
          normal_(rng_) * config_.accel_bias_stability * sqrt_dt,
          normal_(rng_) * config_.accel_bias_stability * sqrt_dt,
          normal_(rng_) * config_.accel_bias_stability * sqrt_dt);
    }

    // Add bias
    imu.angular_velocity += gyro_bias_walk_;
    imu.acceleration += accel_bias_walk_;

    // Publish
    publishIMU(imu);

    last_time_ = current_time;
    rate_controller.sleep();
  }
}

void IMUSimulator::generateMotion(double t, TrueState& state) {
  switch (config_.trajectory) {
    case Trajectory::STATIC:
      staticTrajectory(t, state);
      break;
    case Trajectory::CONSTANT_VELOCITY:
      constantVelocityTrajectory(t, state);
      break;
    case Trajectory::CIRCULAR:
      circularTrajectory(t, state);
      break;
    case Trajectory::FIGURE8:
      figure8Trajectory(t, state);
      break;
    case Trajectory::RANDOM_WALK:
      randomWalkTrajectory(t, state);
      break;
  }
}

void IMUSimulator::staticTrajectory(double /*t*/, TrueState& state) {
  state.position = config_.initial_position;
  state.velocity = Eigen::Vector3d::Zero();
  state.orientation = Eigen::Quaterniond::Identity();
  state.angular_velocity = Eigen::Vector3d::Zero();
  state.acceleration = Eigen::Vector3d::Zero();
}

void IMUSimulator::constantVelocityTrajectory(double t, TrueState& state) {
  state.position = config_.initial_position + config_.initial_velocity * t;
  state.velocity = config_.initial_velocity;
  state.orientation = Eigen::Quaterniond::Identity();
  state.angular_velocity = Eigen::Vector3d::Zero();
  state.acceleration = Eigen::Vector3d::Zero();
}

void IMUSimulator::circularTrajectory(double t, TrueState& state) {
  double omega = 2.0 * M_PI / config_.circular_period;
  double theta = omega * t;

  // Position in circle
  state.position = config_.initial_position + Eigen::Vector3d(
      config_.circular_radius * std::cos(theta),
      config_.circular_radius * std::sin(theta),
      0.0);

  // Velocity tangent to circle
  state.velocity = Eigen::Vector3d(
      -config_.circular_radius * omega * std::sin(theta),
      config_.circular_radius * omega * std::cos(theta),
      0.0);

  // Centripetal acceleration
  state.acceleration = Eigen::Vector3d(
      -config_.circular_radius * omega * omega * std::cos(theta),
      -config_.circular_radius * omega * omega * std::sin(theta),
      0.0);

  // Heading aligned with velocity
  double heading = std::atan2(state.velocity.y(), state.velocity.x());
  state.orientation = Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ());

  // Angular velocity (yaw rate)
  state.angular_velocity = Eigen::Vector3d(0, 0, omega);
}

void IMUSimulator::figure8Trajectory(double t, TrueState& state) {
  double omega = 2.0 * M_PI / config_.circular_period;
  double theta = omega * t;

  // Lemniscate (figure-8) parametric equations
  double scale = config_.circular_radius;
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);
  double denom = 1.0 + sin_theta * sin_theta;

  // Position
  state.position = config_.initial_position + Eigen::Vector3d(
      scale * cos_theta / denom,
      scale * sin_theta * cos_theta / denom,
      0.0);

  // Velocity (derivative of position)
  double denom2 = denom * denom;
  state.velocity = scale * omega * Eigen::Vector3d(
      -(sin_theta * denom + 2.0 * cos_theta * cos_theta * sin_theta) / denom2,
      (cos_theta * cos_theta * denom - sin_theta * sin_theta * cos_theta * 2.0) / denom2,
      0.0);

  // Acceleration (derivative of velocity)
  // Simplified - using numerical approximation
  double dt = 0.001;
  double theta_next = omega * (t + dt);
  double cos_next = std::cos(theta_next);
  double sin_next = std::sin(theta_next);
  double denom_next = 1.0 + sin_next * sin_next;

  Eigen::Vector3d velocity_next = scale * omega * Eigen::Vector3d(
      -(sin_next * denom_next + 2.0 * cos_next * cos_next * sin_next) / (denom_next * denom_next),
      (cos_next * cos_next * denom_next - sin_next * sin_next * cos_next * 2.0) / (denom_next * denom_next),
      0.0);

  state.acceleration = (velocity_next - state.velocity) / dt;

  // Heading aligned with velocity
  if (state.velocity.head<2>().norm() > 0.01) {
    double heading = std::atan2(state.velocity.y(), state.velocity.x());
    state.orientation = Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ());

    // Angular velocity (yaw rate)
    double heading_rate = (state.velocity.x() * state.acceleration.y() -
                          state.velocity.y() * state.acceleration.x()) /
                         state.velocity.head<2>().squaredNorm();
    state.angular_velocity = Eigen::Vector3d(0, 0, heading_rate);
  } else {
    state.orientation = Eigen::Quaterniond::Identity();
    state.angular_velocity = Eigen::Vector3d::Zero();
  }
}

void IMUSimulator::randomWalkTrajectory(double t, TrueState& state) {
  double dt = 1.0 / config_.rate_hz;

  // Random walk on angular velocity and acceleration
  if (t - last_time_ > 0 && t - last_time_ < 1.0) {
    double walk_scale = 0.1;  // rad/s^2 and m/s^3
    random_angular_vel_ += walk_scale * dt * Eigen::Vector3d(
        normal_(rng_), normal_(rng_), normal_(rng_));
    random_accel_ += walk_scale * dt * Eigen::Vector3d(
        normal_(rng_), normal_(rng_), normal_(rng_));

    // Limit magnitudes
    double max_omega = 1.0;  // rad/s
    double max_accel = 2.0;  // m/s^2
    if (random_angular_vel_.norm() > max_omega) {
      random_angular_vel_ = random_angular_vel_.normalized() * max_omega;
    }
    if (random_accel_.norm() > max_accel) {
      random_accel_ = random_accel_.normalized() * max_accel;
    }
  }

  // Integrate motion
  state.angular_velocity = random_angular_vel_;
  state.acceleration = random_accel_;

  // Update orientation
  if (state.angular_velocity.norm() > 0) {
    double angle = state.angular_velocity.norm() * dt;
    Eigen::AngleAxisd rot(angle, state.angular_velocity.normalized());
    state.orientation = rot * true_state_.orientation;
  } else {
    state.orientation = true_state_.orientation;
  }

  // Update velocity and position
  state.velocity = true_state_.velocity + state.acceleration * dt;
  state.position = true_state_.position + state.velocity * dt +
                  0.5 * state.acceleration * dt * dt;
}

void IMUSimulator::addNoise(IMUData& imu) {
  // Add white noise
  double gyro_sigma = config_.gyro_noise_density * std::sqrt(config_.rate_hz);
  double accel_sigma = config_.accel_noise_density * std::sqrt(config_.rate_hz);

  imu.angular_velocity += Eigen::Vector3d(
      normal_(rng_) * gyro_sigma,
      normal_(rng_) * gyro_sigma,
      normal_(rng_) * gyro_sigma);

  imu.acceleration += Eigen::Vector3d(
      normal_(rng_) * accel_sigma,
      normal_(rng_) * accel_sigma,
      normal_(rng_) * accel_sigma);
}

}  // namespace aion