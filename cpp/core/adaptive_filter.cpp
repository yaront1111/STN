#include "adaptive_filter.h"
#include <numeric>
#include <algorithm>

AdaptiveFilter::AdaptiveFilter(const Config& config)
    : config_(config),
      acc_bias_(Eigen::Vector3d::Zero()),
      gyro_bias_(Eigen::Vector3d::Zero()),
      acc_scale_(Eigen::Vector3d::Ones()),
      gyro_scale_(Eigen::Vector3d::Ones()),
      acc_mean_(Eigen::Vector3d::Zero()),
      acc_variance_(Eigen::Vector3d::Ones()),
      gyro_mean_(Eigen::Vector3d::Zero()),
      gyro_variance_(Eigen::Vector3d::Ones()),
      outliers_rejected_(0) {
}

void AdaptiveFilter::updateBiasEstimate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {
    // Add to history
    acc_history_.push_back(acc);
    gyro_history_.push_back(gyro);

    // Maintain window size
    while (acc_history_.size() > config_.window_size) {
        acc_history_.pop_front();
    }
    while (gyro_history_.size() > config_.window_size) {
        gyro_history_.pop_front();
    }

    // Update statistics
    updateStatistics();

    // Outlier rejection
    if (config_.enable_outlier_rejection) {
        if (isOutlier(acc, acc_mean_, acc_variance_)) {
            outliers_rejected_++;
            return;
        }
    }

    // Update bias estimates using exponential moving average
    Eigen::Vector3d acc_error = acc - acc_mean_;
    Eigen::Vector3d gyro_error = gyro - gyro_mean_;

    acc_bias_ = config_.bias_decay * acc_bias_ + config_.learning_rate * acc_error;
    gyro_bias_ = config_.bias_decay * gyro_bias_ + config_.learning_rate * gyro_error;

    // Limit bias magnitudes
    const double max_acc_bias = 0.5;  // m/s^2
    const double max_gyro_bias = 0.01;  // rad/s

    for (int i = 0; i < 3; ++i) {
        acc_bias_(i) = std::clamp(acc_bias_(i), -max_acc_bias, max_acc_bias);
        gyro_bias_(i) = std::clamp(gyro_bias_(i), -max_gyro_bias, max_gyro_bias);
    }
}

Eigen::Vector3d AdaptiveFilter::correctAccelerometer(const Eigen::Vector3d& raw_acc) {
    // Apply bias and scale correction
    Eigen::Vector3d corrected = (raw_acc - acc_bias_).cwiseProduct(acc_scale_);

    // Additional outlier check
    if (config_.enable_outlier_rejection) {
        if (isOutlier(corrected, acc_mean_, acc_variance_)) {
            // Return last known good value
            return acc_mean_;
        }
    }

    return corrected;
}

Eigen::Vector3d AdaptiveFilter::correctGyroscope(const Eigen::Vector3d& raw_gyro) {
    // Apply bias and scale correction
    Eigen::Vector3d corrected = (raw_gyro - gyro_bias_).cwiseProduct(gyro_scale_);

    // Additional outlier check
    if (config_.enable_outlier_rejection) {
        if (isOutlier(corrected, gyro_mean_, gyro_variance_)) {
            // Return last known good value
            return gyro_mean_;
        }
    }

    return corrected;
}

void AdaptiveFilter::updateScaleFactors(const State& state) {
    // Adapt scale factors based on motion state
    double vel_norm = state.v_ECEF.norm();

    if (vel_norm < 1.0) {
        // Nearly stationary - increase accelerometer trust
        acc_scale_ = Eigen::Vector3d(1.02, 1.02, 1.02);
        gyro_scale_ = Eigen::Vector3d(0.98, 0.98, 0.98);
    } else if (vel_norm > 50.0) {
        // High speed - increase gyro trust
        acc_scale_ = Eigen::Vector3d(0.98, 0.98, 0.98);
        gyro_scale_ = Eigen::Vector3d(1.02, 1.02, 1.02);
    } else {
        // Normal operation
        acc_scale_ = Eigen::Vector3d::Ones();
        gyro_scale_ = Eigen::Vector3d::Ones();
    }
}

void AdaptiveFilter::updateStatistics() {
    if (acc_history_.empty()) return;

    // Calculate mean
    acc_mean_ = Eigen::Vector3d::Zero();
    for (const auto& acc : acc_history_) {
        acc_mean_ += acc;
    }
    acc_mean_ /= acc_history_.size();

    // Calculate variance
    acc_variance_ = Eigen::Vector3d::Zero();
    for (const auto& acc : acc_history_) {
        Eigen::Vector3d diff = acc - acc_mean_;
        acc_variance_ += diff.cwiseProduct(diff);
    }
    acc_variance_ /= acc_history_.size();

    // Same for gyro
    if (!gyro_history_.empty()) {
        gyro_mean_ = Eigen::Vector3d::Zero();
        for (const auto& gyro : gyro_history_) {
            gyro_mean_ += gyro;
        }
        gyro_mean_ /= gyro_history_.size();

        gyro_variance_ = Eigen::Vector3d::Zero();
        for (const auto& gyro : gyro_history_) {
            Eigen::Vector3d diff = gyro - gyro_mean_;
            gyro_variance_ += diff.cwiseProduct(diff);
        }
        gyro_variance_ /= gyro_history_.size();
    }
}

bool AdaptiveFilter::isOutlier(const Eigen::Vector3d& measurement,
                               const Eigen::Vector3d& mean,
                               const Eigen::Vector3d& variance) {
    Eigen::Vector3d deviation = (measurement - mean).cwiseAbs();
    Eigen::Vector3d std_dev = variance.cwiseSqrt();

    // Check if any component exceeds threshold
    for (int i = 0; i < 3; ++i) {
        if (std_dev(i) > 1e-6 && deviation(i) > config_.outlier_threshold * std_dev(i)) {
            return true;
        }
    }

    return false;
}

AdaptiveFilter::Stats AdaptiveFilter::getStatistics() const {
    Stats stats;
    stats.acc_bias_norm = acc_bias_.norm();
    stats.gyro_bias_norm = gyro_bias_.norm();
    stats.acc_variance = acc_variance_.norm();
    stats.gyro_variance = gyro_variance_.norm();
    stats.outliers_rejected = outliers_rejected_;
    return stats;
}

// GravityBiasEstimator implementation
GravityBiasEstimator::GravityBiasEstimator()
    : bias_estimate_(Eigen::Vector3d::Zero()),
      variance_(Eigen::Vector3d::Ones()),
      samples_(0) {
}

void GravityBiasEstimator::updateStationary(const Eigen::Vector3d& acc_measurement) {
    // When stationary, accelerometer should measure only gravity
    const double g = 9.81;
    Eigen::Vector3d expected_gravity(0, 0, -g);

    // Estimate bias as difference from expected
    Eigen::Vector3d bias = acc_measurement - expected_gravity;

    // Update estimate with exponential moving average
    const double alpha = 0.1;
    bias_estimate_ = (1 - alpha) * bias_estimate_ + alpha * bias;

    // Update variance
    Eigen::Vector3d error = bias - bias_estimate_;
    variance_ = (1 - alpha) * variance_ + alpha * error.cwiseProduct(error);

    samples_++;
}

void GravityBiasEstimator::updateWithGravity(const Eigen::Vector3d& acc_measurement,
                                            const Eigen::Quaterniond& attitude) {
    // Rotate gravity to body frame
    const double g = 9.81;
    Eigen::Vector3d gravity_ecef(0, 0, -g);
    Eigen::Vector3d gravity_body = attitude.inverse() * gravity_ecef;

    // Estimate bias
    Eigen::Vector3d bias = acc_measurement - gravity_body;

    // Update estimate
    const double alpha = 0.05;
    bias_estimate_ = (1 - alpha) * bias_estimate_ + alpha * bias;

    // Update variance
    Eigen::Vector3d error = bias - bias_estimate_;
    variance_ = (1 - alpha) * variance_ + alpha * error.cwiseProduct(error);

    samples_++;
}

// AdaptiveCovarianceTuner implementation
AdaptiveCovarianceTuner::AdaptiveCovarianceTuner()
    : Q_(Eigen::MatrixXd::Identity(15, 15) * 1e-4),
      R_(Eigen::MatrixXd::Identity(9, 9) * 1e-2),
      innovation_mean_(Eigen::VectorXd::Zero(9)),
      innovation_covariance_(Eigen::MatrixXd::Identity(9, 9)),
      adaptation_rate_(0.01),
      window_size_(100) {
}

void AdaptiveCovarianceTuner::updateFromInnovation(const Eigen::VectorXd& innovation,
                                                   const Eigen::MatrixXd& S) {
    // Add to history
    innovation_history_.push_back(innovation);
    if (innovation_history_.size() > window_size_) {
        innovation_history_.pop_front();
    }

    // Calculate statistics
    innovation_mean_ = Eigen::VectorXd::Zero(innovation.size());
    for (const auto& inn : innovation_history_) {
        innovation_mean_ += inn;
    }
    innovation_mean_ /= innovation_history_.size();

    // Update covariance estimate
    innovation_covariance_ = Eigen::MatrixXd::Zero(innovation.size(), innovation.size());
    for (const auto& inn : innovation_history_) {
        Eigen::VectorXd diff = inn - innovation_mean_;
        innovation_covariance_ += diff * diff.transpose();
    }
    innovation_covariance_ /= innovation_history_.size();

    // Adapt measurement noise based on actual vs expected innovation covariance
    Eigen::MatrixXd covariance_error = innovation_covariance_ - S;
    R_ += adaptation_rate_ * covariance_error;

    // Ensure positive definiteness
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(R_);
    Eigen::VectorXd eigenvalues = solver.eigenvalues();
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) < 1e-6) {
            eigenvalues(i) = 1e-6;
        }
    }
    R_ = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}

void AdaptiveCovarianceTuner::setMotionMode(AdaptiveCovarianceTuner::MotionMode mode) {
    switch (mode) {
        case STATIONARY:
            // Reduce process noise when stationary
            Q_ *= 0.1;
            break;
        case CONSTANT_VELOCITY:
            // Normal process noise
            Q_ = Eigen::MatrixXd::Identity(15, 15) * 1e-4;
            break;
        case MANEUVERING:
            // Increase process noise during maneuvers
            Q_ *= 10.0;
            break;
    }
}