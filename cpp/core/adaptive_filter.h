#pragma once

#include <Eigen/Dense>
#include "types.h"
#include <deque>
#include <memory>

/**
 * Adaptive filter that learns and corrects systematic errors
 * Uses online learning to improve navigation accuracy
 */
class AdaptiveFilter {
public:
    struct Config {
        int window_size;
        double learning_rate;
        double bias_decay;
        bool enable_outlier_rejection;
        double outlier_threshold;  // Standard deviations

        Config() : window_size(100),
                   learning_rate(0.01),
                   bias_decay(0.999),
                   enable_outlier_rejection(true),
                   outlier_threshold(3.0) {}
    };

    AdaptiveFilter(const Config& config = Config());

    // Learn bias patterns from recent measurements
    void updateBiasEstimate(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro);

    // Get corrected measurements
    Eigen::Vector3d correctAccelerometer(const Eigen::Vector3d& raw_acc);
    Eigen::Vector3d correctGyroscope(const Eigen::Vector3d& raw_gyro);

    // Adaptive scaling based on motion state
    void updateScaleFactors(const State& state);

    // Get current estimates
    Eigen::Vector3d getAccBias() const { return acc_bias_; }
    Eigen::Vector3d getGyroBias() const { return gyro_bias_; }
    Eigen::Vector3d getAccScale() const { return acc_scale_; }
    Eigen::Vector3d getGyroScale() const { return gyro_scale_; }

    // Statistics
    struct Stats {
        double acc_bias_norm;
        double gyro_bias_norm;
        double acc_variance;
        double gyro_variance;
        int outliers_rejected;
    };

    Stats getStatistics() const;

private:
    Config config_;

    // Estimated biases (learned online)
    Eigen::Vector3d acc_bias_;
    Eigen::Vector3d gyro_bias_;

    // Scale factors
    Eigen::Vector3d acc_scale_;
    Eigen::Vector3d gyro_scale_;

    // Recent measurement history
    std::deque<Eigen::Vector3d> acc_history_;
    std::deque<Eigen::Vector3d> gyro_history_;

    // Statistics
    Eigen::Vector3d acc_mean_;
    Eigen::Vector3d acc_variance_;
    Eigen::Vector3d gyro_mean_;
    Eigen::Vector3d gyro_variance_;

    int outliers_rejected_;

    // Helper methods
    bool isOutlier(const Eigen::Vector3d& measurement,
                   const Eigen::Vector3d& mean,
                   const Eigen::Vector3d& variance);

    void updateStatistics();
};

/**
 * Gravity-aided bias estimator
 * Uses gravity observations to estimate accelerometer bias
 */
class GravityBiasEstimator {
public:
    GravityBiasEstimator();

    // Estimate bias when vehicle is stationary
    void updateStationary(const Eigen::Vector3d& acc_measurement);

    // Estimate bias using gravity direction
    void updateWithGravity(const Eigen::Vector3d& acc_measurement,
                          const Eigen::Quaterniond& attitude);

    // Get estimated bias
    Eigen::Vector3d getBias() const { return bias_estimate_; }

    // Check if estimate is reliable
    bool isReliable() const { return samples_ > min_samples_ && variance_.norm() < max_variance_; }

private:
    Eigen::Vector3d bias_estimate_;
    Eigen::Vector3d variance_;
    int samples_;

    static constexpr int min_samples_ = 50;
    static constexpr double max_variance_ = 0.1;
};

/**
 * Adaptive covariance tuning
 * Adjusts process and measurement noise based on filter performance
 */
class AdaptiveCovarianceTuner {
public:
    AdaptiveCovarianceTuner();

    // Update based on innovation statistics
    void updateFromInnovation(const Eigen::VectorXd& innovation,
                             const Eigen::MatrixXd& S);  // Innovation covariance

    // Get adapted noise parameters
    Eigen::MatrixXd getProcessNoise() const { return Q_; }
    Eigen::MatrixXd getMeasurementNoise() const { return R_; }

    // Adjust for different motion states
    enum MotionMode { STATIONARY, CONSTANT_VELOCITY, MANEUVERING };
    void setMotionMode(MotionMode mode);

private:
    Eigen::MatrixXd Q_;  // Process noise
    Eigen::MatrixXd R_;  // Measurement noise

    // Innovation statistics
    std::deque<Eigen::VectorXd> innovation_history_;
    Eigen::VectorXd innovation_mean_;
    Eigen::MatrixXd innovation_covariance_;

    // Tuning parameters
    double adaptation_rate_;
    int window_size_;
};