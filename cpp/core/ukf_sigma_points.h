#pragma once
#include "types.h"
#include <Eigen/Dense>
#include <vector>

// Forward declaration
class UKF;

// UKF constants (to avoid circular dependency)
constexpr int UKF_ERROR_STATE_DIM = 15;
constexpr int UKF_NUM_SIGMA_POINTS = 2 * UKF_ERROR_STATE_DIM + 1;

// Sigma point structure
struct SigmaPoint {
    State state;
    double weight_mean;
    double weight_cov;
    
    SigmaPoint() : weight_mean(0), weight_cov(0) {}
    SigmaPoint(const State& s, double wm, double wc) 
        : state(s), weight_mean(wm), weight_cov(wc) {}
};

/**
 * @brief UKF Sigma Point Generation and Management
 * 
 * This class handles all sigma point operations including generation,
 * propagation, and mean/covariance computation. Separated from main UKF
 * for better maintainability.
 */
class UKFSigmaPoints {
public:
    explicit UKFSigmaPoints(UKF& ukf);  // Note: non-const reference needed
    
    // Generate sigma points from current state and covariance
    void generate(const State& nominal_state, 
                  const Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>& P,
                  double lambda);
    
    // Access sigma points
    const std::vector<SigmaPoint>& getSigmaPoints() const { return sigma_points_; }
    std::vector<SigmaPoint>& getSigmaPoints() { return sigma_points_; }
    
    // Propagate all sigma points through motion model
    std::vector<State> propagateStates(const ImuSample& imu, double dt);
    
    // Compute mean and covariance from propagated states
    State computeMeanState(const std::vector<State>& states, 
                          const Eigen::VectorXd& weights_mean);
    
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> 
    computeCovariance(const std::vector<State>& states,
                      const State& mean_state,
                      const Eigen::VectorXd& weights_cov);

private:
    UKF& ukf_;  // Non-const reference
    std::vector<SigmaPoint> sigma_points_;
    
    // Matrix square root computation with fallback
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> 
    computeMatrixSqrt(const Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>& matrix);
    
    // Advanced quaternion averaging
    Eigen::Quaterniond computeMeanQuaternion(const std::vector<State>& states,
                                           const Eigen::VectorXd& weights_mean);
};