#pragma once
#include "ukf.h"
#include <Eigen/Dense>
#include <vector>

/**
 * @brief UKF Sigma Point Generation and Management
 * 
 * This class handles all sigma point operations including generation,
 * propagation, and mean/covariance computation. Separated from main UKF
 * for better maintainability.
 */
class UKFSigmaPoints {
public:
    explicit UKFSigmaPoints(const UKF& ukf);
    
    // Generate sigma points from current state and covariance
    void generate(const State& nominal_state, 
                  const Eigen::Matrix<double, UKF::ERROR_STATE_DIM, UKF::ERROR_STATE_DIM>& P,
                  double lambda);
    
    // Access sigma points
    const std::vector<UKF::SigmaPoint>& getSigmaPoints() const { return sigma_points_; }
    std::vector<UKF::SigmaPoint>& getSigmaPoints() { return sigma_points_; }
    
    // Propagate all sigma points through motion model
    std::vector<State> propagateStates(const ImuSample& imu, double dt) const;
    
    // Compute mean and covariance from propagated states
    State computeMeanState(const std::vector<State>& states, 
                          const Eigen::VectorXd& weights_mean) const;
    
    Eigen::Matrix<double, UKF::ERROR_STATE_DIM, UKF::ERROR_STATE_DIM> 
    computeCovariance(const std::vector<State>& states,
                      const State& mean_state,
                      const Eigen::VectorXd& weights_cov) const;

private:
    const UKF& ukf_;
    std::vector<UKF::SigmaPoint> sigma_points_;
    
    // Matrix square root computation with fallback
    Eigen::Matrix<double, UKF::ERROR_STATE_DIM, UKF::ERROR_STATE_DIM> 
    computeMatrixSqrt(const Eigen::Matrix<double, UKF::ERROR_STATE_DIM, UKF::ERROR_STATE_DIM>& matrix) const;
    
    // Advanced quaternion averaging
    Eigen::Quaterniond computeMeanQuaternion(const std::vector<State>& states,
                                           const Eigen::VectorXd& weights_mean) const;
};