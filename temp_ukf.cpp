#include "ukf.h"
#include "ukf_sigma_points.h"
#include "ukf_measurements.h" 
#include "ukf_math_utils.h"
#include "normal_gravity.h"
#include <cmath>
#include <iostream>
#include <memory>

UKF::UKF(const UKFConfig& cfg) : cfg_(cfg) {
    // Validate configuration
    if (!cfg_.validate()) {
        std::cerr << "Invalid UKF configuration, using defaults\n";
        cfg_.setDefaults();
    }
    
    computeWeights();
    
    // Initialize integrity monitoring
    integrity_stats_.nees = 0.0;
    integrity_stats_.nis = 0.0;
    integrity_stats_.nees_pass_rate = 1.0;
    integrity_stats_.nis_pass_rate = 1.0;
    
    // Initialize modular components
    sigma_points_manager_ = std::make_unique<UKFSigmaPoints>(*this);
    measurements_manager_ = std::make_unique<UKFMeasurements>(*this);
    
    std::cout << "UKF initialized with modular architecture\n";
}

UKF::~UKF() {
    // unique_ptr automatically handles cleanup
}

void UKF::computeWeights() {
    lambda_ = cfg_.alpha * cfg_.alpha * (ERROR_STATE_DIM + cfg_.kappa) - ERROR_STATE_DIM;
    
    weights_mean_ = Eigen::VectorXd(NUM_SIGMA_POINTS);
    weights_cov_ = Eigen::VectorXd(NUM_SIGMA_POINTS);
    
    // Weight for center point
    weights_mean_(0) = lambda_ / (ERROR_STATE_DIM + lambda_);
    weights_cov_(0) = lambda_ / (ERROR_STATE_DIM + lambda_) + (1 - cfg_.alpha * cfg_.alpha + cfg_.beta);
    
    // Weights for other sigma points
    double weight = 0.5 / (ERROR_STATE_DIM + lambda_);
    for (int i = 1; i < NUM_SIGMA_POINTS; ++i) {
        weights_mean_(i) = weight;
        weights_cov_(i) = weight;
    }
}

void UKF::init(const State& x0, const Eigen::Matrix<double, ERROR_STATE_DIM, ERROR_STATE_DIM>& P0) {
    nominal_state_ = x0;
    P_ = P0;
    UKFMathUtils::enforcePositiveDefinite<ERROR_STATE_DIM>(P_);
}

// Old methods removed - now handled by modular components
}

void UKF::updateVirtualDoppler(const Eigen::Vector3d& velocity_meas, const Eigen::Matrix3d& R_virtual) {
    // Placeholder implementation
    std::cerr << "updateVirtualDoppler not implemented in modular version\n";
}

void UKF::updateScalarPseudo(double z_meas, double z_pred, double S_pred, double R_scalar) {
    // Placeholder implementation
    std::cerr << "updateScalarPseudo not implemented in modular version\n";
}

