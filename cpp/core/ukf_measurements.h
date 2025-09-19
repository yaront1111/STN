#pragma once
#include "types.h"
#include <Eigen/Dense>
#include <functional>

// Forward declarations
class UKF;
class GravityGradientProvider;

/**
 * @brief UKF Measurement Update Handler
 * 
 * This class manages all measurement updates for the UKF, including
 * gravity gradients, map matching, magnetometer, and other sensors.
 * Provides a unified interface for different measurement types.
 */
class UKFMeasurements {
public:
    explicit UKFMeasurements(UKF& ukf, GravityGradientProvider* gravity_provider = nullptr);
    
    // Generic measurement update template
    template<int MeasDim>
    void updateMeasurement(
        const Eigen::Matrix<double, MeasDim, 1>& measurement,
        const Eigen::Matrix<double, MeasDim, MeasDim>& noise_cov,
        std::function<Eigen::Matrix<double, MeasDim, 1>(const State&)> measurement_model,
        bool use_joseph_form = true  // DEFAULT TO JOSEPH FORM FOR STABILITY
    );

    // Measurement update without chi-square gating (for trusted measurements like map matching)
    template<int MeasDim>
    void updateMeasurementNoGate(
        const Eigen::Matrix<double, MeasDim, 1>& measurement,
        const Eigen::Matrix<double, MeasDim, MeasDim>& noise_cov,
        std::function<Eigen::Matrix<double, MeasDim, 1>(const State&)> measurement_model,
        bool use_joseph_form = true  // DEFAULT TO JOSEPH FORM FOR STABILITY
    );
    
    // Specific measurement updates
    void updateGravityGradient(const Eigen::Matrix3d& measured, const Eigen::Matrix3d& R);
    void updateGravityMapMatch(const Eigen::Vector3d& matched_position_ECEF, const Eigen::Matrix3d& R_position);
    void updateMagnetometer(const Eigen::Vector3d& mag_body, const Eigen::Vector3d& mag_ref_ECEF, const Eigen::Matrix3d& R_mag);
    void updateBarometer(double pressure_altitude, double noise);
    void updateZUPT(const Eigen::Matrix3d& R_vel);
    void updateTerrainAltitude(double radar_alt, double terrain_height, double noise);
    
    // Outlier detection and rejection
    bool checkInnovation(const Eigen::VectorXd& innovation,
                        const Eigen::MatrixXd& S,
                        double chi_square_threshold = 9.21) const;  // 99% confidence, 3 DOF

    // Gravity gradient prediction (made public for tensor invariants)
    Eigen::Matrix3d predictGravityGradient(const State& state) const;

private:
    UKF& ukf_;
    GravityGradientProvider* gravity_provider_;  // Non-owning pointer

    // Internal measurement update with gate control
    template<int MeasDim>
    void updateMeasurementInternal(
        const Eigen::Matrix<double, MeasDim, 1>& measurement,
        const Eigen::Matrix<double, MeasDim, MeasDim>& noise_cov,
        std::function<Eigen::Matrix<double, MeasDim, 1>(const State&)> measurement_model,
        bool use_joseph_form,
        bool apply_gate
    );

    // Utility functions
    Eigen::VectorXd flattenMatrix(const Eigen::Matrix3d& matrix) const;
    Eigen::Matrix3d unflattenVector(const Eigen::VectorXd& vector) const;
    
    // Robust covariance update using Joseph form
    void updateCovarianceJoseph(
        Eigen::Matrix<double, 15, 15>& P,
        const Eigen::MatrixXd& K,
        const Eigen::MatrixXd& H,
        const Eigen::MatrixXd& R
    );
};