#pragma once

#include "../core/fusion/sr_ukf.h"
#include "../core/fusion/state.h"
#include <Eigen/Dense>

namespace measurements {

/**
 * UKF measurement for gravity-referenced attitude correction using cross-product.
 * Implements y = z × h where:
 * - z = f_b/||f_b|| (normalized specific force in body frame)
 * - h = R_bn*(-g_n/||g_n||) (expected gravity direction in body frame)
 * - y = cross product (3D measurement)
 */
class GravityUKFMeasurement : public aion::MeasurementBase {
public:
    // Constructor takes normalized specific force in body frame and measurement noise
    GravityUKFMeasurement(const Eigen::Vector3d& normalized_specific_force,
                          double cross_product_sigma,
                          double timestamp = 0.0);
    ~GravityUKFMeasurement() override = default;

    // MeasurementBase implementation
    Eigen::VectorXd predict(const aion::State& state) const override;
    Eigen::VectorXd getValue() const override;
    Eigen::MatrixXd getCovariance() const override;
    int getDimension() const override { return 3; }  // 3D cross product
    double getTime() const override;

private:
    Eigen::Vector3d z_measured_;    // Normalized specific force in body frame
    double sigma_;                  // Cross-product measurement noise (1-sigma)
    double timestamp_;              // Measurement timestamp
    double gravity_mag_;            // Gravity magnitude (for normalization)
};

} // namespace measurements