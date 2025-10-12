#include "measurements/gravity_ukf_measurement.h"
#include "core/fusion/state.h"
#include <spdlog/spdlog.h>
#include <algorithm>

namespace measurements {

GravityUKFMeasurement::GravityUKFMeasurement(const Eigen::Vector3d& normalized_specific_force,
                                             double cross_product_sigma,
                                             double timestamp)
    : z_measured_(normalized_specific_force),
      sigma_(cross_product_sigma),
      timestamp_(timestamp),
      gravity_mag_(9.80665) {}

Eigen::VectorXd GravityUKFMeasurement::predict(const aion::State& state) const {
    // Cross-product gravity measurement model for attitude correction
    // Uses tangent space residual y = z × h, linear in small attitude errors

    // Get rotation matrix (body to nav)
    const Eigen::Matrix3d R_nb = state.quaternion.toRotationMatrix();
    const Eigen::Matrix3d R_bn = R_nb.transpose();  // nav to body

    // Expected gravity vector in navigation frame (pointing down)
    const Eigen::Vector3d e3(0.0, 0.0, 1.0);  // [0, 0, 1] down in NED

    // Expected gravity direction in body frame
    // At rest: specific force = -R_bn * g_n, so normalized specific force = R_bn * e3
    Eigen::Vector3d h = R_bn * e3;  // body-frame unit vector (no negation!)
    h.normalize();

    // Measured specific force direction (normalized)
    Eigen::Vector3d z = z_measured_.normalized();

    // Frame/sign validation logging
    const double dot_product = z.dot(h);
    const double cos_tilt = std::clamp(dot_product, -1.0, 1.0);
    const double tilt_deg = std::acos(cos_tilt) * 180.0 / M_PI;

    // Cross-product residual in tangent space of S² (approximately δθ for small angles)
    const Eigen::Vector3d y = z.cross(h);

    // Log validation info every 1000th call to avoid spam
    static int call_count = 0;
    if (++call_count % 1000 == 0) {
        spdlog::info("Gravity validation: dot(z,h)={:.3f}, tilt={:.2f}°, |y|={:.4f}",
                     dot_product, tilt_deg, y.norm());
    }

    // Return cross-product residual
    Eigen::VectorXd prediction(3);
    prediction = y;
    return prediction;
}

Eigen::VectorXd GravityUKFMeasurement::getValue() const {
    // For cross-product gravity measurement:
    // The measurement value should be zero (perfect alignment: z × h = 0)
    // The predict() function computes the expected cross-product residual
    // The innovation (predict - measurement) drives the attitude correction

    Eigen::VectorXd measurement_value = Eigen::Vector3d::Zero();
    return measurement_value;
}

Eigen::MatrixXd GravityUKFMeasurement::getCovariance() const {
    // Diagonal covariance matrix for 3D cross-product measurement
    Eigen::MatrixXd R = Eigen::Matrix3d::Identity() * (sigma_ * sigma_);
    return R;
}

double GravityUKFMeasurement::getTime() const {
    return timestamp_;
}

} // namespace measurements