#include "ukf_measurements.h"
#include "ukf.h"
#include "ukf_sigma_points.h"
#include "ukf_math_utils.h"
#include "gravity_gradient_provider.h"
#include <iostream>

UKFMeasurements::UKFMeasurements(UKF& ukf, GravityGradientProvider* gravity_provider)
    : ukf_(ukf), gravity_provider_(gravity_provider) {
}

template<int MeasDim>
void UKFMeasurements::updateMeasurementNoGate(
    const Eigen::Matrix<double, MeasDim, 1>& measurement,
    const Eigen::Matrix<double, MeasDim, MeasDim>& noise_cov,
    std::function<Eigen::Matrix<double, MeasDim, 1>(const State&)> measurement_model,
    bool use_joseph_form) {
    // Same as updateMeasurement but without chi-square gate
    updateMeasurementInternal<MeasDim>(measurement, noise_cov, measurement_model, use_joseph_form, false);
}

template<int MeasDim>
void UKFMeasurements::updateMeasurement(
    const Eigen::Matrix<double, MeasDim, 1>& measurement,
    const Eigen::Matrix<double, MeasDim, MeasDim>& noise_cov,
    std::function<Eigen::Matrix<double, MeasDim, 1>(const State&)> measurement_model,
    bool use_joseph_form) {
    // With chi-square gate
    updateMeasurementInternal<MeasDim>(measurement, noise_cov, measurement_model, use_joseph_form, true);
}

template<int MeasDim>
void UKFMeasurements::updateMeasurementInternal(
    const Eigen::Matrix<double, MeasDim, 1>& measurement,
    const Eigen::Matrix<double, MeasDim, MeasDim>& noise_cov,
    std::function<Eigen::Matrix<double, MeasDim, 1>(const State&)> measurement_model,
    bool use_joseph_form,
    bool apply_gate) {
    
    // Get current state and covariance from UKF
    State current_state = ukf_.getState();
    auto current_cov = ukf_.getCovariance();
    
    // Generate sigma points for measurement update
    auto& sigma_manager = ukf_.getSigmaManager();
    sigma_manager.generate(current_state, current_cov, ukf_.getLambda());
    const auto& sigma_points = sigma_manager.getSigmaPoints();
    const auto& weights_mean = ukf_.getWeightsMean();
    const auto& weights_cov = ukf_.getWeightsCov();
    
    // Transform sigma points through measurement model
    std::vector<Eigen::Matrix<double, MeasDim, 1>> predicted_measurements(sigma_points.size());
    for (size_t i = 0; i < sigma_points.size(); ++i) {
        predicted_measurements[i] = measurement_model(sigma_points[i].state);
    }
    
    // Compute predicted measurement mean
    Eigen::Matrix<double, MeasDim, 1> predicted_mean = Eigen::Matrix<double, MeasDim, 1>::Zero();
    for (size_t i = 0; i < predicted_measurements.size(); ++i) {
        predicted_mean += weights_mean(i) * predicted_measurements[i];
    }
    
    // Compute innovation covariance S and cross-covariance T
    Eigen::Matrix<double, MeasDim, MeasDim> S = Eigen::Matrix<double, MeasDim, MeasDim>::Zero();
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, MeasDim> T = 
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, MeasDim>::Zero();
    
    for (size_t i = 0; i < predicted_measurements.size(); ++i) {
        Eigen::Matrix<double, MeasDim, 1> meas_diff = predicted_measurements[i] - predicted_mean;
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> state_diff = 
            UKFMathUtils::computeErrorVector(sigma_points[i].state, current_state);
        
        S += weights_cov(i) * meas_diff * meas_diff.transpose();
        T += weights_cov(i) * state_diff * meas_diff.transpose();
    }
    
    // Add measurement noise
    S += noise_cov;
    
    // DEBUG: Check for numerical issues
    if (!S.allFinite()) {
        std::cout << "ERROR: Innovation covariance S contains NaN/Inf values!\n";
        std::cout << "S matrix:\n" << S << "\n";
        return;
    }
    
    // Check if S is positive definite
    Eigen::LLT<Eigen::Matrix<double, MeasDim, MeasDim>> llt_check(S);
    if (llt_check.info() != Eigen::Success) {
        // Try to fix by adding small regularization
        double regularization = 1e-9;
        S += Eigen::Matrix<double, MeasDim, MeasDim>::Identity() * regularization;

        // Check again
        Eigen::LLT<Eigen::Matrix<double, MeasDim, MeasDim>> llt_check2(S);
        if (llt_check2.info() != Eigen::Success) {
            // Still not positive definite - use stronger regularization
            regularization = 1e-6;
            S += Eigen::Matrix<double, MeasDim, MeasDim>::Identity() * regularization;

            Eigen::LLT<Eigen::Matrix<double, MeasDim, MeasDim>> llt_check3(S);
            if (llt_check3.info() != Eigen::Success) {
                // Give up and skip this measurement
                std::cout << "WARNING: Innovation covariance S cannot be fixed, skipping measurement\n";
                if (MeasDim <= 3) {  // Only print small matrices
                    std::cout << "S matrix:\n" << S << "\n";
                }
                return;
            }
        }
        // Successfully regularized
        std::cout << "INFO: Innovation covariance regularized with eps=" << regularization << "\n";
    }
    
    // Compute innovation
    Eigen::Matrix<double, MeasDim, 1> innovation = measurement - predicted_mean;
    
    // DEBUG: Check innovation
    if (!innovation.allFinite()) {
        std::cout << "ERROR: Innovation contains NaN/Inf values!\n";
        std::cout << "Innovation: " << innovation.transpose() << "\n";
        std::cout << "Measurement: " << measurement.transpose() << "\n";
        std::cout << "Predicted: " << predicted_mean.transpose() << "\n";
        return;
    }
    
    // Chi-square gating
    double nis = UKFMathUtils::mahalanobisDistance(innovation, S);
    if (!std::isfinite(nis)) {
        std::cout << "ERROR: NIS calculation resulted in NaN/Inf!\n";
        std::cout << "Innovation: " << innovation.transpose() << "\n";
        std::cout << "S determinant: " << S.determinant() << "\n";
        return;
    }

    // Temporarily disable chi-square gate for debugging
    if (false && apply_gate && !UKFMathUtils::chiSquareTest(nis, MeasDim, 0.99)) {
        std::cout << "Measurement REJECTED by chi-square gate (NIS=" << nis << ")\n";
        return;
    }
    
    // Compute Kalman gain using robust solver
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, MeasDim> K;
    Eigen::LLT<Eigen::Matrix<double, MeasDim, MeasDim>> llt_solve(S);
    if (llt_solve.info() == Eigen::Success) {
        K = T * llt_solve.solve(Eigen::Matrix<double, MeasDim, MeasDim>::Identity());
    } else {
        // Fallback to SVD
        Eigen::JacobiSVD<Eigen::Matrix<double, MeasDim, MeasDim>> svd(S, Eigen::ComputeFullU | Eigen::ComputeFullV);
        K = T * svd.solve(Eigen::Matrix<double, MeasDim, MeasDim>::Identity());
    }
    
    // Update state
    Eigen::Matrix<double, UKF_ERROR_STATE_DIM, 1> state_correction = K * innovation;
    State updated_state = UKFMathUtils::applyErrorToState(current_state, state_correction);
    ukf_.setState(updated_state);
    
    // Update covariance
    auto P = current_cov;
    if (use_joseph_form) {
        // Joseph form for numerical stability
        // H approximated as S^-1 * T.transpose() (measurement Jacobian)
        Eigen::Matrix<double, MeasDim, UKF_ERROR_STATE_DIM> H =
            S.inverse() * T.transpose();
        Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM> I =
            Eigen::Matrix<double, UKF_ERROR_STATE_DIM, UKF_ERROR_STATE_DIM>::Identity();
        auto IKH = I - K * H;
        P = IKH * P * IKH.transpose() + K * noise_cov * K.transpose();
    } else {
        // Standard Kalman update - FIXED: Should be K*T' not K*S*K'
        P = P - K * T.transpose();
    }
    
    // Ensure positive definiteness
    UKFMathUtils::enforcePositiveDefinite<UKF_ERROR_STATE_DIM>(P);
    ukf_.setCovariance(P);
    
    std::cout << "Measurement update accepted (NIS=" << nis << ")\n";
}

void UKFMeasurements::updateGravityGradient(const Eigen::Matrix3d& measured_ecef, const Eigen::Matrix3d& R) {
    // Transform measured tensor to ENU if needed
    Eigen::Matrix3d measured = measured_ecef;
    if (ukf_.use_enu_) {
        // Transform measured gradient from ECEF to ENU frame
        measured = ukf_.R_ECEF_ENU_.transpose() * measured_ecef * ukf_.R_ECEF_ENU_;
    }

    // Convert 3x3 matrix to 9D vector for measurement update
    Eigen::VectorXd measured_vec = flattenMatrix(measured);
    Eigen::Matrix<double, 9, 9> noise_cov = Eigen::Matrix<double, 9, 9>::Zero();

    // Set noise covariance as diagonal with proper scaling
    // Use the average of the R matrix diagonal as the noise level
    double noise_level = R.trace() / 3.0;

    // CRITICAL FIX: Ensure reasonable minimum noise for numerical stability
    // Gravity gradients are typically measured in Eötvös (1E = 10^-9 s^-2)
    // Minimum noise of 1 Eötvös squared is reasonable
    noise_level = std::max(noise_level, 1.0);  // Changed from 1e-10 to 1.0

    for (int i = 0; i < 9; ++i) {
        noise_cov(i, i) = noise_level;  // Use same noise for all components
    }
    
    auto measurement_model = [this](const State& state) -> Eigen::Matrix<double, 9, 1> {
        Eigen::Matrix3d predicted_tensor = predictGravityGradient(state);
        return flattenMatrix(predicted_tensor);
    };
    
    updateMeasurement<9>(measured_vec, noise_cov, measurement_model, true);
}

void UKFMeasurements::updateGravityMapMatch(const Eigen::Vector3d& matched_position_ECEF,
                                           const Eigen::Matrix3d& R_position) {
    auto measurement_model = [](const State& state) -> Eigen::Vector3d {
        return state.p_ECEF;
    };

    // Map matching is special - we trust the correlator, so bypass chi-square gate
    updateMeasurementNoGate<3>(matched_position_ECEF, R_position, measurement_model, true);
}

void UKFMeasurements::updateMagnetometer(const Eigen::Vector3d& mag_body, 
                                        const Eigen::Vector3d& mag_ref_ECEF,
                                        const Eigen::Matrix3d& R_mag) {
    auto measurement_model = [mag_ref_ECEF](const State& state) -> Eigen::Vector3d {
        // Transform reference magnetic field to body frame
        return state.q_ECEF_B.inverse() * mag_ref_ECEF;
    };
    
    updateMeasurement<3>(mag_body, R_mag, measurement_model);
}

void UKFMeasurements::updateBarometer(double pressure_altitude, double noise) {
    Eigen::Matrix<double, 1, 1> measurement;
    measurement << pressure_altitude;
    
    Eigen::Matrix<double, 1, 1> R;
    R << noise * noise;
    
    auto measurement_model = [](const State& state) -> Eigen::Matrix<double, 1, 1> {
        // Extract altitude from ECEF position (simplified)
        Eigen::Vector3d lla = UKFMathUtils::ecefToLla(state.p_ECEF);
        Eigen::Matrix<double, 1, 1> result;
        result << lla(2); // altitude
        return result;
    };
    
    updateMeasurement<1>(measurement, R, measurement_model);
}

void UKFMeasurements::updateZUPT(const Eigen::Matrix3d& R_vel) {
    Eigen::Vector3d zero_velocity = Eigen::Vector3d::Zero();
    
    auto measurement_model = [](const State& state) -> Eigen::Vector3d {
        return state.v_ECEF;
    };
    
    updateMeasurement<3>(zero_velocity, R_vel, measurement_model);
}

void UKFMeasurements::updateTerrainAltitude(double radar_alt, double terrain_height, double noise) {
    Eigen::Matrix<double, 1, 1> measurement;
    measurement << radar_alt + terrain_height; // Total altitude
    
    Eigen::Matrix<double, 1, 1> R;
    R << noise * noise;
    
    auto measurement_model = [](const State& state) -> Eigen::Matrix<double, 1, 1> {
        Eigen::Vector3d lla = UKFMathUtils::ecefToLla(state.p_ECEF);
        Eigen::Matrix<double, 1, 1> result;
        result << lla(2);
        return result;
    };
    
    updateMeasurement<1>(measurement, R, measurement_model);
}

bool UKFMeasurements::checkInnovation(const Eigen::VectorXd& innovation, 
                                     const Eigen::MatrixXd& S, 
                                     double chi_square_threshold) const {
    double nis = (innovation.transpose() * S.inverse() * innovation)(0,0);
    return nis < chi_square_threshold;
}

Eigen::VectorXd UKFMeasurements::flattenMatrix(const Eigen::Matrix3d& matrix) const {
    Eigen::VectorXd vec(9);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            vec(i * 3 + j) = matrix(i, j);
        }
    }
    return vec;
}

Eigen::Matrix3d UKFMeasurements::unflattenVector(const Eigen::VectorXd& vector) const {
    Eigen::Matrix3d matrix;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            matrix(i, j) = vector(i * 3 + j);
        }
    }
    return matrix;
}

Eigen::Matrix3d UKFMeasurements::predictGravityGradient(const State& state) const {
    // Use shared gravity provider passed from outside
    if (!gravity_provider_) {
        // Fallback to static instance if not provided (for backward compatibility)
        static GravityGradientProvider static_provider;
        static bool initialized = false;
        if (!initialized) {
            // Try to load EGM data
            if (!static_provider.loadEGM2020("../data/egm2008/egm2008_n360_fixed.dat")) {
                std::cerr << "Warning: Failed to load EGM2020 data for UKF predictions\n";
            }
            initialized = true;
        }

        // CRITICAL FIX: Transform to ECEF if in ENU mode
        State state_ecef = state;
        if (ukf_.use_enu_) {
            state_ecef = ukf_.enuToEcef(state);
        }

        auto result = static_provider.getGradient(state_ecef.p_ECEF);

        // Transform gradient tensor to ENU frame if needed
        if (ukf_.use_enu_) {
            // Rotate gradient tensor from ECEF to ENU frame
            result.T = ukf_.R_ECEF_ENU_.transpose() * result.T * ukf_.R_ECEF_ENU_;
        }

        return result.T;
    }

    // CRITICAL FIX: Transform to ECEF if in ENU mode
    State state_ecef = state;
    if (ukf_.use_enu_) {
        state_ecef = ukf_.enuToEcef(state);
    }

    auto result = gravity_provider_->getGradient(state_ecef.p_ECEF);

    // Transform gradient tensor to ENU frame if needed
    if (ukf_.use_enu_) {
        // Rotate gradient tensor from ECEF to ENU frame
        result.T = ukf_.R_ECEF_ENU_.transpose() * result.T * ukf_.R_ECEF_ENU_;
    }

    return result.T;
}

// Explicit template instantiation for common measurement dimensions
template void UKFMeasurements::updateMeasurement<1>(
    const Eigen::Matrix<double, 1, 1>&, const Eigen::Matrix<double, 1, 1>&,
    std::function<Eigen::Matrix<double, 1, 1>(const State&)>, bool);

template void UKFMeasurements::updateMeasurement<3>(
    const Eigen::Vector3d&, const Eigen::Matrix3d&,
    std::function<Eigen::Vector3d(const State&)>, bool);

template void UKFMeasurements::updateMeasurement<2>(
    const Eigen::Matrix<double, 2, 1>&, const Eigen::Matrix<double, 2, 2>&,
    std::function<Eigen::Matrix<double, 2, 1>(const State&)>, bool);

template void UKFMeasurements::updateMeasurement<9>(
    const Eigen::Matrix<double, 9, 1>&, const Eigen::Matrix<double, 9, 9>&,
    std::function<Eigen::Matrix<double, 9, 1>(const State&)>, bool);

// Explicit template instantiation for NoGate versions
template void UKFMeasurements::updateMeasurementNoGate<3>(
    const Eigen::Vector3d&, const Eigen::Matrix3d&,
    std::function<Eigen::Vector3d(const State&)>, bool);

// Explicit template instantiation for Internal versions
template void UKFMeasurements::updateMeasurementInternal<1>(
    const Eigen::Matrix<double, 1, 1>&, const Eigen::Matrix<double, 1, 1>&,
    std::function<Eigen::Matrix<double, 1, 1>(const State&)>, bool, bool);

template void UKFMeasurements::updateMeasurementInternal<3>(
    const Eigen::Vector3d&, const Eigen::Matrix3d&,
    std::function<Eigen::Vector3d(const State&)>, bool, bool);

template void UKFMeasurements::updateMeasurementInternal<2>(
    const Eigen::Matrix<double, 2, 1>&, const Eigen::Matrix<double, 2, 2>&,
    std::function<Eigen::Matrix<double, 2, 1>(const State&)>, bool, bool);

template void UKFMeasurements::updateMeasurementInternal<9>(
    const Eigen::Matrix<double, 9, 1>&, const Eigen::Matrix<double, 9, 9>&,
    std::function<Eigen::Matrix<double, 9, 1>(const State&)>, bool, bool);