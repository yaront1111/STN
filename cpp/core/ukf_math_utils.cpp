#include "ukf_math_utils.h"
#include <cmath>
#include <algorithm>
#include <iostream>

Eigen::Quaterniond UKFMathUtils::rotationVectorToQuaternion(const Eigen::Vector3d& rot_vec) {
    double angle = rot_vec.norm();
    
    if (angle < SMALL_ANGLE_THRESHOLD) {
        // Small angle approximation
        Eigen::Quaterniond q;
        q.w() = 1.0;
        q.vec() = 0.5 * rot_vec;
        return q.normalized();
    } else {
        // Standard conversion
        Eigen::Vector3d axis = rot_vec / angle;
        double half_angle = 0.5 * angle;
        Eigen::Quaterniond q;
        q.w() = std::cos(half_angle);
        q.vec() = std::sin(half_angle) * axis;
        return q;
    }
}

Eigen::Vector3d UKFMathUtils::quaternionToRotationVector(const Eigen::Quaterniond& q) {
    Eigen::Quaterniond q_norm = q.normalized();
    
    // Ensure positive scalar part for unique representation
    if (q_norm.w() < 0) {
        q_norm.coeffs() *= -1;
    }
    
    double w = std::abs(q_norm.w());
    if (w >= 1.0 - SMALL_ANGLE_THRESHOLD) {
        // Small angle approximation
        return 2.0 * q_norm.vec();
    } else {
        // Standard conversion
        double angle = 2.0 * std::acos(w);
        double sin_half_angle = std::sqrt(1.0 - w * w);
        return angle * q_norm.vec() / sin_half_angle;
    }
}

Eigen::Quaterniond UKFMathUtils::averageQuaternions(const std::vector<Eigen::Quaterniond>& quaternions,
                                                    const Eigen::VectorXd& weights,
                                                    int max_iterations) {
    if (quaternions.empty()) {
        return Eigen::Quaterniond::Identity();
    }
    
    // Initialize with first quaternion
    Eigen::Quaterniond q_avg = quaternions[0];
    
    for (int iter = 0; iter < max_iterations; ++iter) {
        Eigen::Vector3d error_sum = Eigen::Vector3d::Zero();
        
        for (size_t i = 0; i < quaternions.size(); ++i) {
            Eigen::Quaterniond q_diff = q_avg.inverse() * quaternions[i];
            Eigen::Vector3d error = quaternionToRotationVector(q_diff);
            error_sum += weights(i) * error;
        }
        
        if (error_sum.norm() < SMALL_ANGLE_THRESHOLD) {
            break;
        }
        
        // Update average quaternion
        Eigen::Quaterniond q_error = rotationVectorToQuaternion(error_sum);
        q_avg = q_avg * q_error;
        q_avg.normalize();
    }
    
    return q_avg;
}

double UKFMathUtils::mahalanobisDistance(const Eigen::VectorXd& innovation,
                                        const Eigen::MatrixXd& covariance) {
    // DEBUG: Check inputs for validity
    if (!innovation.allFinite()) {
        std::cout << "ERROR: Innovation contains NaN/Inf in mahalanobisDistance\\n";
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    if (!covariance.allFinite()) {
        std::cout << "ERROR: Covariance contains NaN/Inf in mahalanobisDistance\\n";
        return std::numeric_limits<double>::quiet_NaN();
    }
    
    // Compute Mahalanobis distance: sqrt(innovation^T * S^-1 * innovation)
    // Use robust solver instead of pseudoinverse
    Eigen::LLT<Eigen::MatrixXd> llt(covariance);
    if (llt.info() == Eigen::Success) {
        Eigen::VectorXd solved = llt.solve(innovation);
        double distance_squared = innovation.dot(solved);
        
        if (!std::isfinite(distance_squared) || distance_squared < 0) {
            std::cout << "ERROR: Invalid distance_squared=" << distance_squared << " in mahalanobisDistance\\n";
            return std::numeric_limits<double>::quiet_NaN();
        }
        
        return std::sqrt(distance_squared);
    } else {
        // Fallback: use SVD with regularization
        std::cout << "WARNING: LLT failed, using SVD fallback in mahalanobisDistance\\n";
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
        
        // Regularize small singular values
        Eigen::VectorXd singular_values = svd.singularValues();
        double min_sv = 1e-12;
        for (int i = 0; i < singular_values.size(); ++i) {
            if (singular_values(i) < min_sv) singular_values(i) = min_sv;
        }
        
        // Solve with regularized singular values
        Eigen::VectorXd solved = svd.matrixV() * (singular_values.cwiseInverse().asDiagonal() * (svd.matrixU().transpose() * innovation));
        double distance_squared = innovation.dot(solved);
        
        if (!std::isfinite(distance_squared) || distance_squared < 0) {
            std::cout << "ERROR: Invalid distance_squared=" << distance_squared << " in SVD fallback\\n";
            return std::numeric_limits<double>::quiet_NaN();
        }
        
        return std::sqrt(distance_squared);
    }
}

bool UKFMathUtils::chiSquareTest(double mahalanobis_dist, int dof, double confidence) {
    // Simplified chi-square test thresholds
    double threshold;
    switch (dof) {
        case 1: threshold = (confidence > 0.99) ? 6.63 : 3.84; break;
        case 2: threshold = (confidence > 0.99) ? 9.21 : 5.99; break;
        case 3: threshold = (confidence > 0.99) ? 11.34 : 7.81; break;
        case 6: threshold = (confidence > 0.99) ? 16.81 : 12.59; break;
        case 9: threshold = (confidence > 0.99) ? 21.67 : 16.92; break;
        default: threshold = dof + 3.0 * std::sqrt(2.0 * dof); break;
    }
    
    return mahalanobis_dist * mahalanobis_dist < threshold;
}

Eigen::Vector3d UKFMathUtils::ecefToLla(const Eigen::Vector3d& ecef) {
    double x = ecef(0);
    double y = ecef(1);
    double z = ecef(2);
    
    double lon = std::atan2(y, x);
    double p = std::sqrt(x*x + y*y);
    double lat = std::atan2(z, p * (1.0 - WGS84_E2));
    
    // Iterative solution for latitude
    for (int i = 0; i < 5; ++i) {
        double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat) * std::sin(lat));
        double h = p / std::cos(lat) - N;
        lat = std::atan2(z, p * (1.0 - WGS84_E2 * N / (N + h)));
    }
    
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat) * std::sin(lat));
    double alt = p / std::cos(lat) - N;
    
    return Eigen::Vector3d(lat, lon, alt);
}

Eigen::Vector3d UKFMathUtils::llaToEcef(const Eigen::Vector3d& lla) {
    double lat = lla(0);
    double lon = lla(1);
    double alt = lla(2);
    
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat) * std::sin(lat));
    
    double x = (N + alt) * std::cos(lat) * std::cos(lon);
    double y = (N + alt) * std::cos(lat) * std::sin(lon);
    double z = (N * (1.0 - WGS84_E2) + alt) * std::sin(lat);
    
    return Eigen::Vector3d(x, y, z);
}

// Template specializations
template<>
void UKFMathUtils::enforcePositiveDefinite<15>(Eigen::Matrix<double, 15, 15>& matrix, double min_eigenvalue) {
    // Step 1: Enforce symmetry
    matrix = 0.5 * (matrix + matrix.transpose());

    // Step 2: Eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(matrix);

    if (es.info() != Eigen::Success) {
        std::cerr << "WARNING: Eigenvalue decomposition failed, resetting to diagonal matrix!\n";
        // Reset to safe diagonal matrix
        matrix = Eigen::Matrix<double, 15, 15>::Identity() * min_eigenvalue;
        matrix.block<3,3>(0,0) *= 10000.0;  // Position: ~10cm
        matrix.block<3,3>(3,3) *= 100.0;    // Velocity: ~0.1m/s
        matrix.block<3,3>(6,6) *= 0.01;     // Attitude: ~0.01rad
        matrix.block<3,3>(9,9) *= 0.0001;   // Accel bias
        matrix.block<3,3>(12,12) *= 0.00001; // Gyro bias
        return;
    }

    // Check condition number
    double max_eigenvalue = es.eigenvalues().maxCoeff();
    double min_eig = es.eigenvalues().minCoeff();

    if (max_eigenvalue > 1e10 || min_eig < min_eigenvalue || !std::isfinite(max_eigenvalue)) {
        // Regularize eigenvalues
        Eigen::Matrix<double, 15, 1> eigenvalues = es.eigenvalues();

        // Clamp eigenvalues to reasonable range
        for (int i = 0; i < 15; ++i) {
            if (!std::isfinite(eigenvalues(i)) || eigenvalues(i) < min_eigenvalue) {
                eigenvalues(i) = min_eigenvalue;
            } else if (eigenvalues(i) > 1e10) {
                eigenvalues(i) = 1e10;
            }
        }

        // Reconstruct matrix with regularized eigenvalues
        matrix = es.eigenvectors() * eigenvalues.asDiagonal() * es.eigenvectors().transpose();

        // Enforce symmetry again
        matrix = 0.5 * (matrix + matrix.transpose());

        // Add small regularization to diagonal for numerical stability
        matrix += Eigen::Matrix<double, 15, 15>::Identity() * (min_eigenvalue * 1e-3);
    }
}

template<>
bool UKFMathUtils::checkMatrixValidity<15>(const Eigen::Matrix<double, 15, 15>& matrix) {
    // Check for NaN or Inf
    if (!matrix.allFinite()) {
        std::cerr << "WARNING: Matrix contains NaN or Inf values!\n";
        return false;
    }

    // Check symmetry
    double symmetry_error = (matrix - matrix.transpose()).norm();
    if (symmetry_error > 1e-6) {
        std::cerr << "WARNING: Matrix is not symmetric (error=" << symmetry_error << ")\n";
        return false;
    }

    // Use eigenvalue decomposition for more robust check
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(matrix);

    if (es.info() != Eigen::Success) {
        std::cerr << "WARNING: Eigenvalue decomposition failed in validity check!\n";
        return false;
    }

    double min_eigenvalue = es.eigenvalues().minCoeff();
    double max_eigenvalue = es.eigenvalues().maxCoeff();

    // Check for negative eigenvalues (not positive definite)
    if (min_eigenvalue < 1e-12) {
        std::cerr << "WARNING: Matrix has negative/tiny eigenvalues (min=" << min_eigenvalue << ")\n";
        return false;
    }

    // Check condition number
    double cond = max_eigenvalue / min_eigenvalue;
    if (cond > 1e10) {
        std::cerr << "WARNING: Matrix is poorly conditioned (cond=" << cond << ")\n";
        return false;
    }

    return true;
}

template<>
Eigen::Matrix<double, 15, 15> UKFMathUtils::robustMatrixSqrt<15>(const Eigen::Matrix<double, 15, 15>& matrix) {
    // First enforce symmetry
    Eigen::Matrix<double, 15, 15> symmetric = 0.5 * (matrix + matrix.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 15, 15>> es(symmetric);

    if (es.info() != Eigen::Success) {
        std::cerr << "WARNING: Eigenvalue decomposition failed in matrix sqrt, using identity!\n";
        // Return scaled identity as fallback
        return Eigen::Matrix<double, 15, 15>::Identity() * std::sqrt(std::abs(matrix.trace() / 15.0));
    }

    // Robustly handle negative eigenvalues
    Eigen::Matrix<double, 15, 1> eigenvalues = es.eigenvalues();
    double min_eigenvalue = 1e-12;

    // Check for negative eigenvalues
    if (eigenvalues.minCoeff() < min_eigenvalue) {
        std::cerr << "WARNING: Negative eigenvalues detected, clamping to " << min_eigenvalue << "\n";
    }

    // Clamp eigenvalues and take square root
    Eigen::Matrix<double, 15, 1> sqrt_eigenvalues = eigenvalues.array().max(min_eigenvalue).sqrt().matrix();

    // Reconstruct the matrix square root
    Eigen::Matrix<double, 15, 15> result = es.eigenvectors() * sqrt_eigenvalues.asDiagonal() * es.eigenvectors().transpose();

    // Ensure the result is symmetric
    return 0.5 * (result + result.transpose());
}

// Error-state operations
State UKFMathUtils::applyErrorToState(const State& nominal_state, 
                                     const Eigen::Matrix<double, 15, 1>& error_vector) {
    State perturbed_state = nominal_state;
    
    // Apply position error (indices 0-2)
    perturbed_state.p_ECEF += error_vector.segment<3>(0);
    
    // Apply velocity error (indices 3-5)
    perturbed_state.v_ECEF += error_vector.segment<3>(3);
    
    // Apply attitude error (indices 6-8) - this is a rotation vector
    Eigen::Vector3d delta_theta = error_vector.segment<3>(6);
    Eigen::Quaterniond delta_q = rotationVectorToQuaternion(delta_theta);
    perturbed_state.q_ECEF_B = perturbed_state.q_ECEF_B * delta_q;
    perturbed_state.q_ECEF_B.normalize();
    
    // Apply accelerometer bias error (indices 9-11)
    perturbed_state.b_a += error_vector.segment<3>(9);
    
    // Apply gyroscope bias error (indices 12-14)
    perturbed_state.b_g += error_vector.segment<3>(12);
    
    return perturbed_state;
}

Eigen::Matrix<double, 15, 1> UKFMathUtils::computeErrorVector(const State& state1, 
                                                             const State& state2) {
    Eigen::Matrix<double, 15, 1> error;
    
    // Position error (indices 0-2)
    error.segment<3>(0) = state1.p_ECEF - state2.p_ECEF;
    
    // Velocity error (indices 3-5)
    error.segment<3>(3) = state1.v_ECEF - state2.v_ECEF;
    
    // Attitude error (indices 6-8) - quaternion difference as rotation vector
    Eigen::Quaterniond delta_q = state2.q_ECEF_B.inverse() * state1.q_ECEF_B;
    error.segment<3>(6) = quaternionToRotationVector(delta_q);
    
    // Accelerometer bias error (indices 9-11)
    error.segment<3>(9) = state1.b_a - state2.b_a;
    
    // Gyroscope bias error (indices 12-14)  
    error.segment<3>(12) = state1.b_g - state2.b_g;
    
    return error;
}

// Template specializations for dynamic matrices are not needed since we use direct solvers
template <int Size>
double UKFMathUtils::computeConditionNumber(const Eigen::Matrix<double, Size, Size>& matrix) {
    Eigen::JacobiSVD<Eigen::Matrix<double, Size, Size>> svd(matrix);
    double max_singular = svd.singularValues().maxCoeff();
    double min_singular = svd.singularValues().minCoeff();
    if (min_singular < 1e-15) return 1e15;
    return max_singular / min_singular;
}
// Explicit instantiation for condition number
template double UKFMathUtils::computeConditionNumber<15>(
    const Eigen::Matrix<double, 15, 15>& matrix);
