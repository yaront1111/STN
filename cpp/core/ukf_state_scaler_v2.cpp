#include "ukf_state_scaler_v2.h"
#include <iostream>
#include <iomanip>

UKFStateScalerV2::UKFStateScalerV2() {
    initializeScalingMatrices();
}

void UKFStateScalerV2::initializeScalingMatrices() {
    // Initialize diagonal scaling matrices
    // D = diag(σ_i) and D_inv = diag(1/σ_i)

    D_ = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();
    D_inv_ = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Zero();

    // Position components (indices 0-2)
    for (int i = 0; i < 3; ++i) {
        D_(i, i) = SIGMA_POSITION;
        D_inv_(i, i) = 1.0 / SIGMA_POSITION;
    }

    // Velocity components (indices 3-5)
    for (int i = 3; i < 6; ++i) {
        D_(i, i) = SIGMA_VELOCITY;
        D_inv_(i, i) = 1.0 / SIGMA_VELOCITY;
    }

    // Attitude components (indices 6-8)
    for (int i = 6; i < 9; ++i) {
        D_(i, i) = SIGMA_ATTITUDE;
        D_inv_(i, i) = 1.0 / SIGMA_ATTITUDE;
    }

    // Accelerometer bias components (indices 9-11)
    for (int i = 9; i < 12; ++i) {
        D_(i, i) = SIGMA_ACCEL_BIAS;
        D_inv_(i, i) = 1.0 / SIGMA_ACCEL_BIAS;
    }

    // Gyroscope bias components (indices 12-14)
    for (int i = 12; i < 15; ++i) {
        D_(i, i) = SIGMA_GYRO_BIAS;
        D_inv_(i, i) = 1.0 / SIGMA_GYRO_BIAS;
    }

    std::cout << "=== UKF State Scaler V2 Initialized ===\n";
    std::cout << "Scaling based on expected uncertainties:\n";
    std::cout << "  Position σ: " << SIGMA_POSITION << " m (D_inv = " << 1.0/SIGMA_POSITION << ")\n";
    std::cout << "  Velocity σ: " << SIGMA_VELOCITY << " m/s (D_inv = " << 1.0/SIGMA_VELOCITY << ")\n";
    std::cout << "  Attitude σ: " << SIGMA_ATTITUDE << " rad (D_inv = " << 1.0/SIGMA_ATTITUDE << ")\n";
    std::cout << "  Accel bias σ: " << SIGMA_ACCEL_BIAS << " m/s² (D_inv = " << 1.0/SIGMA_ACCEL_BIAS << ")\n";
    std::cout << "  Gyro bias σ: " << SIGMA_GYRO_BIAS << " rad/s (D_inv = " << 1.0/SIGMA_GYRO_BIAS << ")\n";
    std::cout << "========================================\n\n";
}

Eigen::Matrix<double, UKFStateScalerV2::STATE_DIM, 1>
UKFStateScalerV2::scaleError(const Eigen::Matrix<double, STATE_DIM, 1>& dx_physical) const {
    // Scale error vector from physical to normalized space
    // dx_scaled = D_inv * dx_physical
    return D_inv_ * dx_physical;
}

Eigen::Matrix<double, UKFStateScalerV2::STATE_DIM, 1>
UKFStateScalerV2::unscaleError(const Eigen::Matrix<double, STATE_DIM, 1>& dx_scaled) const {
    // Unscale error vector from normalized to physical space
    // dx_physical = D * dx_scaled
    return D_ * dx_scaled;
}

Eigen::Matrix<double, UKFStateScalerV2::STATE_DIM, UKFStateScalerV2::STATE_DIM>
UKFStateScalerV2::scaleCovariance(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P_physical) const {
    // Scale covariance from physical to normalized space
    // P_scaled = D_inv * P_physical * D_inv'
    return D_inv_ * P_physical * D_inv_.transpose();
}

Eigen::Matrix<double, UKFStateScalerV2::STATE_DIM, UKFStateScalerV2::STATE_DIM>
UKFStateScalerV2::unscaleCovariance(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P_scaled) const {
    // Unscale covariance from normalized to physical space
    // P_physical = D * P_scaled * D'
    return D_ * P_scaled * D_.transpose();
}

Eigen::Matrix<double, UKFStateScalerV2::STATE_DIM, UKFStateScalerV2::STATE_DIM>
UKFStateScalerV2::scaleCholeskyFactor(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& S_physical) const {
    // Scale Cholesky factor from physical to normalized space
    // Since P = S * S' and P_scaled = D_inv * P * D_inv'
    // We have P_scaled = D_inv * S * S' * D_inv'
    //                  = (D_inv * S) * (D_inv * S)'
    // Therefore: S_scaled = D_inv * S
    return D_inv_ * S_physical;
}

Eigen::Matrix<double, UKFStateScalerV2::STATE_DIM, UKFStateScalerV2::STATE_DIM>
UKFStateScalerV2::unscaleCholeskyFactor(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& S_scaled) const {
    // Unscale Cholesky factor from normalized to physical space
    // S_physical = D * S_scaled
    return D_ * S_scaled;
}

Eigen::Matrix<double, UKFStateScalerV2::STATE_DIM, 1>
UKFStateScalerV2::scaleProcessNoise(const Eigen::Matrix<double, STATE_DIM, 1>& q_std_physical) const {
    // Scale process noise standard deviations
    // q_std_scaled = D_inv * q_std_physical
    return D_inv_ * q_std_physical;
}

double UKFStateScalerV2::computeConditionNumber(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& M) const {
    // Compute condition number using eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, STATE_DIM, STATE_DIM>> solver(M);

    if (solver.info() != Eigen::Success) {
        std::cerr << "WARNING: Failed to compute eigenvalues for condition number\n";
        return std::numeric_limits<double>::infinity();
    }

    Eigen::VectorXd eigenvalues = solver.eigenvalues();

    // Find min and max positive eigenvalues
    double min_eig = std::numeric_limits<double>::max();
    double max_eig = 0;

    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) > 1e-15) {  // Only consider positive eigenvalues
            if (eigenvalues(i) < min_eig) min_eig = eigenvalues(i);
            if (eigenvalues(i) > max_eig) max_eig = eigenvalues(i);
        }
    }

    if (min_eig < 1e-15 || max_eig < 1e-15) {
        return std::numeric_limits<double>::infinity();  // Essentially singular
    }

    return max_eig / min_eig;
}

bool UKFStateScalerV2::isWellConditioned(const Eigen::Matrix<double, STATE_DIM, STATE_DIM>& P_scaled) const {
    double cond = computeConditionNumber(P_scaled);

    if (cond > 1e4) {
        std::cout << "WARNING: Scaled covariance poorly conditioned! Condition number: " << cond << "\n";

        // Diagnose which components are problematic
        std::cout << "Diagonal elements of P_scaled:\n";
        for (int i = 0; i < STATE_DIM; ++i) {
            std::cout << "  P_scaled(" << i << "," << i << ") = " << std::scientific
                     << std::setprecision(3) << P_scaled(i,i);

            if (i < 3) std::cout << " (position)";
            else if (i < 6) std::cout << " (velocity)";
            else if (i < 9) std::cout << " (attitude)";
            else if (i < 12) std::cout << " (accel bias)";
            else std::cout << " (gyro bias)";

            if (P_scaled(i,i) < 0.01 || P_scaled(i,i) > 100) {
                std::cout << " [OUT OF RANGE!]";
            }
            std::cout << "\n";
        }

        return false;
    }

    return true;
}