/**
 * Math Utilities Implementation
 * Complete working implementations - no stubs
 */

#include "math_utils.h"
#include <Eigen/QR>
#include <Eigen/Cholesky>
#include <algorithm>
#include <numeric>
#include <map>

namespace NavMath {

// ============================================================================
// SO(3) Operations
// ============================================================================

Quaterniond SO3::expMap(const Vector3d& phi) {
    double angle = phi.norm();

    if (angle < 1e-10) {
        // Small angle approximation
        return Quaterniond(1.0, phi.x() * 0.5, phi.y() * 0.5, phi.z() * 0.5);
    }

    Vector3d axis = phi / angle;
    double half_angle = angle * 0.5;
    double sin_half = std::sin(half_angle);

    return Quaterniond(std::cos(half_angle),
                       axis.x() * sin_half,
                       axis.y() * sin_half,
                       axis.z() * sin_half);
}

Vector3d SO3::logMap(const Quaterniond& q) {
    Quaterniond q_norm = normalizeQuaternion(q);

    double w = q_norm.w();
    Vector3d v(q_norm.x(), q_norm.y(), q_norm.z());
    double v_norm = v.norm();

    if (v_norm < 1e-10) {
        return Vector3d::Zero();
    }

    double angle = 2.0 * std::atan2(v_norm, w);
    return (angle / v_norm) * v;
}

Quaterniond SO3::boxplus(const Quaterniond& q, const Vector3d& delta) {
    Quaterniond dq = expMap(delta);
    return normalizeQuaternion(q * dq);
}

Vector3d SO3::boxminus(const Quaterniond& q1, const Quaterniond& q2) {
    Quaterniond dq = q1 * q2.inverse();
    return logMap(dq);
}

Quaterniond SO3::normalizeQuaternion(const Quaterniond& q) {
    Quaterniond q_norm = q.normalized();

    // Enforce positive scalar convention
    if (q_norm.w() < 0) {
        q_norm.coeffs() *= -1;
    }

    return q_norm;
}

Matrix3d SO3::quaternionToMatrix(const Quaterniond& q) {
    return q.normalized().toRotationMatrix();
}

Quaterniond SO3::matrixToQuaternion(const Matrix3d& R) {
    return normalizeQuaternion(Quaterniond(R));
}

Quaterniond SO3::quaternionDerivative(const Quaterniond& q, const Vector3d& omega) {
    Quaterniond omega_q(0, omega.x(), omega.y(), omega.z());
    Quaterniond q_dot = q * omega_q;
    q_dot.coeffs() *= 0.5;
    return q_dot;
}

Quaterniond SO3::integrateQuaternion(const Quaterniond& q, const Vector3d& omega, double dt) {
    // Use exponential map for integration
    Vector3d delta_theta = omega * dt;
    return boxplus(q, delta_theta);
}

// ============================================================================
// Skew-Symmetric Operations
// ============================================================================

Matrix3d SkewSymmetric::skew(const Vector3d& v) {
    Matrix3d S;
    S << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
         -v.y(), v.x(), 0;
    return S;
}

Vector3d SkewSymmetric::vex(const Matrix3d& S) {
    return Vector3d(S(2,1), S(0,2), S(1,0));
}

Matrix3d SkewSymmetric::crossMatrix(const Vector3d& v) {
    return skew(v);
}

// ============================================================================
// Earth Model
// ============================================================================

Vector3d EarthModel::gravityNED(double latitude, double height) {
    double sin_lat = std::sin(latitude);
    double sin2_lat = sin_lat * sin_lat;

    // WGS84 gravity formula
    double g0 = 9.7803253359 * (1 + 0.001931853 * sin2_lat) /
                std::sqrt(1 - WGS84_E2 * sin2_lat);

    // Height correction (free-air)
    double g = g0 * (1 - 2 * height / WGS84_A);

    return Vector3d(0, 0, g);  // Down component only in NED
}

Vector3d EarthModel::earthRate(double latitude) {
    return Vector3d(EARTH_RATE * std::cos(latitude), 0, -EARTH_RATE * std::sin(latitude));
}

Vector3d EarthModel::transportRate(const Vector3d& velocity, double latitude, double height) {
    double Rm = radiusMeridian(latitude);
    double Rn = radiusPrimeVertical(latitude);

    double omega_en_n = velocity.y() / (Rn + height);  // North component
    double omega_en_e = -velocity.x() / (Rm + height); // East component
    double omega_en_d = -velocity.y() * std::tan(latitude) / (Rn + height); // Down component

    return Vector3d(omega_en_n, omega_en_e, omega_en_d);
}

Vector3d EarthModel::ecefToGeodetic(const Vector3d& ecef) {
    double x = ecef.x();
    double y = ecef.y();
    double z = ecef.z();

    double lon = std::atan2(y, x);

    // Iterative latitude computation
    double p = std::sqrt(x*x + y*y);
    double lat = std::atan2(z, p * (1 - WGS84_E2));

    for (int i = 0; i < 5; ++i) {
        double N = WGS84_A / std::sqrt(1 - WGS84_E2 * std::sin(lat) * std::sin(lat));
        double h = p / std::cos(lat) - N;
        lat = std::atan2(z, p * (1 - WGS84_E2 * N / (N + h)));
    }

    double N = WGS84_A / std::sqrt(1 - WGS84_E2 * std::sin(lat) * std::sin(lat));
    double h = p / std::cos(lat) - N;

    return Vector3d(lat, lon, h);
}

Vector3d EarthModel::geodeticToECEF(double lat, double lon, double height) {
    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double sin_lon = std::sin(lon);
    double cos_lon = std::cos(lon);

    double N = WGS84_A / std::sqrt(1 - WGS84_E2 * sin_lat * sin_lat);

    double x = (N + height) * cos_lat * cos_lon;
    double y = (N + height) * cos_lat * sin_lon;
    double z = (N * (1 - WGS84_E2) + height) * sin_lat;

    return Vector3d(x, y, z);
}

Matrix3d EarthModel::ecefToNedMatrix(double lat, double lon) {
    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double sin_lon = std::sin(lon);
    double cos_lon = std::cos(lon);

    Matrix3d C_ned_ecef;
    C_ned_ecef << -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
                  -sin_lon, cos_lon, 0,
                  -cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat;

    return C_ned_ecef;
}

double EarthModel::radiusMeridian(double latitude) {
    double sin_lat = std::sin(latitude);
    double sin2_lat = sin_lat * sin_lat;
    double denom = std::pow(1 - WGS84_E2 * sin2_lat, 1.5);
    return WGS84_A * (1 - WGS84_E2) / denom;
}

double EarthModel::radiusPrimeVertical(double latitude) {
    double sin_lat = std::sin(latitude);
    double sin2_lat = sin_lat * sin_lat;
    return WGS84_A / std::sqrt(1 - WGS84_E2 * sin2_lat);
}

Vector3d EarthModel::coriolisAcceleration(const Vector3d& velocity, double latitude) {
    Vector3d omega_ie_n = earthRate(latitude);
    return -2.0 * SkewSymmetric::crossMatrix(omega_ie_n) * velocity;
}

// ============================================================================
// Strapdown Mechanization
// ============================================================================

StrapdownMechanization::StrapdownMechanization() {
    reset();
}

void StrapdownMechanization::reset() {
    prev_accel_ = Vector3d::Zero();
    prev_gyro_ = Vector3d::Zero();
    prev_dv_ = Vector3d::Zero();
    prev_dtheta_ = Vector3d::Zero();
}

StrapdownMechanization::CompensatedIMU StrapdownMechanization::compensate(
    const Vector3d& accel, const Vector3d& gyro, double dt) {

    // Compute increments
    Vector3d dv = accel * dt;
    Vector3d dtheta = gyro * dt;

    // Coning compensation (rotation-induced)
    Vector3d coning = coningCompensation(dtheta, prev_dtheta_);

    // Sculling compensation (rotation-acceleration coupling)
    Vector3d sculling = scullingCompensation(dv, dtheta, prev_dv_, prev_dtheta_);

    CompensatedIMU result;
    result.delta_v = dv + sculling;
    result.delta_theta = dtheta + coning;
    result.dt = dt;

    // Update history
    prev_accel_ = accel;
    prev_gyro_ = gyro;
    prev_dv_ = dv;
    prev_dtheta_ = dtheta;

    return result;
}

Vector3d StrapdownMechanization::coningCompensation(
    const Vector3d& dtheta_k, const Vector3d& dtheta_km1) {
    // Savage algorithm for coning compensation
    return dtheta_km1.cross(dtheta_k) / 12.0;
}

Vector3d StrapdownMechanization::scullingCompensation(
    const Vector3d& dv_k, const Vector3d& dtheta_k,
    const Vector3d& dv_km1, const Vector3d& dtheta_km1) {
    // Savage algorithm for sculling compensation
    return (dv_km1.cross(dtheta_k) + dtheta_km1.cross(dv_k)) / 12.0;
}

// ============================================================================
// Matrix Utilities
// ============================================================================

MatrixXd MatrixUtils::choleskyUpdate(const MatrixXd& L, const VectorXd& v, double sigma) {
    int n = L.rows();
    MatrixXd L_new = L;
    VectorXd v_copy = v;

    for (int i = 0; i < n; ++i) {
        double r = std::sqrt(L_new(i,i) * L_new(i,i) + sigma * v_copy(i) * v_copy(i));
        double c = r / L_new(i,i);
        double s = v_copy(i) * sigma / L_new(i,i);

        L_new(i,i) = r;

        for (int j = i+1; j < n; ++j) {
            L_new(j,i) = (L_new(j,i) + s * v_copy(j)) / c;
            v_copy(j) = c * v_copy(j) - s * L_new(j,i);
        }
    }

    return L_new;
}

MatrixXd MatrixUtils::choleskyDowndate(const MatrixXd& L, const VectorXd& v, double sigma) {
    // Similar to update but with sign change
    return choleskyUpdate(L, v, -sigma);
}

void MatrixUtils::qrDecomposition(const MatrixXd& A, MatrixXd& Q, MatrixXd& R) {
    Eigen::HouseholderQR<MatrixXd> qr(A);
    Q = qr.householderQ();
    R = qr.matrixQR().triangularView<Eigen::Upper>();
}

MatrixXd MatrixUtils::makeSymmetric(const MatrixXd& M) {
    return 0.5 * (M + M.transpose());
}

MatrixXd MatrixUtils::makePositiveDefinite(const MatrixXd& M, double epsilon) {
    MatrixXd M_sym = makeSymmetric(M);

    Eigen::SelfAdjointEigenSolver<MatrixXd> solver(M_sym);
    VectorXd eigenvalues = solver.eigenvalues();
    MatrixXd eigenvectors = solver.eigenvectors();

    // Ensure all eigenvalues are positive
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) < epsilon) {
            eigenvalues(i) = epsilon;
        }
    }

    return eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
}

MatrixXd MatrixUtils::matrixSquareRoot(const MatrixXd& M) {
    Eigen::LLT<MatrixXd> llt(M);
    if (llt.info() == Eigen::Success) {
        return llt.matrixL();
    } else {
        // Fallback to eigenvalue decomposition
        Eigen::SelfAdjointEigenSolver<MatrixXd> solver(M);
        VectorXd eigenvalues = solver.eigenvalues();
        MatrixXd eigenvectors = solver.eigenvectors();

        for (int i = 0; i < eigenvalues.size(); ++i) {
            eigenvalues(i) = std::sqrt(std::max(0.0, eigenvalues(i)));
        }

        return eigenvectors * eigenvalues.asDiagonal();
    }
}

MatrixXd MatrixUtils::efficientInverse(const MatrixXd& M) {
    Eigen::LLT<MatrixXd> llt(M);
    if (llt.info() == Eigen::Success) {
        MatrixXd I = MatrixXd::Identity(M.rows(), M.cols());
        return llt.solve(I);
    } else {
        return M.inverse();
    }
}

std::pair<VectorXd, MatrixXd> MatrixUtils::weightedMeanCovariance(
    const std::vector<VectorXd>& samples, const std::vector<double>& weights) {

    int n = samples[0].size();
    VectorXd mean = VectorXd::Zero(n);

    // Compute weighted mean
    double weight_sum = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
        mean += weights[i] * samples[i];
        weight_sum += weights[i];
    }
    mean /= weight_sum;

    // Compute weighted covariance
    MatrixXd cov = MatrixXd::Zero(n, n);
    for (size_t i = 0; i < samples.size(); ++i) {
        VectorXd diff = samples[i] - mean;
        cov += weights[i] * diff * diff.transpose();
    }
    cov /= weight_sum;

    return {mean, cov};
}

// ============================================================================
// Gravity Tensor Operations
// ============================================================================

Eigen::Matrix<double, 5, 1> GravityTensor::tensorToSTF(const Matrix3d& T) {
    // Ensure symmetric and trace-free
    Matrix3d T_stf = makeSTF(T);

    // Extract 5 independent components
    Eigen::Matrix<double, 5, 1> stf;
    stf(0) = T_stf(0,0);  // xx
    stf(1) = T_stf(0,1);  // xy
    stf(2) = T_stf(0,2);  // xz
    stf(3) = T_stf(1,1);  // yy
    stf(4) = T_stf(1,2);  // yz
    // Note: zz is determined by trace-free constraint: zz = -(xx + yy)

    return stf;
}

Matrix3d GravityTensor::stfToTensor(const Eigen::Matrix<double, 5, 1>& stf) {
    Matrix3d T;
    T(0,0) = stf(0);  // xx
    T(0,1) = T(1,0) = stf(1);  // xy
    T(0,2) = T(2,0) = stf(2);  // xz
    T(1,1) = stf(3);  // yy
    T(1,2) = T(2,1) = stf(4);  // yz
    T(2,2) = -(stf(0) + stf(3));  // zz = -(xx + yy) for trace-free

    return T;
}

Matrix3d GravityTensor::rotateTensor(const Matrix3d& T, const Matrix3d& R) {
    return R * T * R.transpose();
}

Matrix3d GravityTensor::makeSTF(const Matrix3d& T) {
    // Make symmetric
    Matrix3d T_sym = 0.5 * (T + T.transpose());

    // Remove trace
    double trace = T_sym.trace();
    Matrix3d T_stf = T_sym - (trace / 3.0) * Matrix3d::Identity();

    return T_stf;
}

// ============================================================================
// Statistics
// ============================================================================

double Statistics::computeNIS(const VectorXd& innovation, const MatrixXd& S) {
    return innovation.transpose() * S.inverse() * innovation;
}

double Statistics::computeNEES(const VectorXd& error, const MatrixXd& P) {
    return error.transpose() * P.inverse() * error;
}

double Statistics::mahalanobisDistance(const VectorXd& x, const VectorXd& mean,
                                      const MatrixXd& cov) {
    VectorXd diff = x - mean;
    return std::sqrt(diff.transpose() * cov.inverse() * diff);
}

double Statistics::chi2Threshold(int dof, double alpha) {
    // Simplified chi-squared critical values (should use proper statistical library)
    // These are approximate values for common DOF at 95% confidence
    static std::map<int, double> chi2_table = {
        {1, 3.841}, {2, 5.991}, {3, 7.815}, {4, 9.488}, {5, 11.070},
        {6, 12.592}, {7, 14.067}, {8, 15.507}, {9, 16.919}, {10, 18.307}
    };

    auto it = chi2_table.find(dof);
    if (it != chi2_table.end()) {
        return it->second;
    }

    // Approximation for larger DOF
    return dof + 2 * std::sqrt(2.0 * dof);
}

double Statistics::effectiveSampleSize(const std::vector<double>& weights) {
    double sum_w = 0;
    double sum_w2 = 0;

    for (double w : weights) {
        sum_w += w;
        sum_w2 += w * w;
    }

    return (sum_w * sum_w) / sum_w2;
}

std::vector<int> Statistics::systematicResampling(const std::vector<double>& weights, int N) {
    std::vector<int> indices(N);
    std::vector<double> cumsum(weights.size());

    // Compute cumulative sum
    std::partial_sum(weights.begin(), weights.end(), cumsum.begin());

    // Systematic resampling
    double step = cumsum.back() / N;
    double u = ((double)rand() / RAND_MAX) * step;  // Random start

    int j = 0;
    for (int i = 0; i < N; ++i) {
        double threshold = u + i * step;
        while (static_cast<size_t>(j) < cumsum.size() && cumsum[j] < threshold) {
            j++;
        }
        indices[i] = std::min(j, (int)weights.size() - 1);
    }

    return indices;
}

// ============================================================================
// Robust Estimation
// ============================================================================

double RobustEstimation::huberWeight(double residual, double threshold) {
    double abs_r = std::abs(residual);
    if (abs_r <= threshold) {
        return 1.0;
    } else {
        return threshold / abs_r;
    }
}

double RobustEstimation::tukeyWeight(double residual, double threshold) {
    double abs_r = std::abs(residual);
    if (abs_r <= threshold) {
        double u = abs_r / threshold;
        return std::pow(1 - u * u, 2);
    } else {
        return 0.0;
    }
}

VectorXd RobustEstimation::applyRobustWeights(const VectorXd& innovation,
                                              const MatrixXd& S, double threshold) {
    VectorXd weighted_innovation = innovation;

    // Compute standardized residuals using Cholesky decomposition
    // S = L*L^T, so S^(-1/2) = L^(-T)
    Eigen::LLT<MatrixXd> llt(S);
    VectorXd std_residuals = llt.matrixL().transpose().solve(innovation);

    // Apply Huber weights
    for (int i = 0; i < std_residuals.size(); ++i) {
        double weight = huberWeight(std_residuals(i), threshold);
        weighted_innovation(i) *= weight;
    }

    return weighted_innovation;
}

} // namespace NavMath