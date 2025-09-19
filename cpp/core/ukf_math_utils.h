#pragma once
#include <Eigen/Dense>
#include "types.h"

/**
 * @brief Mathematical utilities for UKF operations
 * 
 * Contains numerical utility functions for quaternion operations,
 * matrix conditioning, and other mathematical operations needed
 * for robust UKF implementation.
 */
class UKFMathUtils {
public:
    // Quaternion operations
    static Eigen::Quaterniond rotationVectorToQuaternion(const Eigen::Vector3d& rot_vec);
    static Eigen::Vector3d quaternionToRotationVector(const Eigen::Quaterniond& q);
    static Eigen::Quaterniond averageQuaternions(const std::vector<Eigen::Quaterniond>& quaternions,
                                                const Eigen::VectorXd& weights,
                                                int max_iterations = 10);
    
    // Matrix conditioning and numerical stability
    template<int Size>
    static void enforcePositiveDefinite(Eigen::Matrix<double, Size, Size>& matrix, 
                                       double min_eigenvalue = 1e-10);
    
    template<int Size>
    static double computeConditionNumber(const Eigen::Matrix<double, Size, Size>& matrix);
    
    template<int Size>
    static bool checkMatrixValidity(const Eigen::Matrix<double, Size, Size>& matrix);
    
    // Robust matrix operations
    template<int Size>
    static Eigen::Matrix<double, Size, Size> 
    robustMatrixSqrt(const Eigen::Matrix<double, Size, Size>& matrix);
    
    template<int Rows, int Cols>
    static Eigen::Matrix<double, Rows, Cols>
    robustPseudoInverse(const Eigen::Matrix<double, Rows, Cols>& matrix,
                       double tolerance = 1e-12);
    
    // Statistical functions
    static double mahalanobisDistance(const Eigen::VectorXd& innovation,
                                    const Eigen::MatrixXd& covariance);
    
    static bool chiSquareTest(double mahalanobis_dist, int dof, double confidence = 0.99);
    
    // Coordinate transformations
    static Eigen::Vector3d ecefToLla(const Eigen::Vector3d& ecef);
    static Eigen::Vector3d llaToEcef(const Eigen::Vector3d& lla);

    // Gravity computation
    static double getNormalGravity(double latitude_rad);  // WGS84 normal gravity

    // Error-state operations (for UKF sigma points)
    static State applyErrorToState(const State& nominal_state, 
                                  const Eigen::Matrix<double, 15, 1>& error_vector);
    static Eigen::Matrix<double, 15, 1> computeErrorVector(const State& state1, 
                                                           const State& state2);
    
private:
    // Constants
    static constexpr double SMALL_ANGLE_THRESHOLD = 1e-10;
    static constexpr double QUATERNION_NORM_TOLERANCE = 1e-8;
    static constexpr double WGS84_A = 6378137.0;           // Semi-major axis
    static constexpr double WGS84_E2 = 6.69437999014e-3;   // First eccentricity squared
};