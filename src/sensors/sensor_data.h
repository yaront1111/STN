/**
 * Sensor Data Structures
 * Common data types for all sensors
 */

#pragma once

#include <Eigen/Dense>
#include "../utils/math_utils.h"

namespace Navigation {

using namespace NavMath;

/**
 * IMU Data Structure
 */
struct IMUData {
    double timestamp;
    Vector3d accel;  // [m/s^2] in body frame
    Vector3d gyro;   // [rad/s] in body frame
    double temperature;  // [°C]
    bool valid;

    // Additional quality flags for compatibility
    bool accel_saturated = false;
    bool gyro_saturated = false;
};

/**
 * Barometer Data Structure
 */
struct BarometerData {
    double timestamp;
    double pressure;  // [Pa]
    double altitude;  // [m] (if available)
    double temperature;  // [°C]
    double pressure_variance = 1.0;  // [Pa^2]
    bool valid;
};

/**
 * Magnetometer Data Structure
 */
struct MagnetometerData {
    double timestamp;
    Vector3d field;  // [uT] magnetic field in body frame
    double temperature;  // [°C]
    double total_field = 0.0;  // [uT] magnitude of field
    double declination = 0.0;  // [rad] magnetic declination
    double inclination = 0.0;  // [rad] magnetic inclination
    bool is_disturbed = false;  // Magnetic disturbance flag
    bool valid;
};

/**
 * Gradiometer Data Structure
 */
struct GradiometerData {
    double timestamp;
    // Gravity gradient tensor (symmetric, trace-free)
    // Stored as [Txx, Tyy, Tzz, Txy, Txz, Tyz]
    Eigen::Matrix<double, 6, 1> tensor;  // [E] (1E = 10^-9 s^-2)
    Matrix3d gradient_tensor;  // Full 3x3 tensor representation
    double temperature;  // [°C]
    double confidence = 1.0;  // Measurement confidence [0,1]
    double trace = 0.0;  // Should be zero for valid measurements
    double determinant = 0.0;  // Tensor determinant
    double frobenius_norm = 0.0;  // Tensor Frobenius norm
    bool is_saturated = false;  // Saturation flag
    bool valid;

    // Helper to get full 3x3 tensor
    Matrix3d getTensor3x3() const {
        Matrix3d T;
        T(0, 0) = tensor(0);
        T(1, 1) = tensor(1);
        T(2, 2) = tensor(2);
        T(0, 1) = T(1, 0) = tensor(3);
        T(0, 2) = T(2, 0) = tensor(4);
        T(1, 2) = T(2, 1) = tensor(5);
        return T;
    }
};

/**
 * Combined sensor packet
 */
struct SensorPacket {
    double timestamp;
    double dt = 0.01;  // Time since last packet [s]
    IMUData imu;
    BarometerData barometer;
    MagnetometerData magnetometer;
    GradiometerData gradiometer;
    bool has_imu = false;
    bool has_baro = false;
    bool has_mag = false;
    bool has_grad = false;
};

} // namespace Navigation