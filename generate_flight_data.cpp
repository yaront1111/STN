/**
 * Synthetic Flight Data Generator
 * Generates realistic sensor data for a 30-minute GPS-denied flight
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <filesystem>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace fs = std::filesystem;

// Flight parameters
constexpr double FLIGHT_DURATION_S = 1800.0;  // 30 minutes
constexpr double IMU_RATE_HZ = 100.0;
constexpr double BARO_RATE_HZ = 10.0;
constexpr double MAG_RATE_HZ = 10.0;
constexpr double GRAD_RATE_HZ = 1.0;

// Physical constants
constexpr double GRAVITY = 9.80665;  // m/s^2
constexpr double MAG_FIELD_STRENGTH = 50.0;  // microTesla
constexpr double MAG_INCLINATION = 60.0 * M_PI / 180.0;  // degrees to radians
constexpr double MAG_DECLINATION = 15.0 * M_PI / 180.0;

// Initial conditions
constexpr double INIT_LAT = 37.0;  // degrees
constexpr double INIT_LON = -119.0;  // degrees
constexpr double INIT_ALT = 3000.0;  // meters
constexpr double CRUISE_SPEED = 100.0;  // m/s

struct FlightState {
    double time;
    Eigen::Vector3d position;  // NED frame (m)
    Eigen::Vector3d velocity;  // NED frame (m/s)
    Eigen::Quaterniond attitude;  // Body to NED
    Eigen::Vector3d angular_velocity;  // rad/s
    double latitude, longitude, altitude;
};

class FlightDataGenerator {
public:
    FlightDataGenerator() : rng_(42),
                           accel_noise_(0.0, 0.05),
                           gyro_noise_(0.0, 0.001),
                           baro_noise_(0.0, 1.0),
                           mag_noise_(0.0, 0.5),
                           grad_noise_(0.0, 5.0) {}

    bool generateData(const std::string& output_path) {
        std::cout << "=== Flight Data Generator ===" << std::endl;
        std::cout << "Generating 30-minute flight sensor data..." << std::endl;

        // Create output directory
        fs::create_directories(output_path);

        // Generate flight trajectory
        std::vector<FlightState> trajectory = generateTrajectory();

        // Generate sensor data
        if (!generateIMUData(trajectory, output_path + "/imu.csv")) return false;
        if (!generateBarometerData(trajectory, output_path + "/barometer.csv")) return false;
        if (!generateMagnetometerData(trajectory, output_path + "/magnetometer.csv")) return false;
        if (!generateGradiometerData(trajectory, output_path + "/gradiometer.csv")) return false;
        if (!generateTruthData(trajectory, output_path + "/truth.csv")) return false;

        std::cout << "✓ Flight data generation complete!" << std::endl;
        std::cout << "  Duration: " << FLIGHT_DURATION_S << " seconds" << std::endl;
        std::cout << "  IMU samples: " << static_cast<int>(FLIGHT_DURATION_S * IMU_RATE_HZ) << std::endl;
        std::cout << "  Distance traveled: " << CRUISE_SPEED * FLIGHT_DURATION_S / 1000.0 << " km" << std::endl;

        return true;
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<double> accel_noise_;
    std::normal_distribution<double> gyro_noise_;
    std::normal_distribution<double> baro_noise_;
    std::normal_distribution<double> mag_noise_;
    std::normal_distribution<double> grad_noise_;

    std::vector<FlightState> generateTrajectory() {
        std::vector<FlightState> trajectory;
        double dt = 1.0 / IMU_RATE_HZ;
        int num_samples = static_cast<int>(FLIGHT_DURATION_S * IMU_RATE_HZ);

        FlightState state;
        state.time = 0.0;
        state.position = Eigen::Vector3d::Zero();
        state.velocity = Eigen::Vector3d(CRUISE_SPEED, 0, 0);  // North
        state.attitude = Eigen::Quaterniond::Identity();
        state.angular_velocity = Eigen::Vector3d::Zero();
        state.latitude = INIT_LAT;
        state.longitude = INIT_LON;
        state.altitude = INIT_ALT;

        for (int i = 0; i < num_samples; ++i) {
            // Save current state
            trajectory.push_back(state);

            // Update state
            state.time += dt;

            // Flight pattern: Figure-8 with altitude changes
            double phase = 2 * M_PI * state.time / 600.0;  // 10-minute period

            // Horizontal figure-8
            double turn_rate = 0.1 * std::sin(phase);
            state.angular_velocity = Eigen::Vector3d(
                0.01 * std::sin(phase * 2),  // Roll oscillation
                0.005 * std::sin(phase * 3),  // Pitch oscillation
                turn_rate  // Yaw for figure-8
            );

            // Update attitude
            Eigen::Quaterniond dq;
            Eigen::Vector3d axis = state.angular_velocity * dt;
            double angle = axis.norm();
            if (angle > 1e-6) {
                axis.normalize();
                dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
            } else {
                dq = Eigen::Quaterniond::Identity();
            }
            state.attitude = state.attitude * dq;
            state.attitude.normalize();

            // Update velocity (maintain cruise speed, change direction)
            Eigen::Matrix3d R = state.attitude.toRotationMatrix();
            Eigen::Vector3d body_velocity(CRUISE_SPEED, 0, 0);
            state.velocity = R * body_velocity;

            // Add altitude variation
            state.velocity.z() = 10.0 * std::sin(phase * 0.5);

            // Update position
            state.position += state.velocity * dt;

            // Update geodetic coordinates (simplified)
            double R_earth = 6371000.0;  // Earth radius in meters
            state.latitude = INIT_LAT + (state.position.x() / R_earth) * 180.0 / M_PI;
            state.longitude = INIT_LON + (state.position.y() / (R_earth * std::cos(INIT_LAT * M_PI / 180.0))) * 180.0 / M_PI;
            state.altitude = INIT_ALT - state.position.z();  // NED frame
        }

        return trajectory;
    }

    bool generateIMUData(const std::vector<FlightState>& trajectory, const std::string& filename) {
        std::cout << "  Generating IMU data..." << std::endl;
        std::ofstream file(filename);
        if (!file) {
            std::cerr << "Failed to create " << filename << std::endl;
            return false;
        }

        // Header
        file << "timestamp,ax,ay,az,gx,gy,gz,temp\n";

        // IMU biases (constant + random walk)
        Eigen::Vector3d accel_bias(0.01, -0.02, 0.05);
        Eigen::Vector3d gyro_bias(0.001, -0.0005, 0.002);
        double temperature = 25.0;  // Celsius

        for (size_t i = 1; i < trajectory.size(); ++i) {
            const FlightState& state = trajectory[i];
            const FlightState& prev_state = trajectory[i - 1];
            double dt = state.time - prev_state.time;

            // Calculate acceleration (including gravity)
            Eigen::Vector3d accel_ned = (state.velocity - prev_state.velocity) / dt;
            accel_ned.z() -= GRAVITY;  // Remove gravity in NED

            // Transform to body frame
            Eigen::Matrix3d R = state.attitude.toRotationMatrix();
            Eigen::Vector3d accel_body = R.transpose() * accel_ned;

            // Add centrifugal acceleration
            Eigen::Vector3d centrifugal = state.angular_velocity.cross(state.velocity);
            accel_body += R.transpose() * centrifugal;

            // Add IMU errors
            accel_body += accel_bias;
            for (int j = 0; j < 3; ++j) {
                accel_body(j) += accel_noise_(rng_);
            }

            // Gyro measurements
            Eigen::Vector3d gyro = state.angular_velocity + gyro_bias;
            for (int j = 0; j < 3; ++j) {
                gyro(j) += gyro_noise_(rng_);
            }

            // Random walk on biases
            accel_bias += Eigen::Vector3d(accel_noise_(rng_) * 0.0001,
                                         accel_noise_(rng_) * 0.0001,
                                         accel_noise_(rng_) * 0.0001);
            gyro_bias += Eigen::Vector3d(gyro_noise_(rng_) * 0.00001,
                                        gyro_noise_(rng_) * 0.00001,
                                        gyro_noise_(rng_) * 0.00001);

            // Temperature variation
            temperature = 25.0 + 5.0 * std::sin(state.time * 0.001) + 0.1 * accel_noise_(rng_);

            // Write data
            file << std::fixed << std::setprecision(6)
                 << state.time << ","
                 << accel_body.x() << "," << accel_body.y() << "," << accel_body.z() << ","
                 << gyro.x() << "," << gyro.y() << "," << gyro.z() << ","
                 << temperature << "\n";
        }

        return true;
    }

    bool generateBarometerData(const std::vector<FlightState>& trajectory, const std::string& filename) {
        std::cout << "  Generating barometer data..." << std::endl;
        std::ofstream file(filename);
        if (!file) {
            std::cerr << "Failed to create " << filename << std::endl;
            return false;
        }

        // Header
        file << "timestamp,pressure,temperature\n";

        int baro_skip = static_cast<int>(IMU_RATE_HZ / BARO_RATE_HZ);

        for (size_t i = 0; i < trajectory.size(); i += baro_skip) {
            const FlightState& state = trajectory[i];

            // Barometric pressure model
            double P0 = 101325.0;  // Sea level pressure (Pa)
            double T0 = 288.15;    // Sea level temperature (K)
            double L = 0.0065;     // Lapse rate (K/m)
            double pressure = P0 * std::pow(1.0 - L * state.altitude / T0, 5.255);

            // Add noise and bias
            pressure += 10.0 + baro_noise_(rng_);

            // Temperature
            double temperature = T0 - L * state.altitude - 273.15;  // Convert to Celsius
            temperature += 0.5 * baro_noise_(rng_);

            // Write data
            file << std::fixed << std::setprecision(6)
                 << state.time << ","
                 << pressure << ","
                 << temperature << "\n";
        }

        return true;
    }

    bool generateMagnetometerData(const std::vector<FlightState>& trajectory, const std::string& filename) {
        std::cout << "  Generating magnetometer data..." << std::endl;
        std::ofstream file(filename);
        if (!file) {
            std::cerr << "Failed to create " << filename << std::endl;
            return false;
        }

        // Header
        file << "timestamp,mx,my,mz\n";

        int mag_skip = static_cast<int>(IMU_RATE_HZ / MAG_RATE_HZ);

        // Earth magnetic field in NED frame
        Eigen::Vector3d mag_ned(
            MAG_FIELD_STRENGTH * std::cos(MAG_INCLINATION) * std::cos(MAG_DECLINATION),
            MAG_FIELD_STRENGTH * std::cos(MAG_INCLINATION) * std::sin(MAG_DECLINATION),
            MAG_FIELD_STRENGTH * std::sin(MAG_INCLINATION)
        );

        // Hard iron bias
        Eigen::Vector3d hard_iron(2.0, -1.5, 0.5);

        for (size_t i = 0; i < trajectory.size(); i += mag_skip) {
            const FlightState& state = trajectory[i];

            // Transform to body frame
            Eigen::Matrix3d R = state.attitude.toRotationMatrix();
            Eigen::Vector3d mag_body = R.transpose() * mag_ned;

            // Add disturbances
            mag_body += hard_iron;

            // Add noise
            for (int j = 0; j < 3; ++j) {
                mag_body(j) += mag_noise_(rng_);
            }

            // Write data
            file << std::fixed << std::setprecision(6)
                 << state.time << ","
                 << mag_body.x() << "," << mag_body.y() << "," << mag_body.z() << "\n";
        }

        return true;
    }

    bool generateGradiometerData(const std::vector<FlightState>& trajectory, const std::string& filename) {
        std::cout << "  Generating gradiometer data..." << std::endl;
        std::ofstream file(filename);
        if (!file) {
            std::cerr << "Failed to create " << filename << std::endl;
            return false;
        }

        // Header (5 independent STF components)
        file << "timestamp,gxx,gxy,gxz,gyy,gyz\n";

        int grad_skip = static_cast<int>(IMU_RATE_HZ / GRAD_RATE_HZ);

        for (size_t i = 0; i < trajectory.size(); i += grad_skip) {
            const FlightState& state = trajectory[i];

            // Generate gravity gradient based on position
            // Values in Eotvos (1E = 1e-9 s^-2)
            double base_gradient = 10.0;

            // STF components with spatial variation
            double gxx = base_gradient * (1.0 + 0.3 * std::sin(state.position.x() * 0.00001));
            double gxy = base_gradient * 0.1 * std::cos(state.position.y() * 0.00001);
            double gxz = base_gradient * 0.5 * std::sin(state.position.x() * 0.00002);
            double gyy = base_gradient * (1.0 - 0.3 * std::sin(state.position.x() * 0.00001));
            double gyz = base_gradient * 0.5 * std::cos(state.position.y() * 0.00002);

            // Add noise
            gxx += grad_noise_(rng_);
            gxy += grad_noise_(rng_) * 0.5;
            gxz += grad_noise_(rng_) * 0.5;
            gyy += grad_noise_(rng_);
            gyz += grad_noise_(rng_) * 0.5;

            // Write data
            file << std::fixed << std::setprecision(6)
                 << state.time << ","
                 << gxx << "," << gxy << "," << gxz << ","
                 << gyy << "," << gyz << "\n";
        }

        return true;
    }

    bool generateTruthData(const std::vector<FlightState>& trajectory, const std::string& filename) {
        std::cout << "  Generating ground truth data..." << std::endl;
        std::ofstream file(filename);
        if (!file) {
            std::cerr << "Failed to create " << filename << std::endl;
            return false;
        }

        // Header
        file << "timestamp,lat,lon,alt,vn,ve,vd,roll,pitch,yaw\n";

        int truth_skip = static_cast<int>(IMU_RATE_HZ / 10.0);  // 10 Hz truth data

        for (size_t i = 0; i < trajectory.size(); i += truth_skip) {
            const FlightState& state = trajectory[i];

            // Convert quaternion to Euler angles
            Eigen::Vector3d euler = state.attitude.toRotationMatrix().eulerAngles(0, 1, 2);

            // Write data
            file << std::fixed << std::setprecision(8)
                 << state.time << ","
                 << state.latitude << "," << state.longitude << "," << state.altitude << ","
                 << std::setprecision(4)
                 << state.velocity.x() << "," << state.velocity.y() << "," << state.velocity.z() << ","
                 << euler.x() << "," << euler.y() << "," << euler.z() << "\n";
        }

        return true;
    }
};

int main(int argc, char* argv[]) {
    std::string output_path = "data/flight";

    if (argc > 1) {
        output_path = argv[1];
    }

    FlightDataGenerator generator;

    if (!generator.generateData(output_path)) {
        std::cerr << "Failed to generate flight data" << std::endl;
        return 1;
    }

    std::cout << "\nData saved to: " << output_path << "/" << std::endl;
    std::cout << "Ready for navigation system testing!" << std::endl;

    return 0;
}