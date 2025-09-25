/**
 * Comprehensive Logging System
 * Logs every computation, state change, and measurement
 * Real-time file output with no data loss
 */

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <mutex>
#include <memory>
#include <chrono>
#include <iomanip>
#include <unordered_map>
#include <Eigen/Dense>

namespace Navigation {

// Logging macros for easy use
#define LOG_DEBUG(msg) Logger::getInstance().log(LogLevel::DEBUG, __FILE__, __LINE__, msg)
#define LOG_INFO(msg) Logger::getInstance().log(LogLevel::INFO, __FILE__, __LINE__, msg)
#define LOG_WARN(msg) Logger::getInstance().log(LogLevel::WARN, __FILE__, __LINE__, msg)
#define LOG_ERROR(msg) Logger::getInstance().log(LogLevel::ERROR, __FILE__, __LINE__, msg)

// Navigation state for logging
struct NavigationState {
    double timestamp;
    Eigen::Vector3d position;      // NED frame
    Eigen::Vector3d velocity;      // NED frame
    Eigen::Quaterniond quaternion; // Body to NED
    Eigen::Vector3d accel_bias;
    Eigen::Vector3d gyro_bias;
    Eigen::Matrix<double, 5, 1> gravity_bias;  // STF tensor components

    // Additional state info
    double position_uncertainty;  // Trace of position covariance
    double velocity_uncertainty;
    double attitude_uncertainty;
};

// Sensor data for logging
struct LoggedSensorData {
    double timestamp;
    double dt;  // Time since last measurement

    // IMU
    struct {
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
        double temperature;
    } imu;

    // Barometer
    struct {
        double pressure;
        double temperature;
        double altitude;
    } barometer;
    bool has_baro;

    // Magnetometer
    struct {
        Eigen::Vector3d field;
        double declination;
        double inclination;
    } magnetometer;
    bool has_mag;

    // Gradiometer
    struct {
        Eigen::Matrix<double, 5, 1> gradient_tensor;  // STF components
        double confidence;
    } gradiometer;
    bool has_gravity;
};

// Reset event for logging
struct ResetEvent {
    double timestamp;
    Eigen::Vector3d old_position;
    Eigen::Vector3d new_position;
    double position_jump;
    double confidence;
    double pre_reset_nis;
    double post_reset_nis;
    std::string reset_type;  // "full", "partial", "position_only"
};

enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARN = 2,
    ERROR = 3
};

class Logger {
private:
    // Singleton pattern
    Logger() = default;
    ~Logger() { finalize(); }
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // Configuration
    std::string log_directory_;
    LogLevel min_level_ = LogLevel::INFO;
    bool initialized_ = false;

    // File streams for different log types
    std::ofstream system_log_;         // General system logs
    std::ofstream state_log_;          // Navigation state evolution
    std::ofstream measurement_log_;    // All sensor measurements
    std::ofstream covariance_log_;     // Covariance matrix evolution
    std::ofstream residual_log_;       // Innovation/residuals
    std::ofstream ml_prediction_log_;  // ML bias predictions
    std::ofstream reset_log_;          // Reset events
    std::ofstream performance_log_;    // Timing and performance
    std::ofstream debug_log_;          // Detailed debug information

    // Thread safety
    std::mutex log_mutex_;

    // Timing
    std::chrono::high_resolution_clock::time_point start_time_;

    // Statistics
    uint64_t total_logs_ = 0;
    uint64_t state_logs_ = 0;
    uint64_t measurement_logs_ = 0;
    uint64_t reset_count_ = 0;

public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void initialize(const std::string& log_dir, const std::string& log_level);
    void finalize();

    // General logging
    template<typename T>
    void log(LogLevel level, const std::string& file, int line, const T& message) {
        if (!initialized_ || level < min_level_) return;

        std::lock_guard<std::mutex> lock(log_mutex_);

        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - start_time_
        ).count() / 1000.0;

        std::stringstream ss;
        ss << std::fixed << std::setprecision(6) << elapsed << " ";
        ss << "[" << levelToString(level) << "] ";
        ss << file << ":" << line << " - ";
        ss << message;

        // Write to appropriate log file
        std::ofstream* target = nullptr;
        switch (level) {
            case LogLevel::DEBUG:
                target = &debug_log_;
                break;
            case LogLevel::ERROR:
            case LogLevel::WARN:
                target = &system_log_;
                // Also write to stderr for errors
                if (level == LogLevel::ERROR) {
                    std::cerr << ss.str() << std::endl;
                }
                break;
            default:
                target = &system_log_;
        }

        if (target && target->is_open()) {
            *target << ss.str() << std::endl;
            target->flush();  // Ensure data is written immediately
        }

        // Always write to console for all log levels
        std::cout << ss.str() << std::endl;

        total_logs_++;
    }

    // Specialized logging functions
    void logState(const NavigationState& state);

    // Overload for StateVector (from UKF)
    template<typename StateType>
    void logState(const StateType& state) {
        NavigationState nav_state;
        nav_state.timestamp = state.timestamp;
        nav_state.position = state.position;
        nav_state.velocity = state.velocity;
        nav_state.quaternion = state.quaternion;
        nav_state.accel_bias = state.accel_bias;
        nav_state.gyro_bias = state.gyro_bias;
        // Check if state has gravity_bias member (compile-time check not available in C++17)
        // Just assume StateVector has gravity_bias
        nav_state.gravity_bias = state.gravity_bias;
        logState(nav_state);
    }

    void logMeasurements(const LoggedSensorData& data);
    void logCovariance(const Eigen::MatrixXd& P);
    void logResiduals(const std::string& sensor, const Eigen::VectorXd& residual,
                     const Eigen::MatrixXd& S);
    void logMLPrediction(double timestamp, const Eigen::VectorXd& bias,
                        const Eigen::MatrixXd& uncertainty, double confidence);
    void logReset(const ResetEvent& reset);

    // Overload for different reset structure from navigation_system
    template<typename ResetType>
    void logReset(const ResetType& reset) {
        ResetEvent event;
        event.timestamp = 0;  // Will be set internally
        event.position_jump = reset.position_jump;
        event.confidence = reset.confidence;
        event.reset_type = "navigation";
        logReset(event);
    }
    void logPerformance(uint64_t iteration, double ukf_ms, double rbpf_ms,
                       double ml_ms, double total_ms);

    // Innovation monitoring
    void logInnovation(const std::string& sensor, double timestamp,
                       const Eigen::VectorXd& innovation,
                       const Eigen::VectorXd& innovation_cov,
                       double nis);

    // NEES/NIS logging for consistency checks
    void logNEES(double timestamp, double nees, int dof);
    void logNIS(const std::string& sensor, double timestamp, double nis, int dof);

    // Particle filter specific
    void logParticleDistribution(double timestamp,
                                const std::vector<Eigen::Vector3d>& particles,
                                const std::vector<double>& weights,
                                double ess);

    // Debug helpers
    void logMatrix(const std::string& name, const Eigen::MatrixXd& matrix);
    void logVector(const std::string& name, const Eigen::VectorXd& vector);

    // Get statistics
    uint64_t getTotalLogs() const { return total_logs_; }
    uint64_t getStateLogs() const { return state_logs_; }
    uint64_t getResetCount() const { return reset_count_; }

private:
    std::string levelToString(LogLevel level);
    std::string getTimestamp();
    void writeCSVHeader(std::ofstream& file, const std::vector<std::string>& headers);

    // Helper to format Eigen matrices/vectors for CSV
    std::string eigenToCSV(const Eigen::VectorXd& vec);
    std::string eigenToCSV(const Eigen::MatrixXd& mat);
    std::string quaternionToEuler(const Eigen::Quaterniond& q);
};

} // namespace Navigation

// Implementation will be in logger.cpp