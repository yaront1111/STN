/**
 * COMPREHENSIVE NAVIGATION SYSTEM RUNNER
 *
 * Universal test and production runner with full debugging, logging,
 * and feature control via command-line flags.
 *
 * Usage: ./run_navigation_system [OPTIONS]
 *
 * OPTIONS:
 *   --mode <test|production|simulation>  : Operating mode
 *   --scenario <name>                    : Test scenario to run
 *   --duration <seconds>                 : Simulation duration
 *   --log-level <DEBUG|INFO|WARN|ERROR> : Logging verbosity
 *   --log-dir <path>                     : Directory for log files
 *   --enable-gravity                     : Enable gravity measurements
 *   --enable-terrain                     : Enable terrain correlation
 *   --enable-magnetometer                : Enable magnetic heading
 *   --enable-barometer                   : Enable barometric altitude
 *   --debug                              : Enable all debug output
 *   --profile                            : Enable performance profiling
 *   --visualize                          : Enable real-time visualization
 *   --record                             : Record sensor data to file
 *   --playback <file>                    : Playback recorded data
 *   --initial-lat <degrees>              : Initial latitude
 *   --initial-lon <degrees>              : Initial longitude
 *   --initial-alt <meters>               : Initial altitude
 *   --config <file>                      : Configuration file path
 *   --help                               : Show this help message
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <filesystem>
#include <ctime>
#include <map>
#include <vector>
#include <memory>
#include <random>
#include <csignal>

#include "cpp/core/types.h"
#include "cpp/core/ukf_scaled.h"
#include "cpp/core/ukf_config.h"
#include "cpp/core/ukf_math_utils.h"
#include "cpp/core/gravity_gradient_provider.h"
#include "cpp/core/gravity_map_matcher.h"
#include "cpp/core/terrain_correlator.h"
#include "cpp/core/srtm_provider.h"

namespace fs = std::filesystem;

// Global flag for graceful shutdown
volatile bool g_running = true;

void signalHandler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\n[INFO] Received interrupt signal, shutting down gracefully...\n";
        g_running = false;
    }
}

// Logging system
enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARN = 2,
    ERROR = 3
};

class Logger {
private:
    LogLevel level_;
    std::ofstream log_file_;
    std::string log_dir_;
    bool console_output_;
    bool file_output_;
    std::chrono::steady_clock::time_point start_time_;

public:
    Logger(const std::string& log_dir = "logs", LogLevel level = LogLevel::INFO)
        : level_(level), log_dir_(log_dir), console_output_(true), file_output_(true) {

        // Create log directory if it doesn't exist
        fs::create_directories(log_dir_);

        // Generate timestamp for log file name
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << log_dir_ << "/navigation_"
           << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
           << ".log";

        log_file_.open(ss.str());
        start_time_ = std::chrono::steady_clock::now();

        log(LogLevel::INFO, "Logger initialized - File: " + ss.str());
    }

    ~Logger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    void setLevel(LogLevel level) { level_ = level; }
    void enableConsole(bool enable) { console_output_ = enable; }
    void enableFile(bool enable) { file_output_ = enable; }

    void log(LogLevel msg_level, const std::string& message) {
        if (msg_level < level_) return;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - start_time_).count();

        std::stringstream ss;
        ss << "[" << std::fixed << std::setprecision(3) << elapsed << "] ";
        ss << "[" << levelToString(msg_level) << "] ";
        ss << message;

        if (console_output_) {
            if (msg_level == LogLevel::ERROR) {
                std::cerr << ss.str() << std::endl;
            } else {
                std::cout << ss.str() << std::endl;
            }
        }

        if (file_output_ && log_file_.is_open()) {
            log_file_ << ss.str() << std::endl;
            log_file_.flush();
        }
    }

    void debug(const std::string& msg) { log(LogLevel::DEBUG, msg); }
    void info(const std::string& msg) { log(LogLevel::INFO, msg); }
    void warn(const std::string& msg) { log(LogLevel::WARN, msg); }
    void error(const std::string& msg) { log(LogLevel::ERROR, msg); }

private:
    std::string levelToString(LogLevel level) {
        switch(level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO:  return "INFO ";
            case LogLevel::WARN:  return "WARN ";
            case LogLevel::ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }
};

// Performance profiler
class Profiler {
private:
    struct TimingData {
        std::chrono::high_resolution_clock::time_point start;
        double accumulated_ms = 0;
        int count = 0;
    };

    std::map<std::string, TimingData> timings_;
    std::ofstream profile_file_;
    bool enabled_;

public:
    Profiler(bool enabled = false) : enabled_(enabled) {
        if (enabled_) {
            fs::create_directories("logs/profiling");
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << "logs/profiling/profile_"
               << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
               << ".csv";

            profile_file_.open(ss.str());
            profile_file_ << "function,calls,total_ms,avg_ms,max_ms\n";
        }
    }

    ~Profiler() {
        if (profile_file_.is_open()) {
            for (const auto& [name, data] : timings_) {
                double avg = data.count > 0 ? data.accumulated_ms / data.count : 0;
                profile_file_ << name << ","
                             << data.count << ","
                             << data.accumulated_ms << ","
                             << avg << ","
                             << "N/A\n";  // Max not tracked in this simple version
            }
            profile_file_.close();
        }
    }

    void startTimer(const std::string& name) {
        if (!enabled_) return;
        timings_[name].start = std::chrono::high_resolution_clock::now();
    }

    void endTimer(const std::string& name) {
        if (!enabled_) return;
        auto end = std::chrono::high_resolution_clock::now();
        auto& data = timings_[name];
        double elapsed = std::chrono::duration<double, std::milli>(end - data.start).count();
        data.accumulated_ms += elapsed;
        data.count++;
    }

    void printSummary() {
        if (!enabled_) return;
        std::cout << "\n=== PERFORMANCE SUMMARY ===\n";
        for (const auto& [name, data] : timings_) {
            if (data.count > 0) {
                double avg = data.accumulated_ms / data.count;
                std::cout << name << ": "
                         << data.count << " calls, "
                         << "avg " << avg << " ms\n";
            }
        }
    }
};

// Test scenarios
class TestScenario {
public:
    virtual ~TestScenario() = default;
    virtual std::string getName() const = 0;
    virtual void initialize(State& initial_state, Eigen::MatrixXd& initial_cov) = 0;
    virtual void generateIMU(double t, Eigen::Vector3d& acc, Eigen::Vector3d& gyro) = 0;
    virtual Eigen::Vector3d getTruePosition(double t) = 0;
    virtual Eigen::Vector3d getTrueVelocity(double t) = 0;
    virtual Eigen::Quaterniond getTrueAttitude(double t) = 0;
};

class MountainFlightScenario : public TestScenario {
private:
    Eigen::Vector3d initial_lla_;
    std::mt19937 gen_;
    std::normal_distribution<> acc_noise_;
    std::normal_distribution<> gyro_noise_;

    // IMU error model parameters
    Eigen::Vector3d acc_bias_;      // Accelerometer bias (m/s²)
    Eigen::Vector3d gyro_bias_;     // Gyroscope bias (rad/s)
    std::normal_distribution<> acc_bias_walk_;   // Bias random walk
    std::normal_distribution<> gyro_bias_walk_;  // Bias random walk

public:
    MountainFlightScenario()
        : gen_(42),
          acc_noise_(0, 0.005),        // 5 mg noise (tactical grade)
          gyro_noise_(0, 0.0001),      // 0.1 mrad/s noise
          acc_bias_walk_(0, 1e-5),     // 10 μg/√s random walk
          gyro_bias_walk_(0, 1e-7) {   // 0.1 μrad/s/√s random walk

        // Switzerland Alps
        initial_lla_ = Eigen::Vector3d(47.0 * M_PI/180, 8.0 * M_PI/180, 5000);

        // Initialize biases (tactical-grade IMU)
        acc_bias_ = Eigen::Vector3d(0.001, -0.002, 0.0005);   // ~1 mg bias
        gyro_bias_ = Eigen::Vector3d(0.00001, -0.00002, 0.00001);  // ~10 μrad/s bias
    }

    std::string getName() const override { return "Mountain Flight"; }

    void initialize(State& state, Eigen::MatrixXd& cov) override {
        // Convert LLA to ECEF
        double a = 6378137.0;
        double e2 = 0.00669437999014;
        double lat = initial_lla_(0);
        double lon = initial_lla_(1);
        double alt = initial_lla_(2);

        double N = a / std::sqrt(1 - e2 * std::sin(lat) * std::sin(lat));
        state.p_ECEF = Eigen::Vector3d(
            (N + alt) * std::cos(lat) * std::cos(lon),
            (N + alt) * std::cos(lat) * std::sin(lon),
            (N * (1 - e2) + alt) * std::sin(lat)
        );

        state.v_ECEF = Eigen::Vector3d(10, 0, 0); // 10 m/s north (more realistic)
        state.q_ECEF_B = Eigen::Quaterniond::Identity();

        // Initial uncertainty
        cov = Eigen::MatrixXd::Identity(15, 15);
        cov.block<3,3>(0,0) *= 100;   // 10m position
        cov.block<3,3>(3,3) *= 1;     // 1 m/s velocity
        cov.block<3,3>(6,6) *= 0.01;  // 0.1 rad attitude
    }

    void generateIMU(double t, Eigen::Vector3d& acc, Eigen::Vector3d& gyro) override {
        // Update biases with random walk
        double dt = 0.01;  // 100 Hz IMU
        acc_bias_ += Eigen::Vector3d(
            acc_bias_walk_(gen_) * std::sqrt(dt),
            acc_bias_walk_(gen_) * std::sqrt(dt),
            acc_bias_walk_(gen_) * std::sqrt(dt)
        );
        gyro_bias_ += Eigen::Vector3d(
            gyro_bias_walk_(gen_) * std::sqrt(dt),
            gyro_bias_walk_(gen_) * std::sqrt(dt),
            gyro_bias_walk_(gen_) * std::sqrt(dt)
        );

        // Simulated flight dynamics
        double omega = 0.1;  // Turn rate

        // True body accelerations (includes gravity)
        Eigen::Vector3d acc_true = Eigen::Vector3d(
            0.5 * std::sin(omega * t),
            0.3 * std::cos(omega * t),
            9.81 + 0.1 * std::sin(0.05 * t)
        );

        // True body rotation rates
        Eigen::Vector3d gyro_true = Eigen::Vector3d(
            0.01 * std::sin(0.2 * t),
            0.02 * std::cos(0.15 * t),
            omega
        );

        // Add biases and noise
        acc = acc_true + acc_bias_ + Eigen::Vector3d(
            acc_noise_(gen_),
            acc_noise_(gen_),
            acc_noise_(gen_)
        );

        gyro = gyro_true + gyro_bias_ + Eigen::Vector3d(
            gyro_noise_(gen_),
            gyro_noise_(gen_),
            gyro_noise_(gen_)
        );
    }

    Eigen::Vector3d getTruePosition(double t) override {
        // Circular flight pattern starting from initial position
        double radius = 1000;  // 1km radius
        double omega = 0.05;    // rad/s

        // Start at initial position when t=0 (cos(0)=1, sin(0)=0)
        double lat = initial_lla_(0) + (radius * (std::cos(omega * t) - 1)) / 6371000.0;
        double lon = initial_lla_(1) + (radius * std::sin(omega * t)) / 6371000.0;
        double alt = initial_lla_(2) + 100 * std::sin(0.1 * t);

        // Convert to ECEF
        double a = 6378137.0;
        double e2 = 0.00669437999014;
        double N = a / std::sqrt(1 - e2 * std::sin(lat) * std::sin(lat));

        return Eigen::Vector3d(
            (N + alt) * std::cos(lat) * std::cos(lon),
            (N + alt) * std::cos(lat) * std::sin(lon),
            (N * (1 - e2) + alt) * std::sin(lat)
        );
    }

    Eigen::Vector3d getTrueVelocity(double t) override {
        // Numerical derivative of position
        double dt = 0.001;
        Eigen::Vector3d p1 = getTruePosition(t - dt/2);
        Eigen::Vector3d p2 = getTruePosition(t + dt/2);
        return (p2 - p1) / dt;
    }

    Eigen::Quaterniond getTrueAttitude(double t) override {
        // Simple rotation model
        double roll = 0.1 * std::sin(0.2 * t);
        double pitch = 0.05 * std::cos(0.15 * t);
        double yaw = 0.05 * t;

        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

        return yawAngle * pitchAngle * rollAngle;
    }
};

// Data recorder
class DataRecorder {
private:
    std::ofstream data_file_;
    std::ofstream sensor_file_;
    bool recording_;

public:
    DataRecorder(bool enabled = false) : recording_(enabled) {
        if (recording_) {
            fs::create_directories("logs/recordings");
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << "logs/recordings/nav_data_"
               << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");

            data_file_.open(ss.str() + ".csv");
            sensor_file_.open(ss.str() + "_sensors.csv");

            // Write headers
            data_file_ << "time,lat,lon,alt,vn,ve,vd,roll,pitch,yaw,"
                      << "pos_err,vel_err,att_err,cep50,cep95\n";

            sensor_file_ << "time,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,"
                        << "grav_anomaly,gradient_trace,mag_x,mag_y,mag_z,"
                        << "baro_alt,terrain_alt\n";
        }
    }

    ~DataRecorder() {
        if (data_file_.is_open()) data_file_.close();
        if (sensor_file_.is_open()) sensor_file_.close();
    }

    void recordState(double t, const State& state, const Eigen::MatrixXd& cov,
                    double pos_err = 0, double vel_err = 0, double att_err = 0) {
        if (!recording_) return;

        // Convert to geodetic
        Eigen::Vector3d lla = UKFMathUtils::ecefToLla(state.p_ECEF);
        Eigen::Vector3d euler = state.q_ECEF_B.toRotationMatrix().eulerAngles(0, 1, 2);

        // Compute CEP
        double cep50 = 1.1774 * std::sqrt(cov(0,0) + cov(1,1));
        double cep95 = 2.4477 * std::sqrt(cov(0,0) + cov(1,1));

        data_file_ << t << ","
                  << lla(0) * 180/M_PI << "," << lla(1) * 180/M_PI << "," << lla(2) << ","
                  << state.v_ECEF(0) << "," << state.v_ECEF(1) << "," << state.v_ECEF(2) << ","
                  << euler(0) * 180/M_PI << "," << euler(1) * 180/M_PI << "," << euler(2) * 180/M_PI << ","
                  << pos_err << "," << vel_err << "," << att_err << ","
                  << cep50 << "," << cep95 << "\n";
    }

    void recordSensors(double t, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,
                      double grav_anomaly = 0, double gradient_trace = 0,
                      const Eigen::Vector3d& mag = Eigen::Vector3d::Zero(),
                      double baro_alt = 0, double terrain_alt = 0) {
        if (!recording_) return;

        sensor_file_ << t << ","
                    << acc(0) << "," << acc(1) << "," << acc(2) << ","
                    << gyro(0) << "," << gyro(1) << "," << gyro(2) << ","
                    << grav_anomaly << "," << gradient_trace << ","
                    << mag(0) << "," << mag(1) << "," << mag(2) << ","
                    << baro_alt << "," << terrain_alt << "\n";
    }

};

// Command-line parser
struct RunConfig {
    std::string mode = "simulation";
    std::string scenario = "mountain";
    double duration = 300.0;
    LogLevel log_level = LogLevel::INFO;
    std::string log_dir = "logs";
    bool enable_gravity = true;
    bool enable_terrain = true;
    bool enable_magnetometer = true;
    bool enable_barometer = true;
    bool debug = false;
    bool profile = false;
    bool visualize = false;
    bool record = false;
    std::string playback_file = "";
    double initial_lat = 47.0;
    double initial_lon = 8.0;
    double initial_alt = 5000.0;
    std::string config_file = "config/gravity_primary.yaml";

    void parseArgs(int argc, char** argv) {
        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];

            if (arg == "--help" || arg == "-h") {
                printHelp();
                exit(0);
            }
            else if (arg == "--mode" && i + 1 < argc) {
                mode = argv[++i];
            }
            else if (arg == "--scenario" && i + 1 < argc) {
                scenario = argv[++i];
            }
            else if (arg == "--duration" && i + 1 < argc) {
                duration = std::stod(argv[++i]);
            }
            else if (arg == "--log-level" && i + 1 < argc) {
                std::string level = argv[++i];
                if (level == "DEBUG") log_level = LogLevel::DEBUG;
                else if (level == "INFO") log_level = LogLevel::INFO;
                else if (level == "WARN") log_level = LogLevel::WARN;
                else if (level == "ERROR") log_level = LogLevel::ERROR;
            }
            else if (arg == "--log-dir" && i + 1 < argc) {
                log_dir = argv[++i];
            }
            else if (arg == "--enable-gravity") {
                enable_gravity = true;
            }
            else if (arg == "--disable-gravity") {
                enable_gravity = false;
            }
            else if (arg == "--enable-terrain") {
                enable_terrain = true;
            }
            else if (arg == "--disable-terrain") {
                enable_terrain = false;
            }
            else if (arg == "--debug") {
                debug = true;
                log_level = LogLevel::DEBUG;
            }
            else if (arg == "--profile") {
                profile = true;
            }
            else if (arg == "--record") {
                record = true;
            }
            else if (arg == "--initial-lat" && i + 1 < argc) {
                initial_lat = std::stod(argv[++i]);
            }
            else if (arg == "--initial-lon" && i + 1 < argc) {
                initial_lon = std::stod(argv[++i]);
            }
            else if (arg == "--initial-alt" && i + 1 < argc) {
                initial_alt = std::stod(argv[++i]);
            }
        }
    }

    void printHelp() {
        std::cout << R"(
SPACETIME NAVIGATION SYSTEM - Comprehensive Runner
==================================================

Usage: ./run_navigation_system [OPTIONS]

OPTIONS:
  --mode <test|production|simulation>  Operating mode (default: simulation)
  --scenario <name>                    Test scenario (default: mountain)
  --duration <seconds>                 Simulation duration (default: 300)
  --log-level <DEBUG|INFO|WARN|ERROR>  Logging verbosity (default: INFO)
  --log-dir <path>                     Directory for logs (default: logs)
  --enable-gravity                     Enable gravity measurements
  --disable-gravity                    Disable gravity measurements
  --enable-terrain                     Enable terrain correlation
  --disable-terrain                    Disable terrain correlation
  --debug                              Enable all debug output
  --profile                            Enable performance profiling
  --record                             Record data to files
  --initial-lat <degrees>              Initial latitude (default: 47.0)
  --initial-lon <degrees>              Initial longitude (default: 8.0)
  --initial-alt <meters>               Initial altitude (default: 5000)
  --help                               Show this help message

EXAMPLES:
  # Run mountain flight scenario with debug output
  ./run_navigation_system --scenario mountain --debug --duration 600

  # Production mode with custom initial position
  ./run_navigation_system --mode production --initial-lat 37.7749 --initial-lon -122.4194

  # Test with profiling and data recording
  ./run_navigation_system --mode test --profile --record --log-level DEBUG

)";
    }
};

// Main navigation runner
int main(int argc, char** argv) {
    // Parse command-line arguments
    RunConfig config;
    config.parseArgs(argc, argv);

    // Set up signal handler
    std::signal(SIGINT, signalHandler);

    // Initialize logger
    Logger logger(config.log_dir, config.log_level);
    logger.info("=====================================");
    logger.info("  SPACETIME NAVIGATION SYSTEM v2.0");
    logger.info("=====================================");
    logger.info("Mode: " + config.mode);
    logger.info("Duration: " + std::to_string(config.duration) + " seconds");

    // Initialize profiler
    Profiler profiler(config.profile);

    // Initialize data recorder
    DataRecorder recorder(config.record);

    // Create test scenario
    std::unique_ptr<TestScenario> scenario;
    if (config.scenario == "mountain") {
        scenario = std::make_unique<MountainFlightScenario>();
    } else {
        logger.error("Unknown scenario: " + config.scenario);
        return 1;
    }
    logger.info("Scenario: " + scenario->getName());

    // Initialize UKF
    logger.info("Initializing UKF...");
    UKFConfig ukf_config;
    ukf_config.setDefaults();

    // Tune process noise based on realistic sensor characteristics
    // Process noise represents unmodeled dynamics and sensor errors

    // Position process noise: accounts for unmodeled accelerations
    // Typical unmodeled accelerations: 0.01-0.1 m/s^2 over dt
    // Q_pos = (0.5 * a * dt^2)^2 for dt=0.1s => 0.5 * 0.05 * 0.01 = 0.00025 m^2
    ukf_config.process_noise.position = 0.001;  // m^2 per update

    // Velocity process noise: direct acceleration noise
    // Q_vel = (a * dt)^2 for a=0.05 m/s^2, dt=0.1s => 0.0025 m^2/s^2
    ukf_config.process_noise.velocity = 0.005;  // (m/s)^2 per update

    // Attitude process noise: gyro random walk + unmodeled dynamics
    // Typical gyro random walk: 0.01 deg/√hr = 0.00017 rad/√hr = 2.8e-6 rad/√s
    // Q_att = (ARW * √dt)^2 for dt=0.1s => (2.8e-6 * 0.316)^2 = 7.8e-13 rad^2
    // Add unmodeled dynamics: ~0.001 deg/s = 1.7e-5 rad/s
    ukf_config.process_noise.attitude = 1e-6;  // rad^2 per update

    // Accelerometer bias random walk (if modeled)
    // Typical value: 0.001 m/s^2/√hr = 1.67e-5 m/s^2/√s
    ukf_config.process_noise.accel_bias = 1e-8;  // (m/s^2)^2 per update

    // Gyro bias random walk (if modeled)
    // Typical value: 0.1 deg/hr/√hr = 4.8e-7 rad/s/√s
    ukf_config.process_noise.gyro_bias = 1e-12;  // (rad/s)^2 per update

    // Adjust noise based on debug mode
    if (config.debug) {
        ukf_config.verbose = true;
    }

    ScaledUKF ukf(ukf_config);

    // Initialize gravity provider if enabled (must be before UKF init)
    std::unique_ptr<GravityGradientProvider> gravity_provider;
    if (config.enable_gravity) {
        logger.info("Initializing gravity gradient provider...");
        gravity_provider = std::make_unique<GravityGradientProvider>();

        // Initialize with synthetic model for testing
        if (!gravity_provider->initializeSynthetic()) {
            logger.error("Failed to initialize gravity provider");
            return 1;
        }

        // Set gravity provider in UKF
        ukf.setGravityProvider(gravity_provider.get());
        logger.info("Gravity provider set in UKF");
    }

    // Initialize state
    State initial_state;
    Eigen::MatrixXd initial_cov(15, 15);
    scenario->initialize(initial_state, initial_cov);
    ukf.init(initial_state, initial_cov);
    logger.info("UKF initialized");

    // Initialize terrain provider if enabled
    std::unique_ptr<SRTMProvider> terrain_provider;
    if (config.enable_terrain) {
        logger.info("Initializing terrain provider...");
        terrain_provider = std::make_unique<SRTMProvider>();
        // Note: Would load actual SRTM data in production
    }

    // Simulation parameters
    const double dt = 0.01;  // 100 Hz
    const int steps = static_cast<int>(config.duration / dt);
    const int log_interval = static_cast<int>(1.0 / dt);  // Log every second

    // Statistics tracking
    struct Stats {
        double max_pos_error = 0;
        double max_vel_error = 0;
        double sum_pos_error = 0;
        double sum_vel_error = 0;
        double sum_pos_error_sq = 0;  // For RMS calculation
        double sum_vel_error_sq = 0;  // For RMS calculation
        int count = 0;
        int gravity_updates = 0;
        int terrain_updates = 0;
        int mag_updates = 0;
        int baro_updates = 0;
        int zupt_updates = 0;
        int covariance_resets = 0;  // Track filter resets
    } stats;

    logger.info("Starting navigation loop...");
    logger.info("=========================================");

    // Create detailed log files for debugging
    std::ofstream state_log("logs/state_evolution.csv");
    std::ofstream measurement_log("logs/measurement_updates.csv");
    std::ofstream health_log("logs/covariance_health.csv");
    std::ofstream performance_log("logs/performance_metrics.csv");

    // Write headers
    state_log << "time,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,q_w,q_x,q_y,q_z,"
              << "true_pos_x,true_pos_y,true_pos_z,pos_error,vel_error\n";

    measurement_log << "time,sensor,innovation_norm,mahalanobis,threshold,accepted,"
                    << "pre_update_error,post_update_error\n";

    health_log << "time,condition_number,max_eigenvalue,min_eigenvalue,"
               << "position_uncertainty,velocity_uncertainty,attitude_uncertainty,"
               << "divergence_count,filter_healthy\n";

    performance_log << "time,rms_pos,avg_pos,max_pos,rms_vel,drift_rate,"
                    << "gravity_accept_rate,baro_accept_rate,mag_accept_rate\n";

    // Main navigation loop
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int step = 0; step < steps && g_running; step++) {
        double t = step * dt;

        profiler.startTimer("total_step");

        // Generate IMU measurements
        Eigen::Vector3d acc, gyro;
        scenario->generateIMU(t, acc, gyro);

        // Add IMU biases and noise
        std::mt19937 gen(step);
        std::normal_distribution<> imu_noise(0, 0.001);
        acc += Eigen::Vector3d(imu_noise(gen), imu_noise(gen), imu_noise(gen));
        gyro += Eigen::Vector3d(imu_noise(gen), imu_noise(gen), imu_noise(gen));

        // UKF Prediction
        profiler.startTimer("ukf_predict");
        ImuSample imu_sample;
        imu_sample.acc_mps2 = acc;
        imu_sample.gyro_rps = gyro;
        ukf.predict(imu_sample, dt);
        profiler.endTimer("ukf_predict");

        // Get current state
        State current_state = ukf.getState();

        // Get true state for comparison
        Eigen::Vector3d true_pos = scenario->getTruePosition(t);
        Eigen::Vector3d true_vel = scenario->getTrueVelocity(t);
        double pre_update_error = (current_state.p_ECEF - true_pos).norm();

        // Check filter health periodically and reset if unhealthy
        if (step % 100 == 0 && step > 0) {
            double cond = ukf.getCovarianceConditionNumber();
            bool healthy = ukf.isHealthy();

            if (!healthy && cond > 1e6) {
                logger.warn("Filter unhealthy - condition number: " + std::to_string(cond));
                logger.warn("Resetting covariance matrix");
                ukf.resetCovariance();
                stats.covariance_resets++;
            }

            if (config.debug) {
                logger.debug("Filter Health Check:");
                logger.debug("  Condition number: " + std::to_string(cond));
                logger.debug("  Status: " + std::string(healthy ? "Healthy" : "Unhealthy"));
            }

            // Log health metrics
            Eigen::Matrix<double, 15, 15> P = ukf.getCovariance();
            health_log << t << "," << cond << ","
                      << P.eigenvalues().real().maxCoeff() << ","
                      << P.eigenvalues().real().minCoeff() << ","
                      << std::sqrt(P.block<3,3>(0,0).trace()) << ","
                      << std::sqrt(P.block<3,3>(3,3).trace()) << ","
                      << std::sqrt(P.block<3,3>(6,6).trace()) << ","
                      << stats.covariance_resets << ","
                      << (healthy ? 1 : 0) << "\n";
        }

        // Gravity gradient update - optimal frequency based on sensor characteristics
        // Modern gradiometers typically operate at 1-10 Hz
        if (config.enable_gravity && gravity_provider && step % 2 == 0) {  // 50 Hz updates for better tracking
            profiler.startTimer("gravity_update");

            // Get true gravity gradient at current position
            Eigen::Matrix3d gradient_true = gravity_provider->getGradientTensor(
                current_state.p_ECEF, current_state.q_ECEF_B);

            // Add realistic measurement noise (0.1 Eötvös per component)
            // Modern gradiometers achieve 0.01-1 E noise levels
            const double gradient_noise_std = 0.1;  // Eötvös
            std::normal_distribution<> grad_noise(0, gradient_noise_std);

            Eigen::Matrix3d gradient_measured = gradient_true;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    gradient_measured(i, j) += grad_noise(gen);
                }
            }

            // Enforce symmetry (gradiometer constraint)
            gradient_measured = 0.5 * (gradient_measured + gradient_measured.transpose());

            // Use gravity invariants instead of raw tensor for better observability
            // Invariants are rotation-independent, providing more robust position constraints

            // Note: updateGradientInvariants is in UKF class, not ScaledUKF
            // For now, continue using updateGradient but with better parameters
            // TODO: Port invariants update to ScaledUKF

            // Update UKF with appropriate noise covariance
            Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * (gradient_noise_std * gradient_noise_std);
            ukf.updateGradient(gradient_measured, R);
            stats.gravity_updates++;

            profiler.endTimer("gravity_update");

            // Also update with gravity anomaly for better position observability
            // Gravity anomaly provides absolute gravity field reference
            // Note: GravityGradientProvider doesn't have getGravityAnomaly yet
            // This would be a valuable addition for better observability

            if (config.debug) {
                logger.debug("Gravity update: trace = " +
                           std::to_string(gradient_measured.trace()) + " E");
            }
        }

        // Magnetometer update - optimal frequency for heading constraint
        // Magnetometers can operate at high rates but update less frequently to reduce computation
        if (config.enable_magnetometer && step % 100 == 0) {  // 1 Hz updates sufficient for heading
            profiler.startTimer("mag_update");

            // Simulate magnetometer measurement
            Eigen::Vector3d mag_body = Eigen::Vector3d(0, 0, -50e-6);  // 50 μT down
            mag_body = current_state.q_ECEF_B.inverse() * mag_body;

            // Add noise
            std::normal_distribution<> mag_noise(0, 100e-9);
            mag_body += Eigen::Vector3d(mag_noise(gen), mag_noise(gen), mag_noise(gen));

            // Update
            Eigen::Vector3d mag_ref = Eigen::Vector3d(0, 0, -50e-6);
            Eigen::Matrix3d R_mag = Eigen::Matrix3d::Identity() * (100e-9 * 100e-9);  // 100 nT noise
            ukf.updateMagnetometer(mag_body, mag_ref, R_mag);
            stats.mag_updates++;

            profiler.endTimer("mag_update");
        }

        // Gravity Map Matching Update - periodic position fixes
        // Map matching requires sufficient data collection and processing time
        if (config.enable_gravity && gravity_provider && step % 1000 == 0 && step > 0) {  // Every 10 seconds at 100Hz for reliable matching
            profiler.startTimer("map_matching");

            // Simulate map matching algorithm output
            // In reality, this would involve correlating measured gravity with map
            Eigen::Vector3d true_pos = scenario->getTruePosition(t);

            // Add realistic map matching error (10-50m accuracy)
            std::normal_distribution<> map_noise(0, 20.0);  // 20m std dev
            Eigen::Vector3d matched_pos = true_pos;
            matched_pos(0) += map_noise(gen);
            matched_pos(1) += map_noise(gen);
            matched_pos(2) += map_noise(gen) * 0.5;  // Better vertical accuracy

            // Position measurement covariance
            Eigen::Matrix3d R_pos = Eigen::Matrix3d::Identity();
            R_pos(0,0) = 400.0;  // 20m std in X
            R_pos(1,1) = 400.0;  // 20m std in Y
            R_pos(2,2) = 100.0;  // 10m std in Z

            // Apply map matching update
            ukf.updateGravityMapMatching(matched_pos, R_pos);
            stats.gravity_updates++;  // Count as gravity update

            profiler.endTimer("map_matching");

            if (config.debug) {
                Eigen::Vector3d error = matched_pos - current_state.p_ECEF;
                logger.debug("Map matching update: error = " +
                           std::to_string(error.norm()) + " m");
            }
        }

        // Terrain-Relative Navigation (TRN) - radar altimeter + terrain database
        // Provides position fixes in mountainous terrain
        if (config.enable_terrain && step % 200 == 0 && step > 0) {  // Every 2 seconds at 100Hz
            profiler.startTimer("terrain_update");

            // Simulate radar altimeter measurement (height above ground)
            Eigen::Vector3d true_pos = scenario->getTruePosition(t);
            Eigen::Vector3d lla = UKFMathUtils::ecefToLla(true_pos);

            // Simulate terrain height at current position (simplified mountain model)
            double terrain_height = 2000.0 + 500.0 * std::sin(lla(0) * 100) * std::cos(lla(1) * 100);
            double radar_alt = lla(2) - terrain_height;

            // Add radar altimeter noise
            std::normal_distribution<> radar_noise(0, 2.0);  // 2m std dev
            radar_alt += radar_noise(gen);

            // Terrain matching provides position estimate
            // In reality, this would correlate radar altitude profile with terrain database
            std::normal_distribution<> terrain_match_noise(0, 30.0);  // 30m accuracy
            Eigen::Vector3d terrain_pos = true_pos;
            terrain_pos(0) += terrain_match_noise(gen);
            terrain_pos(1) += terrain_match_noise(gen);
            // Vertical is constrained by radar altimeter
            terrain_pos(2) = terrain_height + radar_alt;

            // Apply terrain position fix
            Eigen::Matrix3d R_terrain = Eigen::Matrix3d::Identity();
            R_terrain(0,0) = 900.0;  // 30m std horizontal
            R_terrain(1,1) = 900.0;
            R_terrain(2,2) = 4.0;    // 2m std vertical (radar altimeter)

            ukf.updateGravityMapMatching(terrain_pos, R_terrain);  // Reuse for terrain fix
            stats.terrain_updates++;

            if (config.debug) {
                logger.debug("Terrain update: radar alt = " + std::to_string(radar_alt) + " m");
            }

            profiler.endTimer("terrain_update");
        }

        // Zero Velocity Update (ZUPT) - detect and constrain during stationary periods
        // Check if vehicle is moving slowly (hovering or stationary)
        if (step % 100 == 0) {  // Check every second at 100Hz
            Eigen::Vector3d true_vel = scenario->getTrueVelocity(t);
            double true_speed = true_vel.norm();

            // Detect near-stationary condition (< 0.5 m/s)
            if (true_speed < 0.5) {
                profiler.startTimer("zupt_update");

                // Apply ZUPT constraint
                Eigen::Matrix3d R_zupt = Eigen::Matrix3d::Identity() * 0.01;  // 0.1 m/s uncertainty
                ukf.updateZUPT(R_zupt);
                stats.zupt_updates++;

                if (config.debug) {
                    logger.debug("ZUPT applied at t=" + std::to_string(t) + "s");
                }

                profiler.endTimer("zupt_update");
            }
        }

        // Barometer update - higher frequency for altitude constraint
        // Barometers provide fast, accurate altitude measurements
        if (config.enable_barometer && step % 10 == 0) {  // 10 Hz updates for good altitude tracking
            profiler.startTimer("baro_update");

            // Simulate barometric altitude
            Eigen::Vector3d true_pos = scenario->getTruePosition(t);
            Eigen::Vector3d lla = UKFMathUtils::ecefToLla(true_pos);
            double baro_alt = lla(2);

            // Add noise
            std::normal_distribution<> baro_noise(0, 1.0);
            baro_alt += baro_noise(gen);

            // Apply barometer update
            double baro_noise_var = 1.0;  // 1m std dev altitude noise
            ukf.updateBarometer(baro_alt, baro_noise_var);
            stats.baro_updates++;

            profiler.endTimer("baro_update");
        }

        // Compute errors (already have true_pos and true_vel from above)
        double pos_error = (current_state.p_ECEF - true_pos).norm();
        double vel_error = (current_state.v_ECEF - true_vel).norm();

        // Log state evolution
        if (step % 10 == 0) {  // Log every 0.1 seconds
            state_log << t << ","
                     << current_state.p_ECEF(0) << "," << current_state.p_ECEF(1) << ","
                     << current_state.p_ECEF(2) << ","
                     << current_state.v_ECEF(0) << "," << current_state.v_ECEF(1) << ","
                     << current_state.v_ECEF(2) << ","
                     << current_state.q_ECEF_B.w() << "," << current_state.q_ECEF_B.x() << ","
                     << current_state.q_ECEF_B.y() << "," << current_state.q_ECEF_B.z() << ","
                     << true_pos(0) << "," << true_pos(1) << "," << true_pos(2) << ","
                     << pos_error << "," << vel_error << "\n";
        }

        stats.sum_pos_error += pos_error;
        stats.sum_vel_error += vel_error;
        stats.sum_pos_error_sq += pos_error * pos_error;  // Track squared error for RMS
        stats.sum_vel_error_sq += vel_error * vel_error;  // Track squared error for RMS
        stats.max_pos_error = std::max(stats.max_pos_error, pos_error);
        stats.max_vel_error = std::max(stats.max_vel_error, vel_error);
        stats.count++;

        // Record data
        if (config.record) {
            recorder.recordState(t, current_state, ukf.getCovariance(),
                               pos_error, vel_error, 0);
            recorder.recordSensors(t, acc, gyro);
        }

        // Log status
        if (step % log_interval == 0) {
            double avg_pos = stats.sum_pos_error / stats.count;
            double rms_pos_current = std::sqrt(stats.sum_pos_error_sq / stats.count);
            double rms_vel_current = std::sqrt(stats.sum_vel_error_sq / stats.count);
            double drift_rate = pos_error / (t + 0.001);  // m/s drift

            logger.info("t=" + std::to_string(static_cast<int>(t)) + "s | " +
                       "Pos err: " + std::to_string(static_cast<int>(pos_error)) + "m (avg " +
                       std::to_string(static_cast<int>(avg_pos)) + "m) | " +
                       "Updates: G=" + std::to_string(stats.gravity_updates) +
                       " M=" + std::to_string(stats.mag_updates) +
                       " B=" + std::to_string(stats.baro_updates));

            // Log performance metrics
            performance_log << t << "," << rms_pos_current << "," << avg_pos << ","
                           << stats.max_pos_error << "," << rms_vel_current << ","
                           << drift_rate << ","
                           << 1.0 << "," << 1.0 << "," << 1.0 << "\n";  // TODO: track accept rates
        }

        profiler.endTimer("total_step");
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(end_time - start_time).count();

    // Final statistics
    logger.info("=========================================");
    logger.info("NAVIGATION COMPLETE");
    logger.info("=========================================");
    logger.info("Runtime: " + std::to_string(elapsed) + " seconds");
    logger.info("Real-time factor: " + std::to_string(config.duration / elapsed));

    double rms_pos = std::sqrt(stats.sum_pos_error_sq / stats.count);  // Proper RMS calculation
    double avg_pos = stats.sum_pos_error / stats.count;

    logger.info("");
    logger.info("PERFORMANCE METRICS:");
    logger.info("  Position RMS: " + std::to_string(rms_pos) + " m");
    logger.info("  Position Avg: " + std::to_string(avg_pos) + " m");
    logger.info("  Position Max: " + std::to_string(stats.max_pos_error) + " m");
    logger.info("  Velocity RMS: " +
               std::to_string(std::sqrt(stats.sum_vel_error_sq / stats.count)) + " m/s");  // Proper RMS

    logger.info("");
    logger.info("UPDATE COUNTS:");
    logger.info("  Gravity: " + std::to_string(stats.gravity_updates));
    logger.info("  Magnetometer: " + std::to_string(stats.mag_updates));
    logger.info("  Barometer: " + std::to_string(stats.baro_updates));
    logger.info("  Terrain: " + std::to_string(stats.terrain_updates));

    // Grade assessment
    logger.info("");
    if (rms_pos < 100) {
        logger.info("*** GRADE A: EXCELLENT (<100m RMS) ***");
    } else if (rms_pos < 200) {
        logger.info("*** GRADE B+: VERY GOOD (<200m RMS) ***");
    } else if (rms_pos < 500) {
        logger.info("*** GRADE B: GOOD (<500m RMS) ***");
    } else {
        logger.info("*** GRADE C: NEEDS IMPROVEMENT (>500m RMS) ***");
    }

    // Print profiling summary
    if (config.profile) {
        profiler.printSummary();
    }

    logger.info("");
    logger.info("Log files saved to: " + config.log_dir);

    // Close detailed log files
    state_log.close();
    measurement_log.close();
    health_log.close();
    performance_log.close();

    logger.info("Detailed CSV logs saved: state_evolution.csv, measurement_updates.csv, ");
    logger.info("                         covariance_health.csv, performance_metrics.csv");

    return 0;
}