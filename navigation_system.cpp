/**
 * GPS-Free Navigation System - Production Main Runner
 * Advanced hierarchical navigation for GPS-denied environments
 * Version 2.0.0 - Full production implementation
 */

#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <sstream>
#include <getopt.h>
#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

#include "src/core/hierarchical_filter.h"
#include "src/sensors/sensor_manager.h"
#include "src/maps/composite_map_manager.h"
#include "src/ml/onnx_predictor.h"
#include "src/utils/logger.h"
#include "src/utils/data_validator.h"
#include "src/utils/performance_monitor.h"
#include "src/utils/trajectory_analyzer.h"

using namespace std::chrono;
using namespace Navigation;

// Version information
constexpr const char* VERSION = "2.0.0";
constexpr const char* BUILD_DATE = __DATE__;
constexpr const char* BUILD_TIME = __TIME__;

// Global flags
std::atomic<bool> g_running{true};
std::atomic<bool> g_verbose{false};
std::atomic<bool> g_headless{false};

// Command-line options
struct CommandLineOptions {
    std::string config_file;
    std::string log_level = "INFO";
    std::string log_dir = "logs/";
    std::string output_format = "csv";
    int max_iterations = -1;  // -1 means unlimited
    bool dry_run = false;
    bool show_version = false;
    bool show_help = false;
    bool enable_profiling = false;
    bool headless = false;
    bool validate_only = false;
};

void signalHandler(int signum) {
    std::stringstream msg;
    msg << "Received signal " << signum << ", shutting down gracefully...";
    if (!g_headless) {
        std::cout << "\n" << msg.str() << std::endl;
    }
    LOG_INFO(msg.str());
    g_running = false;
}

void printVersion() {
    std::cout << "GPS-Free Navigation System" << std::endl;
    std::cout << "Version: " << VERSION << std::endl;
    std::cout << "Build Date: " << BUILD_DATE << " " << BUILD_TIME << std::endl;
    std::cout << "Features:" << std::endl;
    std::cout << "  - Square-Root UKF on SO(3) manifold" << std::endl;
    std::cout << "  - Rao-Blackwellized Particle Filter" << std::endl;
    std::cout << "  - XGM2019e Gravity Model" << std::endl;
    std::cout << "  - SRTM Terrain Matching" << std::endl;
    std::cout << "  - Multi-rate Sensor Fusion (100Hz)" << std::endl;
    std::cout << "Target Accuracy: <50m after 30 minutes without GPS" << std::endl;
}

void printHelp(const char* program_name) {
    std::cout << "GPS-Free Navigation System - Production Runner" << std::endl;
    std::cout << "\nUsage: " << program_name << " [OPTIONS] <config.yaml>" << std::endl;
    std::cout << "\nOptions:" << std::endl;
    std::cout << "  -h, --help              Show this help message" << std::endl;
    std::cout << "  -v, --version           Show version information" << std::endl;
    std::cout << "  -c, --config <file>     Configuration file (required)" << std::endl;
    std::cout << "  -l, --log-level <level> Set log level (DEBUG|INFO|WARN|ERROR)" << std::endl;
    std::cout << "  -d, --log-dir <dir>     Set log directory (default: logs/)" << std::endl;
    std::cout << "  -n, --dry-run           Validate configuration without running" << std::endl;
    std::cout << "  -p, --profile           Enable detailed performance profiling" << std::endl;
    std::cout << "  -m, --max-iter <n>      Maximum iterations to process" << std::endl;
    std::cout << "  -f, --format <fmt>      Output format (csv|json)" << std::endl;
    std::cout << "  -q, --headless          Run without console output" << std::endl;
    std::cout << "  --validate              Validate configuration and data files only" << std::endl;
    std::cout << "\nExamples:" << std::endl;
    std::cout << "  " << program_name << " -c config.yaml" << std::endl;
    std::cout << "  " << program_name << " -c config.yaml -l DEBUG -p" << std::endl;
    std::cout << "  " << program_name << " -c config.yaml --dry-run" << std::endl;
    std::cout << "  " << program_name << " -c config.yaml -m 6000 --profile" << std::endl;
}

CommandLineOptions parseCommandLine(int argc, char* argv[]) {
    CommandLineOptions options;

    static struct option long_options[] = {
        {"help",      no_argument,       0, 'h'},
        {"version",   no_argument,       0, 'v'},
        {"config",    required_argument, 0, 'c'},
        {"log-level", required_argument, 0, 'l'},
        {"log-dir",   required_argument, 0, 'd'},
        {"dry-run",   no_argument,       0, 'n'},
        {"profile",   no_argument,       0, 'p'},
        {"max-iter",  required_argument, 0, 'm'},
        {"format",    required_argument, 0, 'f'},
        {"headless",  no_argument,       0, 'q'},
        {"validate",  no_argument,       0, 0},
        {0, 0, 0, 0}
    };

    int option_index = 0;
    int c;

    while ((c = getopt_long(argc, argv, "hvc:l:d:npm:f:q", long_options, &option_index)) != -1) {
        switch (c) {
            case 'h':
                options.show_help = true;
                return options;
            case 'v':
                options.show_version = true;
                return options;
            case 'c':
                options.config_file = optarg;
                break;
            case 'l':
                options.log_level = optarg;
                break;
            case 'd':
                options.log_dir = optarg;
                break;
            case 'n':
                options.dry_run = true;
                break;
            case 'p':
                options.enable_profiling = true;
                break;
            case 'm':
                options.max_iterations = std::stoi(optarg);
                break;
            case 'f':
                options.output_format = optarg;
                break;
            case 'q':
                options.headless = true;
                g_headless = true;
                break;
            case 0:
                if (std::string(long_options[option_index].name) == "validate") {
                    options.validate_only = true;
                }
                break;
            default:
                std::cerr << "Unknown option. Use -h for help." << std::endl;
                exit(1);
        }
    }

    // If no config file specified via -c, check for positional argument
    if (options.config_file.empty() && optind < argc) {
        options.config_file = argv[optind];
    }

    return options;
}

class NavigationSystem {
private:
    // Configuration
    YAML::Node config_;
    CommandLineOptions options_;

    // Core components (no mocks - all real implementations)
    std::unique_ptr<HierarchicalFilter> filter_;
    std::unique_ptr<SensorManager> sensors_;
    std::shared_ptr<CompositeMapManager> maps_;
    std::unique_ptr<ONNXPredictor> ml_predictor_;
    std::unique_ptr<DataValidator> validator_;
    std::unique_ptr<PerformanceMonitor> perf_monitor_;
    std::unique_ptr<TrajectoryAnalyzer> analyzer_;

    // Timing
    high_resolution_clock::time_point system_start_time_;
    high_resolution_clock::time_point last_ukf_update_;
    high_resolution_clock::time_point last_rbpf_update_;
    high_resolution_clock::time_point last_ml_update_;
    high_resolution_clock::time_point last_status_update_;

    // System state
    Navigation::StateVector current_state_;
    bool system_initialized_{false};
    uint64_t iteration_count_{0};

    // Performance tracking
    double ukf_time_ms_{0};
    double rbpf_time_ms_{0};
    double ml_time_ms_{0};
    double total_time_ms_{0};

    // System health metrics
    struct SystemHealth {
        bool sensors_healthy = true;
        bool filter_healthy = true;
        bool maps_loaded = true;
        double cpu_usage_percent = 0.0;
        double memory_usage_mb = 0.0;
        int missed_deadlines = 0;
        double max_position_error = 0.0;
    } health_;

public:
    NavigationSystem(const CommandLineOptions& opts) : options_(opts) {
        if (!g_headless) {
            std::cout << "=== GPS-Free Navigation System v" << VERSION << " ===" << std::endl;
            std::cout << "Initializing with config: " << options_.config_file << std::endl;
        }
        LOG_INFO("=== GPS-Free Navigation System Starting ===");
        {
            std::stringstream msg;
            msg << "Version: " << VERSION << ", Build: " << BUILD_DATE << " " << BUILD_TIME;
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "Loading configuration from: " << options_.config_file;
            LOG_INFO(msg.str());
        }

        // Load configuration
        try {
            config_ = YAML::LoadFile(options_.config_file);
        } catch (const YAML::Exception& e) {
            std::stringstream msg;
            msg << "Failed to load config file: " << e.what();
            LOG_ERROR(msg.str());
            throw std::runtime_error(msg.str());
        }

        // Override config with command-line options
        if (!options_.log_level.empty()) {
            config_["system"]["log_level"] = options_.log_level;
        }
        if (!options_.log_dir.empty()) {
            config_["system"]["log_path"] = options_.log_dir;
        }

        // Initialize logging with real output files
        Logger::getInstance().initialize(
            config_["system"]["log_path"].as<std::string>(),
            config_["system"]["log_level"].as<std::string>()
        );

        LOG_INFO("Configuration loaded successfully");
        {
            std::stringstream msg;
            msg << "System mode: " << config_["system"]["mode"].as<std::string>();
            LOG_DEBUG(msg.str());
        }

        // Enable profiling if requested
        if (options_.enable_profiling) {
            config_["performance"]["profile"] = true;
            LOG_INFO("Performance profiling enabled");
        }

        // Verify we're in real data mode (no simulation allowed)
        if (config_["system"]["mode"].as<std::string>() != "real_data") {
            LOG_ERROR("System must run in real_data mode. No simulations allowed.");
            throw std::runtime_error("Invalid mode - must be real_data");
        }
    }

    bool validateConfiguration() {
        if (!g_headless) {
            std::cout << "Validating configuration..." << std::endl;
        }
        LOG_INFO("Validating configuration...");

        bool valid = true;
        std::vector<std::string> errors;

        // Check required sections
        if (!config_["system"]) {
            errors.push_back("Missing 'system' section");
            valid = false;
        }
        if (!config_["ukf"]) {
            errors.push_back("Missing 'ukf' section");
            valid = false;
        }
        if (!config_["rbpf"]) {
            errors.push_back("Missing 'rbpf' section");
            valid = false;
        }
        if (!config_["sensors"]) {
            errors.push_back("Missing 'sensors' section");
            valid = false;
        }
        if (!config_["maps"]) {
            errors.push_back("Missing 'maps' section");
            valid = false;
        }

        // Validate sensor data files exist
        if (config_["sensors"]["imu"]["csv_path"]) {
            std::string path = config_["sensors"]["imu"]["csv_path"].as<std::string>();
            std::ifstream file(path);
            if (!file.good()) {
                errors.push_back("IMU data file not found: " + path);
                valid = false;
            }
        }

        // Report validation results
        if (!valid) {
            if (!g_headless) {
                std::cerr << "Configuration validation failed:" << std::endl;
                for (const auto& error : errors) {
                    std::cerr << "  - " << error << std::endl;
                }
            }
            for (const auto& error : errors) {
                LOG_ERROR(error);
            }
        } else {
            if (!g_headless) {
                std::cout << "Configuration validation successful!" << std::endl;
            }
            LOG_INFO("Configuration validation successful");
        }

        return valid;
    }

    void initialize() {
        LOG_INFO("Initializing navigation system components...");
        auto init_start = high_resolution_clock::now();

        // 1. Initialize data validator
        LOG_INFO("Initializing data validator...");
        validator_ = std::make_unique<DataValidator>(config_["validation"]);

        // 2. Initialize sensor manager with real data files
        LOG_INFO("Initializing sensor manager with real data...");
        sensors_ = std::make_unique<SensorManager>(config_["sensors"]);

        // Initialize sensors (opens files)
        if (!sensors_->initialize()) {
            LOG_ERROR("Failed to initialize sensors");
            throw std::runtime_error("Failed to initialize sensors");
        }

        // Validate sensor data files exist and are readable
        if (!sensors_->validateDataFiles()) {
            LOG_ERROR("Sensor data files validation failed");
            throw std::runtime_error("Invalid sensor data files");
        }

        // 3. Load real map data
        LOG_INFO("Loading map data (XGM2019e gravity, SRTM terrain)...");
        maps_ = std::make_shared<CompositeMapManager>(config_["maps"]);

        // Initialize maps first
        if (!maps_->initialize()) {
            LOG_ERROR("Map initialization failed");
            throw std::runtime_error("Failed to initialize maps");
        }

        // Then verify map data integrity
        if (!maps_->validateMaps()) {
            LOG_ERROR("Map data validation failed");
            throw std::runtime_error("Invalid map data");
        }

        // 4. Initialize ML predictor with trained model
        if (config_["ml"]["enable"].as<bool>()) {
            LOG_INFO("Loading ML bias predictor model...");
            ml_predictor_ = std::make_unique<ONNXPredictor>(
                config_["ml"]["model"].as<std::string>()
            );

            if (!ml_predictor_->loadModel()) {
                LOG_WARN("Failed to load ML model - continuing without ML bias prediction");
                ml_predictor_.reset();  // Clear the predictor
            }
        }

        // 5. Initialize hierarchical filter (UKF + RBPF)
        LOG_INFO("Initializing hierarchical filter...");
        filter_ = std::make_unique<HierarchicalFilter>(
            config_["ukf"],
            config_["rbpf"],
            maps_
        );

        // 6. Initialize performance monitor
        perf_monitor_ = std::make_unique<PerformanceMonitor>(
            config_["performance"]["budget_ms"].as<double>()
        );

        // 7. Initialize trajectory analyzer for real-time analysis
        if (config_["debug"]["enable_analysis"].as<bool>()) {
            LOG_INFO("Initializing trajectory analyzer...");
            analyzer_ = std::make_unique<TrajectoryAnalyzer>(
                config_["debug"]["truth_file"].as<std::string>()
            );
        }

        // Set initial state from configuration or first sensor reading
        initializeState();

        auto init_duration = duration_cast<milliseconds>(
            high_resolution_clock::now() - init_start
        );
        {
            std::stringstream msg;
            msg << "System initialization complete in " << init_duration.count() << " ms";
            LOG_INFO(msg.str());
        }

        system_initialized_ = true;
        system_start_time_ = high_resolution_clock::now();
    }

    void initializeState() {
        LOG_INFO("Initializing navigation state...");

        // Get initial position from config or first GPS reading (if available for init only)
        Eigen::Vector3d initial_position;
        if (config_["initial_state"]["position"]) {
            auto pos = config_["initial_state"]["position"].as<std::vector<double>>();
            initial_position << pos[0], pos[1], pos[2];
            {
                std::stringstream msg;
                msg << "Using configured initial position: " << initial_position.transpose();
                LOG_INFO(msg.str());
            }
        } else {
            // Try to get from first sensor reading
            initial_position = sensors_->getInitialPosition();
            {
                std::stringstream msg;
                msg << "Using sensor initial position: " << initial_position.transpose();
                LOG_INFO(msg.str());
            }
        }

        // Initialize filter state
        current_state_.position = initial_position;

        // Get initial velocity from config
        if (config_["initial_state"]["velocity"]) {
            auto vel = config_["initial_state"]["velocity"].as<std::vector<double>>();
            current_state_.velocity << vel[0], vel[1], vel[2];
            {
                std::stringstream msg;
                msg << "Using configured initial velocity: " << current_state_.velocity.transpose();
                LOG_INFO(msg.str());
            }
        } else {
            current_state_.velocity = Eigen::Vector3d::Zero();
            LOG_INFO("Using zero initial velocity");
        }

        current_state_.quaternion = Eigen::Quaterniond::Identity();
        current_state_.accel_bias = Eigen::Vector3d::Zero();
        current_state_.gyro_bias = Eigen::Vector3d::Zero();
        current_state_.timestamp = sensors_->getFirstTimestamp();

        filter_->initialize(current_state_);

        // Log initial state
        Logger::getInstance().logState(current_state_);
        {
            std::stringstream msg;
            msg << "Initial state set, starting navigation at time: " << current_state_.timestamp;
            LOG_INFO(msg.str());
        }
    }

    void run() {
        if (!system_initialized_) {
            LOG_ERROR("System not initialized");
            throw std::runtime_error("System not initialized");
        }

        if (!g_headless) {
            std::cout << "\n=== Navigation System Running ===" << std::endl;
            std::cout << "Target rate: 100 Hz (10 ms per iteration)" << std::endl;
            std::cout << "Press Ctrl+C to stop gracefully\n" << std::endl;
        }

        LOG_INFO("Starting main navigation loop...");
        LOG_INFO("Target rate: 100 Hz (10 ms per iteration)");

        last_ukf_update_ = high_resolution_clock::now();
        last_rbpf_update_ = high_resolution_clock::now();
        last_ml_update_ = high_resolution_clock::now();
        last_status_update_ = high_resolution_clock::now();

        while (g_running && sensors_->hasMoreData()) {
            auto iteration_start = high_resolution_clock::now();
            iteration_count_++;

            // Check max iterations limit
            if (options_.max_iterations > 0 && static_cast<int>(iteration_count_) > options_.max_iterations) {
                if (!g_headless) {
                    std::cout << "Reached maximum iterations limit (" << options_.max_iterations << ")" << std::endl;
                }
                LOG_INFO("Reached maximum iterations limit");
                break;
            }

            try {
                // Process one iteration
                processIteration();

                // Check timing budget
                auto iteration_duration = duration_cast<microseconds>(
                    high_resolution_clock::now() - iteration_start
                );

                double iteration_ms = iteration_duration.count() / 1000.0;
                perf_monitor_->recordIteration(iteration_ms);

                if (iteration_ms > 10.0) {
                    health_.missed_deadlines++;
                    if (options_.enable_profiling) {
                        std::stringstream msg;
                        msg << "Iteration " << iteration_count_ << " exceeded budget: "
                            << iteration_ms << " ms";
                        LOG_WARN(msg.str());
                    }
                }

                // Print status updates every second (100 iterations)
                if (!g_headless && iteration_count_ % 100 == 0) {
                    printStatusUpdate();
                }

                // Log performance every second
                if (iteration_count_ % 100 == 0) {
                    logPerformance();
                }

            } catch (const std::exception& e) {
                health_.filter_healthy = false;
                {
                    std::stringstream msg;
                    msg << "Error in iteration " << iteration_count_ << ": " << e.what();
                    LOG_ERROR(msg.str());
                }
                // Continue processing - don't crash on single bad iteration
            }
        }

        if (!g_headless) {
            std::cout << "\nNavigation processing complete." << std::endl;
        }

        {
            std::stringstream msg;
            msg << "Navigation loop completed after " << iteration_count_ << " iterations";
            LOG_INFO(msg.str());
        }
        finalizeAndReport();
    }

    void printStatusUpdate() {
        auto now = high_resolution_clock::now();
        auto runtime = duration_cast<seconds>(now - system_start_time_).count();

        std::cout << "\r[" << std::setw(6) << runtime << "s] "
                  << "Iteration: " << std::setw(8) << iteration_count_
                  << " | Pos: ("
                  << std::fixed << std::setprecision(1)
                  << current_state_.position.x() << ", "
                  << current_state_.position.y() << ", "
                  << current_state_.position.z() << ") m"
                  << " | Vel: " << current_state_.velocity.norm() << " m/s"
                  << " | Resets: " << filter_->getResetCount()
                  << " | Perf: " << std::setprecision(1)
                  << perf_monitor_->getAverageTime() << " ms"
                  << std::flush;
    }

    void processIteration() {
        // 1. Read sensor data (real data from CSV files)
        auto sensor_data = sensors_->readNext();

        // 2. Validate sensor data
        auto validation_result = validator_->validateSensorData(sensor_data);
        if (!validation_result.valid) {
            std::stringstream msg;
            msg << "Invalid sensor data at time " << sensor_data.timestamp << ": " << validation_result.failure_reason;
            LOG_WARN(msg.str());
            return;
        }

        // 3. UKF prediction and update (100 Hz)
        auto ukf_start = high_resolution_clock::now();

        // IMU prediction
        filter_->predictUKF(sensor_data.imu, sensor_data.dt);

        // Measurement updates with logging
        if (sensor_data.has_baro) {
            filter_->updateBarometer(sensor_data.barometer);
            {
                std::stringstream msg;
                msg << "Barometer update: alt=" << sensor_data.barometer.altitude
                    << "m, pressure=" << sensor_data.barometer.pressure << "Pa";
                LOG_DEBUG(msg.str());
            }
        }
        if (sensor_data.has_mag) {
            filter_->updateMagnetometer(sensor_data.magnetometer);
            {
                std::stringstream msg;
                msg << "Magnetometer update: field=" << sensor_data.magnetometer.field.norm() * 1e6 << "μT";
                LOG_DEBUG(msg.str());
            }
        }
        if (sensor_data.has_grad) {
            filter_->updateGravity(sensor_data.gradiometer);
            {
                std::stringstream msg;
                msg << "Gradiometer update: confidence=" << sensor_data.gradiometer.confidence;
                LOG_DEBUG(msg.str());
            }
        }

        ukf_time_ms_ = duration_cast<microseconds>(
            high_resolution_clock::now() - ukf_start
        ).count() / 1000.0;

        // 4. ML bias update (1 Hz)
        auto now = high_resolution_clock::now();
        if (ml_predictor_ &&
            duration_cast<milliseconds>(now - last_ml_update_).count() >= 1000) {

            auto ml_start = high_resolution_clock::now();

            auto imu_buffer = sensors_->getIMUBuffer(1000); // Last 10 seconds
            auto [bias_pred, uncertainty] = ml_predictor_->predictBias(imu_buffer);

            if (ml_predictor_->checkOOD(imu_buffer) < 3.0) {
                filter_->updateMLBias(bias_pred, uncertainty);
                {
                    std::stringstream msg;
                    msg << "ML bias update applied: " << bias_pred.transpose();
                    LOG_DEBUG(msg.str());
                }
            } else {
                LOG_WARN("ML OOD detected, skipping bias update");
            }

            ml_time_ms_ = duration_cast<microseconds>(
                high_resolution_clock::now() - ml_start
            ).count() / 1000.0;

            last_ml_update_ = now;
        }

        // 5. RBPF update (using configured interval)
        int rbpf_interval = config_["rbpf"]["reset"]["min_interval_s"].as<int>(20);
        if (duration_cast<seconds>(now - last_rbpf_update_).count() >= rbpf_interval) {
            auto rbpf_start = high_resolution_clock::now();

            auto gravity_buffer = sensors_->getGravityBuffer();
            filter_->updateRBPF(gravity_buffer);

            // Check for reset
            if (filter_->shouldReset()) {
                auto reset_state = filter_->performReset();
                {
                    std::stringstream msg;
                    msg << "=== POSITION RESET === Jump: " << reset_state.position_jump
                        << " m, Confidence: " << reset_state.confidence;
                    LOG_INFO(msg.str());
                }
                Logger::getInstance().logReset(reset_state);
            }

            rbpf_time_ms_ = duration_cast<microseconds>(
                high_resolution_clock::now() - rbpf_start
            ).count() / 1000.0;

            last_rbpf_update_ = now;
        }

        // 6. Get current state estimate
        current_state_ = filter_->getState();
        current_state_.timestamp = sensor_data.timestamp;

        // 7. Log everything
        Logger::getInstance().logState(current_state_);

        // Convert SensorPacket to LoggedSensorData
        LoggedSensorData logged_data;
        logged_data.timestamp = sensor_data.timestamp;
        logged_data.dt = sensor_data.dt;

        // IMU data
        logged_data.imu.accel = sensor_data.imu.accel;
        logged_data.imu.gyro = sensor_data.imu.gyro;
        logged_data.imu.temperature = sensor_data.imu.temperature;

        // Barometer data
        logged_data.barometer.pressure = sensor_data.barometer.pressure;
        logged_data.barometer.temperature = sensor_data.barometer.temperature;
        logged_data.barometer.altitude = sensor_data.barometer.altitude;
        logged_data.has_baro = sensor_data.has_baro;

        // Magnetometer data
        logged_data.magnetometer.field = sensor_data.magnetometer.field;
        logged_data.magnetometer.declination = sensor_data.magnetometer.declination;
        logged_data.magnetometer.inclination = sensor_data.magnetometer.inclination;
        logged_data.has_mag = sensor_data.has_mag;

        // Gradiometer data
        logged_data.gradiometer.gradient_tensor = sensor_data.gradiometer.tensor.head<5>();
        logged_data.gradiometer.confidence = sensor_data.gradiometer.confidence;
        logged_data.has_gravity = sensor_data.has_grad;

        Logger::getInstance().logMeasurements(logged_data);
        Logger::getInstance().logCovariance(filter_->getCovariance());

        // 8. Real-time analysis (if enabled)
        if (analyzer_) {
            analyzer_->updateEstimate(current_state_);

            if (iteration_count_ % 100 == 0) {
                auto errors = analyzer_->getCurrentErrors();
                {
                    std::stringstream msg;
                    msg << "Current errors - Pos: " << errors.position_error
                        << " m, Vel: " << errors.velocity_error << " m/s";
                    LOG_INFO(msg.str());
                }
            }
        }
    }

    void logPerformance() {
        auto total_runtime = duration_cast<seconds>(
            high_resolution_clock::now() - system_start_time_
        ).count();

        {
            std::stringstream msg;
            msg << "=== Performance Report (Iteration " << iteration_count_ << ") ===";
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "Runtime: " << total_runtime << " seconds";
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "UKF time: " << ukf_time_ms_ << " ms";
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "RBPF time: " << rbpf_time_ms_ << " ms";
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "ML time: " << ml_time_ms_ << " ms";
            LOG_INFO(msg.str());
        }

        // Calculate total processing time
        total_time_ms_ = ukf_time_ms_ + rbpf_time_ms_ + ml_time_ms_;
        {
            std::stringstream msg;
            msg << "Total processing time: " << total_time_ms_ << " ms";
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "Average iteration: " << perf_monitor_->getAverageTime() << " ms";
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "Max iteration: " << perf_monitor_->getMaxTime() << " ms";
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "Budget violations: " << perf_monitor_->getBudgetViolations();
            LOG_INFO(msg.str());
        }

        Logger::getInstance().logPerformance(
            iteration_count_,
            ukf_time_ms_,
            rbpf_time_ms_,
            ml_time_ms_,
            perf_monitor_->getAverageTime()
        );
    }

    void finalizeAndReport() {
        LOG_INFO("=== Final Navigation Report ===");

        // Final position
        {
            std::stringstream msg;
            msg << "Final position: " << current_state_.position.transpose();
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "Final velocity: " << current_state_.velocity.transpose();
            LOG_INFO(msg.str());
        }

        // Overall statistics
        if (analyzer_) {
            auto stats = analyzer_->getFinalStatistics();
            LOG_INFO("=== Accuracy Statistics ===");
            {
                std::stringstream msg;
                msg << "Position RMSE: " << stats.position_rmse << " m";
                LOG_INFO(msg.str());
            }
            {
                std::stringstream msg;
                msg << "Velocity RMSE: " << stats.velocity_rmse << " m/s";
                LOG_INFO(msg.str());
            }
            {
                std::stringstream msg;
                msg << "Maximum position error: " << stats.max_position_error << " m";
                LOG_INFO(msg.str());
            }
            {
                std::stringstream msg;
                msg << "Final position error: " << stats.final_position_error << " m";
                LOG_INFO(msg.str());
            }
            {
                std::stringstream msg;
                msg << "Number of resets: " << filter_->getResetCount();
                LOG_INFO(msg.str());
            }

            // Save trajectory
            analyzer_->saveTrajectory("logs/estimated_trajectory.csv");
            analyzer_->saveErrorPlots("logs/error_analysis.png");
        }

        // Performance summary
        LOG_INFO("=== Performance Summary ===");
        {
            std::stringstream msg;
            msg << "Total iterations: " << iteration_count_;
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "Average iteration time: " << perf_monitor_->getAverageTime() << " ms";
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "Budget violations: " << perf_monitor_->getBudgetViolations();
            LOG_INFO(msg.str());
        }
        {
            std::stringstream msg;
            msg << "Success rate: "
                << (100.0 * (iteration_count_ - perf_monitor_->getBudgetViolations()) / iteration_count_)
                << "%";
            LOG_INFO(msg.str());
        }

        // Close all log files
        Logger::getInstance().finalize();
    }
};

int main(int argc, char* argv[]) {
    // Setup signal handlers for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        // Parse command line arguments
        CommandLineOptions options = parseCommandLine(argc, argv);

        // Handle help and version requests
        if (options.show_help) {
            printHelp(argv[0]);
            return 0;
        }

        if (options.show_version) {
            printVersion();
            return 0;
        }

        // Check for required config file
        if (options.config_file.empty()) {
            std::cerr << "Error: Configuration file is required." << std::endl;
            std::cerr << "Usage: " << argv[0] << " -c <config.yaml>" << std::endl;
            std::cerr << "Use -h for help." << std::endl;
            return 1;
        }

        // Create navigation system
        NavigationSystem nav_system(options);

        // Validate configuration
        if (!nav_system.validateConfiguration()) {
            std::cerr << "Configuration validation failed. Exiting." << std::endl;
            return 1;
        }

        // Handle dry-run mode
        if (options.dry_run) {
            if (!g_headless) {
                std::cout << "\n=== DRY RUN MODE ===" << std::endl;
                std::cout << "Configuration validated successfully." << std::endl;
                std::cout << "System would run with:" << std::endl;
                std::cout << "  Config file: " << options.config_file << std::endl;
                std::cout << "  Log level: " << options.log_level << std::endl;
                std::cout << "  Log directory: " << options.log_dir << std::endl;
                std::cout << "  Output format: " << options.output_format << std::endl;
                if (options.max_iterations > 0) {
                    std::cout << "  Max iterations: " << options.max_iterations << std::endl;
                }
                if (options.enable_profiling) {
                    std::cout << "  Profiling: ENABLED" << std::endl;
                }
                std::cout << "\nNo actual processing performed (dry-run mode)." << std::endl;
            }
            LOG_INFO("Dry-run completed successfully");
            return 0;
        }

        // Handle validate-only mode
        if (options.validate_only) {
            if (!g_headless) {
                std::cout << "\n=== VALIDATE ONLY MODE ===" << std::endl;
            }
            nav_system.initialize();
            if (!g_headless) {
                std::cout << "All components initialized and validated successfully." << std::endl;
                std::cout << "System is ready to run." << std::endl;
            }
            LOG_INFO("Validation completed successfully");
            return 0;
        }

        // Initialize navigation system
        nav_system.initialize();

        // Run navigation processing
        nav_system.run();

        if (!g_headless) {
            std::cout << "\nNavigation system shutdown complete." << std::endl;
        }

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "\nFatal error: " << e.what() << std::endl;
        {
            std::stringstream msg;
            msg << "Fatal error: " << e.what();
            LOG_ERROR(msg.str());
        }
        return 1;
    } catch (...) {
        std::cerr << "\nUnknown fatal error occurred." << std::endl;
        LOG_ERROR("Unknown fatal error occurred");
        return 1;
    }
}