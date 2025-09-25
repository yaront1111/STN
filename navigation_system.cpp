/**
 * GPS-Free Navigation System - Main Runner
 * Complete implementation with real data processing
 * No mocks, no stubs - everything works
 */

#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <sstream>
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

// Global flag for graceful shutdown
std::atomic<bool> g_running{true};

void signalHandler(int signum) {
    std::stringstream msg;
    msg << "Received signal " << signum << ", shutting down...";
    LOG_INFO(msg.str());
    g_running = false;
}

class NavigationSystem {
private:
    // Configuration
    YAML::Node config_;

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

    // System state
    Navigation::StateVector current_state_;
    bool system_initialized_{false};
    uint64_t iteration_count_{0};

    // Performance tracking
    double ukf_time_ms_{0};
    double rbpf_time_ms_{0};
    double ml_time_ms_{0};
    double total_time_ms_{0};

public:
    NavigationSystem(const std::string& config_file) {
        LOG_INFO("=== GPS-Free Navigation System Starting ===");
        {
            std::stringstream msg;
            msg << "Loading configuration from: " << config_file;
            LOG_INFO(msg.str());
        }

        // Load configuration
        config_ = YAML::LoadFile(config_file);

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

        // Verify we're in real data mode (no simulation allowed)
        if (config_["system"]["mode"].as<std::string>() != "real_data") {
            LOG_ERROR("System must run in real_data mode. No simulations allowed.");
            throw std::runtime_error("Invalid mode - must be real_data");
        }
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

        LOG_INFO("Starting main navigation loop...");
        LOG_INFO("Target rate: 100 Hz (10 ms per iteration)");

        last_ukf_update_ = high_resolution_clock::now();
        last_rbpf_update_ = high_resolution_clock::now();
        last_ml_update_ = high_resolution_clock::now();

        while (g_running && sensors_->hasMoreData()) {
            auto iteration_start = high_resolution_clock::now();
            iteration_count_++;

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
                    {
                        std::stringstream msg;
                        msg << "Iteration " << iteration_count_ << " exceeded budget: "
                            << iteration_ms << " ms";
                        LOG_WARN(msg.str());
                    }
                }

                // Log performance every second
                if (iteration_count_ % 100 == 0) {
                    logPerformance();
                }

            } catch (const std::exception& e) {
                {
                    std::stringstream msg;
                    msg << "Error in iteration " << iteration_count_ << ": " << e.what();
                    LOG_ERROR(msg.str());
                }
                // Continue processing - don't crash on single bad iteration
            }
        }

        {
            std::stringstream msg;
            msg << "Navigation loop completed after " << iteration_count_ << " iterations";
            LOG_INFO(msg.str());
        }
        finalizeAndReport();
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

        // Measurement updates with dimension checks
        if (sensor_data.has_baro) {
            filter_->updateBarometer(sensor_data.barometer);
        }
        if (sensor_data.has_mag) {
            // Validate magnetometer data dimension
            if (sensor_data.magnetometer.field.size() == 3) {
                filter_->updateMagnetometer(sensor_data.magnetometer);
            } else {
                LOG_ERROR("Invalid magnetometer field dimension");
            }
        }
        if (sensor_data.has_grad) {
            // Validate gradiometer tensor dimension
            if (sensor_data.gradiometer.tensor.size() == 6) {
                filter_->updateGravity(sensor_data.gradiometer);
            } else {
                LOG_ERROR("Invalid gradiometer tensor dimension");
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

        // 5. RBPF update (every 30 seconds)
        if (duration_cast<seconds>(now - last_rbpf_update_).count() >= 30) {
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
    // Setup signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        // Parse command line arguments
        if (argc != 2) {
            std::cerr << "Usage: " << argv[0] << " <config.yaml>" << std::endl;
            return 1;
        }

        std::string config_file = argv[1];

        // Create and initialize navigation system
        NavigationSystem nav_system(config_file);
        nav_system.initialize();

        // Run navigation processing
        nav_system.run();

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << std::endl;
        {
            std::stringstream msg;
            msg << "Fatal error: " << e.what();
            LOG_ERROR(msg.str());
        }
        return 1;
    }
}