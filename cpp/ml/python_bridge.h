#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <queue>
#include <iostream>
#include <Eigen/Dense>

namespace ml {

/**
 * Python ML Bridge for real-time inference
 * Communicates with Python inference server via subprocess
 */
class PythonMLBridge {
public:
    struct IMUCorrection {
        Eigen::Vector3d acc_bias;
        Eigen::Vector3d gyro_bias;
        double confidence;
        bool ready;
        double inference_time_ms;
    };

    PythonMLBridge();
    ~PythonMLBridge();

    // Start the Python inference server
    bool initialize(const std::string& python_path = "../ml/venv/bin/python",
                   const std::string& script_path = "../ml/ml_inference_server.py");

    // Add IMU sample and get correction
    IMUCorrection addSampleAndPredict(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro);

    // Just get prediction with current buffer
    IMUCorrection predict();

    // Check if server is running
    bool isRunning() const { return is_running_.load(); }

    // Get performance stats
    struct Stats {
        int inference_count;
        double average_time_ms;
    };
    Stats getStats();

    // Shutdown the server
    void shutdown();

private:
    // Process management
    FILE* to_python_;
    FILE* from_python_;
    pid_t python_pid_;
    std::atomic<bool> is_running_;

    // Thread safety
    mutable std::mutex mutex_;

    // Helper methods
    bool sendCommand(const std::string& json_command);
    std::string readResponse();
    IMUCorrection parseCorrection(const std::string& json_response);
};

/**
 * Singleton ML Corrector for global access
 */
class MLCorrector {
public:
    static MLCorrector& getInstance() {
        static MLCorrector instance;
        return instance;
    }

    // Initialize the ML system
    bool initialize() {
        if (!bridge_) {
            bridge_ = std::make_unique<PythonMLBridge>();
            // Use the unified ML server for all sensors
            return bridge_->initialize("../ml/venv/bin/python", "../ml/ml_unified_server.py");
        }
        return true;
    }

    // Apply ML correction to IMU data
    void correctIMU(Eigen::Vector3d& acc, Eigen::Vector3d& gyro) {
        if (!bridge_ || !bridge_->isRunning()) {
            return;  // No correction if ML not available
        }

        auto correction = bridge_->addSampleAndPredict(acc, gyro);

        if (correction.ready && correction.confidence > 0.5) {
            // Apply bias corrections with confidence weighting
            double weight = correction.confidence;
            acc -= correction.acc_bias * weight;
            gyro -= correction.gyro_bias * weight;

            // FIX: Always update last correction, not just high confidence
            last_high_confidence_correction_ = correction;

            // Log correction for debugging
            if (correction.confidence > 0.8) {
                correction_count_++;
                if (correction_count_ % 100 == 0) {
                    std::cout << "[ML] Applied " << correction_count_ << " corrections, "
                              << "last confidence: " << correction.confidence << std::endl;
                }
            }
        }
    }

    // Get last high-confidence correction for diagnostics
    PythonMLBridge::IMUCorrection getLastCorrection() const {
        return last_high_confidence_correction_;
    }

    // Check if ML is available
    bool isAvailable() const {
        return bridge_ && bridge_->isRunning();
    }

    // Shutdown ML system cleanly
    void shutdown() {
        if (bridge_) {
            bridge_->shutdown();
            bridge_.reset();
        }
    }

private:
    MLCorrector() = default;
    ~MLCorrector() {
        shutdown();
    }
    std::unique_ptr<PythonMLBridge> bridge_;
    PythonMLBridge::IMUCorrection last_high_confidence_correction_;
    int correction_count_ = 0;
};

} // namespace ml