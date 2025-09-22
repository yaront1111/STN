#include "python_bridge.h"
#include "json_parser.h"
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <cstring>
#include <chrono>

namespace ml {

PythonMLBridge::PythonMLBridge()
    : to_python_(nullptr), from_python_(nullptr), python_pid_(-1), is_running_(false) {
}

PythonMLBridge::~PythonMLBridge() {
    shutdown();
}

bool PythonMLBridge::initialize(const std::string& python_path, const std::string& script_path) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (is_running_.load()) {
        return true;  // Already initialized
    }

    // Create pipes for bidirectional communication
    int to_python[2], from_python[2];

    if (pipe(to_python) == -1 || pipe(from_python) == -1) {
        std::cerr << "[PythonMLBridge] Failed to create pipes: " << strerror(errno) << std::endl;
        return false;
    }

    // Fork process
    python_pid_ = fork();

    if (python_pid_ == -1) {
        std::cerr << "[PythonMLBridge] Failed to fork process: " << strerror(errno) << std::endl;
        return false;
    }

    if (python_pid_ == 0) {
        // Child process - run Python script

        // Redirect stdin/stdout
        dup2(to_python[0], STDIN_FILENO);
        dup2(from_python[1], STDOUT_FILENO);

        // Close unused pipe ends
        close(to_python[1]);
        close(from_python[0]);
        close(to_python[0]);
        close(from_python[1]);

        // Execute Python script
        execlp(python_path.c_str(), python_path.c_str(), script_path.c_str(), nullptr);

        // If we get here, exec failed
        std::cerr << "[PythonMLBridge] Failed to execute Python script: " << strerror(errno) << std::endl;
        exit(1);
    }

    // Parent process
    close(to_python[0]);
    close(from_python[1]);

    // Create FILE* for easier I/O
    to_python_ = fdopen(to_python[1], "w");
    from_python_ = fdopen(from_python[0], "r");

    if (!to_python_ || !from_python_) {
        std::cerr << "[PythonMLBridge] Failed to create FILE streams" << std::endl;
        return false;
    }

    // Wait for Python server to be ready
    char buffer[256];
    if (fgets(buffer, sizeof(buffer), from_python_)) {
        std::string response(buffer);
        if (response.find("READY") != std::string::npos) {
            is_running_ = true;
            std::cout << "[PythonMLBridge] ML inference server started successfully" << std::endl;
            return true;
        }
    }

    std::cerr << "[PythonMLBridge] Python server failed to start" << std::endl;
    return false;
}

bool PythonMLBridge::sendCommand(const std::string& json_command) {
    if (!to_python_ || !is_running_.load()) {
        return false;
    }

    fprintf(to_python_, "%s\n", json_command.c_str());
    fflush(to_python_);
    return true;
}

std::string PythonMLBridge::readResponse() {
    if (!from_python_ || !is_running_.load()) {
        return "{}";
    }

    char buffer[4096];
    if (fgets(buffer, sizeof(buffer), from_python_)) {
        return std::string(buffer);
    }

    return "{}";
}

PythonMLBridge::IMUCorrection PythonMLBridge::parseCorrection(const std::string& json_response) {
    IMUCorrection correction;
    correction.ready = false;
    correction.confidence = 0.0;
    correction.inference_time_ms = 0.0;

    try {
        // Use the new SimpleJSON parser
        auto parsed = SimpleJSON::parse(json_response);

        // Parse ready flag
        if (parsed.count("ready")) {
            correction.ready = SimpleJSON::parseBool(parsed["ready"]);
        }

        if (!correction.ready) {
            return correction;
        }

        // Parse acc_bias array
        if (parsed.count("acc_bias")) {
            auto acc_values = SimpleJSON::parseArray(parsed["acc_bias"]);
            for (size_t i = 0; i < std::min(acc_values.size(), size_t(3)); ++i) {
                correction.acc_bias(i) = acc_values[i];
            }
        }

        // Parse gyro_bias array
        if (parsed.count("gyro_bias")) {
            auto gyro_values = SimpleJSON::parseArray(parsed["gyro_bias"]);
            for (size_t i = 0; i < std::min(gyro_values.size(), size_t(3)); ++i) {
                correction.gyro_bias(i) = gyro_values[i];
            }
        }

        // Parse confidence
        if (parsed.count("confidence")) {
            correction.confidence = SimpleJSON::parseDouble(parsed["confidence"]);
        }

        // Parse inference time
        if (parsed.count("inference_time_ms")) {
            correction.inference_time_ms = SimpleJSON::parseDouble(parsed["inference_time_ms"]);
        }

    } catch (const std::exception& e) {
        std::cerr << "[PythonMLBridge] Error parsing response: " << e.what() << std::endl;
        std::cerr << "[PythonMLBridge] Response was: " << json_response << std::endl;
        correction.ready = false;
    }

    return correction;
}

PythonMLBridge::IMUCorrection PythonMLBridge::addSampleAndPredict(
    const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro) {

    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_running_.load()) {
        IMUCorrection empty;
        empty.ready = false;
        return empty;
    }

    // Create JSON command
    std::stringstream cmd;
    cmd << "{\"type\": \"add_sample\", ";
    cmd << "\"acc_x\": " << acc(0) << ", ";
    cmd << "\"acc_y\": " << acc(1) << ", ";
    cmd << "\"acc_z\": " << acc(2) << ", ";
    cmd << "\"gyro_x\": " << gyro(0) << ", ";
    cmd << "\"gyro_y\": " << gyro(1) << ", ";
    cmd << "\"gyro_z\": " << gyro(2) << "}";

    if (!sendCommand(cmd.str())) {
        IMUCorrection empty;
        empty.ready = false;
        return empty;
    }

    std::string response = readResponse();
    return parseCorrection(response);
}

PythonMLBridge::IMUCorrection PythonMLBridge::predict() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_running_.load()) {
        IMUCorrection empty;
        empty.ready = false;
        return empty;
    }

    std::string cmd = "{\"type\": \"predict\"}";
    if (!sendCommand(cmd)) {
        IMUCorrection empty;
        empty.ready = false;
        return empty;
    }

    std::string response = readResponse();
    return parseCorrection(response);
}

PythonMLBridge::Stats PythonMLBridge::getStats() {
    std::lock_guard<std::mutex> lock(mutex_);

    Stats stats = {0, 0.0};

    if (!is_running_.load()) {
        return stats;
    }

    std::string cmd = "{\"type\": \"stats\"}";
    if (!sendCommand(cmd)) {
        return stats;
    }

    std::string response = readResponse();

    // Parse stats using SimpleJSON
    try {
        auto parsed = SimpleJSON::parse(response);

        // Check if we have models field (nested object)
        if (parsed.count("models")) {
            // For now, just get IMU stats as it's the primary model
            auto models_str = parsed["models"];
            auto models_data = SimpleJSON::parse(models_str);

            if (models_data.count("imu")) {
                auto imu_stats_str = models_data["imu"];
                auto imu_stats = SimpleJSON::parse(imu_stats_str);

                if (imu_stats.count("inference_count")) {
                    stats.inference_count = static_cast<int>(SimpleJSON::parseDouble(imu_stats["inference_count"]));
                }

                if (imu_stats.count("average_time_ms")) {
                    stats.average_time_ms = SimpleJSON::parseDouble(imu_stats["average_time_ms"]);
                }
            }
        }
    } catch (...) {
        // Ignore parsing errors
    }

    return stats;
}

void PythonMLBridge::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!is_running_.load()) {
        return;
    }

    // Send shutdown command
    std::string cmd = "{\"type\": \"shutdown\"}";
    sendCommand(cmd);

    is_running_ = false;

    // Close pipes
    if (to_python_) {
        fclose(to_python_);
        to_python_ = nullptr;
    }
    if (from_python_) {
        fclose(from_python_);
        from_python_ = nullptr;
    }

    // Wait for Python process to terminate
    if (python_pid_ > 0) {
        int status;
        waitpid(python_pid_, &status, 0);
        python_pid_ = -1;
    }

    std::cout << "[PythonMLBridge] ML inference server shut down" << std::endl;
}

} // namespace ml