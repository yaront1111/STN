#pragma once
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <functional>
#include <Eigen/Dense>
#include "types.h"

/**
 * Hardware Abstraction Layer for STN
 * Provides unified interface for various IMU and sensor hardware
 */

// Sensor data structures
struct IMUData {
    uint64_t timestamp_ns;
    Eigen::Vector3d accel;  // m/s²
    Eigen::Vector3d gyro;   // rad/s
    double temperature;     // Celsius
    bool valid;
};

struct AltimeterData {
    uint64_t timestamp_ns;
    double range_m;
    double signal_strength;
    bool valid;
};

struct GPSData {
    uint64_t timestamp_ns;
    double lat, lon, alt;
    double vel_n, vel_e, vel_d;
    double hdop, vdop;
    int num_satellites;
    bool valid;
};

struct BarometerData {
    uint64_t timestamp_ns;
    double pressure_pa;
    double temperature_c;
    bool valid;
};

// Abstract sensor interfaces
class IMUSensor {
public:
    virtual ~IMUSensor() = default;
    
    virtual bool initialize(const std::string& config) = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual bool readData(IMUData& data) = 0;
    virtual double getSampleRate() const = 0;
    virtual std::string getModelName() const = 0;
};

class AltimeterSensor {
public:
    virtual ~AltimeterSensor() = default;
    
    virtual bool initialize(const std::string& config) = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual bool readData(AltimeterData& data) = 0;
    virtual double getMaxRange() const = 0;
    virtual double getMinRange() const = 0;
};

// Concrete implementations for common hardware

/**
 * VectorNav VN-200 IMU/GPS
 * High-performance tactical grade IMU
 */
class VectorNavVN200 : public IMUSensor {
private:
    std::string port_name_;
    int baudrate_;
    bool is_running_;
    std::thread read_thread_;
    std::queue<IMUData> data_queue_;
    std::mutex queue_mutex_;
    
    void serialReadLoop();
    
public:
    VectorNavVN200() : baudrate_(921600), is_running_(false) {}
    
    bool initialize(const std::string& config) override;
    bool start() override;
    bool stop() override;
    bool readData(IMUData& data) override;
    double getSampleRate() const override { return 200.0; }
    std::string getModelName() const override { return "VectorNav VN-200"; }
};

/**
 * InvenSense MPU-9250 (Consumer grade)
 * Common low-cost MEMS IMU
 */
class MPU9250 : public IMUSensor {
private:
    int i2c_bus_;
    uint8_t i2c_addr_;
    bool is_initialized_;
    
public:
    MPU9250() : i2c_bus_(1), i2c_addr_(0x68), is_initialized_(false) {}
    
    bool initialize(const std::string& config) override;
    bool start() override;
    bool stop() override;
    bool readData(IMUData& data) override;
    double getSampleRate() const override { return 100.0; }
    std::string getModelName() const override { return "InvenSense MPU-9250"; }
};

/**
 * Simulated IMU for testing
 */
class SimulatedIMU : public IMUSensor {
private:
    std::chrono::steady_clock::time_point start_time_;
    double sample_rate_;
    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d accel_bias_;
    
public:
    SimulatedIMU() : sample_rate_(200.0) {}
    
    bool initialize(const std::string& config) override {
        start_time_ = std::chrono::steady_clock::now();
        gyro_bias_ = Eigen::Vector3d::Random() * 0.01;  // 0.01 rad/s bias
        accel_bias_ = Eigen::Vector3d::Random() * 0.1;  // 0.1 m/s² bias
        return true;
    }
    
    bool start() override { return true; }
    bool stop() override { return true; }
    
    bool readData(IMUData& data) override {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_time_);
        
        data.timestamp_ns = elapsed.count();
        data.accel = Eigen::Vector3d(0, 0, -9.80665) + accel_bias_ + Eigen::Vector3d::Random() * 0.01;
        data.gyro = gyro_bias_ + Eigen::Vector3d::Random() * 0.001;
        data.temperature = 25.0 + ((double)rand() / RAND_MAX - 0.5);
        data.valid = true;
        
        return true;
    }
    
    double getSampleRate() const override { return sample_rate_; }
    std::string getModelName() const override { return "Simulated IMU"; }
};

/**
 * Hardware Manager - Main interface for all sensors
 */
class HardwareManager {
private:
    std::unique_ptr<IMUSensor> imu_;
    std::unique_ptr<AltimeterSensor> altimeter_;
    
    // Data buffers with thread-safe access
    std::queue<IMUData> imu_buffer_;
    std::queue<AltimeterData> alt_buffer_;
    std::mutex buffer_mutex_;
    
    // Timing
    std::chrono::steady_clock::time_point init_time_;
    
    // Callbacks for real-time processing
    std::function<void(const IMUData&)> imu_callback_;
    std::function<void(const AltimeterData&)> alt_callback_;
    
public:
    HardwareManager() : init_time_(std::chrono::steady_clock::now()) {}
    
    // Hardware selection and initialization
    bool initializeIMU(const std::string& type, const std::string& config);
    bool initializeAltimeter(const std::string& type, const std::string& config);
    
    // Start/stop all sensors
    bool startAll();
    bool stopAll();
    
    // Data access (polling)
    bool getLatestIMU(IMUData& data);
    bool getLatestAltimeter(AltimeterData& data);
    
    // Data access (callback)
    void setIMUCallback(std::function<void(const IMUData&)> callback) {
        imu_callback_ = callback;
    }
    
    void setAltimeterCallback(std::function<void(const AltimeterData&)> callback) {
        alt_callback_ = callback;
    }
    
    // Time synchronization
    uint64_t getCurrentTimestamp() const {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(now - init_time_);
        return elapsed.count();
    }
    
    // Hardware info
    std::string getIMUInfo() const {
        return imu_ ? imu_->getModelName() : "No IMU";
    }
    
    double getIMUSampleRate() const {
        return imu_ ? imu_->getSampleRate() : 0.0;
    }
};

// Implementation of Hardware Manager methods
inline bool HardwareManager::initializeIMU(const std::string& type, const std::string& config) {
    if (type == "VN200") {
        imu_ = std::make_unique<VectorNavVN200>();
    } else if (type == "MPU9250") {
        imu_ = std::make_unique<MPU9250>();
    } else if (type == "simulated") {
        imu_ = std::make_unique<SimulatedIMU>();
    } else {
        return false;
    }
    
    return imu_->initialize(config);
}

inline bool HardwareManager::startAll() {
    bool success = true;
    
    if (imu_) {
        success &= imu_->start();
    }
    
    if (altimeter_) {
        // altimeter_->start();
    }
    
    return success;
}

inline bool HardwareManager::stopAll() {
    bool success = true;
    
    if (imu_) {
        success &= imu_->stop();
    }
    
    if (altimeter_) {
        // altimeter_->stop();
    }
    
    return success;
}

inline bool HardwareManager::getLatestIMU(IMUData& data) {
    if (!imu_) return false;
    
    if (imu_->readData(data)) {
        // Call callback if registered
        if (imu_callback_) {
            imu_callback_(data);
        }
        return true;
    }
    
    return false;
}

/**
 * Real-time navigation interface
 * Connects hardware to STN navigation engine
 */
class RealTimeNavigator {
private:
    HardwareManager hw_manager_;
    std::thread nav_thread_;
    bool running_;
    
    // Navigation state
    State current_state_;
    std::mutex state_mutex_;
    
    // Output callback
    std::function<void(const State&)> output_callback_;
    
    void navigationLoop();
    
public:
    RealTimeNavigator() : running_(false) {}
    
    bool initialize(const std::string& config_file);
    bool start();
    bool stop();
    
    State getCurrentState() const {
        std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(state_mutex_));
        return current_state_;
    }
    
    void setOutputCallback(std::function<void(const State&)> callback) {
        output_callback_ = callback;
    }
};