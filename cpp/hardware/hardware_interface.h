#pragma once

#include <memory>
#include <string>
#include "../core/types.h"

/**
 * Hardware Interface Factory
 * Creates concrete sensor implementations based on configuration
 */
class HardwareInterface {
public:
    // Factory methods for creating sensor interfaces
    static std::unique_ptr<class IMUInterface> createIMU(const std::string& type);
    static std::unique_ptr<class GradiometerInterface> createGradiometer(const std::string& type);
    static std::unique_ptr<class CSACInterface> createCSAC(const std::string& type);
};

/**
 * IMU Interface base class
 */
class IMUInterface {
public:
    virtual ~IMUInterface() = default;

    // Initialize the IMU with port/configuration
    virtual bool initialize(const std::string& port) = 0;

    // Read IMU data
    virtual bool read(ImuSample& sample) = 0;

    // Get device model name
    virtual std::string getModelName() const = 0;
};

/**
 * Gravity Gradiometer Interface
 */
class GradiometerInterface {
public:
    virtual ~GradiometerInterface() = default;

    // Initialize the gradiometer
    virtual bool initialize() = 0;

    // Check if new data is available
    virtual bool hasNewData() const = 0;

    // Read gradient tensor
    virtual GravityGradientTensor read() = 0;
};

/**
 * Chip Scale Atomic Clock Interface
 */
class CSACInterface {
public:
    virtual ~CSACInterface() = default;

    // Initialize the CSAC
    virtual bool initialize() = 0;

    // Check for new timing data
    virtual bool hasNewData() const = 0;

    // Read CSAC measurement
    struct CSACMeasurement {
        double time_offset;  // seconds
        double frequency_offset;  // Hz
        double allan_deviation;
    };

    virtual CSACMeasurement read() = 0;
};

// Concrete implementations (stubs for now)
namespace {

class VectorNavVN200 : public IMUInterface {
public:
    bool initialize(const std::string& port) override {
        port_ = port;
        // In production: open serial port, configure VN-200
        return true;
    }

    bool read(ImuSample& sample) override {
        // In production: read from VN-200 over serial
        // For now, return simulated data
        sample.t = 0;
        sample.acc_mps2 = Eigen::Vector3d(0, 0, 9.81);
        sample.gyro_rps = Eigen::Vector3d::Zero();
        return true;
    }

    std::string getModelName() const override {
        return "VectorNav VN-200";
    }

private:
    std::string port_;
};

class LockheedQuantumGradiometer : public GradiometerInterface {
public:
    bool initialize() override {
        // In production: initialize quantum sensor
        return false;  // Not yet available
    }

    bool hasNewData() const override {
        return false;
    }

    GravityGradientTensor read() override {
        GravityGradientTensor tensor;
        tensor.T = Eigen::Matrix3d::Zero();
        tensor.t = 0;
        return tensor;
    }
};

class MicrosemiSA45 : public CSACInterface {
public:
    bool initialize() override {
        // In production: initialize CSAC over serial
        return false;  // Optional hardware
    }

    bool hasNewData() const override {
        return false;
    }

    CSACMeasurement read() override {
        return CSACMeasurement{0, 0, 1e-11};
    }
};

} // anonymous namespace

// Factory implementations
inline std::unique_ptr<IMUInterface> HardwareInterface::createIMU(const std::string& type) {
    if (type == "VectorNav-VN200" || type == "VN200") {
        return std::make_unique<VectorNavVN200>();
    }
    // Add other IMU types as needed
    return nullptr;
}

inline std::unique_ptr<GradiometerInterface> HardwareInterface::createGradiometer(const std::string& type) {
    if (type == "Lockheed-Quantum") {
        return std::make_unique<LockheedQuantumGradiometer>();
    }
    return nullptr;
}

inline std::unique_ptr<CSACInterface> HardwareInterface::createCSAC(const std::string& type) {
    if (type == "Microsemi-SA45" || type == "SA45") {
        return std::make_unique<MicrosemiSA45>();
    }
    return nullptr;
}