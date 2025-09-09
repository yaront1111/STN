#pragma once
#include "../core/types.h"
#include <memory>
#include <string>

/**
 * Gravity Navigation Hardware Interfaces
 */

// IMU Interface
class IMUInterface {
public:
    virtual ~IMUInterface() = default;
    virtual bool initialize(const std::string& port) = 0;
    virtual bool read(ImuSample& sample) = 0;
    virtual std::string getModelName() const = 0;
};

// Gravity Gradiometer Interface
class GradiometerInterface {
public:
    virtual ~GradiometerInterface() = default;
    virtual bool initialize() = 0;
    virtual bool hasNewData() const = 0;
    virtual GravityGradientTensor read() = 0;
};

// Chip Scale Atomic Clock Interface
class CSACInterface {
public:
    virtual ~CSACInterface() = default;
    virtual bool initialize() = 0;
    virtual bool hasNewData() const = 0;
    virtual CSACMeasurement read() = 0;
};

// Simulated implementations for testing
class SimulatedIMU : public IMUInterface {
private:
    double t_ = 0.0;
    
public:
    bool initialize(const std::string&) override { return true; }
    
    bool read(ImuSample& sample) override {
        sample.t = t_;
        sample.acc_mps2 = Eigen::Vector3d(0, 0, -9.80665) + Eigen::Vector3d::Random() * 0.01;
        sample.gyro_rps = Eigen::Vector3d::Random() * 0.001;
        t_ += 0.01; // 100Hz
        return true;
    }
    
    std::string getModelName() const override { return "Simulated IMU"; }
};

class SimulatedGradiometer : public GradiometerInterface {
private:
    int count_ = 0;
    
public:
    bool initialize() override { return true; }
    
    bool hasNewData() const override {
        return (count_ % 100) == 0; // 1Hz updates
    }
    
    GravityGradientTensor read() override {
        count_++;
        GravityGradientTensor gradient;
        gradient.T << 3.0, 0.1, 0.2,
                      0.1, -1.5, 0.15,
                      0.2, 0.15, -1.5;
        gradient.R = Eigen::Matrix3d::Identity() * 0.1; // 0.1 E noise
        gradient.t = count_ * 0.01;
        return gradient;
    }
};

class SimulatedCSAC : public CSACInterface {
private:
    int count_ = 0;
    
public:
    bool initialize() override { return true; }
    
    bool hasNewData() const override {
        return (count_ % 1000) == 0; // 0.1Hz updates
    }
    
    CSACMeasurement read() override {
        count_++;
        CSACMeasurement meas;
        meas.offset_s = 1e-9 * std::sin(count_ * 0.001);
        meas.drift_ppm = 1e-12 * std::cos(count_ * 0.001);
        meas.allan_deviation = 1e-13;
        return meas;
    }
};

// Hardware factory
class HardwareInterface {
public:
    static std::unique_ptr<IMUInterface> createIMU(const std::string& type) {
        if (type == "simulated") {
            return std::make_unique<SimulatedIMU>();
        }
        // Add real hardware implementations here
        return std::make_unique<SimulatedIMU>();
    }
    
    static std::unique_ptr<GradiometerInterface> createGradiometer(const std::string& type) {
        if (type == "simulated") {
            return std::make_unique<SimulatedGradiometer>();
        }
        // Add real gradiometer implementations here
        return nullptr;
    }
    
    static std::unique_ptr<CSACInterface> createCSAC(const std::string& type) {
        if (type == "simulated") {
            return std::make_unique<SimulatedCSAC>();
        }
        // Add real CSAC implementations here
        return nullptr;
    }
};