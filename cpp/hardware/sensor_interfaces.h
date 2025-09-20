#pragma once

#include <memory>
#include <Eigen/Dense>
#include "../core/types.h"

/**
 * Sensor Factory for creating auxiliary sensors
 */
class SensorFactory {
public:
    static std::unique_ptr<class MagnetometerInterface> createMagnetometer(const std::string& type);
    static std::unique_ptr<class BarometerInterface> createBarometer(const std::string& type);
    static std::unique_ptr<class RadarAltimeterInterface> createRadarAltimeter(const std::string& type);
};

/**
 * Magnetometer Interface
 */
class MagnetometerInterface {
public:
    virtual ~MagnetometerInterface() = default;
    virtual bool initialize() = 0;
    virtual bool hasNewData() const = 0;
    virtual Eigen::Vector3d read() = 0;  // Returns magnetic field in body frame (Tesla)
};

/**
 * Barometric Altimeter Interface
 */
class BarometerInterface {
public:
    virtual ~BarometerInterface() = default;
    virtual bool initialize() = 0;
    virtual bool hasNewData() const = 0;
    virtual double readPressure() = 0;    // Returns pressure in hPa
    virtual double readAltitude() = 0;    // Returns altitude MSL in meters
};

/**
 * Radar Altimeter Interface
 */
class RadarAltimeterInterface {
public:
    virtual ~RadarAltimeterInterface() = default;
    virtual bool initialize() = 0;
    virtual bool hasNewData() const = 0;
    virtual double readAltitude() = 0;    // Returns altitude AGL in meters
};

/**
 * Terrain Database Interface
 */
class TerrainDatabase {
public:
    bool loadSRTM(const std::string& filepath) {
        // In production: load SRTM elevation data
        filepath_ = filepath;
        return false;  // Not implemented yet
    }

    double getElevation(double lat_deg, double lon_deg) {
        // In production: interpolate elevation from SRTM grid
        // For now, return synthetic terrain
        return 1000.0 + 500.0 * std::sin(lat_deg * M_PI / 30.0) * std::cos(lon_deg * M_PI / 30.0);
    }

private:
    std::string filepath_;
};

// Concrete sensor implementations
namespace {

class HMC5883L : public MagnetometerInterface {
public:
    bool initialize() override {
        // In production: initialize I2C communication
        return true;
    }

    bool hasNewData() const override {
        // In production: check I2C buffer
        return true;
    }

    Eigen::Vector3d read() override {
        // In production: read from HMC5883L over I2C
        // For now, return Earth's field (approx 50 μT)
        return Eigen::Vector3d(20e-6, 5e-6, -45e-6);  // Tesla
    }
};

class BMP388 : public BarometerInterface {
public:
    bool initialize() override {
        // In production: initialize I2C/SPI communication
        return true;
    }

    bool hasNewData() const override {
        return true;
    }

    double readPressure() override {
        // Standard sea-level pressure
        return 1013.25;  // hPa
    }

    double readAltitude() override {
        // Convert pressure to altitude using standard atmosphere
        double pressure = readPressure();
        double altitude = 44330.0 * (1.0 - std::pow(pressure / 1013.25, 0.1903));
        return altitude;
    }
};

class KRA405B : public RadarAltimeterInterface {
public:
    bool initialize() override {
        // In production: initialize radar module
        return true;
    }

    bool hasNewData() const override {
        return true;
    }

    double readAltitude() override {
        // In production: read radar range
        // For now, return simulated AGL
        return 1000.0;  // meters AGL
    }
};

} // anonymous namespace

// Factory implementations
inline std::unique_ptr<MagnetometerInterface> SensorFactory::createMagnetometer(const std::string& type) {
    if (type == "HMC5883L") {
        return std::make_unique<HMC5883L>();
    }
    return nullptr;
}

inline std::unique_ptr<BarometerInterface> SensorFactory::createBarometer(const std::string& type) {
    if (type == "BMP388") {
        return std::make_unique<BMP388>();
    }
    return nullptr;
}

inline std::unique_ptr<RadarAltimeterInterface> SensorFactory::createRadarAltimeter(const std::string& type) {
    if (type == "KRA405B") {
        return std::make_unique<KRA405B>();
    }
    return nullptr;
}