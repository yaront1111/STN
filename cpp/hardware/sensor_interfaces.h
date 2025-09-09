#pragma once
#include <Eigen/Dense>
#include <memory>

/**
 * GPS-DENIED SENSOR INTERFACES
 * For hardware abstraction of additional sensors
 */

// Magnetometer interface for heading reference
class MagnetometerInterface {
public:
    virtual ~MagnetometerInterface() = default;
    virtual bool initialize() = 0;
    virtual bool hasNewData() = 0;
    virtual Eigen::Vector3d read() = 0;  // Returns field in Tesla
};

// Barometric altimeter interface
class BarometerInterface {
public:
    virtual ~BarometerInterface() = default;
    virtual bool initialize() = 0;
    virtual bool hasNewData() = 0;
    virtual double readPressure() = 0;   // Pa
    virtual double readAltitude() = 0;   // meters MSL
};

// Radar altimeter interface for terrain correlation
class RadarAltimeterInterface {
public:
    virtual ~RadarAltimeterInterface() = default;
    virtual bool initialize() = 0;
    virtual bool hasNewData() = 0;
    virtual double readAltitude() = 0;   // meters AGL
};

// Terrain database for correlation
class TerrainDatabase {
public:
    bool loadSRTM(const std::string& filename) {
        // Placeholder - would load real SRTM data
        return true;
    }
    
    double getElevation(double lat_deg, double lon_deg) {
        // Placeholder - would query real terrain
        // Return synthetic terrain for testing
        return 500.0 + 100.0 * std::sin(lat_deg * 0.1) * std::cos(lon_deg * 0.1);
    }
};

// Factory functions for hardware creation
namespace SensorFactory {
    inline std::unique_ptr<MagnetometerInterface> createMagnetometer(const std::string& type) {
        // Stub implementation
        class StubMagnetometer : public MagnetometerInterface {
            bool initialize() override { return true; }
            bool hasNewData() override { return true; }
            Eigen::Vector3d read() override {
                // Return Earth-like field (~50 μT)
                return Eigen::Vector3d(20e-6, 0, -45e-6);
            }
        };
        return std::make_unique<StubMagnetometer>();
    }
    
    inline std::unique_ptr<BarometerInterface> createBarometer(const std::string& type) {
        // Stub implementation
        class StubBarometer : public BarometerInterface {
            bool initialize() override { return true; }
            bool hasNewData() override { return true; }
            double readPressure() override { return 101325.0; }  // Sea level
            double readAltitude() override { return 0.0; }       // MSL
        };
        return std::make_unique<StubBarometer>();
    }
    
    inline std::unique_ptr<RadarAltimeterInterface> createRadarAltimeter(const std::string& type) {
        // Stub implementation
        class StubRadarAlt : public RadarAltimeterInterface {
            bool initialize() override { return true; }
            bool hasNewData() override { return true; }
            double readAltitude() override { return 1000.0; }  // 1km AGL
        };
        return std::make_unique<StubRadarAlt>();
    }
}