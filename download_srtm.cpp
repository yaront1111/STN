/**
 * SRTM Terrain Data Generator
 * Creates synthetic elevation data for testing
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <filesystem>
#include <random>
#include <Eigen/Dense>

namespace fs = std::filesystem;

// SRTM parameters
constexpr double SRTM_RES_ARCSEC = 1.0;  // 1 arc-second (~30m)
constexpr int TILE_SIZE = 3601;          // 1 degree tile (1 arc-sec resolution)
constexpr double NO_DATA_VALUE = -32768.0;

class SRTMGenerator {
public:
    SRTMGenerator() : rng_(42) {}

    bool generateData(const std::string& output_path) {
        std::cout << "=== SRTM Terrain Data Generator ===" << std::endl;
        std::cout << "Generating synthetic terrain elevation data..." << std::endl;

        // Create output directory
        fs::create_directories(output_path);

        // Generate global elevation database (simplified grid)
        if (!generateElevationDatabase(output_path + "/elevation_1arcsec.dat")) {
            return false;
        }

        // Generate metadata
        if (!generateMetadata(output_path + "/metadata.txt")) {
            return false;
        }

        std::cout << "✓ SRTM data generation complete!" << std::endl;
        return true;
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<double> noise_{0.0, 1.0};

    bool generateElevationDatabase(const std::string& filename) {
        std::cout << "  Generating elevation database..." << std::endl;
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            std::cerr << "Failed to create " << filename << std::endl;
            return false;
        }

        // For demo, generate a smaller region: 10x10 degrees
        int lat_tiles = 10;
        int lon_tiles = 10;
        double base_lat = 30.0;  // Start latitude
        double base_lon = -120.0; // Start longitude

        // Write header
        file.write(reinterpret_cast<const char*>(&lat_tiles), sizeof(int));
        file.write(reinterpret_cast<const char*>(&lon_tiles), sizeof(int));
        file.write(reinterpret_cast<const char*>(&base_lat), sizeof(double));
        file.write(reinterpret_cast<const char*>(&base_lon), sizeof(double));
        double res = SRTM_RES_ARCSEC;
        file.write(reinterpret_cast<const char*>(&res), sizeof(double));

        // Generate elevation data for each tile
        int total_points = 0;
        for (int lat_idx = 0; lat_idx < lat_tiles; ++lat_idx) {
            for (int lon_idx = 0; lon_idx < lon_tiles; ++lon_idx) {
                double lat = base_lat + lat_idx;
                double lon = base_lon + lon_idx;

                // Generate tile data (simplified - just key points)
                std::vector<float> elevations = generateTileElevation(lat, lon);

                // Write tile data
                for (float elev : elevations) {
                    file.write(reinterpret_cast<const char*>(&elev), sizeof(float));
                }
                total_points += elevations.size();
            }
        }

        std::cout << "    ✓ Generated " << total_points << " elevation points" << std::endl;
        std::cout << "    ✓ Coverage: " << lat_tiles << "x" << lon_tiles << " degree tiles" << std::endl;
        return true;
    }

    std::vector<float> generateTileElevation(double base_lat, double base_lon) {
        // For efficiency, generate a reduced resolution tile (every 60 arc-seconds)
        int reduced_size = 61;  // 61x61 points per tile (1 arc-minute resolution)
        std::vector<float> elevations;
        elevations.reserve(reduced_size * reduced_size);

        for (int i = 0; i < reduced_size; ++i) {
            double lat = base_lat + i / 60.0;  // Convert to degrees

            for (int j = 0; j < reduced_size; ++j) {
                double lon = base_lon + j / 60.0;  // Convert to degrees

                // Generate realistic terrain elevation
                float elevation = generateElevation(lat, lon);
                elevations.push_back(elevation);
            }
        }

        return elevations;
    }

    float generateElevation(double lat, double lon) {
        // Base elevation (sea level to moderate altitude)
        float base = 500.0;

        // Continental shelf
        if (lon < -122.0) {  // Pacific Ocean
            base = -1000.0 - 3000.0 * std::exp(-0.01 * std::pow(lon + 125, 2));
        }

        // Mountain ranges
        float mountains = 0.0;

        // Sierra Nevada (simplified)
        if (lat > 35 && lat < 40 && lon > -121 && lon < -118) {
            double dist_to_center = std::sqrt(std::pow(lat - 37.5, 2) + std::pow(lon + 119.5, 2));
            mountains = 3000.0 * std::exp(-0.5 * dist_to_center);
        }

        // Rocky Mountains (simplified)
        if (lat > 38 && lat < 42 && lon > -110 && lon < -105) {
            double dist_to_center = std::sqrt(std::pow(lat - 40, 2) + std::pow(lon + 107, 2));
            mountains = 4000.0 * std::exp(-0.3 * dist_to_center);
        }

        // Valleys
        float valleys = 0.0;
        // Central Valley California
        if (lat > 35 && lat < 40 && lon > -122 && lon < -120) {
            valleys = -300.0;
        }

        // Add realistic terrain roughness
        float roughness = 50.0 * std::sin(lat * 20) * std::cos(lon * 20);
        roughness += 20.0 * std::sin(lat * 50) * std::cos(lon * 50);

        // Add noise
        float terrain_noise = 5.0 * noise_(rng_);

        float total = base + mountains + valleys + roughness + terrain_noise;

        // Clamp to realistic range
        if (total < -11000.0) total = -11000.0;  // Mariana Trench depth
        if (total > 8848.0) total = 8848.0;      // Everest height

        return total;
    }

    bool generateMetadata(const std::string& filename) {
        std::ofstream file(filename);
        if (!file) {
            std::cerr << "Failed to create " << filename << std::endl;
            return false;
        }

        file << "SRTM Synthetic Terrain Data\n";
        file << "============================\n\n";
        file << "Generated for GPS-Free Navigation System Testing\n\n";
        file << "Coverage:\n";
        file << "  Region: Western United States (demo)\n";
        file << "  Latitude: 30°N to 40°N\n";
        file << "  Longitude: 120°W to 110°W\n\n";
        file << "Resolution:\n";
        file << "  Spatial: 1 arc-minute (simplified from 1 arc-second)\n";
        file << "  Vertical: 1 meter\n\n";
        file << "Data Format:\n";
        file << "  Type: Binary float32\n";
        file << "  Units: Meters above EGM96 geoid\n";
        file << "  No-data value: " << NO_DATA_VALUE << "\n\n";
        file << "Features:\n";
        file << "  - Pacific Ocean depths\n";
        file << "  - Sierra Nevada mountains\n";
        file << "  - Rocky Mountains\n";
        file << "  - Central Valley\n";
        file << "  - Realistic terrain roughness\n";

        return true;
    }
};

int main(int argc, char* argv[]) {
    std::string output_path = "data/srtm";

    if (argc > 1) {
        output_path = argv[1];
    }

    SRTMGenerator generator;

    if (!generator.generateData(output_path)) {
        std::cerr << "Failed to generate SRTM data" << std::endl;
        return 1;
    }

    std::cout << "\nData saved to: " << output_path << "/" << std::endl;
    std::cout << "Ready for use with navigation system!" << std::endl;

    return 0;
}