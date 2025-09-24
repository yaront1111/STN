/**
 * ML Model Placeholder Generator
 * Creates minimal ONNX model and training stats for testing
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <filesystem>
#include <cstring>

namespace fs = std::filesystem;

// Simplified ONNX file structure (minimal valid ONNX)
class ONNXGenerator {
public:
    bool generateModel(const std::string& output_path) {
        std::cout << "=== ML Model Generator ===" << std::endl;
        std::cout << "Creating placeholder ONNX model and training stats..." << std::endl;

        // Create output directory
        fs::create_directories(output_path);

        // Generate minimal ONNX model
        if (!generateONNXFile(output_path + "/bias_predictor.onnx")) {
            return false;
        }

        // Generate training statistics
        if (!generateTrainingStats(output_path + "/training_stats.npz")) {
            return false;
        }

        std::cout << "✓ ML model generation complete!" << std::endl;
        return true;
    }

private:
    bool generateONNXFile(const std::string& filename) {
        std::cout << "  Generating ONNX model..." << std::endl;
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            std::cerr << "Failed to create " << filename << std::endl;
            return false;
        }

        // Minimal ONNX header (simplified - not a real ONNX file)
        // This is just a placeholder that the system can detect
        const char magic[] = "ONNX_PLACEHOLDER";
        file.write(magic, sizeof(magic));

        // Model metadata
        int32_t version = 1;
        int32_t input_dim = 7;   // ax, ay, az, gx, gy, gz, temp
        int32_t output_dim = 6;  // accel_bias[3], gyro_bias[3]
        int32_t hidden_dim = 64;

        file.write(reinterpret_cast<const char*>(&version), sizeof(version));
        file.write(reinterpret_cast<const char*>(&input_dim), sizeof(input_dim));
        file.write(reinterpret_cast<const char*>(&output_dim), sizeof(output_dim));
        file.write(reinterpret_cast<const char*>(&hidden_dim), sizeof(hidden_dim));

        // Placeholder weights (random values)
        std::vector<float> weights(input_dim * hidden_dim, 0.01f);
        file.write(reinterpret_cast<const char*>(weights.data()),
                  weights.size() * sizeof(float));

        std::cout << "    ✓ Created placeholder ONNX model" << std::endl;
        return true;
    }

    bool generateTrainingStats(const std::string& filename) {
        std::cout << "  Generating training statistics..." << std::endl;
        std::ofstream file(filename, std::ios::binary);
        if (!file) {
            std::cerr << "Failed to create " << filename << std::endl;
            return false;
        }

        // Simplified NPZ format (just markers for our system)
        const char magic[] = "STATS_PLACEHOLDER";
        file.write(magic, sizeof(magic));

        // Statistics for OOD detection
        int32_t num_features = 7;
        file.write(reinterpret_cast<const char*>(&num_features), sizeof(num_features));

        // Mean values
        std::vector<float> mean = {0.0f, 0.0f, -9.8f, 0.0f, 0.0f, 0.0f, 25.0f};
        file.write(reinterpret_cast<const char*>(mean.data()),
                  mean.size() * sizeof(float));

        // Standard deviation
        std::vector<float> std = {2.0f, 2.0f, 2.0f, 0.1f, 0.1f, 0.1f, 5.0f};
        file.write(reinterpret_cast<const char*>(std.data()),
                  std.size() * sizeof(float));

        // Covariance matrix (diagonal only for simplicity)
        std::vector<float> cov_diag(num_features);
        for (int i = 0; i < num_features; ++i) {
            cov_diag[i] = std[i] * std[i];
        }
        file.write(reinterpret_cast<const char*>(cov_diag.data()),
                  cov_diag.size() * sizeof(float));

        std::cout << "    ✓ Created training statistics" << std::endl;
        return true;
    }
};

int main(int argc, char* argv[]) {
    std::string output_path = "ml/models";

    if (argc > 1) {
        output_path = argv[1];
    }

    ONNXGenerator generator;

    if (!generator.generateModel(output_path)) {
        std::cerr << "Failed to generate ML model" << std::endl;
        return 1;
    }

    std::cout << "\nModels saved to: " << output_path << "/" << std::endl;
    std::cout << "Ready for navigation system!" << std::endl;

    return 0;
}