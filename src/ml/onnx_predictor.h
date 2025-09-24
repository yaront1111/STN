/**
 * ONNX Runtime Predictor for ML Bias Estimation
 * Provides real-time bias prediction from IMU data
 */

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <deque>
#include <Eigen/Dense>

#ifdef HAVE_ONNX
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>
#endif

#include "../sensors/imu_reader.h"
#include "../utils/logger.h"

namespace Navigation {

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

/**
 * ML Predictor Configuration
 */
struct ONNXConfig {
    std::string model_path;
    int input_window_size = 1000;      // 10 seconds at 100Hz
    int input_features = 6;            // ax, ay, az, gx, gy, gz
    int output_features = 6;           // bias_ax, bias_ay, bias_az, bias_gx, bias_gy, bias_gz
    bool use_gpu = false;
    int batch_size = 1;
    float confidence_threshold = 0.8f;
};

/**
 * ONNX Predictor for bias estimation
 */
class ONNXPredictor {
private:
#ifdef HAVE_ONNX
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::unique_ptr<Ort::MemoryInfo> memory_info_;

    // Input/Output node names
    std::vector<const char*> input_node_names_;
    std::vector<const char*> output_node_names_;
    std::vector<int64_t> input_node_dims_;
    std::vector<int64_t> output_node_dims_;
#endif

    ONNXConfig config_;
    bool model_loaded_ = false;

    // Data preprocessing
    std::deque<IMUData> imu_buffer_;
    VectorXd input_mean_;
    VectorXd input_std_;

    // Performance metrics
    double last_inference_time_ = 0;
    uint64_t inference_count_ = 0;

public:
    ONNXPredictor(const std::string& model_path);
    ONNXPredictor(const ONNXConfig& config);
    ~ONNXPredictor();

    // Model management
    bool loadModel();
    bool loadModel(const std::string& model_path);
    bool isLoaded() const { return model_loaded_; }

    // Main prediction interface
    std::pair<Vector3d, double> predictBias(const std::vector<IMUData>& imu_buffer);

    // Out-of-distribution detection
    double checkOOD(const std::vector<IMUData>& imu_buffer);

    // Preprocessing
    MatrixXd preprocessIMU(const std::vector<IMUData>& imu_buffer);
    void normalizeInput(MatrixXd& input);

private:
#ifdef HAVE_ONNX
    // ONNX specific methods
    void initializeONNX();
    void getModelMetadata();
    std::vector<float> runInference(const std::vector<float>& input_data);
#endif

    // Data preparation
    std::vector<float> prepareInputTensor(const MatrixXd& input);
    Vector3d extractBiasPrediction(const std::vector<float>& output);
    double computeUncertainty(const std::vector<float>& output);

    // Statistics for normalization
    void loadNormalizationParams();
    void computeInputStatistics(const MatrixXd& data);
};

/**
 * Out-of-Distribution Detector
 */
class OODDetector {
private:
    // Mahalanobis distance parameters
    VectorXd mean_;
    MatrixXd inv_cov_;
    double threshold_ = 3.0;  // Chi-squared threshold

    // Feature extraction
    int feature_dim_ = 12;  // Statistical features from IMU

public:
    OODDetector();

    // Train detector on in-distribution data
    void fit(const std::vector<std::vector<IMUData>>& training_data);

    // Check if sample is OOD
    double computeOODScore(const std::vector<IMUData>& imu_buffer);
    bool isOOD(const std::vector<IMUData>& imu_buffer);

    // Feature extraction
    VectorXd extractFeatures(const std::vector<IMUData>& imu_buffer);

private:
    // Statistical features
    VectorXd computeStatistics(const MatrixXd& data);
    double computeMahalanobisDistance(const VectorXd& features);
};

/**
 * Dummy implementation when ONNX is not available
 */
#ifndef HAVE_ONNX
class ONNXPredictorDummy : public ONNXPredictor {
public:
    ONNXPredictorDummy(const std::string& model_path) : ONNXPredictor(model_path) {
        LOG_WARN("ONNX Runtime not available. Using dummy predictor.");
    }

    bool loadModel() {
        LOG_WARN("ONNX Runtime not available. Cannot load model.");
        return false;
    }

    std::pair<Vector3d, double> predictBias(const std::vector<IMUData>& imu_buffer) {
        // Return zero bias with high uncertainty
        return {Vector3d::Zero(), 100.0};
    }

    double checkOOD(const std::vector<IMUData>& imu_buffer) {
        return 0.0;  // Always in-distribution (no checking)
    }
};
#endif

} // namespace Navigation