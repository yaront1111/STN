#pragma once

#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>

// Forward declarations for ONNX Runtime
namespace Ort {
    class Session;
    class Env;
    class SessionOptions;
    class AllocatorWithDefaultOptions;
    class MemoryInfo;
    class Value;
}

namespace ml {

/**
 * ONNX Runtime inference engine for real-time model execution
 * Supports IMU correction, gravity enhancement, and trajectory prediction
 */
class ONNXInference {
public:
    struct Config {
        std::string model_path;
        bool use_gpu = false;
        int batch_size = 1;
        int num_threads = 4;
        bool enable_profiling = false;
    };

    ONNXInference(const Config& config);
    ~ONNXInference();

    // Initialize the ONNX runtime and load model
    bool initialize();

    // IMU bias/scale correction
    struct IMUCorrection {
        Eigen::Vector3d acc_bias;
        Eigen::Vector3d gyro_bias;
        Eigen::Vector3d acc_scale;
        Eigen::Vector3d gyro_scale;
        float confidence;
    };

    IMUCorrection correctIMU(
        const std::vector<Eigen::Vector3d>& acc_history,
        const std::vector<Eigen::Vector3d>& gyro_history,
        const std::vector<double>& timestamps
    );

    // Gravity field enhancement
    struct GravityEnhancement {
        Eigen::MatrixXd enhanced_field;  // High-res gravity tensor field
        Eigen::Matrix3d local_gradient;  // Enhanced local gradient
        float confidence;
        float resolution_m;  // Grid resolution in meters
    };

    GravityEnhancement enhanceGravityField(
        const Eigen::MatrixXd& low_res_field,
        const Eigen::Vector3d& current_position
    );

    // Trajectory prediction
    struct TrajectoryPrediction {
        std::vector<Eigen::Vector3d> positions;
        std::vector<Eigen::Vector3d> velocities;
        std::vector<Eigen::Quaterniond> attitudes;
        std::vector<float> uncertainties;
        float confidence;
    };

    TrajectoryPrediction predictTrajectory(
        const Eigen::Vector3d& current_pos,
        const Eigen::Vector3d& current_vel,
        const Eigen::Quaterniond& current_att,
        const std::vector<Eigen::Vector3d>& acc_history,
        const std::vector<Eigen::Vector3d>& gyro_history,
        double prediction_horizon_s
    );

    // Model management
    bool loadModel(const std::string& model_path);
    bool isModelLoaded() const { return model_loaded_; }
    std::string getModelVersion() const { return model_version_; }

    // Performance monitoring
    struct PerformanceStats {
        double avg_inference_time_ms;
        double max_inference_time_ms;
        size_t total_inferences;
        size_t failed_inferences;
    };

    PerformanceStats getPerformanceStats() const { return stats_; }

private:
    Config config_;
    bool model_loaded_;
    std::string model_version_;
    PerformanceStats stats_;

    // ONNX Runtime components
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::SessionOptions> session_options_;
    std::unique_ptr<Ort::AllocatorWithDefaultOptions> allocator_;
    std::unique_ptr<Ort::MemoryInfo> memory_info_;

    // Input/output tensor info
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<std::vector<int64_t>> input_shapes_;
    std::vector<std::vector<int64_t>> output_shapes_;

    // Helper methods
    std::vector<float> eigenToVector(const Eigen::MatrixXd& matrix);
    Eigen::MatrixXd vectorToEigen(const std::vector<float>& vec, int rows, int cols);
    void updateStats(double inference_time_ms, bool success);
};

/**
 * Model ensemble for robust predictions
 * Combines multiple models with uncertainty quantification
 */
class ModelEnsemble {
public:
    ModelEnsemble();

    // Add a model to the ensemble
    void addModel(std::unique_ptr<ONNXInference> model, double weight = 1.0);

    // Get ensemble prediction with uncertainty
    ONNXInference::IMUCorrection getIMUCorrection(
        const std::vector<Eigen::Vector3d>& acc_history,
        const std::vector<Eigen::Vector3d>& gyro_history,
        const std::vector<double>& timestamps
    );

    ONNXInference::GravityEnhancement getGravityEnhancement(
        const Eigen::MatrixXd& low_res_field,
        const Eigen::Vector3d& current_position
    );

    // Adaptive weighting based on recent performance
    void updateWeights(const std::vector<double>& errors);

private:
    std::vector<std::unique_ptr<ONNXInference>> models_;
    std::vector<double> weights_;

    // Combine predictions using weighted average or voting
    template<typename T>
    T combineePredictions(const std::vector<T>& predictions, const std::vector<double>& weights);
};

/**
 * Online learning adapter for continuous improvement
 */
class OnlineLearner {
public:
    OnlineLearner(ONNXInference* base_model);

    // Update model with new training samples
    void addTrainingSample(
        const Eigen::Vector3d& true_position,
        const Eigen::Vector3d& estimated_position,
        const std::vector<Eigen::Vector3d>& imu_history
    );

    // Trigger model retraining if needed
    bool shouldRetrain() const;
    void retrainModel();

    // Save collected training data
    void saveTrainingData(const std::string& path);

private:
    ONNXInference* base_model_;
    std::vector<std::tuple<Eigen::Vector3d, Eigen::Vector3d, std::vector<Eigen::Vector3d>>> training_buffer_;
    size_t samples_since_retrain_;
    double cumulative_error_;
};

} // namespace ml