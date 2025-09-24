/**
 * ONNX Runtime Predictor Implementation
 */

#include "onnx_predictor.h"
#include <algorithm>
#include <numeric>
#include <chrono>
#include <sstream>

namespace Navigation {

ONNXPredictor::ONNXPredictor(const std::string& model_path) {
    config_.model_path = model_path;
    {

        std::stringstream msg;

        msg << "ONNXPredictor initialized with model: " << model_path;

        LOG_INFO(msg.str());

    }
}

ONNXPredictor::ONNXPredictor(const ONNXConfig& config)
    : config_(config) {
    LOG_INFO("ONNXPredictor initialized with config");
}

ONNXPredictor::~ONNXPredictor() {
    LOG_DEBUG("ONNXPredictor destroyed");
}

bool ONNXPredictor::loadModel() {
    return loadModel(config_.model_path);
}

bool ONNXPredictor::loadModel(const std::string& model_path) {
#ifdef HAVE_ONNX
    try {
        {

            std::stringstream msg;

            msg << "Loading ONNX model from: " << model_path;

            LOG_INFO(msg.str());

        }

        // Initialize ONNX Runtime
        initializeONNX();

        // Create session
        session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), *session_options_);

        // Get model metadata
        getModelMetadata();

        // Load normalization parameters
        loadNormalizationParams();

        model_loaded_ = true;
        LOG_INFO("ONNX model loaded successfully");
        return true;

    } catch (const Ort::Exception& e) {
        {

            std::stringstream msg;

            msg << "ONNX Runtime error: " << e.what();

            LOG_ERROR(msg.str());

        }
        return false;
    } catch (const std::exception& e) {
        {

            std::stringstream msg;

            msg << "Error loading model: " << e.what();

            LOG_ERROR(msg.str());

        }
        return false;
    }
#else
    LOG_WARN("ONNX Runtime not available. Cannot load model.");
    return false;
#endif
}

std::pair<Vector3d, double> ONNXPredictor::predictBias(const std::vector<IMUData>& imu_buffer) {
    if (!model_loaded_) {
        LOG_WARN("Model not loaded. Returning zero bias.");
        return {Vector3d::Zero(), 100.0};
    }

    if (imu_buffer.size() < static_cast<size_t>(config_.input_window_size)) {
        {
            std::stringstream msg;
            msg << "Insufficient IMU data. Need " << config_.input_window_size
                 << " samples, got " << imu_buffer.size();
            LOG_WARN(msg.str());
        }
        return {Vector3d::Zero(), 100.0};
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    // Preprocess IMU data
    MatrixXd input = preprocessIMU(imu_buffer);
    normalizeInput(input);

    // Prepare input tensor
    std::vector<float> input_data = prepareInputTensor(input);

#ifdef HAVE_ONNX
    // Run inference
    std::vector<float> output = runInference(input_data);

    // Extract bias prediction and uncertainty
    Vector3d bias = extractBiasPrediction(output);
    double uncertainty = computeUncertainty(output);
#else
    Vector3d bias = Vector3d::Zero();
    double uncertainty = 100.0;
#endif

    // Track timing
    auto end_time = std::chrono::high_resolution_clock::now();
    last_inference_time_ = std::chrono::duration<double>(end_time - start_time).count();
    inference_count_++;

    {


        std::stringstream msg;


        msg << "ML bias prediction: " << bias.transpose();


        LOG_DEBUG(msg.str());


    }

    return {bias, uncertainty};
}

double ONNXPredictor::checkOOD(const std::vector<IMUData>& imu_buffer) {
    // Simple OOD detection based on input statistics
    MatrixXd input = preprocessIMU(imu_buffer);

    // Compute statistics
    VectorXd mean = input.colwise().mean();
    VectorXd std = ((input.rowwise() - mean.transpose()).array().square().colwise().sum()
                    / (input.rows() - 1)).sqrt();

    // Compare with training statistics
    double ood_score = 0.0;
    for (int i = 0; i < mean.size(); ++i) {
        double z_score = std::abs(mean(i) - input_mean_(i)) / (input_std_(i) + 1e-6);
        ood_score = std::max(ood_score, z_score);
    }

    return ood_score;
}

MatrixXd ONNXPredictor::preprocessIMU(const std::vector<IMUData>& imu_buffer) {
    // Convert IMU buffer to matrix format
    int n = std::min(static_cast<int>(imu_buffer.size()), config_.input_window_size);
    MatrixXd input(n, config_.input_features);

    for (int i = 0; i < n; ++i) {
        const auto& imu = imu_buffer[imu_buffer.size() - n + i];  // Use most recent data
        input(i, 0) = imu.accel.x();
        input(i, 1) = imu.accel.y();
        input(i, 2) = imu.accel.z();
        input(i, 3) = imu.gyro.x();
        input(i, 4) = imu.gyro.y();
        input(i, 5) = imu.gyro.z();
    }

    return input;
}

void ONNXPredictor::normalizeInput(MatrixXd& input) {
    // Apply z-normalization if statistics are available
    if (input_mean_.size() == config_.input_features &&
        input_std_.size() == config_.input_features) {

        for (int i = 0; i < input.rows(); ++i) {
            for (int j = 0; j < input.cols(); ++j) {
                input(i, j) = (input(i, j) - input_mean_(j)) / (input_std_(j) + 1e-6);
            }
        }
    }
}

#ifdef HAVE_ONNX
void ONNXPredictor::initializeONNX() {
    // Create environment
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ONNXPredictor");

    // Create session options
    session_options_ = std::make_unique<Ort::SessionOptions>();
    session_options_->SetIntraOpNumThreads(4);
    session_options_->SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Use GPU if available and requested
    if (config_.use_gpu) {
        // Note: GPU provider setup would go here
        // For now, using CPU provider
        LOG_INFO("Using CPU execution provider");
    }

    // Create memory info
    memory_info_ = std::make_unique<Ort::MemoryInfo>(
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
}

void ONNXPredictor::getModelMetadata() {
    Ort::AllocatorWithDefaultOptions allocator;

    // Get input info
    size_t num_input_nodes = session_->GetInputCount();
    input_node_names_.resize(num_input_nodes);
    input_node_dims_.clear();

    for (size_t i = 0; i < num_input_nodes; ++i) {
        char* input_name = session_->GetInputName(i, allocator);
        input_node_names_[i] = input_name;

        Ort::TypeInfo type_info = session_->GetInputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        input_node_dims_ = tensor_info.GetShape();

        LOG_DEBUG("Input node " << i << ": " << input_name
                  << " with shape [" << input_node_dims_[0] << ", "
                  << input_node_dims_[1] << ", " << input_node_dims_[2] << "]");
    }

    // Get output info
    size_t num_output_nodes = session_->GetOutputCount();
    output_node_names_.resize(num_output_nodes);
    output_node_dims_.clear();

    for (size_t i = 0; i < num_output_nodes; ++i) {
        char* output_name = session_->GetOutputName(i, allocator);
        output_node_names_[i] = output_name;

        Ort::TypeInfo type_info = session_->GetOutputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
        output_node_dims_ = tensor_info.GetShape();

        LOG_DEBUG("Output node " << i << ": " << output_name
                  << " with shape [" << output_node_dims_[0] << ", "
                  << output_node_dims_[1] << "]");
    }
}

std::vector<float> ONNXPredictor::runInference(const std::vector<float>& input_data) {
    // Create input tensor
    std::vector<int64_t> input_shape = {1, config_.input_window_size, config_.input_features};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        *memory_info_, const_cast<float*>(input_data.data()), input_data.size(),
        input_shape.data(), input_shape.size());

    // Run inference
    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_node_names_.data(), &input_tensor, 1,
        output_node_names_.data(), output_node_names_.size());

    // Extract output
    float* output_data = output_tensors.front().GetTensorMutableData<float>();
    size_t output_size = output_tensors.front().GetTensorTypeAndShapeInfo().GetElementCount();

    return std::vector<float>(output_data, output_data + output_size);
}
#endif

std::vector<float> ONNXPredictor::prepareInputTensor(const MatrixXd& input) {
    // Flatten matrix to vector in row-major order
    std::vector<float> tensor_data;
    tensor_data.reserve(input.rows() * input.cols());

    for (int i = 0; i < input.rows(); ++i) {
        for (int j = 0; j < input.cols(); ++j) {
            tensor_data.push_back(static_cast<float>(input(i, j)));
        }
    }

    // Pad if necessary
    while (tensor_data.size() < static_cast<size_t>(config_.input_window_size * config_.input_features)) {
        tensor_data.push_back(0.0f);
    }

    return tensor_data;
}

Vector3d ONNXPredictor::extractBiasPrediction(const std::vector<float>& output) {
    // Extract accelerometer bias (first 3 elements)
    if (output.size() >= 3) {
        return Vector3d(output[0], output[1], output[2]);
    }
    return Vector3d::Zero();
}

double ONNXPredictor::computeUncertainty(const std::vector<float>& output) {
    // If model outputs uncertainty (elements 6-8), use them
    // Otherwise, use a fixed uncertainty
    if (output.size() >= 9) {
        // RMS of uncertainty estimates
        double unc_x = output[6];
        double unc_y = output[7];
        double unc_z = output[8];
        return std::sqrt((unc_x * unc_x + unc_y * unc_y + unc_z * unc_z) / 3.0);
    }
    return 0.1;  // Default uncertainty
}

void ONNXPredictor::loadNormalizationParams() {
    // Load from file or use defaults
    // For now, using typical IMU values
    input_mean_ = VectorXd::Zero(config_.input_features);
    input_std_ = VectorXd::Ones(config_.input_features);

    // Typical IMU statistics (example values)
    input_mean_ << 0, 0, 9.81, 0, 0, 0;  // ax, ay, az, gx, gy, gz
    input_std_ << 0.5, 0.5, 0.5, 0.1, 0.1, 0.1;

    LOG_DEBUG("Loaded normalization parameters");
}

void ONNXPredictor::computeInputStatistics(const MatrixXd& data) {
    input_mean_ = data.colwise().mean();
    VectorXd variance = ((data.rowwise() - input_mean_.transpose()).array().square().colwise().sum()
                        / (data.rows() - 1));
    input_std_ = variance.array().sqrt();
}

/**
 * OOD Detector Implementation
 */
OODDetector::OODDetector() {
    LOG_DEBUG("OODDetector initialized");
}

void OODDetector::fit(const std::vector<std::vector<IMUData>>& training_data) {
    {

        std::stringstream msg;

        msg << "Fitting OOD detector on " << training_data.size() << " samples";

        LOG_INFO(msg.str());

    }

    // Extract features from training data
    MatrixXd features(training_data.size(), feature_dim_);

    for (size_t i = 0; i < training_data.size(); ++i) {
        features.row(i) = extractFeatures(training_data[i]);
    }

    // Compute mean and covariance
    mean_ = features.colwise().mean();
    MatrixXd centered = features.rowwise() - mean_.transpose();
    MatrixXd cov = (centered.transpose() * centered) / (features.rows() - 1);

    // Compute inverse covariance with regularization
    double epsilon = 1e-6;
    cov += MatrixXd::Identity(feature_dim_, feature_dim_) * epsilon;
    inv_cov_ = cov.inverse();

    LOG_INFO("OOD detector fitted successfully");
}

double OODDetector::computeOODScore(const std::vector<IMUData>& imu_buffer) {
    VectorXd features = extractFeatures(imu_buffer);
    return computeMahalanobisDistance(features);
}

bool OODDetector::isOOD(const std::vector<IMUData>& imu_buffer) {
    return computeOODScore(imu_buffer) > threshold_;
}

VectorXd OODDetector::extractFeatures(const std::vector<IMUData>& imu_buffer) {
    if (imu_buffer.empty()) {
        return VectorXd::Zero(feature_dim_);
    }

    // Convert to matrix
    MatrixXd data(imu_buffer.size(), 6);
    for (size_t i = 0; i < imu_buffer.size(); ++i) {
        const auto& imu = imu_buffer[i];
        data(i, 0) = imu.accel.x();
        data(i, 1) = imu.accel.y();
        data(i, 2) = imu.accel.z();
        data(i, 3) = imu.gyro.x();
        data(i, 4) = imu.gyro.y();
        data(i, 5) = imu.gyro.z();
    }

    return computeStatistics(data);
}

VectorXd OODDetector::computeStatistics(const MatrixXd& data) {
    VectorXd features(feature_dim_);

    // Mean (6 features)
    VectorXd mean = data.colwise().mean();
    features.segment(0, 6) = mean;

    // Standard deviation (6 features)
    VectorXd std = ((data.rowwise() - mean.transpose()).array().square().colwise().sum()
                    / (data.rows() - 1)).sqrt();
    features.segment(6, 6) = std;

    return features;
}

double OODDetector::computeMahalanobisDistance(const VectorXd& features) {
    if (mean_.size() == 0 || inv_cov_.size() == 0) {
        return 0.0;  // Not fitted yet
    }

    VectorXd diff = features - mean_;
    return std::sqrt((diff.transpose() * inv_cov_ * diff).value());
}

} // namespace Navigation