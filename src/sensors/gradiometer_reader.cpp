/**
 * Gradiometer Data Reader Implementation
 */

#include "gradiometer_reader.h"
#include "../utils/math_utils.h"
#include <sstream>
#include <cmath>
#include <algorithm>

namespace Navigation {

GradiometerReader::GradiometerReader(const GradiometerConfig& config) : config_(config) {
    {

        std::stringstream msg;

        msg << "Gradiometer Reader initialized for file: " << config.csv_path;

        LOG_INFO(msg.str());

    }
}

GradiometerReader::~GradiometerReader() {
    if (file_.is_open()) {
        close();
    }
}

bool GradiometerReader::open() {
    if (file_.is_open()) {
        return true;
    }
    
    file_.open(config_.csv_path);
    if (!file_.is_open()) {
        {

            std::stringstream msg;

            msg << "Failed to open gradiometer file: " << config_.csv_path;

            LOG_ERROR(msg.str());

        }
        return false;
    }
    
    if (config_.has_header) {
        std::string header;
        std::getline(file_, header);
        {

            std::stringstream msg;

            msg << "Gradiometer CSV header: " << header;

            LOG_DEBUG(msg.str());

        }
    }
    
    stats_ = Stats();
    
    LOG_INFO("Gradiometer file opened successfully");
    return true;
}

void GradiometerReader::close() {
    if (file_.is_open()) {
        file_.close();
        LOG_INFO("Gradiometer file closed");
        printStatistics();
    }
}

bool GradiometerReader::readNext(GradiometerData& data) {
    if (!file_.is_open()) {
        LOG_ERROR("Gradiometer file not open");
        return false;
    }
    
    std::string line;
    while (std::getline(file_, line)) {
        if (line.empty()) continue;
        
        if (parseLine(line, data)) {
            // Apply mounting correction
            applyMountingCorrection(data);
            
            // Reconstruct full tensor from STF components
            reconstructFullTensor(data);
            
            // Compute invariants
            computeInvariants(data);
            
            if (validateData(data)) {
                // Update statistics
                stats_.total_samples++;
                stats_.valid_samples++;
                
                double gradient_norm = data.gradient_tensor.norm();
                stats_.min_gradient = std::min(stats_.min_gradient, gradient_norm);
                stats_.max_gradient = std::max(stats_.max_gradient, gradient_norm);
                
                stats_.avg_confidence = (stats_.avg_confidence * (stats_.valid_samples - 1) + 
                                        data.confidence) / stats_.valid_samples;
                
                if (data.confidence < config_.min_confidence) {
                    stats_.low_confidence_samples++;
                }
                
                return true;
            } else {
                stats_.total_samples++;
            }
        }
    }
    
    return false;
}

bool GradiometerReader::readAll(std::vector<GradiometerData>& data) {
    data.clear();
    
    GradiometerData sample;
    while (readNext(sample)) {
        data.push_back(sample);
    }
    
    {

    
        std::stringstream msg;

    
        msg << "Read " << data.size() << " gradiometer samples";

    
        LOG_INFO(msg.str());

    
    }
    return !data.empty();
}

bool GradiometerReader::parseLine(const std::string& line, GradiometerData& data) {
    std::vector<std::string> tokens = splitString(line, config_.delimiter);
    
    int max_col = std::max({config_.col_timestamp,
                           config_.col_txx, config_.col_txy, config_.col_txz,
                           config_.col_tyy, config_.col_tyz});
    
    if (tokens.size() <= static_cast<size_t>(max_col)) {
        LOG_ERROR("Insufficient columns in gradiometer data");
        return false;
    }
    
    try {
        data.timestamp = std::stod(tokens[config_.col_timestamp]) * config_.time_scale;
        
        // Parse 5 independent STF components
        data.gradient_tensor(0) = std::stod(tokens[config_.col_txx]) * config_.gradient_scale;
        data.gradient_tensor(1) = std::stod(tokens[config_.col_txy]) * config_.gradient_scale;
        data.gradient_tensor(2) = std::stod(tokens[config_.col_txz]) * config_.gradient_scale;
        data.gradient_tensor(3) = std::stod(tokens[config_.col_tyy]) * config_.gradient_scale;
        data.gradient_tensor(4) = std::stod(tokens[config_.col_tyz]) * config_.gradient_scale;
        
        // Convert units if needed
        if (!config_.in_eotvos) {
            // Convert from SI (s^-2) to Eötvös
            data.gradient_tensor *= 1e9;
        }
        
        // Parse optional fields
        if (config_.col_temperature >= 0 && config_.col_temperature < static_cast<int>(tokens.size())) {
            data.temperature = std::stod(tokens[config_.col_temperature]);
        } else {
            data.temperature = 25.0;
        }
        
        if (config_.col_confidence >= 0 && config_.col_confidence < static_cast<int>(tokens.size())) {
            data.confidence = std::stod(tokens[config_.col_confidence]);
        } else {
            data.confidence = 1.0;
        }
        
        data.valid = true;
        
    } catch (const std::exception& e) {
        {

            std::stringstream msg;

            msg << "Error parsing gradiometer data: " << e.what();

            LOG_ERROR(msg.str());

        }
        return false;
    }
    
    return true;
}

void GradiometerReader::reconstructFullTensor(GradiometerData& data) {
    // Reconstruct full 3x3 symmetric trace-free tensor from 6-component tensor vector
    // tensor stores: [Txx, Tyy, Tzz, Txy, Txz, Tyz]

    data.gradient_tensor = Matrix3d::Zero();

    // Diagonal elements
    data.gradient_tensor(0, 0) = data.tensor(0);  // T_xx
    data.gradient_tensor(1, 1) = data.tensor(1);  // T_yy
    data.gradient_tensor(2, 2) = data.tensor(2);  // T_zz

    // Off-diagonal elements (symmetric)
    data.gradient_tensor(0, 1) = data.tensor(3);  // T_xy
    data.gradient_tensor(1, 0) = data.tensor(3);  // T_yx = T_xy

    data.gradient_tensor(0, 2) = data.tensor(4);  // T_xz
    data.gradient_tensor(2, 0) = data.tensor(4);  // T_zx = T_xz

    data.gradient_tensor(1, 2) = data.tensor(5);  // T_yz
    data.gradient_tensor(2, 1) = data.tensor(5);  // T_zy = T_yz
}

bool GradiometerReader::validateTraceFree(const GradiometerData& data, double tolerance) {
    double trace = data.gradient_tensor.trace();
    return std::abs(trace) < tolerance;
}

void GradiometerReader::computeInvariants(GradiometerData& data) {
    // Compute tensor invariants
    data.trace = data.gradient_tensor.trace();
    data.determinant = data.gradient_tensor.determinant();
    data.frobenius_norm = data.gradient_tensor.norm();
}

void GradiometerReader::applyMountingCorrection(GradiometerData& data) {
    // If sensor is not aligned with vehicle frame, rotate tensor
    if (!config_.mounting_rotation.isIdentity(1e-6)) {
        // First reconstruct full tensor
        reconstructFullTensor(data);
        
        // Rotate tensor: T' = R * T * R^T
        data.gradient_tensor = config_.mounting_rotation * data.gradient_tensor *
                          config_.mounting_rotation.transpose();
        
        // Convert back to STF components - need to convert 5-element to 6-element
        Eigen::Matrix<double, 5, 1> stf5 = GravityTensor::tensorToSTF(data.gradient_tensor);
        // The 6-element tensor has [Txx, Tyy, Tzz, Txy, Txz, Tyz]
        // The 5-element STF has [Txx, Txy, Txz, Tyy, Tyz] (Tzz is derived from trace-free)
        data.tensor(0) = stf5(0);  // Txx
        data.tensor(1) = stf5(3);  // Tyy
        data.tensor(2) = -(stf5(0) + stf5(3));  // Tzz = -(Txx + Tyy) for trace-free
        data.tensor(3) = stf5(1);  // Txy
        data.tensor(4) = stf5(2);  // Txz
        data.tensor(5) = stf5(4);  // Tyz
    }
}

bool GradiometerReader::validateData(GradiometerData& data) {
    // Check for NaN or Inf
    if (!data.gradient_tensor.allFinite()) {
        LOG_WARN("Gradiometer data contains NaN or Inf");
        data.valid = false;
        return false;
    }
    
    // Check gradient magnitude
    double gradient_norm = data.gradient_tensor.norm();
    if (gradient_norm > config_.max_gradient) {
        {

            std::stringstream msg;

            msg << "Gradient exceeds limit: " << gradient_norm << " E";

            LOG_WARN(msg.str());

        }
        data.is_saturated = true;
    }
    
    // Validate trace-free condition
    if (!validateTraceFree(data, config_.trace_tolerance)) {
        {

            std::stringstream msg;

            msg << "Tensor not trace-free: trace = " << data.trace << " E";

            LOG_WARN(msg.str());

        }
        stats_.trace_violations++;
    }
    
    // Check confidence
    if (data.confidence < config_.min_confidence) {
        {

            std::stringstream msg;

            msg << "Low confidence measurement: " << data.confidence;

            LOG_WARN(msg.str());

        }
    }
    
    return true;  // Still return true with warnings
}

std::vector<std::string> GradiometerReader::splitString(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        token.erase(0, token.find_first_not_of(" \t"));
        token.erase(token.find_last_not_of(" \t") + 1);
        tokens.push_back(token);
    }
    
    return tokens;
}

void GradiometerReader::printStatistics() const {
    LOG_INFO("=== Gradiometer Reader Statistics ===");
    {

        std::stringstream msg;

        msg << "Total samples: " << stats_.total_samples;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Valid samples: " << stats_.valid_samples;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Low confidence samples: " << stats_.low_confidence_samples;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Trace violations: " << stats_.trace_violations;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Gradient range: " << stats_.min_gradient << " - " << stats_.max_gradient << " E";

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Average confidence: " << stats_.avg_confidence;

        LOG_INFO(msg.str());

    }
}

} // namespace Navigation