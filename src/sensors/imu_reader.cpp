/**
 * IMU Data Reader Implementation
 * Reads real CSV data - no mocks
 */

#include "imu_reader.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>

namespace Navigation {

IMUReader::IMUReader(const IMUConfig& config) : config_(config) {
    {

        std::stringstream msg;

        msg << "IMU Reader initialized for file: " << config.csv_path;

        LOG_INFO(msg.str());

    }
}

IMUReader::~IMUReader() {
    if (file_.is_open()) {
        close();
    }
}

bool IMUReader::open() {
    if (file_.is_open()) {
        LOG_WARN("IMU file already open");
        return true;
    }
    
    file_.open(config_.csv_path);
    if (!file_.is_open()) {
        {

            std::stringstream msg;

            msg << "Failed to open IMU file: " << config_.csv_path;

            LOG_ERROR(msg.str());

        }
        return false;
    }
    
    // Skip header if present
    if (config_.has_header) {
        std::string header;
        std::getline(file_, header);
        {

            std::stringstream msg;

            msg << "IMU CSV header: " << header;

            LOG_DEBUG(msg.str());

        }
    }
    
    // Reset statistics
    stats_ = Stats();
    has_prev_ = false;
    
    LOG_INFO("IMU file opened successfully");
    return true;
}

void IMUReader::close() {
    if (file_.is_open()) {
        file_.close();
        LOG_INFO("IMU file closed");
        printStatistics();
    }
}

bool IMUReader::readNext(IMUData& data) {
    if (!file_.is_open()) {
        LOG_ERROR("IMU file not open");
        return false;
    }
    
    std::string line;
    while (std::getline(file_, line)) {
        // Skip empty lines
        if (line.empty()) continue;
        
        // Parse line
        if (parseLine(line, data)) {
            // Validate data
            if (validateData(data)) {
                // Apply corrections
                applyMountingCorrection(data);
                
                // Update statistics
                stats_.total_samples++;
                stats_.valid_samples++;
                
                if (has_prev_) {
                    double dt = data.timestamp - prev_sample_.timestamp;
                    stats_.min_dt = std::min(stats_.min_dt, dt);
                    stats_.max_dt = std::max(stats_.max_dt, dt);
                    stats_.avg_dt = (stats_.avg_dt * (stats_.valid_samples - 1) + dt) / stats_.valid_samples;
                }
                
                prev_sample_ = data;
                has_prev_ = true;
                
                return true;
            } else {
                stats_.total_samples++;
                logInvalidData(data, "Validation failed");
            }
        } else {
            {

                std::stringstream msg;

                msg << "Failed to parse IMU line: " << line;

                LOG_WARN(msg.str());

            }
        }
    }
    
    return false;  // End of file
}

bool IMUReader::readAll(std::vector<IMUData>& data) {
    data.clear();
    
    IMUData sample;
    while (readNext(sample)) {
        data.push_back(sample);
    }
    
    {

    
        std::stringstream msg;

    
        msg << "Read " << data.size() << " IMU samples";

    
        LOG_INFO(msg.str());

    
    }
    return !data.empty();
}

bool IMUReader::readTimeRange(double start_time, double end_time, std::vector<IMUData>& data) {
    data.clear();
    
    IMUData sample;
    while (readNext(sample)) {
        if (sample.timestamp >= start_time && sample.timestamp <= end_time) {
            data.push_back(sample);
        }
        if (sample.timestamp > end_time) {
            break;  // Past the desired range
        }
    }
    
    {

    
        std::stringstream msg;

    
        msg << "Read " << data.size() << " IMU samples in range";

    
        LOG_INFO(msg.str());

    
    }
    return !data.empty();
}

bool IMUReader::bufferNextChunk(size_t chunk_size) {
    buffer_.clear();
    buffer_.reserve(chunk_size);
    current_index_ = 0;
    
    IMUData sample;
    for (size_t i = 0; i < chunk_size && readNext(sample); ++i) {
        buffer_.push_back(sample);
    }
    
    return !buffer_.empty();
}

bool IMUReader::getBuffered(IMUData& data) {
    if (current_index_ >= buffer_.size()) {
        // Need to read more data
        if (!bufferNextChunk()) {
            return false;  // No more data
        }
    }
    
    data = buffer_[current_index_++];
    return true;
}

bool IMUReader::parseLine(const std::string& line, IMUData& data) {
    std::vector<std::string> tokens = splitString(line, config_.delimiter);
    
    // Check minimum required columns
    int max_col = std::max({config_.col_timestamp,
                           config_.col_accel_x, config_.col_accel_y, config_.col_accel_z,
                           config_.col_gyro_x, config_.col_gyro_y, config_.col_gyro_z});
    
    if (tokens.size() <= static_cast<size_t>(max_col)) {
        {

            std::stringstream msg;

            msg << "Insufficient columns in IMU data: " << tokens.size() << " <= " << max_col;

            LOG_ERROR(msg.str());

        }
        return false;
    }
    
    try {
        // Parse timestamp
        data.timestamp = std::stod(tokens[config_.col_timestamp]) * config_.time_scale;
        
        // Parse acceleration
        data.accel.x() = std::stod(tokens[config_.col_accel_x]) * config_.accel_scale;
        data.accel.y() = std::stod(tokens[config_.col_accel_y]) * config_.accel_scale;
        data.accel.z() = std::stod(tokens[config_.col_accel_z]) * config_.accel_scale;
        
        // Parse gyroscope
        data.gyro.x() = std::stod(tokens[config_.col_gyro_x]) * config_.gyro_scale;
        data.gyro.y() = std::stod(tokens[config_.col_gyro_y]) * config_.gyro_scale;
        data.gyro.z() = std::stod(tokens[config_.col_gyro_z]) * config_.gyro_scale;
        
        // Convert degrees to radians if needed
        if (config_.gyro_in_degrees) {
            data.gyro *= M_PI / 180.0;
        }
        
        // Parse temperature if available
        if (config_.col_temperature < static_cast<int>(tokens.size())) {
            data.temperature = std::stod(tokens[config_.col_temperature]);
        } else {
            data.temperature = 25.0;  // Default room temperature
        }
        
        // Parse status flags if available (skip for now, not in IMUData struct)
        
        data.valid = true;
        
    } catch (const std::exception& e) {
        {

            std::stringstream msg;

            msg << "Error parsing IMU data: " << e.what();

            LOG_ERROR(msg.str());

        }
        return false;
    }
    
    return true;
}

std::vector<std::string> IMUReader::splitString(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t"));
        token.erase(token.find_last_not_of(" \t") + 1);
        tokens.push_back(token);
    }
    
    return tokens;
}

bool IMUReader::validateData(IMUData& data) {
    // Check for NaN or Inf
    if (!data.accel.allFinite() || !data.gyro.allFinite()) {
        LOG_WARN("IMU data contains NaN or Inf");
        data.valid = false;
        return false;
    }
    
    // Check acceleration limits
    if (data.accel.norm() > config_.max_accel) {
        {

            std::stringstream msg;

            msg << "Acceleration exceeds limit: " << data.accel.norm() << " > " << config_.max_accel;

            LOG_WARN(msg.str());

        }
        data.accel_saturated = true;
        stats_.saturated_samples++;
    }
    
    // Check gyroscope limits
    if (data.gyro.norm() > config_.max_gyro) {
        {

            std::stringstream msg;

            msg << "Gyroscope exceeds limit: " << data.gyro.norm() << " > " << config_.max_gyro;

            LOG_WARN(msg.str());

        }
        data.gyro_saturated = true;
        stats_.saturated_samples++;
    }
    
    // Check temperature
    if (data.temperature < config_.min_temperature || data.temperature > config_.max_temperature) {
        {

            std::stringstream msg;

            msg << "Temperature out of range: " << data.temperature;

            LOG_WARN(msg.str());

        }
        stats_.temperature_warnings++;
    }
    
    // Check saturation
    checkSaturation(data);
    
    return true;  // Still return true even with warnings
}

bool IMUReader::checkSaturation(IMUData& data) const {
    // Check if any axis is at maximum value (likely saturated)
    const double SATURATION_THRESHOLD = 0.95;
    
    bool saturated = false;
    
    for (int i = 0; i < 3; ++i) {
        if (std::abs(data.accel(i)) > config_.max_accel * SATURATION_THRESHOLD) {
            data.accel_saturated = true;
            saturated = true;
        }
        if (std::abs(data.gyro(i)) > config_.max_gyro * SATURATION_THRESHOLD) {
            data.gyro_saturated = true;
            saturated = true;
        }
    }
    
    return saturated;
}

void IMUReader::applyMountingCorrection(IMUData& data) {
    // Apply mounting rotation
    data.accel = config_.mounting_rotation * data.accel;
    data.gyro = config_.mounting_rotation * data.gyro;
    
    // Apply lever arm correction if significant
    if (config_.lever_arm.norm() > 0.01) {  // 1cm threshold
        applyLeverArm(data, data.gyro);
    }
}

void IMUReader::applyLeverArm(IMUData& data, const Vector3d& angular_rate) {
    // Lever arm induces additional acceleration
    // a_corrected = a_measured - omega x (omega x r) - alpha x r
    
    Vector3d omega = angular_rate;
    Vector3d r = config_.lever_arm;
    
    // Centripetal acceleration: omega x (omega x r)
    Vector3d centripetal = omega.cross(omega.cross(r));
    
    // For angular acceleration, we'd need derivative of omega
    // This requires previous sample - simplified here
    
    data.accel -= centripetal;
}

void IMUReader::logInvalidData(const IMUData& data, const std::string& reason) {
    {
        std::stringstream msg;
        msg << "Invalid IMU data at t=" << data.timestamp << ": " << reason
             << " accel=[" << data.accel.transpose() << "]"
             << " gyro=[" << data.gyro.transpose() << "]";
        LOG_DEBUG(msg.str());
    }
}

void IMUReader::printStatistics() const {
    LOG_INFO("=== IMU Reader Statistics ===");
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

        msg << "Saturated samples: " << stats_.saturated_samples;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Temperature warnings: " << stats_.temperature_warnings;

        LOG_INFO(msg.str());

    }
    
    if (stats_.valid_samples > 1) {
        {

            std::stringstream msg;

            msg << "Sample rate: " << 1.0 / stats_.avg_dt << " Hz";

            LOG_INFO(msg.str());

        }
        {

            std::stringstream msg;

            msg << "Min dt: " << stats_.min_dt << " s";

            LOG_INFO(msg.str());

        }
        {

            std::stringstream msg;

            msg << "Max dt: " << stats_.max_dt << " s";

            LOG_INFO(msg.str());

        }
        {

            std::stringstream msg;

            msg << "Avg dt: " << stats_.avg_dt << " s";

            LOG_INFO(msg.str());

        }
    }
}

IMUData IMUReader::interpolate(const IMUData& d1, const IMUData& d2, double t) {
    // Linear interpolation between two IMU samples
    double alpha = (t - d1.timestamp) / (d2.timestamp - d1.timestamp);
    alpha = std::max(0.0, std::min(1.0, alpha));  // Clamp to [0,1]
    
    IMUData result;
    result.timestamp = t;
    result.accel = (1 - alpha) * d1.accel + alpha * d2.accel;
    result.gyro = (1 - alpha) * d1.gyro + alpha * d2.gyro;
    result.temperature = (1 - alpha) * d1.temperature + alpha * d2.temperature;
    result.valid = d1.valid && d2.valid;
    
    return result;
}

double IMUReader::computeSampleRate(const std::vector<IMUData>& data) {
    if (data.size() < 2) return 0.0;
    
    double total_dt = 0;
    for (size_t i = 1; i < data.size(); ++i) {
        total_dt += data[i].timestamp - data[i-1].timestamp;
    }
    
    double avg_dt = total_dt / (data.size() - 1);
    return 1.0 / avg_dt;
}

} // namespace Navigation