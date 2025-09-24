/**
 * Magnetometer Data Reader Implementation
 */

#include "magnetometer_reader.h"
#include <sstream>
#include <cmath>
#include <algorithm>

namespace Navigation {

MagnetometerReader::MagnetometerReader(const MagnetometerConfig& config) : config_(config) {
    {

        std::stringstream msg;

        msg << "Magnetometer Reader initialized for file: " << config.csv_path;

        LOG_INFO(msg.str());

    }
}

MagnetometerReader::~MagnetometerReader() {
    if (file_.is_open()) {
        close();
    }
}

bool MagnetometerReader::open() {
    if (file_.is_open()) {
        return true;
    }
    
    file_.open(config_.csv_path);
    if (!file_.is_open()) {
        {

            std::stringstream msg;

            msg << "Failed to open magnetometer file: " << config_.csv_path;

            LOG_ERROR(msg.str());

        }
        return false;
    }
    
    if (config_.has_header) {
        std::string header;
        std::getline(file_, header);
        {

            std::stringstream msg;

            msg << "Magnetometer CSV header: " << header;

            LOG_DEBUG(msg.str());

        }
    }
    
    stats_ = Stats();
    
    LOG_INFO("Magnetometer file opened successfully");
    return true;
}

void MagnetometerReader::close() {
    if (file_.is_open()) {
        file_.close();
        LOG_INFO("Magnetometer file closed");
        printStatistics();
    }
}

bool MagnetometerReader::readNext(MagnetometerData& data) {
    if (!file_.is_open()) {
        LOG_ERROR("Magnetometer file not open");
        return false;
    }
    
    std::string line;
    while (std::getline(file_, line)) {
        if (line.empty()) continue;
        
        if (parseLine(line, data)) {
            // Apply calibration
            applyCalibration(data);
            
            if (validateData(data)) {
                // Compute magnetic elements
                computeMagneticElements(data);
                
                // Check for disturbances
                data.is_disturbed = detectDisturbance(data);
                
                // Update statistics
                stats_.total_samples++;
                stats_.valid_samples++;
                if (data.is_disturbed) {
                    stats_.disturbed_samples++;
                }
                
                for (int i = 0; i < 3; ++i) {
                    stats_.min_field(i) = std::min(stats_.min_field(i), data.field(i));
                    stats_.max_field(i) = std::max(stats_.max_field(i), data.field(i));
                }
                
                return true;
            } else {
                stats_.total_samples++;
            }
        }
    }
    
    return false;
}

bool MagnetometerReader::readAll(std::vector<MagnetometerData>& data) {
    data.clear();
    
    MagnetometerData sample;
    while (readNext(sample)) {
        data.push_back(sample);
    }
    
    {

    
        std::stringstream msg;

    
        msg << "Read " << data.size() << " magnetometer samples";

    
        LOG_INFO(msg.str());

    
    }
    return !data.empty();
}

bool MagnetometerReader::parseLine(const std::string& line, MagnetometerData& data) {
    std::vector<std::string> tokens = splitString(line, config_.delimiter);
    
    int max_col = std::max({config_.col_timestamp, 
                           config_.col_mag_x, config_.col_mag_y, config_.col_mag_z});
    
    if (tokens.size() <= static_cast<size_t>(max_col)) {
        LOG_ERROR("Insufficient columns in magnetometer data");
        return false;
    }
    
    try {
        data.timestamp = std::stod(tokens[config_.col_timestamp]) * config_.time_scale;
        
        data.field.x() = std::stod(tokens[config_.col_mag_x]) * config_.field_scale;
        data.field.y() = std::stod(tokens[config_.col_mag_y]) * config_.field_scale;
        data.field.z() = std::stod(tokens[config_.col_mag_z]) * config_.field_scale;
        
        // Convert units if needed
        if (config_.field_in_gauss) {
            data.field *= 1e-4;  // Gauss to Tesla
        } else if (config_.field_in_ut) {
            data.field *= 1e-6;  // microTesla to Tesla
        }
        
        // Temperature if available
        if (config_.col_temperature >= 0 && config_.col_temperature < static_cast<int>(tokens.size())) {
            data.temperature = std::stod(tokens[config_.col_temperature]);
        } else {
            data.temperature = 25.0;  // Default
        }
        
        data.valid = true;
        
    } catch (const std::exception& e) {
        {

            std::stringstream msg;

            msg << "Error parsing magnetometer data: " << e.what();

            LOG_ERROR(msg.str());

        }
        return false;
    }
    
    return true;
}

void MagnetometerReader::applyCalibration(MagnetometerData& data) {
    // Apply hard iron correction
    data.field -= config_.hard_iron_offset;
    
    // Apply soft iron correction
    data.field = config_.soft_iron_matrix * data.field;
}

void MagnetometerReader::computeMagneticElements(MagnetometerData& data) {
    // Total field strength
    data.total_field = data.field.norm();
    
    if (data.total_field > 1e-9) {
        // Horizontal component
        double h = sqrt(data.field.x() * data.field.x() + data.field.y() * data.field.y());
        
        // Declination (angle from north)
        data.declination = atan2(data.field.y(), data.field.x());
        
        // Inclination (dip angle)
        data.inclination = atan2(-data.field.z(), h);
    }
}

bool MagnetometerReader::detectDisturbance(const MagnetometerData& data) {
    // Check if field strength is abnormal
    return data.total_field > config_.disturbance_threshold;
}

bool MagnetometerReader::validateData(MagnetometerData& data) {
    // Check for NaN or Inf
    if (!data.field.allFinite()) {
        LOG_WARN("Magnetometer data contains NaN or Inf");
        data.valid = false;
        return false;
    }
    
    // Check field strength
    double field_norm = data.field.norm();
    if (field_norm < config_.min_field || field_norm > config_.max_field) {
        {

            std::stringstream msg;

            msg << "Magnetic field out of range: " << field_norm << " T";

            LOG_WARN(msg.str());

        }
        data.valid = false;
        return false;
    }
    
    return true;
}

std::vector<std::string> MagnetometerReader::splitString(const std::string& str, char delimiter) {
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

void MagnetometerReader::printStatistics() const {
    LOG_INFO("=== Magnetometer Reader Statistics ===");
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
        msg << "Disturbed samples: " << stats_.disturbed_samples
             << " (" << 100.0 * stats_.disturbed_samples / stats_.valid_samples << "%)";
        LOG_INFO(msg.str());
    }
    {

        std::stringstream msg;

        msg << "Field range X: " << stats_.min_field.x() * 1e6 << " - " << stats_.max_field.x() * 1e6 << " µT";

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Field range Y: " << stats_.min_field.y() * 1e6 << " - " << stats_.max_field.y() * 1e6 << " µT";

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Field range Z: " << stats_.min_field.z() * 1e6 << " - " << stats_.max_field.z() * 1e6 << " µT";

        LOG_INFO(msg.str());

    }
}

} // namespace Navigation