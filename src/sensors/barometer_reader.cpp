/**
 * Barometer Data Reader Implementation
 */

#include "barometer_reader.h"
#include "../utils/math_utils.h"
#include <sstream>
#include <cmath>
#include <algorithm>

namespace Navigation {

BarometerReader::BarometerReader(const BarometerConfig& config) : config_(config) {
    {

        std::stringstream msg;

        msg << "Barometer Reader initialized for file: " << config.csv_path;

        LOG_INFO(msg.str());

    }
}

BarometerReader::~BarometerReader() {
    if (file_.is_open()) {
        close();
    }
}

bool BarometerReader::open() {
    if (file_.is_open()) {
        return true;
    }
    
    file_.open(config_.csv_path);
    if (!file_.is_open()) {
        {

            std::stringstream msg;

            msg << "Failed to open barometer file: " << config_.csv_path;

            LOG_ERROR(msg.str());

        }
        return false;
    }
    
    // Skip header
    if (config_.has_header) {
        std::string header;
        std::getline(file_, header);
        {

            std::stringstream msg;

            msg << "Barometer CSV header: " << header;

            LOG_DEBUG(msg.str());

        }
    }
    
    stats_ = Stats();
    
    LOG_INFO("Barometer file opened successfully");
    return true;
}

void BarometerReader::close() {
    if (file_.is_open()) {
        file_.close();
        LOG_INFO("Barometer file closed");
        printStatistics();
    }
}

bool BarometerReader::readNext(BarometerData& data) {
    if (!file_.is_open()) {
        LOG_ERROR("Barometer file not open");
        return false;
    }
    
    std::string line;
    while (std::getline(file_, line)) {
        if (line.empty()) continue;
        
        if (parseLine(line, data)) {
            if (validateData(data)) {
                // Compute altitude if not provided
                if (config_.col_altitude < 0) {
                    data.altitude = NavMath::AtmosphericModel::pressureToAltitude(data.pressure);

                    // Validate altitude bounds (-1000m to 20000m) - very permissive
                    if (data.altitude < -1000.0 || data.altitude > 20000.0) {
                        LOG_WARN("Computed altitude out of reasonable range: " + std::to_string(data.altitude) + " m");
                        data.valid = false;
                        return false;
                    }
                }
                
                // Update statistics
                stats_.total_samples++;
                stats_.valid_samples++;
                stats_.min_pressure = std::min(stats_.min_pressure, data.pressure);
                stats_.max_pressure = std::max(stats_.max_pressure, data.pressure);
                if (stats_.valid_samples == 1) {
                    stats_.avg_pressure = data.pressure;
                } else {
                    stats_.avg_pressure = (stats_.avg_pressure * (stats_.valid_samples - 1) + data.pressure) / stats_.valid_samples;
                }
                stats_.min_altitude = std::min(stats_.min_altitude, data.altitude);
                stats_.max_altitude = std::max(stats_.max_altitude, data.altitude);
                
                return true;
            } else {
                stats_.total_samples++;
            }
        }
    }
    
    return false;
}

bool BarometerReader::readAll(std::vector<BarometerData>& data) {
    data.clear();
    
    BarometerData sample;
    while (readNext(sample)) {
        data.push_back(sample);
    }
    
    {

    
        std::stringstream msg;

    
        msg << "Read " << data.size() << " barometer samples";

    
        LOG_INFO(msg.str());

    
    }
    return !data.empty();
}

bool BarometerReader::parseLine(const std::string& line, BarometerData& data) {
    std::vector<std::string> tokens = splitString(line, config_.delimiter);
    
    int max_col = std::max({config_.col_timestamp, config_.col_pressure, config_.col_temperature});
    if (config_.col_altitude >= 0) {
        max_col = std::max(max_col, config_.col_altitude);
    }
    
    if (tokens.size() <= static_cast<size_t>(max_col)) {
        LOG_ERROR("Insufficient columns in barometer data");
        return false;
    }
    
    try {
        data.timestamp = std::stod(tokens[config_.col_timestamp]) * config_.time_scale;
        data.pressure = std::stod(tokens[config_.col_pressure]) * config_.pressure_scale;
        
        if (config_.pressure_in_hpa) {
            data.pressure *= 100.0;  // hPa to Pa
        }
        
        data.temperature = std::stod(tokens[config_.col_temperature]);
        
        if (config_.col_altitude >= 0) {
            data.altitude = std::stod(tokens[config_.col_altitude]);
        } else {
            // Calculate altitude from pressure using centralized atmospheric model
            data.altitude = NavMath::AtmosphericModel::pressureToAltitude(data.pressure);
        }

        data.valid = true;
        data.pressure_variance = config_.pressure_noise_std * config_.pressure_noise_std;
        
    } catch (const std::exception& e) {
        {

            std::stringstream msg;

            msg << "Error parsing barometer data: " << e.what();

            LOG_ERROR(msg.str());

        }
        return false;
    }
    
    return true;
}

std::vector<std::string> BarometerReader::splitString(const std::string& str, char delimiter) {
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

bool BarometerReader::validateData(BarometerData& data) {
    // Check pressure range
    if (data.pressure < config_.min_pressure || data.pressure > config_.max_pressure) {
        {

            std::stringstream msg;

            msg << "Pressure out of range: " << data.pressure << " Pa";

            LOG_WARN(msg.str());

        }
        data.valid = false;
        return false;
    }
    
    // Check temperature range
    if (data.temperature < config_.min_temperature || data.temperature > config_.max_temperature) {
        {

            std::stringstream msg;

            msg << "Temperature out of range: " << data.temperature << " C";

            LOG_WARN(msg.str());

        }
        data.valid = false;
        return false;
    }
    
    return true;
}

double BarometerReader::pressureToAltitude(double pressure, double temperature, double sea_level_pressure) {
    // Use centralized atmospheric model for consistency
    // Temperature parameter is kept for API compatibility but not used
    // as the centralized model uses ISA standard atmosphere
    return NavMath::AtmosphericModel::pressureToAltitude(pressure);
}

double BarometerReader::altitudeToPressure(double altitude, double temperature, double sea_level_pressure) {
    // Use centralized atmospheric model for consistency
    // Temperature and sea_level_pressure parameters kept for API compatibility
    return NavMath::AtmosphericModel::altitudeToPressure(altitude);
}

void BarometerReader::printStatistics() const {
    LOG_INFO("=== Barometer Reader Statistics ===");
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

        msg << "Pressure range: " << stats_.min_pressure << " - " << stats_.max_pressure << " Pa";

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Average pressure: " << stats_.avg_pressure << " Pa";

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "Altitude range: " << stats_.min_altitude << " - " << stats_.max_altitude << " m";

        LOG_INFO(msg.str());

    }
}

} // namespace Navigation