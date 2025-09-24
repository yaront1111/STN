/**
 * Magnetometer Data Reader Implementation (fixed & hardened)
 *
 * - Robust CSV open/close with header handling
 * - Consistent units (store internally in Tesla)
 * - Safe stats (no div-by-zero, sane min/max init)
 * - Better validation & disturbance detection
 * - Temperature compensation hook
 * - Calib order: hard-iron -> soft-iron -> temp comp
 */

#include "magnetometer_reader.h"

#include <sstream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <limits>

namespace Navigation {

namespace {
// Trim helpers
inline void ltrim(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(),
        [](unsigned char ch){ return !std::isspace(ch); }));
}
inline void rtrim(std::string& s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
        [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
}
inline void trim(std::string& s) { ltrim(s); rtrim(s); }

// Safe stod with context
inline bool try_stod(const std::string& t, double& out) {
    try {
        size_t idx = 0;
        out = std::stod(t, &idx);
        return idx == t.size();
    } catch (...) {
        return false;
    }
}
} // namespace

MagnetometerReader::MagnetometerReader(const MagnetometerConfig& config)
: config_(config)
{
    {
        std::stringstream msg;
        msg << "Magnetometer Reader initialized for file: " << config.csv_path;
        LOG_INFO(msg.str());
    }

    // Initialize stats with safe sentinels
    stats_ = Stats();
    stats_.min_field = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    stats_.max_field = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

    // Precompute validation range in Tesla (config given in microTesla)
    min_field_T_ = static_cast<double>(config_.min_field) * 1e-6;
    max_field_T_ = static_cast<double>(config_.max_field) * 1e-6;

    // Expected total field (optional) in Tesla
    expected_total_field_T_ = (config_.expected_total_field_uT > 0.0)
        ? config_.expected_total_field_uT * 1e-6
        : 0.0;
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
        std::stringstream msg;
        msg << "Failed to open magnetometer file: " << config_.csv_path;
        LOG_ERROR(msg.str());
        return false;
    }

    // Optionally consume UTF-8 BOM
    if (file_.peek() == 0xEF) {
        char bom[3];
        file_.read(bom, 3);
        if (!(static_cast<unsigned char>(bom[0]) == 0xEF &&
              static_cast<unsigned char>(bom[1]) == 0xBB &&
              static_cast<unsigned char>(bom[2]) == 0xBF)) {
            // Not really BOM; rewind
            file_.clear();
            file_.seekg(0);
        }
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

    // Reset stats on open
    stats_ = Stats();
    stats_.min_field = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    stats_.max_field = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());

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
        trim(line);
        if (line.empty()) continue;
        if (line[0] == '#') continue;  // allow comments

        if (parseLine(line, data)) {
            // Calibrate: hard-iron → soft-iron → temperature comp
            applyCalibration(data);

            if (validateData(data)) {
                // Compute magnetic elements (declination/inclination)
                computeMagneticElements(data);

                // Disturbance detection
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

                // Update simple running mean of total field (for z-score)
                const double alpha = 0.01; // EWMA
                if (stats_.valid_samples == 1) {
                    ewma_total_field_T_ = data.total_field;
                    ewma_var_total_field_T2_ = 0.0;
                } else {
                    double diff = data.total_field - ewma_total_field_T_;
                    ewma_total_field_T_ += alpha * diff;
                    ewma_var_total_field_T2_ = (1.0 - alpha) * (ewma_var_total_field_T2_ + alpha * diff * diff);
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

    // Sanity of column indices
    int max_col = std::max({config_.col_timestamp,
                            config_.col_mag_x, config_.col_mag_y, config_.col_mag_z,
                            std::max(0, config_.col_temperature)});
    if (tokens.size() <= static_cast<size_t>(max_col)) {
        LOG_ERROR("Insufficient columns in magnetometer data");
        return false;
    }

    // Parse timestamp
    double ts = 0.0;
    if (!try_stod(tokens[config_.col_timestamp], ts)) {
        std::stringstream msg;
        msg << "Bad timestamp token: '" << tokens[config_.col_timestamp] << "'";
        LOG_ERROR(msg.str());
        return false;
    }
    data.timestamp = ts * config_.time_scale;

    // Raw magnetic field (apply scale first)
    auto parse_scaled = [&](int idx)->double {
        double v = 0.0;
        if (!try_stod(tokens[idx], v)) {
            std::stringstream msg;
            msg << "Bad magnetometer token at col " << idx << ": '" << tokens[idx] << "'";
            LOG_ERROR(msg.str());
            throw std::runtime_error("parse_scaled failed");
        }
        return v * config_.field_scale;
    };

    double mx = 0.0, my = 0.0, mz = 0.0;
    try {
        mx = parse_scaled(config_.col_mag_x);
        my = parse_scaled(config_.col_mag_y);
        mz = parse_scaled(config_.col_mag_z);
    } catch (...) {
        return false;
    }

    // Convert to Tesla BEFORE storing
    if (config_.field_in_gauss) {
        mx *= 1e-4; my *= 1e-4; mz *= 1e-4; // G → T
    } else if (config_.field_in_ut) {
        mx *= 1e-6; my *= 1e-6; mz *= 1e-6; // µT → T
    }
    data.field.x() = mx;
    data.field.y() = my;
    data.field.z() = mz;

    // Optional temperature
    if (config_.col_temperature >= 0 &&
        config_.col_temperature < static_cast<int>(tokens.size())) {
        double t = 25.0;
        if (!try_stod(tokens[config_.col_temperature], t)) {
            t = 25.0;
        }
        data.temperature = t;
    } else {
        data.temperature = 25.0;
    }

    data.valid = true;
    return true;
}

void MagnetometerReader::applyCalibration(MagnetometerData& data) {
    // 1) Hard-iron (bias) correction
    data.field -= config_.hard_iron_offset; // Tesla

    // 2) Soft-iron (scale/skew) correction
    //    Expect a 3x3 matrix in Tesla domain
    data.field = config_.soft_iron_matrix * data.field;

    // 3) Temperature compensation (optional)
    //    field_compensated = field + temp_coeff * (T - Tref)
    if (config_.enable_temperature_comp) {
        const double dT = (data.temperature - config_.temp_ref_C);
        data.field += config_.temp_coeff_T_per_C * dT;
    }
}

void MagnetometerReader::computeMagneticElements(MagnetometerData& data) {
    // Total field strength (Tesla)
    data.total_field = data.field.norm();

    // Protect tiny magnitudes
    if (data.total_field > 1e-12) {
        // Horizontal component H (Tesla)
        const double H = std::hypot(data.field.x(), data.field.y());

        // Declination D: atan2(East, North)
        // Here we assume x ~ North, y ~ East (NED frame common)
        data.declination = std::atan2(data.field.y(), data.field.x());

        // Inclination I (dip): atan2(-Down, Horizontal) for NED -> z is Down
        data.inclination = std::atan2(-data.field.z(), std::max(1e-12, H));
    } else {
        data.declination = 0.0;
        data.inclination = 0.0;
    }
}

bool MagnetometerReader::detectDisturbance(const MagnetometerData& data) {
    // 1) Absolute threshold on total field (Tesla)
    if (config_.disturbance_threshold > 0.0 &&
        data.total_field > config_.disturbance_threshold) {
        return true;
    }

    // 2) If expected total field known, check relative deviation
    if (expected_total_field_T_ > 0.0) {
        double rel_err = std::abs(data.total_field - expected_total_field_T_) /
                         expected_total_field_T_;
        if (rel_err > config_.relative_disturbance_threshold) {
            return true;
        }
    }

    // 3) Z-score on EWMA if variance has settled
    if (stats_.valid_samples > 50 && ewma_var_total_field_T2_ > 0.0) {
        double std_est = std::sqrt(ewma_var_total_field_T2_);
        if (std_est > 0.0) {
            double z = std::abs(data.total_field - ewma_total_field_T_) / std_est;
            if (z > config_.zscore_disturbance_threshold) {
                return true;
            }
        }
    }

    return false;
}

bool MagnetometerReader::validateData(MagnetometerData& data) {
    // Check for NaN or Inf
    if (!data.field.allFinite()) {
        LOG_WARN("Magnetometer data contains NaN or Inf");
        data.valid = false;
        return false;
    }

    // Validate magnitude range in Tesla (converted from µT cfg once)
    const double n = data.field.norm();
    if (n < min_field_T_ || n > max_field_T_) {
        std::stringstream msg;
        msg << "Magnetic field out of range: " << (n * 1e6) << " uT. Valid range: ["
            << config_.min_field << ", " << config_.max_field << "] uT";
        LOG_WARN(msg.str());
        data.valid = false;
        return false;
    }

    data.valid = true;
    return true;
}

std::vector<std::string> MagnetometerReader::splitString(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        trim(token);
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
        double pct = 0.0;
        if (stats_.valid_samples > 0) {
            pct = 100.0 * static_cast<double>(stats_.disturbed_samples) /
                              static_cast<double>(stats_.valid_samples);
        }
        msg << "Disturbed samples: " << stats_.disturbed_samples
            << " (" << pct << "%)";
        LOG_INFO(msg.str());
    }

    auto fmt_range = [](double lo_T, double hi_T) {
        std::stringstream s;
        s << (lo_T * 1e6) << " - " << (hi_T * 1e6) << " µT";
        return s.str();
    };

    // If no valid samples, min/max will be +/-inf. Guard it.
    Eigen::Vector3d minT = stats_.min_field, maxT = stats_.max_field;
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(minT(i))) minT(i) = 0.0;
        if (!std::isfinite(maxT(i))) maxT(i) = 0.0;
    }

    {
        std::stringstream msg;
        msg << "Field range X: " << fmt_range(minT.x(), maxT.x());
        LOG_INFO(msg.str());
    }
    {
        std::stringstream msg;
        msg << "Field range Y: " << fmt_range(minT.y(), maxT.y());
        LOG_INFO(msg.str());
    }
    {
        std::stringstream msg;
        msg << "Field range Z: " << fmt_range(minT.z(), maxT.z());
        LOG_INFO(msg.str());
    }
}

} // namespace Navigation
