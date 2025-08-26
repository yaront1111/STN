#pragma once
#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

// Simple configuration manager for STN
// Supports key-value pairs from file and command-line overrides
class Config {
private:
    std::map<std::string, double> params_;
    std::map<std::string, std::string> strings_;
    
    // Parse a simple key=value line
    void parseLine(const std::string& line) {
        if (line.empty() || line[0] == '#') return;  // Skip comments
        
        size_t eq_pos = line.find('=');
        if (eq_pos == std::string::npos) return;
        
        std::string key = line.substr(0, eq_pos);
        std::string value = line.substr(eq_pos + 1);
        
        // Trim whitespace
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);
        
        // Try to parse as number
        try {
            double num_val = std::stod(value);
            params_[key] = num_val;
        } catch (...) {
            // Store as string if not a number
            strings_[key] = value;
        }
    }
    
public:
    // Default configuration values
    Config() {
        // System parameters
        params_["system.imu_rate_hz"] = 200.0;
        params_["system.trn_rate_hz"] = 2.0;
        params_["system.gravity_rate_hz"] = 2.0;
        params_["system.output_rate_hz"] = 100.0;
        
        // TRN parameters
        params_["trn.enabled"] = 1.0;
        params_["trn.alpha_base"] = 0.001;
        params_["trn.alpha_max"] = 0.1;
        params_["trn.huber_threshold"] = 3.0;
        params_["trn.slope_threshold"] = 0.1;
        params_["trn.velocity_gate"] = 5.0;
        params_["trn.min_agl"] = 10.0;
        params_["trn.max_agl"] = 5000.0;
        
        // EKF parameters
        params_["ekf.q_pos"] = 0.1;
        params_["ekf.q_vel"] = 0.01;
        params_["ekf.q_att"] = 0.001;
        params_["ekf.r_trn_base"] = 25.0;
        params_["ekf.r_gravity"] = 0.01;
        params_["ekf.r_baro"] = 1.0;
        params_["ekf.initial_pos_std"] = 10.0;
        params_["ekf.initial_vel_std"] = 1.0;
        params_["ekf.initial_att_std"] = 0.01;
        
        // Gravity parameters
        params_["gravity.enabled"] = 1.0;
        params_["gravity.use_egm2008"] = 0.0;  // Default to synthetic
        params_["gravity.anomaly_scale"] = 1.0;
        
        // Terrain parameters
        params_["terrain.use_srtm"] = 0.0;  // Default to synthetic
        params_["terrain.cache_tiles"] = 1.0;
        
        // File paths
        strings_["paths.egm2008"] = "data/egm2008.dat";
        strings_["paths.terrain"] = "data/terrain";
        strings_["paths.output"] = "data/output.csv";
    }
    
    // Load configuration from file
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Warning: Config file not found: " << filename << std::endl;
            std::cerr << "Using default configuration" << std::endl;
            return false;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            parseLine(line);
        }
        
        std::cout << "Loaded configuration from: " << filename << std::endl;
        return true;
    }
    
    // Override with command-line arguments
    void parseCommandLine(int argc, char* argv[]) {
        for (int i = 1; i < argc; i++) {
            std::string arg(argv[i]);
            if (arg.substr(0, 2) == "--" && arg.find('=') != std::string::npos) {
                parseLine(arg.substr(2));  // Remove "--"
            }
        }
    }
    
    // Get numeric parameter
    double getDouble(const std::string& key, double default_val = 0.0) const {
        auto it = params_.find(key);
        return (it != params_.end()) ? it->second : default_val;
    }
    
    // Get integer parameter
    int getInt(const std::string& key, int default_val = 0) const {
        return static_cast<int>(getDouble(key, default_val));
    }
    
    // Get boolean parameter
    bool getBool(const std::string& key, bool default_val = false) const {
        return getDouble(key, default_val ? 1.0 : 0.0) != 0.0;
    }
    
    // Get string parameter
    std::string getString(const std::string& key, const std::string& default_val = "") const {
        auto it = strings_.find(key);
        return (it != strings_.end()) ? it->second : default_val;
    }
    
    // Set parameter (for runtime adjustment)
    void setDouble(const std::string& key, double value) {
        params_[key] = value;
    }
    
    void setString(const std::string& key, const std::string& value) {
        strings_[key] = value;
    }
    
    // Print current configuration
    void print() const {
        std::cout << "=== Configuration ===" << std::endl;
        std::cout << "Numeric parameters:" << std::endl;
        for (const auto& p : params_) {
            std::cout << "  " << p.first << " = " << p.second << std::endl;
        }
        std::cout << "String parameters:" << std::endl;
        for (const auto& s : strings_) {
            std::cout << "  " << s.first << " = " << s.second << std::endl;
        }
        std::cout << "===================" << std::endl;
    }
    
    // Singleton access (optional, for global config)
    static Config& instance() {
        static Config config;
        return config;
    }
};