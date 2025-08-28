#pragma once
#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>

/**
 * Simple configuration file reader
 * Supports key-value pairs and sections
 */
class ConfigReader {
private:
    std::map<std::string, std::string> config_;
    
public:
    ConfigReader() = default;
    
    /**
     * Load configuration from YAML-style file
     */
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            // Try default config
            config_["system.rate_hz"] = "100";
            config_["system.max_runtime"] = "3600";
            config_["ukf.alpha"] = "0.001";
            config_["ukf.beta"] = "2.0";
            config_["ukf.kappa"] = "0.0";
            config_["hardware.imu.type"] = "simulated";
            config_["hardware.imu.port"] = "/dev/ttyUSB0";
            config_["hardware.gradiometer.enabled"] = "false";
            config_["hardware.csac.enabled"] = "false";
            config_["output.path"] = "data/gravity_nav.csv";
            return true; // Use defaults
        }
        
        std::string line;
        std::string current_section;
        
        while (std::getline(file, line)) {
            // Skip comments and empty lines
            if (line.empty() || line[0] == '#') continue;
            
            // Remove leading/trailing whitespace
            line.erase(0, line.find_first_not_of(" \t"));
            line.erase(line.find_last_not_of(" \t") + 1);
            
            if (line.empty()) continue;
            
            // Check for section header
            if (line.back() == ':') {
                current_section = line.substr(0, line.length() - 1);
                continue;
            }
            
            // Parse key-value pair
            size_t colon_pos = line.find(':');
            if (colon_pos != std::string::npos) {
                std::string key = line.substr(0, colon_pos);
                std::string value = line.substr(colon_pos + 1);
                
                // Trim whitespace
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);
                
                // Store with section prefix
                if (!current_section.empty()) {
                    config_[current_section + "." + key] = value;
                } else {
                    config_[key] = value;
                }
            }
        }
        
        file.close();
        return true;
    }
    
    std::string getString(const std::string& key, const std::string& default_value = "") const {
        auto it = config_.find(key);
        return (it != config_.end()) ? it->second : default_value;
    }
    
    double getDouble(const std::string& key, double default_value = 0.0) const {
        auto it = config_.find(key);
        if (it != config_.end()) {
            try {
                return std::stod(it->second);
            } catch (...) {
                return default_value;
            }
        }
        return default_value;
    }
    
    int getInt(const std::string& key, int default_value = 0) const {
        auto it = config_.find(key);
        if (it != config_.end()) {
            try {
                return std::stoi(it->second);
            } catch (...) {
                return default_value;
            }
        }
        return default_value;
    }
    
    bool getBool(const std::string& key, bool default_value = false) const {
        auto it = config_.find(key);
        if (it != config_.end()) {
            std::string val = it->second;
            return (val == "true" || val == "True" || val == "1" || val == "yes");
        }
        return default_value;
    }
};