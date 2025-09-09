#pragma once
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <cctype>

/**
 * Production-grade YAML Configuration Reader
 * 
 * Supports:
 * - Nested configuration with dot notation (e.g., "ukf.alpha")
 * - Type-safe access with defaults
 * - Comments and blank lines
 * - String, double, int, and bool types
 * - Environment variable expansion
 * - Configuration validation
 * - Error reporting with line numbers
 */
class ConfigReader {
public:
    using ConfigValue = std::variant<std::string, double, int, bool>;
    using ConfigMap = std::unordered_map<std::string, ConfigValue>;
    
    ConfigReader() = default;
    
    /**
     * Load configuration from YAML file
     * @param filename Path to YAML configuration file
     * @return true if successful, false otherwise
     */
    bool loadFromFile(const std::string& filename);
    
    /**
     * Load configuration from string (for testing)
     * @param yaml_content YAML content as string
     * @return true if successful, false otherwise
     */
    bool loadFromString(const std::string& yaml_content);
    
    /**
     * Get string value with optional default
     */
    std::string getString(const std::string& key, const std::string& default_value = "") const;
    
    /**
     * Get double value with optional default
     */
    double getDouble(const std::string& key, double default_value = 0.0) const;
    
    /**
     * Get integer value with optional default
     */
    int getInt(const std::string& key, int default_value = 0) const;
    
    /**
     * Get boolean value with optional default
     */
    bool getBool(const std::string& key, bool default_value = false) const;
    
    /**
     * Check if a key exists
     */
    bool hasKey(const std::string& key) const;
    
    /**
     * Get all keys matching a prefix
     */
    std::vector<std::string> getKeysWithPrefix(const std::string& prefix) const;
    
    /**
     * Validate configuration against required keys
     * @param required_keys List of keys that must be present
     * @return true if all required keys exist, false otherwise
     */
    bool validate(const std::vector<std::string>& required_keys) const;
    
    /**
     * Get last error message
     */
    std::string getLastError() const { return last_error_; }
    
    /**
     * Clear all configuration
     */
    void clear();
    
    /**
     * Dump configuration to string (for debugging)
     */
    std::string toString() const;
    
private:
    ConfigMap config_;
    std::string last_error_;
    mutable std::string last_accessed_key_;  // For debugging
    
    /**
     * Parse a single line of YAML
     * @param line The line to parse
     * @param line_num Line number for error reporting
     * @param current_prefix Current namespace prefix for nested configs
     * @return true if successful, false on error
     */
    bool parseLine(const std::string& line, int line_num, std::string& current_prefix);
    
    /**
     * Parse a value string into appropriate type
     */
    ConfigValue parseValue(const std::string& value_str);
    
    /**
     * Expand environment variables in string
     * Replaces ${VAR_NAME} with environment variable value
     */
    std::string expandEnvironmentVariables(const std::string& str) const;
    
    /**
     * Trim whitespace from string
     */
    std::string trim(const std::string& str) const;
    
    /**
     * Convert string to lowercase
     */
    std::string toLower(const std::string& str) const;
    
    /**
     * Check if string represents a number
     */
    bool isNumber(const std::string& str) const;
    
    /**
     * Check if string represents a boolean
     */
    bool isBool(const std::string& str) const;
    
    /**
     * Set error message with context
     */
    void setError(const std::string& error, int line_num = -1);
};

/**
 * Global configuration singleton (optional)
 * Usage: Config::getInstance().loadFromFile("config.yaml");
 */
class Config {
public:
    static ConfigReader& getInstance() {
        static ConfigReader instance;
        return instance;
    }
    
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;
    
private:
    Config() = default;
};