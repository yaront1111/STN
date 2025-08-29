#include "config_reader.h"
#include <cstdlib>
#include <regex>
#include <iomanip>

bool ConfigReader::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        setError("Failed to open configuration file: " + filename);
        return false;
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    
    return loadFromString(buffer.str());
}

bool ConfigReader::loadFromString(const std::string& yaml_content) {
    clear();
    
    std::istringstream stream(yaml_content);
    std::string line;
    std::string current_prefix;
    int line_num = 0;
    int indent_level = 0;
    std::vector<std::string> prefix_stack;
    
    while (std::getline(stream, line)) {
        line_num++;
        
        // Skip empty lines and comments
        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }
        
        // Calculate indentation level
        size_t indent = 0;
        while (indent < line.length() && (line[indent] == ' ' || line[indent] == '\t')) {
            indent++;
        }
        
        // Handle indentation changes
        int new_level = indent / 2;  // Assume 2-space indentation
        
        if (new_level < indent_level) {
            // Dedent - pop from prefix stack
            int levels_to_pop = indent_level - new_level;
            for (int i = 0; i < levels_to_pop && !prefix_stack.empty(); i++) {
                prefix_stack.pop_back();
            }
        }
        indent_level = new_level;
        
        // Build current prefix from stack
        current_prefix = "";
        for (const auto& p : prefix_stack) {
            if (!current_prefix.empty()) current_prefix += ".";
            current_prefix += p;
        }
        
        // Parse the line
        if (!parseLine(trimmed, line_num, current_prefix)) {
            return false;
        }
        
        // Check if this line introduces a new namespace
        if (trimmed.back() == ':' && trimmed.find(':') == trimmed.length() - 1) {
            // This is a namespace declaration
            std::string namespace_name = trim(trimmed.substr(0, trimmed.length() - 1));
            prefix_stack.push_back(namespace_name);
        }
    }
    
    return true;
}

bool ConfigReader::parseLine(const std::string& line, int line_num, std::string& current_prefix) {
    // Find the colon separator
    size_t colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
        setError("Invalid syntax - missing colon", line_num);
        return false;
    }
    
    std::string key = trim(line.substr(0, colon_pos));
    std::string value_str = trim(line.substr(colon_pos + 1));
    
    // Skip namespace declarations (empty value after colon)
    if (value_str.empty()) {
        return true;
    }
    
    // Build full key with prefix
    std::string full_key = current_prefix.empty() ? key : current_prefix + "." + key;
    
    // Expand environment variables
    value_str = expandEnvironmentVariables(value_str);
    
    // Parse and store value
    config_[full_key] = parseValue(value_str);
    
    return true;
}

ConfigReader::ConfigValue ConfigReader::parseValue(const std::string& value_str) {
    // Remove quotes if present
    std::string value = value_str;
    if ((value.front() == '"' && value.back() == '"') ||
        (value.front() == '\'' && value.back() == '\'')) {
        value = value.substr(1, value.length() - 2);
        return value;  // String value
    }
    
    // Check for boolean
    if (isBool(value)) {
        std::string lower = toLower(value);
        return (lower == "true" || lower == "yes" || lower == "on" || lower == "1");
    }
    
    // Check for integer
    if (value.find('.') == std::string::npos && isNumber(value)) {
        try {
            return std::stoi(value);
        } catch (...) {
            // Fall through to double
        }
    }
    
    // Check for double
    if (isNumber(value)) {
        try {
            return std::stod(value);
        } catch (...) {
            // Fall through to string
        }
    }
    
    // Default to string
    return value;
}

std::string ConfigReader::getString(const std::string& key, const std::string& default_value) const {
    last_accessed_key_ = key;
    auto it = config_.find(key);
    if (it == config_.end()) {
        return default_value;
    }
    
    if (std::holds_alternative<std::string>(it->second)) {
        return std::get<std::string>(it->second);
    }
    
    // Convert other types to string
    if (std::holds_alternative<int>(it->second)) {
        return std::to_string(std::get<int>(it->second));
    }
    if (std::holds_alternative<double>(it->second)) {
        return std::to_string(std::get<double>(it->second));
    }
    if (std::holds_alternative<bool>(it->second)) {
        return std::get<bool>(it->second) ? "true" : "false";
    }
    
    return default_value;
}

double ConfigReader::getDouble(const std::string& key, double default_value) const {
    last_accessed_key_ = key;
    auto it = config_.find(key);
    if (it == config_.end()) {
        return default_value;
    }
    
    if (std::holds_alternative<double>(it->second)) {
        return std::get<double>(it->second);
    }
    if (std::holds_alternative<int>(it->second)) {
        return static_cast<double>(std::get<int>(it->second));
    }
    if (std::holds_alternative<std::string>(it->second)) {
        try {
            return std::stod(std::get<std::string>(it->second));
        } catch (...) {
            return default_value;
        }
    }
    
    return default_value;
}

int ConfigReader::getInt(const std::string& key, int default_value) const {
    last_accessed_key_ = key;
    auto it = config_.find(key);
    if (it == config_.end()) {
        return default_value;
    }
    
    if (std::holds_alternative<int>(it->second)) {
        return std::get<int>(it->second);
    }
    if (std::holds_alternative<double>(it->second)) {
        return static_cast<int>(std::get<double>(it->second));
    }
    if (std::holds_alternative<std::string>(it->second)) {
        try {
            return std::stoi(std::get<std::string>(it->second));
        } catch (...) {
            return default_value;
        }
    }
    
    return default_value;
}

bool ConfigReader::getBool(const std::string& key, bool default_value) const {
    last_accessed_key_ = key;
    auto it = config_.find(key);
    if (it == config_.end()) {
        return default_value;
    }
    
    if (std::holds_alternative<bool>(it->second)) {
        return std::get<bool>(it->second);
    }
    if (std::holds_alternative<int>(it->second)) {
        return std::get<int>(it->second) != 0;
    }
    if (std::holds_alternative<std::string>(it->second)) {
        std::string lower = toLower(std::get<std::string>(it->second));
        return (lower == "true" || lower == "yes" || lower == "on" || lower == "1");
    }
    
    return default_value;
}

bool ConfigReader::hasKey(const std::string& key) const {
    return config_.find(key) != config_.end();
}

std::vector<std::string> ConfigReader::getKeysWithPrefix(const std::string& prefix) const {
    std::vector<std::string> keys;
    for (const auto& [key, value] : config_) {
        if (key.find(prefix) == 0) {
            keys.push_back(key);
        }
    }
    return keys;
}

bool ConfigReader::validate(const std::vector<std::string>& required_keys) const {
    for (const auto& key : required_keys) {
        if (!hasKey(key)) {
            const_cast<ConfigReader*>(this)->setError("Missing required configuration key: " + key);
            return false;
        }
    }
    return true;
}

void ConfigReader::clear() {
    config_.clear();
    last_error_.clear();
    last_accessed_key_.clear();
}

std::string ConfigReader::toString() const {
    std::stringstream ss;
    ss << "Configuration dump:\n";
    ss << "==================\n";
    
    // Sort keys for consistent output
    std::vector<std::string> keys;
    for (const auto& [key, value] : config_) {
        keys.push_back(key);
    }
    std::sort(keys.begin(), keys.end());
    
    for (const auto& key : keys) {
        ss << std::setw(30) << std::left << key << " = ";
        
        const auto& value = config_.at(key);
        if (std::holds_alternative<std::string>(value)) {
            ss << "\"" << std::get<std::string>(value) << "\"";
        } else if (std::holds_alternative<int>(value)) {
            ss << std::get<int>(value);
        } else if (std::holds_alternative<double>(value)) {
            ss << std::get<double>(value);
        } else if (std::holds_alternative<bool>(value)) {
            ss << (std::get<bool>(value) ? "true" : "false");
        }
        ss << "\n";
    }
    
    return ss.str();
}

std::string ConfigReader::expandEnvironmentVariables(const std::string& str) const {
    std::regex env_regex("\\$\\{([^}]+)\\}");
    std::string result = str;
    
    std::smatch match;
    while (std::regex_search(result, match, env_regex)) {
        const char* env_value = std::getenv(match[1].str().c_str());
        std::string replacement = env_value ? env_value : "";
        result = result.substr(0, match.position()) + replacement + result.substr(match.position() + match.length());
    }
    
    return result;
}

std::string ConfigReader::trim(const std::string& str) const {
    const char* whitespace = " \t\n\r\f\v";
    size_t start = str.find_first_not_of(whitespace);
    if (start == std::string::npos) return "";
    size_t end = str.find_last_not_of(whitespace);
    return str.substr(start, end - start + 1);
}

std::string ConfigReader::toLower(const std::string& str) const {
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::tolower);
    return result;
}

bool ConfigReader::isNumber(const std::string& str) const {
    if (str.empty()) return false;
    
    size_t start = 0;
    if (str[0] == '-' || str[0] == '+') start = 1;
    
    bool has_digit = false;
    bool has_dot = false;
    bool has_e = false;
    
    for (size_t i = start; i < str.length(); i++) {
        if (std::isdigit(str[i])) {
            has_digit = true;
        } else if (str[i] == '.' && !has_dot && !has_e) {
            has_dot = true;
        } else if ((str[i] == 'e' || str[i] == 'E') && has_digit && !has_e) {
            has_e = true;
            if (i + 1 < str.length() && (str[i + 1] == '+' || str[i + 1] == '-')) {
                i++;  // Skip sign after E
            }
        } else {
            return false;
        }
    }
    
    return has_digit;
}

bool ConfigReader::isBool(const std::string& str) const {
    std::string lower = toLower(str);
    return (lower == "true" || lower == "false" ||
            lower == "yes" || lower == "no" ||
            lower == "on" || lower == "off" ||
            lower == "1" || lower == "0");
}

void ConfigReader::setError(const std::string& error, int line_num) {
    if (line_num > 0) {
        last_error_ = "Line " + std::to_string(line_num) + ": " + error;
    } else {
        last_error_ = error;
    }
    
    std::cerr << "ConfigReader Error: " << last_error_ << std::endl;
}