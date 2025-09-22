#pragma once

#include <string>
#include <vector>
#include <map>
#include <sstream>
#include <stdexcept>

namespace ml {

/**
 * Simple JSON parser for ML communication
 * Handles scientific notation and arrays properly
 */
class SimpleJSON {
public:
    static std::map<std::string, std::string> parse(const std::string& json) {
        std::map<std::string, std::string> result;

        // Remove whitespace around colons and commas for easier parsing
        std::string cleaned = json;

        // Find all key-value pairs
        size_t pos = 0;
        while ((pos = cleaned.find("\"", pos)) != std::string::npos) {
            // Find key
            size_t key_start = pos + 1;
            size_t key_end = cleaned.find("\"", key_start);
            if (key_end == std::string::npos) break;

            std::string key = cleaned.substr(key_start, key_end - key_start);

            // Find colon
            size_t colon_pos = cleaned.find(":", key_end);
            if (colon_pos == std::string::npos) break;

            // Find value (could be string, number, boolean, or array)
            size_t value_start = colon_pos + 1;

            // Skip whitespace
            while (value_start < cleaned.length() &&
                   (cleaned[value_start] == ' ' || cleaned[value_start] == '\t')) {
                value_start++;
            }

            if (value_start >= cleaned.length()) break;

            std::string value;

            if (cleaned[value_start] == '"') {
                // String value
                size_t value_end = cleaned.find("\"", value_start + 1);
                if (value_end != std::string::npos) {
                    value = cleaned.substr(value_start + 1, value_end - value_start - 1);
                }
                pos = value_end + 1;
            }
            else if (cleaned[value_start] == '[') {
                // Array value - find matching ]
                int bracket_count = 1;
                size_t i = value_start + 1;
                while (i < cleaned.length() && bracket_count > 0) {
                    if (cleaned[i] == '[') bracket_count++;
                    else if (cleaned[i] == ']') bracket_count--;
                    i++;
                }
                value = cleaned.substr(value_start, i - value_start);
                pos = i;
            }
            else {
                // Number or boolean - find next comma or }
                size_t value_end = cleaned.find_first_of(",}", value_start);
                if (value_end == std::string::npos) {
                    value_end = cleaned.length();
                }
                value = cleaned.substr(value_start, value_end - value_start);

                // Trim whitespace from value
                size_t trim_start = value.find_first_not_of(" \t\n\r");
                size_t trim_end = value.find_last_not_of(" \t\n\r");
                if (trim_start != std::string::npos && trim_end != std::string::npos) {
                    value = value.substr(trim_start, trim_end - trim_start + 1);
                }

                pos = value_end;
            }

            result[key] = value;
        }

        return result;
    }

    static std::vector<double> parseArray(const std::string& array_str) {
        std::vector<double> result;

        // Remove brackets
        std::string content = array_str;
        if (content.front() == '[') content = content.substr(1);
        if (content.back() == ']') content.pop_back();

        // Parse comma-separated values
        std::stringstream ss(content);
        std::string token;

        while (std::getline(ss, token, ',')) {
            // Trim whitespace
            size_t start = token.find_first_not_of(" \t");
            size_t end = token.find_last_not_of(" \t");

            if (start != std::string::npos && end != std::string::npos) {
                std::string num_str = token.substr(start, end - start + 1);

                try {
                    // std::stod handles scientific notation properly
                    double value = std::stod(num_str);
                    result.push_back(value);
                } catch (...) {
                    // Skip invalid numbers
                }
            }
        }

        return result;
    }

    static bool parseBool(const std::string& bool_str) {
        return bool_str == "true" || bool_str == "True" || bool_str == "1";
    }

    static double parseDouble(const std::string& num_str) {
        try {
            return std::stod(num_str);
        } catch (...) {
            return 0.0;
        }
    }
};

} // namespace ml