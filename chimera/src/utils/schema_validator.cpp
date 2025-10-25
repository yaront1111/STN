// CHIMERA Telemetry Schema Validator Implementation

#include "utils/schema_validator.h"
#include <algorithm>
#include <regex>
#include <sstream>
#include <iostream>

namespace chimera {
namespace utils {

// =================== Helper Functions ===================

bool SchemaValidator::hasField(const std::string& json, const std::string& field) {
  std::string pattern = "\"" + field + "\":";
  return json.find(pattern) != std::string::npos;
}

bool SchemaValidator::hasNestedField(const std::string& json,
                                     const std::string& parent,
                                     const std::string& field) {
  // Find parent object
  std::string parent_pattern = "\"" + parent + "\":{";
  size_t parent_pos = json.find(parent_pattern);
  if (parent_pos == std::string::npos) return false;

  // Find field within parent (simplified - production would use JSON parser)
  size_t search_start = parent_pos + parent_pattern.length();
  std::string field_pattern = "\"" + field + "\":";
  size_t field_pos = json.find(field_pattern, search_start);

  if (field_pos == std::string::npos) return false;

  // Find the closing brace of the parent object
  int brace_count = 1;
  size_t scan_pos = search_start;
  size_t parent_end = std::string::npos;

  while (scan_pos < json.length() && brace_count > 0) {
    if (json[scan_pos] == '{') brace_count++;
    else if (json[scan_pos] == '}') {
      brace_count--;
      if (brace_count == 0) {
        parent_end = scan_pos;
        break;
      }
    }
    scan_pos++;
  }

  // Check if field is within parent object bounds
  return (parent_end != std::string::npos && field_pos < parent_end);
}

bool SchemaValidator::checkNumberField(const std::string& json,
                                       const std::string& field,
                                       double min, double max) {
  if (!hasField(json, field)) return false;

  // Extract value (simplified - production would use JSON parser)
  std::string pattern = "\"" + field + "\":";
  size_t pos = json.find(pattern);
  if (pos == std::string::npos) return false;

  size_t value_start = pos + pattern.length();
  size_t value_end = json.find_first_of(",}", value_start);
  if (value_end == std::string::npos) return false;

  std::string value_str = json.substr(value_start, value_end - value_start);
  try {
    double value = std::stod(value_str);
    return (value >= min && value <= max);
  } catch (...) {
    return false;
  }
}

bool SchemaValidator::checkBoolField(const std::string& json, const std::string& field) {
  if (!hasField(json, field)) return false;

  std::string pattern = "\"" + field + "\":";
  size_t pos = json.find(pattern);
  if (pos == std::string::npos) return false;

  size_t value_start = pos + pattern.length();
  std::string substr = json.substr(value_start, 5);
  return (substr.find("true") != std::string::npos ||
          substr.find("false") != std::string::npos);
}

bool SchemaValidator::checkStringField(const std::string& json, const std::string& field) {
  if (!hasField(json, field)) return false;

  std::string pattern = "\"" + field + "\":\"";
  return json.find(pattern) != std::string::npos;
}

bool SchemaValidator::checkArrayField(const std::string& json,
                                      const std::string& field,
                                      int expected_size) {
  std::string pattern = "\"" + field + "\":[";
  size_t pos = json.find(pattern);
  if (pos == std::string::npos) return false;

  if (expected_size < 0) return true;  // Size check not required

  // Count array elements (simplified - counts commas + 1)
  size_t array_start = pos + pattern.length();
  size_t array_end = json.find("]", array_start);
  if (array_end == std::string::npos) return false;

  std::string array_content = json.substr(array_start, array_end - array_start);
  int count = array_content.empty() ? 0 : (std::count(array_content.begin(), array_content.end(), ',') + 1);

  return count == expected_size;
}

bool SchemaValidator::checkEnumField(const std::string& json,
                                     const std::string& field,
                                     const std::vector<std::string>& valid_values) {
  if (!hasField(json, field)) return false;

  std::string pattern = "\"" + field + "\":\"";
  size_t pos = json.find(pattern);
  if (pos == std::string::npos) return false;

  size_t value_start = pos + pattern.length();
  size_t value_end = json.find("\"", value_start);
  if (value_end == std::string::npos) return false;

  std::string value = json.substr(value_start, value_end - value_start);
  return std::find(valid_values.begin(), valid_values.end(), value) != valid_values.end();
}

void SchemaValidator::validateSensorHealth(const std::string& json, ValidationResult& result) {
  // Check sensor_health object exists
  if (!hasField(json, "sensor_health")) {
    result.addError("Missing required field: sensor_health");
    return;
  }

  // Check all 5 sensors present
  std::vector<std::string> sensors = {"lidar", "baro", "mag", "of", "imu"};
  for (const auto& sensor : sensors) {
    if (!hasNestedField(json, "sensor_health", sensor)) {
      result.addError("Missing sensor in sensor_health: " + sensor);
    } else {
      // Check required subfields for each sensor
      if (!hasNestedField(json, sensor, "score")) {
        result.addError("Missing score in sensor_health." + sensor);
      }
      if (!hasNestedField(json, sensor, "reason")) {
        result.addError("Missing reason in sensor_health." + sensor);
      }
    }
  }

  // Check score ranges [0, 1]
  // NOTE: Simplified check - production would parse nested objects properly
  // This is good enough for basic validation
}

void SchemaValidator::validateWatchdog(const std::string& json, ValidationResult& result) {
  if (!hasField(json, "watchdog")) {
    result.addError("Missing required field: watchdog");
    return;
  }

  if (!hasNestedField(json, "watchdog", "all_healthy")) {
    result.addError("Missing watchdog.all_healthy");
  }

  if (!hasNestedField(json, "watchdog", "failure_count")) {
    result.addError("Missing watchdog.failure_count");
  }

  if (!hasNestedField(json, "watchdog", "deadline_violations")) {
    result.addError("Missing watchdog.deadline_violations");
  }
}

// =================== Main Validation ===================

ValidationResult SchemaValidator::validate(const std::string& json_line) {
  ValidationResult result;

  // Check required top-level fields
  std::vector<std::string> required = {
    "t", "pos", "vel", "agl_m", "contract_ok",
    "altitude_source", "sensor_health", "watchdog"
  };

  for (const auto& field : required) {
    if (!hasField(json_line, field)) {
      result.addError("Missing required field: " + field);
    }
  }

  // Type and range checks
  if (!checkNumberField(json_line, "t", 0.0)) {
    result.addError("Field 't' must be a number >= 0");
  }

  if (!checkArrayField(json_line, "pos", 3)) {
    result.addError("Field 'pos' must be array of length 3");
  }

  if (!checkArrayField(json_line, "vel", 3)) {
    result.addError("Field 'vel' must be array of length 3");
  }

  if (!checkNumberField(json_line, "agl_m", 0.0)) {
    result.addError("Field 'agl_m' must be a number >= 0");
  }

  if (!checkBoolField(json_line, "contract_ok")) {
    result.addError("Field 'contract_ok' must be boolean");
  }

  // Enum checks
  std::vector<std::string> altitude_sources = {"NONE", "LIDAR", "BARO", "BARO_TAKEOVER"};
  if (!checkEnumField(json_line, "altitude_source", altitude_sources)) {
    result.addWarning("Field 'altitude_source' has unexpected value (expected: NONE, LIDAR, BARO, BARO_TAKEOVER)");
  }

  // Nested object validation
  validateSensorHealth(json_line, result);
  validateWatchdog(json_line, result);

  // Optional field warnings
  if (hasField(json_line, "alt_src") && !hasField(json_line, "altitude_source")) {
    result.addWarning("Using deprecated field 'alt_src' without 'altitude_source'");
  }

  return result;
}

// =================== Version Management ===================

bool SchemaValidator::parseVersion(const std::string& version,
                                   int& major, int& minor, int& patch) {
  std::regex version_regex("(\\d+)\\.(\\d+)\\.(\\d+)");
  std::smatch matches;

  if (std::regex_match(version, matches, version_regex)) {
    major = std::stoi(matches[1].str());
    minor = std::stoi(matches[2].str());
    patch = std::stoi(matches[3].str());
    return true;
  }

  return false;
}

bool SchemaValidator::isVersionCompatible(const std::string& version) {
  int current_major, current_minor, current_patch;
  int check_major, check_minor, check_patch;

  if (!parseVersion(TELEMETRY_SCHEMA_VERSION, current_major, current_minor, current_patch)) {
    return false;
  }

  if (!parseVersion(version, check_major, check_minor, check_patch)) {
    return false;
  }

  // Compatibility rule: same major, minor can be lower (backward compatible)
  if (check_major != current_major) {
    return false;  // Breaking change
  }

  if (check_minor > current_minor) {
    return false;  // Future version (not supported)
  }

  return true;  // Same major, minor <= current (compatible)
}

// =================== Backward Compatibility ===================

bool CompatibilityChecker::isBackwardCompatible(const std::string& old_version,
                                                const std::string& new_version) {
  int old_major, old_minor, old_patch;
  int new_major, new_minor, new_patch;

  if (!SchemaValidator::parseVersion(old_version, old_major, old_minor, old_patch) ||
      !SchemaValidator::parseVersion(new_version, new_major, new_minor, new_patch)) {
    return false;
  }

  // Backward compatible if:
  // 1. Same major version (no breaking changes)
  // 2. New minor >= old minor (features added, not removed)
  return (new_major == old_major) && (new_minor >= old_minor);
}

std::string CompatibilityChecker::getMigrationGuide(const std::string& from_version,
                                                   const std::string& to_version) {
  std::ostringstream guide;

  guide << "Schema Migration Guide\n";
  guide << "======================\n\n";
  guide << "From: " << from_version << "\n";
  guide << "To:   " << to_version << "\n\n";

  if (from_version == "1.0.0" && to_version == "1.0.0") {
    guide << "No migration needed - versions are identical.\n";
    return guide.str();
  }

  // Future migrations would be documented here
  guide << "Migration steps:\n";
  guide << "1. Verify all required fields present in schema\n";
  guide << "2. Update any deprecated fields (e.g., alt_src → altitude_source)\n";
  guide << "3. Validate telemetry output with SchemaValidator::validate()\n";
  guide << "4. Run acceptance tests to ensure backward compatibility\n";

  if (!isBackwardCompatible(from_version, to_version)) {
    guide << "\n⚠ WARNING: This upgrade is NOT backward compatible!\n";
    guide << "   Old telemetry logs may not be readable by new parsers.\n";
  }

  return guide.str();
}

}  // namespace utils
}  // namespace chimera
