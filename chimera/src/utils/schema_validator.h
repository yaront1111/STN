#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <limits>

namespace chimera {
namespace utils {

/**
 * @brief Telemetry schema version
 *
 * Version format: MAJOR.MINOR.PATCH
 * - MAJOR: Breaking changes (incompatible schema)
 * - MINOR: Backward-compatible additions (new optional fields)
 * - PATCH: Documentation or clarifications only
 */
constexpr const char* TELEMETRY_SCHEMA_VERSION = "1.0.0";

/**
 * @brief Schema validation result
 */
struct ValidationResult {
  bool valid = true;
  std::vector<std::string> errors;
  std::vector<std::string> warnings;

  void addError(const std::string& msg) {
    valid = false;
    errors.push_back(msg);
  }

  void addWarning(const std::string& msg) {
    warnings.push_back(msg);
  }

  std::string summary() const {
    std::ostringstream oss;
    if (valid) {
      oss << "✓ Schema validation PASSED";
      if (!warnings.empty()) {
        oss << " (" << warnings.size() << " warnings)";
      }
    } else {
      oss << "✗ Schema validation FAILED (" << errors.size() << " errors)";
    }
    return oss.str();
  }

  std::string detailedReport() const {
    std::ostringstream oss;
    oss << summary() << "\n";

    if (!errors.empty()) {
      oss << "\nErrors:\n";
      for (const auto& err : errors) {
        oss << "  - " << err << "\n";
      }
    }

    if (!warnings.empty()) {
      oss << "\nWarnings:\n";
      for (const auto& warn : warnings) {
        oss << "  - " << warn << "\n";
      }
    }

    return oss.str();
  }
};

/**
 * @brief Lightweight telemetry schema validator
 *
 * NOTE: This is a simplified validator for production use.
 * For full JSON Schema validation, use external libraries like valijson.
 *
 * This validator checks:
 * - Required fields presence
 * - Type correctness (basic checks)
 * - Value ranges (min/max)
 * - Enum values
 * - Schema version compatibility
 */
class SchemaValidator {
 public:
  /**
   * @brief Validate telemetry JSON string against schema v1.0.0
   * @param json_line Single line of telemetry JSON
   * @return Validation result with errors and warnings
   */
  static ValidationResult validate(const std::string& json_line);

  /**
   * @brief Check if schema version is compatible
   * @param version Version string (e.g., "1.0.0")
   * @return true if compatible with current schema
   *
   * Compatibility rules:
   * - Same MAJOR version required
   * - MINOR version can be lower (backward compatible)
   * - PATCH version ignored
   */
  static bool isVersionCompatible(const std::string& version);

  /**
   * @brief Parse version string into major.minor.patch
   * @param version Version string (e.g., "1.2.3")
   * @param major Output major version
   * @param minor Output minor version
   * @param patch Output patch version
   * @return true if parsing successful
   */
  static bool parseVersion(const std::string& version,
                          int& major, int& minor, int& patch);

  /**
   * @brief Get current schema version
   */
  static std::string getSchemaVersion() {
    return TELEMETRY_SCHEMA_VERSION;
  }

 private:
  // Field existence checks
  static bool hasField(const std::string& json, const std::string& field);
  static bool hasNestedField(const std::string& json,
                             const std::string& parent,
                             const std::string& field);

  // Type and range checks
  static bool checkNumberField(const std::string& json,
                              const std::string& field,
                              double min = -std::numeric_limits<double>::infinity(),
                              double max = std::numeric_limits<double>::infinity());

  static bool checkBoolField(const std::string& json, const std::string& field);
  static bool checkStringField(const std::string& json, const std::string& field);
  static bool checkArrayField(const std::string& json, const std::string& field,
                             int expected_size = -1);

  // Enum validation
  static bool checkEnumField(const std::string& json,
                            const std::string& field,
                            const std::vector<std::string>& valid_values);

  // Sensor health validation
  static void validateSensorHealth(const std::string& json, ValidationResult& result);

  // Watchdog validation
  static void validateWatchdog(const std::string& json, ValidationResult& result);
};

/**
 * @brief Backward compatibility checker for telemetry upgrades
 *
 * When upgrading telemetry schema versions:
 * 1. Check if old logs can still be read
 * 2. Validate new fields have defaults
 * 3. Ensure no required fields were removed
 */
class CompatibilityChecker {
 public:
  /**
   * @brief Check if telemetry from old version can be read by new schema
   * @param old_version Old schema version (e.g., "1.0.0")
   * @param new_version New schema version (e.g., "1.1.0")
   * @return true if backward compatible
   */
  static bool isBackwardCompatible(const std::string& old_version,
                                   const std::string& new_version);

  /**
   * @brief Migration guide for schema version upgrades
   * @param from_version Starting version
   * @param to_version Target version
   * @return Human-readable migration steps
   */
  static std::string getMigrationGuide(const std::string& from_version,
                                      const std::string& to_version);
};

}  // namespace utils
}  // namespace chimera
