// Unit tests for CHIMERA Telemetry Schema Validation
// Tests schema compliance, version management, and backward compatibility

#include <gtest/gtest.h>
#include "utils/schema_validator.h"
#include <sstream>

using namespace chimera::utils;

// =================== Test Helpers ===================

// Minimal valid telemetry JSON
std::string getValidTelemetry() {
  std::ostringstream json;
  json << "{"
       << "\"t\":1.5,"
       << "\"pos\":[0,0,-2.5],"
       << "\"vel\":[0.1,0.2,0],"
       << "\"agl_m\":2.5,"
       << "\"contract_ok\":true,"
       << "\"altitude_source\":\"LIDAR\","
       << "\"sensor_health\":{"
         << "\"lidar\":{\"score\":1.0,\"is_stuck\":false,\"noise_level\":0.01,\"reason\":\"\"},"
         << "\"baro\":{\"score\":0.9,\"noise_level\":0.1,\"drift_rate\":0.01,\"rapid_change\":false,\"reason\":\"\"},"
         << "\"mag\":{\"score\":1.0,\"norm\":1.0,\"interference\":false,\"reason\":\"\"},"
         << "\"of\":{\"score\":0.8,\"texture_quality\":100,\"saturated\":false,\"reason\":\"\"},"
         << "\"imu\":{\"score\":1.0,\"temperature\":25.0,\"gyro_saturated\":false,\"accel_saturated\":false,\"reason\":\"\"}"
       << "},"
       << "\"watchdog\":{"
         << "\"all_healthy\":true,"
         << "\"failure_count\":0,"
         << "\"deadline_violations\":{"
           << "\"has_violations\":false,"
           << "\"timed_out_sensors\":[]"
         << "}"
       << "}"
       << "}";
  return json.str();
}

// =================== Schema Version Tests ===================

TEST(SchemaVersionTest, CurrentVersionFormat) {
  std::string version = SchemaValidator::getSchemaVersion();
  EXPECT_EQ(version, "1.0.0");

  int major, minor, patch;
  EXPECT_TRUE(SchemaValidator::parseVersion(version, major, minor, patch));
  EXPECT_EQ(major, 1);
  EXPECT_EQ(minor, 0);
  EXPECT_EQ(patch, 0);
}

TEST(SchemaVersionTest, ParseValidVersions) {
  int major, minor, patch;

  EXPECT_TRUE(SchemaValidator::parseVersion("1.2.3", major, minor, patch));
  EXPECT_EQ(major, 1);
  EXPECT_EQ(minor, 2);
  EXPECT_EQ(patch, 3);

  EXPECT_TRUE(SchemaValidator::parseVersion("10.20.30", major, minor, patch));
  EXPECT_EQ(major, 10);
  EXPECT_EQ(minor, 20);
  EXPECT_EQ(patch, 30);
}

TEST(SchemaVersionTest, ParseInvalidVersions) {
  int major, minor, patch;

  EXPECT_FALSE(SchemaValidator::parseVersion("1.0", major, minor, patch));
  EXPECT_FALSE(SchemaValidator::parseVersion("v1.0.0", major, minor, patch));
  EXPECT_FALSE(SchemaValidator::parseVersion("1.0.0-beta", major, minor, patch));
  EXPECT_FALSE(SchemaValidator::parseVersion("invalid", major, minor, patch));
}

TEST(SchemaVersionTest, CompatibilitySameMajor) {
  // Same major, same minor - compatible
  EXPECT_TRUE(SchemaValidator::isVersionCompatible("1.0.0"));

  // Same major, lower minor - compatible (backward compatible)
  // (future: if we're at 1.1.0, logs from 1.0.0 should still be readable)
}

TEST(SchemaVersionTest, CompatibilityDifferentMajor) {
  // Different major - incompatible (breaking change)
  EXPECT_FALSE(SchemaValidator::isVersionCompatible("2.0.0"));
  EXPECT_FALSE(SchemaValidator::isVersionCompatible("0.9.0"));
}

// =================== Basic Validation Tests ===================

TEST(SchemaValidationTest, ValidTelemetryPasses) {
  std::string json = getValidTelemetry();
  auto result = SchemaValidator::validate(json);

  EXPECT_TRUE(result.valid) << result.detailedReport();
  EXPECT_EQ(result.errors.size(), 0);
}

TEST(SchemaValidationTest, MissingRequiredField_T) {
  std::string json = getValidTelemetry();
  // Remove "t" field
  size_t pos = json.find("\"t\":");
  json.erase(pos, json.find(",", pos) - pos + 1);

  auto result = SchemaValidator::validate(json);
  EXPECT_FALSE(result.valid);
  EXPECT_GT(result.errors.size(), 0);

  // Check error message mentions the missing field
  bool found_t_error = false;
  for (const auto& err : result.errors) {
    if (err.find("t") != std::string::npos) {
      found_t_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_t_error);
}

TEST(SchemaValidationTest, MissingRequiredField_Pos) {
  std::string json = getValidTelemetry();
  size_t pos = json.find("\"pos\":");
  json.erase(pos, json.find("],", pos) - pos + 2);

  auto result = SchemaValidator::validate(json);
  EXPECT_FALSE(result.valid);

  bool found_pos_error = false;
  for (const auto& err : result.errors) {
    if (err.find("pos") != std::string::npos) {
      found_pos_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_pos_error);
}

TEST(SchemaValidationTest, MissingSensorHealth) {
  std::string json = getValidTelemetry();
  size_t pos = json.find("\"sensor_health\":");
  json.erase(pos, json.find("},", pos) - pos + 2);

  auto result = SchemaValidator::validate(json);
  EXPECT_FALSE(result.valid);

  bool found_sh_error = false;
  for (const auto& err : result.errors) {
    if (err.find("sensor_health") != std::string::npos) {
      found_sh_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_sh_error);
}

TEST(SchemaValidationTest, MissingWatchdog) {
  std::string json = getValidTelemetry();
  size_t pos = json.find("\"watchdog\":");
  json.erase(pos, json.find("}", pos));

  auto result = SchemaValidator::validate(json);
  EXPECT_FALSE(result.valid);

  bool found_wd_error = false;
  for (const auto& err : result.errors) {
    if (err.find("watchdog") != std::string::npos) {
      found_wd_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_wd_error);
}

// =================== Type Validation Tests ===================

TEST(SchemaValidationTest, InvalidTimestamp_Negative) {
  std::string json = getValidTelemetry();
  // Replace "t":1.5 with "t":-1.0
  size_t pos = json.find("\"t\":1.5");
  json.replace(pos, 7, "\"t\":-1.0");

  auto result = SchemaValidator::validate(json);
  EXPECT_FALSE(result.valid);

  bool found_t_error = false;
  for (const auto& err : result.errors) {
    if (err.find("t") != std::string::npos && err.find(">= 0") != std::string::npos) {
      found_t_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_t_error);
}

TEST(SchemaValidationTest, InvalidAGL_Negative) {
  std::string json = getValidTelemetry();
  size_t pos = json.find("\"agl_m\":2.5");
  json.replace(pos, 11, "\"agl_m\":-1.0");

  auto result = SchemaValidator::validate(json);
  EXPECT_FALSE(result.valid);

  bool found_agl_error = false;
  for (const auto& err : result.errors) {
    if (err.find("agl_m") != std::string::npos) {
      found_agl_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_agl_error);
}

TEST(SchemaValidationTest, InvalidContractOk_NotBool) {
  std::string json = getValidTelemetry();
  size_t pos = json.find("\"contract_ok\":true");
  json.replace(pos, 18, "\"contract_ok\":123");

  auto result = SchemaValidator::validate(json);
  EXPECT_FALSE(result.valid);

  bool found_contract_error = false;
  for (const auto& err : result.errors) {
    if (err.find("contract_ok") != std::string::npos) {
      found_contract_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_contract_error);
}

// =================== Enum Validation Tests ===================

TEST(SchemaValidationTest, ValidAltitudeSource) {
  std::vector<std::string> valid_sources = {"NONE", "LIDAR", "BARO", "BARO_TAKEOVER"};

  for (const auto& source : valid_sources) {
    std::string json = getValidTelemetry();
    size_t pos = json.find("\"altitude_source\":\"LIDAR\"");
    json.replace(pos, 26, "\"altitude_source\":\"" + source + "\"");

    auto result = SchemaValidator::validate(json);
    EXPECT_TRUE(result.valid) << "Source: " << source << "\n" << result.detailedReport();
  }
}

TEST(SchemaValidationTest, InvalidAltitudeSource) {
  std::string json = getValidTelemetry();
  size_t pos = json.find("\"altitude_source\":\"LIDAR\"");
  json.replace(pos, 26, "\"altitude_source\":\"INVALID\"");

  auto result = SchemaValidator::validate(json);
  // Should produce warning (not error)
  EXPECT_GT(result.warnings.size(), 0);
}

// =================== Array Validation Tests ===================

TEST(SchemaValidationTest, PosArray_CorrectSize) {
  std::string json = getValidTelemetry();
  auto result = SchemaValidator::validate(json);

  EXPECT_TRUE(result.valid) << result.detailedReport();
}

TEST(SchemaValidationTest, PosArray_WrongSize) {
  std::string json = getValidTelemetry();
  // Change pos to 2 elements
  size_t pos = json.find("\"pos\":[0,0,-2.5]");
  json.replace(pos, 17, "\"pos\":[0,0]");

  auto result = SchemaValidator::validate(json);
  EXPECT_FALSE(result.valid);

  bool found_pos_error = false;
  for (const auto& err : result.errors) {
    if (err.find("pos") != std::string::npos && err.find("length 3") != std::string::npos) {
      found_pos_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_pos_error);
}

// =================== Nested Object Validation Tests ===================

TEST(SchemaValidationTest, SensorHealth_AllSensorsPresent) {
  std::string json = getValidTelemetry();
  auto result = SchemaValidator::validate(json);

  EXPECT_TRUE(result.valid) << result.detailedReport();
}

TEST(SchemaValidationTest, SensorHealth_MissingSensor) {
  std::string json = getValidTelemetry();
  // Remove "lidar" sensor
  size_t pos = json.find("\"lidar\":{");
  size_t end = json.find("},", pos);
  json.erase(pos, end - pos + 2);

  auto result = SchemaValidator::validate(json);
  EXPECT_FALSE(result.valid);

  bool found_lidar_error = false;
  for (const auto& err : result.errors) {
    if (err.find("lidar") != std::string::npos) {
      found_lidar_error = true;
      break;
    }
  }
  EXPECT_TRUE(found_lidar_error);
}

TEST(SchemaValidationTest, Watchdog_RequiredFields) {
  std::string json = getValidTelemetry();
  auto result = SchemaValidator::validate(json);

  EXPECT_TRUE(result.valid) << result.detailedReport();
}

// =================== Backward Compatibility Tests ===================

TEST(BackwardCompatibilityTest, SameVersion_Compatible) {
  EXPECT_TRUE(CompatibilityChecker::isBackwardCompatible("1.0.0", "1.0.0"));
}

TEST(BackwardCompatibilityTest, MinorUpgrade_Compatible) {
  EXPECT_TRUE(CompatibilityChecker::isBackwardCompatible("1.0.0", "1.1.0"));
  EXPECT_TRUE(CompatibilityChecker::isBackwardCompatible("1.1.0", "1.2.0"));
}

TEST(BackwardCompatibilityTest, MajorUpgrade_Incompatible) {
  EXPECT_FALSE(CompatibilityChecker::isBackwardCompatible("1.0.0", "2.0.0"));
  EXPECT_FALSE(CompatibilityChecker::isBackwardCompatible("1.5.0", "2.0.0"));
}

TEST(BackwardCompatibilityTest, MinorDowngrade_Incompatible) {
  EXPECT_FALSE(CompatibilityChecker::isBackwardCompatible("1.2.0", "1.1.0"));
}

TEST(BackwardCompatibilityTest, MigrationGuide_SameVersion) {
  std::string guide = CompatibilityChecker::getMigrationGuide("1.0.0", "1.0.0");

  EXPECT_TRUE(guide.find("No migration needed") != std::string::npos);
}

TEST(BackwardCompatibilityTest, MigrationGuide_BreakingChange) {
  std::string guide = CompatibilityChecker::getMigrationGuide("1.0.0", "2.0.0");

  EXPECT_TRUE(guide.find("WARNING") != std::string::npos);
  EXPECT_TRUE(guide.find("NOT backward compatible") != std::string::npos);
}

// =================== Validation Result Tests ===================

TEST(ValidationResultTest, Summary_Valid) {
  ValidationResult result;
  result.valid = true;

  std::string summary = result.summary();
  EXPECT_TRUE(summary.find("PASSED") != std::string::npos);
}

TEST(ValidationResultTest, Summary_Invalid) {
  ValidationResult result;
  result.addError("Test error");

  std::string summary = result.summary();
  EXPECT_TRUE(summary.find("FAILED") != std::string::npos);
  EXPECT_TRUE(summary.find("1 errors") != std::string::npos);
}

TEST(ValidationResultTest, DetailedReport_WithErrors) {
  ValidationResult result;
  result.addError("Error 1");
  result.addError("Error 2");
  result.addWarning("Warning 1");

  std::string report = result.detailedReport();
  EXPECT_TRUE(report.find("Error 1") != std::string::npos);
  EXPECT_TRUE(report.find("Error 2") != std::string::npos);
  EXPECT_TRUE(report.find("Warning 1") != std::string::npos);
}

// =================== Deprecated Field Warnings ===================

TEST(SchemaValidationTest, DeprecatedField_AltSrc) {
  std::ostringstream json;
  json << "{"
       << "\"t\":1.5,"
       << "\"pos\":[0,0,-2.5],"
       << "\"vel\":[0.1,0.2,0],"
       << "\"agl_m\":2.5,"
       << "\"contract_ok\":true,"
       << "\"alt_src\":\"lidar\","  // Deprecated field without altitude_source
       << "\"sensor_health\":{"
         << "\"lidar\":{\"score\":1.0,\"is_stuck\":false,\"noise_level\":0.01,\"reason\":\"\"},"
         << "\"baro\":{\"score\":0.9,\"noise_level\":0.1,\"drift_rate\":0.01,\"rapid_change\":false,\"reason\":\"\"},"
         << "\"mag\":{\"score\":1.0,\"norm\":1.0,\"interference\":false,\"reason\":\"\"},"
         << "\"of\":{\"score\":0.8,\"texture_quality\":100,\"saturated\":false,\"reason\":\"\"},"
         << "\"imu\":{\"score\":1.0,\"temperature\":25.0,\"gyro_saturated\":false,\"accel_saturated\":false,\"reason\":\"\"}"
       << "},"
       << "\"watchdog\":{"
         << "\"all_healthy\":true,"
         << "\"failure_count\":0,"
         << "\"deadline_violations\":{"
           << "\"has_violations\":false,"
           << "\"timed_out_sensors\":[]"
         << "}"
       << "}"
       << "}";

  auto result = SchemaValidator::validate(json.str());

  // Should fail because altitude_source is required
  EXPECT_FALSE(result.valid);

  // But should also have warning about deprecated field
  bool found_deprecation_warning = false;
  for (const auto& warn : result.warnings) {
    if (warn.find("alt_src") != std::string::npos) {
      found_deprecation_warning = true;
      break;
    }
  }
  EXPECT_TRUE(found_deprecation_warning);
}
