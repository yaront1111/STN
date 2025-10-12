#pragma once

#include <string>
#include <vector>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace aion {

/**
 * Configuration loader (thread-safe singleton).
 *
 * Features:
 *  - Load from file (or env var AION_CONFIG) into a global YAML root
 *  - Dotted-path getters: getPath<T>("ukf.proc_noise.gyro_rw", 0.02)
 *  - Section helpers: getCommon(), getUKF(), etc.
 *  - Pretty toString() for debugging
 */
class Config {
 public:
  // Load configuration from a YAML file path. Returns true on success.
  static bool load(const std::string& config_file);

  // Try load from env var AION_CONFIG; falls back to given default path if provided.
  static bool loadFromEnv(const std::string& fallback_path = "");

  // Has a config been loaded?
  static bool isLoaded();

  // Root YAML node (const-ref; may be !isLoaded()).
  static const YAML::Node& get();

  // Section helpers (may return undefined nodes if missing).
  static YAML::Node getCommon();
  static YAML::Node getUKF();
  static YAML::Node getTRN();
  static YAML::Node getSTN();
  static YAML::Node getRadarOdom();
  static YAML::Node getVIO();
  static YAML::Node getSoOP();
  static YAML::Node getUWB();
  static YAML::Node getIntegrity();
  static YAML::Node getLogging();
  static YAML::Node getBaro();

  // Top-level key getter (NOT dotted); returns default on miss/type error.
  template <typename T>
  static T getTop(const std::string& key, const T& default_value) {
    try {
      if (!root_ || !root_[key]) return default_value;
      return root_[key].as<T>();
    } catch (...) { return default_value; }
  }

  // Dotted-path getter: e.g., "ukf.proc_noise.gyro_rw"
  template <typename T>
  static T getPath(const std::string& dotted_path, const T& default_value) {
    try {
      YAML::Node node = resolvePath(dotted_path);
      if (!node) return default_value;
      return node.as<T>();
    } catch (...) { return default_value; }
  }

  // True if a dotted path exists.
  static bool hasPath(const std::string& dotted_path);

  // Print the loaded YAML as a string (pretty).
  static std::string toString();

 private:
  // Resolve "a.b.c" into a YAML::Node (may be null).
  static YAML::Node resolvePath(const std::string& dotted_path);

  static YAML::Node root_;
  static bool loaded_;
  static std::mutex mutex_;
};

/**
 * Strongly-typed, frequently-used slice of config for fast access.
 * Notes:
 *  - Gyro noise entries with unit comments in deg are converted to rad internally.
 */
struct SystemConfig {
  // Common parameters
  std::string frame = "NED";
  struct {
    double lat   = 32.0853;
    double lon   = 34.7818;
    double alt_m = 20.0;
  } origin;
  std::string gravity_model = "wgs84";

  // UKF parameters
  struct {
    double rate_hz = 200.0;
    struct {
      // Stored internally in **rad** units to match filters.
      double gyro_rw = 0.02; // rad/s/√Hz  (from deg in YAML, we convert)
      double acc_rw  = 0.03; // m/s^2/√Hz
      double bg_rw   = 0.0005; // rad/s^2/√Hz (from deg in YAML)
      double ba_rw   = 0.0005; // m/s^3/√Hz
    } proc_noise;
  } ukf;

  // TRN parameters
  struct {
    std::string dem_path     = "data/dem/srtm";
    double      min_slope_deg= 0.5;
    double      chi2_gate_1d = 6.63;
  } trn;

  // Radar odometry
  struct {
    double      rate_hz     = 15.0;
    std::string robust_loss = "huber";
    double      max_range_m = 120.0;
  } radar_odom;

  // Load values from the global YAML (safe if not loaded; keeps defaults).
  void loadFromYAML();
};

}  // namespace aion
