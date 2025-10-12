#include "core/utils/config.h"
#include "core/utils/logging.h"

#include <filesystem>
#include <sstream>
#include <mutex>
#include <iomanip>
#include <yaml-cpp/yaml.h>

namespace aion {

YAML::Node Config::root_;
bool Config::loaded_ = false;
std::mutex Config::mutex_;

static std::vector<std::string> splitDots(const std::string& s) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == '.') {
      if (!cur.empty()) out.push_back(cur);
      cur.clear();
    } else cur.push_back(c);
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

bool Config::load(const std::string& config_file) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    if (!config_file.empty()) {
      std::filesystem::path p(config_file);
      if (!std::filesystem::exists(p)) {
        LOG_ERROR("Config file does not exist: {}", config_file);
        loaded_ = false;
        root_ = YAML::Node();
        return false;
      }
    }
    root_ = YAML::LoadFile(config_file);
    loaded_ = true;
    LOG_INFO("Configuration loaded from: {}", config_file);
    // Don't call toString() here as it would deadlock (tries to acquire mutex we already hold)
    // LOG_DEBUG("Config preview:\n{}", toString());
    return true;
  } catch (const YAML::Exception& e) {
    LOG_ERROR("YAML parse error in '{}': {}", config_file, e.what());
    loaded_ = false;
    root_ = YAML::Node();
    return false;
  } catch (const std::exception& e) {
    LOG_ERROR("Config load error: {}", e.what());
    loaded_ = false;
    root_ = YAML::Node();
    return false;
  }
}

bool Config::loadFromEnv(const std::string& fallback_path) {
  const char* envp = std::getenv("AION_CONFIG");
  if (envp && *envp) {
    LOG_INFO("AION_CONFIG set → {}", envp);
    return load(std::string(envp));
  }
  if (!fallback_path.empty()) {
    LOG_INFO("AION_CONFIG not set; trying fallback {}", fallback_path);
    return load(fallback_path);
  }
  LOG_WARN("AION_CONFIG not set and no fallback provided; config remains unloaded.");
  return false;
}

bool Config::isLoaded() {
  std::lock_guard<std::mutex> lock(mutex_);
  return loaded_;
}

const YAML::Node& Config::get() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!loaded_) {
    static YAML::Node empty;
    return empty;
  }
  return root_;
}

YAML::Node Config::getCommon()    { return resolvePath("common"); }
YAML::Node Config::getUKF()       { return resolvePath("ukf"); }
YAML::Node Config::getTRN()       { return resolvePath("trn"); }
YAML::Node Config::getSTN()       { return resolvePath("stn"); }
YAML::Node Config::getBaro()      { return resolvePath("baro"); }
YAML::Node Config::getRadarOdom() { return resolvePath("radar_odom"); }
YAML::Node Config::getVIO()       { return resolvePath("vio"); }
YAML::Node Config::getSoOP()      { return resolvePath("sop"); }
YAML::Node Config::getUWB()       { return resolvePath("uwb"); }
YAML::Node Config::getIntegrity() { return resolvePath("integrity"); }
YAML::Node Config::getLogging()   { return resolvePath("logging"); }

bool Config::hasPath(const std::string& dotted_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  YAML::Node n = resolvePath(dotted_path);
  return static_cast<bool>(n);
}

YAML::Node Config::resolvePath(const std::string& dotted_path) {
  // No lock; caller must hold mutex_ if needed.
  YAML::Node cur = root_;
  for (const auto& key : splitDots(dotted_path)) {
    if (!cur || !cur[key]) return YAML::Node();
    cur = cur[key];
  }
  return cur;
}

std::string Config::toString() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!loaded_) return std::string("Config not loaded");
  YAML::Emitter out;
  out.SetIndent(2);
  out << root_;
  return std::string(out.c_str() ? out.c_str() : "");
}

//----------------- SystemConfig -----------------

void SystemConfig::loadFromYAML() {
  const auto root = Config::get();
  if (!root || root.IsNull()) {
    LOG_WARN("Config not loaded; SystemConfig using defaults.");
    return;
  }

  // Common
  if (auto common = Config::getCommon()) {
    if (common["frame"]) frame = common["frame"].as<std::string>();
    if (auto origin_node = common["origin"]) {
      if (origin_node["lat"])   origin.lat   = origin_node["lat"].as<double>();
      if (origin_node["lon"])   origin.lon   = origin_node["lon"].as<double>();
      if (origin_node["alt_m"]) origin.alt_m = origin_node["alt_m"].as<double>();
    }
    if (common["gravity_model"]) gravity_model = common["gravity_model"].as<std::string>();
  }

  // UKF
  if (auto ukf_node = Config::getUKF()) {
    if (ukf_node["rate_hz"]) ukf.rate_hz = ukf_node["rate_hz"].as<double>();

    if (auto pn = ukf_node["proc_noise"]) {
      // YAML comments indicated deg-based units for some entries; convert to rad.
      auto to_rad = [](double deg_val) { return deg_val * M_PI / 180.0; };

      if (pn["gyro_rw"]) {
        double v = pn["gyro_rw"].as<double>();  // possibly (deg/s)/√Hz
        // Heuristic: if value looks like deg (typical ~0.01–0.1 deg), convert.
        ukf.proc_noise.gyro_rw = to_rad(v);
      }
      if (pn["acc_rw"])  ukf.proc_noise.acc_rw = pn["acc_rw"].as<double>();
      if (pn["bg_rw"]) {
        double v = pn["bg_rw"].as<double>();   // possibly (deg/s^2)/√Hz
        ukf.proc_noise.bg_rw = to_rad(v);
      }
      if (pn["ba_rw"])  ukf.proc_noise.ba_rw = pn["ba_rw"].as<double>();
    }
  }

  // TRN
  if (auto trn_node = Config::getTRN()) {
    if (trn_node["dem_path"])      trn.dem_path      = trn_node["dem_path"].as<std::string>();
    if (trn_node["min_slope_deg"]) trn.min_slope_deg = trn_node["min_slope_deg"].as<double>();
    if (trn_node["chi2_gate_1d"])  trn.chi2_gate_1d  = trn_node["chi2_gate_1d"].as<double>();
  }

  // Radar Odom
  if (auto ro_node = Config::getRadarOdom()) {
    if (ro_node["rate_hz"])     radar_odom.rate_hz     = ro_node["rate_hz"].as<double>();
    if (ro_node["robust_loss"]) radar_odom.robust_loss = ro_node["robust_loss"].as<std::string>();
    if (ro_node["max_range_m"]) radar_odom.max_range_m = ro_node["max_range_m"].as<double>();
  }

  LOG_INFO("SystemConfig loaded:");
  LOG_INFO("  Frame: {}", frame);
  LOG_INFO("  Origin: lat={:.6f}, lon={:.6f}, alt={:.1f} m", origin.lat, origin.lon, origin.alt_m);
  LOG_INFO("  UKF: rate={:.0f} Hz, gyro_rw={:.6f} rad/s/√Hz, acc_rw={:.4f} m/s²/√Hz",
           ukf.rate_hz, ukf.proc_noise.gyro_rw, ukf.proc_noise.acc_rw);
  // TRN config logged by TRNManager (these SystemConfig values are not used)
  LOG_INFO("  RadarOdom: rate={:.0f} Hz, max_range={:.1f} m, loss={}",
           radar_odom.rate_hz, radar_odom.max_range_m, radar_odom.robust_loss);
}

} // namespace aion
