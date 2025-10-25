// CHIMERA Flight Computer - Refactored modular implementation
// Multi-sensor GPS-denied navigation with factor graph optimization

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <boost/json.hpp>

#include "core/contract_monitor.h"
#include "core/sensor_health.h"
#include "core/watchdogs.h"
#include "core/smoother_wrapper.h"
#include "core/ukf.h"

#include "factors/aero_velocity_wind_factor.h"
#include "factors/altimeter_range_factor.h"
#include "factors/baro_factor.h"
#include "factors/mag_yaw_factor.h"
#include "factors/optical_flow_factor.h"
#include "factors/velocity_magnitude_factor.h"

#include "utils/telemetry.h"
#include "utils/timing.h"
#include "utils/data_loaders.h"
#include "utils/sensor_helpers.h"
#include "utils/adaptive_constraints.h"
#include "utils/risk_budget.h"
#include "utils/schema_validator.h"
#include "utils/calibration.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace chimera {
namespace core {

using gtsam::symbol_shorthand::X;  // Pose
using gtsam::symbol_shorthand::V;  // Vel
using gtsam::symbol_shorthand::B;  // Bias
using gtsam::symbol_shorthand::W;  // Wind(2D)

// Import sensor types from utils
using chimera::utils::ImuSample;
using chimera::utils::MagSample;
using chimera::utils::LidarSample;
using chimera::utils::BaroSample;
using chimera::utils::AirspeedSample;
using chimera::utils::OpticalFlowSample;
using chimera::utils::OpticalFlowRay;
using chimera::utils::SensorData;

// =================== Helpers / Utilities ===================
enum class RangeUnits { AUTO, M, CM, MM };

// =================== OU Bias Process Helpers ===================
enum class BiasMode { BOOT, TRUSTED, LOST };

inline BiasMode bias_mode(bool lidar_trusted, bool of_healthy, int key){
  if(key <= 10) return BiasMode::BOOT;
  if(lidar_trusted && of_healthy) return BiasMode::TRUSTED;
  return BiasMode::LOST;
}

inline void ou_sigmas(double dt, BiasMode m,
                      double& accel_sig, double& gyro_sig,
                      double& phi_a_out, double& phi_g_out) {
  // Regime parameters (kappa: 1/s, sigma: bias diffusion units)
  double kappa_a, sigma_a, kappa_g, sigma_g;
  switch(m){
    case BiasMode::BOOT:    kappa_a=1.5; sigma_a=0.06; kappa_g=1.5; sigma_g=0.010; break;
    case BiasMode::TRUSTED: kappa_a=0.8; sigma_a=0.02; kappa_g=0.8; sigma_g=0.004; break;
    case BiasMode::LOST:    kappa_a=0.5; sigma_a=0.05; kappa_g=0.5; sigma_g=0.010; break;
  }

  auto Sd = [&](double kappa, double sigma, double& phi_out){
    phi_out = std::exp(-kappa*dt);               // AR(1) coefficient
    const double Qd = (sigma*sigma)/(2.0*kappa) * (1.0 - std::exp(-2.0*kappa*dt));
    return std::sqrt(std::max(Qd, 1e-12));       // numerical floor
  };

  accel_sig = Sd(kappa_a, sigma_a, phi_a_out);
  gyro_sig  = Sd(kappa_g, sigma_g, phi_g_out);
}

struct RangeStuckGuard {
  static constexpr size_t N = 8;
  static constexpr double thresh_var = 0.0001; // m^2; dual-gate threshold (10mm stddev)
  static constexpr double thresh_spread = 0.005; // m; dual-gate threshold (5mm range spread)
  std::deque<double> win;
  void add(double z){ win.push_back(z); if(win.size()>N) win.pop_front(); }
  bool isFull() const { return win.size()>=N; }
  bool check() const {
    if(!isFull()) return false;
    // Gate 1: Low variance
    double mean = std::accumulate(win.begin(), win.end(), 0.0)/win.size();
    double var=0.0; for(double x: win){ double d=x-mean; var+=d*d; } var/=win.size();
    bool low_variance = (var < thresh_var);
    // Gate 2: Low range spread
    double min_val = *std::min_element(win.begin(), win.end());
    double max_val = *std::max_element(win.begin(), win.end());
    double range_spread = max_val - min_val;
    bool low_spread = (range_spread < thresh_spread);
    // Dual-gate: Both must be true
    return low_variance && low_spread;
  }
  bool addAndCheck(double z){ add(z); return check(); }
  void reset(){ win.clear(); }
};

struct RangeBootstrap {
  bool ready=false;
  double scale_to_m=1.0;
  RangeUnits units=RangeUnits::AUTO;
  double z0_agl_m=0.0;
  bool lidar_stuck=true;
  RangeStuckGuard guard;
};

RangeBootstrap bootRangeUnits(const std::vector<LidarSample>& lidar, double t0, double horizon=2.0){
  RangeBootstrap boot;
  if(lidar.empty()) return boot;
  std::vector<double> w;
  for(const auto& s: lidar){ if(s.t-t0>horizon) break; if(s.range_mean>0) w.push_back(s.range_mean); }
  if(w.size()<5) return boot;
  double med = chimera::utils::medianScalar(w);
  if(med > 10000.0){ boot.scale_to_m=0.001; boot.units=RangeUnits::MM; }
  else if(med > 100.0){ boot.scale_to_m=0.01; boot.units=RangeUnits::CM; }
  else { boot.scale_to_m=1.0; boot.units=RangeUnits::M; }
  for(const auto& s: lidar){
    if(s.t-t0>horizon) break;
    if(s.range_mean>0){ boot.guard.add(s.range_mean*boot.scale_to_m); if(boot.guard.isFull()) break; }
  }
  boot.lidar_stuck = boot.guard.isFull()? boot.guard.check() : true;
  boot.ready = true;
  return boot;
}

template<typename T>
std::optional<T> nearestByTime(const std::vector<T>& v, double t, double max_dt){
  if(v.empty()) return std::nullopt;
  auto it = std::lower_bound(v.begin(), v.end(), t, [](const T& s, double tt){ return s.t<tt; });
  auto ok = [&](const T& s){ return std::abs(s.t-t)<=max_dt; };
  if(it==v.end()) return ok(v.back())? std::optional<T>(v.back()) : std::nullopt;
  if(it==v.begin()) return ok(*it)? std::optional<T>(*it) : std::nullopt;
  auto prev=it-1;
  return (std::abs(prev->t-t)<std::abs(it->t-t) ? (ok(*prev)? std::optional<T>(*prev):std::nullopt)
                                                : (ok(*it)?   std::optional<T>(*it)  :std::nullopt));
}

// Import utility functions
using chimera::utils::OrientBoot;
using chimera::utils::pickRwb_from_quat;
using chimera::utils::pickRwb_from_quat_agnostic;
using chimera::utils::medianAccel;
using chimera::utils::medianScalar;
using chimera::utils::yawFromMagTiltComp;
using chimera::utils::effectiveDepthMeters_AGL;

// =================== Config ===================
struct EstimatorConfig {
  // Timing
  double smoother_hz = 5.0;
  double reanchor_period_s = 3.0;
  double lag_seconds = 15.0;  // Increased from 7.0 to reduce marginalization losses
  double max_gap_s = 10.0;
  double boot_secs = 2.0;

  // Constant key-budget control (keeps conditioning stable across lag sizes)
  bool   auto_smoother_rate   = true;
  int    target_keys_in_lag   = 75;   // ~Phase-1 window size
  double smoother_hz_min      = 2.0;
  double smoother_hz_max      = 7.5;

  // Fallbacks
  bool enable_baro_fallback = true;  // Use barometer when LiDAR unavailable/stuck

  // UKF noise
  chimera::core::UKF::NoiseParams ukf_noise = []{
    chimera::core::UKF::NoiseParams n;
    n.gyro_noise_density  = 0.00016968;
    n.gyro_random_walk    = 1.9393e-05;
    n.accel_noise_density = 0.002;
    n.accel_random_walk   = 0.003;
    return n;
  }();

  // Gravity
  double g = 9.81;

  // Camera model / extrinsic
  double focal_px = 320.0;
  gtsam::Matrix3 R_bc = (Eigen::Matrix3d() <<
     0,  1,  0,
    -1,  0,  0,
     0,  0,  1).finished();

  // Gates / thresholds
  double mag_gate_frac = 0.30;     // ±30%
  double of_min_tex = 50.0;
  int    of_min_rays = 3;          // Lowered from 5 to allow more updates
  double of_max_flow = 200.0;      // px/s
  double of_rms_good = 4.0;        // px/s
  double of_rms_bad  = 8.0;        // px/s
  double lidar_min   = 0.3;        // m
  double lidar_max   = 50.0;       // m
  double z_prior1 = 0.05;          // key<=10
  double z_prior2 = 0.30;          // 11..25
  double z_prior3 = 1.00;          // 26..40
};

// =================== FlightComputer ===================
class FlightComputer {
public:
  FlightComputer(const EstimatorConfig& cfg, SensorData&& s, bool safe_boot,
                 const std::string& telem_path)
  : cfg_(cfg), sensors_(std::move(s)), safe_boot_(safe_boot),
    telem_(telem_path), ukf_(cfg_.ukf_noise),
    smoother_(cfg_.lag_seconds), monitor_(cfg_.max_gap_s) {
    init();
  }

  void run(){
    std::cout<<"Running...\n";
    std::size_t processed=0;
    for(const auto& imu: sensors_.imu){
      processImu(imu);
      if(processed%200==0) emitTelemetry(); // ~1 Hz at ~200 Hz IMU
      ++processed;
    }

    // End-of-run summary
    const bool contract_ok = monitor_.isContractOk(last_imu_t_.value_or(0.0));
    std::cout << "==========================================================\n";
    std::cout << "Run complete\n";
    std::cout << "Contract OK: " << (contract_ok ? "YES" : "NO") << "\n";
    std::cout << "Telemetry saved to: " << telem_path_ << "\n";
    std::cout << "==========================================================\n";
  }

private:
  EstimatorConfig cfg_;
  SensorData sensors_;
  bool safe_boot_ = true;
  std::string telem_path_ = "logs/chimera_multi.jsonl";

  chimera::utils::Telemetry telem_;
  chimera::core::UKF ukf_;
  chimera::core::SmootherWrapper smoother_;
  chimera::core::ContractMonitor monitor_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> pim_;
  std::optional<double> last_imu_t_;
  int key_ = 0;
  double last_smoother_t_ = -1e9;
  double last_reanchor_t_ = 0.0;

  // boot / orientation / units
  bool flip_nwu_to_frd_ = false;
  double gyro_scale_ = 1.0; // deg/s→rad/s when needed
  double baro0_msl_ = std::numeric_limits<double>::quiet_NaN();
  double init_agl_m_ = 0.0;
  gtsam::Rot3 init_Rwb_;

  // lidar bootstrap
  RangeBootstrap range_boot_;
  RangeStuckGuard lidar_guard_;
  bool boot_done_ = false;
  bool lidar_stuck_ = true;

  // mag health
  double mag_median_norm_ = 1.0;

  // OF health
  int of_bad_streak_ = 0;
  double of_bad_start_t_ = 0.0;

  // Baro filter state (robust: median + MAD over short window)
  std::deque<std::pair<double,double>> baro_buf_; // (t, altitude_msl)

  // counters
  int mag_updates_=0, lidar_updates_=0, baro_updates_=0, airspeed_updates_=0, of_updates_=0;
  int lidar_stuck_events_=0, of_rays_used_total_=0, of_rays_rejected_total_=0;
  int of_rays_used_=0, of_rays_rejected_=0; double of_resid_sum_sq_=0.0;
  double last_of_rms_=0.0; // Store RMS from most recent OF update

  // LiDAR healthy streak for bias prior schedule
  int lidar_healthy_streak_ = 0;

  // OU bias process: learned mean tracking
  gtsam::imuBias::ConstantBias learned_bias_mean_;
  int bias_mean_update_count_ = 0;

  // factor error tracking (diagnostic)
  double last_total_graph_error_ = 0.0;
  size_t last_lidar_factor_count_ = 0;
  double last_lidar_factor_error_ = 0.0;

  // first-trusted-LiDAR reset
  bool did_lidar_reset_ = false;
  bool force_reanchor_now_ = false;

  // baro takeover state management
  bool baro_takeover_active_ = false;
  bool takeover_just_activated_ = false;
  double saved_lag_seconds_ = 0.0;
  int imu_cut_keys_left_ = 0;

  // altitude memory (last trusted LiDAR)
  bool   has_trusted_alt_  = false;
  double last_trusted_agl_ = std::numeric_limits<double>::quiet_NaN();
  double last_trust_t_     = 0.0;

  // anchor telemetry
  bool last_anchor_applied_ = false;
  double last_anchor_age_s_ = 0.0;

  // CPU timing
  double ukf_cpu_ms_ = 0.0;
  double smoother_cpu_ms_ = 0.0;

  // FIX #7: Telemetry for validation
  bool vel_prior_added_ = false;
  double of_depth_scale_ = 1.0;
  bool enable_airspeed_ = false;
  std::string last_alt_src_ = "none";  // Track which altitude source was used

  // ===== Phase 1: Sensor Health Monitoring =====
  LidarHealthMonitor lidar_health_mon_;
  BaroHealthMonitor baro_health_mon_;
  MagHealthMonitor mag_health_mon_;
  OpticalFlowHealthMonitor of_health_mon_;
  ImuHealthMonitor imu_health_mon_;
  AltitudeSourceSelector alt_source_selector_;

  // Latest health evaluations (for telemetry)
  LidarHealthMonitor::Health last_lidar_health_;
  BaroHealthMonitor::Health last_baro_health_;
  MagHealthMonitor::Health last_mag_health_;
  OpticalFlowHealthMonitor::Health last_of_health_;
  ImuHealthMonitor::Health last_imu_health_;

  // Mode transition tracking
  AltitudeSource current_alt_source_ = AltitudeSource::NONE;
  std::vector<ModeTransition> mode_transitions_;

  // ===== Phase 1: Watchdog System =====
  WatchdogSystem watchdog_;
  bool watchdog_failure_detected_ = false;
  std::string watchdog_failure_reason_;

  // ===== Phase 2: Calibration & Schema Validation =====
  chimera::utils::CalibrationManager calibration_;
  bool enable_schema_validation_ = true;  // Validate telemetry in debug builds
  int schema_validation_errors_ = 0;      // Track validation failures

  // ---- Robust baro filter helpers
  static double qmad(const std::vector<double>& v){
    if(v.empty()) return 0.0;
    std::vector<double> d=v;
    std::nth_element(d.begin(), d.begin()+d.size()/2, d.end());
    double med = d[d.size()/2];
    std::vector<double> r; r.reserve(d.size());
    for(double x: d) r.push_back(std::abs(x-med));
    std::nth_element(r.begin(), r.begin()+r.size()/2, r.end());
    return 1.4826 * r[r.size()/2];
  }

  bool robustBaroAGL(double t_abs, double& agl_out, double& sigma_out){
    const double WINDOW=3.0;   // tolerate sparse 1 Hz baro
    while(!baro_buf_.empty() && (t_abs - baro_buf_.front().first) > WINDOW)
      baro_buf_.pop_front();
    if(baro_buf_.size() < 2 || !std::isfinite(baro0_msl_)) return false;

    std::vector<double> msl; msl.reserve(baro_buf_.size());
    for(auto& kv: baro_buf_) msl.push_back(kv.second);
    std::vector<double> d=msl;
    std::nth_element(d.begin(), d.begin()+d.size()/2, d.end());
    double med = d[d.size()/2];
    double sig = qmad(msl);
    sigma_out = std::clamp(sig*1.5, 0.8, 2.5);  // Increased for noisy baro data

    // Use MEDIAN msl, not latest (more robust to noise)
    agl_out = (med - baro0_msl_) + range_boot_.z0_agl_m;
    // FIX: Wider validity range to make baro "stickier" during LiDAR outages
    return (agl_out > -50.0 && agl_out < 3000.0);
  }

  // ---- helpers (Phase 2: integrated with calibration)
  Eigen::Vector3d fixA(const Eigen::Vector3d& a, double temp = 25.0) const {
    // Apply calibration first
    Eigen::Vector3d calibrated = calibration_.applyImuAccel(a, temp);
    // Then apply frame transformation
    return flip_nwu_to_frd_ ? Eigen::Vector3d(calibrated.x(), -calibrated.y(), -calibrated.z()) : calibrated;
  }
  Eigen::Vector3d fixG(const Eigen::Vector3d& g, double temp = 25.0) const {
    // Apply calibration first
    Eigen::Vector3d calibrated = calibration_.applyImuGyro(g, temp);
    // Then apply frame transformation and scaling
    if(flip_nwu_to_frd_) return { calibrated.x()*gyro_scale_, -calibrated.y()*gyro_scale_, -calibrated.z()*gyro_scale_ };
    return calibrated * gyro_scale_;
  }
  Eigen::Vector3d fixM(const Eigen::Vector3d& m) const {
    // Apply calibration first
    Eigen::Vector3d calibrated = calibration_.applyMag(m);
    // Then apply frame transformation
    return flip_nwu_to_frd_ ? Eigen::Vector3d(calibrated.x(), -calibrated.y(), -calibrated.z()) : calibrated;
  }

  // Phase 2: Baro and LiDAR calibration helpers
  double applyBaroCalib(double raw_altitude, double temp = 25.0) const {
    return calibration_.applyBaro(raw_altitude, temp);
  }

  double applyLidarCalib(double raw_range) const {
    return calibration_.applyLidar(raw_range);
  }

  // Phase 1: Track altitude source mode transitions
  void trackModeTransition(AltitudeSource new_source, double t, const std::string& reason = "") {
    if (new_source != current_alt_source_) {
      ModeTransition trans;
      trans.timestamp = t;
      trans.from = current_alt_source_;
      trans.to = new_source;
      trans.reason = reason;

      mode_transitions_.push_back(trans);

      // Log to console (sparse - every transition)
      std::cout << "[MODE_TRANSITION k=" << key_ << " t=" << t << "] "
                << toString(current_alt_source_) << " → " << toString(new_source);
      if (!reason.empty()) {
        std::cout << " (" << reason << ")";
      }
      std::cout << std::endl;

      current_alt_source_ = new_source;
    }
  }

  void init(){
    // FIX #5: Replace system() with std::filesystem for portability
    std::filesystem::create_directories("logs");
    telem_path_ = "logs/chimera_multi.jsonl";
    std::cout<<"Telemetry: "<<telem_path_<<"\n";

    // Phase 2: Load calibration data (if available)
    std::string calib_path = "config/calibration.json";
    if (std::filesystem::exists(calib_path)) {
      if (calibration_.load(calib_path)) {
        if (!calibration_.checkVersion()) {
          std::cerr << "[CALIBRATION WARN] Version mismatch, using factory defaults\n";
          calibration_.data() = chimera::utils::createFactoryDefaults();
        } else if (calibration_.isStale(365.0)) {
          std::cerr << "[CALIBRATION WARN] Calibration older than 1 year\n";
        }
      } else {
        std::cerr << "[CALIBRATION] Failed to load, using factory defaults\n";
        calibration_.data() = chimera::utils::createFactoryDefaults();
      }
    } else {
      std::cout << "[CALIBRATION] No calibration file found, using factory defaults\n";
      calibration_.data() = chimera::utils::createFactoryDefaults();
    }

    // Auto-tune smoother rate to maintain constant key budget
    if(cfg_.auto_smoother_rate){
      const double hz = std::clamp(
          static_cast<double>(cfg_.target_keys_in_lag) / std::max(1e-6, cfg_.lag_seconds),
          cfg_.smoother_hz_min, cfg_.smoother_hz_max);
      std::cout<<"[SMOOTHER] auto rate "<<cfg_.smoother_hz<<" Hz → "<<hz
               <<" Hz (target_keys="<<cfg_.target_keys_in_lag
               <<", lag="<<cfg_.lag_seconds<<"s)\n";
      cfg_.smoother_hz = hz;
    }

    // 1) IMU accel units: g→m/s^2 if needed (median over first 2s)
    if(safe_boot_ && !sensors_.imu.empty()){
      const double t0 = sensors_.imu.front().t;
      std::vector<double> acc_norm; acc_norm.reserve(256);
      for(const auto& s: sensors_.imu){ if(s.t-t0>2.0) break; acc_norm.push_back(s.accel.norm()); }
      const double med = medianScalar(acc_norm);
      const bool in_g = (med>=0.7 && med<=1.3);
      const bool in_m = (med>=7.0 && med<=13.0);
      if(in_g){ for(auto& s: sensors_.imu) s.accel*=9.81; }
      else if(!in_m) std::cout<<"[IMU BOOT] accel units ambiguous; assuming m/s^2\n";
      std::cout<<"[IMU BOOT] accel_med="<<med<<" → "<<(in_g? "g→m/s^2" : "no-conv")<<"\n";
    }

    // 2) orientation/frame flip
    const auto& imu0 = sensors_.imu.front();
    Eigen::Vector3d a_med = medianAccel(sensors_.imu, sensors_.imu.front().t, 0.5);
    auto r0 = pickRwb_from_quat(imu0.quat, a_med);
    if(r0.g_err>2.0){
      Eigen::Vector3d afl(a_med.x(), -a_med.y(), -a_med.z());
      auto r1 = pickRwb_from_quat(imu0.quat, afl);
      if(r1.g_err + 0.5 < r0.g_err){ flip_nwu_to_frd_=true; r0=r1; std::cout<<"[ORIENT] flip NWU→FRD\n"; }
    }
    init_Rwb_ = r0.Rwb;

    // Fix 180° roll error: if |roll| > 90°, apply 180° rotation around Y-axis (body frame)
    // DISABLED: Testing baseline performance without orientation fix
    /*
    {
      const auto rpy_check = init_Rwb_.rpy();
      const double roll_deg = std::abs(rpy_check(0)) * 57.2958;
      if(roll_deg > 90.0){
        // Apply 180° rotation around Y-axis: R_fix = R_original * Ry(180°)
        const gtsam::Rot3 R_y_180 = gtsam::Rot3::Ry(M_PI);
        init_Rwb_ = init_Rwb_ * R_y_180;
        std::cout << "[ORIENT FIX] Detected |roll|=" << roll_deg
                  << "° > 90°, applying 180° Y-axis correction\n";
      }
    }
    */

    // --- Boot sanity check: gravity residual in NED (Down = +Z) ---
    // Use the final flipped accel value for accurate check
    {
      const Eigen::Vector3d a_med_final = flip_nwu_to_frd_ ? Eigen::Vector3d(a_med.x(), -a_med.y(), -a_med.z()) : a_med;
      const Eigen::Vector3d g_NED(0,0,cfg_.g);  // Use config gravity value
      const Eigen::Vector3d r1 = init_Rwb_.matrix() * a_med_final + g_NED;               // expected
      const Eigen::Vector3d r2 = init_Rwb_.matrix().transpose() * a_med_final + g_NED;   // wrong R?
      const Eigen::Vector3d r3 = init_Rwb_.matrix() * (-a_med_final) + g_NED;            // wrong sign on accel?

      const double m1 = r1.norm(), m2 = r2.norm(), m3 = r3.norm();

      if (m1 > 0.8) {
        std::cerr << "[BOOT WARN] gravity residual = " << m1
                  << " m/s^2 (expected < 0.8). Check IMU units/frame)\n";
        std::cerr << "[BOOT DIAG] resid norms: R*a+g=" << m1
                  << "  RT*a+g=" << m2
                  << "  R*(-a)+g=" << m3
                  << "  (using final flipped accel=" << a_med_final.transpose() << ")\n";
      } else {
        std::cout << "[BOOT OK] gravity residual = " << m1 << " m/s^2\n";
      }
    }

    // 3) mag median norm (first 2s)
    if(!sensors_.mag.empty()){
      std::vector<double> norms;
      double t0=sensors_.mag.front().t;
      for(const auto& m: sensors_.mag){ if(m.t-t0<2.0) norms.push_back(fixM(m.mag).norm()); }
      mag_median_norm_ = norms.empty()? 1.0 : medianScalar(norms);
      std::cout<<"[MAG BOOT] median |m|="<<mag_median_norm_<<" (±"<<(cfg_.mag_gate_frac*100)<<"%)\n";

      // Phase 1: Initialize mag health monitor with median norm
      mag_health_mon_.setMedianNorm(mag_median_norm_);
    }

    // 4) LiDAR bootstrap + in-place scaling
    if(!sensors_.lidar.empty()){
      range_boot_ = bootRangeUnits(sensors_.lidar, sensors_.imu.front().t, cfg_.boot_secs);
      if(!range_boot_.ready){ range_boot_.scale_to_m=1.0; range_boot_.units=RangeUnits::M; range_boot_.ready=true; }
      // Apply unit scaling + Phase 2: calibration
      for(auto& s: sensors_.lidar){
        s.range_min *= range_boot_.scale_to_m;
        s.range_mean *= range_boot_.scale_to_m;
        // Apply lidar calibration (offset, scale, pitch angle correction)
        s.range_min = applyLidarCalib(s.range_min);
        s.range_mean = applyLidarCalib(s.range_mean);
      }
      // establish initial AGL & warm guard
      init_agl_m_ = sensors_.lidar.front().range_mean;
      range_boot_.z0_agl_m = init_agl_m_;                 // ensure available for baro-AGL
      lidar_guard_ = range_boot_.guard;
      lidar_stuck_ = range_boot_.lidar_stuck;
      if(std::isfinite(init_agl_m_)){  // seed memory from boot AGL
        last_trusted_agl_ = init_agl_m_;
        last_trust_t_     = 0.0;
        has_trusted_alt_  = true;
      }

      std::cout<<"[RANGE BOOT] units="<<(range_boot_.units==RangeUnits::MM? "mm": range_boot_.units==RangeUnits::CM? "cm":"m")
               <<" scale="<<range_boot_.scale_to_m<<" z0="<<init_agl_m_<<"m stuck="<<(lidar_stuck_?"YES":"NO")
               <<" guard "<<lidar_guard_.win.size()<<"/"<<lidar_guard_.N<<"\n";
    }

    // 5) Baro baseline (median first 2s)
    if(!sensors_.baro.empty()){
      std::vector<double> win; double t0=sensors_.baro.front().t;
      for(const auto& b: sensors_.baro){ if(b.t-t0>2.0) break; win.push_back(b.altitude); }
      if(!win.empty()){ baro0_msl_ = medianScalar(win); }
      std::cout<<"[BARO BOOT] baseline="<<baro0_msl_<<" m\n";
    }

    // 6) UKF & PIM init
    // NED nav frame: Z-down, so negate AGL (up-positive)
    gtsam::Pose3 init_pose(init_Rwb_, gtsam::Point3(0,0, -init_agl_m_));
    Eigen::Matrix<double,15,15> P0 = Eigen::Matrix<double,15,15>::Identity();
    P0.block<3,3>(0,0)*=2.0; P0.block<3,3>(3,3)*=0.8; P0.block<3,3>(6,6)*=0.2; P0.block<3,3>(9,9)*=0.02; P0.block<3,3>(12,12)*=0.1;
    ukf_.initialize(init_pose, Eigen::Vector3d::Zero(), P0);

    // FRD body -> NED nav: use MakeSharedD (Down-positive) and gravity = [0,0,+g]
    auto pim_params = gtsam::PreintegrationParams::MakeSharedD(cfg_.g);
    pim_params->accelerometerCovariance = gtsam::Matrix33::Identity() * (cfg_.ukf_noise.accel_noise_density*cfg_.ukf_noise.accel_noise_density);
    pim_params->gyroscopeCovariance     = gtsam::Matrix33::Identity() * (cfg_.ukf_noise.gyro_noise_density*cfg_.ukf_noise.gyro_noise_density);
    pim_params->integrationCovariance   = gtsam::Matrix33::Identity() * 1e-8;
    pim_params->n_gravity               = gtsam::Vector3(0,0, cfg_.g);

    gtsam::imuBias::ConstantBias prior_bias(ukf_.getAccelBias(), ukf_.getGyroBias());
    pim_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(pim_params, prior_bias);

    // Phase 1: Initialize watchdog system
    watchdog_.deadlineMonitor().registerSensor("imu", 0.05);        // 50ms @ ~200Hz
    watchdog_.deadlineMonitor().registerSensor("lidar", 0.3);       // 300ms @ ~5Hz
    watchdog_.deadlineMonitor().registerSensor("baro", 1.5);        // 1.5s @ ~1Hz
    watchdog_.deadlineMonitor().registerSensor("mag", 0.3);         // 300ms @ ~5Hz
    watchdog_.deadlineMonitor().registerSensor("optical_flow", 0.5); // 500ms @ variable
    std::cout<<"[WATCHDOG] Initialized: deadline monitors, NaN guards, covariance health\n";
  }

  void processImu(const ImuSample& imu){
    const double t = imu.t - sensors_.imu.front().t;

    // feed baro buffer for robust fallback
    const double t_abs = sensors_.imu.front().t + t;
    if(!sensors_.baro.empty()){
      auto b = nearestByTime(sensors_.baro, t_abs, 0.80);
      if(b) {
        // Phase 2: Apply baro calibration
        double calibrated_altitude = applyBaroCalib(b->altitude);
        baro_buf_.emplace_back(t_abs, calibrated_altitude);
      }
    }

    // propagate UKF (with timing)
    chimera::core::UKF::ImuMeasurement m; m.timestamp=t; m.gyro=fixG(imu.gyro); m.accel=fixA(imu.accel); m.temperature=25.0;
    chimera::utils::Timer t_ukf;
    ukf_.propagate(m);
    ukf_cpu_ms_ = t_ukf.tocMs();

    // Phase 1: Evaluate IMU health
    last_imu_health_ = imu_health_mon_.evaluate(imu);

    // Phase 1: Watchdog checks for IMU
    watchdog_.clearReport();
    auto nan_accel = NaNGuard::check(imu.accel, "imu.accel");
    auto nan_gyro = NaNGuard::check(imu.gyro, "imu.gyro");
    watchdog_.checkNaN(nan_accel);
    watchdog_.checkNaN(nan_gyro);
    watchdog_.deadlineMonitor().update("imu", t_abs);
    auto mono_check = watchdog_.monotonicClock().check(imu.t, "imu");
    watchdog_.checkMonotonic(mono_check);

    // preintegrate
    if(last_imu_t_){
      const double dt = t - *last_imu_t_;
      if(dt>1e-9) pim_->integrateMeasurement(fixA(imu.accel), fixG(imu.gyro), dt);
    }
    last_imu_t_ = t;

    // smoother update @ 5 Hz
    const double dt_sm = 1.0/cfg_.smoother_hz;
    if((t - last_smoother_t_) >= dt_sm){
      updateSmoother(t);
      last_smoother_t_=t;
    }

    // Periodic re-anchor from smoother to UKF (every reanchor_period_s)
    if((t - last_reanchor_t_) >= cfg_.reanchor_period_s && key_>1){
      const auto est = smoother_.calculateEstimate();
      const auto k=key_-1;
      if(est.exists(X(k)) && est.exists(V(k)) && est.exists(B(k))){
        const gtsam::Pose3 pose_opt = est.at<gtsam::Pose3>(X(k));
        const Eigen::Vector3d vel_opt= est.at<gtsam::Vector3>(V(k));
        const auto bias_opt = est.at<gtsam::imuBias::ConstantBias>(B(k));
        const bool z_settled = (std::abs(pose_opt.z()) < 50.0) || (lidar_healthy_streak_ >= 10);
        if(!z_settled) return;  // defer this cycle

        const gtsam::Matrix cov_pose = smoother_.marginalCovariance(X(k));
        const gtsam::Matrix cov_vel  = smoother_.marginalCovariance(V(k));
        const gtsam::Matrix cov_bias = smoother_.marginalCovariance(B(k));

        Eigen::Matrix<double,15,15> cov_gtsam=Eigen::Matrix<double,15,15>::Zero();
        cov_gtsam.block<6,6>(0,0)=cov_pose;
        cov_gtsam.block<3,3>(6,6)=cov_vel;
        cov_gtsam.block<6,6>(9,9)=cov_bias;

        Eigen::Matrix<double,15,15> cov_ukf=Eigen::Matrix<double,15,15>::Zero();
        cov_ukf.block<3,3>(0,0)  = cov_gtsam.block<3,3>(3,3);
        cov_ukf.block<3,3>(3,3)  = cov_gtsam.block<3,3>(6,6);
        cov_ukf.block<3,3>(6,6)  = cov_gtsam.block<3,3>(0,0);
        cov_ukf.block<3,3>(9,9)  = cov_gtsam.block<3,3>(12,12);
        cov_ukf.block<3,3>(12,12)= cov_gtsam.block<3,3>(9,9);

        // Phase 1: Check covariance health before reanchoring
        auto cov_health = watchdog_.covarianceMonitor().evaluate(cov_ukf, "reanchor_cov");
        watchdog_.checkCovariance(cov_health);
        if (!cov_health.is_healthy) {
          std::cerr << "[WATCHDOG] Covariance unhealthy at reanchor: " << cov_health.reason << "\n";
          // Continue anyway for now - just log the warning
        }

        ukf_.resetWithCovariance(pose_opt, vel_opt, bias_opt.gyroscope(), bias_opt.accelerometer(), cov_ukf);
        last_reanchor_t_=t;
        last_anchor_applied_ = true;
        last_anchor_age_s_ = t - last_smoother_t_;
      }
    }
  }

  void restartSmootherForTakeover(double t, double baro_agl){
    // Hot-reset smoother: purge bad altitude history by creating fresh smoother with short lag
    saved_lag_seconds_ = cfg_.lag_seconds;
    cfg_.lag_seconds = std::min(3.0, saved_lag_seconds_);  // Short window (3s)
    smoother_.reinit(cfg_.lag_seconds);

    // Seed current key with baro-anchored priors
    gtsam::NonlinearFactorGraph g;
    gtsam::Values init;
    auto Kp=X(key_), Kv=V(key_), Kb=B(key_), Kw=W(key_);

    // Pose: pin Z to baro, keep R from UKF, loose XY
    auto pose_now = ukf_.getPose();
    auto P = pose_now.translation();
    P.z() = baro_agl;  // Force Z from baro (NED: baro_agl is negative, Down=+Z)
    gtsam::Pose3 x_seed(pose_now.rotation(), P);

    // Sigmas: tight Z (0.8m), loose XY (2m), moderate orientation
    auto Np = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 2.0, 2.0, 0.8, 0.2, 0.2, 1.5).finished());
    g.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Kp, x_seed, Np);
    init.insert(Kp, x_seed);

    // Velocity: near-zero (recovering from divergence)
    gtsam::Vector3 v_zero = gtsam::Vector3::Zero();  // Explicit zero vector
    auto Nv = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << 0.5, 0.5, 0.8).finished());
    g.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(Kv, v_zero, Nv);
    init.insert(Kv, v_zero);

    // Bias: current UKF estimate
    gtsam::imuBias::ConstantBias b_seed(ukf_.getAccelBias(), ukf_.getGyroBias());
    auto Nb = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.3, 0.3, 0.3, 0.05, 0.05, 0.05).finished());
    g.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(Kb, b_seed, Nb);
    init.insert(Kb, b_seed);

    // Wind: initialize to zero (unknown during recovery)
    gtsam::Vector2 wind_zero = gtsam::Vector2::Zero();
    auto Nw = gtsam::noiseModel::Isotropic::Sigma(2, 5.0);  // Loose prior
    g.emplace_shared<gtsam::PriorFactor<gtsam::Vector2>>(Kw, wind_zero, Nw);
    init.insert(Kw, wind_zero);

    smoother_.addFactors(g, init, t);

    // Cut IMU chain for next 2 keys to break continuity
    imu_cut_keys_left_ = 2;
    takeover_just_activated_ = true;

    std::cout << "[SMOOTHER_RESTART k="<<key_<<" t="<<t<<"] lag="<<cfg_.lag_seconds
              <<"s, Z="<<baro_agl<<"m (baro-anchored)\n";
  }

  void immediateReanchorIfRequested(double t){
    // PRIORITY: Normal reanchor (first LiDAR or post-takeover)
    if(!force_reanchor_now_) return;

    // Use the just-added key 'key_' (before key_++)
    const auto k = key_;
    const auto est = smoother_.calculateEstimate();
    if(est.exists(X(k)) && est.exists(V(k)) && est.exists(B(k))){
      const gtsam::Pose3 pose_opt = est.at<gtsam::Pose3>(X(k));
      const Eigen::Vector3d vel_opt= est.at<gtsam::Vector3>(V(k));
      const auto bias_opt = est.at<gtsam::imuBias::ConstantBias>(B(k));

      const gtsam::Matrix cov_pose = smoother_.marginalCovariance(X(k));
      const gtsam::Matrix cov_vel  = smoother_.marginalCovariance(V(k));
      const gtsam::Matrix cov_bias = smoother_.marginalCovariance(B(k));

      Eigen::Matrix<double,15,15> cov_gtsam=Eigen::Matrix<double,15,15>::Zero();
      cov_gtsam.block<6,6>(0,0)=cov_pose;
      cov_gtsam.block<3,3>(6,6)=cov_vel;
      cov_gtsam.block<6,6>(9,9)=cov_bias;

      // Map to UKF ordering (identical to periodic reanchor)
      // GTSAM: [pos(0:2), orient(3:5), vel(6:8), accel_bias(9:11), gyro_bias(12:14)]
      // UKF:   [orient(0:2), vel(3:5), pos(6:8), gyro_bias(9:11), accel_bias(12:14)]
      Eigen::Matrix<double,15,15> cov_ukf=Eigen::Matrix<double,15,15>::Zero();
      cov_ukf.block<3,3>(0,0)  = cov_gtsam.block<3,3>(3,3);   // UKF orient <- GTSAM orient
      cov_ukf.block<3,3>(3,3)  = cov_gtsam.block<3,3>(6,6);   // UKF vel    <- GTSAM vel
      cov_ukf.block<3,3>(6,6)  = cov_gtsam.block<3,3>(0,0);   // UKF pos    <- GTSAM pos
      cov_ukf.block<3,3>(9,9)  = cov_gtsam.block<3,3>(12,12); // UKF gyro   <- GTSAM gyro
      cov_ukf.block<3,3>(12,12)= cov_gtsam.block<3,3>(9,9);   // UKF accel  <- GTSAM accel

      ukf_.resetWithCovariance(pose_opt, vel_opt, bias_opt.gyroscope(), bias_opt.accelerometer(), cov_ukf);
      pim_->resetIntegrationAndSetBias(bias_opt);
      last_reanchor_t_ = t;
      did_lidar_reset_ = true;
      force_reanchor_now_ = false;
      // NED: Z is down+. Report AGL as |Z|.
      double agl_m = std::abs(pose_opt.z());
      std::cout << "[REANCHOR-FIRST-LIDAR] t="<<t<<" k="<<k<<" agl="<<agl_m<<" m\n";
    }
  }

  void updateSmoother(double t){
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values init;

    // seed pose/vel/bias
    const gtsam::Pose3 x0 = ukf_.getPose();
    const Eigen::Vector3d v0 = ukf_.getVelocity();
    const gtsam::imuBias::ConstantBias b0(ukf_.getAccelBias(), ukf_.getGyroBias());

    const auto Kp=X(key_), Kv=V(key_), Kb=B(key_), Kw=W(key_);

    // FIX #3: Compute estimate once and reuse (avoid 9 redundant calls!)
    const gtsam::Values est_prev_all = (key_>0) ? smoother_.calculateEstimate() : gtsam::Values{};

    // ===== Contractive dissipation priors (Tikhonov regularization) =====
    // Provides backbone stiffness in poorly-observed directions (Z pos/vel, yaw)
    // Observability slack: 1.0=fully observed, 0.1=poorly observed
    auto slack01 = [&](bool have_lidar, bool have_of){
      double slack = 1.0;
      if(!have_lidar) slack *= 0.4;
      if(!have_of)    slack *= 0.6;
      return std::clamp(slack, 0.1, 1.0);
    };

    // Compute current observability (using state from previous cycle as proxy)
    bool have_lidar_now = !lidar_stuck_ && (lidar_healthy_streak_ > 0);
    bool have_of_now    = (of_updates_ > 0);
    double s = slack01(have_lidar_now, have_of_now);

    // Map slack → sigmas: small sigma = strong dissipation = large ρ
    double sigma_z_pos = std::clamp(0.10 + 0.90*s, 0.10, 1.00);   // 0.10..1.0 m
    double sigma_vz    = std::clamp(0.15 + 0.85*s, 0.15, 1.00);   // 0.15..1.0 m/s
    double sigma_yaw   = std::clamp(0.05 + 0.45*s, 0.05, 0.50);   // 0.05..0.5 rad

    // Pose prior: tight on Z & yaw, loose on XY & roll/pitch
    // FIX: Skip dissipation priors when tight boot priors are active (keys 0-10)
    // to avoid duplicate priors that over-constrain the factor graph
    bool has_tight_priors = (key_==0) || (safe_boot_ && key_<=10);
    if (!has_tight_priors) {
      gtsam::Vector6 sig_pose;
      sig_pose << 10.0, 10.0, sigma_z_pos,    // x, y, z
                  0.8,  0.8,  sigma_yaw;       // roll, pitch, yaw
      auto Npose_diss = gtsam::noiseModel::Diagonal::Sigmas(sig_pose);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Kp, x0, Npose_diss);

      // Velocity prior: tight on vz, loose on vx/vy
      gtsam::Vector3 sig_vel(10.0, 10.0, sigma_vz);
      auto Nvel_diss = gtsam::noiseModel::Diagonal::Sigmas(sig_vel);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(Kv, v0, Nvel_diss);
    }

    // NOTE: Dissipation priors disabled during boot to avoid conflicts with tight priors

    // ===== Risk-budgeted sigmas (Chebyshev chance-constraints) =====
    // Allocate total 5% tail-risk budget across sensors with provable guarantees
    // Blend with adaptive constraints using max() to get tightest valid bound
    const double total_alpha = 0.05;  // 5% system-wide tail-risk budget
    chimera::utils::RiskSplit risk_split;
    risk_split.lidar_weight = 0.30;      // 30% of risk (critical altitude constraint)
    risk_split.baro_weight = 0.15;       // 15% of risk (backup altitude)
    risk_split.of_weight = 0.40;         // 40% of risk (primary horizontal constraint)
    risk_split.airspeed_weight = 0.15;   // 15% of risk (forward velocity)
    auto [c_lidar, c_baro, c_of, c_airspeed] =
        chimera::utils::split_risk(total_alpha, risk_split);

    // ZUPT: if stationary for >0.5s, add strong velocity=0 constraint
    static double zupt_start_t = -999.0;
    const double vnorm = v0.norm();
    if(vnorm < 0.3){ if(zupt_start_t<0) zupt_start_t=t; }
    else zupt_start_t=-999.0;
    const bool is_zupt = (zupt_start_t>=0 && (t - zupt_start_t)>0.5);
    if(is_zupt && key_>0){
      auto N_zupt = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(Kv, gtsam::Vector3::Zero(), N_zupt);
    }
    if(key_==0){ init.insert(Kp, x0); init.insert(Kv, v0); init.insert(Kb, b0); }
    else {
      if (est_prev_all.exists(X(key_-1)) &&
          est_prev_all.exists(V(key_-1)) &&
          est_prev_all.exists(B(key_-1))) {
        const auto pose_i = est_prev_all.at<gtsam::Pose3>(X(key_-1));
        const auto vel_i  = est_prev_all.at<gtsam::Vector3>(V(key_-1));
        const auto bias_i = est_prev_all.at<gtsam::imuBias::ConstantBias>(B(key_-1));
        gtsam::NavState nav_i(pose_i, vel_i);
        gtsam::NavState pred = pim_->predict(nav_i, bias_i);
        init.insert(Kp, pred.pose()); init.insert(Kv, pred.v()); init.insert(Kb, bias_i);
      } else {
        // Fresh smoother or prior key marginalized: fall back to UKF state
        init.insert(Kp, x0);
        init.insert(Kv, v0);
        init.insert(Kb, b0);
      }
    }
    init.insert(Kw, gtsam::Point2(0.0,0.0));

    // priors first key
    if(key_==0){
      auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<< 0.05,0.05,0.3, 0.2,0.2,2.0).finished());
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Kp, x0, pose_noise);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(Kv, v0, gtsam::noiseModel::Isotropic::Sigma(3,0.5));
      graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(Kb, b0, gtsam::noiseModel::Isotropic::Sigma(6,0.1));
      // wind loose early
      auto wind_prior = gtsam::noiseModel::Isotropic::Sigma(2, 2.0);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(Kw, gtsam::Point2(0,0), wind_prior);
    }

    // safe-boot: pin roll/pitch for first 10 keys (loose yaw/xyz)
    if(safe_boot_ && key_>0 && key_<=10){
      const gtsam::Pose3 seed = init.at<gtsam::Pose3>(Kp);
      auto rp = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6)<< 0.30,0.30,0.30, 0.10,0.10,1.50).finished());
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(Kp, seed, rp);
    }

    // Bias prior: adaptive schedule (matures with system health)
    // CRITICAL: Use current bias estimate from smoother, NOT boot value b0!
    if(safe_boot_ && key_ > 0 && key_ <= 20){
      double accel_sigma, gyro_sigma;
      chimera::utils::AdaptiveConstraints::getBiasPriorSigma(
          key_, lidar_healthy_streak_, accel_sigma, gyro_sigma);

      gtsam::Vector6 bias_sigmas;
      bias_sigmas << accel_sigma, accel_sigma, accel_sigma,
                     gyro_sigma, gyro_sigma, gyro_sigma;
      auto bias_noise = gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas);

      // Get current estimated bias from previous keyframe (not boot value!)
      // Use hoisted estimate (performance optimization from bug fix #3)
      if(est_prev_all.exists(B(key_-1))){
        const auto bias_current = est_prev_all.at<gtsam::imuBias::ConstantBias>(B(key_-1));
        graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(Kb, bias_current, bias_noise);
      }
    }

    // IMU factor & RW (skip IMU factor during takeover to break continuity)
    if(key_>0){
      const int i=key_-1, j=key_;

      // Check if we should cut the IMU chain (during baro takeover)
      if(imu_cut_keys_left_ > 0){
        // SKIP ImuFactor to break continuity - allows baro to pull Z back
        --imu_cut_keys_left_;
        if(key_%5==0) std::cout<<"[IMU_CUT k="<<key_<<"] Skipping ImuFactor (keys_left="
                               <<imu_cut_keys_left_<<")\n";
      } else {
        // Normal case: add IMU preintegration factor
        graph.emplace_shared<gtsam::ImuFactor>(X(i),V(i),X(j),V(j),B(i),*pim_);
      }

      const double dt = t - last_smoother_t_;

      // OU bias process: regime-switching with learned mean
      bool lidar_trusted = (lidar_healthy_streak_ >= 10) && !lidar_stuck_;
      bool of_healthy = (of_updates_ > 0); // simple proxy using previous state
      BiasMode bm = bias_mode(lidar_trusted, of_healthy, key_);

      double sig_a, sig_g, phi_a, phi_g;
      ou_sigmas(dt, bm, sig_a, sig_g, phi_a, phi_g);
      gtsam::Vector6 sigs;
      sigs << sig_a, sig_a, sig_a, sig_g, sig_g, sig_g;
      auto bias_ou = gtsam::noiseModel::Diagonal::Sigmas(sigs);

      // Mean-reverting OU: Bj = phi*Bi + (1-phi)*mu + w,  w ~ N(0, Qd)
      gtsam::imuBias::ConstantBias mean_delta;
      if(bias_mean_update_count_ > 10 && est_prev_all.exists(B(i))){
        const auto mu = learned_bias_mean_;
        auto b_prev = est_prev_all.at<gtsam::imuBias::ConstantBias>(B(i));
        mean_delta = gtsam::imuBias::ConstantBias(
          (1.0 - phi_a) * (mu.accelerometer() - b_prev.accelerometer()),
          (1.0 - phi_g) * (mu.gyroscope() - b_prev.gyroscope())
        );
      } else {
        mean_delta = gtsam::imuBias::ConstantBias(); // zero until mean stabilizes
      }

      graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
        B(i), B(j), mean_delta, bias_ou);

      auto wind_rw = gtsam::noiseModel::Isotropic::Sigma(2, 0.2*std::sqrt(dt));
      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Point2>>(W(i),W(j), gtsam::Point2(0,0), wind_rw);

      // wind restraint (robust)
      auto huber = gtsam::noiseModel::mEstimator::Huber::Create(1.5);
      auto base  = gtsam::noiseModel::Isotropic::Sigma(2,3.0);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(Kw, gtsam::Point2(0,0),
        gtsam::noiseModel::Robust::Create(huber, base));
    }

    // ---------------- MAG ----------------
    auto mag_s = nearestByTime(sensors_.mag, sensors_.imu.front().t + t, 0.05);
    if(mag_s){
      // Phase 1: Evaluate mag health
      last_mag_health_ = mag_health_mon_.evaluate(*mag_s);

      Eigen::Vector3d m_b = fixM(mag_s->mag);
      const double n = m_b.norm();
      const double lo = (1.0 - cfg_.mag_gate_frac) * mag_median_norm_;
      const double hi = (1.0 + cfg_.mag_gate_frac) * mag_median_norm_;
      if(n>=lo && n<=hi){
        const double yaw_mag = yawFromMagTiltComp(m_b, ukf_.getPose().rotation());
        auto robust = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(1.5),
          gtsam::noiseModel::Isotropic::Sigma(1, (key_<20? 0.35 : key_<40? 0.17 : 0.10)) );
        graph.emplace_shared<chimera::factors::MagYawFactor>(Kp, yaw_mag, robust);
        monitor_.updateYaw(t); mag_updates_++;
      }
    }

    // ---------------- LiDAR vs Baro (altitude) ----------------
    const double now_abs = sensors_.imu.front().t + t;
    auto lidar = nearestByTime(sensors_.lidar, now_abs, 0.15);  // Widened: 10Hz samples need >100ms window for jitter
    auto baro  = nearestByTime(sensors_.baro , now_abs, 0.80);

    // Phase 1: Evaluate Baro health
    if(baro){
      last_baro_health_ = baro_health_mon_.evaluate(*baro);
    }

    // cold-start advance
    if(!boot_done_){
      if(lidar && lidar->range_mean>cfg_.lidar_min && lidar->range_mean<cfg_.lidar_max)
        lidar_guard_.add(lidar->range_mean);
      const bool guard_full = lidar_guard_.isFull();
      const bool time_exp   = (t >= cfg_.boot_secs);
      if(guard_full || time_exp){
        // Clear boot guard buffer (stationary samples) to prevent false stuck detection
        // Runtime stuck detection will build fresh buffer from flight samples
        lidar_guard_.reset();
        lidar_stuck_ = false;  // Assume operational until proven stuck
        boot_done_ = true;
        std::cout<<"[COLD-START END k="<<key_<< " t="<<t<<"] stuck=NO (guard cleared for flight samples)"
                 <<" guard_full="<<(guard_full?"YES":"NO")<<" time_exp="<<(time_exp?"YES":"NO")<<"\n";
      }
    }

    bool lidar_available = lidar && lidar->range_mean>cfg_.lidar_min && lidar->range_mean<cfg_.lidar_max;

    // Phase 1: Evaluate LiDAR health
    if(lidar){
      last_lidar_health_ = lidar_health_mon_.evaluate(*lidar);
    }

    // Force LiDAR unavailable during baro takeover (emergency recovery mode)
    if (baro_takeover_active_) {
      lidar_available = false;
    }

    // Always feed the guard so it can actually reach 'ready'
    if(lidar_available){
      const bool guard_was_full = lidar_guard_.isFull();
      const bool stuck_now = lidar_guard_.addAndCheck(lidar->range_mean);
      if(boot_done_ && lidar_guard_.isFull() && !guard_was_full){
        std::cout<<"[LIDAR GUARD] refilled with flight samples; N="<<lidar_guard_.N<<"\n";
      }
      if(boot_done_ && stuck_now){
        lidar_stuck_ = true; lidar_available=false; lidar_stuck_events_++;
        if(key_%20==0) std::cout<<"[LIDAR STUCK] plateau at "<<lidar->range_mean<<" m (events:"<<lidar_stuck_events_<<")\n";
      }
    }
    const bool guard_ready = lidar_guard_.isFull();
    const bool can_trust_lidar = boot_done_ && guard_ready && !lidar_stuck_;
    const bool can_trust_for_reset = can_trust_lidar && !did_lidar_reset_;

    // Track LiDAR healthy streak & altitude memory
    if(lidar_available && !lidar_stuck_){
      lidar_healthy_streak_ = std::min(lidar_healthy_streak_ + 1, 100);
      if(can_trust_lidar){
        last_trusted_agl_ = lidar->range_mean;
        last_trust_t_     = t;
        has_trusted_alt_  = true;
      }
      if(can_trust_for_reset){
        force_reanchor_now_ = true;
      }
    } else {
      lidar_healthy_streak_ = 0;
    }

    // ===== ALTITUDE FACTOR: ONE PER KEY WITH PRIORITY =====
    // Priority: LiDAR > Baro > Alt-Memory > Decaying-Z > Weak fallback
    bool alt_factor_added = false;
    bool used_real_altimeter = false;
    bool alt_constrained = false;
    const double z_pred = x0.z();           // signed NED (Down=+Z), so z_pred<0 means above ground
    const double z_abs  = std::abs(z_pred); // AGL magnitude, use for gating/thresholds

    auto add_alt = [&](double z_meas, const boost::shared_ptr<gtsam::noiseModel::Base>& nm,
                       const char* tag, bool real){
      // NED nav frame: negate AGL measurement (up-positive -> down-positive)
      graph.emplace_shared<chimera::factors::AltimeterRangeFactor>(Kp, -z_meas, nm);
      monitor_.updateAltitude(t);
      if(real) used_real_altimeter = true;
      alt_constrained = alt_constrained || real;
      alt_factor_added = true;
      last_alt_src_ = tag;
      if(key_ % 10 == 0) std::cout << "[ALT_USE k="<<key_<<" t="<<t<<"] "<<tag<<"\n";

      // Debug: Show AGL-vs-AGL comparison for LiDAR factors
      if(std::strcmp(tag, "lidar") == 0 && key_ % 10 == 0){
        double z_nav   = x0.z();              // NED Down+
        double agl_pred = -z_nav;             // Up+
        double agl_err  = agl_pred - z_meas;  // z_meas is already positive AGL
        std::cout << "[LIDAR_AGL k="<<key_<<"] meas="<<z_meas<<"m pred="<<agl_pred<<"m err="<<agl_err<<"m\n";
      }
    };

    // 1) LiDAR (highest priority - actual measurement)
    if(!alt_factor_added && lidar_available && !lidar_stuck_){
      // NED frame: z_pred is negative, lidar is positive AGL, so innovation = z_pred + lidar
      double innov = z_pred + lidar->range_mean;
      double lidar_sigma_adaptive = chimera::utils::AdaptiveConstraints::getLidarSigma(
                   key_, lidar_healthy_streak_, can_trust_lidar, innov);
      // Risk-budgeted sigma: blend with adaptive using max() (tightest valid bound)
      double lidar_sigma_risk = c_lidar * 0.05;
      double sigma = std::max(lidar_sigma_adaptive, lidar_sigma_risk);

      auto nm = gtsam::noiseModel::Robust::Create(
                  gtsam::noiseModel::mEstimator::Tukey::Create(4.685),
                  gtsam::noiseModel::Isotropic::Sigma(1, sigma));
      add_alt(lidar->range_mean, nm, "lidar", /*real=*/true);
      lidar_updates_++;

      // Phase 1: Track mode transition
      trackModeTransition(AltitudeSource::LIDAR, t, "lidar_healthy");
    }

    // ===== BARO TAKEOVER: Circuit breaker when Z diverges during baro-only mode =====
    // Detect takeover conditions: LiDAR unavailable, baro healthy, Z absurd
    double baro_agl_check=0.0, baro_sigma_check=0.0;
    if(!baro_takeover_active_ && !lidar_available && robustBaroAGL(now_abs, baro_agl_check, baro_sigma_check)){
      const bool baro_healthy = (baro_sigma_check <= 1.5) && (baro_agl_check > -20.0 && baro_agl_check < 300.0);
      if(baro_healthy && z_abs > 100.0){
        // ACTIVATE TAKEOVER: hot-reset smoother with short lag + baro anchor
        baro_takeover_active_ = true;
        // NED nav frame: negate AGL (up-positive -> down-positive)
        restartSmootherForTakeover(t, -baro_agl_check);
        std::cout<<"[BARO_TAKEOVER k="<<key_<<" t="<<t<<"] activated (Z_pred="<<z_abs
                 <<"m, baro="<<baro_agl_check<<"m)\n";

        // Phase 1: Track mode transition to takeover
        trackModeTransition(AltitudeSource::BARO_TAKEOVER, t,
                           "z_divergence=" + std::to_string(z_abs) + "m");

        // Skip rest of update - restart already seeded this key
        // Continue normal processing on next key with short lag + cut IMU chain
        takeover_just_activated_ = false;
        key_++;  // Increment key so next update continues from k+1
        return;
      }
    }

    // Check for recovery: deactivate takeover when Z is sane
    if(baro_takeover_active_ && z_abs < 30.0){
      baro_takeover_active_ = false;
      // Restore original lag window
      if(std::abs(cfg_.lag_seconds - saved_lag_seconds_) > 1e-6){
        cfg_.lag_seconds = saved_lag_seconds_;
        // Note: We could reinit here, but it's safer to just let it continue with short lag
        // The short lag won't hurt performance much and keeps the solution stable
      }
      std::cout<<"[BARO_TAKEOVER k="<<key_<<" t="<<t<<"] cleared (Z="<<z_abs<<"m)\n";

      // Phase 1: Note - actual mode will be set when next altitude source is selected
      // This just clears the takeover flag, mode transition happens in next update
    }

    // 2) Baro fallback (when LiDAR unavailable/stuck)
    double baro_agl=0.0, baro_sigma_base=0.0;
    if(!alt_factor_added && cfg_.enable_baro_fallback && robustBaroAGL(now_abs, baro_agl, baro_sigma_base)){
      boost::shared_ptr<gtsam::noiseModel::Base> nm;
      const char* tag = "baro";

      if(baro_takeover_active_){
        // TAKEOVER MODE: Non-robust, strong clamp - optimizer MUST obey this
        // strong clamp => take the smaller sigma
        double sigma = std::min(0.6, baro_sigma_base);
        nm = gtsam::noiseModel::Isotropic::Sigma(1, sigma);
        tag = "baro_takeover";
      } else {
        // NORMAL MODE: Robust (allows graceful outlier rejection)
        double sigma = baro_sigma_base;
        nm = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(1.5),
                    gtsam::noiseModel::Isotropic::Sigma(1, sigma));
      }

      // IMPORTANT: Use BaroFactor, not AltimeterRangeFactor (different semantics)
      // NED nav frame: negate AGL measurement (up-positive -> down-positive)
      graph.emplace_shared<chimera::factors::BaroFactor>(Kp, -baro_agl, nm);
      monitor_.updateAltitude(t);
      baro_updates_++;
      used_real_altimeter = true;
      alt_constrained = true;
      alt_factor_added = true;
      last_alt_src_ = tag;
      if(key_ % 10 == 0) std::cout << "[ALT_USE k="<<key_<<" t="<<t<<"] "<<tag<<"\n";

      // Phase 1: Track mode transition (already in takeover or normal baro)
      if(baro_takeover_active_){
        trackModeTransition(AltitudeSource::BARO_TAKEOVER, t, "continuing_takeover");
      } else {
        trackModeTransition(AltitudeSource::BARO, t, "lidar_unavailable");
      }

      // Refresh altitude memory from baro
      last_trusted_agl_ = baro_agl;
      last_trust_t_     = t;
      has_trusted_alt_  = true;
    }

    // 3) Altitude memory (soft constraint, decays over time)
    if(!alt_factor_added && has_trusted_alt_){
      const double dt_since_trust = std::max(0.0, t - last_trust_t_);
      const double sigma_mem = std::clamp(0.3 + 0.2 * dt_since_trust, 0.3, 5.0);
      auto nm = gtsam::noiseModel::Robust::Create(
                  gtsam::noiseModel::mEstimator::Huber::Create(1.5),
                  gtsam::noiseModel::Isotropic::Sigma(1, sigma_mem));
      add_alt(last_trusted_agl_, nm, "alt_memory", /*real=*/false);
    }

    // 4) Decaying Z prior (early initialization only, robust + innovation-gated)
    if(!alt_factor_added && key_ <= 40){
      double z_ref = (!range_boot_.lidar_stuck && std::isfinite(range_boot_.z0_agl_m))
                       ? range_boot_.z0_agl_m : 1.0;   // AGL (up+)
      double base_sigma = (key_<=10? 1.0 : (key_<=20? 2.0 : 5.0));
      const double z_ref_nav = -z_ref;                // AGL → nav Z (down+)
      double dz = std::abs(z_pred - z_ref_nav);
      // Inflate if way off to avoid yanking the graph
      double sigma = std::max(base_sigma, std::min(10.0, 0.5*dz));
      auto nm = gtsam::noiseModel::Robust::Create(
                  gtsam::noiseModel::mEstimator::Tukey::Create(4.685),
                  gtsam::noiseModel::Isotropic::Sigma(1, sigma));
      add_alt(z_ref, nm, "z_prior", /*real=*/false);
    }

    // 5) Weak fallback (very soft, robust, just to keep observability)
    if(!alt_factor_added){
      double z_ref = -z_pred;  // AGL (up+); add_alt() negates → nav Z = z_pred
      auto nm = gtsam::noiseModel::Robust::Create(
                  gtsam::noiseModel::mEstimator::Tukey::Create(4.685),
                  gtsam::noiseModel::Isotropic::Sigma(1, 10.0));
      add_alt(z_ref, nm, "z_weak", /*real=*/false);
    }

    // ---------------- OPTICAL FLOW ----------------
    of_rays_used_=0; of_rays_rejected_=0; of_resid_sum_sq_=0.0;

    auto flow = nearestByTime(sensors_.optical_flow, now_abs, 0.05);

    // Phase 1: Evaluate Optical Flow health
    if(flow){
      last_of_health_ = of_health_mon_.evaluate(*flow);
    }

    static double last_good_agl = 1.0;
    double agl_gate;
    if(lidar_available){ agl_gate = lidar->range_mean; last_good_agl = agl_gate; }
    else if(key_<=40 && range_boot_.lidar_stuck){ agl_gate = 1.0; last_good_agl=agl_gate; }
    else if(baro && std::isfinite(baro0_msl_)){
      agl_gate = std::clamp((baro->altitude - baro0_msl_) + range_boot_.z0_agl_m, 0.3, 120.0);
      last_good_agl = agl_gate;
    } else agl_gate = last_good_agl;

    const double Z_eff = effectiveDepthMeters_AGL(x0, cfg_.R_bc, agl_gate);
    const double Z_gate = std::clamp(Z_eff, 1.0, 300.0);   // Raised from 80 → 300m
    const auto rpy = x0.rotation().rpy();
    const bool tilt_ok = (std::abs(rpy(0))<1.047) && (std::abs(rpy(1))<1.047);
    const bool flow_ok = (Z_gate>=1.0 && Z_gate<=300.0) && flow && tilt_ok;

    // Debug: Log OF gate status every 10 keyframes
    if(key_ % 10 == 0){
      std::cout << "[OF_GATE k=" << key_ << "] flow=" << (flow ? "YES" : "NO")
                << " Z_eff=" << Z_eff << "m"
                << " roll=" << (rpy(0)*57.2958) << "° pitch=" << (rpy(1)*57.2958) << "°"
                << " tilt_ok=" << (tilt_ok ? "YES" : "NO")
                << " flow_ok=" << (flow_ok ? "YES" : "NO")
                << " key_>0=" << (key_>0 ? "YES" : "NO") << "\n";
    }

    // time-align gyro to the flow timestamp (falls back to zero vec)
    Eigen::Vector3d gyro_for_flow = Eigen::Vector3d::Zero();
    if(flow){
      auto imu_at_flow = nearestByTime(sensors_.imu, flow->t, 0.02);
      if(imu_at_flow) gyro_for_flow = fixG(imu_at_flow->gyro);
    }

    if(flow_ok && key_>0){
      // Optical flow noise: adaptive (depth + quality aware, guards div-by-zero)
      // Compute average texture quality
      double avg_texture = 0.0;
      int valid_rays = 0;
      for(const auto& r: flow->rays){
        const double mag = std::sqrt(r.flow_u*r.flow_u + r.flow_v*r.flow_v);
        if((Z_eff>0.5 && Z_eff<300.0) && (mag<cfg_.of_max_flow) && (r.texture_var>cfg_.of_min_tex)){
          avg_texture += r.texture_var;
          valid_rays++;
        }
      }
      avg_texture = valid_rays > 0 ? avg_texture / valid_rays : 100.0;

      double of_sigma_adaptive = chimera::utils::AdaptiveConstraints::getOpticalFlowSigma(
          Z_gate, flow->rays.size(), avg_texture);

      // Risk-budgeted sigma: inflate adaptive sigma by Chebyshev scale
      // Ensure risk scaling never reduces sigma
      double of_sigma = of_sigma_adaptive * std::max(1.0, c_of);
      of_depth_scale_ = of_sigma / 2.0;  // Telemetry tracking (normalized to base 2.0)

      auto base = gtsam::noiseModel::Isotropic::Sigma(2, of_sigma);
      auto huber= gtsam::noiseModel::mEstimator::Huber::Create(1.5);
      auto nm   = gtsam::noiseModel::Robust::Create(huber, base);

      std::vector<boost::shared_ptr<chimera::factors::OpticalFlowFactor>> facs;
      facs.reserve(flow->rays.size());

      for(const auto& r: flow->rays){
        const double mag = std::sqrt(r.flow_u*r.flow_u + r.flow_v*r.flow_v);
        const bool ok = (Z_eff>0.5 && Z_eff<300.0) && (mag<cfg_.of_max_flow) && (r.texture_var>cfg_.of_min_tex);
        if(ok){
          gtsam::Vector2 meas(r.flow_u, r.flow_v);
          auto f = boost::make_shared<chimera::factors::OpticalFlowFactor>(
              Kp, Kv, Kb, meas, gyro_for_flow,
              cfg_.focal_px, cfg_.focal_px, 0.0, 0.0,
              r.norm_x, r.norm_y, cfg_.R_bc, nm);
          facs.push_back(f); ++of_rays_used_; of_resid_sum_sq_ += mag*mag;
        } else ++of_rays_rejected_;
      }

      // Debug: Show ray counts every 10 keyframes
      if(key_ % 10 == 0){
        std::cout << "[OF_RAYS k=" << key_ << "] total=" << flow->rays.size()
                  << " used=" << of_rays_used_ << " rejected=" << of_rays_rejected_
                  << " min_required=" << cfg_.of_min_rays << "\n";
      }

      if(of_rays_used_ >= cfg_.of_min_rays){
        for(auto& f: facs) graph.push_back(f);
        monitor_.updateVelocity(t);
        of_updates_++; of_rays_used_total_+=of_rays_used_; of_rays_rejected_total_+=of_rays_rejected_;
        // Calculate and store RMS for this update
        last_of_rms_ = (of_rays_used_>0)? std::sqrt(of_resid_sum_sq_/of_rays_used_) : 0.0;
        if(key_ % 10 == 0){
          std::cout << "[OF_UPDATE k=" << key_ << "] Added " << facs.size() << " OF factors, total_updates=" << of_updates_ << "\n";
        }
      } else {
        of_rays_rejected_+=of_rays_used_; of_rays_used_=0; of_resid_sum_sq_=0.0;
        // Don't update last_of_rms_ when update fails - keep previous value
        if(key_ % 10 == 0){
          std::cout << "[OF_SKIP k=" << key_ << "] Insufficient rays (had " << (of_rays_used_+of_rays_rejected_) << ", need " << cfg_.of_min_rays << ")\n";
        }
      }
    }

    // velocity magnitude leash
    if(key_>0){
      const double vmag = v0.norm();
      if(vmag > 15.0){
        const double leash = std::clamp(vmag, 0.0, 15.0);
        auto vleash = gtsam::noiseModel::Isotropic::Sigma(1, 3.0);
        graph.emplace_shared<chimera::factors::VelocityMagnitudeFactor>(Kv, leash, vleash);
      }
    }

    // OF health → airspeed fallback
    // (RMS is now stored in last_of_rms_ for telemetry)
    bool of_healthy = (of_rays_used_>=cfg_.of_min_rays);
    if(!of_healthy){ if(of_bad_streak_==0) of_bad_start_t_=t; of_bad_streak_++; } else of_bad_streak_=0;

    // Airspeed (always use when available)
    auto asp = nearestByTime(sensors_.airspeed, now_abs, 0.15);
    enable_airspeed_ = (asp && key_>=1 && asp->airspeed>3.0 && asp->airspeed<60.0);
    if(enable_airspeed_){
      auto base = gtsam::noiseModel::Isotropic::Sigma(1, safe_boot_? 1.5:1.0);
      auto huber= gtsam::noiseModel::mEstimator::Huber::Create(1.5);
      auto nm   = gtsam::noiseModel::Robust::Create(huber, base);
      graph.emplace_shared<chimera::factors::AeroVelocityWindFactor>(Kp,Kv,Kw, asp->airspeed, nm);
      airspeed_updates_++;
      monitor_.updateVelocity(t);
    }

    // ===== VELOCITY DAMPING: observability-aware when constraints are weak =====
    vel_prior_added_ = false;
    {
      double sigma_xy=5.0, sigma_z=5.0;
      chimera::utils::AdaptiveConstraints::getVelocityPriorSigma(
          /*have_lidar=*/alt_constrained,
          /*have_of=*/of_healthy,
          /*have_airspeed=*/enable_airspeed_,
          sigma_xy, sigma_z);

      // Apply damping when altitude is poorly observed AND no horizontal constraints
      const bool need_damping = !alt_constrained && (!of_healthy && !enable_airspeed_) && key_>0;
      if(need_damping){
        auto Nv = gtsam::noiseModel::Diagonal::Sigmas(
                    (gtsam::Vector(3) << sigma_xy, sigma_xy, sigma_z).finished());
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(Kv, gtsam::Vector3::Zero(), Nv);
        vel_prior_added_ = true;
        if(key_%10==0) std::cout<<"[VEL_DAMP k="<<key_<<"] XY="<<sigma_xy<<" Z="<<sigma_z<<"\n";
      }
    }

    // ========== DIAGNOSTIC: Factor Error Logging ==========
    // Log factor errors BEFORE optimization to identify problematic constraints
    if(key_ > 0){
      // FIX #3: Use hoisted estimate instead of redundant call
      const auto current_est = est_prev_all;

      // Merge current estimate with new initial values for factors using new keys
      gtsam::Values combined_est = current_est;
      for(const auto& key_value : init){
        if(!combined_est.exists(key_value.key)){
          combined_est.insert(key_value.key, key_value.value);
        }
      }

      // Compute errors for each factor type
      double total_error = 0.0;
      size_t lidar_factor_count = 0;
      double lidar_factor_error = 0.0;

      for(const auto& factor : graph){
        try{
          double err = factor->error(combined_est);
          total_error += err;

          // Check if this is a LiDAR AltimeterRangeFactor by attempting dynamic cast
          // (In single-file mode, we check factor size as proxy)
          if(factor->size() == 1){  // Single-key factor (could be altimeter)
            // Check if the key is a pose key (X symbol)
            auto keys = factor->keys();
            if(!keys.empty()){
              gtsam::Symbol sym(keys[0]);
              if(sym.chr() == 'x' && err > 10.0){  // Large error on pose key
                lidar_factor_count++;
                lidar_factor_error += err;

                // Log details for large-error factors
                if(err > 100.0 && key_ % 5 == 0){
                  std::cout << "[FACTOR_ERROR k=" << key_ << " t=" << t
                            << "] Large error on pose factor: " << err
                            << " (likely altitude)" << std::endl;
                }
              }
            }
          }
        } catch(...){
          // Skip factors that can't be evaluated (missing keys, etc.)
        }
      }

      // Store values for telemetry
      last_total_graph_error_ = total_error;
      last_lidar_factor_count_ = lidar_factor_count;
      last_lidar_factor_error_ = lidar_factor_error;

      // Log summary every 5 keyframes during critical period (t=10-15s)
      if(key_ % 5 == 0 || (t >= 10.0 && t <= 15.0)){
        std::cout << "[PRE_OPT k=" << key_ << " t=" << t << "] "
                  << "total_error=" << total_error
                  << " graph_size=" << graph.size();
        if(lidar_factor_count > 0){
          std::cout << " lidar_factors=" << lidar_factor_count
                    << " lidar_error=" << lidar_factor_error
                    << " avg_lidar_err=" << (lidar_factor_error/lidar_factor_count);
        }
        std::cout << std::endl;
      }
    }
    // ========== END DIAGNOSTIC ==========

    // submit factors (with timing)
    chimera::utils::Timer t_smoother;
    smoother_.addFactors(graph, init, t);
    smoother_cpu_ms_ = t_smoother.tocMs();

    // reset preintegration bias to latest
    if(key_>0){
      const auto est = smoother_.calculateEstimate();
      if(est.exists(B(key_))){
        const auto b = est.at<gtsam::imuBias::ConstantBias>(B(key_));
        pim_->resetIntegrationAndSetBias(b);

        // Update learned bias mean (OU process) with EMA smoothing
        if(key_ > 5){
          double alpha = 0.05; // EMA smoothing factor
          learned_bias_mean_ = gtsam::imuBias::ConstantBias(
            learned_bias_mean_.accelerometer() * (1-alpha) + b.accelerometer() * alpha,
            learned_bias_mean_.gyroscope() * (1-alpha) + b.gyroscope() * alpha
          );
          bias_mean_update_count_++;
        }
      }
    }

    // if first trusted LiDAR just arrived, reanchor immediately to lock altitude
    immediateReanchorIfRequested(t);

    key_++;
  }

  void emitTelemetry(){
    constexpr double kPi = 3.14159265358979323846;
    const double t = last_imu_t_.value_or(0.0);
    const auto x = ukf_.getPose(); const auto v = ukf_.getVelocity();

    // Contract monitoring
    const bool contract_ok = monitor_.isContractOk(t);

    // Prefer smoother bias over UKF bias (more accurate after optimization)
    gtsam::Vector3 gb(0,0,0), ab(0,0,0);
    if(key_>0){
      const auto est = smoother_.calculateEstimate();
      if(est.exists(B(key_-1))){
        const auto bias = est.at<gtsam::imuBias::ConstantBias>(B(key_-1));
        gb = bias.gyroscope(); ab = bias.accelerometer();
      }
    }
    if(gb.norm()==0.0){ gb = ukf_.getGyroBias(); ab = ukf_.getAccelBias(); }

    double wind_x=0, wind_y=0;
    if(key_>0){
      const auto est = smoother_.calculateEstimate();
      if(est.exists(W(key_-1))){ auto w = est.at<gtsam::Point2>(W(key_-1)); wind_x=w.x(); wind_y=w.y(); }
    }

    const double agl = std::abs(x.z());
    const double z_eff = effectiveDepthMeters_AGL(x, cfg_.R_bc, agl);

    const double gyro_bias_dps = gb.norm()*180.0/kPi;
    const double accel_bias = ab.norm();
    const double of_rms = last_of_rms_; // Use stored value from most recent OF update

    const char* of_health = "bad";
    // Health based on recent update counts (using _total counters trend)
    const int recent_rays = of_rays_used_total_; // Could track delta if needed
    if((recent_rays>=cfg_.of_min_rays) && (of_rms>0.0 && of_rms<=cfg_.of_rms_good)) of_health="healthy";
    else if((recent_rays>=2) || (of_rms>cfg_.of_rms_good && of_rms<=cfg_.of_rms_bad)) of_health="degraded";

    std::ostringstream j;
    j << "{\"t\":"<<t
      <<",\"pos\":["<<x.x()<<","<<x.y()<<","<<x.z()<<"]"
      <<",\"vel\":["<<v.x()<<","<<v.y()<<","<<v.z()<<"]"
      <<",\"agl_m\":"<<agl<<",\"z_eff_m\":"<<z_eff
      <<",\"mag_updates\":"<<mag_updates_
      <<",\"lidar_updates\":"<<lidar_updates_
      <<",\"baro_updates\":"<<baro_updates_
      <<",\"airspeed_updates\":"<<airspeed_updates_
      <<",\"of_updates\":"<<of_updates_
      <<",\"of_rays_used\":"<<of_rays_used_total_
      <<",\"of_rays_rejected\":"<<of_rays_rejected_total_
      <<",\"of_rms_px\":"<<of_rms
      <<",\"of_health\":\""<<of_health<<"\""
      <<",\"wind\":["<<wind_x<<","<<wind_y<<"]"
      <<",\"gyro_bias_dps\":"<<gyro_bias_dps
      <<",\"accel_bias_mps2\":"<<accel_bias
      <<",\"graph_error_total\":"<<last_total_graph_error_
      <<",\"graph_error_lidar_count\":"<<last_lidar_factor_count_
      <<",\"graph_error_lidar_total\":"<<last_lidar_factor_error_
      <<",\"ukf_cpu_ms\":"<<ukf_cpu_ms_
      <<",\"smoother_cpu_ms\":"<<smoother_cpu_ms_
      <<",\"contract_ok\":"<<(contract_ok?"true":"false")
      <<",\"anchor\":"<<(last_anchor_applied_?"true":"false")
      <<",\"anchor_age_s\":"<<last_anchor_age_s_
      <<",\"lidar_healthy_streak\":"<<lidar_healthy_streak_
      <<",\"of_bad_streak\":"<<of_bad_streak_
      <<",\"enable_airspeed\":"<<(enable_airspeed_?"true":"false")
      <<",\"vel_prior_added\":"<<(vel_prior_added_?"true":"false")
      <<",\"of_depth_scale\":"<<of_depth_scale_
      <<",\"alt_src\":\""<<last_alt_src_<<"\""
      // Phase 1: Sensor Health Monitoring
      <<",\"sensor_health\":{"
        <<"\"lidar\":{\"score\":"<<last_lidar_health_.score
          <<",\"is_stuck\":"<<(last_lidar_health_.is_stuck?"true":"false")
          <<",\"noise_level\":"<<last_lidar_health_.noise_level
          <<",\"reason\":\""<<last_lidar_health_.reason<<"\"}"
        <<",\"baro\":{\"score\":"<<last_baro_health_.score
          <<",\"noise_level\":"<<last_baro_health_.noise_level
          <<",\"drift_rate\":"<<last_baro_health_.drift_rate
          <<",\"rapid_change\":"<<(last_baro_health_.rapid_change?"true":"false")
          <<",\"reason\":\""<<last_baro_health_.reason<<"\"}"
        <<",\"mag\":{\"score\":"<<last_mag_health_.score
          <<",\"norm\":"<<last_mag_health_.norm
          <<",\"interference\":"<<(last_mag_health_.interference?"true":"false")
          <<",\"reason\":\""<<last_mag_health_.reason<<"\"}"
        <<",\"of\":{\"score\":"<<last_of_health_.score
          <<",\"texture_quality\":"<<last_of_health_.texture_quality
          <<",\"saturated\":"<<(last_of_health_.saturated?"true":"false")
          <<",\"reason\":\""<<last_of_health_.reason<<"\"}"
        <<",\"imu\":{\"score\":"<<last_imu_health_.score
          <<",\"temperature\":"<<last_imu_health_.temperature
          <<",\"gyro_saturated\":"<<(last_imu_health_.gyro_saturated?"true":"false")
          <<",\"accel_saturated\":"<<(last_imu_health_.accel_saturated?"true":"false")
          <<",\"reason\":\""<<last_imu_health_.reason<<"\"}"
      <<"}"
      <<",\"altitude_source\":\""<<toString(current_alt_source_)<<"\""
      <<",\"mode_transitions\":"<<mode_transitions_.size()
      // Phase 1: Watchdog System Telemetry
      <<",\"watchdog\":{"
        <<"\"all_healthy\":"<<(watchdog_.getReport().all_healthy?"true":"false")
        <<",\"failure_count\":"<<watchdog_.getReport().failures.size();

    // Add deadline violations
    const auto& deadline_violations = watchdog_.getReport().deadline_violations;
    j <<"," <<"\"deadline_violations\":{"
        <<"\"has_violations\":"<<(deadline_violations.has_violations?"true":"false")
        <<",\"timed_out_sensors\":[";
    for (size_t i = 0; i < deadline_violations.timed_out_sensors.size(); ++i) {
      if (i > 0) j << ",";
      j << "\"" << deadline_violations.timed_out_sensors[i] << "\"";
    }
    j <<"]}"
      <<"}"
      <<"}";

    // Phase 2: Schema validation (optional, enabled by default)
    std::string json_output = j.str();
    if (enable_schema_validation_) {
      auto validation_result = chimera::utils::SchemaValidator::validate(json_output);
      if (!validation_result.valid) {
        schema_validation_errors_++;
        if (schema_validation_errors_ <= 5) {  // Only print first 5 errors
          std::cerr << "[SCHEMA ERROR k=" << key_ << "] " << validation_result.summary() << "\n";
          if (schema_validation_errors_ == 5) {
            std::cerr << "[SCHEMA] Suppressing further errors...\n";
          }
        }
      }
    }

    telem_.writeLine(json_output);

    // Clear one-shot anchor flag
    last_anchor_applied_ = false;
  }
};

}  // namespace core
}  // namespace chimera
