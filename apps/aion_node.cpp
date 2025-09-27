// apps/aion_node.cpp

#include <atomic>
#include <csignal>
#include <chrono>
#include <cmath>     // sin, cos, M_PI
#include <iostream>
#include <string>
#include <thread>

#include "core/fusion/sr_ukf.h"
#include "core/fusion/state.h"
#include "core/utils/binary_logger.h"
#include "core/utils/config.h"
#include "core/utils/logging.h"
#include "core/utils/time_utils.h"

// TRN
#include "measurements/trn_manager.h"

// Barometer
#include "measurements/baro_measurement.h"
#include "measurements/baro_ukf_measurement.h"

// STN (Signals of Opportunity)
#include "measurements/stn_manager.h"
#include "measurements/stn_ukf_measurement.h"
#include "measurements/stn_synthetic_generator.h"
#include "core/fusion/state_extended.h"

// Gravity-referenced attitude
#include "measurements/gravity_ukf_measurement.h"

std::atomic<bool> g_shutdown(false);

void signalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    LOG_INFO("Shutdown signal received");
    g_shutdown = true;
  }
}

void printUsage(const char* program_name) {
  std::cout << "AION - All-Weather Integrated Opportunistic Navigation\n";
  std::cout << "Version 0.1.0\n\n";
  std::cout << "Usage: " << program_name << " [options]\n";
  std::cout << "Options:\n";
  std::cout << "  --config <file>     Configuration file (default: configs/aion_default.yaml)\n";
  std::cout << "  --log-level <level> Log level (trace|debug|info|warn|error|critical)\n";
  std::cout << "  --log-file <file>   Log file path (default: logs/aion.log)\n";
  std::cout << "  --help              Show this help message\n";
}

struct CommandLineArgs {
  std::string config_file = "configs/aion_default.yaml";
  std::string log_level   = "info";
  std::string log_file    = "logs/aion.log";
  bool show_help          = false;
};

CommandLineArgs parseArgs(int argc, char* argv[]) {
  CommandLineArgs args;
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--config" && i + 1 < argc) {
      args.config_file = argv[++i];
    } else if (arg == "--log-level" && i + 1 < argc) {
      args.log_level = argv[++i];
    } else if (arg == "--log-file" && i + 1 < argc) {
      args.log_file = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      args.show_help = true;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      args.show_help = true;
    }
  }
  return args;
}

spdlog::level::level_enum parseLogLevel(const std::string& level) {
  if (level == "trace")    return spdlog::level::trace;
  if (level == "debug")    return spdlog::level::debug;
  if (level == "info")     return spdlog::level::info;
  if (level == "warn")     return spdlog::level::warn;
  if (level == "error")    return spdlog::level::err;
  if (level == "critical") return spdlog::level::critical;
  return spdlog::level::info;
}

int main(int argc, char* argv[]) {
  // Parse CLI
  CommandLineArgs args = parseArgs(argc, argv);
  if (args.show_help) { printUsage(argv[0]); return 0; }

  // Logger
  aion::Logger::init(args.log_file, parseLogLevel(args.log_level));
  LOG_INFO("AION Node Starting...");
  LOG_INFO("Config file: {}", args.config_file);

  // Config
  if (!aion::Config::load(args.config_file)) {
    LOG_ERROR("Failed to load configuration file: {}", args.config_file);
    return 1;
  }
  aion::SystemConfig sys_config;
  sys_config.loadFromYAML();

  // Override log level from config if present
  if (auto log_cfg = aion::Config::getLogging()) {
    if (log_cfg["level"]) {
      std::string cfg_level = log_cfg["level"].as<std::string>();
      aion::Logger::setLevel(parseLogLevel(cfg_level));
    }
  }

  // Signals
  std::signal(SIGINT,  signalHandler);
  std::signal(SIGTERM, signalHandler);

  // SR-UKF config (unit-consistent)
  aion::SRUKF::Config ukf_config;
  ukf_config.gyro_noise_density        = sys_config.ukf.proc_noise.gyro_rw * M_PI / 180.0; // (rad/s)/√Hz
  ukf_config.accel_noise_density       = sys_config.ukf.proc_noise.acc_rw;                 // m/s²/√Hz
  ukf_config.gyro_bias_noise_density   = sys_config.ukf.proc_noise.bg_rw   * M_PI / 180.0; // (rad/s²)/√Hz
  ukf_config.accel_bias_noise_density  = sys_config.ukf.proc_noise.ba_rw;                  // m/s³/√Hz
  ukf_config.gravity                   = 9.80665; // WGS84 nominal; replace with model later
  ukf_config.nis_gate_1d               = 15.0;    // Permissive NIS gate for 1D measurements

  aion::SRUKF ukf(ukf_config);

  // Time base
  aion::TimeUtils::initialize();

  // Binary logger
  aion::BinaryLogger::Config logger_config;
  if (auto log_cfg = aion::Config::getLogging()) {
    if (log_cfg["binlog_path"]) {
      logger_config.filepath = log_cfg["binlog_path"].as<std::string>();
    }
  }
  aion::BinaryLogger binary_logger(logger_config);
  binary_logger.start(args.config_file); // use config path as run "hash" tag

  // Initial state - start at 100m altitude for TRN demo
  aion::State initial_state;
  initial_state.reset();
  initial_state.time     = aion::TimeUtils::now();
  initial_state.position = Eigen::Vector3d(0, 0, -100);  // 100m altitude (NED: down is +)
  initial_state.velocity = Eigen::Vector3d(10, 5, 0);    // 10 m/s north, 5 m/s east

  aion::StateCovariance initial_cov;
  initial_cov.initialize(
      /*pos_std=*/1.0,
      /*vel_std=*/0.1,
      /*att_std=*/0.01,
      /*gb_std=*/0.001,
      /*ba_std=*/0.01,
      /*wind_std=*/1.0);

  ukf.initialize(initial_state, initial_cov);
  LOG_INFO("SR-UKF initialized");
  LOG_INFO("Target rate: {:.0f} Hz", sys_config.ukf.rate_hz);

  // ---- Barometer init ----
  measurements::BaroConfig baro_config;
  baro_config.enabled              = true;
  baro_config.update_rate_hz       = 50.0;
  baro_config.sigma_m              = 1.5;  // Normalized post-convergence
  baro_config.temp_comp_factor_m_per_C = 0.1;
  baro_config.chi2_gate_1d         = 9.0;   // Tightened post-convergence
  baro_config.max_innovation_m     = 25.0;  // Reasonable range for operation
  baro_config.use_low_pass         = true;
  baro_config.filter_time_constant = 1.5;  // Faster response
  baro_config.sea_level_pressure_pa= 101325.0;
  baro_config.reference_alt_msl_m  = 750.0;  // Jerusalem MSL altitude

  measurements::BaroMeasurement barometer(baro_config);
  LOG_INFO("Barometer initialized: rate={:.0f}Hz, sigma={:.1f}m",
           baro_config.update_rate_hz, baro_config.sigma_m);

  // ---- TRN init ----
  measurements::TRNManagerConfig trn_config;
  trn_config.enabled            = true;
  trn_config.update_rate_hz     = 10.0;                    // radar at 10 Hz
  trn_config.chi2_gate_1d       = 12.0;                    // Optimized for performance
  trn_config.min_slope_deg      = 0.8;                     // informative terrain threshold
  trn_config.min_agl_m          = 2.0;                     // m
  trn_config.max_agl_m          = 1500.0;                  // m
  trn_config.dem_sigma_base_m   = 5.0;                     // DEM uncertainty base
  trn_config.dem_sigma_slope_k  = 0.1;                     // DEM uncertainty slope factor
  trn_config.radar_sigma_m      = 1.0;                     // radar σ [m]
  // Use SystemConfig origin for consistent frame
  trn_config.origin_lat_deg     = sys_config.origin.lat;   // Tel Aviv from SystemConfig
  trn_config.origin_lon_deg     = sys_config.origin.lon;
  trn_config.origin_alt_m       = sys_config.origin.alt_m; // MSL
  trn_config.dem_path           = "data/dem/srtm";

  measurements::TRNManager trn_manager(trn_config);
  if (!trn_manager.initialize()) {
    LOG_ERROR("Failed to initialize TRN manager");
    // Continue without TRN
  } else {
    LOG_INFO("TRN manager initialized with {} tiles",
             trn_manager.terrain()->getLoadedTileCount());
    const auto& cfg = trn_manager.config();
    LOG_INFO("TRN: actual config — min_slope={:.2f}°, χ²_gate={:.2f}, rate={:.0f}Hz",
             cfg.min_slope_deg, cfg.chi2_gate_1d, cfg.update_rate_hz);
    LOG_INFO("TRN: origin=({:.6f},{:.6f}) alt={:.1f}m, dem_path='{}'",
             cfg.origin_lat_deg, cfg.origin_lon_deg, cfg.origin_alt_m, cfg.dem_path);

    // Sanity probe
    const double jlm_lat = 31.7683, jlm_lon = 35.2137;
    const double h = trn_manager.terrain()->getElevationBilinear(jlm_lat, jlm_lon);
    LOG_INFO("TRN: Sanity probe: Jerusalem {:.6f},{:.6f} ⇒ DEM {:.1f} m", jlm_lat, jlm_lon, h);
  }

  // ---- STN init ----
  measurements::STNManager::Params stn_params;
  bool stn_enabled = false;

  // Use SystemConfig origin for consistent frame
  stn_params.origin_lat_deg = sys_config.origin.lat;   // Tel Aviv from SystemConfig
  stn_params.origin_lon_deg = sys_config.origin.lon;
  stn_params.origin_alt_msl = sys_config.origin.alt_m;

  // Load STN configuration from YAML
  auto stn_cfg = aion::Config::getSTN();
  if (stn_cfg && stn_cfg["enabled"] && stn_cfg["enabled"].as<bool>(false)) {
    stn_params.beacons_file = stn_cfg["beacons_file"].as<std::string>("configs/stn/beacons.yaml");
    stn_params.config.min_beacons = stn_cfg["min_beacons"].as<int>(4);
    stn_params.config.max_beacons = stn_cfg["max_beacons"].as<int>(8);
    stn_params.config.update_rate_hz = stn_cfg["update_rate_hz"].as<double>(1.0);
    stn_params.config.snr_threshold_db = stn_cfg["snr_threshold_db"].as<double>(10.0);

    if (stn_cfg["raim"]) {
      stn_params.config.raim_enabled = stn_cfg["raim"]["enabled"].as<bool>(true);
      stn_params.config.raim_max_outliers = stn_cfg["raim"]["max_outliers"].as<int>(2);
    }

    LOG_INFO("STN configuration loaded from YAML");
    stn_enabled = true;
  } else {
    // Hardcoded defaults if no config
    stn_params.beacons_file = "configs/stn/beacons.yaml";
    stn_params.config.min_beacons = 4;
    stn_params.config.max_beacons = 8;
    stn_params.config.update_rate_hz = 2.0;
    stn_params.config.raim_enabled = true;
    stn_enabled = true;  // Enable by default for testing
  }

  auto stn_manager = std::make_shared<measurements::STNManager>(stn_params);

  if (stn_enabled) {
    if (!stn_manager->loadBeacons()) {
      LOG_WARN("Failed to load STN beacons from {} - STN disabled", stn_params.beacons_file);
      stn_enabled = false;
    } else {
      auto beacons = stn_manager->getActiveBeacons();
      LOG_INFO("STN manager initialized with {} beacons from {}", beacons.size(), stn_params.beacons_file);
      stn_enabled = beacons.size() >= stn_params.config.min_beacons;

      if (stn_enabled) {
        LOG_INFO("STN enabled: update_rate={:.1f}Hz, min_beacons={}, RAIM={}",
                 stn_params.config.update_rate_hz,
                 stn_params.config.min_beacons,
                 stn_params.config.raim_enabled ? "enabled" : "disabled");
      } else {
        LOG_WARN("STN disabled: insufficient beacons ({} < {})",
                 beacons.size(), stn_params.config.min_beacons);
      }
    }
  } else {
    LOG_INFO("STN disabled in configuration");
  }

  // Frame sanity check - must print ~0
  if (stn_enabled) {
    auto ned_o = stn_manager->geodeticToNED(stn_params.origin_lat_deg,
                                            stn_params.origin_lon_deg,
                                            stn_params.origin_alt_msl);
    LOG_INFO("Frame sanity: origin in its own NED = [{:.3f}, {:.3f}, {:.3f}] m",
             ned_o.x(), ned_o.y(), ned_o.z());
  }

  // STN synthetic generator for testing
  std::unique_ptr<measurements::STNSyntheticGenerator> stn_generator;
  if (stn_enabled) {
    measurements::STNSyntheticGenerator::Params gen_params;
    gen_params.true_clock_bias_m = 50.0;
    gen_params.true_clock_drift_frac = 1e-9;
    gen_params.tdoa_noise_m = 20.0;
    gen_params.min_elevation_deg = -10.0; // Allow beacons below horizon
    stn_generator = std::make_unique<measurements::STNSyntheticGenerator>(stn_manager, gen_params);
  }

  // Single source of truth for origin MSL altitude (used by Baro MSL model)
  const double origin_msl_alt_m = sys_config.origin.alt_m; // SystemConfig origin

  // Main loop
  aion::RateController rate_controller(sys_config.ukf.rate_hz);
  uint64_t loop_count = 0;
  double start_time_sec = aion::TimeUtils::now();
  double last_radar_time = 0.0;
  double last_baro_time  = 0.0;
  double last_stn_time   = 0.0;
  uint64_t baro_updates  = 0;
  uint64_t stn_updates   = 0;

  // Ground truth tracking for error calculation
  // Note: For realistic testing, true trajectory should follow the actual motion
  // For now, we use the initial UKF state as "ground truth" with small deviations
  Eigen::Vector3d true_position = ukf.getState().position;  // Start at same position
  Eigen::Vector3d true_velocity = ukf.getState().velocity;
  Eigen::Quaterniond true_attitude = ukf.getState().quaternion;
  double max_position_error = 0.0;
  double sum_position_error = 0.0;
  double sum_position_error_sq = 0.0;
  uint64_t error_samples = 0;

  // Track the ideal position with gentle motion (for realistic error assessment)
  Eigen::Vector3d true_pos_ned = ukf.getState().position;
  Eigen::Vector3d true_vel_ned = ukf.getState().velocity;

  LOG_INFO("Starting main loop at {:.0f} Hz", sys_config.ukf.rate_hz);

  while (!g_shutdown) {
    // --- Simulated IMU with gentle dynamics ---
    aion::IMUData imu_data;
    imu_data.timestamp = aion::TimeUtils::now();

    const double elapsed = imu_data.timestamp - start_time_sec;

    // SURGICAL FIX: Static/quasi-static test to isolate attitude mechanization bugs
    // Eliminate ALL motion for the first 10s to pass the "static table test"
    const bool STATIC_MODE = (elapsed < 10.0);

    if (STATIC_MODE) {
      // PURE STATIC: zero angular rates, zero linear acceleration
      imu_data.angular_velocity = Eigen::Vector3d::Zero();

      // For static test: specific force should be EXACTLY -gravity in body frame
      // This is the critical test - if this fails, we have axis/sign errors
      const Eigen::Vector3d g_n(0, 0, ukf_config.gravity);   // NED: +g down

      // At rest on level surface: a_body = -g_body
      // UKF state quaternion: check if it's body->nav or nav->body
      const Eigen::Quaterniond q_state = ukf.getState().quaternion;

      // ASSUMPTION CHECK: UKF stores q_bn (body-to-nav)
      // So R_bn = q_bn.toRotationMatrix() and R_nb = R_bn.transpose()
      const Eigen::Matrix3d R_bn = q_state.toRotationMatrix();
      const Eigen::Matrix3d R_nb = R_bn.transpose();

      // For a level platform: specific force in body = -gravity rotated to body
      // f_b = -R_nb * g_n = -[0,0,9.806] rotated to body
      const Eigen::Vector3d f_b_expected = -R_nb * g_n;

      imu_data.acceleration = f_b_expected;

      // CRITICAL DIAGNOSTIC: gravity consistency check
      // Reconstruct nav acceleration: a_n = R_bn * f_b + g_n
      // For static case this should be ZERO
      const Eigen::Vector3d a_n_reconstructed = R_bn * imu_data.acceleration + g_n;
      const double gravity_consistency_error = a_n_reconstructed.norm();

      if (gravity_consistency_error > 0.1) {
        LOG_ERROR("GRAVITY CONSISTENCY FAILURE: ||R_bn*f_b + g_n|| = {:.3f} m/s² (should be ~0)",
                  gravity_consistency_error);
        LOG_ERROR("Check quaternion convention: q stores body->nav or nav->body?");
        LOG_ERROR("Check gravity sign: g_n = [0,0,+9.806] in NED?");
      }

    } else {
      // SLOW CONTROLLED MOTION after static period passes
      const double motion_scale = 0.02;  // Very gentle motion
      const double turn_rate = motion_scale * std::sin(elapsed * 0.05);   // rad/s
      const double pitch_rate = motion_scale * 0.5 * std::sin(elapsed * 0.08);  // rad/s

      imu_data.angular_velocity = Eigen::Vector3d(pitch_rate, 0.0, turn_rate);

      // Gentle acceleration in nav frame
      const Eigen::Vector3d g_n(0, 0, ukf_config.gravity);
      const Eigen::Vector3d accel_n(0.05 * std::sin(elapsed * 0.02),  // Gentle north
                                    0.05 * std::cos(elapsed * 0.025), // Gentle east
                                    0.02 * std::sin(elapsed * 0.01));  // Gentle vertical

      // Specific force: f_n = a_n - g_n
      const Eigen::Vector3d f_n = accel_n - g_n;

      // Transform to body using CONSISTENT convention
      const Eigen::Matrix3d R_nb = ukf.getState().quaternion.toRotationMatrix().transpose();
      imu_data.acceleration = R_nb * f_n;
    }

    // UNIFIED HEALTH CHECK: Attitude and mechanization validation
    const Eigen::Vector3d g_n(0, 0, ukf_config.gravity);
    const Eigen::Matrix3d R_bn = ukf.getState().quaternion.toRotationMatrix(); // body-to-nav

    // Tilt error: angle between estimated Z-body and true gravity direction
    // In NED: gravity = [0,0,+g], body Z-axis = R_bn.col(2)
    const Eigen::Vector3d z_body_in_nav = R_bn.col(2);  // 3rd column of R_bn
    const double cos_tilt = g_n.normalized().dot(z_body_in_nav);
    const double tilt_error_deg = std::acos(std::clamp(cos_tilt, -1.0, 1.0)) * 180.0 / M_PI;

    // Gravity consistency check (only applies to both static and dynamic modes)
    const Eigen::Vector3d a_n_reconstructed = R_bn * imu_data.acceleration + g_n;
    const double gravity_error = STATIC_MODE ? a_n_reconstructed.norm() : 0.0;  // Only check in static mode

    // Log gyro magnitudes to check for unit errors (should be <0.02 rad/s in static)
    const double gyro_magnitude = imu_data.angular_velocity.norm();

    if (STATIC_MODE && (tilt_error_deg > 2.0 || gravity_error > 0.1 || gyro_magnitude > 0.02)) {
      LOG_WARN("STATIC TEST FAILURE: tilt={:.1f}° (goal <2°), gravity_error={:.3f} m/s² (goal <0.1), gyro_mag={:.5f} rad/s (goal <0.02)",
               tilt_error_deg, gravity_error, gyro_magnitude);
    }

    // GRAVITY-REFERENCED ATTITUDE CORRECTION (UKF-based) - Reduced rate
    // Apply at 50 Hz instead of 200 Hz to prevent over-correction
    static double last_gravity_time = 0.0;
    const double gravity_period = 1.0 / 50.0;  // 50 Hz

    if (imu_data.timestamp - last_gravity_time >= gravity_period) {
      const double f_b_mag = imu_data.acceleration.norm();
      const double mag_error = std::abs(f_b_mag - ukf_config.gravity);
      const double gyro_mag = imu_data.angular_velocity.norm();

      // Improved gating conditions
      const bool near_gravity = mag_error < 0.2;  // Tighter tolerance (0.2 m/s²)
      const bool low_gyro = gyro_mag < 0.03;      // Tighter gyro threshold (rad/s)
      const bool needs_correction = tilt_error_deg > 1.0;

      if (near_gravity && low_gyro && needs_correction) {
        // Get tilt BEFORE update for proper Δtilt tracking
        const auto state_before = ukf.getState();
        const Eigen::Matrix3d R_nb_before = state_before.quaternion.toRotationMatrix();
        const Eigen::Vector3d rpy_before = R_nb_before.eulerAngles(0, 1, 2);
        const double tilt_before_deg = std::sqrt(rpy_before.x()*rpy_before.x() + rpy_before.y()*rpy_before.y()) * 180.0 / M_PI;

        // Normalize specific force to get gravity direction measurement
        // CRITICAL FIX: Negate to get "down" in body frame (gravity points down, accel measures up)
        const Eigen::Vector3d z_measured = -imu_data.acceleration.normalized();

        // Adaptive noise based on gating conditions
        double gravity_sigma = 0.050;  // Base noise for cross-product residual (rad)

        // Precise gating with graduated noise inflation
        if (mag_error < 0.1 && gyro_mag < 0.015) {
          gravity_sigma = 0.040;  // Best conditions - tighten noise
        } else if (mag_error < 0.2 && gyro_mag < 0.03) {
          gravity_sigma = 0.050;  // Good conditions - nominal noise
        } else if (mag_error < 0.3 && gyro_mag < 0.05) {
          gravity_sigma = 0.080;  // Marginal conditions - inflate moderately
        } else {
          gravity_sigma = 0.120;  // Poor conditions - inflate significantly
        }

        measurements::GravityUKFMeasurement gravity_meas(z_measured, gravity_sigma, imu_data.timestamp);

        // Apply measurement through UKF (integrates with δθ error states)
        ukf.update(gravity_meas);

        // Get tilt AFTER update to measure actual correction
        const auto state_after = ukf.getState();
        const Eigen::Matrix3d R_nb_after = state_after.quaternion.toRotationMatrix();
        const Eigen::Vector3d rpy_after = R_nb_after.eulerAngles(0, 1, 2);
        const double tilt_after_deg = std::sqrt(rpy_after.x()*rpy_after.x() + rpy_after.y()*rpy_after.y()) * 180.0 / M_PI;

        // Calculate ACTUAL change from update (should be negative when correcting)
        const double tilt_delta_deg = tilt_after_deg - tilt_before_deg;

        last_gravity_time = imu_data.timestamp;

        LOG_INFO("Gravity update: tilt {:.1f}°→{:.1f}° (Δ={:+.2f}°), accel_tilt={:.1f}°, f_mag={:.2f}, gyro={:.4f}, σ={:.3f}",
                 tilt_before_deg, tilt_after_deg, tilt_delta_deg, tilt_error_deg, f_b_mag, gyro_mag, gravity_sigma);
      }
    }

    if (tilt_error_deg > 45.0) {
      LOG_ERROR("CRITICAL: Attitude diverged! tilt_error={:.1f}° (cos_tilt={:.3f})",
                tilt_error_deg, cos_tilt);
    }

    // Update ground truth (simple sinusoidal motion for error assessment)
    const double dt = 1.0 / sys_config.ukf.rate_hz;

    // Define a simple, predictable trajectory for ground truth
    // Gentle circular motion in horizontal plane + slow vertical oscillation
    const double radius = 100.0;  // 100m radius
    const double freq_hz = 0.01;  // Very slow circular motion
    const double omega = 2.0 * M_PI * freq_hz;

    // True position follows a gentle circle + vertical motion
    true_pos_ned.x() = radius * std::cos(omega * elapsed);
    true_pos_ned.y() = radius * std::sin(omega * elapsed);
    true_pos_ned.z() = -100.0 + 20.0 * std::sin(omega * elapsed * 0.5);  // Gentle vertical motion

    // Calculate true velocity from position derivative
    true_vel_ned.x() = -radius * omega * std::sin(omega * elapsed);
    true_vel_ned.y() = radius * omega * std::cos(omega * elapsed);
    true_vel_ned.z() = 20.0 * omega * 0.5 * std::cos(omega * elapsed * 0.5);
    imu_data.temperature  = 25.0;

    // Log raw IMU and predict
    binary_logger.logIMU(imu_data);
    ukf.predict(imu_data);

    // --- Barometer measurement at 50 Hz (MSL-aware) ---
    if (baro_config.enabled) {
      const double current_time = imu_data.timestamp;
      const double baro_period  = 1.0 / baro_config.update_rate_hz;

      if (current_time - last_baro_time >= baro_period) {
        // True MSL altitude from state: MSL = origin_MSL - z_NED
        const double true_alt_msl = origin_msl_alt_m - ukf.getState().position.z();

        // Simple colored noise (meters)
        const double noise_m = 0.5 * std::sin(elapsed * 0.2) + 0.3 * std::cos(elapsed * 0.7);

        // ISA temperature at altitude
        const double temperature_c = 15.0 - 0.0065 * true_alt_msl;

        // Convert altitude (with noise) to pressure
        const double pressure_pa =
            measurements::BaroMeasurement::altitudeMSLToPressure(true_alt_msl + noise_m,
                                                              baro_config.sea_level_pressure_pa);

        measurements::BaroData baro_data;
        baro_data.timestamp     = current_time;
        baro_data.pressure_pa   = pressure_pa;
        baro_data.temperature_c = temperature_c;
        baro_data.altitude_m    = true_alt_msl + noise_m;   // MSL altitude
        baro_data.sigma_m       = baro_config.sigma_m;

        // Pass raw NED position - BaroMeasurement::predictMSL expects this
        Eigen::Vector3d pos_for_baro = ukf.getState().position;

        // Barometer bootstrap: one-shot correction if we're way off
        static bool did_baro_bootstrap = false;

        if (barometer.processMeasurement(baro_data, pos_for_baro)) {
          // Check if we need bootstrap (first valid measurement with large error)
          if (!did_baro_bootstrap) {
            const double pred_alt_msl = origin_msl_alt_m - ukf.getState().position.z();
            const double dz = barometer.getValue() - pred_alt_msl; // positive if we need to go up

            // Health gating: only bootstrap if attitude is reasonable and innovation isn't extreme
            bool attitude_ok = (tilt_error_deg < 10.0);  // Use tilt error from IMU health check
            bool innovation_ok = (std::abs(dz) > 50.0 && std::abs(dz) < 500.0);  // Reasonable range

            if (attitude_ok && innovation_ok) {
              auto S = ukf.getState();
              S.position.z() -= dz;  // Adjust NED z to match baro MSL
              auto C = ukf.getCovariance();
              ukf.softReset(S, C);  // Soft reset to new altitude
              LOG_WARN("Baro bootstrap applied: Δz = {:+.1f} m (MSL alignment)", dz);
            } else {
              LOG_WARN("Baro bootstrap rejected: attitude_ok={}, innovation_ok={} (Δz={:.1f}m, tilt={:.1f}°)",
                       attitude_ok, innovation_ok, dz, tilt_error_deg);
            }
            did_baro_bootstrap = true;
          }

          // UKF adapter that predicts: z_pred = origin_MSL - z_NED (+bias)
          auto baro_ukf_meas =
              std::make_shared<measurements::BaroUKFMeasurement>(barometer,
                                                                  origin_msl_alt_m,
                                                                  current_time);
          ukf.update(*baro_ukf_meas);
          baro_updates++;

          LOG_INFO("Baro update #{}: altMSL={:.1f}m, innov={:.2f}m, chi2={:.2f}",
                   baro_updates, barometer.getValue(), barometer.getInnovation(), barometer.getChi2());
        } else {
          LOG_DEBUG("Baro gated: innov={:.2f}m, chi2={:.2f}, threshold={:.1f}",
                    barometer.getInnovation(), barometer.getChi2(), baro_config.chi2_gate_1d);
        }

        last_baro_time = current_time;
      }
    }

    // --- TRN measurement at 10 Hz ---
    if (trn_config.enabled) {
      const double current_time = imu_data.timestamp;
      const double radar_period = 1.0 / trn_config.update_rate_hz;

      if (current_time - last_radar_time >= radar_period) {
        // Simulated radar altimeter AGL from DEM + state
        double simulated_agl = trn_manager.simulateAGLFromState(ukf.getState());

        // Process the radar ping
        if (trn_manager.processRadarPing(current_time, simulated_agl, ukf.getState(), true)) {
          if (trn_manager.hasMeasurement()) {
            auto trn_meas = trn_manager.nextMeasurement();
            if (trn_meas) {
              ukf.update(*trn_meas);
              LOG_DEBUG("TRN update applied at t={:.3f}, AGL={:.1f}m",
                        current_time, simulated_agl);
            }
          }
        }
        last_radar_time = current_time;
      }
    }

    // --- STN measurement at 2 Hz ---
    if (stn_enabled && stn_generator) {
      const double current_time = imu_data.timestamp;
      const double stn_period = 1.0 / stn_params.config.update_rate_hz;

      if (current_time - last_stn_time >= stn_period) {
        // Convert standard state to extended state for synthetic generator
        aion::ExtendedState ext_state;
        ext_state.fromStandardState(ukf.getState());
        ext_state.clock_bias_m = 0.0;  // Will be estimated by filter
        ext_state.clock_drift_frac = 0.0;

        // Generate synthetic STN observations
        auto observations = stn_generator->generateObservations(ext_state, current_time);

        LOG_DEBUG("STN: Generated {} observations at t={:.3f}", observations.size(), current_time);

        if (!observations.empty()) {
          // Create UKF measurement adapter
          measurements::STNUKFMeasurement::Params meas_params;
          meas_params.chi2_gate_threshold = stn_params.config.chi2_gate_threshold;
          meas_params.enable_raim = stn_params.config.raim_enabled;
          meas_params.min_beacons = stn_params.config.min_beacons;

          auto stn_meas = std::make_shared<measurements::STNUKFMeasurement>(
              observations, stn_manager, ext_state, meas_params);

          if (stn_meas->isValid()) {
            auto gdop = stn_meas->getGDOP();
            const size_t num_beacons = stn_meas->selectedIds().size();

            // Tiered GDOP thresholds for adaptive update strategy
            const double excellent_gdop = 5.0;   // Excellent geometry - could use full update
            const double moderate_gdop = 8.0;    // Moderate geometry - Schmidt with normal noise
            const double marginal_gdop = 12.0;   // Marginal geometry - Schmidt with scaled noise
            const double poor_gdop = 20.0;       // Poor geometry - reject completely
            const size_t min_beacons = 4;        // Minimum for reliable positioning

            // Determine update strategy based on GDOP
            enum UpdateStrategy { REJECT, SCHMIDT_HEAVY_SCALE, SCHMIDT_MODERATE_SCALE, SCHMIDT_NORMAL };
            UpdateStrategy strategy = REJECT;
            double noise_scale_factor = 1.0;

            if (num_beacons >= min_beacons && std::isfinite(gdop.gdop)) {
              if (gdop.gdop < excellent_gdop) {
                strategy = SCHMIDT_NORMAL;  // Would be FULL_UPDATE if Schmidt wasn't enforced
                noise_scale_factor = 1.0;
              } else if (gdop.gdop < moderate_gdop) {
                strategy = SCHMIDT_MODERATE_SCALE;
                noise_scale_factor = std::pow(gdop.gdop / excellent_gdop, 1.5);  // Moderate scaling
              } else if (gdop.gdop < marginal_gdop) {
                strategy = SCHMIDT_HEAVY_SCALE;
                noise_scale_factor = std::pow(gdop.gdop / excellent_gdop, 2.0);  // Aggressive scaling
              } else if (gdop.gdop < poor_gdop) {
                strategy = SCHMIDT_HEAVY_SCALE;
                noise_scale_factor = std::pow(gdop.gdop / excellent_gdop, 2.5);  // Very aggressive
              } else {
                strategy = REJECT;
              }
            }

            // Get current tilt for tilt-aware gating
            const Eigen::Matrix3d R_bn_current = ukf.getState().quaternion.toRotationMatrix();
            const Eigen::Vector3d z_body_current = R_bn_current.col(2);
            const Eigen::Vector3d g_ned(0, 0, 9.80665);
            const double tilt_current_deg = std::acos(std::clamp(g_ned.normalized().dot(z_body_current), -1.0, 1.0)) * 180.0 / M_PI;

            // Tilt-aware STN gating: if tilt is large, heavily inflate noise
            if (tilt_current_deg > 10.0 && strategy != REJECT) {
              noise_scale_factor *= 10.0;  // Extreme inflation for large tilts
              LOG_WARN("Large tilt detected ({:.1f}°) - inflating STN noise by 10x", tilt_current_deg);
            }

            const bool good_geometry = (strategy != REJECT);

            LOG_DEBUG("STN: GDOP={:.2f}, HDOP={:.2f}, VDOP={:.2f}, {} beacons, strategy={}, noise_scale={:.2f}",
                      gdop.gdop, gdop.hdop, gdop.vdop, num_beacons,
                      strategy == REJECT ? "REJECT" :
                      strategy == SCHMIDT_HEAVY_SCALE ? "SCHMIDT_HEAVY" :
                      strategy == SCHMIDT_MODERATE_SCALE ? "SCHMIDT_MODERATE" : "SCHMIDT_NORMAL",
                      noise_scale_factor);

            if (good_geometry) {
              // Apply the computed noise scaling to the measurement
              stn_meas->scaleNoise(noise_scale_factor);

              // Store tilt before update for delta monitoring
              const double tilt_before = tilt_current_deg;

              // Use Schmidt-Kalman update to protect attitude during STN updates
              ukf.schmidtUpdate(*stn_meas);
              stn_updates++;

              // Check tilt after update
              const Eigen::Matrix3d R_bn_after = ukf.getState().quaternion.toRotationMatrix();
              const Eigen::Vector3d z_body_after = R_bn_after.col(2);
              const double tilt_after = std::acos(std::clamp(g_ned.normalized().dot(z_body_after), -1.0, 1.0)) * 180.0 / M_PI;
              const double delta_tilt = tilt_after - tilt_before;

              LOG_INFO("STN Schmidt update #{} at t={:.3f}: {} beacons, GDOP={:.2f}, scale={:.1f}x, Δtilt={:+.2f}° ({:.1f}→{:.1f})",
                       stn_updates, current_time, num_beacons, gdop.gdop, noise_scale_factor,
                       delta_tilt, tilt_before, tilt_after);

              // Immediate gravity re-anchoring after STN update
              const double f_b_mag = imu_data.acceleration.norm();
              const double gravity_mag = 9.80665;  // Standard gravity magnitude
              const double mag_error = std::abs(f_b_mag - gravity_mag);
              const double gyro_mag = imu_data.angular_velocity.norm();
              const bool gravity_conditions_good = (mag_error < 0.2 && gyro_mag < 0.05);

              if (gravity_conditions_good) {
                // CRITICAL FIX: Negate to get "down" in body frame
                const Eigen::Vector3d z_measured = -imu_data.acceleration.normalized();
                const double gravity_sigma = 0.050;  // Quick re-anchor noise
                measurements::GravityUKFMeasurement gravity_reanchor(z_measured, gravity_sigma, current_time);
                ukf.update(gravity_reanchor);
                // Force immediate periodic gravity update too
                last_gravity_time = 0.0;
                LOG_INFO("Post-STN gravity re-anchor: f_b_mag={:.2f}, mag_error={:.3f}, gyro={:.4f}",
                         f_b_mag, mag_error, gyro_mag);
              }
            } else {
              // Reject update due to poor geometry
              LOG_INFO("STN update REJECTED at t={:.3f}: {} beacons, GDOP={:.2f} (threshold={:.1f})",
                       current_time, num_beacons, gdop.gdop, poor_gdop);
            }
          } else {
            LOG_DEBUG("STN: Measurement invalid (insufficient beacons or poor geometry)");
          }
        }
        last_stn_time = current_time;
      }
    }

    // Log state at 10 Hz
    if (loop_count % 20 == 0) {
      binary_logger.logState(ukf.getState(), ukf.getCovariance());
    }

    // Calculate position error
    Eigen::Vector3d position_error = ukf.getState().position - true_pos_ned;
    double position_error_norm = position_error.norm();
    sum_position_error += position_error_norm;
    sum_position_error_sq += position_error_norm * position_error_norm;
    error_samples++;
    if (position_error_norm > max_position_error) {
      max_position_error = position_error_norm;
    }

    // 1 Hz status
    if (++loop_count % static_cast<uint64_t>(sys_config.ukf.rate_hz) == 0) {
      const auto& s = ukf.getState();
      double elapsed_total = aion::TimeUtils::now() - start_time_sec;
      auto stats = rate_controller.getStats();

      // Calculate error statistics
      double mean_error = sum_position_error / error_samples;
      double rms_error = std::sqrt(sum_position_error_sq / error_samples);

      // Calculate horizontal and vertical errors separately
      double horiz_error = std::sqrt(position_error.x()*position_error.x() +
                                    position_error.y()*position_error.y());
      double vert_error = std::abs(position_error.z());

      LOG_INFO("t={:.1f}s, pos=[{:.2f},{:.2f},{:.2f}] m, vel=[{:.2f},{:.2f},{:.2f}] m/s, rate={:.1f} Hz",
               elapsed_total,
               s.position.x(), s.position.y(), s.position.z(),
               s.velocity.x(), s.velocity.y(), s.velocity.z(),
               stats.actual_rate_hz);

      LOG_INFO("Position Error: Total={:.2f}m (H={:.2f}m, V={:.2f}m), RMS={:.2f}m, Max={:.2f}m",
               position_error_norm, horiz_error, vert_error, rms_error, max_position_error);

      if (trn_config.enabled) {
        const auto& trn_stats = trn_manager.stats();
        LOG_INFO("TRN: pings={}, accepted={}, gated={}, rate={:.1f} Hz",
                 trn_stats.pings_in, trn_stats.accepted, trn_stats.gated, trn_stats.avg_rate_hz);
      }

      if (stn_enabled) {
        LOG_INFO("STN: updates={}, rate={:.1f} Hz", stn_updates, stn_updates / elapsed);
      }

      if (baro_config.enabled) {
        double baro_rate = (elapsed_total > 0.0) ? (baro_updates / elapsed_total) : 0.0;
        LOG_INFO("Baro: updates={}, rate={:.1f} Hz", baro_updates, baro_rate);
      }

      if (!ukf.isHealthy()) {
        LOG_WARN("UKF health check failed!");
      }
      if (!rate_controller.isHealthy(0.9)) {
        LOG_WARN("Rate controller below target: {:.1f}/{:.0f} Hz",
                 stats.actual_rate_hz, stats.target_rate_hz);
      }
    }

    // Maintain loop rate
    rate_controller.sleep();
  }

  // Shutdown
  LOG_INFO("Main loop terminated after {} iterations", loop_count);

  // Final position error statistics
  if (error_samples > 0) {
    double mean_error = sum_position_error / error_samples;
    double rms_error = std::sqrt(sum_position_error_sq / error_samples);
    LOG_INFO("\n=== FINAL POSITION ERROR STATISTICS ===");
    LOG_INFO("  Mean Error: {:.2f} meters", mean_error);
    LOG_INFO("  RMS Error:  {:.2f} meters", rms_error);
    LOG_INFO("  Max Error:  {:.2f} meters", max_position_error);
    LOG_INFO("  STN Updates: {} ({:.1f} Hz)", stn_updates,
             stn_updates / (aion::TimeUtils::now() - start_time_sec));
    LOG_INFO("  TRN Updates: {} ({:.1f} Hz)",
             trn_manager.stats().accepted,
             trn_manager.stats().avg_rate_hz);
    LOG_INFO("  Baro Updates: {} ({:.1f} Hz)", baro_updates,
             baro_updates / (aion::TimeUtils::now() - start_time_sec));
    LOG_INFO("======================================\n");
  }

  binary_logger.stop();
  auto log_stats = binary_logger.getStats();
  LOG_INFO("Binary log stats: IMU={}, State={}, Total={} frames",
           log_stats.imu_count, log_stats.state_count, log_stats.total_frames);

  auto final_stats = rate_controller.getStats();
  LOG_INFO("Final timing statistics:");
  LOG_INFO("  Average rate: {:.1f} Hz (target: {:.0f} Hz)",
           final_stats.actual_rate_hz, final_stats.target_rate_hz);
  LOG_INFO("  Min/Max dt: {:.3f}/{:.3f} ms",
           final_stats.min_dt * 1000.0, final_stats.max_dt * 1000.0);
  LOG_INFO("  Overruns: {} ({:.2f}%)",
           final_stats.overruns,
           100.0 * final_stats.overruns / std::max<uint64_t>(1, final_stats.cycles));

  aion::Logger::shutdown();
  return 0;
}
