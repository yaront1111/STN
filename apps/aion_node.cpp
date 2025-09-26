// apps/aion_node.cpp

#include <atomic>
#include <csignal>
#include <chrono>
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
  ukf_config.nis_gate_1d                = 15.0;   // Permissive NIS gate for 1D measurements (TRN)

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
  initial_state.time = aion::TimeUtils::now();
  initial_state.position = Eigen::Vector3d(0, 0, -100);  // 100m altitude (NED: down is positive)
  initial_state.velocity = Eigen::Vector3d(10, 5, 0);    // Moving at 10m/s north, 5m/s east

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

  // ---- TRN init ----
  measurements::TRNManagerConfig trn_config;
  trn_config.enabled          = true;
  trn_config.update_rate_hz   = 10.0;                    // radar at 10 Hz
  trn_config.chi2_gate_1d     = 5.0;                     // Slightly less restrictive
  trn_config.min_slope_deg    = 0.2;                     // Terrain slope threshold for informative terrain
  trn_config.min_agl_m        = 2.0;                     // m
  trn_config.max_agl_m        = 1500.0;                  // m
  trn_config.dem_sigma_base_m = 5.0;                     // DEM uncertainty base
  trn_config.dem_sigma_slope_k = 0.1;                    // DEM uncertainty slope factor
  trn_config.radar_sigma_m    = 1.0;                     // radar σ [m]
  trn_config.origin_lat_deg   = 31.7683;                 // Jerusalem
  trn_config.origin_lon_deg   = 35.2137;                 // Jerusalem
  trn_config.origin_alt_m     = 750.0;                   // Jerusalem elevation ~750m MSL
  trn_config.dem_path         = "data/dem/srtm";         // path to SRTM tiles

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

    // Sanity probe: check Jerusalem elevation immediately after init
    const double jlm_lat = 31.7683, jlm_lon = 35.2137;
    const double h = trn_manager.terrain()->getElevationBilinear(jlm_lat, jlm_lon);
    LOG_INFO("TRN: Sanity probe: Jerusalem {:.6f},{:.6f} ⇒ DEM {:.1f} m", jlm_lat, jlm_lon, h);
  }

  // Main loop
  aion::RateController rate_controller(sys_config.ukf.rate_hz);
  uint64_t loop_count = 0;
  double start_time_sec = aion::TimeUtils::now();
  double last_radar_time = 0.0;

  LOG_INFO("Starting main loop at {:.0f} Hz", sys_config.ukf.rate_hz);

  while (!g_shutdown) {
    // --- Simulated IMU with flight dynamics ---
    aion::IMUData imu_data;
    imu_data.timestamp        = aion::TimeUtils::now();

    // Simulate gentle banking and altitude changes for interesting TRN
    double elapsed = imu_data.timestamp - start_time_sec;
    double turn_rate = 0.05 * std::sin(elapsed * 0.1);  // Gentle turns
    double pitch_rate = 0.02 * std::sin(elapsed * 0.15); // Gentle pitch changes

    imu_data.angular_velocity = Eigen::Vector3d(pitch_rate, 0, turn_rate);  // rad/s

    // Compute specific force including coordinated turn
    const Eigen::Vector3d g_n(0, 0, ukf_config.gravity);   // NED: +g down
    const Eigen::Matrix3d R_nb = ukf.getState().quaternion.conjugate().toRotationMatrix(); // nav->body

    // Add small accelerations for maneuvering
    Eigen::Vector3d accel_n(0.5 * std::sin(elapsed * 0.05),  // North acceleration
                            0.3 * std::cos(elapsed * 0.07),   // East acceleration
                            0.2 * std::sin(elapsed * 0.03));  // Vertical acceleration

    imu_data.acceleration = -R_nb * (g_n - accel_n);  // specific force in body
    imu_data.temperature  = 25.0;

    // Log raw IMU
    binary_logger.logIMU(imu_data);

    // Predict
    ukf.predict(imu_data);

    // --- TRN measurement at 10 Hz ---
    if (trn_config.enabled) {
      const double current_time = imu_data.timestamp;
      const double radar_period = 1.0 / trn_config.update_rate_hz;

      if (current_time - last_radar_time >= radar_period) {
        // Simulate radar altimeter measurement
        double simulated_agl = trn_manager.simulateAGLFromState(ukf.getState());

        // Process the radar ping
        if (trn_manager.processRadarPing(current_time, simulated_agl, ukf.getState(), true)) {
          // Check if measurement is ready
          if (trn_manager.hasMeasurement()) {
            auto trn_meas = trn_manager.nextMeasurement();
            if (trn_meas) {
              ukf.update(*trn_meas);
              LOG_DEBUG("TRN update applied at t={:.3f}, AGL={:.1f}m", current_time, simulated_agl);
            }
          }
        }
        last_radar_time = current_time;
      }
    }

    // Log state at 10 Hz
    if (loop_count % 20 == 0) {
      binary_logger.logState(ukf.getState(), ukf.getCovariance());
    }

    // 1 Hz status
    if (++loop_count % static_cast<uint64_t>(sys_config.ukf.rate_hz) == 0) {
      const auto& s = ukf.getState();
      double elapsed = aion::TimeUtils::now() - start_time_sec;
      auto stats = rate_controller.getStats();

      LOG_INFO("t={:.1f}s, pos=[{:.2f},{:.2f},{:.2f}] m, vel=[{:.2f},{:.2f},{:.2f}] m/s, rate={:.1f} Hz",
               elapsed,
               s.position.x(), s.position.y(), s.position.z(),
               s.velocity.x(), s.velocity.y(), s.velocity.z(),
               stats.actual_rate_hz);

      // TRN statistics (always show to debug gating)
      if (trn_config.enabled) {
        const auto& trn_stats = trn_manager.stats();
        LOG_INFO("TRN: pings={}, accepted={}, gated={}, rate={:.1f} Hz",
                 trn_stats.pings_in, trn_stats.accepted, trn_stats.gated, trn_stats.avg_rate_hz);
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
