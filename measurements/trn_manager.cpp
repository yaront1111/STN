#include "trn_manager.h"
#include "core/utils/logging.h"
#include <chrono>

namespace measurements {

TRNManager::TRNManager(const TRNManagerConfig& cfg)
: cfg_(cfg),
  terrain_service_(std::make_shared<terrain::TerrainService>(cfg.dem_path)),
  trn_cfg_{ // map to TRNConfig used by TRNProcessor
    cfg.min_slope_deg,
    cfg.chi2_gate_1d,
    cfg.min_agl_m,
    cfg.max_agl_m,
    cfg.radar_sigma_m,
    cfg.dem_sigma_base_m,
    cfg.dem_sigma_slope_k,
    cfg.origin_lat_deg,
    cfg.origin_lon_deg,
    cfg.origin_alt_m
  },
  trn_proc_(terrain_service_, trn_cfg_),
  rng_(static_cast<uint32_t>(std::chrono::steady_clock::now().time_since_epoch().count())),
  radar_noise_(0.0, cfg.radar_sigma_m) {}

bool TRNManager::initialize() {
  if (!cfg_.enabled) {
    LOG_INFO("TRN: disabled");
    return true;
  }
  if (!terrain_service_->initialize()) {
    LOG_ERROR("TRN: failed to initialize terrain service at '{}'", cfg_.dem_path);
    return false;
  }

  // Optional preload around origin
  const double d = 0.5; // deg
  terrain_service_->preloadRegion(
    cfg_.origin_lat_deg - d, cfg_.origin_lat_deg + d,
    cfg_.origin_lon_deg - d, cfg_.origin_lon_deg + d
  );

  LOG_INFO("TRN: initialized — tiles loaded: {}", terrain_service_->getLoadedTileCount());
  return true;
}

void TRNManager::nedToGeodetic(const Eigen::Vector3d& ned, double& lat_deg, double& lon_deg) const {
  lat_deg = cfg_.origin_lat_deg + ned.x() / mPerDegLat();
  lon_deg = cfg_.origin_lon_deg + ned.y() / mPerDegLon(cfg_.origin_lat_deg);
}

std::unique_ptr<aion::MeasurementBase>
TRNManager::buildMeasurement(double t, double measured_agl_m, const State& state) {
  // Construct via TRNProcessor (handles slope & chi2 gating with DEM noise)
  return trn_proc_.makeMeasurement(
    RadarAltimeterData{
      t,
      measured_agl_m,
      cfg_.radar_sigma_m,
      /*valid*/ true
    },
    state
  );
}

bool TRNManager::processRadarPing(double timestamp_sec, double measured_agl_m, bool valid) {
  if (!cfg_.enabled || !valid) return false;

  // rate limit
  if (last_ping_time_ > 0.0) {
    const double dt = timestamp_sec - last_ping_time_;
    if (dt < 1.0 / std::max(1e-6, cfg_.update_rate_hz)) return false;
  }

  ++stats_.pings_in;

  // You probably want the CURRENT nav state here; caller should pass it in, but we can
  // fetch it from the UKF in your node loop. For this manager, assume the caller will
  // immediately call nextMeasurement() after pushing it. So we require caller to hand us the state.
  // (If you prefer, add a State parameter to processRadarPing and forward it into buildMeasurement.)
  // For now, we cannot build without State, so return false.  — BUT to keep it useful, overload:
  return false;
}

// Overload with state (recommended)
bool TRNManager::processRadarPing(double timestamp_sec, double measured_agl_m,
                                  const State& state, bool valid) {
  if (!cfg_.enabled || !valid) return false;
  if (last_ping_time_ > 0.0) {
    const double dt = timestamp_sec - last_ping_time_;
    if (dt < 1.0 / std::max(1e-6, cfg_.update_rate_hz)) return false;
  }

  auto m = buildMeasurement(timestamp_sec, measured_agl_m, state);
  if (!m) { ++stats_.gated; return false; }

  {
    std::lock_guard<std::mutex> lk(q_mtx_);
    q_.push(std::move(m));
  }

  ++stats_.accepted;
  if (stats_.last_accept_time > 0.0) {
    const double dt = timestamp_sec - stats_.last_accept_time;
    const double inst = (dt > 0.0) ? 1.0/dt : 0.0;
    stats_.avg_rate_hz = (stats_.avg_rate_hz == 0.0) ? inst : 0.9*stats_.avg_rate_hz + 0.1*inst;
  }
  stats_.last_accept_time = timestamp_sec;
  last_ping_time_ = timestamp_sec;
  return true;
}

bool TRNManager::hasMeasurement() const {
  std::lock_guard<std::mutex> lk(q_mtx_);
  return !q_.empty();
}

std::unique_ptr<aion::MeasurementBase> TRNManager::nextMeasurement() {
  std::lock_guard<std::mutex> lk(q_mtx_);
  if (q_.empty()) return nullptr;
  auto m = std::move(q_.front());
  q_.pop();
  return m;
}

double TRNManager::simulateAGLFromState(const State& s) {
  // Geodetic from NED
  double lat, lon;
  nedToGeodetic(s.position, lat, lon);

  // DEM (bilinear; fallback handled inside TerrainService)
  double elev = terrain_service_->getElevationBilinear(lat, lon);
  if (elev == -32768.0) elev = terrain_service_->getElevation(lat, lon);
  if (elev == -32768.0) {
    elev = 0.0; // sea level fallback
    spdlog::debug("TRN: Using sea level fallback for elevation at ({:.5f},{:.5f})", lat, lon);
  }

  // MSL altitude: origin_alt - z_down (NED)
  const double msl = cfg_.origin_alt_m - s.position.z();
  const double true_agl = msl - elev;
  const double noise = radar_noise_(rng_);
  const double simulated = true_agl + noise;

  spdlog::debug("TRN simulateAGL: pos=({:.1f},{:.1f},{:.1f}) lat={:.5f} lon={:.5f} elev={:.1f}m MSL={:.1f}m AGL={:.1f}m noise={:.2f} => {:.1f}m",
               s.position.x(), s.position.y(), s.position.z(), lat, lon, elev, msl, true_agl, noise, simulated);

  return simulated;
}

void TRNManager::resetStats() {
  stats_ = {};
  last_ping_time_ = 0.0;
  {
    std::lock_guard<std::mutex> lk(q_mtx_);
    while (!q_.empty()) q_.pop();
  }
}

} // namespace measurements
