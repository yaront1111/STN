#include "core/fusion/sr_ukf.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <spdlog/spdlog.h>
#include "core/utils/logging.h"

namespace {

// Apply ErrorState (tangent) to nominal State (boxplus)
inline void boxplus(const aion::ErrorState& dx, aion::State& x) {
  x.position += dx.delta_position;
  x.velocity += dx.delta_velocity;

  const double n = dx.delta_theta.norm();
  if (n > 0.0) {
    Eigen::Quaterniond dq(Eigen::AngleAxisd(n, dx.delta_theta / n));
    x.quaternion = dq * x.quaternion;   // left-multiply; q is NED->body
    x.quaternion.normalize();
  }

  x.gyro_bias  += dx.delta_gyro_bias;
  x.accel_bias += dx.delta_accel_bias;
  x.wind       += dx.delta_wind;        // wind is 2D (NE), z omitted by design
}

// Compute ErrorState mapping x_mean -> x_sigma (boxminus)
inline aion::ErrorState boxminus(const aion::State& x_sigma, const aion::State& x_mean) {
  aion::ErrorState dx;
  dx.delta_position   = x_sigma.position - x_mean.position;
  dx.delta_velocity   = x_sigma.velocity - x_mean.velocity;

  Eigen::Quaterniond dq = x_sigma.quaternion * x_mean.quaternion.conjugate();
  Eigen::AngleAxisd aa(dq);
  if (aa.angle() < 1e-12) {
    dx.delta_theta = Eigen::Vector3d::Zero();
  } else {
    dx.delta_theta = aa.axis() * aa.angle();
  }

  dx.delta_gyro_bias  = x_sigma.gyro_bias  - x_mean.gyro_bias;
  dx.delta_accel_bias = x_sigma.accel_bias - x_mean.accel_bias;
  dx.delta_wind       = x_sigma.wind       - x_mean.wind;
  return dx;
}

} // namespace

namespace aion {

SRUKF::SRUKF(const Config& config)
    : process_model_(std::make_unique<ProcessModel>(config.gravity)),
      config_(config) {
  computeWeights();
  sigma_points_.reserve(NUM_SIGMA_POINTS);
  propagated_sigma_points_.reserve(NUM_SIGMA_POINTS);
}

SRUKF::~SRUKF() = default;

void SRUKF::initialize(const State& initial_state, const StateCovariance& initial_cov) {
  state_ = initial_state;
  state_.quaternion.normalize();
  covariance_ = initial_cov;
  covariance_.enforceSymmetry();

  // Initialize last_imu_ timestamp to avoid large first dt
  last_imu_.timestamp        = state_.time;
  last_imu_.angular_velocity = Eigen::Vector3d::Zero();
  last_imu_.acceleration     = Eigen::Vector3d::Zero();

  // Ensure PD covariance (jitter if needed)
  Eigen::LLT<Eigen::MatrixXd> llt(covariance_.matrix());
  if (llt.info() != Eigen::Success) {
    spdlog::warn("Initial covariance not PD; enforcing symmetry with jitter.");
    covariance_.enforceSymmetry();
  }

  spdlog::info("UKF initialized at NED p=[{:.3f} {:.3f} {:.3f}]",
               state_.position.x(), state_.position.y(), state_.position.z());
}

void SRUKF::computeWeights() {
  const double lambda = config_.alpha * config_.alpha * (L + config_.kappa) - L;

  weights_mean_.resize(NUM_SIGMA_POINTS);
  weights_cov_.resize(NUM_SIGMA_POINTS);

  weights_mean_(0) = lambda / (L + lambda);
  weights_cov_(0)  = lambda / (L + lambda) + (1.0 - config_.alpha * config_.alpha + config_.beta);

  const double w = 1.0 / (2.0 * (L + lambda));
  for (int i = 1; i < NUM_SIGMA_POINTS; ++i) {
    const auto idx = static_cast<Eigen::Index>(i);
    weights_mean_(idx) = w;
    weights_cov_(idx)  = w;
  }
}

void SRUKF::generateSigmaPoints() {
  sigma_points_.clear();
  sigma_points_.reserve(NUM_SIGMA_POINTS);

  ErrorState e0; e0.reset();
  sigma_points_.push_back(e0);

  // Cholesky factor of P
  Eigen::LLT<Eigen::MatrixXd> llt(covariance_.matrix());
  if (llt.info() != Eigen::Success) {
    covariance_.enforceSymmetry();
    llt.compute(covariance_.matrix());
  }
  const Eigen::MatrixXd S = llt.matrixL();

  const double lambda = config_.alpha * config_.alpha * (L + config_.kappa) - L;
  const double gamma  = std::sqrt(L + lambda);

  for (int i = 0; i < L; ++i) {
    ErrorState sp, sm;
    const auto idx = static_cast<Eigen::Index>(i);
    const Eigen::VectorXd col = gamma * S.col(idx);
    sp.fromVector(col);
    sm.fromVector(-col);
    sigma_points_.push_back(sp);
    sigma_points_.push_back(sm);
  }
}

void SRUKF::propagateSigmaPoints(double dt) {
  propagated_sigma_points_.clear();
  propagated_sigma_points_.reserve(NUM_SIGMA_POINTS);

  for (const auto& es : sigma_points_) {
    State s = state_;     // start from nominal
    boxplus(es, s);       // map error -> state space
    process_model_->propagate(s, last_imu_, dt);
    propagated_sigma_points_.push_back(s);
  }
}

void SRUKF::computePrediction() {
  // 1) Mean on manifold via small Gauss-Newton (2 iters are enough here)
  State x_mean = propagated_sigma_points_[0];
  for (int iter = 0; iter < 2; ++iter) {
    ErrorState accum; accum.reset();
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
      const auto idx = static_cast<std::size_t>(i);
      const auto widx = static_cast<Eigen::Index>(i);
      const ErrorState dx = boxminus(propagated_sigma_points_[idx], x_mean);
      accum.delta_position   += weights_mean_(widx) * dx.delta_position;
      accum.delta_velocity   += weights_mean_(widx) * dx.delta_velocity;
      accum.delta_theta      += weights_mean_(widx) * dx.delta_theta;
      accum.delta_gyro_bias  += weights_mean_(widx) * dx.delta_gyro_bias;
      accum.delta_accel_bias += weights_mean_(widx) * dx.delta_accel_bias;
      accum.delta_wind       += weights_mean_(widx) * dx.delta_wind;
    }
    boxplus(accum, x_mean);
  }

  // 2) Covariance from weighted deviations in tangent space
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(L, L);
  for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const auto widx = static_cast<Eigen::Index>(i);
    const ErrorState dx = boxminus(propagated_sigma_points_[idx], x_mean);
    const Eigen::VectorXd v = dx.toVector();
    P += weights_cov_(widx) * (v * v.transpose());
  }

  covariance_.matrix() = P;
  covariance_.enforceSymmetry();

  // 3) Commit mean as new nominal
  state_ = x_mean;
}

void SRUKF::addProcessNoise(double dt) {
  // Simple diagonal discrete-time process noise (keeps filter from overconfidence).
  // For high fidelity, replace with mechanization-consistent Qd.
  Eigen::MatrixXd& P = covariance_.matrix();

  const double g_rw  = config_.gyro_noise_density;
  const double a_rw  = config_.accel_noise_density;
  const double bg_rw = config_.gyro_bias_noise_density;
  const double ba_rw = config_.accel_bias_noise_density;
  const double w_rw  = config_.wind_noise_density;

  const double q_p  = 0.25 * a_rw * a_rw * dt * dt; // tiny position diffusion
  const double q_v  = a_rw * a_rw * dt;             // velocity RW
  const double q_th = g_rw * g_rw * dt;             // attitude RW
  const double q_bg = bg_rw * bg_rw * dt;           // gyro bias RW
  const double q_ba = ba_rw * ba_rw * dt;           // accel bias RW
  const double q_w  = w_rw * w_rw * dt;             // wind RW

  // pos (0:2), vel (3:5), att (6:8), bg (9:11), ba (12:14), wind (15:16)
  P.block<3,3>(0,0).diagonal().array()   += q_p;
  P.block<3,3>(3,3).diagonal().array()   += q_v;
  P.block<3,3>(6,6).diagonal().array()   += q_th;
  P.block<3,3>(9,9).diagonal().array()   += q_bg;
  P.block<3,3>(12,12).diagonal().array() += q_ba;
  P.block<2,2>(15,15).diagonal().array() += Eigen::Array2d::Constant(q_w);
}

void SRUKF::predict(const IMUData& imu_data) {
  const double dt = imu_data.timestamp - state_.time;
  if (!(dt > 0.0 && dt < 1.0)) {
    spdlog::warn("UKF predict skipped: invalid dt={}", dt);
    return;
  }

  last_imu_ = imu_data;

  generateSigmaPoints();
  propagateSigmaPoints(dt);
  computePrediction();
  addProcessNoise(dt);

  state_.time = imu_data.timestamp;
}

void SRUKF::update(const MeasurementBase& z) {
  const int m = z.getDimension();
  if (m <= 0) return;

  // Predict measurement for each sigma
  Eigen::MatrixXd Y(m, NUM_SIGMA_POINTS);
  for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const auto cidx = static_cast<Eigen::Index>(i);
    Y.col(cidx) = z.predict(propagated_sigma_points_[idx]);
  }

  // Mean and innovation
  const Eigen::VectorXd y_mean = Y * weights_mean_;
  const Eigen::VectorXd innov  = z.getValue() - y_mean;

  // Pyy and Pxy (state side via boxminus wrt current nominal state_)
  Eigen::MatrixXd Pyy = z.getCovariance();
  Eigen::MatrixXd Pxy = Eigen::MatrixXd::Zero(L, m);

  for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const auto widx = static_cast<Eigen::Index>(i);
    const Eigen::VectorXd dy = Y.col(widx) - y_mean;
    Pyy += weights_cov_(widx) * (dy * dy.transpose());

    const ErrorState dx = boxminus(propagated_sigma_points_[idx], state_);
    Pxy += weights_cov_(widx) * (dx.toVector() * dy.transpose());
  }

  // Gain via LDLT (more stable than explicit inverse)
  Eigen::LDLT<Eigen::MatrixXd> ldlt(Pyy);
  if (ldlt.info() != Eigen::Success) {
    spdlog::warn("UKF update skipped: Pyy not PD.");
    return;
  }
  const Eigen::MatrixXd K = Pxy * ldlt.solve(Eigen::MatrixXd::Identity(m, m));

  // Compute NIS for gating
  nis_ = innov.transpose() * ldlt.solve(innov);

  // Apply NIS gate if configured (gate on full Pyy, not just R)
  if (config_.nis_gate_1d < std::numeric_limits<double>::infinity() &&
      m == 1 && nis_ > config_.nis_gate_1d) {
    spdlog::info("UKF: update rejected (NIS={:.3f} > gate={:.3f}, innov={:.3f})",
                 nis_, config_.nis_gate_1d, innov(0));
    innovation_history_.push_back(nis_);
    if (innovation_history_.size() > 100) innovation_history_.erase(innovation_history_.begin());
    return; // Reject update but keep NIS for diagnostics
  }

  // Correction
  ErrorState dxe; dxe.fromVector(K * innov);
  applyCorrection(dxe);

  // Covariance update: P = P - K Pyy K^T
  Eigen::MatrixXd P_new = covariance_.matrix() - K * (Pyy * K.transpose());
  covariance_.matrix() = P_new;
  covariance_.enforceSymmetry();

  // Store NIS for health monitoring
  innovation_history_.push_back(nis_);
  if (innovation_history_.size() > 100) innovation_history_.erase(innovation_history_.begin());
}

void SRUKF::schmidtUpdate(const MeasurementBase& z) {
  const int m = z.getDimension();
  if (m <= 0) return;

  // Predict measurement for each sigma point
  Eigen::MatrixXd Y(m, NUM_SIGMA_POINTS);
  for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const auto cidx = static_cast<Eigen::Index>(i);
    Y.col(cidx) = z.predict(propagated_sigma_points_[idx]);
  }

  // Mean and innovation
  const Eigen::VectorXd y_mean = Y * weights_mean_;
  const Eigen::VectorXd innov  = z.getValue() - y_mean;

  // Pyy and Pxy (state side via boxminus wrt current nominal state_)
  Eigen::MatrixXd Pyy = z.getCovariance();
  Eigen::MatrixXd Pxy = Eigen::MatrixXd::Zero(L, m);

  for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
    const auto idx = static_cast<std::size_t>(i);
    const auto widx = static_cast<Eigen::Index>(i);
    const Eigen::VectorXd dy = Y.col(widx) - y_mean;
    Pyy += weights_cov_(widx) * (dy * dy.transpose());

    const ErrorState dx = boxminus(propagated_sigma_points_[idx], state_);
    Pxy += weights_cov_(widx) * (dx.toVector() * dy.transpose());
  }

  // Gain via LDLT (more stable than explicit inverse)
  Eigen::LDLT<Eigen::MatrixXd> ldlt(Pyy);
  if (ldlt.info() != Eigen::Success) {
    spdlog::warn("UKF Schmidt update skipped: Pyy not PD.");
    return;
  }
  Eigen::MatrixXd K = Pxy * ldlt.solve(Eigen::MatrixXd::Identity(m, m));

  // Monitor cross-covariance coupling before protection
  const Eigen::Matrix3d P_att_pos = covariance_.matrix().block<3, 3>(6, 0);  // Attitude-position coupling
  const Eigen::Matrix3d P_att_vel = covariance_.matrix().block<3, 3>(6, 3);  // Attitude-velocity coupling
  const double coupling_strength = std::sqrt(P_att_pos.squaredNorm() + P_att_vel.squaredNorm());

  // Schmidt-Kalman protection: Zero attitude gain rows (indices 6-8)
  const Eigen::MatrixXd K_attitude_original = K.block(6, 0, 3, m); // Save for logging
  K.block(6, 0, 3, m).setZero();

  // Log attitude protection and cross-covariance monitoring (every 50th call)
  static int schmidt_call_count = 0;
  static double max_coupling = 0.0;
  static double avg_coupling = 0.0;

  max_coupling = std::max(max_coupling, coupling_strength);
  avg_coupling = (avg_coupling * schmidt_call_count + coupling_strength) / (schmidt_call_count + 1);

  if (++schmidt_call_count % 50 == 0) {
    const double attitude_gain_norm = K_attitude_original.norm();
    spdlog::info("Schmidt-Kalman: att_gain_zeroed={:.4f}, P_xy_coupling={:.4f} (max={:.4f}, avg={:.4f})",
                 attitude_gain_norm, coupling_strength, max_coupling, avg_coupling);

    // Warn if coupling is getting strong
    if (coupling_strength > 0.1) {
      spdlog::warn("High attitude-position coupling detected: {:.4f} - consider decorrelation", coupling_strength);
    }
  }

  // Compute NIS for gating
  nis_ = innov.transpose() * ldlt.solve(innov);

  // Apply NIS gate if configured
  if (config_.nis_gate_1d < std::numeric_limits<double>::infinity() &&
      m == 1 && nis_ > config_.nis_gate_1d) {
    spdlog::info("UKF Schmidt: update rejected (NIS={:.3f} > gate={:.3f}, innov={:.3f})",
                 nis_, config_.nis_gate_1d, innov(0));
    innovation_history_.push_back(nis_);
    if (innovation_history_.size() > 100) innovation_history_.erase(innovation_history_.begin());
    return;
  }

  // Correction with protected attitude
  ErrorState dxe; dxe.fromVector(K * innov);
  applyCorrection(dxe);

  // Simplified covariance update: P = P - K Pyy K^T (same as standard update)
  Eigen::MatrixXd P_new = covariance_.matrix() - K * (Pyy * K.transpose());
  covariance_.matrix() = P_new;
  covariance_.enforceSymmetry();

  // Store NIS for health monitoring
  innovation_history_.push_back(nis_);
  if (innovation_history_.size() > 100) innovation_history_.erase(innovation_history_.begin());
}

void SRUKF::softReset(const State& target, const StateCovariance& cov_target) {
  // Error between target and current nominal (tangent)
  ErrorState e;
  e.delta_position   = target.position - state_.position;
  e.delta_velocity   = target.velocity - state_.velocity;

  Eigen::Quaterniond dq = target.quaternion * state_.quaternion.conjugate();
  Eigen::AngleAxisd aa(dq);
  if (aa.angle() < 1e-12) {
    e.delta_theta = Eigen::Vector3d::Zero();
  } else {
    e.delta_theta = aa.axis() * aa.angle();
  }

  e.delta_gyro_bias  = target.gyro_bias  - state_.gyro_bias;
  e.delta_accel_bias = target.accel_bias - state_.accel_bias;
  e.delta_wind       = target.wind       - state_.wind;

  // Blend correction & covariance
  const double alpha = 0.5;
  ErrorState pe = e;
  pe.delta_position   *= alpha;
  pe.delta_velocity   *= alpha;
  pe.delta_theta      *= alpha;
  pe.delta_gyro_bias  *= alpha;
  pe.delta_accel_bias *= alpha;
  pe.delta_wind       *= alpha;

  applyCorrection(pe);

  covariance_.matrix() =
      (1.0 - alpha) * covariance_.matrix() + alpha * cov_target.matrix();
  covariance_.enforceSymmetry();

  spdlog::debug("Soft reset applied (alpha={})", alpha);
}

void SRUKF::applyCorrection(const ErrorState& error) {
  state_.position += error.delta_position;
  state_.velocity += error.delta_velocity;

  const double n = error.delta_theta.norm();
  if (n > 0.0) {
    Eigen::Quaterniond dq(Eigen::AngleAxisd(n, error.delta_theta / n));
    state_.quaternion = dq * state_.quaternion;
    state_.quaternion.normalize();
  }

  state_.gyro_bias  += error.delta_gyro_bias;
  state_.accel_bias += error.delta_accel_bias;
  state_.wind       += error.delta_wind;
}

bool SRUKF::isHealthy() const {
  if (!state_.position.allFinite() || !state_.velocity.allFinite()) return false;

  if (!innovation_history_.empty()) {
    double avg = 0.0;
    for (double v : innovation_history_) avg += v;
    avg /= innovation_history_.size();
    if (avg > 20.0) return false; // coarse bound until measurement-specific checks exist
  }
  return true;
}

}  // namespace aion
