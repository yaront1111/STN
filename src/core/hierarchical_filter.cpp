/**
 * Hierarchical Filter Orchestrator Implementation
 * Coordinates SR-UKF, RBPF, and reset mechanisms
 */

#include "hierarchical_filter.h"
#include "../utils/math_utils.h"
#include "../maps/composite_map_manager.h"
#include <algorithm>
#include <numeric>
#include <sstream>

namespace Navigation {

/**
 * Hierarchical Filter - YAML Constructor
 */
HierarchicalFilter::HierarchicalFilter(const YAML::Node& ukf_node,
                                     const YAML::Node& rbpf_node,
                                     MapManager* maps)
    : maps_(maps), perf_monitor_(nullptr) {

    // Parse YAML configs
    HierarchicalConfig config;
    if (ukf_node["rate"]) config.ukf_rate = ukf_node["rate"].as<double>(100.0);
    if (rbpf_node["rate"]) config.rbpf_rate = rbpf_node["rate"].as<double>(1.0);

    if (ukf_node["adaptive"] && ukf_node["adaptive"]["enable"]) {
        config.enable_adaptive_rate = ukf_node["adaptive"]["enable"].as<bool>(true);
    }

    if (ukf_node["reset"]) {
        config.enable_hard_reset = ukf_node["reset"]["enable"].as<bool>(true);
    }

    config_ = config;
    current_mode_ = FilterMode::UKF_ONLY;
    adaptive_ukf_rate_ = config.ukf_rate;
    adaptive_rbpf_rate_ = config.rbpf_rate;

    LOG_INFO("Initializing Hierarchical Filter from YAML");

    // Initialize SR-UKF
    SRUKFConfig ukf_config;
    // Note: SRUKFConfig doesn't have use_schmidt_trigger field
    ukf_ = std::make_unique<SquareRootUKF>(ukf_config);

    // Initialize RBPF
    RBPFConfig rbpf_config;
    rbpf_config.num_particles = 100;
    rbpf_ = std::make_unique<RBPF>(rbpf_config);

    // Set maps for RBPF if available
    if (maps_) {
        auto composite_map = dynamic_cast<CompositeMapManager*>(maps_);
        if (composite_map) {
            // Note: RBPF expects shared_ptr, but we have raw pointers from composite map
            // For now, we'll skip setting these to avoid ownership issues
            // TODO: Refactor to use shared_ptr throughout
        }
    }

    LOG_INFO("Hierarchical filter initialized from YAML");
}

/**
 * Hierarchical Filter - Standard Constructor
 */
HierarchicalFilter::HierarchicalFilter(const HierarchicalConfig& config,
                                     MapManager* maps,
                                     PerformanceMonitor* perf)
    : maps_(maps), perf_monitor_(perf), config_(config),
      current_mode_(FilterMode::UKF_ONLY),
      adaptive_ukf_rate_(config.ukf_rate),
      adaptive_rbpf_rate_(config.rbpf_rate) {

    LOG_INFO("Initializing Hierarchical Filter Orchestrator");

    // Initialize SR-UKF
    SRUKFConfig ukf_config;
    // Note: SRUKFConfig doesn't have use_schmidt_trigger field
    ukf_ = std::make_unique<SquareRootUKF>(ukf_config);

    // Initialize RBPF
    RBPFConfig rbpf_config;
    rbpf_config.num_particles = 100;  // Adjust based on performance
    rbpf_ = std::make_unique<RBPF>(rbpf_config);

    // Set maps for RBPF if available
    if (maps_) {
        auto composite_map = dynamic_cast<CompositeMapManager*>(maps_);
        if (composite_map) {
            // Note: RBPF expects shared_ptr, but we have raw pointers from composite map
            // For now, we'll skip setting these to avoid ownership issues
            // TODO: Refactor to use shared_ptr throughout
        }
    }

    {
        std::stringstream msg;
        msg << "Hierarchical filter initialized with UKF rate: " << config.ukf_rate
            << " Hz, RBPF rate: " << config.rbpf_rate << " Hz";
        LOG_INFO(msg.str());
    }
}

bool HierarchicalFilter::initialize(const StateVector& initial_state) {
    // Create default covariance
    Eigen::Matrix<double, 21, 21> initial_cov = Eigen::Matrix<double, 21, 21>::Identity();
    initial_cov.block<3,3>(0,0) *= 100.0;    // Position: 10m std
    initial_cov.block<3,3>(3,3) *= 1.0;      // Velocity: 1m/s std
    initial_cov.block<3,3>(6,6) *= 0.01;     // Attitude: 0.1rad std
    initial_cov.block<3,3>(9,9) *= 0.01;     // Accel bias: 0.1m/s² std
    initial_cov.block<3,3>(12,12) *= 0.001;  // Gyro bias: 0.03rad/s std
    initial_cov.block<5,5>(15,15) *= 1.0;    // Gravity bias: 1mGal std

    return initialize(initial_state, initial_cov);
}

bool HierarchicalFilter::initialize(const StateVector& initial_state,
                                   const Eigen::Matrix<double, 21, 21>& initial_cov) {
    LOG_INFO("Initializing filter with initial state");

    // Initialize UKF
    ukf_->initialize(initial_state);

    // Initialize RBPF with full state and covariance
    rbpf_->initialize(initial_state, initial_cov);

    // Set initial combined state
    current_state_.ukf_state = initial_state;
    current_state_.ukf_covariance = initial_cov;
    current_state_.position_correction = Vector3d::Zero();
    current_state_.position_covariance = Matrix3d::Identity() * 10.0;

    current_state_.position = initial_state.position;
    current_state_.velocity = initial_state.velocity;
    current_state_.attitude = initial_state.quaternion;
    current_state_.covariance.setIdentity();
    current_state_.covariance.block<3,3>(0,0) = initial_cov.block<3,3>(0,0);
    current_state_.covariance.block<3,3>(3,3) = initial_cov.block<3,3>(3,3);
    current_state_.covariance.block<3,3>(6,6) = initial_cov.block<3,3>(6,6);

    current_state_.timestamp = 0;
    current_state_.mode = FilterMode::UKF_ONLY;
    current_state_.confidence = 1.0;
    current_state_.is_valid = true;

    LOG_INFO("Filter initialization complete");
    return true;
}

CombinedState HierarchicalFilter::processIMU(const IMUData& imu, double dt) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // High-frequency UKF prediction
    ukf_->predict(imu, dt);

    // Get updated UKF state
    auto ukf_state = ukf_->getState();
    auto ukf_cov = ukf_->getCovariance();

    // Update combined state with UKF prediction
    current_state_.ukf_state = ukf_state;
    current_state_.ukf_covariance = ukf_cov;

    // Apply RBPF correction if available
    if (current_mode_ == FilterMode::UKF_RBPF) {
        current_state_ = fuseEstimates(ukf_state, ukf_cov,
                                      current_state_.position_correction,
                                      current_state_.position_covariance);
    } else {
        // Direct UKF state
        current_state_.position = ukf_state.position;
        current_state_.velocity = ukf_state.velocity;
        current_state_.attitude = ukf_state.quaternion;
    }

    current_state_.timestamp = imu.timestamp;

    // Track performance
    auto end_time = std::chrono::high_resolution_clock::now();
    ukf_compute_time_ = std::chrono::duration<double>(end_time - start_time).count();

    if (perf_monitor_) {
        perf_monitor_->recordMetric("ukf_time", ukf_compute_time_);
    }

    return current_state_;
}

CombinedState HierarchicalFilter::processMeasurement(const SensorData& data) {
    // Check for outliers
    if (isOutlier(data)) {
        consecutive_outlier_count_++;
        {
            std::stringstream msg;
            msg << "Measurement rejected as outlier. Count: " << consecutive_outlier_count_;
            LOG_WARN(msg.str());
        }

        if (consecutive_outlier_count_ > config_.reset_triggers.consecutive_outliers) {
            LOG_WARN("Too many consecutive outliers. Checking reset conditions.");
            checkResetConditions();
        }

        return current_state_;  // Return without update
    }

    consecutive_outlier_count_ = 0;

    // Process based on available sensor data
    if (data.has_imu) {
        return processIMU(data.imu, data.dt);
    }

    if (data.has_baro) {
        ukf_->updateBarometer(data.barometer);
    }

    if (data.has_mag) {
        ukf_->updateMagnetometer(data.magnetometer);
    }

    if (data.has_grad) {
        ukf_->updateGradiometer(data.gradiometer);
        // Also update RBPF with gravity data
        if (current_mode_ == FilterMode::UKF_RBPF) {
            // Convert 6-element tensor to 5-element STF for RBPF
            Eigen::Matrix<double, 5, 1> tensor5;
            tensor5 << data.gradiometer.tensor(0),   // Txx
                      data.gradiometer.tensor(3),   // Tyy
                      data.gradiometer.tensor(4),   // Tzz
                      data.gradiometer.tensor(1),   // Txy
                      data.gradiometer.tensor(2);   // Txz
            rbpf_->updateGravity(tensor5);
        }
    }

    // Update combined state after measurement
    auto ukf_state = ukf_->getState();
    auto ukf_cov = ukf_->getCovariance();

    current_state_ = fuseEstimates(ukf_state, ukf_cov,
                                  current_state_.position_correction,
                                  current_state_.position_covariance);

    return current_state_;
}

CombinedState HierarchicalFilter::orchestrateFilters(const SensorData& sensor_data, double dt) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // 1. Process sensor data through appropriate filter
    runUKF(sensor_data, dt);

    // 2. Check if RBPF should run (based on rate and mode)
    static double rbpf_timer = 0;
    rbpf_timer += dt;

    if (rbpf_timer >= 1.0 / adaptive_rbpf_rate_ && current_mode_ != FilterMode::UKF_ONLY) {
        runRBPF(current_state_, rbpf_timer);
        rbpf_timer = 0;
    }

    // 3. Check mode transitions
    if (shouldTransitionMode()) {
        FilterMode new_mode = FilterMode::UKF_RBPF;  // Determine appropriate mode

        if (checkResetConditions()) {
            new_mode = FilterMode::RBPF_RESET;
        } else if (computeUncertaintyMetric() > 300.0) {
            new_mode = FilterMode::DEGRADED;
        }

        transitionMode(new_mode);
    }

    // 4. Update adaptive parameters
    if (config_.enable_adaptive_rate) {
        updateAdaptiveRates();
    }

    // 5. Fuse estimates from multiple filters
    auto ukf_state = ukf_->getState();
    auto ukf_cov = ukf_->getCovariance();
    auto rbpf_correction = getRBPFCorrection();

    current_state_ = fuseEstimates(ukf_state, ukf_cov,
                                  rbpf_correction,
                                  current_state_.position_covariance);

    // 6. Update state history
    state_history_.push(current_state_);
    if (state_history_.size() > max_history_size_) {
        state_history_.pop();
    }

    // 7. Log diagnostics
    logFilterState();
    logPerformanceMetrics();

    // Track total orchestration time
    auto end_time = std::chrono::high_resolution_clock::now();
    fusion_compute_time_ = std::chrono::duration<double>(end_time - start_time).count();

    return current_state_;
}

void HierarchicalFilter::runUKF(const SensorData& data, double dt) {
    // Process through UKF based on available sensors
    if (data.has_imu) {
        ukf_->predict(data.imu, dt);
    }
    updateUKFMeasurement(data);
}

void HierarchicalFilter::updateUKFMeasurement(const SensorData& data) {
    if (data.has_baro) {
        ukf_->updateBarometer(data.barometer);
    }
    if (data.has_mag) {
        ukf_->updateMagnetometer(data.magnetometer);
    }
    if (data.has_grad) {
        ukf_->updateGradiometer(data.gradiometer);
    }
}

void HierarchicalFilter::runRBPF(const CombinedState& ukf_state, double dt) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Propagate particles using UKF as proposal
    // RBPF prediction using IMU data (use zero biases for now)
    rbpf_->predict(Vector3d::Zero(), Vector3d::Zero(), dt);

    // Update weights based on map matching
    if (maps_) {
        // Get current altitude for terrain matching
        double altitude = -ukf_state.position.z();  // Convert to altitude

        // Update RBPF with terrain correlation
        rbpf_->updateTerrain(altitude);
    }

    // Resample if necessary
    // Resample is private - done internally by RBPF

    // Get position correction from RBPF
    // Get RBPF estimates
    auto rbpf_state = rbpf_->getMMSE();
    current_state_.position_correction = rbpf_state.position - ukf_state.position;
    current_state_.position_covariance = rbpf_->getCovariance().block<3,3>(0,0);

    // Track RBPF compute time
    auto end_time = std::chrono::high_resolution_clock::now();
    rbpf_compute_time_ = std::chrono::duration<double>(end_time - start_time).count();

    if (perf_monitor_) {
        perf_monitor_->recordMetric("rbpf_time", rbpf_compute_time_);
    }
}

Vector3d HierarchicalFilter::getRBPFCorrection() {
    if (current_mode_ == FilterMode::UKF_ONLY) {
        return Vector3d::Zero();
    }

    return current_state_.position_correction;
}

CombinedState HierarchicalFilter::fuseEstimates(const StateVector& ukf_state,
                                               const Eigen::Matrix<double, 21, 21>& ukf_cov,
                                               const Vector3d& rbpf_correction,
                                               const Eigen::Matrix3d& rbpf_cov) {
    CombinedState fused;

    // Store raw estimates
    fused.ukf_state = ukf_state;
    fused.ukf_covariance = ukf_cov;
    fused.position_correction = rbpf_correction;
    fused.position_covariance = rbpf_cov;

    // Weighted fusion based on uncertainties
    Matrix3d P_ukf = ukf_cov.block<3,3>(0,0);
    Matrix3d P_rbpf = rbpf_cov;

    // Information filter fusion
    Matrix3d P_fused_inv = P_ukf.inverse() + config_.map_weight * P_rbpf.inverse();
    Matrix3d P_fused = P_fused_inv.inverse();

    Vector3d pos_ukf = ukf_state.position;
    Vector3d pos_rbpf = ukf_state.position + rbpf_correction;

    fused.position = P_fused * (P_ukf.inverse() * pos_ukf +
                               config_.map_weight * P_rbpf.inverse() * pos_rbpf);

    // Direct UKF velocity and attitude (not corrected by RBPF)
    fused.velocity = ukf_state.velocity;
    fused.attitude = ukf_state.quaternion;

    // Build combined covariance
    fused.covariance.setZero();
    fused.covariance.block<3,3>(0,0) = P_fused;
    fused.covariance.block<3,3>(3,3) = ukf_cov.block<3,3>(3,3);  // Velocity
    fused.covariance.block<3,3>(6,6) = ukf_cov.block<3,3>(6,6);  // Attitude

    // Compute confidence based on uncertainty
    double pos_uncertainty = std::sqrt(P_fused.trace());
    fused.confidence = std::exp(-pos_uncertainty / 100.0);  // Exponential decay

    fused.mode = current_mode_;
    fused.timestamp = ukf_state.timestamp;
    fused.is_valid = true;

    return fused;
}

bool HierarchicalFilter::checkResetConditions() {
    // Check multiple reset triggers
    bool should_reset = false;

    // 1. Position uncertainty
    double pos_uncertainty = std::sqrt(current_state_.covariance.block<3,3>(0,0).trace());
    if (pos_uncertainty > config_.reset_triggers.position_uncertainty_threshold) {
        {
            std::stringstream msg;
            msg << "Position uncertainty exceeds threshold: " << pos_uncertainty;
            LOG_WARN(msg.str());
        }
        should_reset = true;
    }

    // 2. NEES consistency
    double nees = computeNEES();
    if (nees > config_.reset_triggers.nees_threshold) {
        {
            std::stringstream msg;
            msg << "NEES exceeds threshold: " << nees;
            LOG_WARN(msg.str());
        }
        should_reset = true;
    }

    // 3. Consecutive outliers
    if (consecutive_outlier_count_ > config_.reset_triggers.consecutive_outliers) {
        {
            std::stringstream msg;
            msg << "Too many consecutive outliers: " << consecutive_outlier_count_;
            LOG_WARN(msg.str());
        }
        should_reset = true;
    }

    // 4. Time since last fix
    double time_since_fix = current_state_.timestamp - last_map_fix_time_;
    if (time_since_fix > config_.reset_triggers.time_since_last_fix) {
        {
            std::stringstream msg;
            msg << "Too long since last fix: " << time_since_fix << " seconds";
            LOG_WARN(msg.str());
        }
        should_reset = true;
    }

    return should_reset && config_.enable_hard_reset;
}

void HierarchicalFilter::executeHardReset(const Vector3d& position_fix) {
    LOG_INFO("Executing hard reset with position fix");

    // Schmidt partial reset - reset position but preserve velocity/attitude
    StateVector reset_state = current_state_.ukf_state;
    reset_state.position = position_fix;

    // Reset covariance for position only
    Eigen::Matrix<double, 21, 21> reset_cov = current_state_.ukf_covariance;
    reset_cov.block<3,3>(0,0) = Matrix3d::Identity() * 25.0;  // 5m std

    // Reinitialize filters
    ukf_->setState(reset_state);
    ukf_->setCovariance(reset_cov);

    rbpf_->initialize(reset_state, reset_cov);

    // Update state
    current_state_.position = position_fix;
    current_state_.position_correction = Vector3d::Zero();
    last_reset_time_ = current_state_.timestamp;
    last_map_fix_time_ = current_state_.timestamp;
    consecutive_outlier_count_ = 0;

    // Transition back to normal mode
    transitionMode(FilterMode::UKF_RBPF);

    {
        std::stringstream msg;
        msg << "Hard reset complete. Position: " << position_fix.transpose();
        LOG_INFO(msg.str());
    }
}

void HierarchicalFilter::executePartialReset(const StateVector& partial_state) {
    LOG_INFO("Executing partial reset");

    // Selective state update
    StateVector current = ukf_->getState();

    // Update only specified components (non-zero values)
    if (partial_state.position.norm() > 0) {
        current.position = partial_state.position;
    }
    if (partial_state.velocity.norm() > 0) {
        current.velocity = partial_state.velocity;
    }

    ukf_->setState(current);

    LOG_INFO("Partial reset complete");
}

void HierarchicalFilter::updateAdaptiveRates() {
    // Adjust filter rates based on uncertainty
    double uncertainty = computeUncertaintyMetric();

    if (uncertainty < 50.0) {
        // Low uncertainty - reduce RBPF rate
        adaptive_rbpf_rate_ = config_.rbpf_rate * 0.5;
    } else if (uncertainty > 200.0) {
        // High uncertainty - increase RBPF rate
        adaptive_rbpf_rate_ = config_.rbpf_rate * 2.0;
    } else {
        adaptive_rbpf_rate_ = config_.rbpf_rate;
    }

    // Clamp rates
    adaptive_ukf_rate_ = std::max(10.0, std::min(200.0, adaptive_ukf_rate_));
    adaptive_rbpf_rate_ = std::max(0.1, std::min(10.0, adaptive_rbpf_rate_));
}

double HierarchicalFilter::computeUncertaintyMetric() const {
    // Combined uncertainty from position covariance
    return std::sqrt(current_state_.covariance.block<3,3>(0,0).trace());
}

double HierarchicalFilter::computeMapMatchingQuality() const {
    if (!rbpf_) return 0.0;

    // Get particle spread as quality metric
    // Get particle spread from covariance
    auto cov = rbpf_->getCovariance();
    double spread = sqrt(cov.block<3,3>(0,0).trace());
    double quality = std::exp(-spread / 100.0);  // Convert to [0,1]

    return quality;
}

void HierarchicalFilter::transitionMode(FilterMode new_mode) {
    if (new_mode == current_mode_) return;

    {
        std::stringstream msg;
        msg << "Transitioning from mode " << static_cast<int>(current_mode_)
             << " to " << static_cast<int>(new_mode);
        LOG_INFO(msg.str());
    }

    current_mode_ = new_mode;
    current_state_.mode = new_mode;

    // Mode-specific transitions
    switch (new_mode) {
        case FilterMode::UKF_ONLY:
            // Disable RBPF updates
            config_.map_weight = 0.0;
            break;

        case FilterMode::UKF_RBPF:
            // Normal operation
            config_.map_weight = 0.3;
            break;

        case FilterMode::RBPF_RESET:
            // Prepare for reset
            config_.map_weight = 0.5;
            if (maps_) {
                // Get best position from map matching
                Vector3d map_position = rbpf_->getMMSE().position;
                executeHardReset(map_position);
            }
            break;

        case FilterMode::DEGRADED:
            // Reduce all corrections
            config_.map_weight = 0.1;
            LOG_WARN("Entering degraded mode due to high uncertainty");
            break;
    }
}

bool HierarchicalFilter::shouldTransitionMode() {
    // Check conditions for mode transitions
    double uncertainty = computeUncertaintyMetric();
    double map_quality = computeMapMatchingQuality();

    if (current_mode_ == FilterMode::UKF_ONLY && map_quality > 0.7) {
        return true;  // Enable RBPF
    }

    if (current_mode_ == FilterMode::UKF_RBPF && uncertainty > 300.0) {
        return true;  // Consider reset or degraded
    }

    if (current_mode_ == FilterMode::DEGRADED && uncertainty < 100.0) {
        return true;  // Return to normal
    }

    return false;
}

bool HierarchicalFilter::checkFilterConsistency() {
    double nees = computeNEES();
    bool consistent = nees < config_.reset_triggers.nees_threshold;

    if (!consistent) {
        {
            std::stringstream msg;
            msg << "Filter inconsistent. NEES: " << nees;
            LOG_WARN(msg.str());
        }
    }

    return consistent;
}

double HierarchicalFilter::computeNEES() const {
    // Normalized Estimation Error Squared
    // Assumes ground truth available for comparison
    // In practice, use map constraints or other reference

    // Simplified version using covariance trace
    double cov_trace = current_state_.covariance.trace();
    return cov_trace / 9.0;  // Normalized by state dimension
}

double HierarchicalFilter::computeNIS(const VectorXd& innovation,
                                     const MatrixXd& S) const {
    // Normalized Innovation Squared
    return innovation.transpose() * S.inverse() * innovation;
}

bool HierarchicalFilter::isOutlier(const SensorData& data) {
    // Simplified outlier detection
    // In practice, use Mahalanobis distance with proper thresholds

    if (data.has_baro) {
        // Check altitude consistency
        double expected_alt = -current_state_.position.z();
        double measured_alt = data.barometer.altitude;
        double diff = std::abs(expected_alt - measured_alt);

        return diff > 50.0;  // 50m threshold
    }

    return false;
}

void HierarchicalFilter::updateOutlierStatistics(bool is_outlier) {
    if (is_outlier) {
        consecutive_outlier_count_++;
    } else {
        consecutive_outlier_count_ = 0;
    }
}

void HierarchicalFilter::logFilterState() {
    {
        std::stringstream msg;
        msg << "Filter State - Mode: " << static_cast<int>(current_mode_)
              << ", Position: " << current_state_.position.transpose()
              << ", Uncertainty: " << computeUncertaintyMetric()
              << ", Confidence: " << current_state_.confidence;
        LOG_DEBUG(msg.str());
    }
}

void HierarchicalFilter::logPerformanceMetrics() {
    if (perf_monitor_) {
        perf_monitor_->recordMetric("fusion_time", fusion_compute_time_);
        perf_monitor_->recordMetric("total_filter_time",
                                   ukf_compute_time_ + rbpf_compute_time_ + fusion_compute_time_);
        perf_monitor_->recordMetric("position_uncertainty", computeUncertaintyMetric());
    }
}

/**
 * Filter Health Monitor
 */
FilterHealthMonitor::FilterHealthMonitor() {
    nees_history_.reserve(window_size_);
    nis_history_.reserve(window_size_);
}

void FilterHealthMonitor::updateNEES(double nees) {
    nees_history_.push_back(nees);
    if (nees_history_.size() > window_size_) {
        nees_history_.erase(nees_history_.begin());
    }

    metrics_.nees_avg = std::accumulate(nees_history_.begin(), nees_history_.end(), 0.0) /
                       nees_history_.size();
}

void FilterHealthMonitor::updateNIS(double nis) {
    nis_history_.push_back(nis);
    if (nis_history_.size() > window_size_) {
        nis_history_.erase(nis_history_.begin());
    }

    metrics_.nis_avg = std::accumulate(nis_history_.begin(), nis_history_.end(), 0.0) /
                      nis_history_.size();
}

void FilterHealthMonitor::updateCovariance(const MatrixXd& P) {
    // Extract standard deviations
    metrics_.position_std = std::sqrt(P.block<3,3>(0,0).trace() / 3.0);
    metrics_.velocity_std = std::sqrt(P.block<3,3>(3,3).trace() / 3.0);
    metrics_.attitude_std = std::sqrt(P.block<3,3>(6,6).trace() / 3.0);
}

void FilterHealthMonitor::recordOutlier() {
    metrics_.outlier_count++;
}

bool FilterHealthMonitor::isHealthy() const {
    // Check health criteria
    bool nees_ok = metrics_.nees_avg < 15.0;
    bool nis_ok = metrics_.nis_avg < 12.0;
    bool outliers_ok = metrics_.outlier_count < 10;
    bool cov_ok = metrics_.position_std < 100.0;

    return nees_ok && nis_ok && outliers_ok && cov_ok;
}

double FilterHealthMonitor::getHealthScore() const {
    double score = 1.0;

    // Penalize based on metrics
    score *= std::exp(-metrics_.nees_avg / 20.0);
    score *= std::exp(-metrics_.nis_avg / 15.0);
    score *= std::exp(-metrics_.outlier_count / 20.0);
    score *= std::exp(-metrics_.position_std / 200.0);

    return std::max(0.0, std::min(1.0, score));
}

std::string FilterHealthMonitor::getHealthStatus() const {
    double score = getHealthScore();

    if (score > 0.8) return "EXCELLENT";
    if (score > 0.6) return "GOOD";
    if (score > 0.4) return "FAIR";
    if (score > 0.2) return "POOR";
    return "CRITICAL";
}

void FilterHealthMonitor::reset() {
    metrics_ = HealthMetrics();
    nees_history_.clear();
    nis_history_.clear();
}

/**
 * Adaptive Controller
 */
AdaptiveController::AdaptiveController(const HierarchicalConfig& config) {
    nominal_params_.ukf_rate = config.ukf_rate;
    nominal_params_.rbpf_rate = config.rbpf_rate;
    nominal_params_.ukf_weight = config.ukf_weight;
    nominal_params_.process_noise_scale = 1.0;
    nominal_params_.measurement_noise_scale = 1.0;

    params_ = nominal_params_;
}

void AdaptiveController::adapt(const CombinedState& state,
                              const FilterHealthMonitor& health) {
    // Adapt rates based on uncertainty
    params_.ukf_rate = computeOptimalUKFRate(state);
    params_.rbpf_rate = computeOptimalRBPFRate(state);

    // Adapt weighting based on health
    params_.ukf_weight = computeOptimalWeighting(health);

    // Adapt noise parameters
    adaptNoiseParameters(health);
}

double AdaptiveController::computeOptimalUKFRate(const CombinedState& state) {
    // Higher rate when uncertainty is high
    double uncertainty = std::sqrt(state.covariance.block<3,3>(0,0).trace());
    double rate_factor = 1.0 + (uncertainty / 100.0);

    return std::min(200.0, nominal_params_.ukf_rate * rate_factor);
}

double AdaptiveController::computeOptimalRBPFRate(const CombinedState& state) {
    // Higher rate when position uncertainty is high
    double pos_uncertainty = std::sqrt(state.covariance.block<3,3>(0,0).trace());

    if (pos_uncertainty < 50.0) {
        return nominal_params_.rbpf_rate * 0.5;
    } else if (pos_uncertainty > 200.0) {
        return nominal_params_.rbpf_rate * 2.0;
    }

    return nominal_params_.rbpf_rate;
}

double AdaptiveController::computeOptimalWeighting(const FilterHealthMonitor& health) {
    double health_score = health.getHealthScore();

    // Trust UKF more when health is good
    if (health_score > 0.8) {
        return 0.8;  // High UKF weight
    } else if (health_score < 0.4) {
        return 0.4;  // Rely more on RBPF
    }

    return nominal_params_.ukf_weight;
}

void AdaptiveController::adaptNoiseParameters(const FilterHealthMonitor& health) {
    // Increase noise when filter is unhealthy
    if (health.getAverageNEES() > 10.0) {
        params_.process_noise_scale *= 1.1;
    } else if (health.getAverageNEES() < 5.0) {
        params_.process_noise_scale *= 0.95;
    }

    // Clamp scales
    params_.process_noise_scale = std::max(0.1, std::min(10.0, params_.process_noise_scale));
    params_.measurement_noise_scale = std::max(0.1, std::min(10.0, params_.measurement_noise_scale));
}

/**
 * Multi-Hypothesis Tracker
 */
MultiHypothesisTracker::MultiHypothesisTracker() {
    LOG_DEBUG("MultiHypothesisTracker initialized");
}

void MultiHypothesisTracker::addHypothesis(const CombinedState& state, double likelihood) {
    Hypothesis hyp;
    hyp.state = state;
    hyp.likelihood = likelihood;
    hyp.weight = likelihood;  // Initial weight
    hyp.id = next_id_++;

    hypotheses_.push_back(hyp);

    // Normalize weights
    double total_weight = 0;
    for (const auto& h : hypotheses_) {
        total_weight += h.weight;
    }
    for (auto& h : hypotheses_) {
        h.weight /= total_weight;
    }

    {
        std::stringstream msg;
        msg << "Added hypothesis " << hyp.id << " with weight " << hyp.weight;
        LOG_DEBUG(msg.str());
    }
}

void MultiHypothesisTracker::updateHypotheses(const SensorData& measurement) {
    // Update each hypothesis with measurement
    for (auto& hyp : hypotheses_) {
        // Compute likelihood of measurement given hypothesis
        // Simplified version - in practice, use proper measurement model
        double innovation = 0;  // Compute actual innovation

        hyp.likelihood = std::exp(-innovation * innovation / 2.0);
        hyp.weight *= hyp.likelihood;
    }

    // Normalize weights
    double total_weight = 0;
    for (const auto& h : hypotheses_) {
        total_weight += h.weight;
    }
    if (total_weight > 0) {
        for (auto& h : hypotheses_) {
            h.weight /= total_weight;
        }
    }
}

void MultiHypothesisTracker::pruneHypotheses() {
    // Remove low-weight hypotheses
    hypotheses_.erase(
        std::remove_if(hypotheses_.begin(), hypotheses_.end(),
                      [this](const Hypothesis& h) {
                          return h.weight < pruning_threshold_;
                      }),
        hypotheses_.end()
    );

    // Keep only top N hypotheses
    if (hypotheses_.size() > max_hypotheses_) {
        std::partial_sort(hypotheses_.begin(),
                         hypotheses_.begin() + max_hypotheses_,
                         hypotheses_.end(),
                         [](const Hypothesis& a, const Hypothesis& b) {
                             return a.weight > b.weight;
                         });
        hypotheses_.resize(max_hypotheses_);
    }
}

void MultiHypothesisTracker::mergeHypotheses() {
    // Merge similar hypotheses
    const double merge_threshold = 10.0;  // meters

    for (size_t i = 0; i < hypotheses_.size(); ++i) {
        for (size_t j = i + 1; j < hypotheses_.size(); ) {
            double dist = (hypotheses_[i].state.position -
                         hypotheses_[j].state.position).norm();

            if (dist < merge_threshold) {
                // Merge j into i
                hypotheses_[i].weight += hypotheses_[j].weight;
                hypotheses_.erase(hypotheses_.begin() + j);
            } else {
                ++j;
            }
        }
    }
}

CombinedState MultiHypothesisTracker::getBestEstimate() const {
    if (hypotheses_.empty()) {
        CombinedState invalid;
        invalid.is_valid = false;
        return invalid;
    }

    // Return highest weight hypothesis
    auto best = std::max_element(hypotheses_.begin(), hypotheses_.end(),
                                [](const Hypothesis& a, const Hypothesis& b) {
                                    return a.weight < b.weight;
                                });

    return best->state;
}

bool MultiHypothesisTracker::isMultiModal() const {
    if (hypotheses_.size() < 2) return false;

    // Check if multiple hypotheses have significant weight
    int significant_count = 0;
    for (const auto& h : hypotheses_) {
        if (h.weight > 0.2) {
            significant_count++;
        }
    }

    return significant_count > 1;
}

double MultiHypothesisTracker::getAmbiguityLevel() const {
    if (hypotheses_.empty()) return 1.0;

    // Compute entropy of weight distribution
    double entropy = 0;
    for (const auto& h : hypotheses_) {
        if (h.weight > 0) {
            entropy -= h.weight * std::log(h.weight);
        }
    }

    return entropy / std::log(hypotheses_.size());  // Normalize to [0,1]
}

/**
 * Navigation System Interface Methods
 * These provide compatibility with navigation_system.cpp
 */

void HierarchicalFilter::updateBarometer(const BarometerData& baro) {
    SensorData data;
    data.barometer = baro;
    data.has_baro = true;
    data.timestamp = baro.timestamp;

    processMeasurement(data);
}

void HierarchicalFilter::updateMagnetometer(const MagnetometerData& mag) {
    SensorData data;
    data.magnetometer = mag;
    data.has_mag = true;
    data.timestamp = mag.timestamp;

    processMeasurement(data);
}

void HierarchicalFilter::updateGravity(const GradiometerData& grad) {
    SensorData data;
    data.gradiometer = grad;
    data.has_grad = true;
    data.timestamp = grad.timestamp;

    processMeasurement(data);
}

void HierarchicalFilter::updateMLBias(const Vector3d& bias_pred, double uncertainty) {
    // Update UKF with ML bias prediction
    if (ukf_) {
        // Create pseudo-measurement for bias
        StateVector current = ukf_->getState();

        // Update accelerometer bias
        current.accel_bias = bias_pred;

        // Partial state update with uncertainty
        Eigen::Matrix<double, 21, 21> P = ukf_->getCovariance();
        P.block<3,3>(9,9) = Eigen::Matrix3d::Identity() * uncertainty;

        ukf_->setState(current);
        ukf_->setCovariance(P);

        {
            std::stringstream msg;
            msg << "ML bias update applied: " << bias_pred.transpose();
            LOG_DEBUG(msg.str());
        }
    }
}

void HierarchicalFilter::updateRBPF(const std::vector<GradiometerData>& gravity_buffer) {
    if (rbpf_ && !gravity_buffer.empty()) {
        // Process gravity measurements through RBPF
        for (const auto& grad : gravity_buffer) {
            // Convert 6-element tensor to 5-element STF for RBPF
            Eigen::Matrix<double, 5, 1> tensor5;
            tensor5 << grad.tensor(0),   // Txx
                      grad.tensor(3),   // Tyy
                      grad.tensor(4),   // Tzz
                      grad.tensor(1),   // Txy
                      grad.tensor(2);   // Txz
            rbpf_->updateGravity(tensor5);
        }

        // Run full RBPF update
        runRBPF(current_state_, 1.0 / config_.rbpf_rate);

        // Update position correction
        current_state_.position_correction = getRBPFCorrection();

        {
            std::stringstream msg;
            msg << "RBPF update complete with " << gravity_buffer.size() << " gravity measurements";
            LOG_DEBUG(msg.str());
        }
    }
}

HierarchicalFilter::ResetResult HierarchicalFilter::performReset() {
    ResetResult result;

    if (checkResetConditions()) {
        // Get best position estimate from RBPF
        Vector3d reset_position = rbpf_->getMMSE().position;
        Vector3d position_before = current_state_.position;

        // Execute reset
        executeHardReset(reset_position);

        // Calculate jump and confidence
        result.position_jump = (reset_position - position_before).norm();
        result.confidence = current_state_.confidence;

        reset_count_++;

        {
            std::stringstream msg;
            msg << "Position reset #" << reset_count_ << " executed. Jump: "
                 << result.position_jump << "m";
            LOG_INFO(msg.str());
        }
    } else {
        result.position_jump = 0;
        result.confidence = current_state_.confidence;
    }

    return result;
}

} // namespace Navigation