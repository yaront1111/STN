/**
 * Hierarchical Filter Orchestrator
 * Coordinates SR-UKF, RBPF, and hard reset mechanisms
 * for GPS-free navigation
 */

#pragma once

#include "ukf/sr_ukf.h"
#include "rbpf/rbpf.h"
#include "../sensors/sensor_manager.h"
#include "../maps/map_manager.h"
#include "../utils/logger.h"
#include "../utils/performance_monitor.h"
#include <memory>
#include <queue>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace Navigation {

/**
 * Filter mode enum
 */
enum class FilterMode {
    UKF_ONLY,        // Pure SR-UKF (high frequency)
    UKF_RBPF,        // UKF + RBPF (normal operation)
    RBPF_RESET,      // RBPF triggering hard reset
    DEGRADED         // Degraded mode (large uncertainty)
};

/**
 * Reset trigger conditions
 */
struct ResetTrigger {
    double position_uncertainty_threshold = 500.0;  // meters
    double nees_threshold = 15.0;                   // chi-squared bound
    double nis_threshold = 12.0;                    // chi-squared bound
    int consecutive_outliers = 10;                  // consecutive rejections
    double time_since_last_fix = 300.0;            // seconds
};

/**
 * Filter orchestration config
 */
struct HierarchicalConfig {
    double ukf_rate = 100.0;           // Hz
    double rbpf_rate = 1.0;            // Hz
    bool enable_adaptive_rate = true;   // Adaptive filter rates
    bool enable_hard_reset = true;      // Enable Schmidt reset
    ResetTrigger reset_triggers;        // Reset conditions
    double ukf_weight = 0.7;            // UKF vs RBPF weight
    double map_weight = 0.3;            // Map matching weight
};

/**
 * Combined navigation state
 */
struct CombinedState {
    // Primary state (from UKF)
    StateVector ukf_state;
    Eigen::Matrix<double, 21, 21> ukf_covariance;

    // RBPF correction
    Vector3d position_correction;
    Eigen::Matrix3d position_covariance;

    // Combined estimate
    Vector3d position;
    Vector3d velocity;
    Quaterniond attitude;
    Eigen::Matrix<double, 9, 9> covariance;  // Pos, vel, att

    // Metadata
    double timestamp;
    FilterMode mode;
    double confidence;  // [0, 1]
    bool is_valid;
};

/**
 * Hierarchical Filter Orchestrator
 */
class HierarchicalFilter {
private:
    // Core filters
    std::unique_ptr<SquareRootUKF> ukf_;
    std::unique_ptr<RBPF> rbpf_;

    // External interfaces
    std::shared_ptr<MapManager> maps_;
    PerformanceMonitor* perf_monitor_;

    // Configuration
    HierarchicalConfig config_;

    // State management
    CombinedState current_state_;
    FilterMode current_mode_;
    std::queue<CombinedState> state_history_;
    size_t max_history_size_ = 1000;

    // Reset management
    double last_reset_time_ = 0;
    double last_map_fix_time_ = 0;
    int consecutive_outlier_count_ = 0;
    int reset_count_ = 0;

    // Performance tracking
    double ukf_compute_time_ = 0;
    double rbpf_compute_time_ = 0;
    double fusion_compute_time_ = 0;
    double rbpf_timer_ = 0.0;

    // Adaptive rate control
    double adaptive_ukf_rate_;
    double adaptive_rbpf_rate_;

public:
    HierarchicalFilter(const HierarchicalConfig& config,
                      std::shared_ptr<MapManager> maps,
                      PerformanceMonitor* perf = nullptr);
    HierarchicalFilter(const YAML::Node& ukf_config,
                      const YAML::Node& rbpf_config,
                      std::shared_ptr<MapManager> maps);  // YAML constructor
    ~HierarchicalFilter() = default;

    // Initialize filters with initial state
    bool initialize(const StateVector& initial_state,
                   const Eigen::Matrix<double, 21, 21>& initial_cov);

    // Simplified initialization (creates default covariance)
    bool initialize(const StateVector& initial_state);

    // Main processing functions
    CombinedState processIMU(const IMUData& imu, double dt);
    CombinedState processMeasurement(const SensorData& data);

    // High-level orchestration
    CombinedState orchestrateFilters(const SensorData& sensor_data, double dt);

    // Get current state
    CombinedState getCurrentState() const { return current_state_; }
    FilterMode getCurrentMode() const { return current_mode_; }

    // Navigation system interface methods (wrappers for compatibility)
    void predictUKF(const IMUData& imu, double dt) { processIMU(imu, dt); }
    void updateBarometer(const BarometerData& baro);
    void updateMagnetometer(const MagnetometerData& mag);
    void updateGravity(const GradiometerData& grad);
    void updateMLBias(const Vector3d& bias_pred, double uncertainty);
    void updateRBPF(const std::vector<GradiometerData>& gravity_buffer);

    // State access for navigation system
    StateVector getState() const { return current_state_.ukf_state; }
    Eigen::Matrix<double, 21, 21> getCovariance() const { return current_state_.ukf_covariance; }

    // Reset interface
    bool shouldReset() { return checkResetConditions(); }
    struct ResetResult {
        double position_jump;
        double confidence;
    };
    ResetResult performReset();
    int getResetCount() const { return reset_count_; }

    // Reset mechanisms
    bool checkResetConditions();
    void executeHardReset(const Vector3d& position_fix);
    void executePartialReset(const StateVector& partial_state);

private:
    // UKF processing
    void runUKF(const SensorData& data, double dt);
    void updateUKFMeasurement(const SensorData& data);

    // RBPF processing
    void runRBPF(const CombinedState& ukf_state, double dt);
    Vector3d getRBPFCorrection();

    // State fusion
    CombinedState fuseEstimates(const StateVector& ukf_state,
                               const Eigen::Matrix<double, 21, 21>& ukf_cov,
                               const Vector3d& rbpf_correction,
                               const Eigen::Matrix3d& rbpf_cov);

    // Adaptive control
    void updateAdaptiveRates();
    double computeUncertaintyMetric() const;
    double computeMapMatchingQuality() const;

    // Mode transitions
    void transitionMode(FilterMode new_mode);
    bool shouldTransitionMode();

    // Consistency monitoring
    bool checkFilterConsistency();
    double computeNEES() const;
    double computeNIS(const VectorXd& innovation,
                     const MatrixXd& S) const;

    // Outlier detection
    bool isOutlier(const SensorData& data);
    void updateOutlierStatistics(bool is_outlier);

    // Logging and diagnostics
    void logFilterState();
    void logPerformanceMetrics();
};

/**
 * Filter Health Monitor
 */
class FilterHealthMonitor {
private:
    struct HealthMetrics {
        double nees_avg = 0;
        double nis_avg = 0;
        double position_std = 0;
        double velocity_std = 0;
        double attitude_std = 0;
        int outlier_count = 0;
        double last_update_time = 0;
    };

    HealthMetrics metrics_;
    std::vector<double> nees_history_;
    std::vector<double> nis_history_;
    size_t window_size_ = 100;

public:
    FilterHealthMonitor();

    // Update health metrics
    void updateNEES(double nees);
    void updateNIS(double nis);
    void updateCovariance(const MatrixXd& P);
    void recordOutlier();

    // Health assessment
    bool isHealthy() const;
    double getHealthScore() const;  // [0, 1]
    std::string getHealthStatus() const;

    // Get specific metrics
    double getAverageNEES() const { return metrics_.nees_avg; }
    double getAverageNIS() const { return metrics_.nis_avg; }
    int getOutlierCount() const { return metrics_.outlier_count; }

    // Reset metrics
    void reset();
};

/**
 * Adaptive Filter Controller
 */
class AdaptiveController {
private:
    struct AdaptiveParams {
        double ukf_rate;
        double rbpf_rate;
        double ukf_weight;
        double process_noise_scale;
        double measurement_noise_scale;
    };

    AdaptiveParams params_;
    AdaptiveParams nominal_params_;

    // Adaptation gains (unused for now)
    // double rate_adaptation_gain_ = 0.1;
    // double weight_adaptation_gain_ = 0.05;
    // double noise_adaptation_gain_ = 0.01;

public:
    AdaptiveController(const HierarchicalConfig& config);

    // Adapt parameters based on current conditions
    void adapt(const CombinedState& state,
              const FilterHealthMonitor& health);

    // Get adapted parameters
    AdaptiveParams getParameters() const { return params_; }

    // Manual overrides
    void setUKFRate(double rate) { params_.ukf_rate = rate; }
    void setRBPFRate(double rate) { params_.rbpf_rate = rate; }
    void setUKFWeight(double weight) { params_.ukf_weight = weight; }

    // Reset to nominal
    void reset() { params_ = nominal_params_; }

private:
    // Adaptation logic
    double computeOptimalUKFRate(const CombinedState& state);
    double computeOptimalRBPFRate(const CombinedState& state);
    double computeOptimalWeighting(const FilterHealthMonitor& health);
    void adaptNoiseParameters(const FilterHealthMonitor& health);
};

/**
 * Multi-hypothesis tracker for ambiguous situations
 */
class MultiHypothesisTracker {
private:
    struct Hypothesis {
        CombinedState state;
        double likelihood;
        double weight;
        int id;
    };

    std::vector<Hypothesis> hypotheses_;
    int next_id_ = 0;
    size_t max_hypotheses_ = 5;
    double pruning_threshold_ = 0.01;

public:
    MultiHypothesisTracker();

    // Hypothesis management
    void addHypothesis(const CombinedState& state, double likelihood);
    void updateHypotheses(const SensorData& measurement);
    void pruneHypotheses();
    void mergeHypotheses();

    // Get best hypothesis
    CombinedState getBestEstimate() const;
    std::vector<Hypothesis> getHypotheses() const { return hypotheses_; }

    // Multi-modal detection
    bool isMultiModal() const;
    double getAmbiguityLevel() const;
};

} // namespace Navigation