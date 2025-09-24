/**
 * Trajectory Analyzer
 * Real-time analysis and debugging of navigation trajectories
 * Compares estimated trajectory with ground truth if available
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <fstream>
#include <memory>
#include <optional>
#include "../core/hierarchical_filter.h"
#include "../core/ukf/sr_ukf.h"  // For StateVector
#include "logger.h"

namespace Navigation {

using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::MatrixXd;

/**
 * Trajectory point with full state and metadata
 */
struct TrajectoryPoint {
    double timestamp;
    Vector3d position;
    Vector3d velocity;
    Quaterniond attitude;
    Vector3d acceleration;
    Vector3d angular_velocity;

    // Uncertainty
    Eigen::Matrix3d position_covariance;
    Eigen::Matrix3d velocity_covariance;
    Eigen::Matrix3d attitude_covariance;

    // Metadata
    FilterMode filter_mode;
    double confidence;
    int outlier_count;
    double nees;
    double nis;
};

/**
 * Ground truth data
 */
struct GroundTruth {
    double timestamp;
    Vector3d position;
    Vector3d velocity;
    Quaterniond attitude;
    bool valid = true;
};

/**
 * Trajectory metrics
 */
struct TrajectoryMetrics {
    // Position errors
    double mean_position_error = 0;
    double rms_position_error = 0;
    double max_position_error = 0;
    double final_position_error = 0;

    // Velocity errors
    double mean_velocity_error = 0;
    double rms_velocity_error = 0;
    double max_velocity_error = 0;

    // Attitude errors
    double mean_attitude_error = 0;  // degrees
    double rms_attitude_error = 0;   // degrees
    double max_attitude_error = 0;   // degrees

    // Along-track/cross-track errors
    double mean_along_track_error = 0;
    double mean_cross_track_error = 0;
    double mean_vertical_error = 0;

    // Consistency metrics
    double average_nees = 0;
    double average_nis = 0;
    double consistency_score = 0;  // [0,1]

    // Performance metrics
    double total_distance_traveled = 0;
    double computation_time = 0;
    int total_outliers = 0;
    int total_resets = 0;
};

/**
 * Error statistics over time windows
 */
struct WindowedStatistics {
    double window_duration;  // seconds
    std::vector<double> timestamps;
    std::vector<double> position_errors;
    std::vector<double> velocity_errors;
    std::vector<double> attitude_errors;
    std::vector<double> nees_values;
    std::vector<double> nis_values;
};

/**
 * Trajectory Analyzer class
 */
class TrajectoryAnalyzer {
private:
    // Trajectory storage
    std::deque<TrajectoryPoint> estimated_trajectory_;
    std::deque<GroundTruth> truth_trajectory_;
    size_t max_trajectory_size_ = 10000;

    // Analysis windows
    std::vector<double> analysis_windows_ = {30, 60, 300, 600, 1800};  // seconds
    std::map<double, WindowedStatistics> windowed_stats_;

    // Real-time metrics
    TrajectoryMetrics current_metrics_;
    TrajectoryMetrics cumulative_metrics_;

    // Output files
    std::ofstream trajectory_file_;
    std::ofstream metrics_file_;
    std::ofstream error_file_;
    bool log_to_file_ = true;

    // Analysis parameters
    double outlier_threshold_ = 3.0;  // sigma
    double reset_detection_threshold_ = 50.0;  // meters

public:
    TrajectoryAnalyzer(bool enable_file_logging = true);
    TrajectoryAnalyzer(const std::string& truth_file);
    ~TrajectoryAnalyzer();

    // Add trajectory points
    void addEstimatedPoint(const CombinedState& state);
    void addEstimatedPoint(const TrajectoryPoint& point);
    void addGroundTruth(const GroundTruth& truth);

    // Load ground truth from file
    bool loadGroundTruth(const std::string& filename);

    // Real-time analysis
    TrajectoryMetrics analyzeTrajectory();
    TrajectoryMetrics analyzeWindow(double window_seconds);

    // Get metrics
    TrajectoryMetrics getCurrentMetrics() const { return current_metrics_; }
    TrajectoryMetrics getCumulativeMetrics() const { return cumulative_metrics_; }
    WindowedStatistics getWindowedStats(double window) const;

    // Error analysis
    Vector3d computePositionError(double timestamp) const;
    double computeVelocityError(double timestamp) const;
    double computeAttitudeError(double timestamp) const;

    // Track-relative errors
    void computeTrackErrors(const TrajectoryPoint& estimated,
                           const GroundTruth& truth,
                           double& along_track,
                           double& cross_track,
                           double& vertical) const;

    // Consistency analysis
    double computeConsistencyScore() const;
    bool isConsistent(double nees, double nis) const;

    // Outlier and reset detection
    bool detectOutlier(const TrajectoryPoint& point) const;
    bool detectReset(const TrajectoryPoint& current,
                    const TrajectoryPoint& previous) const;

    // Trajectory smoothness
    double computeSmoothness() const;
    double computeJerk(size_t index) const;

    // Export functions
    void exportTrajectory(const std::string& filename) const;
    void exportMetrics(const std::string& filename) const;
    void exportKML(const std::string& filename) const;

    // Navigation system compatibility methods
    void updateEstimate(const StateVector& state) {
        NavigationState nav_state;
        nav_state.timestamp = state.timestamp;
        nav_state.position = state.position;
        nav_state.velocity = state.velocity;
        nav_state.quaternion = state.quaternion;
        nav_state.accel_bias = state.accel_bias;
        nav_state.gyro_bias = state.gyro_bias;

        TrajectoryPoint point;
        point.timestamp = state.timestamp;
        point.position = state.position;
        point.velocity = state.velocity;
        point.attitude = state.quaternion;
        addEstimatedPoint(point);
    }

    struct ErrorStats {
        double position_error;
        double velocity_error;
    };

    ErrorStats getCurrentErrors() const {
        ErrorStats errors;
        errors.position_error = current_metrics_.mean_position_error;
        errors.velocity_error = current_metrics_.mean_velocity_error;
        return errors;
    }

    struct FinalStats {
        double position_rmse;
        double velocity_rmse;
        double max_position_error;
        double final_position_error;
    };

    FinalStats getFinalStatistics() const {
        FinalStats stats;
        stats.position_rmse = cumulative_metrics_.rms_position_error;
        stats.velocity_rmse = cumulative_metrics_.rms_velocity_error;
        stats.max_position_error = cumulative_metrics_.max_position_error;
        stats.final_position_error = cumulative_metrics_.final_position_error;
        return stats;
    }

    void saveTrajectory(const std::string& filename) { exportTrajectory(filename); }
    void saveErrorPlots(const std::string& filename) {
        // For now, just export metrics - plotting would require additional library
        exportMetrics(filename);
    }

    // Visualization helpers
    std::vector<Vector3d> getEstimatedPath() const;
    std::vector<Vector3d> getTruthPath() const;
    std::vector<double> getErrorMagnitudes() const;

private:
    // Internal analysis functions
    void updateMetrics(const TrajectoryPoint& estimated,
                      const GroundTruth& truth);
    void updateWindowedStats(const TrajectoryPoint& estimated,
                            const GroundTruth& truth);

    // Interpolation for time alignment
    std::optional<GroundTruth> interpolateTruth(double timestamp) const;

    // Statistics computation
    double computeMean(const std::vector<double>& values) const;
    double computeRMS(const std::vector<double>& values) const;
    double computeMax(const std::vector<double>& values) const;
    double computeStd(const std::vector<double>& values) const;

    // File I/O
    void logTrajectoryPoint(const TrajectoryPoint& point);
    void logMetrics(const TrajectoryMetrics& metrics);
    void logError(double timestamp, const Vector3d& error);
};

/**
 * Specialized analyzer for filter debugging
 */
class FilterDebugger : public TrajectoryAnalyzer {
private:
    struct FilterEvent {
        double timestamp;
        std::string event_type;  // "reset", "mode_change", "outlier", etc.
        std::string description;
        std::map<std::string, double> parameters;
    };

    std::vector<FilterEvent> events_;
    std::ofstream debug_log_;

public:
    FilterDebugger(const std::string& debug_file);

    // Log filter events
    void logReset(double timestamp, const Vector3d& reset_position);
    void logModeChange(double timestamp, FilterMode from, FilterMode to);
    void logOutlier(double timestamp, const std::string& sensor_type);
    void logMeasurementUpdate(double timestamp,
                             const std::string& sensor_type,
                             double innovation,
                             double nis);

    // Analyze filter behavior
    void analyzeResets();
    void analyzeOutliers();
    void analyzeModeTransitions();
    void analyzeInnovations();

    // Generate debug report
    void generateReport(const std::string& filename);
};

/**
 * Monte Carlo analyzer for statistical validation
 */
class MonteCarloAnalyzer {
private:
    struct RunResult {
        int run_id;
        TrajectoryMetrics metrics;
        std::vector<TrajectoryPoint> trajectory;
        bool converged;
        double convergence_time;
    };

    std::vector<RunResult> results_;
    TrajectoryMetrics mean_metrics_;
    TrajectoryMetrics std_metrics_;

public:
    MonteCarloAnalyzer();

    // Add Monte Carlo run
    void addRun(int run_id, const TrajectoryMetrics& metrics,
               const std::vector<TrajectoryPoint>& trajectory);

    // Statistical analysis
    void computeStatistics();
    TrajectoryMetrics getMeanMetrics() const { return mean_metrics_; }
    TrajectoryMetrics getStdMetrics() const { return std_metrics_; }

    // Convergence analysis
    double getConvergenceRate() const;
    double getMeanConvergenceTime() const;

    // Generate statistical report
    void generateStatisticalReport(const std::string& filename);
    void plotHistograms(const std::string& output_dir);
};

/**
 * Real-time performance analyzer
 */
class PerformanceAnalyzer {
private:
    struct TimingData {
        double timestamp;
        double ukf_time;
        double rbpf_time;
        double fusion_time;
        double total_time;
        size_t memory_usage;
    };

    std::deque<TimingData> timing_history_;
    size_t max_history_ = 1000;

    // Statistics
    double mean_ukf_time_ = 0;
    double mean_rbpf_time_ = 0;
    double mean_fusion_time_ = 0;
    double mean_total_time_ = 0;
    double max_total_time_ = 0;

public:
    PerformanceAnalyzer();

    // Record timing data
    void recordTiming(double timestamp,
                     double ukf_time,
                     double rbpf_time,
                     double fusion_time,
                     size_t memory_usage);

    // Compute statistics
    void updateStatistics();

    // Get performance metrics
    double getMeanProcessingTime() const { return mean_total_time_; }
    double getMaxProcessingTime() const { return max_total_time_; }
    double getProcessingRate() const;  // Hz

    // Check real-time constraints
    bool meetsRealtimeConstraints(double max_time_ms) const;

    // Generate performance report
    void generatePerformanceReport(const std::string& filename);
};

} // namespace Navigation