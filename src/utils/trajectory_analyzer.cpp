/**
 * Trajectory Analyzer Implementation
 * Real-time analysis and debugging of navigation trajectories
 */

#include "trajectory_analyzer.h"
#include "../utils/math_utils.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace Navigation {

/**
 * Trajectory Analyzer
 */
TrajectoryAnalyzer::TrajectoryAnalyzer(bool enable_file_logging)
    : log_to_file_(enable_file_logging) {

    LOG_INFO("Initializing Trajectory Analyzer");

    if (log_to_file_) {
        // Open output files
        trajectory_file_.open("logs/trajectory.csv");
        metrics_file_.open("logs/metrics.csv");
        error_file_.open("logs/errors.csv");

        // Write headers
        if (trajectory_file_.is_open()) {
            trajectory_file_ << "timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw,"
                           << "cov_xx,cov_yy,cov_zz,confidence,mode,nees,nis\n";
        }

        if (metrics_file_.is_open()) {
            metrics_file_ << "timestamp,pos_error,vel_error,att_error,"
                         << "along_track,cross_track,vertical,"
                         << "nees,nis,consistency\n";
        }

        if (error_file_.is_open()) {
            error_file_ << "timestamp,error_x,error_y,error_z,error_mag\n";
        }
    }

    // Initialize analysis windows
    for (double window : analysis_windows_) {
        windowed_stats_[window] = WindowedStatistics{window, {}, {}, {}, {}, {}, {}};
    }
}

TrajectoryAnalyzer::TrajectoryAnalyzer(const std::string& truth_file)
    : TrajectoryAnalyzer(true) {

    if (!loadGroundTruth(truth_file)) {
        {

            std::stringstream msg;

            msg << "Failed to load ground truth from: " << truth_file;

            LOG_WARN(msg.str());

        }
    }
}

TrajectoryAnalyzer::~TrajectoryAnalyzer() {
    // Close files
    if (trajectory_file_.is_open()) trajectory_file_.close();
    if (metrics_file_.is_open()) metrics_file_.close();
    if (error_file_.is_open()) error_file_.close();

    // Log final metrics
    LOG_INFO("Final trajectory metrics:");
    {

        std::stringstream msg;

        msg << "  Mean position error: " << cumulative_metrics_.mean_position_error << " m";

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "  RMS position error: " << cumulative_metrics_.rms_position_error << " m";

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "  Final position error: " << cumulative_metrics_.final_position_error << " m";

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "  Total outliers: " << cumulative_metrics_.total_outliers;

        LOG_INFO(msg.str());

    }
    {

        std::stringstream msg;

        msg << "  Total resets: " << cumulative_metrics_.total_resets;

        LOG_INFO(msg.str());

    }
}

void TrajectoryAnalyzer::addEstimatedPoint(const CombinedState& state) {
    TrajectoryPoint point;

    point.timestamp = state.timestamp;
    point.position = state.position;
    point.velocity = state.velocity;
    point.attitude = state.attitude;

    // Extract covariances
    point.position_covariance = state.covariance.block<3,3>(0,0);
    point.velocity_covariance = state.covariance.block<3,3>(3,3);
    point.attitude_covariance = state.covariance.block<3,3>(6,6);

    // Metadata
    point.filter_mode = state.mode;
    point.confidence = state.confidence;

    addEstimatedPoint(point);
}

void TrajectoryAnalyzer::addEstimatedPoint(const TrajectoryPoint& point) {
    TrajectoryPoint mutable_point = point;

    // Detect outliers
    if (detectOutlier(mutable_point)) {
        cumulative_metrics_.total_outliers++;
        mutable_point.outlier_count = cumulative_metrics_.total_outliers;
    }

    // Detect resets
    if (!estimated_trajectory_.empty()) {
        if (detectReset(mutable_point, estimated_trajectory_.back())) {
            cumulative_metrics_.total_resets++;
            {

                std::stringstream msg;

                msg << "Reset detected at t=" << mutable_point.timestamp;

                LOG_WARN(msg.str());

            }
        }
    }

    // Add to trajectory
    estimated_trajectory_.push_back(mutable_point);

    // Limit size
    if (estimated_trajectory_.size() > max_trajectory_size_) {
        estimated_trajectory_.pop_front();
    }

    // Find corresponding ground truth
    auto truth_opt = interpolateTruth(point.timestamp);
    if (truth_opt.has_value()) {
        updateMetrics(point, truth_opt.value());
        updateWindowedStats(point, truth_opt.value());

        // Log error
        Vector3d error = point.position - truth_opt.value().position;
        if (log_to_file_ && error_file_.is_open()) {
            logError(point.timestamp, error);
        }
    }

    // Log trajectory point
    if (log_to_file_ && trajectory_file_.is_open()) {
        logTrajectoryPoint(point);
    }

    // Update total distance
    if (estimated_trajectory_.size() > 1) {
        auto& prev = estimated_trajectory_[estimated_trajectory_.size() - 2];
        cumulative_metrics_.total_distance_traveled +=
            (point.position - prev.position).norm();
    }
}

void TrajectoryAnalyzer::addGroundTruth(const GroundTruth& truth) {
    truth_trajectory_.push_back(truth);

    // Limit size
    if (truth_trajectory_.size() > max_trajectory_size_) {
        truth_trajectory_.pop_front();
    }
}

bool TrajectoryAnalyzer::loadGroundTruth(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        {

            std::stringstream msg;

            msg << "Cannot open ground truth file: " << filename;

            LOG_ERROR(msg.str());

        }
        return false;
    }

    std::string line;
    // Skip header
    std::getline(file, line);

    int count = 0;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        GroundTruth truth;

        // Parse CSV: timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw
        char comma;
        double roll, pitch, yaw;

        ss >> truth.timestamp >> comma
           >> truth.position.x() >> comma
           >> truth.position.y() >> comma
           >> truth.position.z() >> comma
           >> truth.velocity.x() >> comma
           >> truth.velocity.y() >> comma
           >> truth.velocity.z() >> comma
           >> roll >> comma >> pitch >> comma >> yaw;

        // Convert Euler angles to quaternion
        double roll_rad = roll * M_PI / 180.0;
        double pitch_rad = pitch * M_PI / 180.0;
        double yaw_rad = yaw * M_PI / 180.0;

        // Manual conversion from Euler angles to quaternion
        double cy = cos(yaw_rad * 0.5);
        double sy = sin(yaw_rad * 0.5);
        double cp = cos(pitch_rad * 0.5);
        double sp = sin(pitch_rad * 0.5);
        double cr = cos(roll_rad * 0.5);
        double sr = sin(roll_rad * 0.5);

        truth.attitude = Quaterniond(
            cr * cp * cy + sr * sp * sy,  // w
            sr * cp * cy - cr * sp * sy,  // x
            cr * sp * cy + sr * cp * sy,  // y
            cr * cp * sy - sr * sp * cy   // z
        );

        truth.valid = true;
        truth_trajectory_.push_back(truth);
        count++;
    }

    {


        std::stringstream msg;


        msg << "Loaded " << count << " ground truth points";


        LOG_INFO(msg.str());


    }
    return count > 0;
}

TrajectoryMetrics TrajectoryAnalyzer::analyzeTrajectory() {
    if (estimated_trajectory_.empty()) {
        return TrajectoryMetrics{};
    }

    TrajectoryMetrics metrics;
    std::vector<double> position_errors;
    std::vector<double> velocity_errors;
    std::vector<double> attitude_errors;
    std::vector<double> nees_values;
    std::vector<double> nis_values;

    for (const auto& point : estimated_trajectory_) {
        auto truth_opt = interpolateTruth(point.timestamp);
        if (!truth_opt.has_value()) continue;

        const auto& truth = truth_opt.value();

        // Position error
        Vector3d pos_error = point.position - truth.position;
        position_errors.push_back(pos_error.norm());

        // Velocity error
        Vector3d vel_error = point.velocity - truth.velocity;
        velocity_errors.push_back(vel_error.norm());

        // Attitude error
        Quaterniond q_error = point.attitude * truth.attitude.inverse();
        double angle_error = 2.0 * std::acos(std::abs(q_error.w()));
        attitude_errors.push_back(angle_error * 180.0 / M_PI);

        // Track errors
        double along, cross, vertical;
        computeTrackErrors(point, truth, along, cross, vertical);
        metrics.mean_along_track_error += along;
        metrics.mean_cross_track_error += cross;
        metrics.mean_vertical_error += vertical;

        // Consistency
        nees_values.push_back(point.nees);
        nis_values.push_back(point.nis);
    }

    if (!position_errors.empty()) {
        // Position metrics
        metrics.mean_position_error = computeMean(position_errors);
        metrics.rms_position_error = computeRMS(position_errors);
        metrics.max_position_error = computeMax(position_errors);
        metrics.final_position_error = position_errors.back();

        // Velocity metrics
        metrics.mean_velocity_error = computeMean(velocity_errors);
        metrics.rms_velocity_error = computeRMS(velocity_errors);
        metrics.max_velocity_error = computeMax(velocity_errors);

        // Attitude metrics
        metrics.mean_attitude_error = computeMean(attitude_errors);
        metrics.rms_attitude_error = computeRMS(attitude_errors);
        metrics.max_attitude_error = computeMax(attitude_errors);

        // Track errors (already accumulated)
        size_t n = position_errors.size();
        metrics.mean_along_track_error /= n;
        metrics.mean_cross_track_error /= n;
        metrics.mean_vertical_error /= n;

        // Consistency
        metrics.average_nees = computeMean(nees_values);
        metrics.average_nis = computeMean(nis_values);
        metrics.consistency_score = computeConsistencyScore();
    }

    metrics.total_distance_traveled = cumulative_metrics_.total_distance_traveled;
    metrics.total_outliers = cumulative_metrics_.total_outliers;
    metrics.total_resets = cumulative_metrics_.total_resets;

    current_metrics_ = metrics;

    // Log metrics
    if (log_to_file_ && metrics_file_.is_open()) {
        logMetrics(metrics);
    }

    return metrics;
}

TrajectoryMetrics TrajectoryAnalyzer::analyzeWindow(double window_seconds) {
    if (estimated_trajectory_.empty()) {
        return TrajectoryMetrics{};
    }

    double current_time = estimated_trajectory_.back().timestamp;
    double start_time = current_time - window_seconds;

    TrajectoryMetrics metrics;
    std::vector<double> position_errors;
    std::vector<double> velocity_errors;
    std::vector<double> attitude_errors;

    for (const auto& point : estimated_trajectory_) {
        if (point.timestamp < start_time) continue;

        auto truth_opt = interpolateTruth(point.timestamp);
        if (!truth_opt.has_value()) continue;

        const auto& truth = truth_opt.value();

        // Compute errors
        Vector3d pos_error = point.position - truth.position;
        position_errors.push_back(pos_error.norm());

        Vector3d vel_error = point.velocity - truth.velocity;
        velocity_errors.push_back(vel_error.norm());

        Quaterniond q_error = point.attitude * truth.attitude.inverse();
        double angle_error = 2.0 * std::acos(std::abs(q_error.w()));
        attitude_errors.push_back(angle_error * 180.0 / M_PI);
    }

    if (!position_errors.empty()) {
        metrics.mean_position_error = computeMean(position_errors);
        metrics.rms_position_error = computeRMS(position_errors);
        metrics.max_position_error = computeMax(position_errors);

        metrics.mean_velocity_error = computeMean(velocity_errors);
        metrics.rms_velocity_error = computeRMS(velocity_errors);

        metrics.mean_attitude_error = computeMean(attitude_errors);
        metrics.rms_attitude_error = computeRMS(attitude_errors);
    }

    return metrics;
}

WindowedStatistics TrajectoryAnalyzer::getWindowedStats(double window) const {
    auto it = windowed_stats_.find(window);
    if (it != windowed_stats_.end()) {
        return it->second;
    }
    return WindowedStatistics{window, {}, {}, {}, {}, {}, {}};
}

Vector3d TrajectoryAnalyzer::computePositionError(double timestamp) const {
    // Find estimated point
    auto est_it = std::find_if(estimated_trajectory_.begin(),
                              estimated_trajectory_.end(),
                              [timestamp](const TrajectoryPoint& p) {
                                  return std::abs(p.timestamp - timestamp) < 0.01;
                              });

    if (est_it == estimated_trajectory_.end()) {
        return Vector3d::Zero();
    }

    // Find ground truth
    auto truth_opt = interpolateTruth(timestamp);
    if (!truth_opt.has_value()) {
        return Vector3d::Zero();
    }

    return est_it->position - truth_opt.value().position;
}

double TrajectoryAnalyzer::computeVelocityError(double timestamp) const {
    // Find estimated point
    auto est_it = std::find_if(estimated_trajectory_.begin(),
                              estimated_trajectory_.end(),
                              [timestamp](const TrajectoryPoint& p) {
                                  return std::abs(p.timestamp - timestamp) < 0.01;
                              });

    if (est_it == estimated_trajectory_.end()) {
        return 0;
    }

    // Find ground truth
    auto truth_opt = interpolateTruth(timestamp);
    if (!truth_opt.has_value()) {
        return 0;
    }

    return (est_it->velocity - truth_opt.value().velocity).norm();
}

double TrajectoryAnalyzer::computeAttitudeError(double timestamp) const {
    // Find estimated point
    auto est_it = std::find_if(estimated_trajectory_.begin(),
                              estimated_trajectory_.end(),
                              [timestamp](const TrajectoryPoint& p) {
                                  return std::abs(p.timestamp - timestamp) < 0.01;
                              });

    if (est_it == estimated_trajectory_.end()) {
        return 0;
    }

    // Find ground truth
    auto truth_opt = interpolateTruth(timestamp);
    if (!truth_opt.has_value()) {
        return 0;
    }

    Quaterniond q_error = est_it->attitude * truth_opt.value().attitude.inverse();
    return 2.0 * std::acos(std::abs(q_error.w())) * 180.0 / M_PI;
}

void TrajectoryAnalyzer::computeTrackErrors(const TrajectoryPoint& estimated,
                                           const GroundTruth& truth,
                                           double& along_track,
                                           double& cross_track,
                                           double& vertical) const {
    Vector3d error = estimated.position - truth.position;

    // Get track direction from truth velocity
    Vector3d track_dir = truth.velocity.normalized();

    // Along-track error (projection onto track)
    along_track = error.dot(track_dir);

    // Cross-track error (perpendicular to track in horizontal plane)
    Vector3d horizontal_error = error;
    horizontal_error.z() = 0;
    Vector3d cross_track_vec = horizontal_error - along_track * track_dir;
    cross_track = cross_track_vec.norm();

    // Vertical error
    vertical = std::abs(error.z());
}

double TrajectoryAnalyzer::computeConsistencyScore() const {
    if (estimated_trajectory_.empty()) return 0;

    int consistent_count = 0;
    int total_count = 0;

    for (const auto& point : estimated_trajectory_) {
        if (isConsistent(point.nees, point.nis)) {
            consistent_count++;
        }
        total_count++;
    }

    return static_cast<double>(consistent_count) / total_count;
}

bool TrajectoryAnalyzer::isConsistent(double nees, double nis) const {
    // Chi-squared bounds for 95% confidence
    // For 9-DOF state: chi2(9, 0.025) = 2.7, chi2(9, 0.975) = 19.02
    bool nees_ok = (nees > 2.7 && nees < 19.02);

    // For typical measurement dimension
    bool nis_ok = (nis > 0.35 && nis < 7.81);

    return nees_ok && nis_ok;
}

bool TrajectoryAnalyzer::detectOutlier(const TrajectoryPoint& point) const {
    if (estimated_trajectory_.size() < 10) return false;

    // Compute recent statistics
    std::vector<double> recent_positions;
    size_t n = std::min(size_t(20), estimated_trajectory_.size());

    for (size_t i = estimated_trajectory_.size() - n; i < estimated_trajectory_.size(); ++i) {
        recent_positions.push_back(estimated_trajectory_[i].position.norm());
    }

    double mean = computeMean(recent_positions);
    double std = computeStd(recent_positions);

    // Check if current point is outlier
    double z_score = std::abs(point.position.norm() - mean) / (std + 1e-6);

    return z_score > outlier_threshold_;
}

bool TrajectoryAnalyzer::detectReset(const TrajectoryPoint& current,
                                    const TrajectoryPoint& previous) const {
    // Detect sudden position jumps
    double position_jump = (current.position - previous.position).norm();
    double dt = current.timestamp - previous.timestamp;

    if (dt < 0.001) return false;  // Too close in time

    // Expected maximum displacement based on velocity
    double expected_displacement = previous.velocity.norm() * dt + 0.5 * 9.81 * dt * dt;
    double max_expected = expected_displacement * 3.0;  // 3x margin

    return position_jump > reset_detection_threshold_ ||
           position_jump > max_expected;
}

double TrajectoryAnalyzer::computeSmoothness() const {
    if (estimated_trajectory_.size() < 3) return 0;

    double total_jerk = 0;
    int count = 0;

    for (size_t i = 2; i < estimated_trajectory_.size(); ++i) {
        double jerk = computeJerk(i);
        total_jerk += jerk * jerk;
        count++;
    }

    return std::sqrt(total_jerk / count);
}

double TrajectoryAnalyzer::computeJerk(size_t index) const {
    if (index < 2 || index >= estimated_trajectory_.size()) return 0;

    const auto& p0 = estimated_trajectory_[index - 2];
    const auto& p1 = estimated_trajectory_[index - 1];
    const auto& p2 = estimated_trajectory_[index];

    double dt1 = p1.timestamp - p0.timestamp;
    double dt2 = p2.timestamp - p1.timestamp;

    if (dt1 < 0.001 || dt2 < 0.001) return 0;

    // Compute accelerations
    Vector3d acc1 = (p1.velocity - p0.velocity) / dt1;
    Vector3d acc2 = (p2.velocity - p1.velocity) / dt2;

    // Compute jerk
    Vector3d jerk = (acc2 - acc1) / ((dt1 + dt2) / 2.0);

    return jerk.norm();
}

void TrajectoryAnalyzer::exportTrajectory(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        {

            std::stringstream msg;

            msg << "Cannot open file for export: " << filename;

            LOG_ERROR(msg.str());

        }
        return;
    }

    // Header
    file << "timestamp,x_est,y_est,z_est,x_truth,y_truth,z_truth,"
         << "error_x,error_y,error_z,error_mag\n";

    for (const auto& point : estimated_trajectory_) {
        auto truth_opt = interpolateTruth(point.timestamp);
        if (!truth_opt.has_value()) continue;

        const auto& truth = truth_opt.value();
        Vector3d error = point.position - truth.position;

        file << std::fixed << std::setprecision(6)
             << point.timestamp << ","
             << point.position.x() << "," << point.position.y() << "," << point.position.z() << ","
             << truth.position.x() << "," << truth.position.y() << "," << truth.position.z() << ","
             << error.x() << "," << error.y() << "," << error.z() << ","
             << error.norm() << "\n";
    }

    {


        std::stringstream msg;


        msg << "Trajectory exported to: " << filename;


        LOG_INFO(msg.str());


    }
}

void TrajectoryAnalyzer::exportMetrics(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        {

            std::stringstream msg;

            msg << "Cannot open file for export: " << filename;

            LOG_ERROR(msg.str());

        }
        return;
    }

    const auto& m = cumulative_metrics_;

    file << "Trajectory Analysis Report\n";
    file << "==========================\n\n";

    file << "Position Errors:\n";
    file << "  Mean: " << m.mean_position_error << " m\n";
    file << "  RMS: " << m.rms_position_error << " m\n";
    file << "  Max: " << m.max_position_error << " m\n";
    file << "  Final: " << m.final_position_error << " m\n\n";

    file << "Velocity Errors:\n";
    file << "  Mean: " << m.mean_velocity_error << " m/s\n";
    file << "  RMS: " << m.rms_velocity_error << " m/s\n";
    file << "  Max: " << m.max_velocity_error << " m/s\n\n";

    file << "Attitude Errors:\n";
    file << "  Mean: " << m.mean_attitude_error << " deg\n";
    file << "  RMS: " << m.rms_attitude_error << " deg\n";
    file << "  Max: " << m.max_attitude_error << " deg\n\n";

    file << "Track Errors:\n";
    file << "  Mean Along-track: " << m.mean_along_track_error << " m\n";
    file << "  Mean Cross-track: " << m.mean_cross_track_error << " m\n";
    file << "  Mean Vertical: " << m.mean_vertical_error << " m\n\n";

    file << "Consistency:\n";
    file << "  Average NEES: " << m.average_nees << "\n";
    file << "  Average NIS: " << m.average_nis << "\n";
    file << "  Consistency Score: " << m.consistency_score << "\n\n";

    file << "Performance:\n";
    file << "  Total Distance: " << m.total_distance_traveled << " m\n";
    file << "  Total Outliers: " << m.total_outliers << "\n";
    file << "  Total Resets: " << m.total_resets << "\n";

    {


        std::stringstream msg;


        msg << "Metrics exported to: " << filename;


        LOG_INFO(msg.str());


    }
}

void TrajectoryAnalyzer::exportKML(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        {

            std::stringstream msg;

            msg << "Cannot open file for KML export: " << filename;

            LOG_ERROR(msg.str());

        }
        return;
    }

    // KML header
    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n";
    file << "<Document>\n";
    file << "  <name>Navigation Trajectory</name>\n";

    // Style for estimated trajectory
    file << "  <Style id=\"estimated\">\n";
    file << "    <LineStyle>\n";
    file << "      <color>ff0000ff</color>\n";  // Red
    file << "      <width>2</width>\n";
    file << "    </LineStyle>\n";
    file << "  </Style>\n";

    // Style for ground truth
    file << "  <Style id=\"truth\">\n";
    file << "    <LineStyle>\n";
    file << "      <color>ff00ff00</color>\n";  // Green
    file << "      <width>2</width>\n";
    file << "    </LineStyle>\n";
    file << "  </Style>\n";

    // Estimated trajectory
    file << "  <Placemark>\n";
    file << "    <name>Estimated</name>\n";
    file << "    <styleUrl>#estimated</styleUrl>\n";
    file << "    <LineString>\n";
    file << "      <coordinates>\n";

    for (const auto& point : estimated_trajectory_) {
        // Convert to lat/lon/alt (simplified)
        double lat = point.position.x() / 111111.0;
        double lon = point.position.y() / (111111.0 * cos(lat * M_PI / 180));
        double alt = -point.position.z();

        file << "        " << lon << "," << lat << "," << alt << "\n";
    }

    file << "      </coordinates>\n";
    file << "    </LineString>\n";
    file << "  </Placemark>\n";

    // Ground truth trajectory
    if (!truth_trajectory_.empty()) {
        file << "  <Placemark>\n";
        file << "    <name>Ground Truth</name>\n";
        file << "    <styleUrl>#truth</styleUrl>\n";
        file << "    <LineString>\n";
        file << "      <coordinates>\n";

        for (const auto& truth : truth_trajectory_) {
            double lat = truth.position.x() / 111111.0;
            double lon = truth.position.y() / (111111.0 * cos(lat * M_PI / 180));
            double alt = -truth.position.z();

            file << "        " << lon << "," << lat << "," << alt << "\n";
        }

        file << "      </coordinates>\n";
        file << "    </LineString>\n";
        file << "  </Placemark>\n";
    }

    file << "</Document>\n";
    file << "</kml>\n";

    {


        std::stringstream msg;


        msg << "KML exported to: " << filename;


        LOG_INFO(msg.str());


    }
}

std::vector<Vector3d> TrajectoryAnalyzer::getEstimatedPath() const {
    std::vector<Vector3d> path;
    path.reserve(estimated_trajectory_.size());

    for (const auto& point : estimated_trajectory_) {
        path.push_back(point.position);
    }

    return path;
}

std::vector<Vector3d> TrajectoryAnalyzer::getTruthPath() const {
    std::vector<Vector3d> path;
    path.reserve(truth_trajectory_.size());

    for (const auto& truth : truth_trajectory_) {
        path.push_back(truth.position);
    }

    return path;
}

std::vector<double> TrajectoryAnalyzer::getErrorMagnitudes() const {
    std::vector<double> errors;

    for (const auto& point : estimated_trajectory_) {
        auto truth_opt = interpolateTruth(point.timestamp);
        if (!truth_opt.has_value()) continue;

        double error = (point.position - truth_opt.value().position).norm();
        errors.push_back(error);
    }

    return errors;
}

void TrajectoryAnalyzer::updateMetrics(const TrajectoryPoint& estimated,
                                      const GroundTruth& truth) {
    // This is called for each new point
    // Update cumulative metrics incrementally

    Vector3d pos_error = estimated.position - truth.position;
    double pos_error_mag = pos_error.norm();

    // Update cumulative average (running average)
    static int count = 0;
    count++;

    cumulative_metrics_.mean_position_error =
        (cumulative_metrics_.mean_position_error * (count - 1) + pos_error_mag) / count;

    // Update RMS
    cumulative_metrics_.rms_position_error =
        std::sqrt((cumulative_metrics_.rms_position_error * cumulative_metrics_.rms_position_error * (count - 1) +
                  pos_error_mag * pos_error_mag) / count);

    // Update max
    cumulative_metrics_.max_position_error =
        std::max(cumulative_metrics_.max_position_error, pos_error_mag);

    // Update final
    cumulative_metrics_.final_position_error = pos_error_mag;

    // Similar updates for velocity and attitude...
}

void TrajectoryAnalyzer::updateWindowedStats(const TrajectoryPoint& estimated,
                                            const GroundTruth& truth) {
    Vector3d pos_error = estimated.position - truth.position;

    for (auto& [window, stats] : windowed_stats_) {
        // Remove old data outside window
        double cutoff_time = estimated.timestamp - window;
        auto it = std::remove_if(stats.timestamps.begin(), stats.timestamps.end(),
                                [cutoff_time](double t) { return t < cutoff_time; });

        if (it != stats.timestamps.end()) {
            size_t removed = std::distance(it, stats.timestamps.end());
            stats.timestamps.erase(it, stats.timestamps.end());
            stats.position_errors.erase(stats.position_errors.end() - removed,
                                       stats.position_errors.end());
            // Remove from other vectors too...
        }

        // Add new data
        stats.timestamps.push_back(estimated.timestamp);
        stats.position_errors.push_back(pos_error.norm());
        stats.nees_values.push_back(estimated.nees);
        stats.nis_values.push_back(estimated.nis);
    }
}

std::optional<GroundTruth> TrajectoryAnalyzer::interpolateTruth(double timestamp) const {
    if (truth_trajectory_.empty()) {
        return std::nullopt;
    }

    // Find bounding truth points
    auto it = std::lower_bound(truth_trajectory_.begin(), truth_trajectory_.end(),
                              timestamp,
                              [](const GroundTruth& t, double time) {
                                  return t.timestamp < time;
                              });

    if (it == truth_trajectory_.end()) {
        // Use last point if beyond range
        return truth_trajectory_.back();
    }

    if (it == truth_trajectory_.begin()) {
        // Use first point if before range
        return *it;
    }

    // Interpolate between previous and current
    auto prev = std::prev(it);
    double dt = it->timestamp - prev->timestamp;

    if (dt < 1e-6) {
        return *it;
    }

    double alpha = (timestamp - prev->timestamp) / dt;

    GroundTruth interpolated;
    interpolated.timestamp = timestamp;
    interpolated.position = prev->position + alpha * (it->position - prev->position);
    interpolated.velocity = prev->velocity + alpha * (it->velocity - prev->velocity);
    interpolated.attitude = prev->attitude.slerp(alpha, it->attitude);
    interpolated.valid = true;

    return interpolated;
}

double TrajectoryAnalyzer::computeMean(const std::vector<double>& values) const {
    if (values.empty()) return 0;
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

double TrajectoryAnalyzer::computeRMS(const std::vector<double>& values) const {
    if (values.empty()) return 0;

    double sum_squares = 0;
    for (double v : values) {
        sum_squares += v * v;
    }

    return std::sqrt(sum_squares / values.size());
}

double TrajectoryAnalyzer::computeMax(const std::vector<double>& values) const {
    if (values.empty()) return 0;
    return *std::max_element(values.begin(), values.end());
}

double TrajectoryAnalyzer::computeStd(const std::vector<double>& values) const {
    if (values.size() < 2) return 0;

    double mean = computeMean(values);
    double sum_squares = 0;

    for (double v : values) {
        sum_squares += (v - mean) * (v - mean);
    }

    return std::sqrt(sum_squares / (values.size() - 1));
}

void TrajectoryAnalyzer::logTrajectoryPoint(const TrajectoryPoint& point) {
    // Convert quaternion to Euler angles for logging
    Matrix3d R = point.attitude.toRotationMatrix();
    Vector3d euler = R.eulerAngles(0, 1, 2) * 180.0 / M_PI;

    trajectory_file_ << std::fixed << std::setprecision(6)
                    << point.timestamp << ","
                    << point.position.x() << "," << point.position.y() << "," << point.position.z() << ","
                    << point.velocity.x() << "," << point.velocity.y() << "," << point.velocity.z() << ","
                    << euler.x() << "," << euler.y() << "," << euler.z() << ","
                    << point.position_covariance(0,0) << ","
                    << point.position_covariance(1,1) << ","
                    << point.position_covariance(2,2) << ","
                    << point.confidence << ","
                    << static_cast<int>(point.filter_mode) << ","
                    << point.nees << "," << point.nis << "\n";
}

void TrajectoryAnalyzer::logMetrics(const TrajectoryMetrics& metrics) {
    metrics_file_ << std::fixed << std::setprecision(6)
                 << estimated_trajectory_.back().timestamp << ","
                 << metrics.mean_position_error << ","
                 << metrics.mean_velocity_error << ","
                 << metrics.mean_attitude_error << ","
                 << metrics.mean_along_track_error << ","
                 << metrics.mean_cross_track_error << ","
                 << metrics.mean_vertical_error << ","
                 << metrics.average_nees << ","
                 << metrics.average_nis << ","
                 << metrics.consistency_score << "\n";
}

void TrajectoryAnalyzer::logError(double timestamp, const Vector3d& error) {
    error_file_ << std::fixed << std::setprecision(6)
               << timestamp << ","
               << error.x() << "," << error.y() << "," << error.z() << ","
               << error.norm() << "\n";
}

/**
 * Filter Debugger - Extended implementation for debugging
 */
FilterDebugger::FilterDebugger(const std::string& debug_file)
    : TrajectoryAnalyzer(true) {

    debug_log_.open(debug_file);
    if (debug_log_.is_open()) {
        debug_log_ << "timestamp,event_type,description,parameters\n";
    }
}

void FilterDebugger::logReset(double timestamp, const Vector3d& reset_position) {
    FilterEvent event;
    event.timestamp = timestamp;
    event.event_type = "reset";
    event.description = "Hard reset executed";
    event.parameters["pos_x"] = reset_position.x();
    event.parameters["pos_y"] = reset_position.y();
    event.parameters["pos_z"] = reset_position.z();

    events_.push_back(event);

    if (debug_log_.is_open()) {
        debug_log_ << timestamp << ",reset,\"" << event.description << "\",\""
                  << "pos=[" << reset_position.transpose() << "]\"\n";
    }
}

void FilterDebugger::logModeChange(double timestamp, FilterMode from, FilterMode to) {
    FilterEvent event;
    event.timestamp = timestamp;
    event.event_type = "mode_change";
    event.description = "Filter mode transition";
    event.parameters["from_mode"] = static_cast<double>(from);
    event.parameters["to_mode"] = static_cast<double>(to);

    events_.push_back(event);

    if (debug_log_.is_open()) {
        debug_log_ << timestamp << ",mode_change,\""
                  << "from=" << static_cast<int>(from)
                  << " to=" << static_cast<int>(to) << "\",\"\"\n";
    }
}

void FilterDebugger::generateReport(const std::string& filename) {
    std::ofstream report(filename);
    if (!report.is_open()) {
        {

            std::stringstream msg;

            msg << "Cannot open debug report file: " << filename;

            LOG_ERROR(msg.str());

        }
        return;
    }

    report << "Filter Debug Report\n";
    report << "===================\n\n";

    // Count events by type
    std::map<std::string, int> event_counts;
    for (const auto& event : events_) {
        event_counts[event.event_type]++;
    }

    report << "Event Summary:\n";
    for (const auto& [type, count] : event_counts) {
        report << "  " << type << ": " << count << "\n";
    }

    report << "\nDetailed Events:\n";
    for (const auto& event : events_) {
        report << "  [" << event.timestamp << "] " << event.event_type
              << ": " << event.description << "\n";
    }

    {


        std::stringstream msg;


        msg << "Debug report generated: " << filename;


        LOG_INFO(msg.str());


    }
}

} // namespace Navigation