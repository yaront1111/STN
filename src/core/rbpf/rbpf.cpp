/**
 * Rao-Blackwellized Particle Filter Implementation
 * Complete implementation with map matching
 */

#include "rbpf.h"
#include <algorithm>
#ifdef __cpp_lib_execution
#include <execution>
#endif
#include <numeric>
#include <cmath>
#include <sstream>
#include "../../utils/logger.h"
#include "../../utils/math_utils.h"
#include "../../maps/xgm2019e_map.h"
#include "../../maps/srtm_terrain.h"
#include "../../maps/composite_map_manager.h"

namespace Navigation {

// RBPFConfig YAML constructor
RBPFConfig::RBPFConfig(const YAML::Node& config) {
    // Particle filter parameters
    num_particles = config["num_particles"].as<int>(1000);
    effective_sample_size_threshold = config["effective_sample_size_threshold"].as<double>(0.5);

    // Position sampling
    position_noise_std = config["position_noise_std"].as<double>(10.0);
    altitude_noise_std = config["altitude_noise_std"].as<double>(5.0);

    // Map matching parameters
    gravity_weight = config["gravity_weight"].as<double>(0.6);
    terrain_weight = config["terrain_weight"].as<double>(0.4);
    correlation_threshold = config["correlation_threshold"].as<double>(0.7);

    // Grid refinement
    use_adaptive_sampling = config["use_adaptive_sampling"].as<bool>(true);
    grid_resolution = config["grid_resolution"].as<double>(100.0);
    refinement_iterations = config["refinement_iterations"].as<int>(3);

    // Robust weighting
    outlier_threshold = config["outlier_threshold"].as<double>(3.0);
    use_kernel_smoothing = config["use_kernel_smoothing"].as<bool>(true);
    kernel_bandwidth = config["kernel_bandwidth"].as<double>(50.0);

    // Performance
    parallel_evaluation = config["parallel_evaluation"].as<bool>(true);
    num_threads = config["num_threads"].as<int>(4);

    // Resampling
    std::string resampling_str = config["resampling_method"].as<std::string>("SYSTEMATIC");
    if (resampling_str == "SYSTEMATIC") resampling_method = SYSTEMATIC;
    else if (resampling_str == "STRATIFIED") resampling_method = STRATIFIED;
    else if (resampling_str == "RESIDUAL") resampling_method = RESIDUAL;
    else if (resampling_str == "MULTINOMIAL") resampling_method = MULTINOMIAL;
    else resampling_method = SYSTEMATIC;

    // Deterministic mode for testing
    deterministic = config["deterministic"].as<bool>(false);
    random_seed = config["random_seed"].as<uint32_t>(42);
}

// Thread pool for parallel particle evaluation
struct RBPF::ThreadPool {
    // Simplified thread pool - in production use std::thread pool library
    int num_threads;
    ThreadPool(int n) : num_threads(n) {}
};

RBPF::RBPF(const RBPFConfig& config) : config_(config) {
    // Initialize random number generator
    if (config_.deterministic) {
        rng_.seed(config_.random_seed);
    } else {
        std::random_device rd;
        rng_.seed(rd());
    }
    
    normal_dist_ = std::normal_distribution<double>(0.0, 1.0);
    uniform_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);
    
    // Allocate particles
    particles_.resize(config_.num_particles);
    particles_buffer_.resize(config_.num_particles);
    
    // Initialize particle IDs
    for (int i = 0; i < config_.num_particles; ++i) {
        particles_[i].id = i;
    }
    
    // Create thread pool if parallel evaluation enabled
    if (config_.parallel_evaluation) {
        thread_pool_ = std::make_unique<ThreadPool>(config_.num_threads);
    }
    
    // Initialize grid
    grid_dimensions_ = Vector3i(100, 100, 20);  // 100x100x20 grid
    grid_.resize(grid_dimensions_.x() * grid_dimensions_.y() * grid_dimensions_.z());
    
    {
        std::stringstream msg;
        msg << "RBPF initialized with " << config_.num_particles << " particles";
        LOG_INFO(msg.str());
    }
}

// YAML Constructor
RBPF::RBPF(const YAML::Node& config)
    : RBPF(RBPFConfig(config)) {
    // Delegate to main constructor via RBPFConfig
}

RBPF::~RBPF() = default;

void RBPF::setMaps(std::shared_ptr<MapManager> maps) {
    if (!maps) return;

    // Try to cast to CompositeMapManager to get individual maps
    if (dynamic_cast<CompositeMapManager*>(maps.get())) {
        // Create shared_ptrs from raw pointers (be careful with ownership)
        // Since CompositeMapManager owns these with unique_ptr, we can't directly share
        // Instead, we'll need to modify CompositeMapManager or use a different approach
        // For now, we'll leave the maps unset if they're not already shared_ptr
        LOG_INFO("RBPF: Maps provided but cannot extract gravity/terrain components");
    }
}

void RBPF::initialize(const StateVector& initial_state, const MatrixXd& initial_cov) {
    initializeParticles(initial_state, initial_cov);
    
    // Set grid origin
    grid_origin_ = initial_state.position - Vector3d(5000, 5000, 1000);  // 5km x 5km x 1km grid
    
    // Initialize grid cells
    updateGrid();
    
    // Reset statistics
    stats_ = FilterStats();
    computeStatistics();
    
    last_update_time_ = 0.0;
    
    {
        std::stringstream msg;
        msg << "RBPF initialized at position: " << initial_state.position.transpose();
        LOG_INFO(msg.str());
    }
    logParticleDistribution();
}

void RBPF::initializeParticles(const StateVector& state, const MatrixXd& cov) {
    // Extract position covariance
    Matrix3d pos_cov = cov.block(0, 0, 3, 3);
    Eigen::LLT<Matrix3d> llt(pos_cov);
    Matrix3d L = llt.matrixL();
    
    // Initialize analytical covariance (velocity, attitude, biases)
    MatrixXd P_analytical = MatrixXd::Zero(17, 17);
    P_analytical.block(0, 0, 3, 3) = cov.block(3, 3, 3, 3);    // Velocity
    P_analytical.block(3, 3, 3, 3) = cov.block(6, 6, 3, 3);    // Attitude
    P_analytical.block(6, 6, 3, 3) = cov.block(9, 9, 3, 3);    // Accel bias
    P_analytical.block(9, 9, 3, 3) = cov.block(12, 12, 3, 3);  // Gyro bias
    P_analytical.block(12, 12, 5, 5) = cov.block(15, 15, 5, 5); // Gravity bias
    
    for (int i = 0; i < config_.num_particles; ++i) {
        auto& p = particles_[i];
        
        // Sample position from multivariate normal
        Vector3d noise;
        noise.x() = normal_dist_(rng_);
        noise.y() = normal_dist_(rng_);
        noise.z() = normal_dist_(rng_);
        p.position = state.position + L * noise;
        
        // Analytical states (same for all particles initially)
        p.velocity = state.velocity;
        p.quaternion = state.quaternion;
        p.accel_bias = state.accel_bias;
        p.gyro_bias = state.gyro_bias;
        p.gravity_bias = state.gravity_bias;
        
        // Analytical covariance
        p.P = P_analytical;
        
        // Equal weights initially
        p.weight = 1.0 / config_.num_particles;
        p.log_weight = log(p.weight);
        
        // Initialize scores
        p.gravity_score = 0.0;
        p.terrain_score = 0.0;
        p.total_score = 0.0;
        
        // Clear history
        p.position_history.clear();
        p.position_history.push_back(p.position);
    }
}

void RBPF::predict(const Vector3d& accel, const Vector3d& gyro, double dt) {
    // Propagate all particles
    propagateParticles(accel, gyro, dt);
    
    // Update grid organization
    updateGrid();
    
    // Adaptive sampling if enabled
    if (config_.use_adaptive_sampling) {
        adaptiveSampling();
    }
    
    // Update statistics
    computeStatistics();
    
    {
        std::stringstream msg;
        msg << "RBPF prediction complete. ESS: " << stats_.effective_sample_size;
        LOG_DEBUG(msg.str());
    }
}

void RBPF::propagateParticles(const Vector3d& accel, const Vector3d& gyro, double dt) {
    // Process particles (parallel if enabled)
    auto process_particle = [&](RBParticle& p) {
        // Update analytical states (velocity, attitude, biases)
        updateAnalyticalStates(p, accel, gyro, dt);
        
        // Propagate position with noise
        Vector3d vel_noise;
        vel_noise.x() = normal_dist_(rng_) * config_.position_noise_std * sqrt(dt);
        vel_noise.y() = normal_dist_(rng_) * config_.position_noise_std * sqrt(dt);
        vel_noise.z() = normal_dist_(rng_) * config_.altitude_noise_std * sqrt(dt);
        
        p.position += p.velocity * dt + vel_noise;
        
        // Add to history (keep last 10 positions)
        p.position_history.push_back(p.position);
        if (p.position_history.size() > 10) {
            p.position_history.erase(p.position_history.begin());
        }
    };
    
    // Execute (parallel or serial)
#ifdef __cpp_lib_execution
    if (config_.parallel_evaluation) {
        std::for_each(std::execution::par_unseq,
                     particles_.begin(), particles_.end(),
                     process_particle);
    } else {
        std::for_each(particles_.begin(), particles_.end(), process_particle);
    }
#else
    std::for_each(particles_.begin(), particles_.end(), process_particle);
#endif
}

void RBPF::updateAnalyticalStates(RBParticle& particle, const Vector3d& accel,
                                 const Vector3d& gyro, double dt) {
    // Simplified analytical update (full Kalman filter for non-position states)
    
    // Remove biases
    Vector3d accel_corrected = accel - particle.accel_bias;
    Vector3d gyro_corrected = gyro - particle.gyro_bias;
    
    // Update attitude
    particle.quaternion = SO3::integrateQuaternion(particle.quaternion, gyro_corrected, dt);
    
    // Update velocity (simplified - no Coriolis for now)
    Matrix3d C_bn = particle.quaternion.toRotationMatrix();
    Vector3d gravity(0, 0, GRAVITY_NOM);
    particle.velocity += (C_bn * accel_corrected + gravity) * dt;
    
    // Process noise for analytical covariance
    MatrixXd Q = MatrixXd::Zero(17, 17);
    Q.block(0, 0, 3, 3) = Matrix3d::Identity() * 0.1 * dt;     // Velocity
    Q.block(3, 3, 3, 3) = Matrix3d::Identity() * 1e-6 * dt;    // Attitude
    Q.block(6, 6, 3, 3) = Matrix3d::Identity() * 1e-8 * dt;    // Accel bias
    Q.block(9, 9, 3, 3) = Matrix3d::Identity() * 1e-10 * dt;   // Gyro bias
    Q.block(12, 12, 5, 5) = MatrixXd::Identity(5,5) * 1e-12 * dt; // Gravity bias
    
    // Simple prediction step for covariance
    // F matrix would be state transition matrix (simplified here)
    particle.P += Q;
    
    // Ensure positive definite
    particle.P = MatrixUtils::makePositiveDefinite(particle.P);
}

void RBPF::updateGravity(const Eigen::Matrix<double, 5, 1>& measurement) {
    // Map-free gravity update using relative measurements
    if (!gravity_map_) {
        // Use relative gravity gradient for particle weighting
        updateParticleWeightsMapFree(measurement);
    } else {
        // Original map-based update
        updateParticleWeights(measurement, 0.0);
    }

    // Normalize weights
    normalizeWeights();

    // Resample if needed
    if (needsResampling()) {
        resample();
    }

    // Update statistics
    computeStatistics();

    {
        std::stringstream msg;
        msg << "Gravity update complete. Max weight: " << stats_.max_weight
            << ", ESS: " << stats_.effective_sample_size;
        LOG_DEBUG(msg.str());
    }
    if (gravity_map_) {
        logMapCorrelation();
    }
}

void RBPF::updateTerrain(double altitude) {
    if (!terrain_map_) {
        LOG_WARN("Terrain map not set - skipping terrain update");
        return;
    }
    
    // Update weights based on terrain correlation
    for (auto& p : particles_) {
        p.terrain_score = computeTerrainLikelihood(p, altitude);
        p.total_score = config_.gravity_weight * p.gravity_score + 
                       config_.terrain_weight * p.terrain_score;
        
        // Update weight
        p.log_weight += p.terrain_score;
        p.weight = exp(p.log_weight);
    }
    
    // Normalize and resample if needed
    normalizeWeights();
    if (needsResampling()) {
        resample();
    }
    
    computeStatistics();
    {
        std::stringstream msg;
        msg << "Terrain update complete. Correlation: " << stats_.terrain_correlation;
        LOG_DEBUG(msg.str());
    }
}

void RBPF::updateBarometer(double pressure, double temperature) {
    // Use centralized atmospheric model for consistency
    double altitude = NavMath::AtmosphericModel::pressureToAltitude(pressure);
    
    // Update particle weights based on altitude consistency
    for (auto& p : particles_) {
        double predicted_alt = -p.position.z();  // NED to altitude
        double innovation = altitude - predicted_alt;
        double variance = 25.0;  // 5m std dev for barometer
        
        // Gaussian likelihood
        double likelihood = exp(-0.5 * innovation * innovation / variance) / sqrt(2 * M_PI * variance);
        p.log_weight += log(likelihood + 1e-10);
        p.weight = exp(p.log_weight);
    }
    
    normalizeWeights();
    if (needsResampling()) {
        resample();
    }
    
    LOG_DEBUG("Barometer update complete");
}

void RBPF::updateParticleWeights(const Eigen::Matrix<double, 5, 1>& gravity_meas,
                                double terrain_alt) {
    // Compute likelihood for each particle
    auto compute_weight = [&](RBParticle& p) {
        // Gravity likelihood
        if (gravity_map_) {
            p.gravity_score = computeGravityLikelihood(p, gravity_meas);
        }
        
        // Terrain likelihood
        if (terrain_map_ && terrain_alt > 0) {
            p.terrain_score = computeTerrainLikelihood(p, terrain_alt);
        }
        
        // Combined score
        p.total_score = config_.gravity_weight * p.gravity_score + 
                       config_.terrain_weight * p.terrain_score;
        
        // Update weight (log domain for stability)
        p.log_weight += p.total_score;
        p.weight = exp(p.log_weight);
    };
    
    // Execute (parallel or serial)
#ifdef __cpp_lib_execution
    if (config_.parallel_evaluation) {
        std::for_each(std::execution::par_unseq,
                     particles_.begin(), particles_.end(),
                     compute_weight);
    } else {
        std::for_each(particles_.begin(), particles_.end(), compute_weight);
    }
#else
    std::for_each(particles_.begin(), particles_.end(), compute_weight);
#endif
}

void RBPF::updateParticleWeightsMapFree(const Eigen::Matrix<double, 5, 1>& gravity_meas) {
    // Map-free update using relative gravity measurements
    // Weight particles based on measurement consistency

    LOG_DEBUG("Map-free gravity update with measurement: [" +
              std::to_string(gravity_meas(0)) + ", " + std::to_string(gravity_meas(1)) + ", " +
              std::to_string(gravity_meas(2)) + ", " + std::to_string(gravity_meas(3)) + ", " +
              std::to_string(gravity_meas(4)) + "]");

    // Store measurement history
    static std::vector<Eigen::Matrix<double, 5, 1>> measurement_history;
    measurement_history.push_back(gravity_meas);

    // Need at least 2 measurements for gradient estimation
    if (measurement_history.size() < 2) {
        LOG_DEBUG("Insufficient measurement history for map-free update");
        return;
    }

    // Keep only recent measurements
    if (measurement_history.size() > 10) {
        measurement_history.erase(measurement_history.begin());
    }

    // Compute measurement statistics
    Eigen::Matrix<double, 5, 1> mean_gradient = Eigen::Matrix<double, 5, 1>::Zero();
    for (const auto& m : measurement_history) {
        mean_gradient += m;
    }
    mean_gradient /= measurement_history.size();

    // Compute variance
    double variance = 0;
    for (const auto& m : measurement_history) {
        variance += (m - mean_gradient).squaredNorm();
    }
    variance /= measurement_history.size();
    variance = std::max(variance, 1.0);  // Minimum variance

    LOG_DEBUG("Measurement statistics: mean=[" +
              std::to_string(mean_gradient(0)) + ", " + std::to_string(mean_gradient(1)) + ", " +
              std::to_string(mean_gradient(2)) + ", " + std::to_string(mean_gradient(3)) + ", " +
              std::to_string(mean_gradient(4)) + "], variance=" + std::to_string(variance));

    // Update particle weights based on distance from mean gradient
    for (auto& p : particles_) {
        // Add position-based variation to expected gradient
        // Particles at different positions should expect different gradients
        Eigen::Matrix<double, 5, 1> expected = mean_gradient;

        // CRITICAL FIX: Much stronger spatial differentiation for map-free operation
        // Gravity gradients can vary by 0.1-1 E per km, so for 100m particle spread:
        expected(0) += p.position.x() * 0.01;   // 1 E per 100m in x (increased 10x)
        expected(1) += p.position.y() * 0.01;   // 1 E per 100m in y (increased 10x)
        expected(2) += (p.position.x() + p.position.y()) * 0.005;  // Cross terms (increased 10x)

        // Add altitude-dependent variation (gravity decreases with altitude)
        double altitude = -p.position.z();  // Convert NED to altitude
        expected(0) += altitude * 0.0001;   // Altitude effect on Txx
        expected(1) += altitude * 0.0001;   // Altitude effect on Tyy

        // Add some randomness for diversity (helps prevent particle degeneracy)
        static std::mt19937 rng(42);  // Deterministic for testing
        std::normal_distribution<double> noise_dist(0.0, 0.1);
        for (int i = 0; i < 5; ++i) {
            expected(i) += noise_dist(rng);
        }

        // Compute likelihood based on current measurement
        Eigen::Matrix<double, 5, 1> innovation = gravity_meas - expected;

        // CRITICAL FIX: Better variance estimation to prevent numerical issues
        double effective_variance = std::max(variance + 1.0, 5.0);  // Minimum of 5.0 for stability
        double mahal_dist = innovation.squaredNorm() / effective_variance;

        // Convert to log likelihood with stronger influence for map-free mode
        p.gravity_score = -0.5 * mahal_dist;

        // CRITICAL FIX: Stronger weight influence for map-free operation
        // Need to create more discrimination between particles
        double weight_influence = 2.0;  // Increased from 0.5 to create stronger differences
        p.log_weight += p.gravity_score * weight_influence;

        // Apply bounds to prevent extreme weights
        p.log_weight = std::max(p.log_weight, -50.0);  // Prevent underflow
        p.log_weight = std::min(p.log_weight, 10.0);   // Prevent overflow

        p.weight = exp(p.log_weight);
    }

    // Normalize weights and compute statistics for debugging
    normalizeWeights();

    // Calculate weight variance for debugging
    auto weights = getParticleWeights();
    double mean_weight = 1.0 / weights.size();
    double weight_variance = 0;
    for (double w : weights) {
        weight_variance += (w - mean_weight) * (w - mean_weight);
    }
    weight_variance /= weights.size();

    LOG_DEBUG("Map-free update complete. Weight variance: " + std::to_string(weight_variance) +
              ", ESS: " + std::to_string(getEffectiveSampleSize()) +
              ", Max weight: " + std::to_string(*std::max_element(weights.begin(), weights.end())));
}

double RBPF::computeGravityLikelihood(const RBParticle& particle,
                                     const Eigen::Matrix<double, 5, 1>& measurement) {
    // Get predicted gravity gradient at particle position
    auto predicted = gravity_map_->getGradient(particle.position);
    
    // Account for bias
    predicted += particle.gravity_bias;
    
    // Innovation
    Eigen::Matrix<double, 5, 1> innovation = measurement - predicted;
    
    // Innovation covariance (simplified)
    Eigen::Matrix<double, 5, 5> R = Eigen::Matrix<double, 5, 5>::Identity() * 100.0;  // E²
    
    // Mahalanobis distance
    double mahal_dist = innovation.transpose() * R.inverse() * innovation;
    
    // Convert to likelihood (log domain)
    return -0.5 * mahal_dist;
}

double RBPF::computeTerrainLikelihood(const RBParticle& particle, double altitude) {
    // Get terrain elevation at particle position
    double terrain_elev = terrain_map_->getElevationXY(particle.position.x(),
                                                      particle.position.y());
    
    // Predicted altitude
    double predicted_alt = terrain_elev - particle.position.z();
    
    // Innovation
    double innovation = altitude - predicted_alt;
    
    // Variance
    double variance = 100.0;  // 10m std dev
    
    // Log likelihood
    return -0.5 * innovation * innovation / variance;
}

void RBPF::resample() {
    {
        std::stringstream msg;
        msg << "Resampling particles. ESS: " << stats_.effective_sample_size;
        LOG_DEBUG(msg.str());
    }
    
    switch (config_.resampling_method) {
        case RBPFConfig::SYSTEMATIC:
            systematicResampling();
            break;
        case RBPFConfig::STRATIFIED:
            stratifiedResampling();
            break;
        case RBPFConfig::RESIDUAL:
            residualResampling();
            break;
        default:
            systematicResampling();
    }
    
    stats_.resampling_count++;
    
    // Reset weights after resampling
    double uniform_weight = 1.0 / config_.num_particles;
    for (auto& p : particles_) {
        p.weight = uniform_weight;
        p.log_weight = log(uniform_weight);
    }
}

void RBPF::systematicResampling() {
    // Cumulative weights
    std::vector<double> cumsum(config_.num_particles);
    cumsum[0] = particles_[0].weight;
    for (int i = 1; i < config_.num_particles; ++i) {
        cumsum[i] = cumsum[i-1] + particles_[i].weight;
    }
    
    // Systematic resampling
    double step = 1.0 / config_.num_particles;
    double u = uniform_dist_(rng_) * step;
    
    int j = 0;
    for (int i = 0; i < config_.num_particles; ++i) {
        double threshold = u + i * step;
        while (j < config_.num_particles - 1 && cumsum[j] < threshold) {
            j++;
        }
        particles_buffer_[i] = particles_[j];
        particles_buffer_[i].id = i;  // Update ID
    }
    
    // Swap buffers
    particles_.swap(particles_buffer_);
}

void RBPF::stratifiedResampling() {
    // Similar to systematic but with independent random numbers
    std::vector<double> cumsum(config_.num_particles);
    cumsum[0] = particles_[0].weight;
    for (int i = 1; i < config_.num_particles; ++i) {
        cumsum[i] = cumsum[i-1] + particles_[i].weight;
    }
    
    double step = 1.0 / config_.num_particles;
    
    for (int i = 0; i < config_.num_particles; ++i) {
        double u = (i + uniform_dist_(rng_)) * step;
        
        // Binary search for particle
        auto it = std::lower_bound(cumsum.begin(), cumsum.end(), u);
        int idx = std::distance(cumsum.begin(), it);
        
        particles_buffer_[i] = particles_[idx];
        particles_buffer_[i].id = i;
    }
    
    particles_.swap(particles_buffer_);
}

void RBPF::residualResampling() {
    // Residual resampling - deterministic + stochastic
    int idx = 0;
    
    // Deterministic part
    for (int i = 0; i < config_.num_particles; ++i) {
        int copies = static_cast<int>(particles_[i].weight * config_.num_particles);
        for (int j = 0; j < copies; ++j) {
            if (idx < config_.num_particles) {
                particles_buffer_[idx++] = particles_[i];
            }
        }
    }
    
    // Stochastic part for residuals
    if (idx < config_.num_particles) {
        std::vector<double> residuals(config_.num_particles);
        for (int i = 0; i < config_.num_particles; ++i) {
            residuals[i] = particles_[i].weight * config_.num_particles - 
                          floor(particles_[i].weight * config_.num_particles);
        }
        
        // Normalize residuals
        double sum = std::accumulate(residuals.begin(), residuals.end(), 0.0);
        for (auto& r : residuals) {
            r /= sum;
        }
        
        // Sample from residuals
        while (idx < config_.num_particles) {
            double u = uniform_dist_(rng_);
            double cumsum = 0;
            for (int i = 0; i < config_.num_particles; ++i) {
                cumsum += residuals[i];
                if (u <= cumsum) {
                    particles_buffer_[idx++] = particles_[i];
                    break;
                }
            }
        }
    }
    
    particles_.swap(particles_buffer_);
}

void RBPF::normalizeWeights() {
    // Find max log-weight for numerical stability
    double max_log_weight = -std::numeric_limits<double>::infinity();
    for (const auto& p : particles_) {
        max_log_weight = std::max(max_log_weight, p.log_weight);
    }
    
    // Compute weights in stable way
    double sum = 0;
    for (auto& p : particles_) {
        p.weight = exp(p.log_weight - max_log_weight);
        sum += p.weight;
    }
    
    // Normalize
    if (sum > 0) {
        for (auto& p : particles_) {
            p.weight /= sum;
            p.log_weight = log(p.weight + 1e-100);
        }
    }
}

bool RBPF::needsResampling() const {
    return stats_.effective_sample_size < 
           config_.effective_sample_size_threshold * config_.num_particles;
}

double RBPF::getEffectiveSampleSize() const {
    return Statistics::effectiveSampleSize(getParticleWeights());
}

void RBPF::computeStatistics() {
    // Effective sample size
    stats_.effective_sample_size = getEffectiveSampleSize();
    
    // Weight statistics
    stats_.max_weight = 0;
    double sum_weights = 0;
    double sum_weights_sq = 0;
    
    for (const auto& p : particles_) {
        stats_.max_weight = std::max(stats_.max_weight, p.weight);
        sum_weights += p.weight;
        sum_weights_sq += p.weight * p.weight;
    }
    
    stats_.weight_variance = (sum_weights_sq - sum_weights * sum_weights / config_.num_particles) / 
                            (config_.num_particles - 1);
    
    // Mean position and covariance
    stats_.mean_position.setZero();
    for (const auto& p : particles_) {
        stats_.mean_position += p.weight * p.position;
    }
    
    stats_.position_covariance.setZero();
    for (const auto& p : particles_) {
        Vector3d diff = p.position - stats_.mean_position;
        stats_.position_covariance += p.weight * diff * diff.transpose();
    }
    
    // Map correlation statistics
    stats_.gravity_correlation = 0;
    stats_.terrain_correlation = 0;
    for (const auto& p : particles_) {
        stats_.gravity_correlation += p.weight * p.gravity_score;
        stats_.terrain_correlation += p.weight * p.terrain_score;
    }
    
    // Check for particle degeneracy
    stats_.particle_degeneracy_count = 0;
    for (const auto& p : particles_) {
        if (p.weight < 1e-6) {
            stats_.particle_degeneracy_count++;
        }
    }
}

StateVector RBPF::getMMSE() const {
    // Minimum Mean Square Error estimate
    StateVector state;
    
    state.position.setZero();
    state.velocity.setZero();
    state.accel_bias.setZero();
    state.gyro_bias.setZero();
    state.gravity_bias.setZero();
    
    // Weighted average
    for (const auto& p : particles_) {
        state.position += p.weight * p.position;
        state.velocity += p.weight * p.velocity;
        state.accel_bias += p.weight * p.accel_bias;
        state.gyro_bias += p.weight * p.gyro_bias;
        state.gravity_bias += p.weight * p.gravity_bias;
    }
    
    // Quaternion averaging (special handling)
    Matrix4d Q_avg = Matrix4d::Zero();
    for (const auto& p : particles_) {
        Vector4d q_vec;
        q_vec << p.quaternion.w(), p.quaternion.x(), p.quaternion.y(), p.quaternion.z();
        Q_avg += p.weight * q_vec * q_vec.transpose();
    }
    
    Eigen::SelfAdjointEigenSolver<Matrix4d> solver(Q_avg);
    Vector4d q_mean = solver.eigenvectors().col(3);
    state.quaternion = Quaterniond(q_mean(0), q_mean(1), q_mean(2), q_mean(3));
    state.quaternion.normalize();
    
    return state;
}

StateVector RBPF::getMAP() const {
    // Maximum A Posteriori - return state of highest weight particle
    auto max_particle = std::max_element(particles_.begin(), particles_.end(),
                                        [](const RBParticle& a, const RBParticle& b) {
                                            return a.weight < b.weight;
                                        });
    
    StateVector state;
    state.position = max_particle->position;
    state.velocity = max_particle->velocity;
    state.quaternion = max_particle->quaternion;
    state.accel_bias = max_particle->accel_bias;
    state.gyro_bias = max_particle->gyro_bias;
    state.gravity_bias = max_particle->gravity_bias;
    
    return state;
}

MatrixXd RBPF::getCovariance() const {
    MatrixXd cov = MatrixXd::Zero(20, 20);
    
    // Position covariance from particle spread
    cov.block(0, 0, 3, 3) = stats_.position_covariance;
    
    // Analytical covariances (average)
    for (const auto& p : particles_) {
        cov.block(3, 3, 3, 3) += p.weight * p.P.block(0, 0, 3, 3);   // Velocity
        cov.block(6, 6, 3, 3) += p.weight * p.P.block(3, 3, 3, 3);   // Attitude
        cov.block(9, 9, 3, 3) += p.weight * p.P.block(6, 6, 3, 3);   // Accel bias
        cov.block(12, 12, 3, 3) += p.weight * p.P.block(9, 9, 3, 3); // Gyro bias
        cov.block(15, 15, 5, 5) += p.weight * p.P.block(12, 12, 5, 5); // Gravity bias
    }
    
    return cov;
}

std::vector<Vector3d> RBPF::getParticlePositions() const {
    std::vector<Vector3d> positions;
    positions.reserve(particles_.size());
    
    for (const auto& p : particles_) {
        positions.push_back(p.position);
    }
    
    return positions;
}

std::vector<double> RBPF::getParticleWeights() const {
    std::vector<double> weights;
    weights.reserve(particles_.size());
    
    for (const auto& p : particles_) {
        weights.push_back(p.weight);
    }
    
    return weights;
}

void RBPF::injectPositionHypothesis(const Vector3d& position, double confidence) {
    // Inject more particles for higher confidence
    int num_to_inject = static_cast<int>(config_.num_particles * confidence * 0.5);  // Up to 50% of particles
    num_to_inject = std::max(num_to_inject, static_cast<int>(config_.num_particles * 0.1));  // At least 10%

    // Sort particles by weight
    std::sort(particles_.begin(), particles_.end(),
             [](const RBParticle& a, const RBParticle& b) {
                 return a.weight < b.weight;
             });

    // Calculate spread based on confidence (tighter spread for higher confidence)
    double pos_spread = 50.0 * (1.0 - confidence);  // 0-50m spread
    double alt_spread = 25.0 * (1.0 - confidence);  // 0-25m altitude spread

    // Replace lowest weight particles and redistribute around hypothesis
    for (int i = 0; i < num_to_inject; ++i) {
        particles_[i].position = position + Vector3d(normal_dist_(rng_) * pos_spread,
                                                    normal_dist_(rng_) * pos_spread,
                                                    normal_dist_(rng_) * alt_spread);

        // Give injected particles higher initial weight based on confidence
        particles_[i].weight = (1.0 / config_.num_particles) * (1.0 + confidence);
        particles_[i].log_weight = log(particles_[i].weight);

        // Reset particle history
        particles_[i].position_history.clear();
        particles_[i].position_history.push_back(particles_[i].position);
    }

    normalizeWeights();

    // Force immediate resampling to spread the hypothesis
    if (confidence > 0.7) {
        resample();
    }

    {
        std::stringstream msg;
        msg << "Injected " << num_to_inject << " particles at position " << position.transpose()
            << " with spread " << pos_spread << "m, confidence " << confidence;
        LOG_INFO(msg.str());
    }
}

void RBPF::updateGrid() {
    // Clear grid
    for (auto& cell : grid_) {
        cell.particle_indices.clear();
        cell.max_weight = 0;
        cell.total_weight = 0;
    }
    
    // Assign particles to grid cells
    for (int i = 0; i < config_.num_particles; ++i) {
        auto& cell = getGridCell(particles_[i].position);
        cell.particle_indices.push_back(i);
        cell.max_weight = std::max(cell.max_weight, particles_[i].weight);
        cell.total_weight += particles_[i].weight;
    }
}

GridCell& RBPF::getGridCell(const Vector3d& position) {
    // Convert position to grid indices
    Vector3d rel_pos = position - grid_origin_;
    int ix = static_cast<int>(rel_pos.x() / config_.grid_resolution);
    int iy = static_cast<int>(rel_pos.y() / config_.grid_resolution);
    int iz = static_cast<int>(rel_pos.z() / config_.grid_resolution);
    
    // Clamp to grid bounds
    ix = std::max(0, std::min(grid_dimensions_.x() - 1, ix));
    iy = std::max(0, std::min(grid_dimensions_.y() - 1, iy));
    iz = std::max(0, std::min(grid_dimensions_.z() - 1, iz));
    
    int idx = iz * grid_dimensions_.x() * grid_dimensions_.y() + 
              iy * grid_dimensions_.x() + ix;
    
    return grid_[idx];
}

void RBPF::adaptiveSampling() {
    // Identify high-weight regions
    std::vector<GridCell*> high_weight_cells;
    
    for (auto& cell : grid_) {
        if (cell.total_weight > 1.0 / config_.num_particles) {
            high_weight_cells.push_back(&cell);
        }
    }
    
    // Refine sampling in high-weight regions
    if (!high_weight_cells.empty()) {
        refineHighWeightRegions();
    }
}

void RBPF::refineHighWeightRegions() {
    // Find particles with highest weights
    std::vector<int> elite_indices;
    for (int i = 0; i < config_.num_particles; ++i) {
        if (particles_[i].weight > 2.0 / config_.num_particles) {
            elite_indices.push_back(i);
        }
    }
    
    if (elite_indices.empty()) return;
    
    // Replace some low-weight particles with variations of elite particles
    std::sort(particles_.begin(), particles_.end(),
             [](const RBParticle& a, const RBParticle& b) {
                 return a.weight < b.weight;
             });
    
    int num_to_refine = std::min(50, config_.num_particles / 10);
    
    for (int i = 0; i < num_to_refine; ++i) {
        int elite_idx = elite_indices[i % elite_indices.size()];
        
        // Create variation
        particles_[i] = particles_[elite_idx];
        particles_[i].position += Vector3d(normal_dist_(rng_) * config_.grid_resolution * 0.5,
                                          normal_dist_(rng_) * config_.grid_resolution * 0.5,
                                          normal_dist_(rng_) * config_.grid_resolution * 0.2);
        particles_[i].id = i;
    }
}

void RBPF::logParticleDistribution() {
    // Log particle statistics
    {
        std::stringstream msg;
        msg << "Particle distribution - Mean: " << stats_.mean_position.transpose()
            << ", Max weight: " << stats_.max_weight
            << ", ESS: " << stats_.effective_sample_size;
        LOG_DEBUG(msg.str());
    }
    
    // Log covariance eigenvalues
    Eigen::SelfAdjointEigenSolver<Matrix3d> solver(stats_.position_covariance);
    {
        std::stringstream msg;
        msg << "Position uncertainty (m): " << sqrt(solver.eigenvalues().maxCoeff());
        LOG_DEBUG(msg.str());
    }
}

void RBPF::logMapCorrelation() {
    {
        std::stringstream msg;
        msg << "Map correlation - Gravity: " << stats_.gravity_correlation
            << ", Terrain: " << stats_.terrain_correlation;
        LOG_DEBUG(msg.str());
    }
    
    // Warn if correlation is low
    if (stats_.gravity_correlation < config_.correlation_threshold) {
        {
            std::stringstream msg;
            msg << "Low gravity correlation: " << stats_.gravity_correlation;
            LOG_WARN(msg.str());
        }
    }
}

} // namespace Navigation