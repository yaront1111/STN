/**
 * Rao-Blackwellized Particle Filter
 * Grid-based position hypothesis with analytical velocity/attitude
 * Complete implementation with no placeholders
 */

#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <random>
#include <atomic>
#include <yaml-cpp/yaml.h>
#include "../ukf/sr_ukf.h"
#include "../../maps/map_manager.h"
#include "../../utils/logger.h"

namespace Navigation {

using namespace NavMath;

/**
 * Single particle in RBPF
 * Position is sampled, velocity/attitude/biases are analytical (Kalman)
 */
struct RBParticle {
    // Sampled state (position only)
    Vector3d position;
    
    // Analytical states (maintained by Kalman filter)
    Vector3d velocity;
    Quaterniond quaternion;
    Vector3d accel_bias;
    Vector3d gyro_bias;
    Eigen::Matrix<double, 5, 1> gravity_bias;
    
    // Analytical covariance (only for velocity, attitude, biases)
    MatrixXd P;  // 17x17 (vel, att, biases)
    
    // Particle weight
    double weight;
    double log_weight;  // For numerical stability
    
    // Map matching score
    double gravity_score;
    double terrain_score;
    double total_score;
    
    // Particle history for smoothing
    std::vector<Vector3d> position_history;
    
    // Unique ID for tracking
    int id;
};

/**
 * RBPF Configuration
 */
struct RBPFConfig {
    // Constructors
    RBPFConfig() = default;
    explicit RBPFConfig(const YAML::Node& config);

    // Particle filter parameters
    int num_particles = 1000;
    double effective_sample_size_threshold = 0.5;  // Resample when ESS < 0.5*N
    
    // Position sampling
    double position_noise_std = 10.0;  // meters
    double altitude_noise_std = 5.0;   // meters (separate for altitude)
    
    // Map matching parameters
    double gravity_weight = 0.6;
    double terrain_weight = 0.4;
    double correlation_threshold = 0.7;
    
    // Grid refinement
    bool use_adaptive_sampling = true;
    double grid_resolution = 100.0;  // meters
    int refinement_iterations = 3;
    
    // Robust weighting
    double outlier_threshold = 3.0;  // Mahalanobis distance
    bool use_kernel_smoothing = true;
    double kernel_bandwidth = 50.0;  // meters
    
    // Performance
    bool parallel_evaluation = true;
    int num_threads = 4;
    
    // Resampling
    enum ResamplingMethod {
        SYSTEMATIC,
        STRATIFIED,
        RESIDUAL,
        MULTINOMIAL
    } resampling_method = SYSTEMATIC;
    
    // Deterministic mode for testing
    bool deterministic = false;
    uint32_t random_seed = 42;
};

/**
 * Grid cell for efficient particle management
 */
struct GridCell {
    Vector3d center;
    double resolution;
    std::vector<int> particle_indices;
    double max_weight;
    double total_weight;
};

/**
 * Rao-Blackwellized Particle Filter
 */
class RBPF {
private:
    // Configuration
    RBPFConfig config_;
    
    // Particles
    std::vector<RBParticle> particles_;
    std::vector<RBParticle> particles_buffer_;  // For resampling
    
    // Map interfaces
    std::shared_ptr<GravityMapManager> gravity_map_;
    std::shared_ptr<TerrainMapManager> terrain_map_;
    
    // Grid for spatial organization
    std::vector<GridCell> grid_;
    Vector3d grid_origin_;
    Vector3i grid_dimensions_;
    
    // Random number generation
    std::mt19937 rng_;
    std::normal_distribution<double> normal_dist_;
    std::uniform_real_distribution<double> uniform_dist_;
    
    // Statistics
    struct FilterStats {
        double effective_sample_size;
        int resampling_count;
        double max_weight;
        double weight_variance;
        Vector3d mean_position;
        Matrix3d position_covariance;
        double gravity_correlation;
        double terrain_correlation;
        int particle_degeneracy_count;
    } stats_;
    
    // Timing
    double last_update_time_;
    
    // Thread pool for parallel evaluation
    struct ThreadPool;
    std::unique_ptr<ThreadPool> thread_pool_;
    
public:
    RBPF(const RBPFConfig& config);
    explicit RBPF(const YAML::Node& config);
    ~RBPF();
    
    // Initialize filter
    void initialize(const StateVector& initial_state, const MatrixXd& initial_cov);
    
    // Set map managers
    void setGravityMap(std::shared_ptr<GravityMapManager> map) { gravity_map_ = map; }
    void setTerrainMap(std::shared_ptr<TerrainMapManager> map) { terrain_map_ = map; }
    
    // Main filter functions
    void predict(const Vector3d& accel, const Vector3d& gyro, double dt);
    void updateGravity(const Eigen::Matrix<double, 5, 1>& measurement);
    void updateTerrain(double altitude);
    void updateBarometer(double pressure, double temperature);
    
    // Get estimates
    StateVector getMMSE() const;  // Minimum Mean Square Error estimate
    StateVector getMAP() const;    // Maximum A Posteriori estimate
    MatrixXd getCovariance() const;
    
    // Get particle cloud for visualization
    std::vector<Vector3d> getParticlePositions() const;
    std::vector<double> getParticleWeights() const;
    
    // Check filter health
    bool needsResampling() const;
    double getEffectiveSampleSize() const;
    FilterStats getStatistics() const { return stats_; }
    
    // Reset with new position hypothesis
    void injectPositionHypothesis(const Vector3d& position, double confidence);
    
private:
    // Particle operations
    void initializeParticles(const StateVector& state, const MatrixXd& cov);
    void propagateParticles(const Vector3d& accel, const Vector3d& gyro, double dt);
    void updateParticleWeights(const Eigen::Matrix<double, 5, 1>& gravity_meas,
                               double terrain_alt);
    
    // Resampling
    void resample();
    void systematicResampling();
    void stratifiedResampling();
    void residualResampling();
    
    // Map correlation
    double computeGravityLikelihood(const RBParticle& particle,
                                    const Eigen::Matrix<double, 5, 1>& measurement);
    double computeTerrainLikelihood(const RBParticle& particle, double altitude);
    
    // Grid operations
    void updateGrid();
    GridCell& getGridCell(const Vector3d& position);
    std::vector<int> getNearbyParticles(const Vector3d& position, double radius);
    
    // Adaptive sampling
    void adaptiveSampling();
    void refineHighWeightRegions();
    
    // Analytical updates (Kalman filter part)
    void updateAnalyticalStates(RBParticle& particle, const Vector3d& accel,
                               const Vector3d& gyro, double dt);
    
    // Weight operations
    void normalizeWeights();
    void computeStatistics();
    
    // Kernel density estimation
    double kernelDensity(const Vector3d& position) const;
    
    // Outlier detection
    bool isOutlier(const RBParticle& particle) const;
    
    // Logging
    void logParticleDistribution();
    void logMapCorrelation();
};

} // namespace Navigation