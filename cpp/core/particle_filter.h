#pragma once

#include <vector>
#include <random>
#include <memory>
#include <Eigen/Dense>
#include "types.h"
#include "gravity_gradient_provider.h"

/**
 * Particle Filter for Non-Linear Map Matching
 * Handles multi-modal distributions and non-Gaussian noise
 */
class ParticleFilter {
public:
    struct Config {
        int num_particles;
        double initial_spread_m;
        double initial_vel_spread_mps;
        double resampling_threshold;  // Effective sample size threshold
        bool adaptive_particles;
        double process_noise_pos;
        double process_noise_vel;
        double process_noise_att;

        Config() : num_particles(1000),
                   initial_spread_m(100.0),
                   initial_vel_spread_mps(10.0),
                   resampling_threshold(0.5),
                   adaptive_particles(true),
                   process_noise_pos(1.0),
                   process_noise_vel(0.1),
                   process_noise_att(0.01) {}
    };

    struct Particle {
        State state;
        double weight;
        double log_likelihood;
        int id;

        // Track particle history for smoothing
        std::vector<State> history;

        Particle() : weight(1.0), log_likelihood(0.0), id(0) {}
    };

    ParticleFilter(const Config& config = Config());

    // Initialize particle cloud
    void initialize(const State& initial_state, const Eigen::MatrixXd& covariance);

    // Prediction step
    void predict(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt);

    // Update steps for different sensors
    void updateGravity(const Eigen::Matrix3d& measured_gradient, GravityGradientProvider* provider);
    void updateTerrain(double measured_altitude, double terrain_height);
    void updateMagnetometer(const Eigen::Vector3d& measured_field);
    void updateBarometer(double measured_pressure);

    // Map matching with multiple hypotheses
    struct MapMatchResult {
        std::vector<State> hypotheses;
        std::vector<double> probabilities;
        State best_estimate;
        Eigen::MatrixXd covariance;
        double confidence;
    };

    MapMatchResult performMapMatching(
        const Eigen::Matrix3d& gravity_gradient,
        GravityGradientProvider* gravity_provider,
        double terrain_altitude = 0.0
    );

    // Get state estimate
    State getEstimate() const;
    Eigen::MatrixXd getCovariance() const;

    // Get particle cloud for visualization
    std::vector<Particle> getParticles() const { return particles_; }

    // Resampling strategies
    void resampleSystematic();
    void resampleStratified();
    void resampleAdaptive();

    // Compute effective sample size
    double getEffectiveSampleSize() const;

    // Divergence detection
    bool isDiverged() const;
    void resetIfDiverged();

    // Performance metrics
    struct Statistics {
        double effective_sample_size;
        double weight_variance;
        double position_spread;
        double max_weight;
        int unique_particles;
        bool resampled;
    };

    Statistics getStatistics() const;

private:
    Config config_;
    std::vector<Particle> particles_;
    std::mt19937 rng_;
    int particle_id_counter_;

    // Motion model with process noise
    void propagateParticle(Particle& particle,
                          const Eigen::Vector3d& acc,
                          const Eigen::Vector3d& gyro,
                          double dt);

    // Measurement likelihood functions
    double computeGravityLikelihood(const State& state,
                                   const Eigen::Matrix3d& measured,
                                   GravityGradientProvider* provider);

    double computeTerrainLikelihood(const State& state,
                                   double measured_altitude,
                                   double terrain_height);

    // Weight normalization
    void normalizeWeights();

    // Clustering for multi-modal tracking
    std::vector<std::vector<int>> clusterParticles(double distance_threshold);

    // Add process noise
    void addProcessNoise(Particle& particle);

    // Roughening to prevent sample impoverishment
    void roughenParticles();
};

/**
 * Rao-Blackwellized Particle Filter
 * Combines particle filter for position with Kalman filter for velocity/attitude
 */
class RBParticleFilter {
public:
    struct RBParticle {
        Eigen::Vector3d position;

        // Kalman filter states
        Eigen::Vector3d velocity;
        Eigen::Quaterniond attitude;
        Eigen::MatrixXd covariance;  // 6x6 for vel and att

        double weight;

        RBParticle() : weight(1.0), covariance(Eigen::MatrixXd::Identity(6, 6)) {}
    };

    RBParticleFilter(int num_particles = 500);

    void initialize(const State& initial_state, const Eigen::MatrixXd& covariance);

    void predict(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt);

    void updateGravity(const Eigen::Matrix3d& measured_gradient,
                      GravityGradientProvider* provider);

    State getEstimate() const;

private:
    std::vector<RBParticle> particles_;
    std::mt19937 rng_;

    void propagateKalmanStates(RBParticle& particle,
                               const Eigen::Vector3d& acc,
                               const Eigen::Vector3d& gyro,
                               double dt);
};

/**
 * Adaptive Particle Filter with dynamic particle count
 */
class AdaptiveParticleFilter : public ParticleFilter {
public:
    struct AdaptiveConfig : Config {
        int min_particles = 100;
        int max_particles = 5000;
        double kld_error = 0.05;
        double kld_z = 2.326;  // 99% confidence
    };

    AdaptiveParticleFilter(const AdaptiveConfig& config);

    // Dynamically adjust particle count based on complexity
    void adaptParticleCount();

    // KLD-sampling for efficiency
    int computeRequiredParticles();

private:
    AdaptiveConfig adaptive_config_;

    // Measure distribution complexity
    double computeEntropy() const;
    double computeKLDivergence(const std::vector<Particle>& old_particles) const;
};