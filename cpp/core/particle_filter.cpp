#include "particle_filter.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <set>

ParticleFilter::ParticleFilter(const Config& config)
    : config_(config), particle_id_counter_(0) {
    particles_.reserve(config.num_particles);
    rng_.seed(std::random_device{}());
}

void ParticleFilter::initialize(const State& initial_state, const Eigen::MatrixXd& covariance) {
    particles_.clear();
    particles_.resize(config_.num_particles);

    // Extract standard deviations
    Eigen::Vector3d pos_std = covariance.block<3,3>(0,0).diagonal().cwiseSqrt();
    Eigen::Vector3d vel_std = covariance.block<3,3>(3,3).diagonal().cwiseSqrt();
    Eigen::Vector3d att_std = covariance.block<3,3>(6,6).diagonal().cwiseSqrt();

    // Create distributions
    std::normal_distribution<double> pos_dist_x(0, pos_std(0));
    std::normal_distribution<double> pos_dist_y(0, pos_std(1));
    std::normal_distribution<double> pos_dist_z(0, pos_std(2));
    std::normal_distribution<double> vel_dist_x(0, vel_std(0));
    std::normal_distribution<double> vel_dist_y(0, vel_std(1));
    std::normal_distribution<double> vel_dist_z(0, vel_std(2));
    std::normal_distribution<double> att_dist(0, att_std.norm());

    // Initialize particles
    for (auto& particle : particles_) {
        particle.state = initial_state;
        particle.id = particle_id_counter_++;

        // Add random perturbations
        particle.state.p_ECEF += Eigen::Vector3d(
            pos_dist_x(rng_), pos_dist_y(rng_), pos_dist_z(rng_)
        );

        particle.state.v_ECEF += Eigen::Vector3d(
            vel_dist_x(rng_), vel_dist_y(rng_), vel_dist_z(rng_)
        );

        // Perturb attitude
        if (att_std.norm() > 0) {
            Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();
            double angle = att_dist(rng_);
            Eigen::Quaterniond perturbation(Eigen::AngleAxisd(angle, axis));
            particle.state.q_ECEF_B = particle.state.q_ECEF_B * perturbation;
            particle.state.q_ECEF_B.normalize();
        }

        particle.weight = 1.0 / config_.num_particles;
        particle.log_likelihood = 0.0;
    }
}

void ParticleFilter::predict(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double dt) {
    for (auto& particle : particles_) {
        propagateParticle(particle, acc, gyro, dt);
        addProcessNoise(particle);
    }
}

void ParticleFilter::propagateParticle(Particle& particle,
                                       const Eigen::Vector3d& acc,
                                       const Eigen::Vector3d& gyro,
                                       double dt) {
    State& state = particle.state;

    // Rotate acceleration to ECEF frame
    Eigen::Vector3d acc_ECEF = state.q_ECEF_B * acc;

    // Add gravity (simplified model)
    const double g = 9.81;
    Eigen::Vector3d gravity = -g * state.p_ECEF.normalized();
    acc_ECEF += gravity;

    // Update position and velocity
    state.p_ECEF += state.v_ECEF * dt + 0.5 * acc_ECEF * dt * dt;
    state.v_ECEF += acc_ECEF * dt;

    // Update attitude using gyroscope
    Eigen::Quaterniond q_delta(1.0,
        0.5 * gyro(0) * dt,
        0.5 * gyro(1) * dt,
        0.5 * gyro(2) * dt);
    state.q_ECEF_B = state.q_ECEF_B * q_delta;
    state.q_ECEF_B.normalize();

    // Store history for smoothing
    particle.history.push_back(state);
    if (particle.history.size() > 100) {
        particle.history.erase(particle.history.begin());
    }
}

void ParticleFilter::addProcessNoise(Particle& particle) {
    std::normal_distribution<double> pos_noise(0, config_.process_noise_pos);
    std::normal_distribution<double> vel_noise(0, config_.process_noise_vel);
    std::normal_distribution<double> att_noise(0, config_.process_noise_att);

    // Add position noise
    particle.state.p_ECEF += Eigen::Vector3d(
        pos_noise(rng_), pos_noise(rng_), pos_noise(rng_)
    );

    // Add velocity noise
    particle.state.v_ECEF += Eigen::Vector3d(
        vel_noise(rng_), vel_noise(rng_), vel_noise(rng_)
    );

    // Add attitude noise
    if (config_.process_noise_att > 0) {
        Eigen::Vector3d axis = Eigen::Vector3d::Random().normalized();
        double angle = att_noise(rng_);
        Eigen::Quaterniond perturbation(Eigen::AngleAxisd(angle, axis));
        particle.state.q_ECEF_B = particle.state.q_ECEF_B * perturbation;
        particle.state.q_ECEF_B.normalize();
    }
}

void ParticleFilter::updateGravity(const Eigen::Matrix3d& measured_gradient,
                                   GravityGradientProvider* provider) {
    for (auto& particle : particles_) {
        double likelihood = computeGravityLikelihood(particle.state, measured_gradient, provider);
        particle.weight *= likelihood;
        particle.log_likelihood += std::log(likelihood + 1e-100);
    }

    normalizeWeights();

    // Resample if needed
    if (getEffectiveSampleSize() < config_.resampling_threshold * config_.num_particles) {
        resampleSystematic();
    }
}

double ParticleFilter::computeGravityLikelihood(const State& state,
                                               const Eigen::Matrix3d& measured,
                                               GravityGradientProvider* provider) {
    if (!provider) return 1.0;

    // Get expected gradient at particle position
    Eigen::Matrix3d expected = provider->getGradientTensor(state.p_ECEF);

    // Rotate to body frame
    Eigen::Matrix3d R = state.q_ECEF_B.toRotationMatrix();
    expected = R.transpose() * expected * R;

    // Compute error
    Eigen::Matrix3d diff = measured - expected;
    double error = diff.norm();

    // Gaussian likelihood with adaptive variance
    double sigma = 10.0;  // Eötvös units
    return std::exp(-0.5 * error * error / (sigma * sigma));
}

void ParticleFilter::normalizeWeights() {
    double sum = 0.0;
    for (const auto& particle : particles_) {
        sum += particle.weight;
    }

    if (sum > 0) {
        for (auto& particle : particles_) {
            particle.weight /= sum;
        }
    } else {
        // Reset weights if all are zero
        for (auto& particle : particles_) {
            particle.weight = 1.0 / config_.num_particles;
        }
    }
}

State ParticleFilter::getEstimate() const {
    State estimate;
    estimate.p_ECEF = Eigen::Vector3d::Zero();
    estimate.v_ECEF = Eigen::Vector3d::Zero();

    // Weighted average of particles
    for (const auto& particle : particles_) {
        estimate.p_ECEF += particle.weight * particle.state.p_ECEF;
        estimate.v_ECEF += particle.weight * particle.state.v_ECEF;
    }

    // Find particle with highest weight for attitude (can't average quaternions simply)
    auto max_particle = std::max_element(particles_.begin(), particles_.end(),
        [](const Particle& a, const Particle& b) { return a.weight < b.weight; });

    estimate.q_ECEF_B = max_particle->state.q_ECEF_B;

    return estimate;
}

Eigen::MatrixXd ParticleFilter::getCovariance() const {
    State mean = getEstimate();
    Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(15, 15);

    for (const auto& particle : particles_) {
        Eigen::VectorXd diff(15);
        diff.head<3>() = particle.state.p_ECEF - mean.p_ECEF;
        diff.segment<3>(3) = particle.state.v_ECEF - mean.v_ECEF;

        // Attitude error (simplified)
        Eigen::Quaterniond q_error = mean.q_ECEF_B.inverse() * particle.state.q_ECEF_B;
        Eigen::AngleAxisd angle_axis(q_error);
        diff.segment<3>(6) = angle_axis.angle() * angle_axis.axis();

        cov += particle.weight * (diff * diff.transpose());
    }

    return cov;
}

double ParticleFilter::getEffectiveSampleSize() const {
    double sum_sq = 0.0;
    for (const auto& particle : particles_) {
        sum_sq += particle.weight * particle.weight;
    }
    return 1.0 / (sum_sq + 1e-100);
}

void ParticleFilter::resampleSystematic() {
    std::vector<Particle> new_particles;
    new_particles.reserve(config_.num_particles);

    // Compute cumulative weights
    std::vector<double> cumulative_weights(particles_.size());
    cumulative_weights[0] = particles_[0].weight;
    for (size_t i = 1; i < particles_.size(); ++i) {
        cumulative_weights[i] = cumulative_weights[i-1] + particles_[i].weight;
    }

    // Systematic resampling
    std::uniform_real_distribution<double> uniform(0.0, 1.0 / config_.num_particles);
    double u = uniform(rng_);

    for (int i = 0; i < config_.num_particles; ++i) {
        double target = u + i / static_cast<double>(config_.num_particles);

        // Find particle to copy
        auto it = std::lower_bound(cumulative_weights.begin(), cumulative_weights.end(), target);
        int idx = std::distance(cumulative_weights.begin(), it);
        idx = std::min(idx, static_cast<int>(particles_.size() - 1));

        new_particles.push_back(particles_[idx]);
        new_particles.back().weight = 1.0 / config_.num_particles;
        new_particles.back().id = particle_id_counter_++;
    }

    particles_ = std::move(new_particles);

    // Roughening to prevent sample impoverishment
    roughenParticles();
}

void ParticleFilter::roughenParticles() {
    // Add small random noise to prevent particle depletion
    const double roughening_factor = 0.01;

    for (auto& particle : particles_) {
        std::normal_distribution<double> noise(0, roughening_factor);

        particle.state.p_ECEF += Eigen::Vector3d(
            noise(rng_), noise(rng_), noise(rng_)
        );

        particle.state.v_ECEF += Eigen::Vector3d(
            noise(rng_) * 0.1, noise(rng_) * 0.1, noise(rng_) * 0.1
        );
    }
}

bool ParticleFilter::isDiverged() const {
    // Check if effective sample size is too low
    if (getEffectiveSampleSize() < 10) {
        return true;
    }

    // Check if position spread is too large
    State mean = getEstimate();
    double max_distance = 0.0;

    for (const auto& particle : particles_) {
        double distance = (particle.state.p_ECEF - mean.p_ECEF).norm();
        max_distance = std::max(max_distance, distance);
    }

    return max_distance > 10000.0;  // 10km spread indicates divergence
}

void ParticleFilter::resetIfDiverged() {
    if (isDiverged()) {
        State current_estimate = getEstimate();
        Eigen::MatrixXd current_cov = getCovariance();
        current_cov *= 10.0;  // Increase uncertainty
        initialize(current_estimate, current_cov);
    }
}

ParticleFilter::Statistics ParticleFilter::getStatistics() const {
    Statistics stats;
    stats.effective_sample_size = getEffectiveSampleSize();

    // Weight variance
    double mean_weight = 1.0 / config_.num_particles;
    double weight_var = 0.0;
    stats.max_weight = 0.0;

    for (const auto& particle : particles_) {
        weight_var += std::pow(particle.weight - mean_weight, 2);
        stats.max_weight = std::max(stats.max_weight, particle.weight);
    }
    stats.weight_variance = weight_var / config_.num_particles;

    // Position spread
    State mean = getEstimate();
    double pos_var = 0.0;
    for (const auto& particle : particles_) {
        pos_var += (particle.state.p_ECEF - mean.p_ECEF).squaredNorm();
    }
    stats.position_spread = std::sqrt(pos_var / config_.num_particles);

    // Count unique particles
    std::set<int> unique_ids;
    for (const auto& particle : particles_) {
        unique_ids.insert(particle.id);
    }
    stats.unique_particles = unique_ids.size();

    stats.resampled = (stats.effective_sample_size < config_.resampling_threshold * config_.num_particles);

    return stats;
}

ParticleFilter::MapMatchResult ParticleFilter::performMapMatching(
    const Eigen::Matrix3d& gravity_gradient,
    GravityGradientProvider* gravity_provider,
    double terrain_altitude) {

    MapMatchResult result;

    // Update weights based on gravity matching
    updateGravity(gravity_gradient, gravity_provider);

    // Cluster particles to find multiple hypotheses
    auto clusters = clusterParticles(100.0);  // 100m clustering threshold

    // Extract hypothesis from each cluster
    for (const auto& cluster : clusters) {
        if (cluster.size() < 10) continue;  // Skip small clusters

        State hypothesis;
        hypothesis.p_ECEF = Eigen::Vector3d::Zero();
        hypothesis.v_ECEF = Eigen::Vector3d::Zero();
        double cluster_weight = 0.0;

        for (int idx : cluster) {
            hypothesis.p_ECEF += particles_[idx].weight * particles_[idx].state.p_ECEF;
            hypothesis.v_ECEF += particles_[idx].weight * particles_[idx].state.v_ECEF;
            cluster_weight += particles_[idx].weight;
        }

        if (cluster_weight > 0) {
            hypothesis.p_ECEF /= cluster_weight;
            hypothesis.v_ECEF /= cluster_weight;
            hypothesis.q_ECEF_B = particles_[cluster[0]].state.q_ECEF_B;  // Use first particle's attitude

            result.hypotheses.push_back(hypothesis);
            result.probabilities.push_back(cluster_weight);
        }
    }

    // Normalize probabilities
    double total_prob = std::accumulate(result.probabilities.begin(),
                                       result.probabilities.end(), 0.0);
    if (total_prob > 0) {
        for (auto& prob : result.probabilities) {
            prob /= total_prob;
        }
    }

    // Best estimate is weighted average or highest probability hypothesis
    result.best_estimate = getEstimate();
    result.covariance = getCovariance();
    result.confidence = getEffectiveSampleSize() / config_.num_particles;

    return result;
}

std::vector<std::vector<int>> ParticleFilter::clusterParticles(double distance_threshold) {
    std::vector<std::vector<int>> clusters;
    std::vector<bool> assigned(particles_.size(), false);

    for (size_t i = 0; i < particles_.size(); ++i) {
        if (assigned[i]) continue;

        std::vector<int> cluster;
        cluster.push_back(i);
        assigned[i] = true;

        // Find all particles within threshold
        for (size_t j = i + 1; j < particles_.size(); ++j) {
            if (assigned[j]) continue;

            double distance = (particles_[i].state.p_ECEF -
                             particles_[j].state.p_ECEF).norm();

            if (distance < distance_threshold) {
                cluster.push_back(j);
                assigned[j] = true;
            }
        }

        clusters.push_back(cluster);
    }

    return clusters;
}