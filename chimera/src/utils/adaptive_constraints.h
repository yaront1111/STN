#pragma once

#include <cmath>
#include <algorithm>
#include <vector>   // getBaroSigma uses std::vector

namespace chimera {
namespace utils {

/**
 * @brief Adaptive constraint scheduler for intelligent noise model tuning
 *
 * Dynamically adjusts sensor noise models based on:
 * - System initialization phase
 * - Sensor health/quality metrics
 * - Innovation (measurement residual) magnitude
 * - Historical performance
 *
 * This replaces static noise values with intelligent scheduling that adapts
 * to the current system state and measurement quality.
 */
class AdaptiveConstraints {
public:
    /**
     * @brief Get adaptive LiDAR sigma based on system state
     * @param key Current keyframe index
     * @param healthy_streak Consecutive healthy LiDAR measurements
     * @param is_trusted Whether LiDAR is validated (guard full, not stuck)
     * @param innovation Measurement residual (predicted - measured)
     * @return Adaptive sigma value (meters)
     */
    static double getLidarSigma(int key, int healthy_streak, bool is_trusted, double innovation) {
        // Early authority ramp: trust quickly but safely.
        if (key < 6) {
            return is_trusted ? 0.08 : 0.30; // warm-up
        }
        // If we're still early and the innovation is huge, don't yank the solution.
        if (key <= 30 && std::abs(innovation) > 50.0) {
            return 5.0; // single-sample soften
        }
        // Trusted LiDAR ramps tighter over first 10 healthy samples.
        double trusted_sigma = (healthy_streak < 10)
                               ? (0.06 - 0.01 * std::min(healthy_streak, 10)) // 0.06 → 0.05
                               : 0.05;
        double base_sigma = is_trusted ? trusted_sigma : (key < 20 ? 0.30 : 0.20);
        // After init, scale gently with innovation to avoid fighting outliers.
        if (key >= 20) {
            double innovation_scale = std::clamp(std::abs(innovation) / 5.0, 0.8, 3.0);
            base_sigma *= innovation_scale;
        }
        return std::clamp(base_sigma, 0.04, 5.0);
    }

    /**
     * @brief Get adaptive bias prior sigma based on initialization phase
     * @param key Current keyframe index
     * @param healthy_streak LiDAR healthy streak (proxy for system maturity)
     * @return Bias sigma values (6-vector: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z])
     */
    static void getBiasPriorSigma(int key, int healthy_streak,
                                  double& accel_sigma, double& gyro_sigma) {
        // Early phase: tight tether
        if (key <= 10) { accel_sigma = 0.15; gyro_sigma = 0.03; return; }
        // Mid phase: relax; if LiDAR is already healthy, relax a bit more
        if (key <= 20) {
            bool mature = healthy_streak >= 10;
            accel_sigma = mature ? 0.25 : 0.30;
            gyro_sigma  = mature ? 0.05 : 0.05;
            return;
        }
        // After k>20: caller should stop adding the prior (let biases adapt).
        accel_sigma = 999.0; gyro_sigma = 999.0;
    }

    /**
     * @brief Get adaptive optical flow sigma based on depth and quality
     * @param depth_m Effective depth to ground (meters)
     * @param num_rays Number of flow rays used
     * @param avg_texture Average texture variance
     * @return Adaptive sigma value (pixels/sec)
     */
    static double getOpticalFlowSigma(double depth_m, int num_rays, double avg_texture) {
        // Base sigma: 2.0 px/s (good conditions)
        double base_sigma = 2.0;
        // Clamp depth to a sane range and scale (uncertainty ↑ with altitude)
        double d = std::clamp(depth_m, 0.5, 300.0);
        double depth_scale = std::clamp(std::sqrt(d / 1.0), 1.0, 4.0);
        // Fewer rays → less confidence. Guard num_rays==0.
        double ray_scale = 1.0;
        if (num_rays <= 0) ray_scale = 2.0;
        else if (num_rays < 20) ray_scale = std::clamp(20.0 / static_cast<double>(num_rays), 1.0, 2.0);
        // Low texture → higher uncertainty. Guard tiny/zero texture.
        double texture_scale = 1.0;
        if (avg_texture > 0.0 && avg_texture < 100.0) {
            texture_scale = std::clamp(100.0 / avg_texture, 1.0, 1.5);
        } else if (avg_texture <= 0.0) {
            texture_scale = 1.5;
        }
        double sigma = base_sigma * depth_scale * ray_scale * texture_scale;
        return std::clamp(sigma, 2.0, 24.0);
    }

    /**
     * @brief Get adaptive barometer sigma based on innovation sequence
     * @param recent_innovations Recent measurement residuals (for consistency check)
     * @return Adaptive sigma value (meters)
     */
    static double getBaroSigma(const std::vector<double>& recent_innovations) {
        // Base sigma: 0.8m (typical baro accuracy)
        double base_sigma = 0.8;

        if (recent_innovations.size() < 3) {
            return base_sigma;
        }

        // Compute innovation variance (measurement consistency)
        double mean = 0.0;
        for (double innov : recent_innovations) {
            mean += innov;
        }
        mean /= recent_innovations.size();

        double variance = 0.0;
        for (double innov : recent_innovations) {
            double diff = innov - mean;
            variance += diff * diff;
        }
        variance /= recent_innovations.size();
        double stddev = std::sqrt(variance);

        // If innovations are inconsistent (high variance), increase sigma
        // Consistent (stddev<0.5m): scale=1.0
        // Noisy (stddev>2.0m): scale=3.0
        double consistency_scale = std::clamp(stddev / 0.5, 1.0, 3.0);
        double sigma = base_sigma * consistency_scale;
        return std::clamp(sigma, 0.8, 2.5); // align with robustBaroAGL caps
    }

    /**
     * @brief Get adaptive velocity prior sigma based on observability
     * @param have_lidar Whether we have altitude constraint
     * @param have_of Whether we have optical flow constraint
     * @param have_airspeed Whether we have airspeed constraint
     * @return Velocity prior sigma (3-vector: [vx, vy, vz])
     */
    static void getVelocityPriorSigma(bool have_lidar, bool have_of, bool have_airspeed,
                                      double& sigma_xy, double& sigma_z) {
        // Vertical velocity: tight when no altitude constraint
        if (!have_lidar) {
            sigma_z = 0.8;  // 0.8 m/s - strong damping when no altitude
        } else {
            sigma_z = 5.0;  // 5 m/s - loose (altitude + preintegration controls this)
        }

        // Horizontal velocity: observability-aware damping
        // NOTE: Airspeed only constrains forward velocity magnitude, not full 2D velocity!
        if (have_of) {
            sigma_xy = 5.0;  // 5 m/s - loose (OF provides full 2D velocity constraint)
        } else if (have_airspeed) {
            sigma_xy = 1.5;  // 1.5 m/s - moderate (airspeed provides limited constraint)
        } else {
            sigma_xy = 0.5;  // 0.5 m/s - tight (no horizontal constraints)
        }
    }
};

}  // namespace utils
}  // namespace chimera
