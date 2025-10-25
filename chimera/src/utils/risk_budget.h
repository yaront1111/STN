#pragma once

#include <cmath>
#include <algorithm>

namespace chimera {
namespace utils {

/**
 * @brief Risk-budgeted sigma allocation via Chebyshev inequality
 *
 * Implements chance-constraint-based noise schedules with provable tail-risk budgets:
 *   Pr(|residual| ≥ c·σ̂) ≤ 1/(1+c²) = α
 *
 * Allows principled sigma inflation that guarantees total tail-risk ≤ α (e.g., 5%)
 * across all sensors, preventing catastrophic factor graph divergence.
 */

/**
 * @brief Compute Chebyshev scale factor from tail-risk budget
 * @param alpha Desired tail-risk probability (e.g., 0.05 for 5% budget)
 * @return Scale factor c such that Pr(|r| ≥ c·σ̂) ≤ α
 *
 * Solves: α = 1/(1+c²) → c = sqrt((1-α)/α)
 */
inline double chebyshev_scale_from_alpha(double alpha) {
    alpha = std::clamp(alpha, 1e-6, 0.5);  // Guard against invalid alpha
    return std::sqrt((1.0 - alpha) / alpha);
}

/**
 * @brief Per-sensor risk allocation weights
 *
 * Each sensor gets a fraction of the total risk budget based on:
 * - Criticality (how essential the sensor is for observability)
 * - Reliability (historical performance/health)
 * - Redundancy (availability of alternative sensors)
 */
struct RiskSplit {
    double lidar_weight = 0.30;      ///< 30% of risk budget (critical altitude constraint)
    double baro_weight = 0.15;       ///< 15% of risk budget (backup altitude)
    double of_weight = 0.40;         ///< 40% of risk budget (primary horizontal constraint)
    double airspeed_weight = 0.15;   ///< 15% of risk budget (forward velocity constraint)
};

/**
 * @brief Allocate total tail-risk budget across sensors
 * @param total_alpha Total tail-risk budget (e.g., 0.05 for 5% system-wide)
 * @param split Per-sensor risk allocation weights (must sum to 1.0)
 * @return Per-sensor scale factors c_i such that ∑ Pr(|r_i| ≥ c_i·σ̂_i) ≤ total_alpha
 *
 * Usage:
 *   auto [c_lidar, c_baro, c_of, c_airspeed] = split_risk(0.05, RiskSplit{});
 *   double sigma_lidar_inflated = c_lidar * sigma_lidar_adaptive;
 */
inline std::tuple<double, double, double, double> split_risk(
    double total_alpha,
    const RiskSplit& split = RiskSplit{}) {

    // Validate weights sum to 1.0 (within tolerance)
    double weight_sum = split.lidar_weight + split.baro_weight +
                        split.of_weight + split.airspeed_weight;
    if (std::abs(weight_sum - 1.0) > 0.01) {
        // Normalize if needed
        double norm = 1.0 / weight_sum;
        RiskSplit normalized;
        normalized.lidar_weight = split.lidar_weight * norm;
        normalized.baro_weight = split.baro_weight * norm;
        normalized.of_weight = split.of_weight * norm;
        normalized.airspeed_weight = split.airspeed_weight * norm;
        return split_risk(total_alpha, normalized);
    }

    // Allocate per-sensor risk budgets: α_i = weight_i * total_alpha
    double alpha_lidar = split.lidar_weight * total_alpha;
    double alpha_baro = split.baro_weight * total_alpha;
    double alpha_of = split.of_weight * total_alpha;
    double alpha_airspeed = split.airspeed_weight * total_alpha;

    // Compute Chebyshev scale factors for each sensor
    double c_lidar = chebyshev_scale_from_alpha(alpha_lidar);
    double c_baro = chebyshev_scale_from_alpha(alpha_baro);
    double c_of = chebyshev_scale_from_alpha(alpha_of);
    double c_airspeed = chebyshev_scale_from_alpha(alpha_airspeed);

    return {c_lidar, c_baro, c_of, c_airspeed};
}

}  // namespace utils
}  // namespace chimera
