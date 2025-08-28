#pragma once
#include <Eigen/Dense>
#include <cmath>

struct TrnAdaptiveCfg {
    double sigma_agl = 0.5;      // Radar altimeter noise (m) - reduced for cleaner data
    double nis_gate = 12.59;     // Chi-squared gate for scalar - more permissive
    double slope_thresh = 0.002; // Minimum slope for observability (0.2% grade for flat terrain)
    double alpha_base = 0.15;    // Base update strength - start aggressive
    double alpha_boost = 2.0;    // Boost factor for high slopes
    double huber_c = 2.5;        // Huber threshold - less aggressive
    double max_step = 1.0;       // Maximum position step (m) - allow larger corrections
};

inline bool trn_update_adaptive(
    Eigen::Vector3d& p_NED,
    Eigen::Matrix3d& P_pos,
    const Eigen::Vector3d& v_NED,
    double z_agl,
    double terrain_h,
    const Eigen::Vector2d& terrain_grad,
    const TrnAdaptiveCfg& cfg,
    double* nis_out = nullptr,
    double* alpha_out = nullptr,
    double z_agl_prev = 0.0,
    double dt = 0.0
) {
    // Predicted AGL
    double z_pred = (-p_NED.z()) - terrain_h;
    double y = z_agl - z_pred;  // Innovation
    
    // Slope magnitude
    double slope = terrain_grad.norm();
    
    // Don't reject flat terrain - use it with appropriate weighting
    // Remove hard threshold to allow all terrain to contribute
    
    // Measurement Jacobian: H = [∂AGL/∂pn, ∂AGL/∂pe, ∂AGL/∂pd]
    Eigen::RowVector3d H;
    H << -terrain_grad.x(), -terrain_grad.y(), -1.0;
    
    // Dynamic measurement variance based on terrain observability
    double base_R = cfg.sigma_agl * cfg.sigma_agl;
    
    // Exponentially inflate noise for flat terrain
    // This provides smooth degradation rather than hard cutoff
    const double slope_optimal = 0.1;  // 10% slope is optimal
    const double slope_floor = 0.002;  // Below 0.2% is nearly unobservable
    
    double R;
    if (slope < slope_floor) {
        // Extremely flat - essentially no horizontal information
        R = base_R * 10000.0;  // Massive inflation
    } else if (slope < cfg.slope_thresh) {
        // Sub-threshold but not completely flat
        // Exponential inflation based on how far below threshold
        double inflation = std::pow(cfg.slope_thresh / slope, 3.0);
        R = base_R * std::min(inflation, 1000.0);
    } else if (slope < slope_optimal) {
        // Above threshold but below optimal - mild inflation
        double inflation = 1.0 + 2.0 * (slope_optimal - slope) / slope_optimal;
        R = base_R * inflation;
    } else {
        // Optimal terrain - use base noise
        R = base_R;
    }
    
    // Innovation covariance
    double S = (H * P_pos * H.transpose())(0,0) + R;
    
    // NIS for gating
    double nis = (y * y) / S;
    if (nis_out) *nis_out = nis;
    
    // Gate check
    if (nis > cfg.nis_gate) {
        return false;
    }
    
    // Smart adaptive alpha with time-varying confidence
    // Note: We don't have access to x.t here, so we'll use a different approach
    // Base alpha varies with slope
    double slope_factor = std::min(1.0, slope / 0.01);  // Full strength at 1% grade
    double alpha = cfg.alpha_base * (1.0 + (cfg.alpha_boost - 1.0) * slope_factor);
    
    // Velocity consistency check (less aggressive)
    if (dt > 0 && dt < 1.0) {  // Valid time delta
        double rdot_pred = -v_NED.z() - (terrain_grad.x() * v_NED.x() + terrain_grad.y() * v_NED.y());
        double rdot_meas = (z_agl - z_agl_prev) / dt;
        double rdot_err = std::abs(rdot_meas - rdot_pred);
        
        // Only reduce for very large inconsistencies
        if (rdot_err > 10.0) {  // Very large inconsistency
            alpha *= 0.5;
        } else if (rdot_err > 5.0) {
            alpha *= 0.75;
        }
        // Normal errors don't affect alpha
    }
    
    alpha = std::min(alpha, 0.8);  // Cap at 0.8
    if (alpha_out) *alpha_out = alpha;
    
    // Huber robustification (less aggressive)
    double huber_weight = 1.0;
    
    if (std::abs(y) > cfg.huber_c) {
        // Linear Huber for less aggressive outlier handling
        huber_weight = cfg.huber_c / std::abs(y);
    }
    
    // Kalman gain
    Eigen::Vector3d K = (P_pos * H.transpose()) / S;
    
    // Position correction with Huber weight
    Eigen::Vector3d dp = alpha * huber_weight * K * y;
    
    // Limit step size - apply limit to full 3D correction
    double step_norm = dp.norm();  // Full 3D step
    if (step_norm > cfg.max_step) {
        dp *= cfg.max_step / step_norm;
    }
    
    // Update state
    p_NED += dp;
    
    // Joseph form covariance update for numerical stability
    Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - alpha * huber_weight * K * H;
    P_pos = I_KH * P_pos * I_KH.transpose() + alpha * alpha * huber_weight * huber_weight * K * R * K.transpose();
    
    // Ensure positive definite
    for (int i = 0; i < 3; i++) {
        P_pos(i,i) = std::max(P_pos(i,i), 0.01);
    }
    
    return true;
}