#pragma once
#include <Eigen/Dense>
#include <cmath>

struct TrnAdaptiveCfg {
    double sigma_agl = 0.8;      // Radar altimeter noise (m)
    double nis_gate = 9.21;      // Chi-squared gate for scalar
    double slope_thresh = 0.06;  // Minimum slope for full observability (6% grade)
    double alpha_base = 0.35;    // Base update strength
    double alpha_boost = 1.5;    // Boost factor for high slopes
    double huber_c = 1.5;        // Huber threshold
    double max_step = 0.5;       // Maximum position step (m)
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
    
    // Reject flat terrain
    if (slope < cfg.slope_thresh) {
        return false;
    }
    
    // Measurement Jacobian: H = [∂AGL/∂pn, ∂AGL/∂pe, ∂AGL/∂pd]
    Eigen::RowVector3d H;
    H << -terrain_grad.x(), -terrain_grad.y(), -1.0;
    
    // Adaptive measurement variance based on terrain characteristics
    double R = cfg.sigma_agl * cfg.sigma_agl;
    
    // Increase measurement noise for very flat terrain (less observable)
    if (slope < cfg.slope_thresh * 2) {
        R *= (2.0 - slope / (cfg.slope_thresh * 2));
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
    
    // Adaptive alpha based on slope - adjusted for flat terrain
    // For flat terrain, use a different scaling
    double slope_factor = std::min(1.0, slope / 0.01);  // Full strength at 1% grade (was 15%)
    double alpha = cfg.alpha_base * (1.0 + (cfg.alpha_boost - 1.0) * slope_factor);
    
    // Enhanced velocity consistency check
    if (dt > 0 && dt < 1.0) {  // Valid time delta
        double rdot_pred = -v_NED.z() - (terrain_grad.x() * v_NED.x() + terrain_grad.y() * v_NED.y());
        double rdot_meas = (z_agl - z_agl_prev) / dt;
        double rdot_err = std::abs(rdot_meas - rdot_pred);
        
        // Adaptive scaling based on velocity consistency
        if (rdot_err > 5.0) {  // Large inconsistency
            // Strong reduction for likely outliers
            alpha *= std::exp(-rdot_err / 5.0);
        } else if (rdot_err > 2.0) {
            // Moderate reduction
            alpha *= (1.0 - 0.3 * (rdot_err - 2.0) / 3.0);
        }
        // Small errors don't affect alpha
    }
    
    alpha = std::min(alpha, 0.8);  // Cap at 0.8
    if (alpha_out) *alpha_out = alpha;
    
    // Enhanced Huber robustification with adaptive threshold
    double huber_weight = 1.0;
    double adaptive_huber_c = cfg.huber_c;
    
    // Tighten Huber threshold for flat terrain (more prone to errors)
    if (slope < 0.05) {
        adaptive_huber_c *= 0.7;
    }
    
    if (std::abs(y) > adaptive_huber_c) {
        // Smooth Huber function for better continuity
        double ratio = adaptive_huber_c / std::abs(y);
        huber_weight = ratio * ratio;  // Quadratic taper
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