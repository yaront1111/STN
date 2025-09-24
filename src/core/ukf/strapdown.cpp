/**
 * Strapdown Mechanization Implementation
 * Includes coning and sculling compensation
 * Complete implementation with no placeholders
 */

#include "strapdown.h"
#include "../../utils/logger.h"
#include <cmath>
#include <sstream>

namespace Navigation {

StrapdownMechanization::StrapdownMechanization() {
    reset();
}

void StrapdownMechanization::reset() {
    prev_accel_.setZero();
    prev_gyro_.setZero();
    prev_dv_.setZero();
    prev_dtheta_.setZero();
    initialized_ = false;
    
    LOG_DEBUG("Strapdown mechanization reset");
}

StrapdownMechanization::CompensatedIMU 
StrapdownMechanization::compensate(const Vector3d& accel, const Vector3d& gyro, double dt) {
    CompensatedIMU result;
    result.dt = dt;
    
    // Raw increments
    Vector3d dv = accel * dt;
    Vector3d dtheta = gyro * dt;
    
    if (!initialized_) {
        // First sample - no compensation possible
        result.delta_v = dv;
        result.delta_theta = dtheta;
        
        // Store for next iteration
        prev_accel_ = accel;
        prev_gyro_ = gyro;
        prev_dv_ = dv;
        prev_dtheta_ = dtheta;
        initialized_ = true;
        
        return result;
    }
    
    // Apply Savage algorithm for coning compensation
    Vector3d coning = coningCompensation(dtheta, prev_dtheta_);
    
    // Apply sculling compensation
    Vector3d sculling = scullingCompensation(dv, dtheta, prev_dv_, prev_dtheta_);
    
    // Compensated increments
    result.delta_theta = dtheta + coning;
    result.delta_v = dv + sculling;
    
    // Store for next iteration
    prev_accel_ = accel;
    prev_gyro_ = gyro;
    prev_dv_ = dv;
    prev_dtheta_ = dtheta;
    
    // Log compensation magnitudes for monitoring
    if (coning.norm() > 1e-6 || sculling.norm() > 1e-6) {
        {
            std::stringstream msg;
            msg << "Coning: " << coning.norm() << " rad, Sculling: " << sculling.norm() << " m/s";
            LOG_DEBUG(msg.str());
        }
    }
    
    return result;
}

Vector3d StrapdownMechanization::coningCompensation(const Vector3d& dtheta_k, 
                                                   const Vector3d& dtheta_km1) {
    // Savage two-sample coning correction
    // Compensates for rotation non-commutativity
    return 0.5 * dtheta_km1.cross(dtheta_k);
}

Vector3d StrapdownMechanization::scullingCompensation(const Vector3d& dv_k, 
                                                      const Vector3d& dtheta_k,
                                                      const Vector3d& dv_km1, 
                                                      const Vector3d& dtheta_km1) {
    // Savage two-sample sculling correction
    // Compensates for rotation during acceleration
    return 0.5 * (dv_km1.cross(dtheta_k) + dtheta_km1.cross(dv_k));
}

// Extended mechanization for high-dynamic environments

ExtendedMechanization::ExtendedMechanization() : StrapdownMechanization() {
    // sample_buffer_ is managed with MAX_BUFFER_SIZE in header
}

void ExtendedMechanization::reset() {
    StrapdownMechanization::reset();
    sample_buffer_.clear();
}

StrapdownMechanization::CompensatedIMU 
ExtendedMechanization::compensate(const Vector3d& accel, const Vector3d& gyro, double dt) {
    // Store sample
    IMUSample sample;
    sample.accel = accel;
    sample.gyro = gyro;
    sample.dt = dt;
    sample.dv = accel * dt;
    sample.dtheta = gyro * dt;
    
    sample_buffer_.push_back(sample);
    // Keep buffer size limited
    while (sample_buffer_.size() > MAX_BUFFER_SIZE) {
        sample_buffer_.pop_front();
    }
    
    // Need at least 2 samples for compensation
    if (sample_buffer_.size() < 2) {
        CompensatedIMU result;
        result.delta_v = sample.dv;
        result.delta_theta = sample.dtheta;
        result.dt = dt;
        return result;
    }
    
    // Use higher-order compensation if enough samples
    if (sample_buffer_.size() >= 3) {
        return thirdOrderCompensation();
    } else {
        return secondOrderCompensation();
    }
}

StrapdownMechanization::CompensatedIMU 
ExtendedMechanization::secondOrderCompensation() {
    // Two-sample compensation (Savage algorithm)
    const auto& s0 = sample_buffer_[sample_buffer_.size() - 2];
    const auto& s1 = sample_buffer_[sample_buffer_.size() - 1];
    
    CompensatedIMU result;
    result.dt = s1.dt;
    
    // Coning correction
    Vector3d coning = 0.5 * s0.dtheta.cross(s1.dtheta);
    
    // Sculling correction
    Vector3d sculling = 0.5 * (s0.dv.cross(s1.dtheta) + s0.dtheta.cross(s1.dv));
    
    result.delta_theta = s1.dtheta + coning;
    result.delta_v = s1.dv + sculling;
    
    return result;
}

StrapdownMechanization::CompensatedIMU 
ExtendedMechanization::thirdOrderCompensation() {
    // Three-sample compensation for higher accuracy
    const auto& s0 = sample_buffer_[sample_buffer_.size() - 3];
    const auto& s1 = sample_buffer_[sample_buffer_.size() - 2];
    const auto& s2 = sample_buffer_[sample_buffer_.size() - 1];
    
    CompensatedIMU result;
    result.dt = s2.dt;
    
    // Third-order coning correction
    Vector3d coning = 0.5 * s1.dtheta.cross(s2.dtheta);
    coning += (1.0/12.0) * s0.dtheta.cross(s2.dtheta);
    coning += (1.0/12.0) * (s0.dtheta.cross(s1.dtheta) - s1.dtheta.cross(s2.dtheta));
    
    // Third-order sculling correction
    Vector3d sculling = 0.5 * (s1.dv.cross(s2.dtheta) + s1.dtheta.cross(s2.dv));
    sculling += (1.0/12.0) * (s0.dv.cross(s2.dtheta) - s2.dv.cross(s0.dtheta));
    sculling += (1.0/12.0) * (s0.dtheta.cross(s2.dv) - s2.dtheta.cross(s0.dv));
    
    result.delta_theta = s2.dtheta + coning;
    result.delta_v = s2.dv + sculling;
    
    // Log if corrections are significant
    if (coning.norm() > 1e-5 || sculling.norm() > 1e-5) {
        {
            std::stringstream msg;
            msg << "3rd-order - Coning: " << coning.norm() << " rad, Sculling: " << sculling.norm() << " m/s";
            LOG_DEBUG(msg.str());
        }
    }
    
    return result;
}

} // namespace Navigation