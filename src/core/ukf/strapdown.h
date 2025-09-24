/**
 * Strapdown Mechanization Header
 * Coning and sculling compensation for INS
 */

#pragma once

#include <Eigen/Dense>
#include <deque>
#include "../../utils/math_utils.h"

namespace Navigation {

using namespace NavMath;

/**
 * Basic strapdown mechanization with 2-sample compensation
 */
class StrapdownMechanization {
public:
    struct CompensatedIMU {
        Vector3d delta_v;      // Compensated velocity increment
        Vector3d delta_theta;  // Compensated angle increment
        double dt;
    };
    
    StrapdownMechanization();
    virtual ~StrapdownMechanization() = default;
    
    // Main compensation function
    virtual CompensatedIMU compensate(const Vector3d& accel, const Vector3d& gyro, double dt);
    
    // Reset mechanization
    virtual void reset();
    
protected:
    // Previous IMU samples for compensation
    Vector3d prev_accel_;
    Vector3d prev_gyro_;
    Vector3d prev_dv_;
    Vector3d prev_dtheta_;
    bool initialized_ = false;
    
    // Compensation algorithms
    Vector3d coningCompensation(const Vector3d& dtheta_k, const Vector3d& dtheta_km1);
    Vector3d scullingCompensation(const Vector3d& dv_k, const Vector3d& dtheta_k,
                                 const Vector3d& dv_km1, const Vector3d& dtheta_km1);
};

/**
 * Extended mechanization with higher-order compensation
 * For high-dynamic applications (fighter jets, missiles)
 */
class ExtendedMechanization : public StrapdownMechanization {
public:
    ExtendedMechanization();
    
    CompensatedIMU compensate(const Vector3d& accel, const Vector3d& gyro, double dt) override;
    void reset() override;
    
private:
    struct IMUSample {
        Vector3d accel;
        Vector3d gyro;
        Vector3d dv;
        Vector3d dtheta;
        double dt;
    };
    
    std::deque<IMUSample> sample_buffer_;
    static constexpr size_t MAX_BUFFER_SIZE = 17;
    
    // Higher-order compensation
    CompensatedIMU secondOrderCompensation();
    CompensatedIMU thirdOrderCompensation();
};

} // namespace Navigation