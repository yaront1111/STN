#pragma once

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <vector>

namespace chimera {
namespace utils {

// Compute median of scalar values
double medianScalar(std::vector<double> v);

// Compute median acceleration vector from IMU samples
template<typename ImuSample>
Eigen::Vector3d medianAccel(const std::vector<ImuSample>& imu, double t0, double horizon=0.5);

// Tilt-compensated yaw from magnetometer
double yawFromMagTiltComp(const Eigen::Vector3d& m_b, const gtsam::Rot3& Rwb);

// Compute effective depth for camera given altitude and rotation
double effectiveDepthMeters_AGL(const gtsam::Pose3& x_wb, const gtsam::Matrix3& R_bc, double agl_m);

// Boot orientation from quaternion and median acceleration
struct OrientBoot {
  gtsam::Rot3 Rwb;
  std::string tag;
  double g_err;
};

OrientBoot pickRwb_from_quat(const Eigen::Vector4d& qraw, const Eigen::Vector3d& a_med_b);

// Frame-agnostic variant:
// chooses the candidate quaternion that minimizes || R*a_med_b + g_world || over g_world ∈ { [0,0, +g], [0,0, -g] }.
// Returns the same OrientBoot (Rwb, tag, g_err). 'tag' gets a suffix "/NED" or "/ENU" for diagnostics.
OrientBoot pickRwb_from_quat_agnostic(const Eigen::Vector4d& qraw, const Eigen::Vector3d& a_med_b, double g=9.81);

// Template implementation for medianAccel (must be in header)
template<typename ImuSample>
Eigen::Vector3d medianAccel(const std::vector<ImuSample>& imu, double t0, double horizon){
  std::vector<Eigen::Vector3d> w;
  for(const auto& s: imu){
    if(s.t - t0 > horizon) break;
    w.push_back(s.accel);
  }
  if(w.empty()) return {0,0,-9.81};

  auto medk = [&](int k){
    std::vector<double> v;
    v.reserve(w.size());
    for(auto& a: w) v.push_back(a(k));
    std::nth_element(v.begin(), v.begin()+v.size()/2, v.end());
    return v[v.size()/2];
  };

  return {medk(0), medk(1), medk(2)};
}

}  // namespace utils
}  // namespace chimera
