#include "utils/sensor_helpers.h"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace chimera {
namespace utils {

double medianScalar(std::vector<double> v){
  if(v.empty()) return 0.0;
  std::nth_element(v.begin(), v.begin()+v.size()/2, v.end());
  return v[v.size()/2];
}

double yawFromMagTiltComp(const Eigen::Vector3d& m_b, const gtsam::Rot3& Rwb){
  Eigen::Vector3d m_w = Rwb.matrix() * m_b;
  return std::atan2(m_w.y(), m_w.x());
}

double effectiveDepthMeters_AGL(const gtsam::Pose3& x_wb, const gtsam::Matrix3& R_bc, double agl_m){
  const gtsam::Matrix3 R_wc = x_wb.rotation().matrix() * R_bc;
  const Eigen::Vector3d ez_cam_world = R_wc.col(2);
  const Eigen::Vector3d up(0, 0, 1);
  const double cosi = std::abs(up.dot(ez_cam_world));
  const double clamp = std::max(0.15, std::min(1.0, cosi));
  return agl_m / clamp;
}

OrientBoot pickRwb_from_quat(const Eigen::Vector4d& qraw, const Eigen::Vector3d& a_med_b){
  auto mk = [&](double w, double x, double y, double z){
    return gtsam::Rot3::Quaternion(w, x, y, z);
  };

  struct Candidate { gtsam::Rot3 R; const char* tag; };
  std::vector<Candidate> candidates = {
    { mk(qraw(0), qraw(1), qraw(2), qraw(3)),           "wxyz"     },
    { mk(qraw(0), qraw(1), qraw(2), qraw(3)).inverse(), "wxyz_inv" },
    { mk(qraw(3), qraw(0), qraw(1), qraw(2)),           "xyzw"     },
    { mk(qraw(3), qraw(0), qraw(1), qraw(2)).inverse(), "xyzw_inv" },
  };

  double best_err = 1e9;
  gtsam::Rot3 best_R;
  const char* best_tag = "";

  for(auto& c: candidates){
    Eigen::Vector3d aw = c.R.matrix() * a_med_b;
    // NED expectation: R*a_b ≈ -g_world, g_world = [0,0,+g]
    const double err = (aw + Eigen::Vector3d(0, 0, 9.81)).norm();
    if(err < best_err){ best_err = err; best_R = c.R; best_tag = c.tag; }
  }
  std::cout << "[ORIENT BOOT] pick " << best_tag
            << " g_err=" << best_err << " m/s^2" << std::endl;
  return {best_R, best_tag, best_err};
}

OrientBoot pickRwb_from_quat_agnostic(const Eigen::Vector4d& qraw, const Eigen::Vector3d& a_med_b, double g){
  auto mk = [&](double w, double x, double y, double z){
    return gtsam::Rot3::Quaternion(w, x, y, z);
  };
  struct Candidate { gtsam::Rot3 R; const char* tag; };
  std::vector<Candidate> candidates = {
    { mk(qraw(0), qraw(1), qraw(2), qraw(3)),           "wxyz"     },
    { mk(qraw(0), qraw(1), qraw(2), qraw(3)).inverse(), "wxyz_inv" },
    { mk(qraw(3), qraw(0), qraw(1), qraw(2)),           "xyzw"     },
    { mk(qraw(3), qraw(0), qraw(1), qraw(2)).inverse(), "xyzw_inv" },
  };

  const Eigen::Vector3d gNED(0,0, +g);
  const Eigen::Vector3d gENU(0,0, -g);
  auto err_for = [&](const gtsam::Rot3& R, const Eigen::Vector3d& gvec){
    // residual ‖R*a_b + g_world‖ — at rest specific force should cancel gravity in world
    return (R.matrix() * a_med_b + gvec).norm();
  };

  double best_err = 1e9;
  gtsam::Rot3 best_R;
  std::string best_tag;

  for(auto& c: candidates){
    double eNED = err_for(c.R, gNED);
    double eENU = err_for(c.R, gENU);
    double e    = std::min(eNED, eENU);
    std::string suffix = (eNED <= eENU) ? "/NED" : "/ENU";
    if(e < best_err){
      best_err = e;
      best_R   = c.R;
      best_tag = std::string(c.tag) + suffix;
    }
  }
  std::cout << "[ORIENT BOOT] pick " << best_tag
            << " g_err=" << best_err << " m/s^2" << std::endl;
  return {best_R, best_tag, best_err};
}

}  // namespace utils
}  // namespace chimera
