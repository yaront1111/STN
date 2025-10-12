#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>

namespace aion {

//--------------- State (Nominal) -----------------

/** Navigation nominal state for AION.
 *  Frames: N = NED, B = body. Quaternion q_bn rotates B→N (body to NED).
 */
struct State {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Position and velocity in N (NED) [m], [m/s]
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};

  // Attitude quaternion q_bn (body→NED)
  Eigen::Quaterniond quaternion{Eigen::Quaterniond::Identity()};

  // IMU errors
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};   // [rad/s]
  Eigen::Vector3d accel_bias{Eigen::Vector3d::Zero()};  // [m/s^2]
  Eigen::Vector3d accel_scale{Eigen::Vector3d::Ones()}; // unitless (diag)

  // Wind in N (x=North,y=East) [m/s]
  Eigen::Vector2d wind{Eigen::Vector2d::Zero()};

  // Timestamp [s]
  double time{0.0};

  // State nominal dimension (for full-state containers; error-state differs)
  static constexpr int DIM = 18; // 3 p + 3 v + 4 q + 3 bg + 3 ba + 2 wind

  void reset();                          // zero p/v/biases, identity q, t=0
  void normalizeAttitude() { quaternion.normalize(); }

  // Rotation helpers
  // Attitude quaternion q_nb (NED -> body)
  inline Eigen::Matrix3d R_nb() const { return quaternion.toRotationMatrix(); }         // nav->body
  inline Eigen::Matrix3d R_bn() const { return quaternion.toRotationMatrix().transpose(); } // body->nav

  // Euler angles (roll, pitch, yaw) from quaternion, radians
  Eigen::Vector3d eulerRPY() const;

  // Backwards-compatible alias (prefer R_nb()/R_bn())
  // getRotationMatrix(): returns R_nb (nav->body)
  Eigen::Matrix3d getRotationMatrix() const { return R_nb(); }
  Eigen::Vector3d getEulerAngles() const { return eulerRPY(); }
};

//--------------- Error-State (Minimal) -----------------

/** Error-state (15+2 DOF). Rotation as small-angle delta-theta in body frame. */
struct ErrorState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static constexpr int DIM = 17; // 3 dp + 3 dv + 3 dtheta + 3 dbg + 3 dba + 2 dwind

  Eigen::Vector3d delta_position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_theta{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_gyro_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_accel_bias{Eigen::Vector3d::Zero()};
  Eigen::Vector2d delta_wind{Eigen::Vector2d::Zero()};

  Eigen::VectorXd toVector() const;
  void fromVector(const Eigen::VectorXd& vec);
  void reset();
};

//--------------- Covariance Wrapper -----------------

class StateCovariance {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateCovariance();

  const Eigen::MatrixXd& matrix() const { return P_; }
  Eigen::MatrixXd& matrix() { return P_; }

  // 3x3 block accessors (const and non-const)
  Eigen::Block<Eigen::MatrixXd,3,3> positionBlock() { return P_.block<3,3>(0,0); }
  Eigen::Block<const Eigen::MatrixXd,3,3> positionBlock() const { return P_.block<3,3>(0,0); }

  Eigen::Block<Eigen::MatrixXd,3,3> velocityBlock() { return P_.block<3,3>(3,3); }
  Eigen::Block<const Eigen::MatrixXd,3,3> velocityBlock() const { return P_.block<3,3>(3,3); }

  Eigen::Block<Eigen::MatrixXd,3,3> attitudeBlock() { return P_.block<3,3>(6,6); }
  Eigen::Block<const Eigen::MatrixXd,3,3> attitudeBlock() const { return P_.block<3,3>(6,6); }

  Eigen::Block<Eigen::MatrixXd,3,3> gyroBiasBlock() { return P_.block<3,3>(9,9); }
  Eigen::Block<const Eigen::MatrixXd,3,3> gyroBiasBlock() const { return P_.block<3,3>(9,9); }

  Eigen::Block<Eigen::MatrixXd,3,3> accelBiasBlock() { return P_.block<3,3>(12,12); }
  Eigen::Block<const Eigen::MatrixXd,3,3> accelBiasBlock() const { return P_.block<3,3>(12,12); }

  Eigen::Block<Eigen::MatrixXd,2,2> windBlock() { return P_.block<2,2>(15,15); }
  Eigen::Block<const Eigen::MatrixXd,2,2> windBlock() const { return P_.block<2,2>(15,15); }

  void initialize(double pos_std = 1.0,
                  double vel_std = 0.1,
                  double att_std = 0.1,
                  double gb_std  = 0.01,
                  double ba_std  = 0.01,
                  double wind_std= 1.0);

  bool isPositiveDefinite() const;
  void enforceSymmetry();          // symmetrize + adaptive jitter

 private:
  Eigen::MatrixXd P_; // 17x17 error-state covariance
};

static_assert(ErrorState::DIM == 17, "ErrorState DIM must be 17");

} // namespace aion
