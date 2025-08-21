#include "ins.h"
#include <algorithm>
using Eigen::Vector3d;

void StrapdownINS::propagate(State& x, const ImuSample& z) {
  // Use per-sample dt for better accuracy
  double dt = std::clamp(z.t - x.t, 1e-4, 0.1);
  if (x.t == 0.0) dt = P.dt;  // First sample, use nominal dt

  // Remove biases
  Vector3d omega_b = z.gyro_rps - x.b_g;
  Vector3d f_b     = z.acc_mps2 - x.b_a;

  // Attitude update (small-angle quaternion)
  Vector3d dtheta = omega_b * dt;
  Eigen::Quaterniond dq(1, 0.5*dtheta.x(), 0.5*dtheta.y(), 0.5*dtheta.z());
  x.q_BN = (x.q_BN * dq).normalized();

  // Transform specific force to NED
  Vector3d f_N = x.q_BN * f_b;

  // Gravity in NED (down is +Z)
  Vector3d g_N(0, 0, P.g);

  // CORRECT PHYSICS: Accelerometer measures specific force (f = a - g)
  // To get true acceleration: a = f + g
  // In level flight: f = [0,0,-g], so a = f + g = [0,0,0] (no acceleration)
  x.v_NED += (f_N + g_N) * dt;
  x.p_NED += x.v_NED * dt;
  x.t += dt;
}
