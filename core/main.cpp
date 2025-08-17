#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

#include "types.h"
#include "ins.h"
#include "ekf.h"
#include "gravity_model.h"
#include "filters.h"
#include "terrain_provider.h"

struct Row { double t, ax, ay, az, gx, gy, gz; };

struct RadaltRow { double t; double agl; };

static std::vector<RadaltRow> read_radalt(const std::string& path){
  std::ifstream f(path); std::string line;
  std::vector<RadaltRow> v; if(!f.good()) return v;
  std::getline(f,line); // header
  while(std::getline(f,line)){
    std::stringstream ss(line); std::string tok; RadaltRow r{};
    std::getline(ss,tok,','); r.t = std::stod(tok);
    std::getline(ss,tok,','); r.agl = std::stod(tok);
    v.push_back(r);
  }
  return v;
}

static std::vector<Row> read_csv(const std::string& path) {
  std::ifstream f(path); std::string line;
  std::vector<Row> v; if(!f.good()) return v;
  std::getline(f,line); // header
  while(std::getline(f,line)) {
    std::stringstream ss(line); std::string tok; Row r{};
    std::getline(ss,tok,','); r.t  = std::stod(tok);
    std::getline(ss,tok,','); r.ax = std::stod(tok);
    std::getline(ss,tok,','); r.ay = std::stod(tok);
    std::getline(ss,tok,','); r.az = std::stod(tok);
    std::getline(ss,tok,','); r.gx = std::stod(tok);
    std::getline(ss,tok,','); r.gy = std::stod(tok);
    std::getline(ss,tok,','); r.gz = std::stod(tok);
    v.push_back(r);
  }
  return v;
}

int main(int argc, char** argv) {
  std::string in  = (argc>1)? argv[1] : "data/sim_imu.csv";
  std::string out = (argc>2)? argv[2] : "data/run_output.csv";
  auto rows = read_csv(in);
  if(rows.empty()){ std::cerr<<"No input: "<<in<<"\n"; return 1; }
  auto radalt = read_radalt("data/radalt.csv");
  size_t ir=0;
  std::cout << "Loaded " << radalt.size() << " radalt measurements\n";

  // Config (v0.1 constants)
  const double dt = 0.01;
  const double lat_rad = 32.0 * M_PI/180.0;  // pick a reasonable latitude
  const double FREE_AIR = 3.086e-6;          // m/s^2 per meter (gravity decreases with altitude)

  State x; StrapdownINS ins({dt, 9.80665}); EKF ekf;
  
  // Initialize state to match truth initial conditions better
  x.v_NED = Eigen::Vector3d(50.0, 0.0, 0.0);  // Truth starts with 50 m/s north
  
  // Initialize quaternion to map from body to NED
  // The simulator says "body aligned with NED" so we use identity
  // The accelerometer reads specific force: [0, 0, -g] (upward)
  // After rotating with identity, still [0, 0, -g]
  // Adding gravity [0, 0, +g] cancels out -> zero acceleration
  x.q_BN = Eigen::Quaterniond::Identity();
  
  ekf.init(x);
  
  // TRN setup
  TerrainProvider terrain;
  constexpr bool USE_TRN = true;   // toggle TRN fusion
  const int DECIM_HZ = int(1.0/dt);
  int decim_trn = 0;
  int trn_updates = 0;
  
  // Filters for smooth residuals
  IIR1 filt_g_obs_z;  filt_g_obs_z.alpha = dt/2.0;  // tau≈2s
  IIR1 filt_g_pred_z; filt_g_pred_z.alpha = dt/2.0;
  
  std::ofstream fo(out); fo<<"t,pn,pe,pd,vn,ve,vd\n";

  // Keep previous velocity for finite-difference acceleration
  Eigen::Vector3d v_prev = x.v_NED;

  for(const auto& r: rows){
    // IMU packet
    ImuSample z; z.t=r.t; z.acc_mps2={r.ax,r.ay,r.az}; z.gyro_rps={r.gx,r.gy,r.gz};

    // Propagate strapdown (updates attitude, velocity, position)
    ins.propagate(x, z);

    // --- Gravity likelihood (vertical, gated + filtered + 1 Hz) ---
    static constexpr bool USE_GRAVITY_LIKELIHOOD = false;  // Keep OFF by default (enable when needed)
    
    Eigen::Vector3d z_pos = x.p_NED;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    R(0,0) = 1e12; R(1,1) = 1e12; R(2,2) = 1e12; // default: ignore (no-op)
    
    static int decim = 0; 
    const int DECIM = int(1.0/dt); // 1 Hz update rate
    
    // Recompute specific force in NED with current attitude
    Eigen::Vector3d f_N = x.q_BN * (z.acc_mps2 - x.b_a);
    double fmag = f_N.norm();
    double gyro_norm = (z.gyro_rps - x.b_g).norm();
    
    // Predicted gravity at (lat, alt)
    double alt_m = -x.p_NED.z();
    GravityResult gr = GravityModel::normal({lat_rad, alt_m});
    
    // Observed gravity (accel) in NED ~ -specific_force when a≈0
    double g_obs_z_raw = -f_N.z();
    double g_pred_z_raw = gr.g_NED.z();
    double g_obs_z = filt_g_obs_z.step(g_obs_z_raw);
    double g_pred_z = filt_g_pred_z.step(g_pred_z_raw);
    double r_down = g_obs_z - g_pred_z; // m/s^2
    
    // Gating: quasi-static only
    bool quasi_static = (std::abs(fmag - gr.g_mps2) < 0.05) && (gyro_norm < 0.01);
    const double gate_mps2 = 0.05; // residual gate
    
    if (USE_GRAVITY_LIKELIHOOD && quasi_static && std::abs(r_down) < gate_mps2 && (++decim % DECIM) == 0) {
      double dz_est = (g_pred_z - g_obs_z) / FREE_AIR; // altitude correction (m)
      dz_est = std::clamp(dz_est, -0.5, 0.5); // at most 0.5 m per update
      z_pos.z() = x.p_NED.z() - dz_est;  // NED: down = -alt
      R(2,2) = 100.0; // cautious weight
    }
    
    // --- TRN update (1 Hz) using AGL residual -> horizontal nudge along terrain slope ---
    if (USE_TRN && (++decim_trn % DECIM_HZ) == 0 && ir < radalt.size()) {
      // Interpolate AGL at current time r.t for better time alignment
      while (ir+1 < radalt.size() && radalt[ir+1].t < r.t) ++ir;
      double z_agl;
      if (ir+1 < radalt.size()) {
        double t0 = radalt[ir].t, t1 = radalt[ir+1].t;
        double a0 = radalt[ir].agl, a1 = radalt[ir+1].agl;
        double alpha = (r.t - t0) / std::max(1e-6, t1 - t0);
        alpha = std::clamp(alpha, 0.0, 1.0);
        z_agl = a0 + alpha * (a1 - a0);
      } else {
        z_agl = radalt[ir].agl;  // Use last available
      }

      // terrain at current estimate
      auto ts = terrain.sample(x.p_NED.x(), x.p_NED.y());
      double agl_pred = (-x.p_NED.z()) - ts.h;   // alt - terrain
      double r_agl = z_agl - agl_pred;           // positive means we think we're higher AGL than map

      // gate: need slope for observability, and limit dynamics (simple gyro gate via z.gyro_rps)
      double slope = ts.grad.norm();              // m/m
      bool ok_slope = slope > 0.01 && slope < 2.0;  // Relaxed slope gate
      bool ok_dyn   = (z.gyro_rps - x.b_g).norm() < 0.1; // Relaxed dynamics gate

      if (ok_slope && ok_dyn && std::isfinite(r_agl)) {
        // Convert AGL residual to horizontal correction along gradient direction.
        // Linearization: delta h ≈ -grad^T * delta_p_NE (holding altitude)
        // We need delta_p_NE such that delta h = r_agl => -grad^T * dp = r_agl
        Eigen::Vector2d g = ts.grad; double g2 = g.squaredNorm() + 1e-3;
        Eigen::Vector2d dp_ne = (-r_agl) * g / g2;

        // Step clamp schedule: aggressive at start, then conservative
        double max_step_m;
        if (x.t < 20.0) {
          max_step_m = 50.0;  // First 20s: allow large corrections for capture
        } else if (x.t < 30.0) {
          max_step_m = 10.0;  // 20-30s: medium corrections
        } else {
          max_step_m = 3.0;   // After 30s: conservative corrections for stability
        }
        
        // Clamp correction
        if (dp_ne.norm() > max_step_m) dp_ne = dp_ne.normalized() * max_step_m;
        
        // Debug first few updates
        if (trn_updates < 3) {
          std::cout << "TRN update " << trn_updates << ": r_agl=" << r_agl 
                    << ", slope=" << slope << ", dp=[" << dp_ne.x() << "," << dp_ne.y() << "]\n";
        }

        // Build a pseudo position measurement that nudges N/E only
        Eigen::Vector3d z_pos_trn = x.p_NED;
        z_pos_trn.x() += dp_ne.x();
        z_pos_trn.y() += dp_ne.y();
        // Adaptive measurement confidence based on terrain slope
        Eigen::Matrix3d R_trn = Eigen::Matrix3d::Identity();
        double r_base = 1.0 / (1.0 + 10.0 * slope * slope);  // Higher slope = better observability
        R_trn(0,0) = r_base;  
        R_trn(1,1) = r_base;
        R_trn(2,2) = 1e12;  // do not change Down here

        // Apply
        ekf.update_position(x, z_pos_trn, R_trn);
        trn_updates++;
      }
    }
    
    // EKF propagate (cov) + position update
    ekf.propagate(x, dt);
    ekf.update_position(x, z_pos, R);

    // Log
    fo<<x.t<<","<<x.p_NED.x()<<","<<x.p_NED.y()<<","<<x.p_NED.z()<<","
      <<x.v_NED.x()<<","<<x.v_NED.y()<<","<<x.v_NED.z()<<"\n";
  }

  std::cout<<"Wrote "<<out<<" (TRN updates: "<<trn_updates<<")\n";
  return 0;
}
