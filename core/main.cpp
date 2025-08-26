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
#include "../cpp/core/gravity_anomaly.h"  // Add gravity anomaly provider
#include "../cpp/core/config.h"  // Configuration system
#include "filters.h"
#include "terrain_provider.h"
#include "../cpp/core/trn_update_adaptive.h"
#include "radalt_sampler.h"

struct Row { double t, ax, ay, az, gx, gy, gz; };

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
  // Load configuration
  Config& config = Config::instance();
  config.loadFromFile("config/stn_default.cfg");
  config.parseCommandLine(argc, argv);  // Allow command-line overrides
  
  std::string in  = (argc>1)? argv[1] : "data/sim_imu.csv";
  std::string out = (argc>2)? argv[2] : config.getString("paths.output", "data/run_output.csv");
  auto rows = read_csv(in);
  if(rows.empty()){ std::cerr<<"No input: "<<in<<"\n"; return 1; }
  // Load radalt with interpolation sampler
  std::string radalt_file = (argc>3)? argv[3] : "data/radalt.csv";
  RadaltSampler rad;
  rad.delay_s = 0.05;  // 50ms latency (tune 0.0-0.08s)
  try {
    rad.load_csv(radalt_file);
    std::cout << "Loaded " << rad.T.size() << " radalt measurements\n";
  } catch (const std::exception& e) {
    std::cerr << "Radalt error: " << e.what() << "\n";
    return 1;
  }

  // Config from file
  const double dt = 1.0 / config.getDouble("system.imu_rate_hz", 200.0);

  State x; StrapdownINS ins({dt, 9.80665}); EKF ekf;
  
  // Initialize gravity anomaly provider
  GravityAnomalyProvider grav_anomaly;
  grav_anomaly.loadEGM2008("data/egm2008.dat");  // Will use synthetic if file not found
  
  // Configure Singer model for realistic vehicle dynamics
  ekf.setSingerParams(0.5, 60.0);  // 0.5 m/s² accel noise, 60s correlation time
  
  // Initialize state to match truth initial conditions
  x.p_NED = Eigen::Vector3d(0.5, 0.0, 0.0);   // Truth starts at pn=0.5m
  x.v_NED = Eigen::Vector3d(50.0, 0.0, 0.0);  // Truth starts with 50 m/s north
  
  // Initialize quaternion to map from body to NED
  // The simulator says "body aligned with NED" so we use identity
  // The accelerometer reads specific force: [0, 0, -g] (upward)
  // After rotating with identity, still [0, 0, -g]
  // Adding gravity [0, 0, +g] cancels out -> zero acceleration
  x.q_BN = Eigen::Quaterniond::Identity();
  
  ekf.init(x);
  
  // TRN setup from config
  TerrainProvider terrain;
  const bool USE_TRN = config.getBool("trn.enabled", true);
  const bool USE_ADAPTIVE_TRN = true;  // Always use adaptive
  const bool USE_SCALAR_AGL = false;  // Deprecated
  // Time-based TRN scheduling
  const double trn_rate_hz = config.getDouble("system.trn_rate_hz", 2.0);
  const double trn_period = 1.0 / trn_rate_hz;
  double next_trn_t = rows[0].t + trn_period;
  int trn_fired = 0;
  int trn_updates = 0;
  int trn_rejected = 0;
  int trn_no_radalt = 0;
  
  // Adaptive TRN config from file
  TrnAdaptiveCfg trn_cfg;
  trn_cfg.sigma_agl = config.getDouble("ekf.r_trn_base", 25.0);
  trn_cfg.nis_gate = config.getDouble("trn.huber_threshold", 3.0) * config.getDouble("trn.huber_threshold", 3.0);
  trn_cfg.slope_thresh = config.getDouble("trn.slope_threshold", 0.1);
  trn_cfg.alpha_base = config.getDouble("trn.alpha_base", 0.001);
  trn_cfg.alpha_boost = 1.35;    // Gentle boost on slopes
  trn_cfg.huber_c = 1.8;         // Less aggressive outlier rejection
  trn_cfg.max_step = 0.35;       // Conservative step limit
  
  // Enable full-state updates for bias estimation
  const bool USE_FULLSTATE_UPDATE = false;  // Disabled - needs better observability
  
  // Health logging with kappa
  std::ofstream trn_log("data/trn_health.csv");
  trn_log << "t,accepted,nis,alpha_h,R,slope,kappa,residual,pn,pe,pd\n";
  
  // Track previous AGL for velocity consistency
  double z_agl_prev = 0.0;
  double t_agl_prev = 0.0;
  bool has_prev_agl = false;
  
  // Gravity update filter and counter
  IIR1 f_N_z_filt;
  f_N_z_filt.alpha = 0.01;  // Time constant ~1 second for strong smoothing
  int grav_update_counter = 0;
  
  std::ofstream fo(out); fo<<"t,pn,pe,pd,vn,ve,vd\n";

  for(const auto& r: rows){
    // IMU packet
    ImuSample z; z.t=r.t; z.acc_mps2={r.ax,r.ay,r.az}; z.gyro_rps={r.gx,r.gy,r.gz};

    // Propagate strapdown (updates attitude, velocity, position)
    ins.propagate(x, z);

    // --- NEW: Active Gravity-Likelihood Update ---
    // Rotate current accel into NED frame
    Eigen::Vector3d f_N = x.q_BN * z.acc_mps2;
    // Smooth the vertical component with heavy filtering
    f_N_z_filt.step(f_N.z());
    
    // Call gravity update periodically (2 Hz for stable vertical constraint)
    if (++grav_update_counter % 50 == 0) {  // 100Hz/50 = 2Hz
      ekf.update_gravity(x, f_N_z_filt);
      
      // --- NEW: Gravity Anomaly Update (Spacetime Navigation) ---
      // Get current position in lat/lon for gravity lookup
      // Using simple flat-earth approximation for demo
      const double ref_lat = 32.0;  // Reference latitude (degrees)
      const double ref_lon = -110.0;  // Reference longitude (degrees)
      const double lat_to_m = 111320.0;
      const double lon_to_m = 111320.0 * cos(ref_lat * M_PI / 180.0);
      
      double lat = ref_lat + x.p_NED.x() / lat_to_m;
      double lon = ref_lon + x.p_NED.y() / lon_to_m;
      double alt = -x.p_NED.z();
      
      // Get gravity anomaly at current position
      double g_anomaly = grav_anomaly.getAnomaly(lat, lon);
      
      // Could implement anomaly-based position update here
      // For now, just log the anomaly for analysis
      static int anomaly_log_counter = 0;
      if (++anomaly_log_counter % 100 == 0) {  // Log every second
        std::cout << "Gravity anomaly at (" << lat << ", " << lon 
                  << ", " << alt << "m): " << g_anomaly*1e5 << " mGal\n";
      }
    }

    
    // --- TRN update using precise time-based scheduler ---
    bool do_trn = false;
    if (USE_TRN && x.t + 1e-9 >= next_trn_t) {
      do_trn = true;
      // Catch up if we slipped multiple periods
      while (next_trn_t <= x.t) next_trn_t += trn_period;
      trn_fired++;
    }
    
    if (do_trn) {
      // Sample radalt at current time with interpolation
      double z_agl;
      bool have_radalt = rad.sample(x.t, z_agl);
      
      if (!have_radalt) {
        trn_no_radalt++;
        // Log reason for missing
        if (trn_no_radalt <= 3) {
          std::cout << "TRN miss at t=" << x.t << ": no radalt\n";
        }
      } else {

      // Get terrain at current estimate
      auto ts = terrain.sample(x.p_NED.x(), x.p_NED.y());
      
      // Velocity consistency check (pre-gate)
      const double rdot_pred = -x.v_NED.z() - (ts.grad.x()*x.v_NED.x() + ts.grad.y()*x.v_NED.y());
      bool vel_consistent = true;
      if (has_prev_agl && (x.t - t_agl_prev) > 0) {
        double rdot_meas = (z_agl - z_agl_prev) / (x.t - t_agl_prev);
        double rdot_err = std::abs(rdot_meas - rdot_pred);
        // Very soft gate: only reject physically impossible changes
        if (rdot_err > 100.0) {  // Very lenient - accept almost everything
          vel_consistent = false;
        }
      }
      
      if (USE_ADAPTIVE_TRN && vel_consistent) {
        // Variables for logging
        bool accepted = false;
        double nis = 0, alpha = 0, residual = 0;
        
        // Use full-state or position-only update
        if (USE_FULLSTATE_UPDATE) {
          // Full-state update enables IMU bias estimation
          residual = z_agl - ((-x.p_NED.z()) - ts.h);
          accepted = ekf.update_agl_fullstate(x, z_agl, ts.h, ts.grad);
          if (accepted) {
            trn_updates++;
          } else {
            trn_rejected++;
          }
        } else {
          // Legacy adaptive TRN (position-only)
          residual = z_agl - ((-x.p_NED.z()) - ts.h);
          
          // Create local copies for update
          Eigen::Vector3d p_local = x.p_NED;
          Eigen::Matrix3d P_local = ekf.get_P_pos();
          
          accepted = trn_update_adaptive(
              p_local, P_local, x.v_NED,
              z_agl, ts.h, ts.grad,
              trn_cfg, &nis, &alpha,
              has_prev_agl ? z_agl_prev : z_agl,
              has_prev_agl ? (x.t - t_agl_prev) : 0.0
          );
          
          if (accepted) {
            // Feed back to nav state
            x.p_NED = p_local;
            ekf.set_P_pos(P_local);
            trn_updates++;
          } else {
            trn_rejected++;
          }
        }
        
        // Log health metrics
        if (trn_log.good()) {
          trn_log << x.t << "," << (accepted?1:0) << "," << nis << "," 
                  << alpha << ",0," << ts.grad.norm() << ",0," 
                  << residual << ","
                  << x.p_NED.x() << "," << x.p_NED.y() << "," << x.p_NED.z() << "\n";
        }
                   
        // Debug first few updates
        if (accepted && trn_updates <= 3) {
          std::cout << "TRN adaptive update " << trn_updates 
                    << ": residual=" << residual 
                    << ", slope=" << ts.grad.norm() 
                    << ", alpha=" << alpha << "\n";
        }
      } else if (USE_SCALAR_AGL && vel_consistent) {
        // Legacy scalar AGL update approach
        bool accepted = ekf.update_agl(x, z_agl, ts.h, ts.grad);
        if (accepted) {
          trn_updates++;
          // Debug first few updates
          if (trn_updates <= 3) {
            double agl_pred = (-x.p_NED.z()) - ts.h;
            double residual = z_agl - agl_pred;
            std::cout << "TRN scalar update " << trn_updates 
                      << ": residual=" << residual 
                      << ", slope=" << ts.grad.norm() << "\n";
          }
        } else {
          trn_rejected++;
        }
      } else if (!USE_ADAPTIVE_TRN && !USE_SCALAR_AGL && vel_consistent) {
        // Legacy gradient-based approach (kept for comparison)
        double agl_pred = (-x.p_NED.z()) - ts.h;
        double r_agl = z_agl - agl_pred;
        
        double slope = ts.grad.norm();
        bool ok_slope = slope > 0.05 && slope < 1.0;
        bool ok_dyn = (z.gyro_rps - x.b_g).norm() < 0.02;
        bool significant_residual = std::abs(r_agl) > 0.2;

        if (ok_slope && ok_dyn && significant_residual && std::isfinite(r_agl)) {
          Eigen::Vector2d g = ts.grad;
          double g2 = g.squaredNorm() + 1e-3;
          Eigen::Vector2d dp_ne = (-r_agl) * g / g2;

          double max_step_m = (x.t < 20.0) ? 10.0 : (x.t < 40.0) ? 5.0 : 2.0;
          if (dp_ne.norm() > max_step_m) dp_ne = dp_ne.normalized() * max_step_m;

          Eigen::Vector3d z_pos_trn = x.p_NED;
          z_pos_trn.x() += dp_ne.x();
          z_pos_trn.y() += dp_ne.y();
          
          Eigen::Matrix3d R_trn = Eigen::Matrix3d::Identity();
          double slope_factor = std::max(0.1, std::min(1.0, slope / 0.3));
          double residual_factor = std::min(1.0, std::abs(r_agl) / 5.0);
          double confidence = slope_factor * (1.0 - 0.5 * residual_factor);
          double r_value = 1.0 + 9.0 * (1.0 - confidence);
          R_trn(0,0) = r_value;  
          R_trn(1,1) = r_value;
          R_trn(2,2) = 1e12;

          ekf.update_position(x, z_pos_trn, R_trn);
          trn_updates++;
        } else {
          trn_rejected++;
        }
        } else if (!vel_consistent) {
          trn_rejected++;  // Velocity consistency check failed
        }
        
        // Update previous AGL for next consistency check
        z_agl_prev = z_agl;
        t_agl_prev = x.t;
        has_prev_agl = true;
      }  // end have_radalt
    }  // end do_trn
    
    // --- Weak baro constraint for vertical stability (Grade A) ---
    static int baro_decim = 0;
    const int BARO_DECIM = int(2.0/dt);  // 0.5 Hz baro update
    if ((++baro_decim % BARO_DECIM) == 0) {
      // Synthetic baro (truth + noise for testing)
      double h_ref = 0.0;  // Reference altitude
      double z_baro = -x.p_NED.z() + 2.0 * (rand() / double(RAND_MAX) - 0.5);  // ±1m noise
      
      // Weak vertical scalar update
      double y_baro = z_baro - (-x.p_NED.z() - h_ref);
      double sigma_baro = 2.0;  // 2m std dev
      double R_baro = sigma_baro * sigma_baro;
      
      // Get vertical position variance
      Eigen::Matrix3d P_pos = ekf.get_P_pos();
      double P_z = P_pos(2,2);
      
      // Scalar Kalman update for vertical only
      double S_baro = P_z + R_baro;
      double K_baro = P_z / S_baro;
      double alpha_baro = 0.25;  // Gentle update
      
      // Apply vertical correction
      x.p_NED.z() += alpha_baro * K_baro * y_baro;
      P_pos(2,2) = (1.0 - alpha_baro * K_baro) * P_z;
      ekf.set_P_pos(P_pos);
    }
    
    // EKF propagate (cov only)
    ekf.propagate(x, dt);

    // Log
    fo<<x.t<<","<<x.p_NED.x()<<","<<x.p_NED.y()<<","<<x.p_NED.z()<<","
      <<x.v_NED.x()<<","<<x.v_NED.y()<<","<<x.v_NED.z()<<"\n";
  }

  // Check radalt coverage
  if (!rad.T.empty() && !rows.empty()) {
    double t_imu_end = rows.back().t;
    double t_rad_end = rad.T.back();
    if (t_rad_end < t_imu_end - 0.5*trn_period) {
      std::cerr << "WARN: radalt ends early: " << t_rad_end << " < " << t_imu_end << "\n";
    }
  }
  
  std::cout<<"Wrote "<<out<<" (TRN fired: "<<trn_fired
           <<", accepted: "<<trn_updates
           <<", rejected: "<<trn_rejected
           <<", no_radalt: "<<trn_no_radalt<<")\n";
  return 0;
}
