#pragma once

#include "processors/i_range_altimeter_processor.h"

namespace chimera {
namespace processors {

// =================== Range Sensor Fusion Policy ===================
// Handles intelligent switching/fusion between multiple range altimeters (LiDAR, ToF)
class RangeFusionPolicy {
public:
  enum class FusionMode {
    LIDAR_ONLY,      // Use only LiDAR
    TOF_ONLY,        // Use only ToF
    TOF_PRIMARY,     // Use ToF if available, fallback to LiDAR
    LIDAR_PRIMARY,   // Use LiDAR if available, fallback to ToF
    WEIGHTED_FUSION, // Use both with confidence-weighted factors
    RANGE_BASED      // Switch based on altitude (ToF < 4m, LiDAR > 4m)
  };

  struct Config {
    FusionMode mode = FusionMode::LIDAR_PRIMARY;

    // Range-based switching thresholds
    double tof_max_altitude = 3.5;    // Above this, prefer LiDAR
    double lidar_min_altitude = 0.5;  // Below this, prefer ToF (LiDAR too close)

    // Weighted fusion parameters
    double min_confidence_for_fusion = 0.5;
    double confidence_weight_exponent = 2.0;  // Higher = more aggressive weighting

    // Fallback behavior
    bool allow_stuck_fallback = true;  // If primary stuck, use secondary
  };

  RangeFusionPolicy(const Config& cfg) : cfg_(cfg) {}

  // Determine which sensor(s) to use based on policy and sensor states
  struct FusionDecision {
    bool use_lidar = false;
    bool use_tof = false;
    double lidar_weight = 1.0;  // For weighted fusion
    double tof_weight = 1.0;    // For weighted fusion
    const char* reason = "";    // For debugging/logging
  };

  FusionDecision decide(
    IRangeAltimeterProcessor* lidar,
    IRangeAltimeterProcessor* tof,
    const SensorProcessingContext& ctx,
    double current_altitude_estimate
  ) {
    FusionDecision decision;

    // Check sensor availability and health
    const bool lidar_available = lidar && lidar->isAvailable(ctx.time_absolute);
    const bool tof_available = tof && tof->isAvailable(ctx.time_absolute);

    const bool lidar_stuck = lidar && lidar->isStuck();
    const bool tof_stuck = tof && tof->isStuck();

    const bool lidar_healthy = lidar_available && !lidar_stuck;
    const bool tof_healthy = tof_available && !tof_stuck;

    // Apply fusion policy
    switch(cfg_.mode){
      case FusionMode::LIDAR_ONLY:
        decision.use_lidar = lidar_healthy;
        decision.reason = "LIDAR_ONLY mode";
        break;

      case FusionMode::TOF_ONLY:
        decision.use_tof = tof_healthy;
        decision.reason = "TOF_ONLY mode";
        break;

      case FusionMode::TOF_PRIMARY:
        if(tof_healthy){
          decision.use_tof = true;
          decision.reason = "ToF primary (healthy)";
        } else if(cfg_.allow_stuck_fallback && lidar_healthy){
          decision.use_lidar = true;
          decision.reason = "ToF unavailable, fallback to LiDAR";
        }
        break;

      case FusionMode::LIDAR_PRIMARY:
        if(lidar_healthy){
          decision.use_lidar = true;
          decision.reason = "LiDAR primary (healthy)";
        } else if(cfg_.allow_stuck_fallback && tof_healthy){
          decision.use_tof = true;
          decision.reason = "LiDAR unavailable, fallback to ToF";
        }
        break;

      case FusionMode::WEIGHTED_FUSION:
        if(lidar_healthy && tof_healthy){
          const double lidar_conf = lidar->getConfidence();
          const double tof_conf = tof->getConfidence();

          if(lidar_conf >= cfg_.min_confidence_for_fusion &&
             tof_conf >= cfg_.min_confidence_for_fusion){
            decision.use_lidar = true;
            decision.use_tof = true;

            // Confidence-based weighting
            const double lw = std::pow(lidar_conf, cfg_.confidence_weight_exponent);
            const double tw = std::pow(tof_conf, cfg_.confidence_weight_exponent);
            const double sum = lw + tw;
            decision.lidar_weight = lw / sum;
            decision.tof_weight = tw / sum;
            decision.reason = "Weighted fusion (both healthy)";
          } else if(lidar_conf >= cfg_.min_confidence_for_fusion){
            decision.use_lidar = true;
            decision.reason = "Weighted fusion (ToF low confidence)";
          } else if(tof_conf >= cfg_.min_confidence_for_fusion){
            decision.use_tof = true;
            decision.reason = "Weighted fusion (LiDAR low confidence)";
          }
        } else if(lidar_healthy){
          decision.use_lidar = true;
          decision.reason = "Weighted fusion (ToF unavailable)";
        } else if(tof_healthy){
          decision.use_tof = true;
          decision.reason = "Weighted fusion (LiDAR unavailable)";
        }
        break;

      case FusionMode::RANGE_BASED:
        {
          const double alt = std::abs(current_altitude_estimate);

          // Close to ground: prefer ToF (LiDAR may have min-range issues)
          if(alt < cfg_.lidar_min_altitude){
            if(tof_healthy){
              decision.use_tof = true;
              decision.reason = "Range-based: too close for LiDAR";
            } else if(lidar_healthy){
              decision.use_lidar = true;
              decision.reason = "Range-based: ToF unavailable at close range";
            }
          }
          // High altitude: prefer LiDAR (ToF out of range)
          else if(alt > cfg_.tof_max_altitude){
            if(lidar_healthy){
              decision.use_lidar = true;
              decision.reason = "Range-based: above ToF max range";
            } else if(tof_healthy){
              decision.use_tof = true;
              decision.reason = "Range-based: LiDAR unavailable at high alt";
            }
          }
          // Mid-range: use best available
          else {
            if(lidar_healthy && tof_healthy){
              // Both available - use LiDAR as primary (typically more accurate)
              decision.use_lidar = true;
              decision.reason = "Range-based: mid-range (LiDAR primary)";
            } else if(lidar_healthy){
              decision.use_lidar = true;
              decision.reason = "Range-based: mid-range (ToF unavailable)";
            } else if(tof_healthy){
              decision.use_tof = true;
              decision.reason = "Range-based: mid-range (LiDAR unavailable)";
            }
          }
        }
        break;
    }

    return decision;
  }

private:
  Config cfg_;
};

}  // namespace processors
}  // namespace chimera
