Review sensor fusion implementation for a specific sensor.

When using this command, the user will specify which sensor to review (IMU, Mag, Baro, LiDAR, OF, or Airspeed).

For each sensor, review:

1. **Data Loading** (apps/chimera_node_multi.cpp):
   - JSON parsing correctness
   - Unit conversions applied
   - Frame transformations

2. **Factor Implementation** (include/chimera/factors/):
   - Error function correctness
   - Jacobian implementation (analytical vs numerical)
   - Noise model appropriateness

3. **Integration** (apps/chimera_node_multi.cpp):
   - When factor is added (gating logic)
   - Noise scheduling (tight→loose or vice versa)
   - Interaction with other sensors

4. **Common Issues**:
   - IMU: Units (deg/s vs rad/s), frame flips, bias priors
   - Mag: Frame alignment, tilt compensation, declination
   - Baro: MSL→AGL rebasing, early tightening
   - LiDAR: Stuck detection, z-sign convention
   - OF: Depth scaling, px/s conversion, ray gating
   - Airspeed: Wind coupling, enable timing

5. **Suggest Improvements**:
   - Better gating
   - Adaptive noise
   - Robustness checks
