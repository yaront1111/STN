Debug and fix filter divergence issues (position explosions, velocity runaway, etc.).

When filter diverges, systematically check:

1. **Sensor Units** (most common cause):
   - Run /check-units to verify all sensors
   - Look for 57× or 33× mismatches (deg/rad signature)
   - Check mag-gyro ratio (should be ~1.0 if both in rad/s)

2. **Frame Alignment**:
   - Verify IMU flip (NWU→FRD) if needed
   - Check mag uses same flip as IMU
   - Verify world frame (Z-up vs Z-down)

3. **Factor Gating**:
   - Check which sensors are active (LiDAR stuck? OF rejected?)
   - Verify altitude observability (baro enabled early?)
   - Check velocity constraints (OF + airspeed working?)

4. **Noise Models**:
   - Too tight → filter overconfident → diverges
   - Too loose → sensor ignored → drifts
   - Check IMU covariances vs sensor noise

5. **Initial Conditions**:
   - Orientation boot correct? (gravity alignment < 1 m/s²)
   - Bias priors reasonable? (gyro < 0.1 rad/s, accel < 0.5 m/s²)

6. **Systematic Isolation**:
   - Disable optical flow → still diverges?
   - Disable magnetometer → still diverges?
   - Disable airspeed → still diverges?
   - Find the culprit sensor

Report findings and suggest targeted fixes.
