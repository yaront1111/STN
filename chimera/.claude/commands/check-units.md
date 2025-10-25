Verify sensor units are correct throughout the codebase.

Steps:
1. Check IMU data loading:
   - Gyro: Should be rad/s (typical values: 0.001-0.1 rad/s for stationary)
   - Accel: Should be m/s² (norm ≈ 9.81 m/s² when stationary)
2. Check conversion paths:
   - Search for deg→rad conversions (M_PI/180)
   - Verify conversions happen exactly once
   - Check for frame flips (NWU→FRD) applied consistently
3. Check Magnetometer:
   - Frame alignment with IMU
   - Magnitude normalization (if any)
4. Check Optical Flow:
   - Verify px/s (not px/frame) - should divide by dt
5. Check Airspeed:
   - Verify m/s (not mph, knots, km/h)
   - Typical values: 5-25 m/s for UAV
6. Print first 3 samples of each sensor with units labeled
