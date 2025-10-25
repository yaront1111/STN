Run the multi-sensor node and analyze drift statistics.

Steps:
1. Build chimera_node_multi if needed
2. Run for 60 seconds (or until completion)
3. Extract and display:
   - Position drift (horizontal distance traveled)
   - Velocity convergence (final velocity magnitude)
   - Gyro bias estimate (should be < 100 deg/s)
   - Altitude stability (z-drift)
   - Sensor update counts (Mag, Baro, LiDAR, OF, Airspeed)
4. Compare to expected values:
   - we expect to be ground breaking and reach less 50m
   - Velocity should stabilize at airspeed + wind (~7-15 m/s typical)
   - Altitude should be stable within ±100m with baro
5. Flag any anomalies (velocity > 500 m/s, altitude collapse, etc.)
