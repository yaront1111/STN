# STN v0.1 — Space-Time Navigation Prototype

## Vision
A proof-of-concept navigation system that uses spacetime features (gravity, anomalies, relativistic signals) as a positioning reference.

## Phase 1 (Software-only MVP, < $1k)
- **C++ Core**:
  - EKF baseline
  - Strapdown INS
  - Gravity likelihood stub (EGM2020/terrain)
- **Python Tools**:
  - Simulation drivers
  - Visualization (matplotlib)

## Future Phases
- **Phase 2**: Hardware-in-the-loop experiments (low-cost IMU + gravimeter data).
- **Phase 3**: High-resolution gravity anomaly navigation (aircraft).
- **Phase 4**: Scaling to orbital/space navigation.

