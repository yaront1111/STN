Perfect—here’s a **start-to-finish execution plan** focused **only** on **ground, GPS-denied STN (Signals-of-Opportunity/Terrestrial) positioning**.
Each step has **two sentences** and a **Test** line.

1. **Repo bootstrap & toolchain**
   Initialize the repo, pin compilers, and install CMake, Eigen, yaml-cpp, GTest, and pre-commit. Add a top-level `Makefile` with `make build test format`.
   **Test:** `cmake -S . -B build && cmake --build build -j && pre-commit run -a`

2. **Logging + config loader**
   Wire `Logger::init()` and YAML parsing with `--config` CLI, emitting all STN params on startup. Include `configs/stn_default.yaml` with receiver, beacons, and noise.
   **Test:** `./build/apps/aion_node --config configs/stn_default.yaml --log-level debug`

3. **Frames, map datum, and units contract**
   Lock **NED** with WGS84 geodetic conversions and meters/radians everywhere; document in `docs/frames.md`. Provide helpers for (lat,lon,alt) ↔ NED around a fixed origin.
   **Test:** `ctest -R frames_conv` (verifies round-trip error < 1e-6)

4. **Monotonic timebase & clock model**
   Add a monotonic clock wrapper and a **receiver clock state** (bias, drift) to the estimator. Expose `t_meas` vs `t_recv` in all STN messages.
   **Test:** `ctest -R timebase` (timestamps strictly increasing; bias/drift propagate)

5. **State + UKF skeleton (STN-only)**
   Define state = {pos, vel, att (optional yaw only for 2D), **clock bias**, **clock drift**}; start with 2D pos + height from config for ground trials. Run a 200 Hz predict with IMU optional (can be disabled).
   **Test:** `ctest -R ukf_init` (state initializes and propagates; covariances positive-definite)

6. **Beacon catalog & loader**
   Create `beacons.sqlite` or YAML with {id, type, lat, lon, alt, tx_bias, freq, height}. Provide an API to query visible beacons near the origin with caching.
   **Test:** `tools/beacon_inspect --list --bbox "lat0,lat1,lon0,lon1"` prints N entries

7. **Receiver adapter (sim/log parser)**
   Implement an input that yields **TOA/TDOA/RTT/RSSI/AoA** records with `id, t_meas, quality, σ`. Start with a simulator and a CSV parser so you can replay logs.
   **Test:** `tools/stn_feed --from sample_logs/stn_demo.csv | head` shows normalized messages

8. **Measurement models (pseudorange & TDOA)**
   Implement TOA pseudorange: ρ = c·(t_rx−t_tx−b_rx−b_tx) + ε, and TDOA between beacon pairs removing absolute clock bias. Provide Jacobians or sigma-point prediction for UKF.
   **Test:** `ctest -R meas_models` (analytic vs numeric Jacobian error < 1e-6)

9. **Delay calibration & environment terms**
   Add configurable RF/cable/ADC delays and optional tropospheric correction (simple Saastamoinen, dry term). Treat unknown beacon `tx_bias` as fixed from catalog or per-beacon nuisance if enabled.
   **Test:** `ctest -R delay_cal` (inject known delay; estimator recovers bias within 5%)

10. **Robust gating & RAIM/NIS**
    Do chi-squared gating per measurement and RAIM-style subset checks for outlier rejection. Use Huber/Tukey loss to down-weight heavy-tailed errors from multipath.
    **Test:** `ctest -R raim_gate` (synthetic 20% outliers → <2% accepted after gating)

11. **UKF STN update (2D/3D modes)**
    Fuse ≥3 beacons for 2D (x,y, clock bias) or ≥4 for 3D (x,y,z, clock bias), always estimating clock drift. Fall back to TDOA-only when absolute timing is missing.
    **Test:** `ctest -R ukf_stn_update` (with four beacons → CEP50 < 15 m on clean sim)

12. **Geometry checks & mode manager (STN-only)**
    Compute GDOP/HDOP and freeze updates when geometry is ill-conditioned; surface a health flag. Log counts of visible/used/gated measurements per second.
    **Test:** `./build/apps/aion_node --config configs/stn_bad_geom.yaml` → logs show STN disabled with high DOP

13. **Metrics & integrity reporting**
    Emit rolling **NIS/NEES**, GDOP, accepted rate, and CEP50/CEP90 to JSON after each run. Provide `tools/metrics/summary.py` to print pass/fail against targets.
    **Test:** `python tools/metrics/summary.py logs/stn_run.json` → all KPIs present with thresholds

14. **Ground drive runner & map view**
    Build `apps/stn_drive.cpp` that subscribes to STN input and prints PVT at 5–10 Hz. Add a lightweight OpenGL or matplotlib live plot with track and beacon locations.
    **Test:** `./build/apps/stn_drive --replay sample_logs/stn_city.csv --plot`

15. **Replay & simulator scenarios**
    Add a scenario generator: urban canyon, suburban, and open-field with configurable beacon density and SNR. Save deterministic seeds for regression.
    **Test:** `tools/sim/gen_scenario.py --preset urban --seed 42 > sample_logs/urban_sim.csv`

16. **Acceptance run (GPS-denied, ground)**
    Drive a 10–15 km loop with at least 4 terrestrial beacons in view for most segments; keep GNSS off for truth only at endpoints. Target CEP90 ≤ 40 m overall and ≤ 20 m in good geometry.
    **Test:** `./build/apps/stn_drive --replay logs/drive_001.csv && python tools/metrics/summary.py logs/drive_001.json`

17. **CI, artifacts, and docs (STN-only)**
    Add a CI pipeline to build, run unit tests, and publish `stn_drive` plus sample logs. Document setup, configs, and known limitations in `docs/stn_getting_started.md`.
    **Test:** CI badge green; `docker run … make build test` passes on a clean machine

18. **Release v0.1 (STN ground MVP)**
    Freeze configs, export the simulator + two replayable real-drive logs, and tag `v0.1-stn`. Include a one-page KPI table and reproducibility script.
    **Test:** `scripts/reproduce_release.sh` regenerates the KPI JSONs within ±5% of tagged numbers

If you want, I can turn this into a ready-to-commit folder skeleton (`configs/`, `tools/`, `apps/`, `docs/`, `tests/`) with stub code and the sample tests wired up.
