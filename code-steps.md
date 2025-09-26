Perfect. Here’s a **start-to-finish execution plan**.
Each step has **two sentences** and a **Test** line.

1. **Repo bootstrap & toolchain**
   Initialize the repo from the skeleton, install deps (CMake, Eigen, yaml-cpp, GTest, pre-commit). Pin compilers and run pre-commit to enforce style.
   **Test:** `cmake -S . -B build && cmake --build build -j && pre-commit run -a` → build succeeds, no lint errors.

2. **Logging + config loader**
   Wire `Logger::init()` and YAML config parsing in `aion_node.cpp`. Expose `--config` CLI and print parsed params.
   **Test:** `./build/apps/aion_node --config configs/aion_telaviv.yaml` → logs show loaded values.

3. **State + SR-UKF skeleton**
   Implement state struct, predict stub, and softReset. Add a fixed dt loop calling `predict()` at 200 Hz.
   **Test:** unit test checks that `softReset()` updates state and `predict()` maintains invariants.

4. **Timebase & timestamping**
   Add monotonic clock wrapper and per-sensor timestamps. Ensure simulated sensor messages carry `t_recv` and `t_meas`.
   **Test:** unit test asserts strictly increasing timestamps and bounded latency.

5. **Binary logging & replay tool**
   Implement binary log writer/reader (e.g., FlatBuffers/CBOR) and `tools/replay/aion_replay`. Make every message serializable.
   **Test:** record a 60 s dummy session; replay reproduces identical message counts and timestamps (±1 ms).

6. **IMU driver + preintegration**
   Create IMU driver (sim mode first) and preintegration struct for graph. Publish IMU at 200 Hz to UKF + graph buffer.
   **Test:** unit test compares preintegrated delta against analytic constant-motion ground truth.

7. **Process model & gravity**
   Implement strapdown INS (coning/sculling comp) with WGS84 gravity. Add configurable process noise for biases.
   **Test:** simulation with constant velocity → position error <0.5 m after 10 s without aids.

8. **Terrain map service**
   Add DEM tile loader with bilinear height and ∂DEM/∂x, ∂DEM/∂y. Cache tiles and expose query API.
   **Test:** query random points; finite-difference gradient matches analytic gradient within tolerance.

9. **TRN measurement update**
   Implement scalar TRN residual (radar-alt vs DEM) with χ² gating and slope gating. Update UKF covariance & state.
   **Test:** synthetic hill profile: enabling TRN reduces vertical error by >80% and lateral by >30% when slope>0.5°.

10. **Radar altimeter driver (sim)**
    Create ralt driver with validity, range gates, and noise model. Feed TRN at 10–20 Hz.
    **Test:** inject step in AGL; TRN accepts inliers and rejects out-of-range/invalid samples.

11. **Radar odometry frontend (RO)**
    Implement minimal scan-to-scan odom (start with ICP on radar “power image” or point cloud). Output Δpose + 6×6 cov.
    **Test:** dataset/car loop: drift <1% of distance (e.g., <10 m per 1 km).

12. **Fixed-lag smoother (graph)**
    Add a 5–10 s fixed-lag factor graph (IMU preintegration, RO factors, TRN pseudo-range). Optimize at 20–60 Hz.
    **Test:** graph-only odometry on dataset beats UKF-only drift by ≥2×.

13. **UKF⇄Graph reconciliation**
    Every 1–2 Hz, soft-reset UKF to graph posterior (pos/vel/att/bias). Bound resets with Mahalanobis checks.
    **Test:** with RO dropouts, UKF stays stable and re-locks within 2 s when RO returns.

14. **Integrity layer (NEES/NIS + gating)**
    Compute rolling NEES/NIS and per-modality χ² gates; expose an integrity score S∈[0,1]. Down-weight failing modalities.
    **Test:** force bad radar frames; integrity score drops and gates exclude them; NEES within 95% bounds on clean data.

15. **Event/thermal VIO stub (optional path A)**
    Add minimal VIO (keyframe, PnP, IMU preintegration) or thermal-inertial odom. Publish Δpose factors into graph.
    **Test:** night/low-light dataset shows feature inliers ≥100/keyframe and drift ≤1%/distance.

16. **LiDAR odometry stub (optional path B)**
    Alternatively, wire a light ICP/feature LOAM-like frontend. Use whichever of VIO/LO is healthy via mode manager.
    **Test:** texture-poor scene (fields) keeps drift under 1.5% with LiDAR on, VIO off.

17. **SoOP adapter (STL/TerraPoiNT/5G sim)**
    Create a generic SoOP interface that yields pseudorange/time fixes with provider ID, DOP, and σ. Start with a simulator or log parser.
    **Test:** inject fixes every 10–60 s; position jumps are ≤σ and covariance tightens measurably.

18. **Mode & environment manager**
    Gate TRN by slope/AGL, VIO by inlier count, RO by SNR, SoOP by DOP; publish mode (NOMINAL/TRN_ONLY/RO_SOOP/DEGRADED).
    **Test:** scripted scenes flip modes as expected; no oscillation (hysteresis applied).

19. **Metrics pipeline (CEP, drift, inliers)**
    Implement `tools/metrics/metrics_summarizer.py` to compute CEP50/CEP90, drift %, inlier rates. Save JSON per run.
    **Test:** `python tools/metrics/metrics_summarizer.py logs/run.bag` → JSON with KPIs and pass/fail flags.

20. **Ground drive MVP**
    Roof-mount rig and run mixed routes (day/night/fog if possible). Tune gates and process/measurement noise.
    **Test:** CEP90 ≤ 40 m on rolling terrain segments; drift ≤ 1% overall.

21. **Airborne integration basics**
    Measure lever arms/boresight; ensure PPS/PTP time sync; damp vibration. Reduce logging rate; validate IMU ranges.
    **Test:** static alignment compares baro+DEM with ralt within spec; no time skew >2 ms across sensors.

22. **Flight test (low AGL → mid AGL)**
    Fly 300–800 m AGL, 25–40 m/s; enable TRN + RO + SoOP and optionally VIO/LO depending on conditions. Log full integrity.
    **Test:** CEP90 ≤ 50 m over 30–40 min legs; integrity ≥ 0.8 for ≥80% of time; NEES/NIS within bounds.

23. **Failure-injection & degraded ops**
    Simulate SoOP dropouts, flat terrain (TRN off), radar speckle bursts, and vision blackout. Verify graceful degradation and recovery time.
    **Test:** each fault scenario meets recovery SLA (e.g., <5 s to nominal after fault clears).

24. **Interfaces & SDK**
    Expose gRPC/ROS2 PVT + Integrity reports; publish NMEA for legacy consumers. Add versioned config and schema.
    **Test:** sample client decodes PVT at 10 Hz and asserts integrity transitions correctly.

25. **CI/CD + Docker**
    Add Dockerfile.dev and pipelines for build/test/format; run unit + scenario tests on CI. Cache build artifacts.
    **Test:** merge request runs green; artifacts include `aion_node` and sample logs.

26. **Documentation & checklists**
    Write `docs/design/*` and ops checklists (calibration, flight prep, incident replay). Provide acceptance criteria and playbooks.
    **Test:** new engineer reproduces a ground demo in <2 hours using docs only.

27. **Release & tag v0.1**
    Freeze configs, export a reference dataset + metrics JSON, and tag. Prepare a short demo video and KPI table.
    **Test:** reproducible build + replay yields metrics within ±5% of the release numbers.

If you want, I’ll convert this into **GitLab issues** with owners, estimates, and acceptance tests you can run in order.
Short answer: **yes — that plan + README are enough to build a v0.1 MVP** and flight-test. To finish confidently (and avoid rework), add the few missing “glue” pieces below.

## The 10 gaps to close (now)

1. **Exact sensor BOM + part numbers**
   Pick specific IMU, 77 GHz radar, radar-alt, and camera/LiDAR models with specs pinned in `docs/design/hw_bom.md`.
   **Test:** dry-run procurement sheet compiles; all drivers have a target device.

2. **Coordinate frames & units contract**
   Lock NED vs ENU, body axes, radians vs degrees, meters everywhere; publish in `docs/design/frames.md`.
   **Test:** a unit test converts a synthetic pose through every module and returns identical pose (ε < 1e-6).

3. **Time sync design**
   Decide PPS/PTP source, timestamp policy (`t_meas` vs `t_recv`), and max skew bounds; document in `docs/design/time_sync.md`.
   **Test:** log cross-sensor timestamp deltas; assert |Δt| < 2 ms in a 10-min run.

4. **Calibration procedures & file formats**
   Write step-by-step camera/IMU/radar boresight + lever-arm procedures; choose YAML schema in `configs/hw/calibrations/*.yaml`.
   **Test:** run calibration twice; extrinsics agree within set tolerances (e.g., <0.2° / <5 mm).

5. **Numerics & error budget**
   Create a simple spreadsheet (or `tools/metrics/error_budget.py`) with expected drift, TRN clamp, SoOP σ, and overall CEP targets.
   **Test:** simulated run falls within ±20% of the budget per segment.

6. **Acceptance criteria per phase**
   Turn the big 27-step plan into pass/fail gates (CEP, drift %, inliers, integrity) in `docs/design/acceptance.md`.
   **Test:** `metrics_summarizer.py` emits ✅ / ❌ flags for each KPI.

7. **Reproducible dev env**
   Finish `docker/Dockerfile.dev` + `Makefile` targets; pin compiler/dep versions; add `pre-commit`.
   **Test:** clean clone → `make build run` works on any dev machine.

8. **Data management**
   Define folder schema for logs/datasets, retention, and anonymization policy in `docs/design/data_policy.md`.
   **Test:** 30-minute drive produces ≤1 GB, replayable with a single command.

9. **Integrity modes & UX**
   Document modes (NOMINAL/TRN_ONLY/RO_SOOP/DEGRADED), lights/flags, and autopilot contract.
   **Test:** scripted faults flip modes with hysteresis; client demo app shows clear status.

10. **Safety & legal**
    Add a brief FMEA, air test checklist, and third-party licenses; confirm no export/licensing blockers.
    **Test:** pre-flight checklist ticked; license script lists all deps.

---

## Green-light checklist (paste to your repo’s README end)

* [ ] BOM fixed; drivers mapped
* [ ] Frames/units spec merged
* [ ] Time sync spec merged
* [ ] Calib procedures + sample YAMLs committed
* [ ] Error-budget doc committed
* [ ] Acceptance gates encoded in metrics tool
* [ ] Docker dev image builds; pre-commit passes
* [ ] Log schema + replay tool verified
* [ ] Mode/UX contract documented
* [ ] Safety/FMEA/licenses added

If you tick these, **the plan you posted is enough** to build, test, and demo AION v0.1. Want me to drop boilerplate files for frames/time-sync/calibration into the canvas so you can commit them as-is?
